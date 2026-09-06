#!/usr/bin/env python3
"""Lightweight white-card letter recognition for the head ZED camera.

The runtime path is intentionally small: HSV white-card proposals, local
brightness validation, perspective rectification, line-occlusion suppression,
and rotation-invariant glyph template matching.  It requires no depth image,
neural-network runtime, or network service.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class LetterDetection:
    letter: str
    confidence: float
    center_x_norm: float
    center_y_norm: float
    area_norm: float
    row: str
    quad: tuple[tuple[float, float], ...]

    def as_dict(self) -> dict:
        return asdict(self)


def order_quad(points: np.ndarray) -> np.ndarray:
    points = points.reshape(4, 2).astype(np.float32)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).ravel()
    return np.array(
        [
            points[np.argmin(sums)],
            points[np.argmin(diffs)],
            points[np.argmax(sums)],
            points[np.argmax(diffs)],
        ],
        dtype=np.float32,
    )


def warp_quad(image: np.ndarray, quad: np.ndarray, size: int = 192) -> np.ndarray:
    destination = np.array(
        [[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], np.float32
    )
    transform = cv2.getPerspectiveTransform(order_quad(quad), destination)
    return cv2.warpPerspective(image, transform, (size, size))


def white_card_quads(image: np.ndarray) -> list[np.ndarray]:
    """Return only white-paper proposals; coloured cups never become candidates."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = image.shape[:2]
    scale = max(1.0, width / 640.0)
    white = cv2.inRange(hsv, (0, 0, 100), (179, 60, 255))
    kernel_size = max(3, int(round(3 * scale)) | 1)
    white = cv2.morphologyEx(
        white,
        cv2.MORPH_CLOSE,
        np.ones((kernel_size, kernel_size), np.uint8),
        iterations=2,
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(white)
    quads: list[np.ndarray] = []
    min_area = 55.0 * scale * scale
    max_area = 5000.0 * scale * scale
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if not min_area <= area <= max_area:
            continue
        if y < int(0.14 * height) or min(w, h) < 8 * scale or max(w, h) > 115 * scale:
            continue
        if max(w, h) / max(1.0, min(w, h)) > 2.7:
            continue
        pad = max(6, int(round(7 * scale)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(width, x + w + pad), min(height, y + h + pad)
        outer = hsv[y0:y1, x0:x1]
        ring = np.ones(outer.shape[:2], dtype=bool)
        ring[y - y0 : y - y0 + h, x - x0 : x - x0 + w] = False
        inner = hsv[y : y + h, x : x + w]
        if float(np.mean(inner[:, :, 1])) > 63.0:
            continue
        if float(np.mean(inner[:, :, 2])) - float(np.mean(outer[:, :, 2][ring])) < 24.0:
            continue
        component = np.zeros(white.shape, np.uint8)
        component[labels == label] = 255
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        rectangle = cv2.minAreaRect(max(contours, key=cv2.contourArea))
        rw, rh = rectangle[1]
        if min(rw, rh) < 7 * scale:
            continue
        quad = cv2.boxPoints(rectangle).astype(np.float32)
        qx, qy, qw, qh = cv2.boundingRect(quad.astype(np.int32))
        if qx < 3 or qy < 3 or qx + qw >= width - 3 or qy + qh >= height - 3:
            continue
        quads.append(quad)
    # A low-saturation card can become connected to a large white robot part
    # in the HSV mask and therefore disappear as its own component.  Recover
    # those cards from their rectangular edge contour.  This is still a
    # lightweight CPU path and is especially important for the near A card.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 45, 130)
    edge_kernel = max(3, int(round(2.5 * scale)) | 1)
    edges = cv2.morphologyEx(
        edges,
        cv2.MORPH_CLOSE,
        np.ones((edge_kernel, edge_kernel), np.uint8),
        iterations=2,
    )
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    edge_quads: list[np.ndarray] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not min_area <= area <= 4.0 * max_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if y < int(0.14 * height) or min(w, h) < 7 * scale or max(w, h) > 120 * scale:
            continue
        if max(w, h) / max(1.0, min(w, h)) > 2.7:
            continue
        rectangle = cv2.minAreaRect(contour)
        rw, rh = rectangle[1]
        rectangle_area = max(1.0, float(rw * rh))
        if min(rw, rh) < 7 * scale or area / rectangle_area < 0.42:
            continue
        quad = cv2.boxPoints(rectangle).astype(np.float32)
        center = np.mean(quad, axis=0)
        area_norm = area / float(width * height)
        # The edge fallback is specifically for close cards whose white mask
        # merged with the robot/background.  Applying it to the whole robot
        # silhouette created many convincing but false A glyphs.
        if center[1] / height < 0.52 or not 0.0015 <= area_norm <= 0.0080:
            continue
        qx, qy, qw, qh = cv2.boundingRect(quad.astype(np.int32))
        if qx < 3 or qy < 3 or qx + qw >= width - 3 or qy + qh >= height - 3:
            continue
        card = warp_quad(image, quad)
        card_hsv = cv2.cvtColor(card, cv2.COLOR_BGR2HSV)
        pale_fraction = float(
            np.mean((card_hsv[:, :, 1] <= 100) & (card_hsv[:, :, 2] >= 135))
        )
        if pale_fraction < 0.55 or normalized_glyph(card) is None:
            continue
        edge_quads.append(quad)

    # Prefer the largest outline and suppress inner letter contours / duplicate
    # HSV proposals for the same physical card.
    candidates = sorted(
        [*quads, *edge_quads], key=lambda item: abs(float(cv2.contourArea(item))), reverse=True
    )
    selected: list[np.ndarray] = []
    for candidate in candidates:
        x, y, w, h = cv2.boundingRect(candidate.astype(np.int32))
        duplicate = False
        for existing in selected:
            ex, ey, ew, eh = cv2.boundingRect(existing.astype(np.int32))
            ix0, iy0 = max(x, ex), max(y, ey)
            ix1, iy1 = min(x + w, ex + ew), min(y + h, ey + eh)
            intersection = max(0, ix1 - ix0) * max(0, iy1 - iy0)
            smaller = max(1, min(w * h, ew * eh))
            if intersection / smaller >= 0.45:
                duplicate = True
                break
        if not duplicate:
            selected.append(candidate)
    return selected


def _remove_long_occlusions(mask: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    horizontal = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((1, max(9, int(0.72 * width))), np.uint8)
    )
    vertical = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((max(9, int(0.72 * height)), 1), np.uint8)
    )
    cleaned = cv2.bitwise_and(mask, cv2.bitwise_not(cv2.bitwise_or(horizontal, vertical)))
    return cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)


def normalized_glyph(card: np.ndarray, canvas_size: int = 96) -> np.ndarray | None:
    gray = cv2.cvtColor(card, cv2.COLOR_BGR2GRAY)
    margin = max(5, int(round(0.12 * min(gray.shape))))
    interior = gray[margin:-margin, margin:-margin]
    if interior.size == 0:
        return None
    background = float(np.percentile(interior, 82))
    threshold = int(max(50.0, background - 24.0))
    ink = np.zeros_like(gray)
    ink[gray < threshold] = 255
    ink[:margin, :] = 0
    ink[-margin:, :] = 0
    ink[:, :margin] = 0
    ink[:, -margin:] = 0
    ink = _remove_long_occlusions(ink)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(ink)
    accepted = np.zeros_like(ink)
    minimum = max(8, int(round(0.0012 * ink.size)))
    candidates = []
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area < minimum:
            continue
        if max(w, h) / max(1.0, min(w, h)) > 7.5:
            continue
        candidates.append((area, label))
    if candidates:
        # Uppercase glyphs in the competition alphabet are connected.  Keeping
        # the dominant component removes card borders/table seams without
        # confusing them with a second character.
        accepted[labels == max(candidates)[1]] = 255
    return _normalize_binary_mask(accepted, canvas_size)


def _normalize_binary_mask(mask: np.ndarray, canvas_size: int = 96) -> np.ndarray | None:
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, w, h = cv2.boundingRect(points)
    if w * h < 0.006 * mask.size:
        return None
    glyph = mask[y : y + h, x : x + w]
    target = canvas_size - 18
    factor = min(target / max(1, w), target / max(1, h))
    resized = cv2.resize(
        glyph,
        (max(1, int(round(w * factor))), max(1, int(round(h * factor)))),
        interpolation=cv2.INTER_AREA if factor < 1.0 else cv2.INTER_CUBIC,
    )
    canvas = np.zeros((canvas_size, canvas_size), np.uint8)
    oy = (canvas_size - resized.shape[0]) // 2
    ox = (canvas_size - resized.shape[1]) // 2
    canvas[oy : oy + resized.shape[0], ox : ox + resized.shape[1]] = resized
    return canvas


def _template(letter: str, thickness: int, canvas_size: int = 96) -> np.ndarray:
    image = np.zeros((canvas_size, canvas_size), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 2.45
    (w, h), _ = cv2.getTextSize(letter, font, scale, thickness)
    cv2.putText(
        image,
        letter,
        ((canvas_size - w) // 2, (canvas_size + h) // 2),
        font,
        scale,
        255,
        thickness,
        cv2.LINE_AA,
    )
    image[image > 80] = 255
    image[image <= 80] = 0
    normalized = _normalize_binary_mask(image, canvas_size)
    return normalized if normalized is not None else image


def _binary_similarity(first: np.ndarray, second: np.ndarray) -> float:
    a = first > 0
    b = second > 0
    union = int(np.count_nonzero(a | b))
    intersection = int(np.count_nonzero(a & b))
    iou = intersection / max(1, union)
    distance_to_b = cv2.distanceTransform((~b).astype(np.uint8), cv2.DIST_L2, 3)
    distance_to_a = cv2.distanceTransform((~a).astype(np.uint8), cv2.DIST_L2, 3)
    chamfer = 0.5 * (
        float(np.mean(distance_to_b[a])) if np.any(a) else 20.0
    ) + 0.5 * (
        float(np.mean(distance_to_a[b])) if np.any(b) else 20.0
    )
    return float(0.58 * iou + 0.42 * math.exp(-chamfer / 4.0))


def card_surface_is_white(
    card: np.ndarray,
    minimum_fraction: float = 0.45,
    minimum_border_fraction: float = 0.50,
    minimum_corner_fraction: float = 0.30,
) -> bool:
    """Require a bright rectangular paper substrate, not merely a grey patch.

    The floor reflections and the white food plate both produced plausible
    glyph contours in competition frames.  A real label has bright neutral
    paper over its whole surface, around its border, and in all four corner
    regions.  The circular plate fails the corner test; grey floor tiles fail
    all three brightness tests.
    """
    hsv = cv2.cvtColor(card, cv2.COLOR_BGR2HSV)
    bright_white = (hsv[:, :, 1] <= 70) & (hsv[:, :, 2] >= 170)
    height, width = bright_white.shape
    margin_y = max(2, int(round(0.16 * height)))
    margin_x = max(2, int(round(0.16 * width)))
    border = np.ones_like(bright_white, dtype=bool)
    border[margin_y:-margin_y, margin_x:-margin_x] = False
    corners = np.zeros_like(bright_white, dtype=bool)
    corners[:margin_y, :margin_x] = True
    corners[:margin_y, -margin_x:] = True
    corners[-margin_y:, :margin_x] = True
    corners[-margin_y:, -margin_x:] = True
    return bool(
        float(np.mean(bright_white)) >= float(minimum_fraction)
        and float(np.mean(bright_white[border])) >= float(minimum_border_fraction)
        and float(np.mean(bright_white[corners])) >= float(minimum_corner_fraction)
    )


class LetterCardRecognizer:
    def __init__(
        self,
        alphabet: str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
        minimum_confidence: float = 0.22,
        row_split_y_norm: float = 0.52,
    ) -> None:
        unique = "".join(dict.fromkeys(letter.upper() for letter in alphabet if letter.isalpha()))
        if not unique:
            raise ValueError("alphabet must contain at least one letter")
        self.alphabet = unique
        self.minimum_confidence = float(minimum_confidence)
        self.row_split_y_norm = float(row_split_y_norm)
        self.templates = {
            letter: [_template(letter, thickness) for thickness in (2, 3, 4)]
            for letter in self.alphabet
        }

    def classify(self, card: np.ndarray) -> tuple[str, float]:
        scores: dict[str, float] = {letter: 0.0 for letter in self.alphabet}
        reference_glyph = normalized_glyph(card)
        holes = 0
        if reference_glyph is not None:
            contours, hierarchy = cv2.findContours(
                reference_glyph, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
            )
            if hierarchy is not None:
                minimum_hole_area = 0.004 * reference_glyph.size
                holes = sum(
                    1
                    for index, relation in enumerate(hierarchy[0])
                    if relation[3] >= 0 and cv2.contourArea(contours[index]) >= minimum_hole_area
                )
        for turns in range(4):
            oriented = np.rot90(card, turns).copy()
            glyph = normalized_glyph(oriented)
            if glyph is None:
                continue
            for letter, templates in self.templates.items():
                scores[letter] = max(
                    scores[letter], max(_binary_similarity(glyph, template) for template in templates)
                )
        if holes >= 2 and "B" in scores and scores["B"] >= 0.60:
            # Two holes are supporting evidence, not proof of B.  The old
            # unconditional 0.86 result promoted unrelated two-hole shapes
            # in a no-card scene to a target letter immediately after turn.
            scores["B"] = min(1.0, scores["B"] + 0.06)
        # Topology is an authorization condition for the three competition
        # targets, not just a score hint.  Previously, an E card was forced to
        # A when recognition was restricted to ABD, and a floor seam could do
        # the same.  Clean printed A/D have one counter and B has two.
        required_holes = {"A": 1, "B": 2, "D": 1}
        raw_scores = dict(scores)
        scores = {
            letter: score
            for letter, score in scores.items()
            if letter not in required_holes or holes == required_holes[letter]
        }
        if not scores:
            return "", 0.0
        ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
        letter, score = ranked[0]
        # Topology may disqualify a class, but it must not artificially inflate
        # the winning confidence by erasing a visually similar runner-up.
        runner_up = max(
            (value for name, value in raw_scores.items() if name != letter),
            default=0.0,
        )
        confidence = max(0.0, min(1.0, 0.65 * score + 0.35 * max(0.0, score - runner_up) * 3.0))
        return letter, confidence

    def detect(self, image: np.ndarray) -> list[LetterDetection]:
        height, width = image.shape[:2]
        detections = []
        for quad in white_card_quads(image):
            card = warp_quad(image, quad)
            if not card_surface_is_white(card):
                continue
            letter, confidence = self.classify(card)
            if confidence < self.minimum_confidence:
                continue
            center = np.mean(quad, axis=0)
            area = abs(float(cv2.contourArea(quad)))
            y_norm = float(center[1] / height)
            detections.append(
                LetterDetection(
                    letter=letter,
                    confidence=confidence,
                    center_x_norm=float(center[0] / width),
                    center_y_norm=y_norm,
                    area_norm=area / float(width * height),
                    row="near" if y_norm >= self.row_split_y_norm else "far",
                    quad=tuple((float(point[0]), float(point[1])) for point in quad),
                )
            )
        return sorted(detections, key=lambda item: (item.row, item.center_x_norm))


def annotate(image: np.ndarray, detections: Iterable[LetterDetection]) -> np.ndarray:
    output = image.copy()
    for detection in detections:
        quad = np.array(detection.quad, np.int32)
        cv2.polylines(output, [quad], True, (0, 255, 0), 2)
        x, y = map(int, np.min(quad, axis=0))
        label = f"{detection.letter} {detection.confidence:.2f} {detection.row}"
        cv2.putText(output, label, (x, max(22, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 255), 2, cv2.LINE_AA)
    return output


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--alphabet", default="ABCDEFGHIJKLMNOPQRSTUVWXYZ")
    parser.add_argument("--annotated", type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"cannot read image: {args.image}")
    detections = LetterCardRecognizer(args.alphabet).detect(image)
    if args.annotated:
        args.annotated.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.annotated), annotate(image, detections))
    print(json.dumps([item.as_dict() for item in detections], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
