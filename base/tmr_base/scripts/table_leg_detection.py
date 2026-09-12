#!/usr/bin/env python3
"""Pure-Python table-leg clustering and constrained pair selection."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class LegCluster:
    x: float
    y: float
    hits: int
    diameter: float


@dataclass(frozen=True)
class PairDetection:
    legs: tuple[LegCluster, LegCluster]
    midpoint: tuple[float, float]
    approach: tuple[float, float, float]
    score: float
    second_score: float | None


class DetectionError(RuntimeError):
    pass


def _angle_difference_axis(a: float, b: float) -> float:
    """Difference of two unoriented axes, in radians, in [0, pi/2]."""
    delta = abs(math.atan2(math.sin(a - b), math.cos(a - b)))
    return min(delta, abs(math.pi - delta))


def cluster_points(points: Iterable[tuple[float, float]], cfg: dict) -> list[LegCluster]:
    xmin, xmax = map(float, cfg["roi"]["x"])
    ymin, ymax = map(float, cfg["roi"]["y"])
    resolution = float(cfg["grid_resolution"])
    connection_cells = max(1, math.ceil(float(cfg.get("cluster_connect_distance", resolution * 1.5)) / resolution))
    counts: dict[tuple[int, int], int] = defaultdict(int)
    for x, y in points:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            key = (math.floor((x - xmin) / resolution), math.floor((y - ymin) / resolution))
            counts[key] += 1

    active = {key for key, count in counts.items() if count >= int(cfg["min_cell_hits"])}
    clusters: list[LegCluster] = []
    while active:
        seed = active.pop()
        component = [seed]
        queue = deque([seed])
        while queue:
            cx, cy = queue.popleft()
            for dx in range(-connection_cells, connection_cells + 1):
                for dy in range(-connection_cells, connection_cells + 1):
                    if dx == 0 and dy == 0:
                        continue
                    if math.hypot(dx * resolution, dy * resolution) > float(
                        cfg.get("cluster_connect_distance", resolution * 1.5)
                    ):
                        continue
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in active:
                        active.remove(neighbour)
                        component.append(neighbour)
                        queue.append(neighbour)

        hits = sum(counts[key] for key in component)
        if hits < int(cfg["min_cluster_hits"]):
            continue
        xs = [xmin + (key[0] + 0.5) * resolution for key in component]
        ys = [ymin + (key[1] + 0.5) * resolution for key in component]
        diameter = max(max(xs) - min(xs) + resolution, max(ys) - min(ys) + resolution)
        if diameter > float(cfg["max_leg_diameter"]):
            continue
        weighted_x = sum((xmin + (key[0] + 0.5) * resolution) * counts[key] for key in component)
        weighted_y = sum((ymin + (key[1] + 0.5) * resolution) * counts[key] for key in component)
        clusters.append(LegCluster(weighted_x / hits, weighted_y / hits, hits, diameter))
    return sorted(clusters, key=lambda item: (item.x, item.y))


def select_leg_pair(
    clusters: list[LegCluster], cfg: dict, observer_xy: tuple[float, float]
) -> PairDetection:
    expected_spacing = float(cfg["expected_pair_spacing"])
    spacing_tolerance = float(cfg["pair_spacing_tolerance"])
    expected_axis = math.radians(float(cfg["expected_pair_axis_deg"]))
    axis_tolerance = math.radians(float(cfg["pair_axis_tolerance_deg"]))
    expected_mid_x, expected_mid_y = map(float, cfg["expected_pair_midpoint"])
    maximum_midpoint_shift = float(cfg["max_midpoint_shift"])
    matches: list[tuple[float, LegCluster, LegCluster, float, float]] = []

    for index, first in enumerate(clusters):
        for second in clusters[index + 1 :]:
            dx, dy = second.x - first.x, second.y - first.y
            spacing = math.hypot(dx, dy)
            spacing_error = abs(spacing - expected_spacing)
            if spacing_error > spacing_tolerance:
                continue
            axis_error = _angle_difference_axis(math.atan2(dy, dx), expected_axis)
            if axis_error > axis_tolerance:
                continue
            midpoint_x = (first.x + second.x) * 0.5
            midpoint_y = (first.y + second.y) * 0.5
            midpoint_error = math.hypot(midpoint_x - expected_mid_x, midpoint_y - expected_mid_y)
            if midpoint_error > maximum_midpoint_shift:
                continue
            score = spacing_error / spacing_tolerance + axis_error / axis_tolerance + midpoint_error / maximum_midpoint_shift
            matches.append((score, first, second, midpoint_x, midpoint_y))

    if not matches:
        summary = ", ".join(f"({c.x:.2f},{c.y:.2f},h={c.hits})" for c in clusters)
        raise DetectionError(f"no constrained leg pair; clusters=[{summary}]")
    matches.sort(key=lambda item: item[0])
    best = matches[0]
    second_score = matches[1][0] if len(matches) > 1 else None
    maximum_pair_score = cfg.get("max_pair_score")
    if maximum_pair_score is not None and best[0] > float(maximum_pair_score):
        raise DetectionError(
            f"best leg pair score is too weak: {best[0]:.3f} > {float(maximum_pair_score):.3f}"
        )
    if second_score is not None and second_score - best[0] < float(cfg["ambiguity_score_margin"]):
        raise DetectionError(f"ambiguous leg pairs: best={best[0]:.3f}, second={second_score:.3f}")

    _, first, second, midpoint_x, midpoint_y = best
    axis_x, axis_y = second.x - first.x, second.y - first.y
    axis_length = math.hypot(axis_x, axis_y)
    normal_a = (-axis_y / axis_length, axis_x / axis_length)
    normal_b = (-normal_a[0], -normal_a[1])
    toward_observer = (observer_xy[0] - midpoint_x, observer_xy[1] - midpoint_y)
    normal = normal_a if normal_a[0] * toward_observer[0] + normal_a[1] * toward_observer[1] >= 0 else normal_b
    standoff = float(cfg["approach_standoff"])
    approach_x = midpoint_x + normal[0] * standoff
    approach_y = midpoint_y + normal[1] * standoff
    approach_yaw = math.atan2(midpoint_y - approach_y, midpoint_x - approach_x)

    expected_to_observer = (
        observer_xy[0] - expected_mid_x,
        observer_xy[1] - expected_mid_y,
    )
    expected_observer_distance = math.hypot(*expected_to_observer)
    if expected_observer_distance < 1e-6:
        raise DetectionError("expected leg midpoint cannot coincide with the observer")
    expected_approach = (
        expected_mid_x + expected_to_observer[0] / expected_observer_distance * standoff,
        expected_mid_y + expected_to_observer[1] / expected_observer_distance * standoff,
    )
    if math.hypot(approach_x - expected_approach[0], approach_y - expected_approach[1]) > float(cfg["max_approach_shift"]):
        raise DetectionError("computed approach is too far from the expected safe side")
    return PairDetection((first, second), (midpoint_x, midpoint_y), (approach_x, approach_y, approach_yaw), best[0], second_score)


def detect_pair(
    points: Iterable[tuple[float, float]], cfg: dict, observer_xy: tuple[float, float]
) -> tuple[list[LegCluster], PairDetection]:
    clusters = cluster_points(points, cfg)
    return clusters, select_leg_pair(clusters, cfg, observer_xy)
