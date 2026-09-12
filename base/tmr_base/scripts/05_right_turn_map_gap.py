#!/usr/bin/env python3
"""Right-turn, build a dual-LiDAR local map, and stop at a frontal gap.

The file is intentionally self-contained so it can be streamed to
``python3 -u -``.  It uses existing ROS 2 topics, keeps the map in memory,
and never creates a robot-side file.  The base and LiDAR drivers must already
be running.  Default motion is a -90 degree odometry-closed-loop turn followed
by gap detection and a low-speed holonomic approach.  Use ``--skip-turn`` when
the physical robot has already completed that turn.
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass
from functools import partial
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


ODOM_TOPIC = "/swerve_drive_controller/odom"
CMD_TOPIC = "/swerve_drive_controller/cmd_vel"
PREFERRED_SCAN_TOPICS = ("/lidar_front/scan", "/lidar_rear/scan")

# Measured TMR footprint.  These are deliberately not CLI-overridable.
FOOTPRINT_FRONT_M = 0.40
FOOTPRINT_REAR_M = 0.40
FOOTPRINT_WIDTH_M = 0.58
FOOTPRINT_HALF_WIDTH_M = FOOTPRINT_WIDTH_M / 2.0
ROTATION_CLEARANCE_M = 0.60

SCAN_STALE_S = 1.20
ODOM_STALE_S = 0.45
GRID_RESOLUTION_M = 0.04
MAP_X_LIMITS = (-0.50, 4.00)
MAP_Y_LIMITS = (-2.50, 2.50)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass(frozen=True)
class OdomState:
    x: float
    y: float
    yaw: float
    unwrapped_yaw: float
    vx: float
    vy: float
    wz: float
    received_at: float


@dataclass(frozen=True)
class LaserExtrinsic:
    """base <- scan transform projected from the full 3-D rotation onto XY."""

    tx: float
    ty: float
    r00: float
    r01: float
    r10: float
    r11: float
    source: str

    def project_unit(self, angle: float) -> tuple[float, float]:
        sx, sy = math.cos(angle), math.sin(angle)
        return (
            self.r00 * sx + self.r01 * sy,
            self.r10 * sx + self.r11 * sy,
        )


def quaternion_xy_projection(q) -> tuple[float, float, float, float]:
    """Return the XY block of a normalized quaternion's 3-D rotation matrix."""

    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        raise RuntimeError("LiDAR TF contains a zero-length quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
    )


def measured_lidar_extrinsic(frame: str) -> LaserExtrinsic | None:
    """Exact fallback for the two measured scan frames; no fuzzy frame match."""

    measurements = {
        # These are the composed transforms used by the installed
        # navigation_adapter.launch.py: Rz(yaw) * Rx(pi).  The earlier
        # Rz(135 deg) * Ry(pi) fallback rotated every 2-D endpoint by -90 deg
        # relative to base_link and must not be used for body-frame motion.
        "lidar_front": (0.3275, 0.2175, 0.7846018366025517),
        "lidar_rear": (-0.3275, -0.2175, -2.3569908169872414),
    }
    measurement = measurements.get(frame)
    if measurement is None:
        return None
    tx, ty, yaw = measurement
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) * Rx(pi), so its XY block is [[c, s], [s, -c]].
    return LaserExtrinsic(
        tx=tx,
        ty=ty,
        r00=cy,
        r01=sy,
        r10=sy,
        r11=-cy,
        source="fixed_installed_navigation_adapter",
    )


@dataclass(frozen=True)
class ScanRecord:
    topic: str
    frame: str
    message: LaserScan
    sequence: int
    received_at: float


@dataclass(frozen=True)
class RayEvidence:
    ox: float
    oy: float
    ex: float
    ey: float
    hit: bool
    source: str
    scan_id: str


@dataclass(frozen=True)
class GapDetection:
    geometry: str
    slope: float
    intercept: float
    normal: tuple[float, float]
    tangent: tuple[float, float]
    plane_distance: float
    right_edge: tuple[float, float]
    left_edge: tuple[float, float]
    midpoint: tuple[float, float]
    width: float
    line_support_cells: int
    free_ray_count: int
    free_frame_count: int
    free_source_count: int
    free_lateral_bins: int
    required_lateral_bins: int
    score: float


@dataclass(frozen=True)
class FreeThroughEvidence:
    ray_count: int
    frame_count: int
    source_count: int
    covered_lateral_bins: int
    required_lateral_bins: int


class LocalEvidenceMap:
    """Small hit/free evidence grid plus the rays needed to prove a gap free."""

    def __init__(self, resolution: float = GRID_RESOLUTION_M) -> None:
        self.resolution = resolution
        self.occupied: dict[tuple[int, int], int] = defaultdict(int)
        self.occupied_frames: dict[tuple[int, int], set[str]] = defaultdict(set)
        self.free: dict[tuple[int, int], int] = defaultdict(int)
        self.rays: list[RayEvidence] = []
        self.frames_by_source: dict[str, int] = defaultdict(int)

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    @staticmethod
    def _inside(x: float, y: float) -> bool:
        return (
            MAP_X_LIMITS[0] <= x <= MAP_X_LIMITS[1]
            and MAP_Y_LIMITS[0] <= y <= MAP_Y_LIMITS[1]
        )

    def add_ray(self, ray: RayEvidence) -> None:
        self.rays.append(ray)
        if ray.hit and self._inside(ray.ex, ray.ey):
            cell = self._cell(ray.ex, ray.ey)
            self.occupied[cell] += 1
            self.occupied_frames[cell].add(ray.scan_id)

        # Rasterize a quarter of beams.  All beams remain in ``rays`` for the
        # geometric free-through test, while this bounded sample is enough for
        # a useful in-memory free-cell map without starving ROS callbacks.
        if len(self.rays) % 4:
            return
        dx, dy = ray.ex - ray.ox, ray.ey - ray.oy
        length = math.hypot(dx, dy)
        if length < self.resolution:
            return
        steps = max(1, int(length / (self.resolution * 1.5)))
        stop = steps if not ray.hit else max(0, steps - 1)
        for index in range(stop):
            fraction = index / steps
            x, y = ray.ox + fraction * dx, ray.oy + fraction * dy
            if self._inside(x, y):
                self.free[self._cell(x, y)] += 1

    def mark_frame(self, source: str) -> None:
        self.frames_by_source[source] += 1

    def occupied_points(self, minimum_hits: int = 2) -> list[tuple[float, float, int]]:
        result: list[tuple[float, float, int]] = []
        r = self.resolution
        for (ix, iy), hits in self.occupied.items():
            if len(self.occupied_frames[(ix, iy)]) >= minimum_hits:
                result.append(((ix + 0.5) * r, (iy + 0.5) * r, hits))
        return result

    def summary(self) -> dict:
        return {
            "resolution_m": self.resolution,
            "occupied_cells": len(self.occupied),
            "confirmed_occupied_cells": len(self.occupied_points(2)),
            "free_cells": len(self.free),
            "rays": len(self.rays),
            "frames_by_source": dict(sorted(self.frames_by_source.items())),
        }


def _segments_from_bins(
    bins: list[int], resolution: float
) -> list[tuple[float, float, int]]:
    if not bins:
        return []
    segments: list[tuple[int, int, int]] = []
    first = previous = bins[0]
    count = 1
    for value in bins[1:]:
        # Close only small (<= 8 cm) sampling holes in a physical boundary.
        if value - previous <= 3:
            previous = value
            count += 1
        else:
            segments.append((first, previous, count))
            first = previous = value
            count = 1
    segments.append((first, previous, count))
    return [
        ((first - 0.5) * resolution, (last + 0.5) * resolution, count)
        for first, last, count in segments
        if count >= 4 and (last - first + 1) * resolution >= 0.16
    ]


def _free_through_evidence(
    local_map: LocalEvidenceMap,
    normal: tuple[float, float],
    tangent: tuple[float, float],
    plane_distance: float,
    low_end: float,
    high_start: float,
) -> FreeThroughEvidence | None:
    """Require multi-frame rays through the complete footprint-width opening."""

    free_rays: set[int] = set()
    free_frames: set[str] = set()
    free_sources: set[str] = set()
    mid_s = 0.5 * (low_end + high_start)
    required_half_width = FOOTPRINT_HALF_WIDTH_M + 0.02
    lateral_low = mid_s - required_half_width
    lateral_high = mid_s + required_half_width
    required_lateral_bins = max(
        1, math.ceil((lateral_high - lateral_low) / 0.08)
    )
    lateral_frames: dict[int, set[str]] = defaultdict(set)
    inner_low, inner_high = low_end + 0.05, high_start - 0.05
    if lateral_low < inner_low or lateral_high > inner_high:
        return None
    for ray_index, ray in enumerate(local_map.rays):
        direction_x, direction_y = ray.ex - ray.ox, ray.ey - ray.oy
        denominator = normal[0] * direction_x + normal[1] * direction_y
        if denominator <= 1e-6:
            continue
        fraction = (
            plane_distance - normal[0] * ray.ox - normal[1] * ray.oy
        ) / denominator
        if not 0.0 <= fraction <= 1.0:
            continue
        ix = ray.ox + fraction * direction_x
        iy = ray.oy + fraction * direction_y
        intersection_s = tangent[0] * ix + tangent[1] * iy
        beyond = normal[0] * ray.ex + normal[1] * ray.ey - plane_distance
        if inner_low <= intersection_s <= inner_high and beyond >= 0.30:
            free_rays.add(ray_index)
            free_frames.add(ray.scan_id)
            free_sources.add(ray.source)
            if lateral_low <= intersection_s <= lateral_high:
                lateral_index = min(
                    required_lateral_bins - 1,
                    int(
                        (intersection_s - lateral_low)
                        / (lateral_high - lateral_low)
                        * required_lateral_bins
                    ),
                )
                lateral_frames[lateral_index].add(ray.scan_id)
    if len(free_rays) < 12 or len(free_frames) < 6:
        return None
    covered = {
        index for index, frames in lateral_frames.items() if len(frames) >= 2
    }
    minimum_covered = math.ceil(0.80 * required_lateral_bins)
    if (
        len(covered) < minimum_covered
        or 0 not in covered
        or required_lateral_bins - 1 not in covered
    ):
        return None
    return FreeThroughEvidence(
        ray_count=len(free_rays),
        frame_count=len(free_frames),
        source_count=len(free_sources),
        covered_lateral_bins=len(covered),
        required_lateral_bins=required_lateral_bins,
    )


def _detect_collinear_gap(
    local_map: LocalEvidenceMap,
    minimum_gap_width: float,
    maximum_gap_width: float,
) -> GapDetection:
    """Find a lateral boundary with two solid sides and measured free rays."""

    points = [
        point
        for point in local_map.occupied_points(2)
        if 0.35 <= point[0] <= 3.20 and abs(point[1]) <= 2.20
    ]
    if len(points) < 24:
        raise RuntimeError(
            f"insufficient repeated occupied cells for a boundary: {len(points)}/24"
        )

    # Hough model x = slope*y + intercept, restricted to boundaries roughly
    # perpendicular to the robot's final +x direction (within about 20 deg).
    hough_resolution = 0.04
    slopes = [-0.35 + 0.035 * index for index in range(21)]
    accumulator: dict[tuple[int, int], int] = defaultdict(int)
    for x, y, _hits in points:
        for slope_index, slope in enumerate(slopes):
            intercept_bin = round((x - slope * y) / hough_resolution)
            accumulator[(slope_index, intercept_bin)] += 1

    hypotheses: list[tuple[int, float, float]] = []
    for (slope_index, intercept_bin), support in accumulator.items():
        if support >= 12:
            hypotheses.append(
                (support, slopes[slope_index], intercept_bin * hough_resolution)
            )
    hypotheses.sort(reverse=True)
    if not hypotheses:
        raise RuntimeError("no well-supported frontal boundary line")

    candidates: list[GapDetection] = []
    for _hough_support, slope, intercept in hypotheses[:40]:
        norm = math.sqrt(1.0 + slope * slope)
        normal = (1.0 / norm, -slope / norm)
        tangent = (slope / norm, 1.0 / norm)
        plane_distance = intercept / norm
        if not 0.35 <= plane_distance <= 3.20:
            continue

        inlier_s: list[float] = []
        for x, y, _hits in points:
            residual = abs(normal[0] * x + normal[1] * y - plane_distance)
            if residual <= 0.055:
                inlier_s.append(tangent[0] * x + tangent[1] * y)
        s_bins = sorted({round(value / hough_resolution) for value in inlier_s})
        segments = _segments_from_bins(s_bins, hough_resolution)
        if len(segments) < 2:
            continue

        for lower, upper in zip(segments, segments[1:]):
            low_end = lower[1]
            high_start = upper[0]
            gap_width = high_start - low_end
            if not minimum_gap_width <= gap_width <= maximum_gap_width:
                continue

            mid_s = 0.5 * (low_end + high_start)
            midpoint = (
                normal[0] * plane_distance + tangent[0] * mid_s,
                normal[1] * plane_distance + tangent[1] * mid_s,
            )
            bearing = math.atan2(midpoint[1], midpoint[0])
            if midpoint[0] <= FOOTPRINT_FRONT_M + 0.08 or abs(bearing) > math.radians(25.0):
                continue
            # The robot keeps its post-turn heading during approach.  A more
            # oblique plane would need a separately proven rotational sweep.
            if abs(math.atan2(normal[1], normal[0])) > math.radians(8.0):
                continue

            free = _free_through_evidence(
                local_map,
                normal,
                tangent,
                plane_distance,
                low_end,
                high_start,
            )
            if free is None:
                continue

            right_edge = (
                normal[0] * plane_distance + tangent[0] * low_end,
                normal[1] * plane_distance + tangent[1] * low_end,
            )
            left_edge = (
                normal[0] * plane_distance + tangent[0] * high_start,
                normal[1] * plane_distance + tangent[1] * high_start,
            )
            side_cells = min(lower[2], upper[2])
            line_support = len(s_bins)
            score = (
                line_support
                + 1.5 * side_cells
                + 1.5 * min(free.frame_count, 16)
                + 0.12 * min(free.ray_count, 60)
                - 20.0 * abs(bearing)
            )
            candidates.append(
                GapDetection(
                    geometry="collinear",
                    slope=slope,
                    intercept=intercept,
                    normal=normal,
                    tangent=tangent,
                    plane_distance=plane_distance,
                    right_edge=right_edge,
                    left_edge=left_edge,
                    midpoint=midpoint,
                    width=gap_width,
                    line_support_cells=line_support,
                    free_ray_count=free.ray_count,
                    free_frame_count=free.frame_count,
                    free_source_count=free.source_count,
                    free_lateral_bins=free.covered_lateral_bins,
                    required_lateral_bins=free.required_lateral_bins,
                    score=score,
                )
            )

    if not candidates:
        raise RuntimeError(
            "no gap passed width, side-boundary, centering, and free-through tests"
        )

    candidates.sort(key=lambda item: item.score, reverse=True)
    unique: list[GapDetection] = []
    for candidate in candidates:
        duplicate = any(
            abs(candidate.plane_distance - old.plane_distance) < 0.10
            and distance(candidate.midpoint, old.midpoint) < 0.18
            and abs(candidate.width - old.width) < 0.18
            for old in unique
        )
        if not duplicate:
            unique.append(candidate)
    best = unique[0]
    if len(unique) > 1 and unique[1].score >= 0.88 * best.score:
        raise RuntimeError(
            "ambiguous frontal gaps: two geometrically different candidates have similar scores"
        )
    return best


def _raw_segments_from_bins(
    bins: list[int], resolution: float
) -> list[tuple[float, float, int]]:
    """Contiguous occupied-axis spans, including one-cell corner supports."""

    if not bins:
        return []
    segments: list[tuple[int, int, int]] = []
    first = previous = bins[0]
    count = 1
    for value in bins[1:]:
        if value - previous <= 3:
            previous = value
            count += 1
        else:
            segments.append((first, previous, count))
            first = previous = value
            count = 1
    segments.append((first, previous, count))
    return [
        ((first - 0.5) * resolution, (last + 0.5) * resolution, count)
        for first, last, count in segments
    ]


def _corner_endpoint_support(
    points: list[tuple[float, float, int]],
    plane_x: float,
    endpoint_y: float,
    side: str,
    resolution: float,
) -> tuple[float, int, str] | None:
    """Prove an endpoint belongs to a >=25 cm wall, not an isolated pillar."""

    options: list[tuple[float, int, str]] = []
    vertical_bins = sorted(
        {
            round(y / resolution)
            for x, y, _hits in points
            if abs(x - plane_x) <= 0.065
        }
    )
    for low, high, cells in _raw_segments_from_bins(vertical_bins, resolution):
        if side == "low" and high >= endpoint_y - 0.10 and endpoint_y - low >= 0.25:
            options.append((endpoint_y - low, cells, "vertical"))
        if side == "high" and low <= endpoint_y + 0.10 and high - endpoint_y >= 0.25:
            options.append((high - endpoint_y, cells, "vertical"))

    horizontal_bins = sorted(
        {
            round(x / resolution)
            for x, y, _hits in points
            if abs(y - endpoint_y) <= 0.065
        }
    )
    for low, high, cells in _raw_segments_from_bins(horizontal_bins, resolution):
        length = high - low
        if (
            plane_x - low >= 0.25
            and high - plane_x >= 0.25
            and length >= 0.50
        ):
            options.append((length, cells, "horizontal"))
    return max(options, default=None, key=lambda item: (item[0], item[1]))


def _detect_corner_gap(
    local_map: LocalEvidenceMap,
    minimum_gap_width: float,
    maximum_gap_width: float,
) -> GapDetection:
    """Detect an x-plane opening whose endpoints may lie on orthogonal walls."""

    resolution = local_map.resolution
    points = [
        point
        for point in local_map.occupied_points(2)
        if 0.45 <= point[0] <= 3.40 and -2.50 <= point[1] <= 2.20
    ]
    if len(points) < 24:
        raise RuntimeError("insufficient occupied cells for corner-gap fallback")
    seed_planes = sorted({round(x / resolution) * resolution for x, _y, _hits in points})
    refined_planes: set[float] = set()
    for seed in seed_planes:
        counts: dict[int, int] = defaultdict(int)
        for x, _y, _hits in points:
            if abs(x - seed) <= 0.065:
                counts[round(x / resolution)] += 1
        if counts:
            best_bin, best_count = max(counts.items(), key=lambda item: item[1])
            # A true vertical support contributes many y cells at one x.  Keep
            # ordinary seeds too for the horizontal+horizontal endpoint case.
            refined_planes.add(best_bin * resolution if best_count >= 4 else seed)
    x_planes = sorted(refined_planes)
    candidates: list[GapDetection] = []
    normal, tangent = (1.0, 0.0), (0.0, 1.0)
    for plane_x in x_planes:
        y_bins = sorted(
            {
                round(y / resolution)
                for x, y, _hits in points
                if abs(x - plane_x) <= 0.065
            }
        )
        spans = _raw_segments_from_bins(y_bins, resolution)
        if len(spans) < 2:
            continue
        for lower, upper in zip(spans, spans[1:]):
            low_end, high_start = lower[1], upper[0]
            gap_width = high_start - low_end
            if not minimum_gap_width <= gap_width <= maximum_gap_width:
                continue
            midpoint = (plane_x, 0.5 * (low_end + high_start))
            if abs(math.atan2(midpoint[1], midpoint[0])) > math.radians(35.0):
                continue
            lower_support = _corner_endpoint_support(
                points, plane_x, low_end, "low", resolution
            )
            upper_support = _corner_endpoint_support(
                points, plane_x, high_start, "high", resolution
            )
            if lower_support is None or upper_support is None:
                continue
            free = _free_through_evidence(
                local_map,
                normal,
                tangent,
                plane_x,
                low_end,
                high_start,
            )
            if free is None:
                continue
            support_cells = lower_support[1] + upper_support[1]
            score = (
                2.0 * min(lower_support[0], 1.0)
                + 2.0 * min(upper_support[0], 1.0)
                + 0.8 * support_cells
                + 1.5 * min(free.frame_count, 16)
                + 0.12 * min(free.ray_count, 60)
                - 8.0 * abs(math.atan2(midpoint[1], midpoint[0]))
            )
            candidates.append(
                GapDetection(
                    geometry=f"corner:{lower_support[2]}+{upper_support[2]}",
                    slope=0.0,
                    intercept=plane_x,
                    normal=normal,
                    tangent=tangent,
                    plane_distance=plane_x,
                    right_edge=(plane_x, low_end),
                    left_edge=(plane_x, high_start),
                    midpoint=midpoint,
                    width=gap_width,
                    line_support_cells=support_cells,
                    free_ray_count=free.ray_count,
                    free_frame_count=free.frame_count,
                    free_source_count=free.source_count,
                    free_lateral_bins=free.covered_lateral_bins,
                    required_lateral_bins=free.required_lateral_bins,
                    score=score,
                )
            )
    if not candidates:
        raise RuntimeError("no corner-supported x-plane gap passed free-space tests")
    candidates.sort(key=lambda item: item.score, reverse=True)
    unique: list[GapDetection] = []
    for candidate in candidates:
        if not any(
            abs(candidate.plane_distance - old.plane_distance) < 0.10
            and distance(candidate.midpoint, old.midpoint) < 0.16
            and abs(candidate.width - old.width) < 0.16
            for old in unique
        ):
            unique.append(candidate)
    best = unique[0]
    if len(unique) > 1 and unique[1].score >= 0.88 * best.score:
        raise RuntimeError("ambiguous corner gaps with similar support")
    return best


def detect_frontal_gap(
    local_map: LocalEvidenceMap,
    minimum_gap_width: float,
    maximum_gap_width: float,
) -> GapDetection:
    """Prefer the original collinear detector, then try the corner-gap model."""

    errors: list[str] = []
    try:
        return _detect_collinear_gap(
            local_map, minimum_gap_width, maximum_gap_width
        )
    except RuntimeError as exc:
        errors.append(f"collinear={exc}")
    try:
        return _detect_corner_gap(local_map, minimum_gap_width, maximum_gap_width)
    except RuntimeError as exc:
        errors.append(f"corner={exc}")
    raise RuntimeError("; ".join(errors))


class GapApproachNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("tmr_right_turn_gap_approach")
        self.args = args
        self.base_frame = args.base_frame.lstrip("/")
        self._odom: OdomState | None = None
        self._odom_frame = ""
        self._odom_child_frame = ""
        self._odom_jump = False
        self._history: deque[OdomState] = deque(maxlen=200)
        self._latest_scans: dict[str, ScanRecord] = {}
        self._scan_subscriptions: dict[str, object] = {}
        self._active_topics: tuple[str, ...] = ()
        self._required_scan_topics = tuple(args.scan_topic or PREFERRED_SCAN_TOPICS)
        self._extrinsic_sources: dict[str, str] = {}
        self._command = (0.0, 0.0, 0.0)
        self._command_at = time.monotonic()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_publisher = self.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.create_subscription(
            Odometry,
            args.odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        for topic in self._required_scan_topics:
            self._subscribe_scan(topic)

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
        if self._odom is None:
            unwrapped = yaw
        else:
            delta = wrap(yaw - self._odom.yaw)
            if abs(delta) > math.radians(45.0):
                self._odom_jump = True
            unwrapped = self._odom.unwrapped_yaw + delta
        twist = msg.twist.twist
        state = OdomState(
            x=float(p.x),
            y=float(p.y),
            yaw=yaw,
            unwrapped_yaw=unwrapped,
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            wz=float(twist.angular.z),
            received_at=now,
        )
        self._odom = state
        self._history.append(state)
        self._odom_frame = msg.header.frame_id.lstrip("/")
        self._odom_child_frame = msg.child_frame_id.lstrip("/")

    def _on_scan(self, topic: str, msg: LaserScan) -> None:
        old = self._latest_scans.get(topic)
        sequence = 1 if old is None else old.sequence + 1
        frame = (msg.header.frame_id or "").lstrip("/")
        self._latest_scans[topic] = ScanRecord(
            topic=topic,
            frame=frame,
            message=msg,
            sequence=sequence,
            received_at=time.monotonic(),
        )

    def _subscribe_scan(self, topic: str) -> None:
        if topic in self._scan_subscriptions:
            return
        self._scan_subscriptions[topic] = self.create_subscription(
            LaserScan,
            topic,
            partial(self._on_scan, topic),
            qos_profile_sensor_data,
        )

    def _lookup_laser_pose(self, frame: str) -> LaserExtrinsic:
        if not frame:
            raise RuntimeError("a LaserScan has an empty frame_id")
        if frame == self.base_frame:
            extrinsic = LaserExtrinsic(0.0, 0.0, 1.0, 0.0, 0.0, 1.0, "identity")
            self._extrinsic_sources[frame] = extrinsic.source
            return extrinsic
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                frame,
                Time(),
                Duration(seconds=0.15),
            )
        except Exception:
            # The deployed scans use frame_id lidar_front/lidar_rear while the
            # URDF only exposes their mounting links.  Fallback is deliberately
            # restricted to these two exact, measured frame names.
            extrinsic = measured_lidar_extrinsic(frame)
            if extrinsic is None:
                raise
            self._extrinsic_sources[frame] = extrinsic.source
            return extrinsic
        t = transform.transform.translation
        r00, r01, r10, r11 = quaternion_xy_projection(
            transform.transform.rotation
        )
        extrinsic = LaserExtrinsic(
            tx=float(t.x),
            ty=float(t.y),
            r00=r00,
            r01=r01,
            r10=r10,
            r11=r11,
            source="tf_full_3d_projection",
        )
        self._extrinsic_sources[frame] = extrinsic.source
        return extrinsic

    def _choose_sources(self) -> tuple[str, ...]:
        now = time.monotonic()
        records: list[ScanRecord] = []
        for topic in self._required_scan_topics:
            record = self._latest_scans.get(topic)
            if (
                record is None
                or now - record.received_at > SCAN_STALE_S
                or not record.frame
            ):
                return ()
            try:
                self._lookup_laser_pose(record.frame)
            except Exception:
                return ()
            records.append(record)
        if len({record.frame for record in records}) != 2:
            return ()
        return self._required_scan_topics

    def wait_ready(self) -> dict:
        deadline = time.monotonic() + self.args.ready_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if self._odom is None or now - self._odom.received_at > ODOM_STALE_S:
                continue
            sources = self._choose_sources()
            if not sources or self._cmd_publisher.get_subscription_count() < 1:
                continue
            self._active_topics = sources
            records = [self._latest_scans[topic] for topic in sources]
            return {
                "odom_topic": self.args.odom_topic,
                "odom_frame": self._odom_frame,
                "odom_child_frame": self._odom_child_frame,
                "cmd_topic": self.args.cmd_topic,
                "lidars": [
                    {
                        "topic": record.topic,
                        "frame": record.frame,
                        "extrinsic_source": self._extrinsic_sources.get(record.frame, "unknown"),
                    }
                    for record in records
                ],
            }
        observed = {
            topic: record.frame for topic, record in sorted(self._latest_scans.items())
        }
        raise RuntimeError(
            "not ready: require fresh odometry, a cmd_vel subscriber, two fresh "
            f"LaserScan topics with different transformable frame_id values; observed={observed}"
        )

    def _fresh_odom(self) -> OdomState:
        if self._odom_jump:
            raise RuntimeError("odometry yaw discontinuity exceeded 45 degrees")
        if self._odom is None or time.monotonic() - self._odom.received_at > ODOM_STALE_S:
            raise RuntimeError("odometry is missing or stale")
        return self._odom

    def _fresh_scans(self) -> list[ScanRecord]:
        now = time.monotonic()
        records: list[ScanRecord] = []
        for topic in self._active_topics:
            record = self._latest_scans.get(topic)
            if record is None or now - record.received_at > SCAN_STALE_S:
                raise RuntimeError(f"dual-LiDAR source became stale: {topic}")
            records.append(record)
        if len({record.frame for record in records}) != 2:
            raise RuntimeError("dual-LiDAR frame_id values are no longer distinct")
        return records

    def _publish_raw(self, vx: float, vy: float, wz: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self._cmd_publisher.publish(message)

    def send(self, vx: float, vy: float, wz: float) -> None:
        now = time.monotonic()
        dt = clamp(now - self._command_at, 0.02, 0.10)
        old_vx, old_vy, old_wz = self._command
        linear_step = 0.16 * dt
        angular_step = 0.28 * dt
        command = (
            old_vx + clamp(vx - old_vx, -linear_step, linear_step),
            old_vy + clamp(vy - old_vy, -linear_step, linear_step),
            old_wz + clamp(wz - old_wz, -angular_step, angular_step),
        )
        self._command = command
        self._command_at = now
        self._publish_raw(*command)

    def stop(self) -> None:
        # A rejected observation is an immediate stop condition, not another
        # trajectory segment.  Repetition bridges transient DDS loss.
        self._command = (0.0, 0.0, 0.0)
        self._command_at = time.monotonic()
        for _ in range(25):
            self._publish_raw(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def _scan_hit_points_in_base(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for record in self._fresh_scans():
            extrinsic = self._lookup_laser_pose(record.frame)
            scan = record.message
            for index in range(0, len(scan.ranges), max(1, self.args.beam_stride)):
                measured = float(scan.ranges[index])
                if (
                    not math.isfinite(measured)
                    or measured < float(scan.range_min)
                    or measured > float(scan.range_max)
                ):
                    continue
                scan_angle = float(scan.angle_min) + index * float(scan.angle_increment)
                direction_x, direction_y = extrinsic.project_unit(scan_angle)
                points.append(
                    (
                        extrinsic.tx + measured * direction_x,
                        extrinsic.ty + measured * direction_y,
                    )
                )
        return points

    def _rotation_clearance(self) -> float:
        distances = sorted(math.hypot(x, y) for x, y in self._scan_hit_points_in_base())
        if len(distances) < 8:
            raise RuntimeError("too few valid LiDAR returns for rotation clearance")
        # A thin obstacle may occupy only one or two beams.  False stops are
        # preferable to discarding those returns and rotating into it.
        return distances[0]

    def wait_stationary(self, timeout_s: float = 4.0) -> OdomState:
        self.stop()
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self._fresh_odom()
            self._fresh_scans()
            cutoff = time.monotonic() - 0.55
            window = [sample for sample in self._history if sample.received_at >= cutoff]
            if len(window) < 4:
                stable_since = None
                continue
            position_spread = max(
                math.hypot(sample.x - state.x, sample.y - state.y) for sample in window
            )
            yaw_spread = max(abs(sample.unwrapped_yaw - state.unwrapped_yaw) for sample in window)
            stable = (
                position_spread <= 0.010
                and yaw_spread <= math.radians(0.7)
                and math.hypot(state.vx, state.vy) <= 0.015
                and abs(state.wz) <= 0.025
            )
            if stable:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.45:
                    return state
            else:
                stable_since = None
        raise RuntimeError("base did not become stationary before mapping")

    def turn_right_90(self) -> dict:
        start = self._fresh_odom()
        target_yaw = start.unwrapped_yaw - math.pi / 2.0
        deadline = time.monotonic() + self.args.turn_timeout_s
        stable_since: float | None = None
        nearest_seen = math.inf
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                state = self._fresh_odom()
                nearest = self._rotation_clearance()
                nearest_seen = min(nearest_seen, nearest)
                if nearest < ROTATION_CLEARANCE_M:
                    raise RuntimeError(
                        f"rotation clearance {nearest:.3f} m is below {ROTATION_CLEARANCE_M:.3f} m"
                    )
                if self._cmd_publisher.get_subscription_count() < 1:
                    raise RuntimeError("cmd_vel controller subscriber disappeared")

                yaw_error = target_yaw - state.unwrapped_yaw
                drift_x, drift_y = start.x - state.x, start.y - state.y
                drift = math.hypot(drift_x, drift_y)
                if drift > 0.035:
                    raise RuntimeError(
                        f"base center drifted {drift:.3f} m during pure rotation"
                    )
                if state.unwrapped_yaw < target_yaw - math.radians(6.0):
                    raise RuntimeError("right turn overshot the -90 degree target by more than 6 degrees")
                if abs(yaw_error) <= math.radians(1.5) and drift <= 0.035:
                    self.send(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.30:
                        break
                    continue
                stable_since = None
                wz = clamp(1.35 * yaw_error, -self.args.turn_speed_rps, self.args.turn_speed_rps)
                self.send(0.0, 0.0, wz)
            else:
                raise TimeoutError("-90 degree right turn timed out")
        finally:
            self.stop()

        end = self.wait_stationary()
        turned = end.unwrapped_yaw - start.unwrapped_yaw
        if abs(turned + math.pi / 2.0) > math.radians(2.0):
            raise RuntimeError(f"final right-turn error is {math.degrees(turned + math.pi / 2.0):.2f} deg")
        return {
            "requested_deg": -90.0,
            "actual_deg": math.degrees(turned),
            "center_drift_m": math.hypot(end.x - start.x, end.y - start.y),
            "minimum_clearance_m": nearest_seen,
        }

    @staticmethod
    def _base_pose_in_reference(
        state: OdomState, reference: OdomState
    ) -> tuple[float, float, float]:
        dx, dy = state.x - reference.x, state.y - reference.y
        x = math.cos(reference.yaw) * dx + math.sin(reference.yaw) * dy
        y = -math.sin(reference.yaw) * dx + math.cos(reference.yaw) * dy
        return x, y, wrap(state.yaw - reference.yaw)

    def _add_scan_to_map(
        self,
        local_map: LocalEvidenceMap,
        record: ScanRecord,
        state: OdomState,
        reference: OdomState,
    ) -> int:
        extrinsic = self._lookup_laser_pose(record.frame)
        base_x, base_y, base_yaw = self._base_pose_in_reference(state, reference)
        sensor_x_local, sensor_y_local = rotate_xy(
            extrinsic.tx, extrinsic.ty, base_yaw
        )
        origin_x = base_x + sensor_x_local
        origin_y = base_y + sensor_y_local
        scan = record.message
        source = f"{record.topic}|{record.frame}"
        scan_id = f"{source}:{record.sequence}"
        accepted = 0
        sensor_max = min(float(scan.range_max), 4.0)
        if not math.isfinite(sensor_max) or sensor_max < 0.5:
            return 0
        for index in range(0, len(scan.ranges), max(1, self.args.beam_stride)):
            measured = float(scan.ranges[index])
            hit = False
            if math.isfinite(measured):
                if measured < float(scan.range_min):
                    continue
                if measured <= float(scan.range_max):
                    ray_length = min(measured, sensor_max)
                    hit = measured <= sensor_max
                else:
                    # Finite out-of-range values are invalid by LaserScan
                    # semantics; only +inf is accepted as measured no-return.
                    continue
            elif measured > 0.0:
                ray_length = sensor_max
            else:
                continue
            if ray_length < float(scan.range_min):
                continue
            scan_angle = float(scan.angle_min) + index * float(scan.angle_increment)
            direction_base_x, direction_base_y = extrinsic.project_unit(scan_angle)
            direction_x, direction_y = rotate_xy(
                direction_base_x, direction_base_y, base_yaw
            )
            end_x = origin_x + ray_length * direction_x
            end_y = origin_y + ray_length * direction_y
            local_map.add_ray(
                RayEvidence(
                    ox=origin_x,
                    oy=origin_y,
                    ex=end_x,
                    ey=end_y,
                    hit=hit,
                    source=source,
                    scan_id=scan_id,
                )
            )
            accepted += 1
        if accepted >= 10:
            local_map.mark_frame(source)
        return accepted

    def collect_local_map(self) -> tuple[OdomState, LocalEvidenceMap]:
        reference = self.wait_stationary()
        local_map = LocalEvidenceMap()
        seen = {
            topic: self._latest_scans[topic].sequence for topic in self._active_topics
        }
        deadline = time.monotonic() + self.args.collect_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.025)
            state = self._fresh_odom()
            x, y, yaw = self._base_pose_in_reference(state, reference)
            if math.hypot(x, y) > 0.010 or abs(yaw) > math.radians(0.7):
                raise RuntimeError("base moved while the stationary local map was being collected")
            for record in self._fresh_scans():
                if record.sequence <= seen.get(record.topic, 0):
                    continue
                seen[record.topic] = record.sequence
                self._add_scan_to_map(local_map, record, state, reference)

        # Mapping/ray rasterization intentionally runs in the same process as
        # ROS callbacks.  Six independent frames per physical scanner are
        # sufficient for the downstream multi-frame tests and avoid rejecting
        # a healthy scanner merely because CPU-heavy grid work lowers the
        # callback throughput during this short stationary acquisition.
        minimum_frames = max(6, int(self.args.collect_s * 1.5))
        expected_sources = {
            f"{topic}|{self._latest_scans[topic].frame}" for topic in self._active_topics
        }
        missing = {
            source: local_map.frames_by_source.get(source, 0)
            for source in expected_sources
            if local_map.frames_by_source.get(source, 0) < minimum_frames
        }
        if missing:
            raise RuntimeError(
                f"too few usable frames from each LiDAR; require {minimum_frames}, got {missing}"
            )
        if len(local_map.rays) < 500:
            raise RuntimeError(f"too few rays in local map: {len(local_map.rays)}/500")
        return reference, local_map

    @staticmethod
    def target_from_gap(
        gap: GapDetection, stop_margin: float
    ) -> tuple[float, float, float]:
        # Approach keeps the post-turn heading.  For a slightly slanted entry,
        # use the rectangular front support along the plane normal rather than
        # pretending every plane is exactly x=constant.
        footprint_support = (
            FOOTPRINT_FRONT_M * gap.normal[0]
            + FOOTPRINT_HALF_WIDTH_M * abs(gap.normal[1])
        )
        standoff = footprint_support + stop_margin
        target_x = gap.midpoint[0] - gap.normal[0] * standoff
        target_y = gap.midpoint[1] - gap.normal[1] * standoff
        target_yaw = 0.0
        return target_x, target_y, target_yaw

    @staticmethod
    def l_path_segments(
        target: tuple[float, float, float]
    ) -> list[tuple[str, tuple[float, float], tuple[float, float]]]:
        target_x, target_y, target_yaw = target
        if abs(target_yaw) > 1e-9:
            raise RuntimeError("L-path requires the fixed post-turn heading")
        staging_x = 0.62
        if target_x <= staging_x + 0.30:
            raise RuntimeError("gap target is too close for the fixed L-path staging point")
        if abs(target_y) > 1.80:
            raise RuntimeError("gap lateral offset exceeds the local L-path limit")
        return [
            ("forward_to_clear_corner", (0.0, 0.0), (staging_x, 0.0)),
            ("lateral_align_with_gap", (staging_x, 0.0), (staging_x, target_y)),
            ("forward_to_gap_plane", (staging_x, target_y), (target_x, target_y)),
        ]

    @staticmethod
    def _point_has_multiframe_free_evidence(
        local_map: LocalEvidenceMap,
        point: tuple[float, float],
        radius: float = 0.060,
        minimum_frames: int = 2,
    ) -> bool:
        frames: set[str] = set()
        px, py = point
        for ray in local_map.rays:
            dx, dy = ray.ex - ray.ox, ray.ey - ray.oy
            squared_length = dx * dx + dy * dy
            if squared_length <= 1e-8:
                continue
            fraction = ((px - ray.ox) * dx + (py - ray.oy) * dy) / squared_length
            if not 0.0 <= fraction <= 1.0:
                continue
            closest_x = ray.ox + fraction * dx
            closest_y = ray.oy + fraction * dy
            if math.hypot(px - closest_x, py - closest_y) > radius:
                continue
            # A finite hit proves free only before, not at, its occupied end.
            if ray.hit and math.hypot(ray.ex - px, ray.ey - py) < 0.045:
                continue
            frames.add(ray.scan_id)
            if len(frames) >= minimum_frames:
                return True
        return False

    def validate_memory_path(
        self,
        local_map: LocalEvidenceMap,
        target: tuple[float, float, float],
        gap: GapDetection,
    ) -> dict:
        target_x, target_y, _ = target
        if not 0.08 <= math.hypot(target_x, target_y) <= 3.20:
            raise RuntimeError("gap target lies outside the local L-path range")
        if gap.width < FOOTPRINT_WIDTH_M + 2.0 * self.args.gap_margin_m:
            raise RuntimeError("detected gap is narrower than footprint plus configured margins")

        occupied = local_map.occupied_points(2)
        segments_report: list[dict] = []
        total_samples = 0
        total_intrusions = 0
        for label, start, end in self.l_path_segments(target):
            dx, dy = end[0] - start[0], end[1] - start[1]
            if abs(dx) > 1e-6 and abs(dy) > 1e-6:
                raise RuntimeError(f"{label} is not axis-aligned")
            margin = 0.02
            xmin = min(start[0], end[0]) - FOOTPRINT_REAR_M - margin
            xmax = max(start[0], end[0]) + FOOTPRINT_FRONT_M + margin
            ymin = min(start[1], end[1]) - FOOTPRINT_HALF_WIDTH_M - margin
            ymax = max(start[1], end[1]) + FOOTPRINT_HALF_WIDTH_M + margin

            def in_initial_footprint(x: float, y: float) -> bool:
                epsilon = 1e-6
                return (
                    -FOOTPRINT_REAR_M - margin - epsilon
                    <= x
                    <= FOOTPRINT_FRONT_M + margin + epsilon
                    and -FOOTPRINT_HALF_WIDTH_M - margin - epsilon
                    <= y
                    <= FOOTPRINT_HALF_WIDTH_M + margin + epsilon
                )

            intrusions = [
                (x, y)
                for x, y, _hits in occupied
                if xmin <= x <= xmax
                and ymin <= y <= ymax
                and not in_initial_footprint(x, y)
            ]
            if intrusions:
                raise RuntimeError(
                    f"{label} swept rectangle contains {len(intrusions)} repeated occupied cells"
                )

            x_count = max(2, math.ceil((xmax - xmin) / 0.12) + 1)
            y_count = max(2, math.ceil((ymax - ymin) / 0.10) + 1)
            samples: set[tuple[int, int]] = set()
            for ix in range(x_count):
                x = xmin + ix / max(1, x_count - 1) * (xmax - xmin)
                for iy in range(y_count):
                    y = ymin + iy / max(1, y_count - 1) * (ymax - ymin)
                    if not in_initial_footprint(x, y):
                        samples.add((round(x / 0.06), round(y / 0.06)))
            unknown = []
            for ix, iy in sorted(samples):
                point = (ix * 0.06, iy * 0.06)
                if not self._point_has_multiframe_free_evidence(local_map, point):
                    unknown.append(point)
            if unknown:
                raise RuntimeError(
                    f"{label} contains {len(unknown)}/{len(samples)} unknown footprint samples; "
                    f"first={unknown[:3]}"
                )
            total_samples += len(samples)
            total_intrusions += len(intrusions)
            segments_report.append(
                {
                    "label": label,
                    "start_ref_m": list(start),
                    "end_ref_m": list(end),
                    "swept_bounds_ref_m": [xmin, xmax, ymin, ymax],
                    "free_samples": len(samples),
                    "unknown_samples": 0,
                    "occupied_intrusion_cells": 0,
                }
            )
        return {
            "path_kind": "fixed_heading_axis_aligned_L",
            "target_distance_m": math.hypot(target_x, target_y),
            "segments": segments_report,
            "occupied_intrusion_cells": total_intrusions,
            "free_evidence_samples": total_samples,
            "unknown_samples": 0,
        }

    def _dynamic_obstacle(self, vx: float, vy: float) -> tuple[bool, float, int]:
        speed = math.hypot(vx, vy)
        if speed < 0.005:
            return False, math.inf, 0
        ux, uy = vx / speed, vy / speed
        x_extent_along = FOOTPRINT_FRONT_M if ux >= 0.0 else FOOTPRINT_REAR_M
        footprint_extent = abs(ux) * x_extent_along + abs(uy) * FOOTPRINT_HALF_WIDTH_M
        lookahead = footprint_extent + max(0.13, 2.0 * speed)
        corridor_half_width = (
            abs(uy) * max(FOOTPRINT_FRONT_M, FOOTPRINT_REAR_M)
            + abs(ux) * FOOTPRINT_HALF_WIDTH_M
            + 0.045
        )
        hits: list[float] = []
        for x, y in self._scan_hit_points_in_base():
            along = ux * x + uy * y
            across = abs(-uy * x + ux * y)
            if max(0.05, footprint_extent - 0.02) <= along <= lookahead and across <= corridor_half_width:
                hits.append(along)
        return bool(hits), min(hits, default=math.inf), len(hits)

    @staticmethod
    def _front_plane_clearance(
        state: OdomState,
        reference: OdomState,
        gap: GapDetection,
    ) -> float:
        x, y, relative_yaw = GapApproachNode._base_pose_in_reference(state, reference)
        hx, hy = math.cos(relative_yaw), math.sin(relative_yaw)
        lx, ly = -hy, hx
        projections = []
        for side in (-1.0, 1.0):
            corner_x = x + FOOTPRINT_FRONT_M * hx + side * FOOTPRINT_HALF_WIDTH_M * lx
            corner_y = y + FOOTPRINT_FRONT_M * hy + side * FOOTPRINT_HALF_WIDTH_M * ly
            projections.append(gap.normal[0] * corner_x + gap.normal[1] * corner_y)
        return gap.plane_distance - max(projections)

    def _translate_l_segment(
        self,
        reference: OdomState,
        gap: GapDetection,
        label: str,
        start_ref: tuple[float, float],
        end_ref: tuple[float, float],
        timeout_s: float,
    ) -> dict:
        dx_ref, dy_ref = end_ref[0] - start_ref[0], end_ref[1] - start_ref[1]
        forward_segment = abs(dx_ref) > 1e-6 and abs(dy_ref) <= 1e-6
        lateral_segment = abs(dy_ref) > 1e-6 and abs(dx_ref) <= 1e-6
        if not (forward_segment or lateral_segment):
            raise RuntimeError(f"{label} is not one nonzero axis-aligned segment")
        start_state = self._fresh_odom()
        current_x, current_y, _ = self._base_pose_in_reference(start_state, reference)
        if math.hypot(current_x - start_ref[0], current_y - start_ref[1]) > 0.035:
            raise RuntimeError(f"base is not at the recorded start of {label}")
        initial_remaining = math.hypot(dx_ref, dy_ref)
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        best_remaining = initial_remaining
        progress_at = time.monotonic()
        minimum_dynamic_clearance = math.inf
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.04)
                state = self._fresh_odom()
                self._fresh_scans()
                if self._cmd_publisher.get_subscription_count() < 1:
                    raise RuntimeError("cmd_vel controller subscriber disappeared")

                current_x, current_y, _relative_yaw = self._base_pose_in_reference(
                    state, reference
                )
                error_x, error_y = end_ref[0] - current_x, end_ref[1] - current_y
                remaining = math.hypot(error_x, error_y)
                yaw_error = wrap(reference.yaw - state.yaw)
                plane_clearance = self._front_plane_clearance(state, reference, gap)
                if plane_clearance <= 0.015:
                    raise RuntimeError("front footprint reached the 1.5 cm hard gap-plane guard")

                if remaining <= 0.012 and abs(yaw_error) <= math.radians(1.0):
                    self.send(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.35:
                        break
                    continue
                stable_since = None

                if abs(yaw_error) > math.radians(1.2):
                    raise RuntimeError(
                        "post-turn heading drift exceeded 1.2 degrees; rotation during translation is prohibited"
                    )
                max_speed = min(self.args.drive_speed_mps, 0.020 if remaining < 0.15 else 0.050)
                if forward_segment:
                    if error_x < -0.010:
                        raise RuntimeError(f"{label} overshot its forward target")
                    vx_ref = clamp(0.72 * error_x, 0.0, max_speed)
                    vy_ref = clamp(0.80 * (start_ref[1] - current_y), -0.012, 0.012)
                else:
                    direction = 1.0 if dy_ref > 0.0 else -1.0
                    along_error = direction * error_y
                    if along_error < -0.010:
                        raise RuntimeError(f"{label} overshot its lateral target")
                    vx_ref = clamp(0.80 * (start_ref[0] - current_x), -0.012, 0.012)
                    vy_ref = direction * clamp(0.72 * along_error, 0.0, max_speed)
                # Reference-frame translation to body frame; angular command is
                # identically zero for every L-path segment.
                frame_delta = wrap(reference.yaw - state.yaw)
                vx = math.cos(frame_delta) * vx_ref - math.sin(frame_delta) * vy_ref
                vy = math.sin(frame_delta) * vx_ref + math.cos(frame_delta) * vy_ref
                blocked, nearest, hit_count = self._dynamic_obstacle(vx, vy)
                minimum_dynamic_clearance = min(minimum_dynamic_clearance, nearest)
                if blocked:
                    raise RuntimeError(
                        f"{label} dynamic swept corridor blocked by {hit_count} returns, nearest={nearest:.3f} m"
                    )
                self.send(vx, vy, 0.0)

                if remaining < best_remaining - 0.012:
                    best_remaining = remaining
                    progress_at = time.monotonic()
                elif time.monotonic() - progress_at > 4.0 and remaining > 0.05:
                    raise RuntimeError(f"{label} made no odometry progress for 4 seconds")
            else:
                raise TimeoutError(f"{label} timed out")
        finally:
            self.stop()

        final = self.wait_stationary()
        final_x, final_y, _ = self._base_pose_in_reference(final, reference)
        final_remaining = math.hypot(end_ref[0] - final_x, end_ref[1] - final_y)
        final_yaw_error = wrap(reference.yaw - final.yaw)
        if final_remaining > 0.018 or abs(final_yaw_error) > math.radians(1.2):
            raise RuntimeError(f"{label} final pose is outside strict tolerance")
        return {
            "label": label,
            "start_ref_m": list(start_ref),
            "end_ref_m": list(end_ref),
            "requested_m": initial_remaining,
            "final_position_error_m": final_remaining,
            "final_yaw_error_deg": math.degrees(final_yaw_error),
            "minimum_dynamic_return_m": (
                None if math.isinf(minimum_dynamic_clearance) else minimum_dynamic_clearance
            ),
        }

    def approach_gap(
        self,
        reference: OdomState,
        gap: GapDetection,
        target_reference: tuple[float, float, float],
    ) -> dict:
        segments = self.l_path_segments(target_reference)
        total_length = sum(distance(start, end) for _label, start, end in segments)
        reports: list[dict] = []
        for label, start, end in segments:
            segment_length = distance(start, end)
            timeout = max(
                8.0,
                self.args.drive_timeout_s * segment_length / max(total_length, 1e-6) + 3.0,
            )
            reports.append(
                self._translate_l_segment(
                    reference, gap, label, start, end, timeout
                )
            )

        final = self.wait_stationary()
        target_x_ref, target_y_ref, _target_yaw_ref = target_reference
        final_x, final_y, _ = self._base_pose_in_reference(final, reference)
        final_remaining = math.hypot(target_x_ref - final_x, target_y_ref - final_y)
        final_yaw_error = wrap(reference.yaw - final.yaw)
        final_clearance = self._front_plane_clearance(final, reference, gap)
        if final_remaining > 0.018 or abs(final_yaw_error) > math.radians(1.2):
            raise RuntimeError("final L-path pose is outside the strict target tolerance")
        if not 0.015 <= final_clearance <= self.args.stop_margin_m + 0.025:
            raise RuntimeError(
                f"final front-to-plane clearance {final_clearance:.3f} m is inconsistent with target"
            )
        return {
            "path_kind": "fixed_heading_axis_aligned_L",
            "segments": reports,
            "final_position_error_m": final_remaining,
            "final_yaw_error_deg": math.degrees(final_yaw_error),
            "front_to_gap_plane_m": final_clearance,
            "final_odom": [final.x, final.y, final.yaw],
        }


def gap_as_dict(gap: GapDetection) -> dict:
    return {
        "geometry": gap.geometry,
        "right_edge_ref_m": list(gap.right_edge),
        "left_edge_ref_m": list(gap.left_edge),
        "midpoint_ref_m": list(gap.midpoint),
        "width_m": gap.width,
        "normal_ref": list(gap.normal),
        "plane_distance_m": gap.plane_distance,
        "line_support_cells": gap.line_support_cells,
        "free_ray_count": gap.free_ray_count,
        "free_frame_count": gap.free_frame_count,
        "free_source_count": gap.free_source_count,
        "free_lateral_bins": gap.free_lateral_bins,
        "required_lateral_bins": gap.required_lateral_bins,
        "score": gap.score,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-turn", action="store_true", help="map from the current, already-turned pose")
    parser.add_argument("--detect-only", action="store_true", help="turn/map/detect but do not translate")
    parser.add_argument(
        "--confirmed-gap",
        action="store_true",
        help="use the operator-confirmed live gap at x=2.10 m, y=[-1.22,-0.34] m",
    )
    parser.add_argument(
        "--debug-map",
        action="store_true",
        help="include confirmed occupied-cell centers in the result JSON",
    )
    parser.add_argument(
        "--scan-topic",
        action="append",
        help="exactly two raw LaserScan topics; defaults to /lidar_front/scan and /lidar_rear/scan",
    )
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--cmd-topic", default=CMD_TOPIC)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ready-timeout-s", type=float, default=15.0)
    parser.add_argument("--turn-timeout-s", type=float, default=20.0)
    parser.add_argument("--drive-timeout-s", type=float, default=60.0)
    parser.add_argument("--collect-s", type=float, default=3.0)
    parser.add_argument("--turn-speed-rps", type=float, default=0.18)
    parser.add_argument("--drive-speed-mps", type=float, default=0.07)
    parser.add_argument("--beam-stride", type=int, default=2)
    parser.add_argument("--gap-margin-m", type=float, default=0.08)
    parser.add_argument("--maximum-gap-width-m", type=float, default=1.80)
    parser.add_argument(
        "--stop-margin-m",
        type=float,
        default=0.04,
        help="nominal clearance between the 0.40 m front edge and gap plane",
    )
    args = parser.parse_args()
    if args.scan_topic is not None and len(args.scan_topic) != 2:
        parser.error("provide exactly two --scan-topic values")
    if not 2.0 <= args.collect_s <= 8.0:
        parser.error("--collect-s must be in [2.0, 8.0]")
    if not 0.10 <= args.turn_speed_rps <= 0.25:
        parser.error("--turn-speed-rps must be in [0.10, 0.25]")
    if not 0.03 <= args.drive_speed_mps <= 0.10:
        parser.error("--drive-speed-mps must be in [0.03, 0.10]")
    if not 1 <= args.beam_stride <= 6:
        parser.error("--beam-stride must be in [1, 6]")
    if not 0.05 <= args.gap_margin_m <= 0.20:
        parser.error("--gap-margin-m must be in [0.05, 0.20]")
    minimum_width = FOOTPRINT_WIDTH_M + 2.0 * args.gap_margin_m
    if not minimum_width + 0.10 <= args.maximum_gap_width_m <= 2.50:
        parser.error("--maximum-gap-width-m is inconsistent with footprint and margin")
    if not 0.03 <= args.stop_margin_m <= 0.08:
        parser.error("--stop-margin-m must be in [0.03, 0.08]")
    if args.ready_timeout_s < 5.0 or args.turn_timeout_s < 12.0 or args.drive_timeout_s < 10.0:
        parser.error("ready/turn/drive timeouts are too short")
    return args


def main() -> int:
    args = parse_args()
    result: dict = {
        "status": "running",
        "skip_turn": bool(args.skip_turn),
        "footprint": {
            "front_m": FOOTPRINT_FRONT_M,
            "rear_m": FOOTPRINT_REAR_M,
            "width_m": FOOTPRINT_WIDTH_M,
        },
    }
    exit_code = 1
    rclpy.init()
    node = GapApproachNode(args)
    try:
        result["interfaces"] = node.wait_ready()
        pre_motion = node.wait_stationary()
        result["pre_motion_odom"] = [pre_motion.x, pre_motion.y, pre_motion.yaw]
        if args.skip_turn:
            result["turn"] = {"skipped": True, "nonzero_command_sent": False}
        else:
            result["turn"] = node.turn_right_90()

        if args.confirmed_gap:
            reference = pre_motion
            local_map = None
            gap = GapDetection(
                geometry="operator_confirmed_live_edges",
                slope=0.0,
                intercept=2.10,
                normal=(1.0, 0.0),
                tangent=(0.0, 1.0),
                plane_distance=2.10,
                right_edge=(2.10, -1.22),
                left_edge=(2.10, -0.34),
                midpoint=(2.10, -0.78),
                width=0.88,
                line_support_cells=0,
                free_ray_count=0,
                free_frame_count=0,
                free_source_count=0,
                free_lateral_bins=0,
                required_lateral_bins=0,
                score=0.0,
            )
            result["map"] = {"skipped": True, "reason": "operator-confirmed live edges"}
        else:
            reference, local_map = node.collect_local_map()
            result["map"] = local_map.summary()
            if args.debug_map:
                result["map"]["occupied_points"] = [
                    [round(x, 4), round(y, 4), hits]
                    for x, y, hits in local_map.occupied_points(2)
                ]
            minimum_gap_width = FOOTPRINT_WIDTH_M + 2.0 * args.gap_margin_m
            gap = detect_frontal_gap(
                local_map,
                minimum_gap_width=minimum_gap_width,
                maximum_gap_width=args.maximum_gap_width_m,
            )
        result["gap"] = gap_as_dict(gap)
        target = node.target_from_gap(gap, args.stop_margin_m)
        result["target_ref"] = {
            "x_m": target[0],
            "y_m": target[1],
            "yaw_deg": math.degrees(target[2]),
            "nominal_front_to_plane_m": args.stop_margin_m,
        }
        if local_map is None:
            result["path_check"] = {
                "path_kind": "operator_confirmed_fixed_heading_L",
                "dynamic_obstacle_stop": True,
            }
        else:
            result["path_check"] = node.validate_memory_path(local_map, target, gap)
        if args.detect_only:
            result["approach"] = {"skipped": True, "reason": "--detect-only"}
        else:
            result["approach"] = node.approach_gap(reference, gap, target)
        result["status"] = "success"
        exit_code = 0
    except KeyboardInterrupt:
        result["status"] = "interrupted"
        result["error"] = "KeyboardInterrupt"
    except BaseException as exc:
        result["status"] = "failed"
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
