#!/usr/bin/env python3
"""Doorway detection in a live ROS OccupancyGrid, without ROS dependencies.

The public API deliberately accepts a small :class:`GridSnapshot` rather than a
``nav_msgs/OccupancyGrid``.  The ROS node can therefore copy the latest map under
its subscriber lock and run this module without holding that lock.

Coordinate convention
---------------------
``map_from_reference=(x, y, yaw)`` is the pose of the post-turn local reference
frame in the map frame.  In other words, a reference-frame point ``p`` is mapped
with ``R(yaw) * p + (x, y)``.  Returned geometry is always expressed in that
reference frame.  The detector looks for a wall roughly perpendicular to the
reference +X direction and a known-free opening between two supported wall
segments.  Unknown cells are never treated as free.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]
Pose2 = Tuple[float, float, float]


class DoorDetectionError(RuntimeError):
    """Base class for map-door detection failures."""


class NoDoorwayFound(DoorDetectionError):
    """Raised when no candidate passes the geometric and free-space tests."""


class AmbiguousDoorwayError(DoorDetectionError):
    """Raised when two spatially different candidates have comparable scores."""

    def __init__(self, message: str, candidates: Sequence["DoorCandidate"]):
        super().__init__(message)
        self.candidates = tuple(candidates)


@dataclass(frozen=True)
class GridSnapshot:
    """A dependency-free snapshot of a ROS occupancy grid.

    ``data`` uses ROS semantics: ``-1`` is unknown and values from 0 through 100
    are occupancy probabilities.  The grid origin is the pose of cell corner
    ``(0, 0)`` in the map frame, including its yaw.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("grid resolution must be finite and positive")
        if len(self.data) != self.width * self.height:
            raise ValueError(
                "grid data length {} does not equal width*height {}".format(
                    len(self.data), self.width * self.height
                )
            )
        for value in (self.origin_x, self.origin_y, self.origin_yaw):
            if not math.isfinite(value):
                raise ValueError("grid origin must contain only finite values")


@dataclass(frozen=True)
class DoorDetectorConfig:
    """Runtime-map doorway detector tuning in metres and reference coordinates."""

    occupied_threshold: int = 65
    free_threshold: int = 25

    roi_x_min: float = 0.30
    roi_x_max: float = 4.00
    roi_y_min: float = -2.60
    roi_y_max: float = 2.60

    min_width_m: float = 0.72
    max_width_m: float = 1.65
    min_wall_support_m: float = 0.34
    min_wall_density: float = 0.42

    max_wall_normal_angle_deg: float = 18.0
    wall_angle_step_deg: float = 3.0
    wall_bin_m: float = 0.055
    wall_band_m: float = 0.085
    merge_wall_holes_m: float = 0.13

    corridor_before_m: float = 0.38
    corridor_after_m: float = 0.58
    corridor_edge_margin_m: float = 0.11
    sample_spacing_m: float = 0.055
    min_corridor_known_ratio: float = 0.72
    min_corridor_free_ratio: float = 0.70
    max_corridor_unknown_ratio: float = 0.28
    max_corridor_occupied_ratio: float = 0.035
    min_edge_support_ratio: float = 0.50
    max_edge_unknown_ratio: float = 0.45

    min_score: float = 0.58
    ambiguity_score_delta: float = 0.065
    ambiguity_midpoint_distance_m: float = 0.28

    def __post_init__(self) -> None:
        if not (0 <= self.free_threshold < self.occupied_threshold <= 100):
            raise ValueError("thresholds must satisfy 0 <= free < occupied <= 100")
        if not (0.0 <= self.roi_x_min < self.roi_x_max):
            raise ValueError("invalid forward ROI")
        if not (self.roi_y_min < self.roi_y_max):
            raise ValueError("invalid lateral ROI")
        if not (0.0 < self.min_width_m < self.max_width_m):
            raise ValueError("invalid doorway width interval")
        if self.corridor_edge_margin_m * 2.0 >= self.min_width_m:
            raise ValueError("corridor edge margins leave no testable doorway interior")
        for value in (
            self.min_corridor_known_ratio,
            self.min_corridor_free_ratio,
            self.max_corridor_unknown_ratio,
            self.max_corridor_occupied_ratio,
            self.min_edge_support_ratio,
            self.max_edge_unknown_ratio,
            self.min_wall_density,
            self.min_score,
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("ratio and score settings must be in [0, 1]")


@dataclass(frozen=True)
class DoorCandidate:
    """A map-supported doorway expressed in the post-turn reference frame."""

    midpoint: Point
    right_edge: Point
    left_edge: Point
    normal: Point
    tangent: Point
    width: float
    score: float
    plane_distance: float
    right_wall_support_m: float
    left_wall_support_m: float
    right_edge_support_ratio: float
    left_edge_support_ratio: float
    corridor_free_ratio: float
    corridor_known_ratio: float
    corridor_unknown_ratio: float
    corridor_occupied_ratio: float


@dataclass(frozen=True)
class CandidateValidation:
    """Map evidence for a LiDAR (or other live-sensor) doorway candidate."""

    status: str  # "supported", "conflict", or "unknown"
    supported: bool
    right_edge_support_ratio: float
    left_edge_support_ratio: float
    right_edge_unknown_ratio: float
    left_edge_unknown_ratio: float
    corridor_free_ratio: float
    corridor_known_ratio: float
    corridor_unknown_ratio: float
    corridor_occupied_ratio: float
    reasons: Tuple[str, ...]


@dataclass(frozen=True)
class _Evidence:
    free_ratio: float
    known_ratio: float
    unknown_ratio: float
    occupied_ratio: float
    samples: int


@dataclass(frozen=True)
class _EdgeEvidence:
    support_ratio: float
    unknown_ratio: float
    samples: int


@dataclass(frozen=True)
class _CandidateGeometry:
    midpoint: Point
    right_edge: Point
    left_edge: Point
    normal: Point
    tangent: Point
    width: float


_FREE = 0
_OCCUPIED = 1
_UNKNOWN = -1


def _finite_pose(pose: Pose2) -> Pose2:
    if len(pose) != 3:
        raise ValueError("map_from_reference must be an (x, y, yaw) tuple")
    result = (float(pose[0]), float(pose[1]), float(pose[2]))
    if not all(math.isfinite(v) for v in result):
        raise ValueError("map_from_reference must contain only finite values")
    return result


def _normalise(vector: Point) -> Point:
    norm = math.hypot(vector[0], vector[1])
    if norm <= 1.0e-9 or not math.isfinite(norm):
        raise ValueError("zero or non-finite direction vector")
    return (vector[0] / norm, vector[1] / norm)


def _dot(point: Point, axis: Point) -> float:
    return point[0] * axis[0] + point[1] * axis[1]


def _reference_to_map(point: Point, map_from_reference: Pose2) -> Point:
    tx, ty, yaw = map_from_reference
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        tx + cosine * point[0] - sine * point[1],
        ty + sine * point[0] + cosine * point[1],
    )


def _map_to_reference(point: Point, map_from_reference: Pose2) -> Point:
    tx, ty, yaw = map_from_reference
    dx = point[0] - tx
    dy = point[1] - ty
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _grid_local_to_map(snapshot: GridSnapshot, gx: float, gy: float) -> Point:
    cosine = math.cos(snapshot.origin_yaw)
    sine = math.sin(snapshot.origin_yaw)
    return (
        snapshot.origin_x + cosine * gx - sine * gy,
        snapshot.origin_y + sine * gx + cosine * gy,
    )


def _map_to_grid_local(snapshot: GridSnapshot, point: Point) -> Point:
    dx = point[0] - snapshot.origin_x
    dy = point[1] - snapshot.origin_y
    cosine = math.cos(snapshot.origin_yaw)
    sine = math.sin(snapshot.origin_yaw)
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _classify(value: int, config: DoorDetectorConfig) -> int:
    if value < 0:
        return _UNKNOWN
    if value >= config.occupied_threshold:
        return _OCCUPIED
    if value <= config.free_threshold:
        return _FREE
    # Probability values in the undecided band are deliberately not free.
    return _UNKNOWN


def _state_at_reference(
    snapshot: GridSnapshot,
    point: Point,
    map_from_reference: Pose2,
    config: DoorDetectorConfig,
) -> int:
    map_point = _reference_to_map(point, map_from_reference)
    gx, gy = _map_to_grid_local(snapshot, map_point)
    col = int(math.floor(gx / snapshot.resolution))
    row = int(math.floor(gy / snapshot.resolution))
    if col < 0 or row < 0 or col >= snapshot.width or row >= snapshot.height:
        return _UNKNOWN
    return _classify(snapshot.data[row * snapshot.width + col], config)


def _roi_cells(
    snapshot: GridSnapshot,
    map_from_reference: Pose2,
    config: DoorDetectorConfig,
) -> Iterable[Tuple[float, float, int]]:
    """Yield reference-frame cell centres in the configured ROI.

    Only the grid-coordinate bounding box of the transformed ROI is visited, so
    a large global SLAM map does not have to be scanned in full.
    """

    corners = (
        (config.roi_x_min, config.roi_y_min),
        (config.roi_x_min, config.roi_y_max),
        (config.roi_x_max, config.roi_y_min),
        (config.roi_x_max, config.roi_y_max),
    )
    grid_corners = [
        _map_to_grid_local(snapshot, _reference_to_map(p, map_from_reference))
        for p in corners
    ]
    res = snapshot.resolution
    min_col = max(0, int(math.floor(min(p[0] for p in grid_corners) / res)) - 1)
    max_col = min(
        snapshot.width - 1,
        int(math.ceil(max(p[0] for p in grid_corners) / res)) + 1,
    )
    min_row = max(0, int(math.floor(min(p[1] for p in grid_corners) / res)) - 1)
    max_row = min(
        snapshot.height - 1,
        int(math.ceil(max(p[1] for p in grid_corners) / res)) + 1,
    )
    if min_col > max_col or min_row > max_row:
        return

    for row in range(min_row, max_row + 1):
        gy = (row + 0.5) * res
        base_index = row * snapshot.width
        for col in range(min_col, max_col + 1):
            gx = (col + 0.5) * res
            ref_x, ref_y = _map_to_reference(
                _grid_local_to_map(snapshot, gx, gy), map_from_reference
            )
            if not (
                config.roi_x_min <= ref_x <= config.roi_x_max
                and config.roi_y_min <= ref_y <= config.roi_y_max
            ):
                continue
            yield ref_x, ref_y, _classify(snapshot.data[base_index + col], config)


def _sample_values(start: float, stop: float, spacing: float) -> List[float]:
    if stop < start:
        start, stop = stop, start
    span = stop - start
    count = max(1, int(math.ceil(span / max(spacing, 1.0e-6))))
    if count == 1:
        return [(start + stop) * 0.5]
    return [start + span * i / count for i in range(count + 1)]


def _corridor_evidence(
    snapshot: GridSnapshot,
    geometry: _CandidateGeometry,
    map_from_reference: Pose2,
    config: DoorDetectorConfig,
) -> _Evidence:
    normal = geometry.normal
    tangent = geometry.tangent
    plane = _dot(geometry.midpoint, normal)
    right_t = _dot(geometry.right_edge, tangent)
    left_t = _dot(geometry.left_edge, tangent)
    if left_t < right_t:
        right_t, left_t = left_t, right_t

    inner_right = right_t + config.corridor_edge_margin_m
    inner_left = left_t - config.corridor_edge_margin_m
    if inner_left <= inner_right:
        return _Evidence(0.0, 0.0, 1.0, 0.0, 0)

    n_values = _sample_values(
        plane - config.corridor_before_m,
        plane + config.corridor_after_m,
        max(config.sample_spacing_m, snapshot.resolution),
    )
    t_values = _sample_values(
        inner_right,
        inner_left,
        max(config.sample_spacing_m, snapshot.resolution),
    )
    free = occupied = unknown = 0
    for n_value in n_values:
        for t_value in t_values:
            point = (
                normal[0] * n_value + tangent[0] * t_value,
                normal[1] * n_value + tangent[1] * t_value,
            )
            state = _state_at_reference(snapshot, point, map_from_reference, config)
            if state == _FREE:
                free += 1
            elif state == _OCCUPIED:
                occupied += 1
            else:
                unknown += 1
    total = free + occupied + unknown
    if total == 0:
        return _Evidence(0.0, 0.0, 1.0, 0.0, 0)
    return _Evidence(
        free_ratio=free / total,
        known_ratio=(free + occupied) / total,
        unknown_ratio=unknown / total,
        occupied_ratio=occupied / total,
        samples=total,
    )


def _edge_evidence(
    snapshot: GridSnapshot,
    edge: Point,
    normal: Point,
    tangent: Point,
    outward_sign: float,
    map_from_reference: Pose2,
    config: DoorDetectorConfig,
) -> _EdgeEvidence:
    plane = _dot(edge, normal)
    edge_t = _dot(edge, tangent)
    spacing = max(config.sample_spacing_m, snapshot.resolution)
    distances = _sample_values(spacing * 0.5, config.min_wall_support_m, spacing)
    n_offsets = (-config.wall_band_m, 0.0, config.wall_band_m)
    supported = unknown = 0
    for distance in distances:
        t_value = edge_t + outward_sign * distance
        states = []
        for offset in n_offsets:
            point = (
                normal[0] * (plane + offset) + tangent[0] * t_value,
                normal[1] * (plane + offset) + tangent[1] * t_value,
            )
            states.append(
                _state_at_reference(snapshot, point, map_from_reference, config)
            )
        if _OCCUPIED in states:
            supported += 1
        elif all(state == _UNKNOWN for state in states):
            unknown += 1
    total = len(distances)
    if total == 0:
        return _EdgeEvidence(0.0, 1.0, 0)
    return _EdgeEvidence(supported / total, unknown / total, total)


def _segment_density(values: Sequence[float], resolution: float) -> float:
    if not values:
        return 0.0
    start = min(values)
    stop = max(values)
    bins = {int(round(value / resolution)) for value in values}
    expected = max(1, int(round((stop - start) / resolution)) + 1)
    return min(1.0, len(bins) / expected)


def _cluster_projection(values: Sequence[float], merge_gap: float) -> List[List[float]]:
    if not values:
        return []
    ordered = sorted(values)
    clusters: List[List[float]] = [[ordered[0]]]
    for value in ordered[1:]:
        if value - clusters[-1][-1] <= merge_gap:
            clusters[-1].append(value)
        else:
            clusters.append([value])
    return clusters


def _candidate_is_duplicate(
    first: DoorCandidate, second: DoorCandidate, resolution: float
) -> bool:
    midpoint_distance = math.hypot(
        first.midpoint[0] - second.midpoint[0],
        first.midpoint[1] - second.midpoint[1],
    )
    normal_dot = max(-1.0, min(1.0, _dot(first.normal, second.normal)))
    angle = math.acos(normal_dot)
    return (
        midpoint_distance <= max(0.20, resolution * 4.0)
        and abs(first.width - second.width) <= max(0.18, resolution * 4.0)
        and angle <= math.radians(8.0)
    )


def _build_candidate(
    snapshot: GridSnapshot,
    map_from_reference: Pose2,
    config: DoorDetectorConfig,
    normal: Point,
    tangent: Point,
    plane: float,
    lower_cluster: Sequence[float],
    upper_cluster: Sequence[float],
    normal_angle: float,
) -> Optional[DoorCandidate]:
    resolution = snapshot.resolution
    right_t = max(lower_cluster) + resolution * 0.5
    left_t = min(upper_cluster) - resolution * 0.5
    width = left_t - right_t
    if not config.min_width_m <= width <= config.max_width_m:
        return None

    right_support_m = max(lower_cluster) - min(lower_cluster) + resolution
    left_support_m = max(upper_cluster) - min(upper_cluster) + resolution
    if (
        right_support_m < config.min_wall_support_m
        or left_support_m < config.min_wall_support_m
    ):
        return None
    right_density = _segment_density(lower_cluster, resolution)
    left_density = _segment_density(upper_cluster, resolution)
    density = 0.5 * (right_density + left_density)
    if density < config.min_wall_density:
        return None

    midpoint_t = 0.5 * (right_t + left_t)
    midpoint = (
        normal[0] * plane + tangent[0] * midpoint_t,
        normal[1] * plane + tangent[1] * midpoint_t,
    )
    if not (
        config.roi_x_min <= midpoint[0] <= config.roi_x_max
        and config.roi_y_min <= midpoint[1] <= config.roi_y_max
    ):
        return None
    right_edge = (
        normal[0] * plane + tangent[0] * right_t,
        normal[1] * plane + tangent[1] * right_t,
    )
    left_edge = (
        normal[0] * plane + tangent[0] * left_t,
        normal[1] * plane + tangent[1] * left_t,
    )
    geometry = _CandidateGeometry(
        midpoint, right_edge, left_edge, normal, tangent, width
    )
    corridor = _corridor_evidence(
        snapshot, geometry, map_from_reference, config
    )
    if (
        corridor.known_ratio < config.min_corridor_known_ratio
        or corridor.free_ratio < config.min_corridor_free_ratio
        or corridor.unknown_ratio > config.max_corridor_unknown_ratio
        or corridor.occupied_ratio > config.max_corridor_occupied_ratio
    ):
        return None

    right_edge_evidence = _edge_evidence(
        snapshot,
        right_edge,
        normal,
        tangent,
        -1.0,
        map_from_reference,
        config,
    )
    left_edge_evidence = _edge_evidence(
        snapshot,
        left_edge,
        normal,
        tangent,
        1.0,
        map_from_reference,
        config,
    )
    if (
        right_edge_evidence.support_ratio < config.min_edge_support_ratio
        or left_edge_evidence.support_ratio < config.min_edge_support_ratio
        or right_edge_evidence.unknown_ratio > config.max_edge_unknown_ratio
        or left_edge_evidence.unknown_ratio > config.max_edge_unknown_ratio
    ):
        return None

    support_quality = min(
        1.0,
        (right_support_m + left_support_m)
        / (4.0 * config.min_wall_support_m),
    )
    edge_quality = 0.5 * (
        right_edge_evidence.support_ratio + left_edge_evidence.support_ratio
    )
    orientation_quality = max(
        0.0,
        1.0
        - abs(normal_angle)
        / max(math.radians(config.max_wall_normal_angle_deg), 1.0e-6),
    )
    preferred_width = 0.5 * (config.min_width_m + config.max_width_m)
    half_width_range = 0.5 * (config.max_width_m - config.min_width_m)
    width_quality = max(0.0, 1.0 - abs(width - preferred_width) / half_width_range)
    score = (
        0.23 * corridor.free_ratio
        + 0.10 * corridor.known_ratio
        + 0.19 * support_quality
        + 0.14 * density
        + 0.17 * edge_quality
        + 0.10 * orientation_quality
        + 0.07 * width_quality
    )
    if score < config.min_score:
        return None
    return DoorCandidate(
        midpoint=midpoint,
        right_edge=right_edge,
        left_edge=left_edge,
        normal=normal,
        tangent=tangent,
        width=width,
        score=score,
        plane_distance=plane,
        right_wall_support_m=right_support_m,
        left_wall_support_m=left_support_m,
        right_edge_support_ratio=right_edge_evidence.support_ratio,
        left_edge_support_ratio=left_edge_evidence.support_ratio,
        corridor_free_ratio=corridor.free_ratio,
        corridor_known_ratio=corridor.known_ratio,
        corridor_unknown_ratio=corridor.unknown_ratio,
        corridor_occupied_ratio=corridor.occupied_ratio,
    )


def detect_doorway(
    snapshot: GridSnapshot,
    map_from_reference: Pose2,
    config: Optional[DoorDetectorConfig] = None,
) -> DoorCandidate:
    """Detect one unambiguous, map-supported doorway in front of the robot.

    The returned edges and midpoint are in the supplied post-turn reference
    frame.  A candidate must have occupied wall support on both sides and a
    known-free rectangular corridor through the opening.  If two distinct
    candidates score within ``ambiguity_score_delta``, this function raises
    :class:`AmbiguousDoorwayError` instead of guessing.
    """

    config = config or DoorDetectorConfig()
    map_from_reference = _finite_pose(map_from_reference)
    occupied_points = [
        (x, y)
        for x, y, state in _roi_cells(snapshot, map_from_reference, config)
        if state == _OCCUPIED
    ]
    if not occupied_points:
        raise NoDoorwayFound("no occupied wall cells in the forward map ROI")

    candidates: List[DoorCandidate] = []
    max_angle = math.radians(config.max_wall_normal_angle_deg)
    angle_step = math.radians(config.wall_angle_step_deg)
    angle_count = max(1, int(math.floor((2.0 * max_angle) / angle_step)))
    angles = [-max_angle + i * (2.0 * max_angle / angle_count) for i in range(angle_count + 1)]
    wall_bin = max(config.wall_bin_m, snapshot.resolution * 1.4)
    wall_band = max(config.wall_band_m, snapshot.resolution * 1.7)
    merge_gap = max(config.merge_wall_holes_m, snapshot.resolution * 2.4)

    for angle in angles:
        normal = (math.cos(angle), math.sin(angle))
        tangent = (-math.sin(angle), math.cos(angle))
        projected = [(_dot(point, normal), _dot(point, tangent)) for point in occupied_points]
        bins = {}
        for normal_value, tangent_value in projected:
            index = int(round(normal_value / wall_bin))
            bins.setdefault(index, []).append((normal_value, tangent_value))
        for index, bin_points in bins.items():
            if len(bin_points) < 6:
                continue
            # Ignore non-peak bins; neighbouring bins otherwise produce many
            # copies of the same physical wall.
            count = len(bin_points)
            if any(len(bins.get(index + offset, ())) > count for offset in (-1, 1)):
                continue
            plane_seed = sorted(value[0] for value in bin_points)[len(bin_points) // 2]
            line_points = [
                (normal_value, tangent_value)
                for normal_value, tangent_value in projected
                if abs(normal_value - plane_seed) <= wall_band
            ]
            if len(line_points) < 8:
                continue
            plane_values = sorted(value[0] for value in line_points)
            plane = plane_values[len(plane_values) // 2]
            clusters = _cluster_projection(
                [value[1] for value in line_points], merge_gap
            )
            if len(clusters) < 2:
                continue
            for lower, upper in zip(clusters, clusters[1:]):
                candidate = _build_candidate(
                    snapshot,
                    map_from_reference,
                    config,
                    normal,
                    tangent,
                    plane,
                    lower,
                    upper,
                    angle,
                )
                if candidate is None:
                    continue
                replaced = False
                for candidate_index, existing in enumerate(candidates):
                    if _candidate_is_duplicate(candidate, existing, snapshot.resolution):
                        if candidate.score > existing.score:
                            candidates[candidate_index] = candidate
                        replaced = True
                        break
                if not replaced:
                    candidates.append(candidate)

    if not candidates:
        raise NoDoorwayFound(
            "no doorway has two wall supports and a sufficiently known-free corridor"
        )
    candidates.sort(key=lambda item: item.score, reverse=True)
    best = candidates[0]
    ambiguous = [best]
    for candidate in candidates[1:]:
        if candidate.score < best.score - config.ambiguity_score_delta:
            continue
        distance = math.hypot(
            candidate.midpoint[0] - best.midpoint[0],
            candidate.midpoint[1] - best.midpoint[1],
        )
        if distance >= config.ambiguity_midpoint_distance_m:
            ambiguous.append(candidate)
    if len(ambiguous) > 1:
        raise AmbiguousDoorwayError(
            "{} similarly scored doorway candidates; refusing to guess".format(
                len(ambiguous)
            ),
            ambiguous,
        )
    return best


def _coerce_candidate(candidate: object) -> _CandidateGeometry:
    """Accept this module's candidate or the existing dual-LiDAR GapDetection."""

    try:
        midpoint = tuple(getattr(candidate, "midpoint"))
        right_edge = tuple(getattr(candidate, "right_edge"))
        left_edge = tuple(getattr(candidate, "left_edge"))
        normal = _normalise(tuple(getattr(candidate, "normal")))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(
            "candidate must expose midpoint, right_edge, left_edge and normal"
        ) from exc
    if len(midpoint) != 2 or len(right_edge) != 2 or len(left_edge) != 2:
        raise ValueError("candidate points must contain exactly two coordinates")
    midpoint = (float(midpoint[0]), float(midpoint[1]))
    right_edge = (float(right_edge[0]), float(right_edge[1]))
    left_edge = (float(left_edge[0]), float(left_edge[1]))
    if not all(math.isfinite(value) for point in (midpoint, right_edge, left_edge) for value in point):
        raise ValueError("candidate geometry must be finite")

    try:
        tangent = _normalise(tuple(getattr(candidate, "tangent")))
    except (AttributeError, TypeError, ValueError):
        tangent = (-normal[1], normal[0])
    # Force an orthonormal right-handed (normal, tangent) basis and orient the
    # wall normal toward the reference frame's forward half-plane.
    if normal[0] < 0.0:
        normal = (-normal[0], -normal[1])
    tangent = (-normal[1], normal[0])
    if _dot(right_edge, tangent) > _dot(left_edge, tangent):
        right_edge, left_edge = left_edge, right_edge
    width = _dot(left_edge, tangent) - _dot(right_edge, tangent)
    midpoint = (
        0.5 * (right_edge[0] + left_edge[0]),
        0.5 * (right_edge[1] + left_edge[1]),
    )
    return _CandidateGeometry(
        midpoint=midpoint,
        right_edge=right_edge,
        left_edge=left_edge,
        normal=normal,
        tangent=tangent,
        width=width,
    )


def validate_candidate(
    snapshot: GridSnapshot,
    candidate: object,
    map_from_reference: Pose2,
    config: Optional[DoorDetectorConfig] = None,
) -> CandidateValidation:
    """Compare a dual-LiDAR doorway candidate with the current runtime map.

    ``candidate`` is duck-typed so the existing ``GapDetection`` object can be
    passed directly.  It must expose ``midpoint``, ``right_edge``,
    ``left_edge`` and ``normal``; ``tangent`` is optional.  The result always
    distinguishes positive support, known contradiction, and insufficient map
    coverage.  Unknown cells never contribute to the free-space ratio.
    """

    config = config or DoorDetectorConfig()
    map_from_reference = _finite_pose(map_from_reference)
    geometry = _coerce_candidate(candidate)
    corridor = _corridor_evidence(
        snapshot, geometry, map_from_reference, config
    )
    right_edge = _edge_evidence(
        snapshot,
        geometry.right_edge,
        geometry.normal,
        geometry.tangent,
        -1.0,
        map_from_reference,
        config,
    )
    left_edge = _edge_evidence(
        snapshot,
        geometry.left_edge,
        geometry.normal,
        geometry.tangent,
        1.0,
        map_from_reference,
        config,
    )

    conflicts: List[str] = []
    unknowns: List[str] = []
    if not config.min_width_m <= geometry.width <= config.max_width_m:
        conflicts.append("candidate width is outside the configured doorway range")
    if corridor.occupied_ratio > config.max_corridor_occupied_ratio:
        conflicts.append("runtime map contains occupied cells inside the doorway corridor")
    if corridor.known_ratio < config.min_corridor_known_ratio:
        unknowns.append("runtime map coverage through the doorway is insufficient")
    if corridor.unknown_ratio > config.max_corridor_unknown_ratio:
        unknowns.append("too much of the doorway corridor is unknown")
    if corridor.free_ratio < config.min_corridor_free_ratio and not unknowns:
        conflicts.append("known map cells do not form a free corridor")

    for label, evidence in (("right", right_edge), ("left", left_edge)):
        if evidence.unknown_ratio > config.max_edge_unknown_ratio:
            unknowns.append("{} door edge is mostly outside known map".format(label))
        elif evidence.support_ratio < config.min_edge_support_ratio:
            conflicts.append("{} door edge lacks occupied wall support".format(label))

    # A directly observed obstacle is a conflict even when other parts of the
    # corridor remain unknown.  Otherwise lack of map coverage takes precedence
    # over absence of wall evidence.
    if conflicts and corridor.occupied_ratio > config.max_corridor_occupied_ratio:
        status = "conflict"
        reasons = tuple(conflicts + unknowns)
    elif unknowns:
        status = "unknown"
        reasons = tuple(unknowns + conflicts)
    elif conflicts:
        status = "conflict"
        reasons = tuple(conflicts)
    else:
        status = "supported"
        reasons = ("runtime map supports both door edges and the free corridor",)
    return CandidateValidation(
        status=status,
        supported=status == "supported",
        right_edge_support_ratio=right_edge.support_ratio,
        left_edge_support_ratio=left_edge.support_ratio,
        right_edge_unknown_ratio=right_edge.unknown_ratio,
        left_edge_unknown_ratio=left_edge.unknown_ratio,
        corridor_free_ratio=corridor.free_ratio,
        corridor_known_ratio=corridor.known_ratio,
        corridor_unknown_ratio=corridor.unknown_ratio,
        corridor_occupied_ratio=corridor.occupied_ratio,
        reasons=reasons,
    )


__all__ = [
    "AmbiguousDoorwayError",
    "CandidateValidation",
    "DoorCandidate",
    "DoorDetectionError",
    "DoorDetectorConfig",
    "GridSnapshot",
    "NoDoorwayFound",
    "detect_doorway",
    "validate_candidate",
]

