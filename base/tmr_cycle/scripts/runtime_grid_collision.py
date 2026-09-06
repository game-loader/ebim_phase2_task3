#!/usr/bin/env python3
"""Pure-Python collision evidence from a live ROS OccupancyGrid snapshot.

The module never reads a rendered map image.  ``map_from_base`` is the current
pose of ``base_link`` in the OccupancyGrid frame (the result of a TF lookup
whose target is the map frame and whose source is ``base_link``).  The robot's
rectangular carried envelope is swept along the requested body-frame
translation through reaction and braking distance.

Only *known occupied* cells can block motion.  Unknown/out-of-map/undecided
cells are reported, but neither treated as obstacles nor allowed to make the
map claim that a path is confirmed clear.  A live LiDAR guard must therefore
remain active whenever this module is used on a robot.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

from runtime_map_door import GridSnapshot


Point = Tuple[float, float]
Pose2 = Tuple[float, float, float]


@dataclass(frozen=True)
class RectangularFootprint:
    front_m: float
    rear_m: float
    width_m: float

    def __post_init__(self) -> None:
        values = (self.front_m, self.rear_m, self.width_m)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("footprint dimensions must be finite and positive")


@dataclass(frozen=True)
class GridSweepConfig:
    brake_accel_mps2: float
    reaction_time_s: float
    hard_margin_m: float
    occupied_threshold: int = 65
    free_threshold: int = 25
    minimum_occupied_cluster_cells: int = 2
    maximum_test_cells: int = 20_000

    def __post_init__(self) -> None:
        if not math.isfinite(self.brake_accel_mps2) or self.brake_accel_mps2 <= 0.0:
            raise ValueError("brake_accel_mps2 must be finite and positive")
        if not math.isfinite(self.reaction_time_s) or self.reaction_time_s < 0.0:
            raise ValueError("reaction_time_s must be finite and non-negative")
        if not math.isfinite(self.hard_margin_m) or self.hard_margin_m < 0.0:
            raise ValueError("hard_margin_m must be finite and non-negative")
        if not 0 <= self.free_threshold < self.occupied_threshold <= 100:
            raise ValueError("thresholds must satisfy 0 <= free < occupied <= 100")
        if self.minimum_occupied_cluster_cells <= 0:
            raise ValueError("minimum_occupied_cluster_cells must be positive")
        if self.maximum_test_cells <= 0:
            raise ValueError("maximum_test_cells must be positive")


@dataclass(frozen=True)
class GridSweepResult:
    blocked: bool
    reason: str
    speed_mps: float
    swept_distance_m: float
    collision_progress_m: float | None
    occupied_cells: int
    largest_occupied_cluster_cells: int
    unknown_cells: int
    tested_cells: int
    map_can_confirm_clear: bool


def _finite_pose(pose: Pose2) -> Pose2:
    if len(pose) != 3:
        raise ValueError("map_from_base must be an (x, y, yaw) tuple")
    result = (float(pose[0]), float(pose[1]), float(pose[2]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError("map_from_base must contain only finite values")
    return result


def _transform(point: Point, pose: Pose2) -> Point:
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        x + cosine * point[0] - sine * point[1],
        y + sine * point[0] + cosine * point[1],
    )


def _inverse_transform(point: Point, pose: Pose2) -> Point:
    x, y, yaw = pose
    dx = point[0] - x
    dy = point[1] - y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _cross(origin: Point, first: Point, second: Point) -> float:
    return (first[0] - origin[0]) * (second[1] - origin[1]) - (
        first[1] - origin[1]
    ) * (second[0] - origin[0])


def _convex_hull(points: Iterable[Point]) -> list[Point]:
    ordered = sorted(set(points))
    if len(ordered) <= 2:
        return ordered
    lower: list[Point] = []
    for point in ordered:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[Point] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _overlap(first: Sequence[float], second: Sequence[float]) -> bool:
    epsilon = 1.0e-10
    return not (max(first) < min(second) - epsilon or max(second) < min(first) - epsilon)


def _polygon_intersects_cell(
    polygon: Sequence[Point], col: int, row: int, resolution: float
) -> bool:
    """SAT intersection against a grid-local, axis-aligned closed cell."""

    x0 = col * resolution
    x1 = x0 + resolution
    y0 = row * resolution
    y1 = y0 + resolution
    cell = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    axes: list[Point] = [(1.0, 0.0), (0.0, 1.0)]
    for index, point in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]
        dx = following[0] - point[0]
        dy = following[1] - point[1]
        norm = math.hypot(dx, dy)
        if norm > 1.0e-12:
            axes.append((-dy / norm, dx / norm))
    for axis in axes:
        polygon_projection = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
        cell_projection = [point[0] * axis[0] + point[1] * axis[1] for point in cell]
        if not _overlap(polygon_projection, cell_projection):
            return False
    return True


def _grid_local(snapshot: GridSnapshot, map_point: Point) -> Point:
    return _inverse_transform(
        map_point,
        (snapshot.origin_x, snapshot.origin_y, snapshot.origin_yaw),
    )


def _cell_centre_in_map(snapshot: GridSnapshot, col: int, row: int) -> Point:
    local = ((col + 0.5) * snapshot.resolution, (row + 0.5) * snapshot.resolution)
    return _transform(
        local,
        (snapshot.origin_x, snapshot.origin_y, snapshot.origin_yaw),
    )


def evaluate_occupancy_grid_sweep(
    snapshot: GridSnapshot,
    map_from_base: Pose2,
    velocity_base: Point,
    footprint: RectangularFootprint,
    config: GridSweepConfig,
) -> GridSweepResult:
    """Check a braking-distance rectangular translation against known occupancy.

    ``velocity_base`` supports forward, reverse, lateral, and diagonal motion.
    Rotation is intentionally outside this API: the caller should use a
    circumscribed-radius check for in-place rotation.
    """

    map_from_base = _finite_pose(map_from_base)
    vx, vy = float(velocity_base[0]), float(velocity_base[1])
    if not math.isfinite(vx) or not math.isfinite(vy):
        raise ValueError("velocity_base must contain only finite values")
    speed = math.hypot(vx, vy)
    if speed < 1.0e-6:
        return GridSweepResult(
            blocked=False,
            reason="stationary",
            speed_mps=speed,
            swept_distance_m=0.0,
            collision_progress_m=None,
            occupied_cells=0,
            largest_occupied_cluster_cells=0,
            unknown_cells=0,
            tested_cells=0,
            map_can_confirm_clear=False,
        )

    direction = (vx / speed, vy / speed)
    brake_distance = speed * config.reaction_time_s + speed * speed / (
        2.0 * config.brake_accel_mps2
    )
    swept_distance = brake_distance + config.hard_margin_m
    displacement = (direction[0] * swept_distance, direction[1] * swept_distance)
    half_width = footprint.width_m * 0.5
    corners = (
        (footprint.front_m, half_width),
        (footprint.front_m, -half_width),
        (-footprint.rear_m, half_width),
        (-footprint.rear_m, -half_width),
    )
    swept_base = _convex_hull(
        list(corners)
        + [(point[0] + displacement[0], point[1] + displacement[1]) for point in corners]
    )
    swept_grid = [
        _grid_local(snapshot, _transform(point, map_from_base)) for point in swept_base
    ]
    resolution = snapshot.resolution
    minimum_col = int(math.floor(min(point[0] for point in swept_grid) / resolution))
    maximum_col = int(math.floor(max(point[0] for point in swept_grid) / resolution))
    minimum_row = int(math.floor(min(point[1] for point in swept_grid) / resolution))
    maximum_row = int(math.floor(max(point[1] for point in swept_grid) / resolution))
    candidate_count = (maximum_col - minimum_col + 1) * (maximum_row - minimum_row + 1)
    if candidate_count > config.maximum_test_cells:
        raise ValueError(
            f"swept footprint spans {candidate_count} cells; maximum is {config.maximum_test_cells}"
        )

    # Only the volume newly occupied by the translation is collision
    # evidence.  The live SLAM map can contain inflated/ghost cells under the
    # robot itself; treating the whole initial footprint as a future collision
    # made a valid INITIAL_FORWARD abort before it moved.  Cell-centre progress
    # is used here so a cell immediately beyond the directional leading edge
    # remains blocking even when its closed boundary touches the start pose.
    directional_support = (
        (footprint.front_m if direction[0] >= 0.0 else footprint.rear_m)
        * abs(direction[0])
        + half_width * abs(direction[1])
    )
    occupied = unknown = tested = 0
    occupied_progress: dict[tuple[int, int], float] = {}
    for row in range(minimum_row, maximum_row + 1):
        for col in range(minimum_col, maximum_col + 1):
            if not _polygon_intersects_cell(swept_grid, col, row, resolution):
                continue
            centre_map = _cell_centre_in_map(snapshot, col, row)
            centre_base = _inverse_transform(centre_map, map_from_base)
            centre_along = (
                centre_base[0] * direction[0] + centre_base[1] * direction[1]
            )
            if centre_along < directional_support - 1.0e-10:
                continue
            tested += 1
            if col < 0 or row < 0 or col >= snapshot.width or row >= snapshot.height:
                unknown += 1
                continue
            value = int(snapshot.data[row * snapshot.width + col])
            if value >= config.occupied_threshold:
                occupied += 1
                occupied_progress[(col, row)] = max(
                    0.0, centre_along - directional_support
                )
            elif value < 0 or value > config.free_threshold:
                # Unknown (-1) and the undecided probability band cannot
                # authorize a clear path, but do not trigger a stop by themselves.
                unknown += 1

    # Rendered SLAM maps commonly contain isolated occupied speckles.  They are
    # useful diagnostics but are not enough to stop the robot by themselves;
    # the raw LiDAR layer still retains its one-beam hard stop.  Require a
    # small 8-connected map cluster for this secondary, position-based layer.
    remaining = set(occupied_progress)
    components: list[list[tuple[int, int]]] = []
    while remaining:
        seed = remaining.pop()
        component = [seed]
        stack = [seed]
        while stack:
            col, row = stack.pop()
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    neighbour = (col + dc, row + dr)
                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        component.append(neighbour)
                        stack.append(neighbour)
        components.append(component)
    largest_component = max(components, key=len, default=[])
    blocked = len(largest_component) >= config.minimum_occupied_cluster_cells
    collision_progress = [occupied_progress[cell] for cell in largest_component]
    return GridSweepResult(
        blocked=blocked,
        reason=(
            "supported known-occupied cluster intersects braking sweep"
            if blocked
            else "no supported known-occupied cluster in braking sweep"
        ),
        speed_mps=speed,
        swept_distance_m=swept_distance,
        collision_progress_m=min(collision_progress) if collision_progress else None,
        occupied_cells=occupied,
        largest_occupied_cluster_cells=len(largest_component),
        unknown_cells=unknown,
        tested_cells=tested,
        map_can_confirm_clear=(occupied == 0 and tested > 0 and unknown == 0),
    )
