#!/usr/bin/env python3
"""Pure geometry used by the continuous TMR base route controller.

This module deliberately has no ROS dependency.  It contains only the pieces
that must be deterministic and easy to test offline:

* braking-distance-aware swept-corridor collision checks;
* consecutive-frame stabilization of a detected doorway/gap; and
* pre-door and post-door target construction for a rectangular footprint.

Coordinates are right handed in metres.  A gap normal points from the approach
side through the doorway.  ``robot_heading`` points along the robot's positive
body-x axis in the same coordinate frame.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Hashable, Iterable, Sequence


Point2 = tuple[float, float]

_EPS = 1.0e-9


def _is_finite_point(point: Sequence[float]) -> bool:
    return len(point) == 2 and math.isfinite(point[0]) and math.isfinite(point[1])


def _unit(vector: Sequence[float], *, label: str) -> Point2:
    if not _is_finite_point(vector):
        raise ValueError(f"{label} must be a finite 2-D vector")
    length = math.hypot(vector[0], vector[1])
    if length <= _EPS:
        raise ValueError(f"{label} must be non-zero")
    return vector[0] / length, vector[1] / length


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


@dataclass(frozen=True)
class Footprint:
    """Rectangular base footprint about ``base_link``.

    The defaults are the measured TMR base extents used by the project:
    0.40 m forward, 0.40 m rearward and 0.58 m total width.
    """

    front_m: float = 0.40
    rear_m: float = 0.40
    width_m: float = 0.58

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.front_m, self.rear_m, self.width_m)
        ):
            raise ValueError("footprint dimensions must be finite and positive")

    @property
    def half_width_m(self) -> float:
        return self.width_m / 2.0

    def support(self, direction: Sequence[float], heading: Sequence[float] = (1.0, 0.0)) -> float:
        """Return the furthest footprint projection along ``direction``.

        ``direction`` and ``heading`` need not already be normalized.  This is
        the support function of the asymmetric body rectangle, so it remains
        correct for strafing, diagonal travel and a doorway that is slightly
        skewed relative to the held robot heading.
        """

        axis = _unit(direction, label="direction")
        forward = _unit(heading, label="heading")
        left = (-forward[1], forward[0])
        longitudinal = _dot(axis, forward)
        longitudinal_support = (
            self.front_m * longitudinal
            if longitudinal >= 0.0
            else self.rear_m * -longitudinal
        )
        return longitudinal_support + self.half_width_m * abs(_dot(axis, left))

    def projected_width(
        self, axis: Sequence[float], heading: Sequence[float] = (1.0, 0.0)
    ) -> float:
        """Return the full footprint span along an arbitrary axis."""

        unit_axis = _unit(axis, label="axis")
        return self.support(unit_axis, heading) + self.support(
            (-unit_axis[0], -unit_axis[1]), heading
        )


TMR_FOOTPRINT = Footprint()


@dataclass(frozen=True)
class BrakingModel:
    """Parameters for the commanded-motion stopping horizon."""

    deceleration_mps2: float = 0.35
    reaction_time_s: float = 0.20
    margin_m: float = 0.08

    def __post_init__(self) -> None:
        if not math.isfinite(self.deceleration_mps2) or self.deceleration_mps2 <= 0.0:
            raise ValueError("deceleration_mps2 must be finite and positive")
        if not math.isfinite(self.reaction_time_s) or self.reaction_time_s < 0.0:
            raise ValueError("reaction_time_s must be finite and non-negative")
        if not math.isfinite(self.margin_m) or self.margin_m < 0.0:
            raise ValueError("margin_m must be finite and non-negative")

    def required_clearance(self, speed_mps: float) -> float:
        """Clearance beyond the leading footprint edge needed to stop."""

        if not math.isfinite(speed_mps) or speed_mps < 0.0:
            raise ValueError("speed_mps must be finite and non-negative")
        return (
            speed_mps * speed_mps / (2.0 * self.deceleration_mps2)
            + self.reaction_time_s * speed_mps
            + self.margin_m
        )


@dataclass(frozen=True)
class SafetyConfig:
    """Compact configuration accepted by the route controller safety call."""

    footprint: Footprint = TMR_FOOTPRINT
    braking: BrakingModel = BrakingModel()
    corridor_margin_m: float = 0.04

    def __post_init__(self) -> None:
        if not math.isfinite(self.corridor_margin_m) or self.corridor_margin_m < 0.0:
            raise ValueError("corridor_margin_m must be finite and non-negative")


@dataclass(frozen=True)
class SweptCorridorResult:
    """Result of checking scan hits against the commanded swept corridor."""

    should_stop: bool
    speed_mps: float
    direction: Point2
    footprint_leading_extent_m: float
    corridor_half_width_m: float
    required_clearance_m: float
    lookahead_from_base_m: float
    nearest_obstacle_along_m: float
    nearest_clearance_m: float
    hit_count: int
    nearest_point: Point2 | None

    @property
    def blocked(self) -> bool:
        """Controller-friendly alias for ``should_stop``."""

        return self.should_stop

    @property
    def scale(self) -> float:
        """Safe command scale; this hard safety layer either passes or stops."""

        return 0.0 if self.should_stop else 1.0

    @property
    def nearest_m(self) -> float:
        """Nearest clearance measured beyond the leading footprint edge."""

        return self.nearest_clearance_m


def evaluate_swept_corridor(
    obstacle_points: Iterable[Sequence[float]],
    velocity_xy_mps: Sequence[float] | float,
    vy_mps: float | None = None,
    config: SafetyConfig | None = None,
    *,
    footprint: Footprint | None = None,
    braking: BrakingModel | None = None,
    robot_heading: Sequence[float] = (1.0, 0.0),
    corridor_margin_m: float | None = None,
) -> SweptCorridorResult:
    """Check whether a commanded translation must stop for an obstacle.

    Scan hits are expected in the same frame as ``velocity_xy_mps`` and
    ``robot_heading``.  A hit is relevant when it lies inside the rectangular
    footprint swept along the velocity direction up to the braking horizon.
    Obstacles behind the leading footprint edge are ignored.  This is
    intentional: a planar scanner mounted on the robot cannot distinguish a
    real overlap from chassis/cable/arm returns in the volume already occupied
    at the start of the command.  Collision evidence is therefore restricted
    to the *newly swept* strip from the leading edge through the braking
    horizon.  A point immediately beyond that edge is still a one-beam stop.
    ``velocity_xy_mps`` may be a ``(vx, vy)`` pair.  For a controller hot path,
    the equivalent compact call ``evaluate_swept_corridor(points, vx, vy, cfg)``
    is also accepted.
    """

    if isinstance(velocity_xy_mps, (int, float)):
        if vy_mps is None:
            raise ValueError("vy_mps is required when vx is passed separately")
        velocity = float(velocity_xy_mps), float(vy_mps)
    else:
        if vy_mps is not None:
            raise ValueError("do not pass vy_mps with a (vx, vy) velocity pair")
        velocity = velocity_xy_mps
    if not _is_finite_point(velocity):
        raise ValueError("velocity_xy_mps must be a finite 2-D vector")

    safety = config if config is not None else SafetyConfig()
    resolved_footprint = footprint if footprint is not None else safety.footprint
    resolved_braking = braking if braking is not None else safety.braking
    resolved_corridor_margin = (
        corridor_margin_m
        if corridor_margin_m is not None
        else safety.corridor_margin_m
    )
    if not math.isfinite(resolved_corridor_margin) or resolved_corridor_margin < 0.0:
        raise ValueError("corridor_margin_m must be finite and non-negative")

    speed = math.hypot(velocity[0], velocity[1])
    if speed <= _EPS:
        return SweptCorridorResult(
            should_stop=False,
            speed_mps=0.0,
            direction=(0.0, 0.0),
            footprint_leading_extent_m=0.0,
            corridor_half_width_m=0.0,
            required_clearance_m=0.0,
            lookahead_from_base_m=0.0,
            nearest_obstacle_along_m=math.inf,
            nearest_clearance_m=math.inf,
            hit_count=0,
            nearest_point=None,
        )

    direction = velocity[0] / speed, velocity[1] / speed
    normal = (-direction[1], direction[0])
    heading = _unit(robot_heading, label="robot_heading")
    leading_extent = resolved_footprint.support(direction, heading)
    corridor_half_width = max(
        resolved_footprint.support(normal, heading),
        resolved_footprint.support((-normal[0], -normal[1]), heading),
    ) + resolved_corridor_margin
    required_clearance = resolved_braking.required_clearance(speed)
    lookahead = leading_extent + required_clearance

    nearest_along = math.inf
    nearest_clearance = math.inf
    nearest_point: Point2 | None = None
    hit_count = 0

    for point in obstacle_points:
        if not _is_finite_point(point):
            continue
        candidate = float(point[0]), float(point[1])
        along = _dot(candidate, direction)
        across = abs(_dot(candidate, normal))
        # Test the Minkowski sweep minus its initial footprint.  Including the
        # initial rectangle caused the deployed front/rear scanners to report
        # carried-robot returns as collisions (for example clearance=-0.735 m
        # during a forward command).  This directional leading-plane test does
        # not hide a true obstacle in front of, behind, or beside the robot:
        # the leading plane rotates with the requested travel direction.
        if along < leading_extent - _EPS or along > lookahead + _EPS:
            continue
        if across > corridor_half_width + _EPS:
            continue
        hit_count += 1
        clearance = along - leading_extent
        if clearance < nearest_clearance:
            nearest_clearance = clearance
            nearest_along = along
            nearest_point = candidate

    return SweptCorridorResult(
        should_stop=hit_count > 0,
        speed_mps=speed,
        direction=direction,
        footprint_leading_extent_m=leading_extent,
        corridor_half_width_m=corridor_half_width,
        required_clearance_m=required_clearance,
        lookahead_from_base_m=lookahead,
        nearest_obstacle_along_m=nearest_along,
        nearest_clearance_m=nearest_clearance,
        hit_count=hit_count,
        nearest_point=nearest_point,
    )


@dataclass(frozen=True)
class GapObservation:
    """One independently detected doorway/gap candidate."""

    frame_id: Hashable
    midpoint: Point2
    width_m: float
    normal: Point2
    stamp_s: float | None = None


@dataclass(frozen=True)
class StableGap:
    """A doorway accepted only after multiple mutually consistent frames."""

    midpoint: Point2
    width_m: float
    normal: Point2
    sample_count: int
    first_frame_id: Hashable
    last_frame_id: Hashable
    first_stamp_s: float | None
    last_stamp_s: float | None


class TemporalGapStabilizer:
    """Require a run of consistent doorway detections before accepting one.

    A missing, invalid or inconsistent observation breaks the current run.
    Re-reading the same ``frame_id`` never increases the consecutive count.
    Normals are oriented toward ``expected_normal`` before comparison, which
    removes the harmless sign ambiguity of a fitted wall line while preserving
    the travel-side convention needed to construct crossing targets.
    """

    def __init__(
        self,
        *,
        required_consecutive: int = 4,
        minimum_width_m: float = TMR_FOOTPRINT.width_m + 0.16,
        maximum_midpoint_delta_m: float = 0.08,
        maximum_width_delta_m: float = 0.10,
        maximum_normal_angle_deg: float = 7.0,
        maximum_interframe_s: float | None = 0.50,
        expected_normal: Sequence[float] = (1.0, 0.0),
    ) -> None:
        if required_consecutive < 2:
            raise ValueError("required_consecutive must be at least two")
        finite_nonnegative = (
            minimum_width_m,
            maximum_midpoint_delta_m,
            maximum_width_delta_m,
            maximum_normal_angle_deg,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in finite_nonnegative):
            raise ValueError("stabilizer thresholds must be finite and non-negative")
        if maximum_interframe_s is not None and (
            not math.isfinite(maximum_interframe_s) or maximum_interframe_s <= 0.0
        ):
            raise ValueError("maximum_interframe_s must be positive or None")

        self.required_consecutive = required_consecutive
        self.minimum_width_m = minimum_width_m
        self.maximum_midpoint_delta_m = maximum_midpoint_delta_m
        self.maximum_width_delta_m = maximum_width_delta_m
        self.maximum_normal_angle_rad = math.radians(maximum_normal_angle_deg)
        self.maximum_interframe_s = maximum_interframe_s
        self.expected_normal = _unit(expected_normal, label="expected_normal")
        self._samples: deque[GapObservation] = deque(maxlen=required_consecutive)
        self._last_seen_frame_id: Hashable | None = None
        self._last_seen_stamp_s: float | None = None

    @property
    def consecutive_count(self) -> int:
        return len(self._samples)

    @property
    def is_confirmed(self) -> bool:
        return len(self._samples) >= self.required_consecutive

    def reset(self) -> None:
        self._samples.clear()
        self._last_seen_frame_id = None
        self._last_seen_stamp_s = None

    def _prepare(self, observation: GapObservation) -> GapObservation | None:
        if not _is_finite_point(observation.midpoint):
            return None
        if not math.isfinite(observation.width_m) or observation.width_m < self.minimum_width_m:
            return None
        if observation.stamp_s is not None and not math.isfinite(observation.stamp_s):
            return None
        try:
            normal = _unit(observation.normal, label="gap normal")
        except ValueError:
            return None
        if _dot(normal, self.expected_normal) < 0.0:
            normal = -normal[0], -normal[1]
        return GapObservation(
            frame_id=observation.frame_id,
            midpoint=(float(observation.midpoint[0]), float(observation.midpoint[1])),
            width_m=float(observation.width_m),
            normal=normal,
            stamp_s=observation.stamp_s,
        )

    @staticmethod
    def _average(samples: Sequence[GapObservation]) -> StableGap:
        count = len(samples)
        midpoint = (
            sum(sample.midpoint[0] for sample in samples) / count,
            sum(sample.midpoint[1] for sample in samples) / count,
        )
        width = sum(sample.width_m for sample in samples) / count
        normal = _unit(
            (
                sum(sample.normal[0] for sample in samples),
                sum(sample.normal[1] for sample in samples),
            ),
            label="averaged gap normal",
        )
        first = samples[0]
        last = samples[-1]
        return StableGap(
            midpoint=midpoint,
            width_m=width,
            normal=normal,
            sample_count=count,
            first_frame_id=first.frame_id,
            last_frame_id=last.frame_id,
            first_stamp_s=first.stamp_s,
            last_stamp_s=last.stamp_s,
        )

    def _consistent_with_run(self, observation: GapObservation) -> bool:
        if not self._samples:
            return True
        current = self._average(tuple(self._samples))
        midpoint_delta = math.hypot(
            observation.midpoint[0] - current.midpoint[0],
            observation.midpoint[1] - current.midpoint[1],
        )
        width_delta = abs(observation.width_m - current.width_m)
        cosine = max(-1.0, min(1.0, _dot(observation.normal, current.normal)))
        normal_delta = math.acos(cosine)
        return (
            midpoint_delta <= self.maximum_midpoint_delta_m + _EPS
            and width_delta <= self.maximum_width_delta_m + _EPS
            and normal_delta <= self.maximum_normal_angle_rad + _EPS
        )

    def update(self, observation: GapObservation | None) -> StableGap | None:
        """Add one frame and return a stable gap only after confirmation."""

        if observation is None:
            self.reset()
            return None

        prepared = self._prepare(observation)
        if prepared is None:
            self.reset()
            return None

        # Polling the same scan/map callback repeatedly is not temporal evidence.
        if prepared.frame_id == self._last_seen_frame_id:
            return self._average(tuple(self._samples)) if self.is_confirmed else None

        if (
            self.maximum_interframe_s is not None
            and prepared.stamp_s is not None
            and self._last_seen_stamp_s is not None
            and (
                prepared.stamp_s <= self._last_seen_stamp_s
                or prepared.stamp_s - self._last_seen_stamp_s > self.maximum_interframe_s
            )
        ):
            self._samples.clear()

        self._last_seen_frame_id = prepared.frame_id
        self._last_seen_stamp_s = prepared.stamp_s

        if not self._consistent_with_run(prepared):
            self._samples.clear()
        self._samples.append(prepared)

        if not self.is_confirmed:
            return None
        return self._average(tuple(self._samples))


# Door terminology aliases used by the continuous controller.  Keeping the gap
# names as well makes this module reusable by the existing mapping detector.
DoorObservation = GapObservation
StableDoorTracker = TemporalGapStabilizer


@dataclass(frozen=True)
class GapTargets:
    """Centred poses immediately before and safely beyond a doorway."""

    pre_door: Point2
    post_door: Point2
    normal: Point2
    tangent: Point2
    traversal_distance_m: float
    required_opening_width_m: float
    available_side_clearance_m: float


def compute_gap_targets(
    gap: StableGap | GapObservation,
    *,
    footprint: Footprint = TMR_FOOTPRINT,
    robot_heading: Sequence[float] = (1.0, 0.0),
    pre_door_clearance_m: float = 0.20,
    post_door_clearance_m: float = 0.25,
    side_clearance_m: float = 0.08,
) -> GapTargets:
    """Compute centred staging and clear-through base positions for a gap.

    The pre-door target leaves ``pre_door_clearance_m`` between the foremost
    footprint point and the wall plane.  The post-door target places the entire
    rear edge ``post_door_clearance_m`` beyond that plane.  Door width is checked
    against the *projected* rectangular footprint, not just nominal base width.
    """

    clearances = (pre_door_clearance_m, post_door_clearance_m, side_clearance_m)
    if not all(math.isfinite(value) and value >= 0.0 for value in clearances):
        raise ValueError("gap target clearances must be finite and non-negative")
    if not _is_finite_point(gap.midpoint):
        raise ValueError("gap midpoint must be a finite 2-D point")
    if not math.isfinite(gap.width_m) or gap.width_m <= 0.0:
        raise ValueError("gap width must be finite and positive")

    normal = _unit(gap.normal, label="gap normal")
    heading = _unit(robot_heading, label="robot_heading")
    tangent = (-normal[1], normal[0])
    opening_footprint_width = footprint.projected_width(tangent, heading)
    required_width = opening_footprint_width + 2.0 * side_clearance_m
    if gap.width_m + _EPS < required_width:
        raise ValueError(
            f"gap width {gap.width_m:.3f} m is below required {required_width:.3f} m"
        )

    leading_support = footprint.support(normal, heading)
    trailing_support = footprint.support((-normal[0], -normal[1]), heading)
    pre_offset = leading_support + pre_door_clearance_m
    post_offset = trailing_support + post_door_clearance_m
    pre_door = (
        gap.midpoint[0] - normal[0] * pre_offset,
        gap.midpoint[1] - normal[1] * pre_offset,
    )
    post_door = (
        gap.midpoint[0] + normal[0] * post_offset,
        gap.midpoint[1] + normal[1] * post_offset,
    )
    return GapTargets(
        pre_door=pre_door,
        post_door=post_door,
        normal=normal,
        tangent=tangent,
        traversal_distance_m=pre_offset + post_offset,
        required_opening_width_m=required_width,
        available_side_clearance_m=(gap.width_m - opening_footprint_width) / 2.0,
    )


compute_door_targets = compute_gap_targets


__all__ = [
    "BrakingModel",
    "DoorObservation",
    "Footprint",
    "GapObservation",
    "GapTargets",
    "SafetyConfig",
    "StableGap",
    "SweptCorridorResult",
    "TMR_FOOTPRINT",
    "TemporalGapStabilizer",
    "StableDoorTracker",
    "compute_door_targets",
    "compute_gap_targets",
    "evaluate_swept_corridor",
]
