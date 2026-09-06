#!/usr/bin/env python3
"""Continuous start-to-pickup base mission with live-map doorway detection.

The competition route is one persistent ROS 2 process:

    forward -> clockwise 90 deg -> detect door -> align to its perpendicular
    bisector -> stop 0.5 m before the door -> advance 1.2 m -> stop

The reference PNG is never read.  Door geometry is estimated from the current
``/map`` OccupancyGrid and independently confirmed by a short, multi-frame
dual-LiDAR evidence map.  Motion uses odometry feedback and the existing
``/tmr_cycle/mission_cmd_vel`` exclusive adapter channel.  Every motion loop has fresh-sensor, swept-
footprint, braking-distance, progress, and controller-subscriber guards.

Without ``--execute`` this file only validates configuration and prints the
route.  This makes copying or inspecting it unable to move the robot.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass
from enum import Enum
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Any

import yaml

from continuous_route_geometry import (
    BrakingModel as VerifiedBrakingModel,
    Footprint as VerifiedFootprint,
    GapObservation as VerifiedGapObservation,
    SafetyConfig as VerifiedSafetyConfig,
    TemporalGapStabilizer,
    compute_gap_targets as compute_verified_gap_targets,
    evaluate_swept_corridor as evaluate_verified_swept_corridor,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR.parent / "config" / "start_to_pickup.yaml"


def _load_numbered_module(filename: str, module_name: str):
    path = SCRIPT_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def median_angle(angles: list[float]) -> float:
    if not angles:
        raise ValueError("median_angle requires at least one angle")
    return math.atan2(
        statistics.median([math.sin(value) for value in angles]),
        statistics.median([math.cos(value) for value in angles]),
    )


class RouteState(str, Enum):
    WAIT_READY = "WAIT_READY"
    INITIAL_FORWARD = "INITIAL_FORWARD"
    TURN_CW90 = "TURN_CW90"
    ACQUIRE_DOOR = "ACQUIRE_DOOR"
    ALIGN_TO_MIDPOINT = "ALIGN_TO_MIDPOINT"
    VERIFY_CORRIDOR = "VERIFY_CORRIDOR"
    CROSS_DOOR = "CROSS_DOOR"
    FINAL_STOP = "FINAL_STOP"
    ABORT = "ABORT"


class MissionAbort(RuntimeError):
    pass


class SensorTimestampError(RuntimeError):
    pass


class StrictTimestampTracker:
    """Accept each sensor frame once; zero/backward stamps fail closed."""

    def __init__(self) -> None:
        self._last: dict[str, int] = {}

    def accept(self, source: str, stamp_ns: int) -> bool:
        if stamp_ns <= 0:
            raise SensorTimestampError(f"{source} has a zero header timestamp")
        previous = self._last.get(source)
        if previous is not None and stamp_ns < previous:
            raise SensorTimestampError(f"{source} header timestamp moved backwards")
        if previous == stamp_ns:
            return False
        self._last[source] = stamp_ns
        return True


@dataclass(frozen=True)
class SafetyResult:
    blocked: bool
    speed_scale: float
    nearest_along_m: float | None
    hard_stop_distance_m: float
    hit_count: int
    reason: str


@dataclass(frozen=True)
class DoorTargets:
    predoor: tuple[float, float]
    postdoor: tuple[float, float]
    centre_tangent_coordinate: float
    low_edge_tangent_coordinate: float
    high_edge_tangent_coordinate: float
    required_opening_width_m: float
    available_side_clearance_m: float


@dataclass(frozen=True)
class FixedDoorRouteTargets:
    travel_normal: tuple[float, float]
    half_metre_before_door: tuple[float, float]
    final_after_forward: tuple[float, float]


def compute_fixed_door_route_targets(
    midpoint: tuple[float, float],
    predoor: tuple[float, float],
    postdoor: tuple[float, float],
    before_door_m: float,
    forward_from_before_door_m: float,
) -> FixedDoorRouteTargets:
    """Return base-centre targets on the detected doorway perpendicular bisector."""
    dx = float(postdoor[0]) - float(predoor[0])
    dy = float(postdoor[1]) - float(predoor[1])
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        raise ValueError("door travel normal is degenerate")
    nx, ny = dx / length, dy / length
    before = (
        float(midpoint[0]) - nx * float(before_door_m),
        float(midpoint[1]) - ny * float(before_door_m),
    )
    final = (
        before[0] + nx * float(forward_from_before_door_m),
        before[1] + ny * float(forward_from_before_door_m),
    )
    return FixedDoorRouteTargets((nx, ny), before, final)


def compute_door_targets(
    gap,
    front_m: float,
    rear_m: float,
    half_width_m: float,
    predoor_clearance_m: float,
    postdoor_clearance_m: float,
    side_clearance_m: float,
) -> DoorTargets:
    """Compute frozen-reference targets using rectangular footprint support."""

    nx, ny = gap.normal
    tx, ty = gap.tangent
    centre_s = tx * gap.midpoint[0] + ty * gap.midpoint[1]
    edge_values = sorted(
        [
            tx * gap.right_edge[0] + ty * gap.right_edge[1],
            tx * gap.left_edge[0] + ty * gap.left_edge[1],
        ]
    )
    verified = compute_verified_gap_targets(
        VerifiedGapObservation(
            frame_id="frozen_door",
            midpoint=tuple(gap.midpoint),
            width_m=float(gap.width),
            normal=tuple(gap.normal),
        ),
        footprint=VerifiedFootprint(
            front_m=front_m,
            rear_m=rear_m,
            width_m=2.0 * half_width_m,
        ),
        robot_heading=(1.0, 0.0),
        pre_door_clearance_m=predoor_clearance_m,
        post_door_clearance_m=postdoor_clearance_m,
        side_clearance_m=side_clearance_m,
    )
    return DoorTargets(
        predoor=verified.pre_door,
        postdoor=verified.post_door,
        centre_tangent_coordinate=centre_s,
        low_edge_tangent_coordinate=edge_values[0],
        high_edge_tangent_coordinate=edge_values[1],
        required_opening_width_m=verified.required_opening_width_m,
        available_side_clearance_m=verified.available_side_clearance_m,
    )


def evaluate_swept_corridor(
    points: list[tuple[float, float]],
    vx: float,
    vy: float,
    cfg: dict,
    footprint: dict,
) -> SafetyResult:
    """Braking-aware rectangular swept-corridor guard in current body frame."""

    speed = math.hypot(vx, vy)
    if speed < 1e-4:
        return SafetyResult(False, 1.0, None, 0.0, 0, "stationary")
    ux, uy = vx / speed, vy / speed
    front = float(footprint["front_m"])
    rear = float(footprint["rear_m"])
    half_width = float(footprint["width_m"]) * 0.5
    longitudinal = front if ux >= 0.0 else rear
    extent = abs(ux) * longitudinal + abs(uy) * half_width
    across_extent = (
        abs(uy) * max(front, rear)
        + abs(ux) * half_width
        + float(cfg["side_margin_m"])
    )
    brake = speed * speed / (2.0 * float(cfg["brake_accel_mps2"]))
    brake += speed * float(cfg["reaction_time_s"])
    hard_distance = extent + float(cfg["hard_margin_m"]) + brake
    slow_distance = hard_distance + float(cfg["slowdown_extra_m"])
    hits: list[float] = []
    for x, y in points:
        along = ux * x + uy * y
        across = abs(-uy * x + ux * y)
        # Ignore returns already inside the nominal footprint.  Those are
        # normally chassis/self returns; anything immediately beyond it is a
        # hard stop even if only one beam sees it.
        if extent - 0.025 <= along <= slow_distance and across <= across_extent:
            hits.append(along)
    if not hits:
        return SafetyResult(False, 1.0, None, hard_distance, 0, "clear")
    nearest = min(hits)
    hard_hits = sum(value <= hard_distance for value in hits)
    if hard_hits:
        return SafetyResult(
            True,
            0.0,
            nearest,
            hard_distance,
            hard_hits,
            "return inside braking envelope",
        )
    if len(hits) < int(cfg["minimum_slow_hits"]):
        return SafetyResult(False, 1.0, nearest, hard_distance, len(hits), "isolated soft-zone return")
    scale = clamp((nearest - hard_distance) / max(0.01, slow_distance - hard_distance), 0.18, 1.0)
    return SafetyResult(False, scale, nearest, hard_distance, len(hits), "slow zone")


def filter_self_returns(
    points: list[tuple[float, float]],
    footprint: dict,
    padding_m: float,
) -> list[tuple[float, float]]:
    """Remove only returns inside the measured carried-body rectangle."""

    front = float(footprint["front_m"]) + padding_m
    rear = float(footprint["rear_m"]) + padding_m
    half_width = 0.5 * float(footprint["width_m"]) + padding_m
    return [
        (x, y)
        for x, y in points
        if not (-rear <= x <= front and abs(y) <= half_width)
    ]


def filter_future_motion_returns(
    points: list[tuple[float, float]],
    vx: float,
    vy: float,
    footprint: dict,
    leading_slack_m: float = 0.025,
) -> list[tuple[float, float]]:
    """Keep only returns in the volume newly swept by this translation.

    A point behind the current leading face moves farther away during that
    command.  Counting rear-corner returns in a forward sweep caused the live
    TMR false stop (three fixed returns, negative clearance).  Directional
    projection keeps front, reverse, lateral and diagonal checks symmetric
    without hard-coding any scene or sensor coordinates.
    """

    speed = math.hypot(vx, vy)
    if speed < 1e-6:
        return []
    ux, uy = vx / speed, vy / speed
    front = float(footprint["front_m"])
    rear = float(footprint["rear_m"])
    half_width = 0.5 * float(footprint["width_m"])
    x_support = front if ux >= 0.0 else rear
    leading_support = abs(ux) * x_support + abs(uy) * half_width
    threshold = max(0.0, leading_support - float(leading_slack_m))
    return [(x, y) for x, y in points if ux * x + uy * y >= threshold]


def _validate_config(cfg: dict) -> None:
    for section in ("interfaces", "mission", "footprint", "motion", "safety", "door", "table"):
        if section not in cfg:
            raise ValueError(f"missing config section: {section}")
    footprint = cfg["footprint"]
    if any(
        float(footprint[key]) <= 0.0
        for key in ("front_m", "rear_m", "width_m", "rotation_clearance_m")
    ):
        raise ValueError("footprint dimensions must be positive")
    door = cfg["door"]
    minimum_width = float(footprint["width_m"]) + 2.0 * float(door["side_margin_m"])
    if float(door["maximum_width_m"]) <= minimum_width:
        raise ValueError("door maximum width must exceed footprint plus both side margins")
    if not 0.0 < float(cfg["mission"]["initial_forward_m"]) <= 3.0:
        raise ValueError("initial_forward_m is outside the supported range")
    if abs(float(cfg["mission"]["clockwise_turn_deg"]) - 90.0) > 0.1:
        raise ValueError("this mission requires a clockwise 90 degree turn")
    if not 0.10 <= float(cfg["mission"]["before_door_m"]) <= 1.50:
        raise ValueError("mission.before_door_m must be in [0.10, 1.50]")
    if not 0.10 <= float(cfg["mission"]["forward_from_before_door_m"]) <= 3.0:
        raise ValueError("mission.forward_from_before_door_m must be in [0.10, 3.0]")
    if str(cfg["interfaces"]["command_topic"]) != "/tmr_cycle/mission_cmd_vel":
        raise ValueError(
            "mission must use the exclusive /tmr_cycle/mission_cmd_vel adapter channel"
        )
    if str(cfg["interfaces"].get("mission_lease_topic", "")) != "/tmr_cycle/mission_active":
        raise ValueError("mission lease topic must be /tmr_cycle/mission_active")
    if str(cfg["interfaces"].get("adapter_output_topic", "")) != "/swerve_drive_controller/cmd_vel":
        raise ValueError("adapter output must be the installed swerve controller input")
    scans = cfg["interfaces"]["scan_topics"]
    if not isinstance(scans, list) or len(scans) != 2 or scans[0] == scans[1]:
        raise ValueError("exactly two distinct raw scan topics are required")
    for key in (
        "initial_speed_mps",
        "align_speed_mps",
        "door_speed_mps",
        "table_search_speed_mps",
        "table_approach_speed_mps",
    ):
        if not 0.01 <= float(cfg["motion"][key]) <= 0.20:
            raise ValueError(f"motion.{key} must be in [0.01, 0.20]")
    if not 0.15 <= float(cfg["safety"]["scan_stale_s"]) <= 1.0:
        raise ValueError("safety.scan_stale_s must be in [0.15, 1.0]")
    if not 0.15 <= float(cfg["safety"]["odom_stale_s"]) <= 1.0:
        raise ValueError("safety.odom_stale_s must be in [0.15, 1.0]")
    safety = cfg["safety"]
    if not 0.0 <= float(safety["self_mask_padding_m"]) <= 0.03:
        raise ValueError("safety.self_mask_padding_m must be in [0.0, 0.03]")
    if not 0.05 <= float(safety["brake_accel_mps2"]) <= 2.0:
        raise ValueError("safety.brake_accel_mps2 must be in [0.05, 2.0]")
    if not 0.0 <= float(safety["reaction_time_s"]) <= 1.0:
        raise ValueError("safety.reaction_time_s must be in [0.0, 1.0]")
    if not 0.03 <= float(safety["hard_margin_m"]) <= 0.30:
        raise ValueError("safety.hard_margin_m must be in [0.03, 0.30]")
    if not 2 <= int(safety["map_minimum_occupied_cluster_cells"]) <= 12:
        raise ValueError("safety.map_minimum_occupied_cluster_cells must be in [2, 12]")
    if not 0.02 <= float(safety["side_margin_m"]) <= 0.20:
        raise ValueError("safety.side_margin_m must be in [0.02, 0.20]")
    if not 0.05 <= float(safety["slowdown_extra_m"]) <= 0.60:
        raise ValueError("safety.slowdown_extra_m must be in [0.05, 0.60]")
    turning_radius = math.hypot(
        max(float(footprint["front_m"]), float(footprint["rear_m"])),
        0.5 * float(footprint["width_m"]),
    )
    if float(footprint["rotation_clearance_m"]) < turning_radius + 0.03:
        raise ValueError("footprint.rotation_clearance_m is below the rectangular swept radius")
    if not 1 <= int(door["beam_stride"]) <= 4:
        raise ValueError("door.beam_stride must be in [1, 4]")
    if not 2.0 <= float(door["local_collect_s"]) <= 5.0:
        raise ValueError("door.local_collect_s must be in [2.0, 5.0]")
    if not 1 <= int(door["minimum_partition_detections"]) <= 2:
        raise ValueError("door.minimum_partition_detections must be 1 or 2")
    table = cfg["table"]
    if not 2 <= int(table["min_observations"]) <= 10:
        raise ValueError("table.min_observations must be in [2, 10]")
    if not 0.15 <= float(table["desired_front_clearance_m"]) <= 0.80:
        raise ValueError("table.desired_front_clearance_m must be in [0.15, 0.80]")
    if not 0.30 <= float(table["search_max_forward_m"]) <= 3.0:
        raise ValueError("table.search_max_forward_m must be in [0.30, 3.0]")


def _load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    _validate_config(cfg)
    return cfg


def _dry_run_summary(cfg: dict, config_path: Path) -> dict:
    minimum_door = float(cfg["footprint"]["width_m"]) + 2.0 * float(cfg["door"]["side_margin_m"])
    return {
        "status": "configuration_valid",
        "motion_enabled": False,
        "config": str(config_path),
        "route": [state.value for state in RouteState if state not in (RouteState.ABORT,)],
        "initial_forward_m": float(cfg["mission"]["initial_forward_m"]),
        "turn_deg": -float(cfg["mission"]["clockwise_turn_deg"]),
        "before_door_m": float(cfg["mission"]["before_door_m"]),
        "forward_from_before_door_m": float(cfg["mission"]["forward_from_before_door_m"]),
        "door_source": "current /map candidate + current dual-LiDAR confirmation",
        "minimum_accepted_door_width_m": minimum_door,
        "command_topic": str(cfg["interfaces"]["command_topic"]),
        "example_image_used": False,
    }


def run_ros_mission(cfg: dict) -> tuple[int, dict]:
    # ROS imports are intentionally delayed until --execute.  A local syntax/
    # configuration check cannot discover or command a robot by accident.
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rclpy.time import Time
    from std_msgs.msg import Bool, String

    gap_core = _load_numbered_module("05_right_turn_map_gap.py", "tmr_gap_core")
    import runtime_grid_collision
    import runtime_map_door
    from table_leg_detection import DetectionError, detect_pair

    # Tighten the inherited freshness policy for every inherited motion method.
    gap_core.SCAN_STALE_S = float(cfg["safety"]["scan_stale_s"])
    gap_core.ODOM_STALE_S = float(cfg["safety"]["odom_stale_s"])

    interfaces = cfg["interfaces"]
    motion = cfg["motion"]
    door_cfg = cfg["door"]
    footprint = cfg["footprint"]
    safety_cfg = cfg["safety"]
    table_cfg = cfg["table"]
    # The reused detector reads these module constants at call time.  Keep its
    # free-through and turn-clearance proof aligned with the configured full
    # carried envelope instead of silently assuming the bare base.
    gap_core.FOOTPRINT_FRONT_M = float(footprint["front_m"])
    gap_core.FOOTPRINT_REAR_M = float(footprint["rear_m"])
    gap_core.FOOTPRINT_WIDTH_M = float(footprint["width_m"])
    gap_core.FOOTPRINT_HALF_WIDTH_M = 0.5 * float(footprint["width_m"])
    gap_core.ROTATION_CLEARANCE_M = float(footprint["rotation_clearance_m"])

    inherited_args = argparse.Namespace(
        scan_topic=list(interfaces["scan_topics"]),
        odom_topic=str(interfaces["odom_topic"]),
        cmd_topic=str(interfaces["command_topic"]),
        base_frame=str(interfaces["base_frame"]),
        ready_timeout_s=15.0,
        turn_timeout_s=24.0,
        drive_timeout_s=float(motion["segment_timeout_s"]),
        collect_s=float(door_cfg["local_collect_s"]),
        turn_speed_rps=float(motion["turn_speed_rps"]),
        drive_speed_mps=float(motion["initial_speed_mps"]),
        beam_stride=int(door_cfg["beam_stride"]),
        gap_margin_m=float(door_cfg["side_margin_m"]),
        maximum_gap_width_m=float(door_cfg["maximum_width_m"]),
        stop_margin_m=float(door_cfg["predoor_clearance_m"]),
    )

    class ContinuousRouteNode(gap_core.GapApproachNode):
        def __init__(self) -> None:
            super().__init__(inherited_args)
            self._state = RouteState.WAIT_READY
            self._abort_latched = False
            self._abort_reason = ""
            self._map_msg = None
            self._map_snapshot = None
            self._map_received_at = 0.0
            self._map_stamp_ns = 0
            self._map_stamp_fault = ""
            self._nav_goal_active = False
            self._sensor_stamps = StrictTimestampTracker()
            self._sensor_stamp_fault = ""
            self._verified_safety = VerifiedSafetyConfig(
                footprint=VerifiedFootprint(
                    front_m=float(footprint["front_m"]),
                    rear_m=float(footprint["rear_m"]),
                    width_m=float(footprint["width_m"]),
                ),
                braking=VerifiedBrakingModel(
                    deceleration_mps2=float(safety_cfg["brake_accel_mps2"]),
                    reaction_time_s=float(safety_cfg["reaction_time_s"]),
                    margin_m=float(safety_cfg["hard_margin_m"]),
                ),
                corridor_margin_m=float(safety_cfg["side_margin_m"]),
            )
            self._state_publisher = self.create_publisher(String, str(interfaces["state_topic"]), 10)
            self._lease_publisher = self.create_publisher(
                Bool,
                str(interfaces["mission_lease_topic"]),
                10,
            )
            self._last_owner_check = 0.0
            self._control_lease_acquired = False
            # Hold the selected command at 25 Hz for the whole process.  This
            # keeps the adapter's manual channel selected during stationary
            # map collection as well as motion; after a process crash the
            # adapter's independent 0.5 s watchdog still sends zero.
            self.create_timer(0.040, self._command_heartbeat)
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                OccupancyGrid,
                str(interfaces["map_topic"]),
                self._on_map,
                map_qos,
            )
            self.create_subscription(
                GoalStatusArray,
                "/navigate_to_pose/_action/status",
                self._on_nav_status,
                10,
            )

        def _on_odom(self, msg) -> None:
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            try:
                if not self._sensor_stamps.accept("odometry", stamp_ns):
                    # A bridge replaying the same sample must not refresh
                    # arrival-time freshness.  The previous state will age out.
                    return
            except SensorTimestampError as exc:
                self._sensor_stamp_fault = str(exc)
                self._command = (0.0, 0.0, 0.0)
                return
            previous = self._odom
            super()._on_odom(msg)
            if previous is None or self._odom is None:
                return
            dt = max(0.0, self._odom.received_at - previous.received_at)
            translation = math.hypot(self._odom.x - previous.x, self._odom.y - previous.y)
            plausible = max(0.12, 0.03 + 0.50 * dt)
            if translation > plausible:
                self._odom_jump = True
                self._abort_reason = (
                    f"odometry translation jumped {translation:.3f} m in {dt:.3f} s "
                    f"(limit {plausible:.3f} m)"
                )
            if self._odom_jump:
                if not self._abort_reason:
                    self._abort_reason = "odometry yaw discontinuity exceeded 45 degrees"
                self._command = (0.0, 0.0, 0.0)

        def _fresh_odom(self):
            if self._sensor_stamp_fault:
                raise MissionAbort(self._sensor_stamp_fault)
            if self._abort_latched and self._abort_reason:
                raise MissionAbort(self._abort_reason)
            if self._odom_jump and self._abort_reason:
                raise MissionAbort(self._abort_reason)
            return super()._fresh_odom()

        def _on_scan(self, topic: str, msg) -> None:
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            source = f"LaserScan {topic}"
            try:
                if not self._sensor_stamps.accept(source, stamp_ns):
                    return
            except SensorTimestampError as exc:
                self._sensor_stamp_fault = str(exc)
                self._command = (0.0, 0.0, 0.0)
                return
            super()._on_scan(topic, msg)

        def _on_map(self, msg) -> None:
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            try:
                if not self._sensor_stamps.accept("OccupancyGrid /map", stamp_ns):
                    # Do not let a replayed latched map refresh its receipt age.
                    return
            except SensorTimestampError as exc:
                # The map is advisory: reject its evidence and continue with
                # the independently verified live dual-LiDAR doorway only.
                self._map_stamp_fault = str(exc)
                return
            self._map_stamp_fault = ""
            self._map_msg = msg
            origin = msg.info.origin
            self._map_snapshot = runtime_map_door.GridSnapshot(
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=float(origin.position.x),
                origin_y=float(origin.position.y),
                origin_yaw=gap_core.yaw_from_quaternion(origin.orientation),
                # The ROS message remains owned by this node; avoid copying a
                # potentially large global map on every 25 Hz safety pass.
                data=msg.data,
            )
            self._map_received_at = time.monotonic()
            self._map_stamp_ns = stamp_ns

        def _on_nav_status(self, msg) -> None:
            # ACCEPTED, EXECUTING, and CANCELING can all resume controller
            # output if the manual heartbeat disappears; refuse mixed owners.
            self._nav_goal_active = any(item.status in (1, 2, 3) for item in msg.status_list)

        def _command_heartbeat(self) -> None:
            if self._control_lease_acquired and not self._abort_latched:
                try:
                    self.assert_control_ownership()
                except MissionAbort as exc:
                    self._abort_latched = True
                    self._abort_reason = str(exc)
                    self._command = (0.0, 0.0, 0.0)
                    self.set_state(RouteState.ABORT, self._abort_reason)
            command = (0.0, 0.0, 0.0) if self._abort_latched else self._command
            self._publish_raw(*command)

        def acquire_control_lease(self) -> None:
            message = Bool()
            message.data = True
            for _ in range(5):
                self._lease_publisher.publish(message)
                self._publish_raw(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.025)
            self.assert_control_ownership(force=True)
            self._control_lease_acquired = True

        def assert_control_ownership(self, force: bool = False) -> None:
            now = time.monotonic()
            if not force and now - self._last_owner_check < 0.20:
                return
            self._last_owner_check = now
            mission_publishers = self.count_publishers(str(interfaces["command_topic"]))
            output_publishers = self.count_publishers(str(interfaces["adapter_output_topic"]))
            if mission_publishers != 1:
                raise MissionAbort(
                    f"exclusive mission topic has {mission_publishers} publishers; expected this process only"
                )
            if output_publishers != 1:
                raise MissionAbort(
                    f"controller input has {output_publishers} publishers; expected cmd_vel_adapter only"
                )

        def set_state(self, state: RouteState, detail: str = "") -> None:
            self._state = state
            message = String()
            message.data = json.dumps(
                {"state": state.value, "detail": detail, "time": time.time()},
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self._state_publisher.publish(message)
            self.get_logger().info(message.data)

        def stop(self) -> None:
            # Faster normal stage handoff than the old 0.6 s repetition.  The
            # adapter also has a 0.5 s zero watchdog.
            self._command = (0.0, 0.0, 0.0)
            self._command_at = time.monotonic()
            for _ in range(8):
                self._publish_raw(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.018)

        def latch_abort(self, reason: str) -> None:
            if not self._abort_latched:
                self._abort_latched = True
                self._abort_reason = reason
                self.set_state(RouteState.ABORT, reason)
            self._command = (0.0, 0.0, 0.0)
            for _ in range(25):
                self._publish_raw(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.020)

        def send(self, vx: float, vy: float, wz: float) -> None:
            self.assert_control_ownership()
            if self._nav_goal_active and (abs(vx) + abs(vy) + abs(wz)) > 1e-9:
                raise MissionAbort("an active Nav2 goal is competing for the base controller")
            if self._abort_latched and (abs(vx) + abs(vy) + abs(wz)) > 1e-9:
                self._publish_raw(0.0, 0.0, 0.0)
                return
            super().send(vx, vy, wz)

        def _runtime_map_collision(self, vx: float, vy: float):
            """Return known-occupancy evidence for the current braking sweep.

            An absent/stale/unknown map is deliberately inconclusive: it never
            authorizes motion and never stops the independent live-LiDAR guard.
            """

            if self._map_snapshot is None or self._map_stamp_fault:
                return None
            receipt_age = time.monotonic() - self._map_received_at
            stamp_age = (
                self.get_clock().now().nanoseconds - self._map_stamp_ns
            ) / 1_000_000_000.0
            maximum_age = float(door_cfg["map_max_age_s"])
            if (
                self._map_stamp_ns <= 0
                or receipt_age > maximum_age
                or stamp_age > maximum_age
                or stamp_age < -0.50
            ):
                return None
            try:
                map_frame = (self._map_msg.header.frame_id or "map").lstrip("/")
                # TF target=map, source=base_link is the pose of base_link in
                # the map frame, exactly the transform expected by the pure
                # occupancy-grid checker.
                transform = self._tf_buffer.lookup_transform(
                    map_frame,
                    self.base_frame,
                    Time(),
                )
                translation = transform.transform.translation
                map_from_base = (
                    float(translation.x),
                    float(translation.y),
                    gap_core.yaw_from_quaternion(transform.transform.rotation),
                )
                return runtime_grid_collision.evaluate_occupancy_grid_sweep(
                    self._map_snapshot,
                    map_from_base,
                    (float(vx), float(vy)),
                    runtime_grid_collision.RectangularFootprint(
                        front_m=float(footprint["front_m"]),
                        rear_m=float(footprint["rear_m"]),
                        width_m=float(footprint["width_m"]),
                    ),
                    runtime_grid_collision.GridSweepConfig(
                        brake_accel_mps2=float(safety_cfg["brake_accel_mps2"]),
                        reaction_time_s=float(safety_cfg["reaction_time_s"]),
                        hard_margin_m=float(safety_cfg["hard_margin_m"]),
                        occupied_threshold=int(door_cfg["occupied_threshold"]),
                        free_threshold=int(door_cfg["free_threshold"]),
                        minimum_occupied_cluster_cells=int(
                            safety_cfg["map_minimum_occupied_cluster_cells"]
                        ),
                    ),
                )
            except Exception:
                # TF/map computation is supporting evidence, not a reason to
                # bypass or stall the always-on live LiDAR braking guard.
                return None

        def _guard_velocity(self, vx: float, vy: float) -> SafetyResult:
            if self._nav_goal_active:
                raise MissionAbort("an active Nav2 goal is competing for the base controller")
            if os.environ.get("TMR_CYCLE_DISABLE_COLLISION_GUARD") == "1":
                return SafetyResult(False, 1.0, None, 0.0, 0, "collision guard explicitly disabled")
            raw_points = self._scan_hit_points_in_base()
            mask_pad = float(safety_cfg["self_mask_padding_m"])
            # The raw scanners may see chassis edges, cables, or the carried
            # arm.  They cannot distinguish those returns from an obstacle
            # already inside the footprint, so remove only the precisely
            # measured body rectangle.  A point immediately outside remains a
            # one-beam hard stop.
            points = filter_self_returns(raw_points, footprint, mask_pad)
            commands = [("current", float(self._command[0]), float(self._command[1]))]
            if math.hypot(vx - self._command[0], vy - self._command[1]) > 1e-5:
                commands.append(("requested", vx, vy))
            decisions: list[SafetyResult] = []
            for label, check_vx, check_vy in commands:
                map_result = self._runtime_map_collision(check_vx, check_vy)
                if map_result is not None and map_result.blocked:
                    raise MissionAbort(
                        f"collision guard ({label} command): runtime OccupancyGrid has "
                        f"{map_result.occupied_cells} known occupied cell(s), largest cluster "
                        f"{map_result.largest_occupied_cluster_cells}, in the "
                        f"rectangular braking sweep; first_contact_progress="
                        f"{map_result.collision_progress_m}, sweep="
                        f"{map_result.swept_distance_m:.3f} m"
                    )
                future_points = filter_future_motion_returns(
                    points,
                    check_vx,
                    check_vy,
                    footprint,
                )
                verified = evaluate_verified_swept_corridor(
                    future_points,
                    (check_vx, check_vy),
                    config=self._verified_safety,
                )
                if verified.blocked:
                    raise MissionAbort(
                        f"collision guard ({label} command): swept footprint entered braking horizon; "
                        f"nearest_clearance={verified.nearest_clearance_m:.3f} m, "
                        f"required={verified.required_clearance_m:.3f} m, hits={verified.hit_count}, "
                        f"nearest_point={verified.nearest_point}"
                    )
                decision = evaluate_swept_corridor(
                    future_points,
                    check_vx,
                    check_vy,
                    safety_cfg,
                    footprint,
                )
                if decision.blocked:
                    nearest = decision.nearest_along_m
                    raise MissionAbort(
                        f"collision guard ({label} command): {decision.reason}; nearest={nearest}, "
                        f"hard_distance={decision.hard_stop_distance_m:.3f}, hits={decision.hit_count}"
                    )
                decisions.append(decision)
            nearest_values = [
                item.nearest_along_m
                for item in decisions
                if item.nearest_along_m is not None
            ]
            return SafetyResult(
                blocked=False,
                speed_scale=min(item.speed_scale for item in decisions),
                nearest_along_m=min(nearest_values) if nearest_values else None,
                hard_stop_distance_m=max(item.hard_stop_distance_m for item in decisions),
                hit_count=max(item.hit_count for item in decisions),
                reason="; ".join(sorted({item.reason for item in decisions})),
            )

        def _rotation_clearance(self) -> float:
            if os.environ.get("TMR_CYCLE_DISABLE_COLLISION_GUARD") == "1":
                return math.inf
            points = filter_self_returns(
                self._scan_hit_points_in_base(),
                footprint,
                float(safety_cfg["self_mask_padding_m"]),
            )
            distances = sorted(math.hypot(x, y) for x, y in points)
            if len(distances) < 8:
                raise MissionAbort("too few external LiDAR returns for rotation clearance")
            return distances[0]

        def turn_relative_deg(self, requested_deg: float) -> dict:
            start = self._fresh_odom()
            target = start.unwrapped_yaw + math.radians(requested_deg)
            deadline = time.monotonic() + 12.0
            stable_since: float | None = None
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.04)
                    state = self._fresh_odom()
                    error = target - state.unwrapped_yaw
                    if abs(error) <= math.radians(1.0):
                        self.send(0.0, 0.0, 0.0)
                        stable_since = stable_since or time.monotonic()
                        if time.monotonic() - stable_since >= 0.25:
                            break
                        continue
                    stable_since = None
                    self.send(
                        0.0,
                        0.0,
                        clamp(1.35 * error, -float(motion["turn_speed_rps"]), float(motion["turn_speed_rps"])),
                    )
                else:
                    raise TimeoutError("relative yaw correction timed out")
            finally:
                self.stop()
            end = self.wait_stationary()
            return {
                "requested_deg": requested_deg,
                "actual_deg": math.degrees(end.unwrapped_yaw - start.unwrapped_yaw),
            }

        def wait_fresh_inputs(self, timeout_s: float = 0.80):
            """Bridge a bounded CPU-heavy detection step without false stale aborts."""
            deadline = time.monotonic() + timeout_s
            last_error = "no sensor callback"
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                try:
                    return self._fresh_odom(), self._fresh_scans()
                except RuntimeError as exc:
                    last_error = str(exc)
            raise MissionAbort(f"sensors did not become fresh within {timeout_s:.2f}s: {last_error}")

        def move_to_reference(
            self,
            reference,
            target: tuple[float, float],
            label: str,
            maximum_speed: float,
            axis: str | None = None,
            door_gap=None,
            door_targets: DoorTargets | None = None,
        ) -> dict:
            start, _records = self.wait_fresh_inputs()
            start_x, start_y, _ = self._base_pose_in_reference(start, reference)
            initial_distance = math.hypot(target[0] - start_x, target[1] - start_y)
            if initial_distance <= float(motion["position_tolerance_m"]):
                return {"label": label, "distance_m": 0.0, "already_at_target": True}
            deadline = time.monotonic() + float(motion["segment_timeout_s"])
            best_remaining = initial_distance
            progress_at = time.monotonic()
            minimum_return = math.inf
            stable_since = None
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.025)
                    state = self._fresh_odom()
                    self._fresh_scans()
                    if self._cmd_publisher.get_subscription_count() < 1:
                        raise MissionAbort("velocity adapter subscriber disappeared")
                    x, y, relative_yaw = self._base_pose_in_reference(state, reference)
                    ex, ey = target[0] - x, target[1] - y
                    remaining = math.hypot(ex, ey)
                    yaw_error = wrap(reference.yaw - state.yaw)
                    if abs(yaw_error) > math.radians(float(motion["max_heading_error_deg"])):
                        raise MissionAbort(
                            f"{label}: heading error {math.degrees(yaw_error):.2f} deg exceeds recoverable range"
                        )
                    if (
                        remaining <= float(motion["position_tolerance_m"])
                        and abs(yaw_error) <= math.radians(float(motion["yaw_tolerance_deg"]))
                    ):
                        self._guard_velocity(0.0, 0.0)
                        self.send(0.0, 0.0, 0.0)
                        stable_since = stable_since or time.monotonic()
                        if time.monotonic() - stable_since >= 0.22:
                            break
                        continue
                    stable_since = None

                    speed_cap = min(maximum_speed, max(0.025, 0.72 * remaining))
                    if axis == "x":
                        vx_ref = clamp(0.75 * ex, -speed_cap, speed_cap)
                        vy_ref = clamp(0.90 * ey, -0.020, 0.020)
                    elif axis == "y":
                        vx_ref = clamp(0.90 * ex, -0.020, 0.020)
                        vy_ref = clamp(0.75 * ey, -speed_cap, speed_cap)
                    else:
                        norm = max(remaining, 1e-9)
                        vx_ref, vy_ref = speed_cap * ex / norm, speed_cap * ey / norm

                    # Keep the doorway-centre correction small and geometric;
                    # never chase a jumping SLAM map while the body is crossing.
                    if door_gap is not None and door_targets is not None:
                        tangent_s = door_gap.tangent[0] * x + door_gap.tangent[1] * y
                        centre_error = door_targets.centre_tangent_coordinate - tangent_s
                        vy_ref += clamp(0.55 * centre_error, -0.018, 0.018)
                        body_shape = self._verified_safety.footprint
                        heading_ref = (math.cos(relative_yaw), math.sin(relative_yaw))
                        tangent = tuple(door_gap.tangent)
                        low_support = body_shape.support((-tangent[0], -tangent[1]), heading_ref)
                        high_support = body_shape.support(tangent, heading_ref)
                        low_clearance = tangent_s - door_targets.low_edge_tangent_coordinate - low_support
                        high_clearance = door_targets.high_edge_tangent_coordinate - tangent_s - high_support
                        if (
                            os.environ.get("TMR_CYCLE_DISABLE_COLLISION_GUARD") != "1"
                            and min(low_clearance, high_clearance) < 0.035
                        ):
                            raise MissionAbort(
                                f"{label}: live doorway side clearance fell below 3.5 cm"
                            )

                    frame_delta = wrap(reference.yaw - state.yaw)
                    vx = math.cos(frame_delta) * vx_ref - math.sin(frame_delta) * vy_ref
                    vy = math.sin(frame_delta) * vx_ref + math.cos(frame_delta) * vy_ref
                    safety = self._guard_velocity(vx, vy)
                    if safety.nearest_along_m is not None:
                        minimum_return = min(minimum_return, safety.nearest_along_m)
                    vx *= safety.speed_scale
                    vy *= safety.speed_scale
                    wz = clamp(
                        1.1 * yaw_error,
                        -float(motion["max_yaw_correction_rps"]),
                        float(motion["max_yaw_correction_rps"]),
                    )
                    self.send(vx, vy, wz)

                    if remaining < best_remaining - 0.010:
                        best_remaining = remaining
                        progress_at = time.monotonic()
                    elif time.monotonic() - progress_at > float(motion["no_progress_timeout_s"]):
                        raise MissionAbort(f"{label}: no odometry progress")
                else:
                    raise MissionAbort(f"{label}: segment timeout")
            finally:
                self.stop()
            end = self.wait_stationary(timeout_s=2.5)
            end_x, end_y, _ = self._base_pose_in_reference(end, reference)
            error = math.hypot(target[0] - end_x, target[1] - end_y)
            if error > max(0.045, 1.6 * float(motion["position_tolerance_m"])):
                raise MissionAbort(f"{label}: final position error {error:.3f} m")
            return {
                "label": label,
                "requested_m": initial_distance,
                "final_error_m": error,
                "minimum_dynamic_return_m": None if math.isinf(minimum_return) else minimum_return,
            }

        @staticmethod
        def _partition_map(local_map, index: int, count: int):
            partition = gap_core.LocalEvidenceMap(local_map.resolution)
            marked: set[tuple[str, str]] = set()
            for ray in local_map.rays:
                try:
                    sequence = int(ray.scan_id.rsplit(":", 1)[1])
                except (IndexError, ValueError):
                    sequence = abs(hash(ray.scan_id))
                if sequence % count != index:
                    continue
                partition.add_ray(ray)
                key = (ray.source, ray.scan_id)
                if key not in marked:
                    partition.mark_frame(ray.source)
                    marked.add(key)
            return partition

        def detect_stable_door(self) -> tuple[Any, Any, dict]:
            errors: list[str] = []
            minimum_width = float(footprint["width_m"]) + 2.0 * float(door_cfg["side_margin_m"])
            for attempt in range(1, 3):
                try:
                    reference, local_map = self.collect_local_map()
                    full = gap_core.detect_frontal_gap(local_map, minimum_width, float(door_cfg["maximum_width_m"]))
                    observations = [full]
                    partition_errors: list[str] = []
                    for index in range(2):
                        partition = self._partition_map(local_map, index, 2)
                        try:
                            observations.append(
                                gap_core.detect_frontal_gap(
                                    partition,
                                    minimum_width,
                                    float(door_cfg["maximum_width_m"]),
                                )
                            )
                        except RuntimeError as exc:
                            partition_errors.append(f"partition{index}={exc}")
                    required = int(door_cfg["minimum_partition_detections"])
                    if len(observations) - 1 < required:
                        raise RuntimeError(
                            f"only {len(observations)-1}/{required} independent partitions detected the door; "
                            + "; ".join(partition_errors)
                        )
                    tracker = TemporalGapStabilizer(
                        required_consecutive=len(observations),
                        minimum_width_m=minimum_width,
                        maximum_midpoint_delta_m=float(door_cfg["midpoint_stability_m"]),
                        maximum_width_delta_m=float(door_cfg["width_stability_m"]),
                        maximum_normal_angle_deg=float(door_cfg["normal_stability_deg"]),
                        maximum_interframe_s=None,
                        expected_normal=(1.0, 0.0),
                    )
                    stable_gap = None
                    for index, observation in enumerate(observations):
                        stable_gap = tracker.update(
                            VerifiedGapObservation(
                                frame_id=f"partition-{index}",
                                midpoint=tuple(observation.midpoint),
                                width_m=float(observation.width),
                                normal=tuple(observation.normal),
                            )
                        )
                    if stable_gap is None:
                        raise RuntimeError("independent map partitions did not form a stable door track")
                    mid_x = statistics.median([item.midpoint[0] for item in observations])
                    mid_y = statistics.median([item.midpoint[1] for item in observations])
                    width = statistics.median([item.width for item in observations])
                    normals = [math.atan2(item.normal[1], item.normal[0]) for item in observations]
                    normal = median_angle(normals)
                    maximum_midpoint_error = max(
                        math.hypot(item.midpoint[0] - mid_x, item.midpoint[1] - mid_y)
                        for item in observations
                    )
                    maximum_width_error = max(abs(item.width - width) for item in observations)
                    maximum_normal_error = max(abs(wrap(value - normal)) for value in normals)
                    if maximum_midpoint_error > float(door_cfg["midpoint_stability_m"]):
                        raise RuntimeError(f"door midpoint is unstable by {maximum_midpoint_error:.3f} m")
                    if maximum_width_error > float(door_cfg["width_stability_m"]):
                        raise RuntimeError(f"door width is unstable by {maximum_width_error:.3f} m")
                    if maximum_normal_error > math.radians(float(door_cfg["normal_stability_deg"])):
                        raise RuntimeError(
                            f"door normal is unstable by {math.degrees(maximum_normal_error):.2f} deg"
                        )
                    stability = {
                        "attempt": attempt,
                        "partition_detections": len(observations) - 1,
                        "midpoint_spread_m": maximum_midpoint_error,
                        "width_spread_m": maximum_width_error,
                        "normal_spread_deg": math.degrees(maximum_normal_error),
                        "verified_stable_gap": asdict(stable_gap),
                        "map_summary": local_map.summary(),
                    }
                    return reference, full, stability
                except RuntimeError as exc:
                    errors.append(f"attempt{attempt}={exc}")
                    self.stop()
            raise MissionAbort("door detection failed after one immediate re-observation: " + " | ".join(errors))

        def runtime_map_report(self, reference, lidar_gap) -> dict:
            """Use this run's OccupancyGrid as a candidate/cross-check, never pixels."""
            if os.environ.get("TMR_CYCLE_DISABLE_COLLISION_GUARD") == "1":
                return {
                    "available": False,
                    "decision": "disabled",
                    "reason": "runtime map collision/contradiction checks explicitly disabled",
                }
            if self._map_stamp_fault:
                return {
                    "available": False,
                    "decision": "lidar_only",
                    "reason": f"rejected untrustworthy /map update: {self._map_stamp_fault}",
                }
            if self._map_msg is None:
                return {"available": False, "decision": "lidar_only", "reason": "no /map received"}
            receipt_age = time.monotonic() - self._map_received_at
            if self._map_stamp_ns <= 0:
                return {
                    "available": False,
                    "decision": "lidar_only",
                    "reason": "/map has no trustworthy header timestamp; latched old maps cannot veto",
                }
            stamp_age = (
                self.get_clock().now().nanoseconds - self._map_stamp_ns
            ) / 1_000_000_000.0
            maximum_age = float(door_cfg["map_max_age_s"])
            if stamp_age < -0.50:
                return {
                    "available": False,
                    "decision": "lidar_only",
                    "reason": f"/map timestamp is {abs(stamp_age):.2f}s in the future",
                }
            if receipt_age > maximum_age or stamp_age > maximum_age:
                return {
                    "available": False,
                    "decision": "lidar_only",
                    "reason": f"/map stale: receipt_age={receipt_age:.2f}s, stamp_age={stamp_age:.2f}s",
                }
            try:
                import runtime_map_door

                msg = self._map_msg
                frame = (msg.header.frame_id or "map").lstrip("/")
                transform = self._tf_buffer.lookup_transform(
                    frame,
                    self.base_frame,
                    Time(),
                    Duration(seconds=0.20),
                )
                t = transform.transform.translation
                map_yaw = gap_core.yaw_from_quaternion(transform.transform.rotation)
                origin = msg.info.origin
                snapshot = runtime_map_door.GridSnapshot(
                    width=int(msg.info.width),
                    height=int(msg.info.height),
                    resolution=float(msg.info.resolution),
                    origin_x=float(origin.position.x),
                    origin_y=float(origin.position.y),
                    origin_yaw=gap_core.yaw_from_quaternion(origin.orientation),
                    data=tuple(int(value) for value in msg.data),
                )
                transform_tuple = (float(t.x), float(t.y), map_yaw)
                detector_config = runtime_map_door.DoorDetectorConfig(
                    occupied_threshold=int(door_cfg["occupied_threshold"]),
                    free_threshold=int(door_cfg["free_threshold"]),
                    min_width_m=float(footprint["width_m"]) + 2.0 * float(door_cfg["side_margin_m"]),
                    max_width_m=float(door_cfg["maximum_width_m"]),
                )
                # Validate the exact LiDAR door first.  A global coarse search
                # may legitimately fail because the map is sparse or contains
                # several doors; it must never hide a known obstacle inside
                # the candidate we are actually about to cross.
                validation = runtime_map_door.validate_candidate(
                    snapshot,
                    lidar_gap,
                    transform_tuple,
                    detector_config,
                )
                report = {
                    "available": True,
                    "map_receipt_age_s": receipt_age,
                    "map_stamp_age_s": stamp_age,
                    "map_resolution_m": snapshot.resolution,
                    "validation": asdict(validation),
                }
                if validation.status == "conflict":
                    raise MissionAbort(
                        "runtime map contains known occupied evidence that contradicts "
                        f"the LiDAR doorway: {'; '.join(validation.reasons)}"
                    )
                try:
                    candidate = runtime_map_door.detect_doorway(
                        snapshot,
                        transform_tuple,
                        detector_config,
                    )
                    candidate_midpoint = tuple(candidate.midpoint)
                    disagreement = math.hypot(
                        candidate_midpoint[0] - lidar_gap.midpoint[0],
                        candidate_midpoint[1] - lidar_gap.midpoint[1],
                    )
                    report["candidate"] = asdict(candidate)
                    report["map_lidar_midpoint_disagreement_m"] = disagreement
                    report["coarse_candidate_matches_lidar"] = disagreement <= 0.18
                except Exception as exc:
                    report["coarse_candidate"] = None
                    report["coarse_candidate_reason"] = f"{type(exc).__name__}: {exc}"
                report["decision"] = "map_supports_lidar_door" if validation.supported else "lidar_confirmed_map_inconclusive"
                return report
            except MissionAbort:
                raise
            except Exception as exc:
                # A new map can lag or be sparse at the door.  Unknown is not
                # treated as free; the independent multi-frame LiDAR detector
                # remains sufficient and crossing stays under live hard guard.
                return {
                    "available": True,
                    "decision": "lidar_only",
                    "reason": f"runtime map inconclusive: {type(exc).__name__}: {exc}",
                }

        def _points_in_reference(self, state, reference) -> list[tuple[float, float]]:
            base_x, base_y, base_yaw = self._base_pose_in_reference(state, reference)
            c, s = math.cos(base_yaw), math.sin(base_yaw)
            external_points = filter_self_returns(
                self._scan_hit_points_in_base(),
                footprint,
                float(safety_cfg["self_mask_padding_m"]),
            )
            return [
                (base_x + c * x - s * y, base_y + s * x + c * y)
                for x, y in external_points
            ]

        def search_left_table_legs(self, reference) -> tuple[Any, dict]:
            start = self._fresh_odom()
            start_x, start_y, _ = self._base_pose_in_reference(start, reference)
            windows: deque[tuple[float, list[tuple[float, float]]]] = deque()
            stable: deque[tuple[float, float]] = deque(maxlen=int(table_cfg["min_observations"]))
            last_sequences: tuple[int, ...] | None = None
            deadline = time.monotonic() + float(motion["segment_timeout_s"])
            progress_at = time.monotonic()
            best_x = start_x
            last_error = "no samples yet"
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.035)
                    state = self._fresh_odom()
                    records = self._fresh_scans()
                    x, y, relative_yaw = self._base_pose_in_reference(state, reference)
                    travelled = x - start_x
                    if travelled > float(table_cfg["search_max_forward_m"]):
                        raise MissionAbort(f"left table-leg pair not found; last={last_error}")
                    sequences = tuple(record.sequence for record in records)
                    new_scan = sequences != last_sequences
                    if new_scan:
                        last_sequences = sequences
                        windows.append((time.monotonic(), self._points_in_reference(state, reference)))
                    cutoff = time.monotonic() - float(table_cfg["rolling_window_s"])
                    while windows and windows[0][0] < cutoff:
                        windows.popleft()
                    points = [point for _stamp, frame_points in windows for point in frame_points]
                    dynamic_cfg = {
                        "roi": {
                            "x": [x + float(table_cfg["roi_ahead_m"][0]), x + float(table_cfg["roi_ahead_m"][1])],
                            "y": [y + float(table_cfg["roi_left_m"][0]), y + float(table_cfg["roi_left_m"][1])],
                        },
                        "grid_resolution": float(table_cfg["grid_resolution"]),
                        "cluster_connect_distance": float(table_cfg["cluster_connect_distance"]),
                        "min_cell_hits": int(table_cfg["min_cell_hits"]),
                        "min_cluster_hits": int(table_cfg["min_cluster_hits"]),
                        "max_leg_diameter": float(table_cfg["max_leg_diameter"]),
                        "expected_pair_spacing": float(table_cfg["expected_pair_spacing_m"]),
                        "pair_spacing_tolerance": float(table_cfg["pair_spacing_tolerance_m"]),
                        "expected_pair_axis_deg": float(table_cfg["expected_pair_axis_deg"]),
                        "pair_axis_tolerance_deg": float(table_cfg["pair_axis_tolerance_deg"]),
                        "expected_pair_midpoint": [x + 0.85, y + 0.58],
                        "max_midpoint_shift": 1.10,
                        "ambiguity_score_margin": float(table_cfg["ambiguity_score_margin"]),
                        "approach_standoff": float(footprint["front_m"]) + float(table_cfg["desired_front_clearance_m"]),
                        "max_approach_shift": 1.80,
                        "max_pair_score": 2.90,
                    }
                    if new_scan:
                        try:
                            clusters, pair = detect_pair(points, dynamic_cfg, (x, y))
                            stable.append(tuple(pair.midpoint))
                            if len(stable) == stable.maxlen:
                                spread = max(
                                    math.hypot(item[0] - pair.midpoint[0], item[1] - pair.midpoint[1])
                                    for item in stable
                                )
                                if spread <= float(table_cfg["stable_midpoint_m"]):
                                    return pair, {
                                        "travelled_while_searching_m": travelled,
                                        "cluster_count": len(clusters),
                                        "stable_spread_m": spread,
                                        "pair_midpoint_ref_m": list(pair.midpoint),
                                        "approach_ref_m": list(pair.approach[:2]),
                                        "centre_standoff_m": dynamic_cfg["approach_standoff"],
                                    }
                            last_error = "candidate not yet temporally stable"
                        except DetectionError as exc:
                            stable.clear()
                            last_error = str(exc)

                    if abs(relative_yaw) > math.radians(float(motion["max_heading_error_deg"])):
                        raise MissionAbort("heading drift while searching for table legs")
                    vx_ref = float(motion["table_search_speed_mps"])
                    vy_ref = clamp(0.8 * (start_y - y), -0.015, 0.015)
                    frame_delta = wrap(reference.yaw - state.yaw)
                    vx = math.cos(frame_delta) * vx_ref - math.sin(frame_delta) * vy_ref
                    vy = math.sin(frame_delta) * vx_ref + math.cos(frame_delta) * vy_ref
                    decision = self._guard_velocity(vx, vy)
                    self.send(vx * decision.speed_scale, vy * decision.speed_scale, clamp(-1.0 * relative_yaw, -0.05, 0.05))
                    if x > best_x + 0.010:
                        best_x = x
                        progress_at = time.monotonic()
                    elif time.monotonic() - progress_at > float(motion["no_progress_timeout_s"]):
                        raise MissionAbort("no odometry progress during table-leg search")
                raise MissionAbort("table-leg search timeout")
            finally:
                self.stop()

    result: dict[str, Any] = {
        "status": "running",
        "states": [],
        "collision_guard_enabled": os.environ.get("TMR_CYCLE_DISABLE_COLLISION_GUARD") != "1",
    }
    rclpy.init()
    node = ContinuousRouteNode()
    exit_code = 1
    try:
        node.set_state(RouteState.WAIT_READY)
        result["interfaces"] = node.wait_ready()
        node.acquire_control_lease()
        start = node.wait_stationary()

        node.set_state(RouteState.INITIAL_FORWARD)
        result["states"].append(RouteState.INITIAL_FORWARD.value)
        if os.environ.get("TMR_CYCLE_SKIP_INITIAL_FORWARD") == "1":
            result["initial_forward"] = {"skipped": True, "reason": "resume after completed segment"}
        else:
            result["initial_forward"] = node.move_to_reference(
                start,
                (float(cfg["mission"]["initial_forward_m"]), 0.0),
                "initial_forward",
                float(motion["initial_speed_mps"]),
                axis="x",
            )

        node.set_state(RouteState.TURN_CW90)
        result["states"].append(RouteState.TURN_CW90.value)
        if os.environ.get("TMR_CYCLE_SKIP_TURN") == "1":
            result["turn"] = {"skipped": True, "reason": "resume after completed segment"}
        else:
            result["turn"] = node.turn_right_90()
        correction_deg = float(os.environ.get("TMR_CYCLE_RESUME_YAW_CORRECTION_DEG", "0"))
        if abs(correction_deg) >= 0.1:
            result["resume_yaw_correction"] = node.turn_relative_deg(correction_deg)

        node.set_state(RouteState.ACQUIRE_DOOR)
        result["states"].append(RouteState.ACQUIRE_DOOR.value)
        door_reference, gap, stability = node.detect_stable_door()
        result["door_lidar"] = gap_core.gap_as_dict(gap)
        result["door_stability"] = stability
        result["door_runtime_map"] = node.runtime_map_report(door_reference, gap)
        targets = compute_door_targets(
            gap,
            float(footprint["front_m"]),
            float(footprint["rear_m"]),
            0.5 * float(footprint["width_m"]),
            float(door_cfg["predoor_clearance_m"]),
            float(door_cfg["postdoor_clearance_m"]),
            float(door_cfg["side_margin_m"]),
        )
        result["door_targets_ref_m"] = asdict(targets)
        fixed_targets = compute_fixed_door_route_targets(
            tuple(gap.midpoint),
            targets.predoor,
            targets.postdoor,
            float(cfg["mission"]["before_door_m"]),
            float(cfg["mission"]["forward_from_before_door_m"]),
        )
        result["fixed_door_route_targets_ref_m"] = asdict(fixed_targets)

        # User-confirmed order: strafe first, then advance.  No fixed 0.62 m
        # staging distance and no old hard-coded gap coordinates are used.
        node.set_state(RouteState.ALIGN_TO_MIDPOINT)
        result["states"].append(RouteState.ALIGN_TO_MIDPOINT.value)
        tangent_y = float(gap.tangent[1])
        if abs(tangent_y) < 0.70:
            raise MissionAbort("door tangent is too oblique for a lateral-first fixed-heading entry")
        lateral_target = (0.0, targets.centre_tangent_coordinate / tangent_y)
        result["door_lateral_align"] = node.move_to_reference(
            door_reference,
            lateral_target,
            "door_lateral_align_first",
            float(motion["align_speed_mps"]),
            axis="y",
        )
        result["door_predoor_advance"] = node.move_to_reference(
            door_reference,
            fixed_targets.half_metre_before_door,
            "advance_to_half_metre_before_door",
            float(motion["door_speed_mps"]),
            axis=None,
        )

        node.set_state(RouteState.VERIFY_CORRIDOR)
        result["states"].append(RouteState.VERIFY_CORRIDOR.value)
        # A zero command plus a fresh scan is the final bounded check.  It does
        # not start a second long mapping pass.
        node.stop()
        node._fresh_odom()
        node._fresh_scans()
        result["predoor_live_guard"] = asdict(
            node._guard_velocity(float(motion["door_speed_mps"]), 0.0)
        )

        node.set_state(RouteState.CROSS_DOOR)
        result["states"].append(RouteState.CROSS_DOOR.value)
        result["door_crossing"] = node.move_to_reference(
            door_reference,
            fixed_targets.final_after_forward,
            "forward_1p2_from_half_metre_before_door",
            float(motion["door_speed_mps"]),
            axis=None,
            door_gap=gap,
            door_targets=targets,
        )

        node.set_state(RouteState.FINAL_STOP)
        result["states"].append(RouteState.FINAL_STOP.value)
        node.stop()
        # Do not hand control to the arm merely because the final odometry
        # target returned.  Require a bounded stationary window while the
        # exclusive lease and zero command are still actively held.
        settled = node.wait_stationary(timeout_s=4.0)
        node.assert_control_ownership(force=True)
        node.stop()
        zero_latched = all(abs(float(value)) <= 1e-9 for value in node._command)
        if not zero_latched or not node._control_lease_acquired:
            raise MissionAbort("final zero-speed lease was not retained")
        result["final_stationary"] = {
            "confirmed": True,
            "x_m": float(settled.x),
            "y_m": float(settled.y),
            "yaw_rad": float(settled.yaw),
            "vx_mps": float(settled.vx),
            "vy_mps": float(settled.vy),
            "wz_rps": float(settled.wz),
        }
        result["zero_command_latched"] = True
        result["control_lease_held"] = True
        result["status"] = "success"
        result["final_state"] = RouteState.FINAL_STOP.value
        exit_code = 0
    except KeyboardInterrupt:
        node.latch_abort("operator interrupt")
        result["status"] = "interrupted"
        result["error"] = "operator interrupt"
    except BaseException as exc:
        try:
            node.latch_abort(f"{type(exc).__name__}: {exc}")
        except BaseException:
            pass
        result["status"] = "aborted"
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["final_state"] = RouteState.ABORT.value
    finally:
        try:
            if exit_code != 0:
                node.latch_abort(result.get("error", "mission did not complete"))
            else:
                node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return exit_code, result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable ROS connections and base commands; omitted means local config check only",
    )
    parser.add_argument(
        "--disable-collision-guard",
        action="store_true",
        help=(
            "disable runtime LiDAR/map/rotation/door-side collision decisions; "
            "odometry, ownership, freshness, timeout and progress guards remain active"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = _load_config(args.config.resolve())
    if not args.execute:
        print(json.dumps(_dry_run_summary(cfg, args.config.resolve()), ensure_ascii=False, indent=2))
        return 0
    if args.disable_collision_guard:
        os.environ["TMR_CYCLE_DISABLE_COLLISION_GUARD"] = "1"
    exit_code, result = run_ros_mission(cfg)
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
