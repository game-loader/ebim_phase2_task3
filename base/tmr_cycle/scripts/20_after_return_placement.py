#!/usr/bin/env python3
"""Insert a placement detour after 15; stop for externally performed placement.

outbound: left 0.85 m, align base front with live far leg, CW90, stop.
return: retrace recorded waypoints, then optionally run the existing script 16.
No arm policy is launched. No motion or ROS initialization without --execute.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import importlib.util
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

from far_leg_target import driver_session, read_config
from route_process import spawn_route_child, signal_route_child


ROOT = Path(__file__).resolve().parents[1]
POSITION_TOLERANCE = 0.06
YAW_TOLERANCE = math.radians(3.0)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def checked_pose(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("expected an odometry pose [x, y, yaw]")
    pose = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in pose):
        raise ValueError("odometry pose must be finite")
    return pose


def assert_at_pose(actual, expected):
    actual, expected = checked_pose(actual), checked_pose(expected)
    distance = math.hypot(actual[0] - expected[0], actual[1] - expected[1])
    angle = abs(wrap(actual[2] - expected[2]))
    if distance > POSITION_TOLERANCE or angle > YAW_TOLERANCE:
        raise RuntimeError(
            f"checkpoint mismatch: {distance:.3f} m, {math.degrees(angle):.2f} deg; "
            "do not restart odometry or move the base between stages"
        )


def relative_target(current, target):
    x, y, yaw = checked_pose(current)
    tx, ty, _ = checked_pose(target)
    dx, dy = tx - x, ty - y
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(
        yaw
    ) * dy


def load_after15(path):
    value = json.loads(path.read_text(encoding="utf-8"))
    door = value.get("door_report", {})
    stationary = door.get("final_stationary", {})
    if not (
        value.get("status") == "complete"
        and value.get("phase") == "COMPLETE"
        and value.get("zero_command_latched") is True
        and door.get("status") == "success"
        and door.get("final_state") == "FINAL_STOP"
        and door.get("zero_command_latched") is True
        and stationary.get("confirmed") is True
    ):
        raise ValueError("script 15 must have a successful, stationary COMPLETE result")
    pose = checked_pose([stationary[k] for k in ("x_m", "y_m", "yaw_rad")])
    return pose, door.get("interfaces", {}).get("odom_frame", "")


def write_state(path, state):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(state, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


@contextmanager
def mission_lock():
    import fcntl

    # Shared with script 15. The command adapter remains the only controller publisher.
    with open("/tmp/tmr_letter_return.lock", "w") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def mark(path, state, phase):
    state["phase"] = phase
    state["zero_command_latched"] = False
    write_state(path, state)
    print(json.dumps({"phase": phase}), flush=True)


def outbound(node, state, path):
    node.wait_stationary()
    assert_at_pose(node._fresh_pose(), state["after15_pose"])
    if state.get("roi_origin") is None:
        state["roi_origin"] = {
            "reference": "current_pose",
            "pose_odom": list(node._fresh_pose()),
            "odom_frame": node.odom_frame,
            "driver_session": state["driver_session"],
            "stationary_confirmed": True,
        }
    node.prepare_leg_approach(state["roi_origin"], state["leg_config"])
    state["origin"] = list(node._fresh_pose())
    mark(path, state, "LEFT_0P70")
    state["reports"].append(node.translate(0.0, 0.85, 30.0))
    state["after_left"] = list(node._fresh_pose())
    mark(path, state, "APPROACH_FAR_LEG_FRONT_ALIGNMENT")
    state["leg_approach"] = node.approach_far_leg()
    state["reports"].append(state["leg_approach"])
    state["before_turn"] = list(node._fresh_pose())
    mark(path, state, "TURN_CW90")
    state["reports"].append(node.rotate_ccw(-math.pi / 2, 30.0))
    node.wait_stationary()
    state.update(
        status="awaiting_placement",
        phase="WAITING_FOR_PLACEMENT",
        placement_pose=list(node._fresh_pose()),
        zero_command_latched=True,
        placement_completed=False,
        arm_policy_started=False,
    )
    write_state(path, state)


def return_to_pickup(node, state, path):
    node.wait_stationary()
    assert_at_pose(node._fresh_pose(), state["placement_pose"])
    state["placement_completed"] = True
    state["status"] = "returning"
    mark(path, state, "RESTORE_PRETURN_HEADING")
    angle = wrap(state["before_turn"][2] - node._fresh_pose()[2])
    state["reports"].append(node.rotate_ccw(angle, 30.0))
    for phase, key in (
        ("RETRACE_FORWARD_SEGMENT", "after_left"),
        ("RETRACE_LEFT_SEGMENT", "origin"),
    ):
        mark(path, state, phase)
        forward, left = relative_target(node._fresh_pose(), state[key])
        timeout = math.hypot(forward, left) / node.linear_speed + 20.0
        state["reports"].append(node.translate(forward, left, timeout))
    angle = wrap(state["origin"][2] - node._fresh_pose()[2])
    if abs(angle) > math.radians(0.8):
        state["reports"].append(node.rotate_ccw(angle, 15.0))
    node.wait_stationary()
    assert_at_pose(node._fresh_pose(), state["origin"])
    state.update(
        status="at_pickup",
        phase="AT_PICKUP_AFTER_PLACEMENT",
        return_pose=list(node._fresh_pose()),
        zero_command_latched=True,
    )
    write_state(path, state)


def load_motion():
    name = "tmr_post15_motion"
    spec = importlib.util.spec_from_file_location(
        name, ROOT / "scripts/13_post_grasp_route.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def create_node(expected_frame, expected_session):
    import rclpy

    motion = load_motion()

    class PlacementController(motion.RouteController):
        def __init__(self):
            self.odom_frame = ""
            self.odom_fault = None
            self.velocity = None
            self.odom_history = deque(maxlen=2000)
            self.last_odom_stamp = 0.0
            self.next_session_check = 0.0
            super().__init__(0.08, 0.18)

        def _on_odom(self, message):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if stamp < self.last_odom_stamp:
                self.odom_fault = "odometry timestamp moved backwards"
                return
            if stamp == self.last_odom_stamp:
                return
            if not 0 <= now - stamp <= 0.30:
                self.odom_fault = "raw odometry timestamp is stale or in future"
                return
            self.last_odom_stamp = stamp
            frame = message.header.frame_id.lstrip("/")
            # Installed SwerveDriveController omits child_frame_id in nav odometry;
            # its source uses the same base_link pose for nav odometry and TF.
            if not frame or message.child_frame_id.lstrip("/") not in ("", "base_link"):
                self.odom_fault = "raw odometry must describe base_link"
            if self.odom_frame and frame != self.odom_frame:
                self.odom_fault = "odometry frame changed"
            self.odom_frame = frame
            position = message.pose.pose.position
            q = message.pose.pose.orientation
            norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
            if not math.isfinite(norm) or abs(norm - 1) > 0.02:
                self.odom_fault = "invalid odometry quaternion"
                return
            values = (
                position.x,
                position.y,
                motion.yaw_of(message.pose.pose.orientation),
            )
            if not all(math.isfinite(v) for v in values):
                self.odom_fault = "invalid odometry pose"
                return
            if self.pose is not None:
                dx, dy = values[0] - self.pose[0], values[1] - self.pose[1]
                if math.hypot(dx, dy) > 0.20 or abs(
                    wrap(values[2] - self.pose[2])
                ) > math.radians(25):
                    self.odom_fault = "odometry jumped; possible reset"
            super()._on_odom(message)
            self.odom_history.append((stamp, values))
            twist = message.twist.twist
            self.velocity = (twist.linear.x, twist.linear.y, twist.angular.z)

        def _fresh_pose(self):
            if self.odom_fault:
                raise RuntimeError(self.odom_fault)
            if expected_frame and self.odom_frame != expected_frame.lstrip("/"):
                raise RuntimeError("odometry frame differs from script 15/checkpoint")
            if time.monotonic() >= self.next_session_check:
                if driver_session() != expected_session:
                    raise RuntimeError(
                        "base driver session changed; odometry invalidated"
                    )
                self.next_session_check = time.monotonic() + 0.5
            return super()._fresh_pose()

        def _tick(self, desired, last_tick):
            self._fresh_pose()
            if self.command_pub.get_subscription_count() != 1:
                raise RuntimeError("exclusive velocity adapter unavailable")
            if self.count_publishers(motion.COMMAND_TOPIC) != 1:
                raise RuntimeError("another mission velocity publisher is present")
            return super()._tick(desired, last_tick)

        def spin(self):
            rclpy.spin_once(self, timeout_sec=0.025)

        def prepare_leg_approach(self, origin, cfg):
            from far_leg_target import DetectionError, alignment_error, in_reference
            from live_leg_approach import LiveLegSensors

            if origin["odom_frame"] != self.odom_frame:
                raise RuntimeError("green START and current odometry frames differ")
            alignment_error(
                in_reference(self._fresh_pose(), origin["pose_odom"]), {"x": 0.0}, cfg
            )
            self.leg_sensors = LiveLegSensors(self, origin, cfg)
            deadline = time.monotonic() + cfg["acquire_timeout_s"]
            while time.monotonic() < deadline:
                self.stop(1)
                self._fresh_pose()
                try:
                    self.leg_sensors.healthy()
                    return
                except DetectionError:
                    continue
            raise TimeoutError("both raw LiDAR streams required before left motion")

        def approach_far_leg(self):
            from live_leg_approach import approach_far_leg

            return approach_far_leg(self, self.leg_sensors, self.leg_sensors.cfg)

        def wait_stationary(self):
            deadline = time.monotonic() + 4.0
            settled_at = None
            previous = self._fresh_pose()
            while time.monotonic() < deadline:
                if self.command_pub.get_subscription_count() != 1:
                    raise RuntimeError(
                        "velocity adapter unavailable during stationary check"
                    )
                self.stop(1)
                pose = self._fresh_pose()
                speed = self.velocity
                still = (
                    speed is not None
                    and all(math.isfinite(v) for v in speed)
                    and math.hypot(speed[0], speed[1]) < 0.01
                    and abs(speed[2]) < 0.02
                    and math.hypot(pose[0] - previous[0], pose[1] - previous[1]) < 0.005
                    and abs(wrap(pose[2] - previous[2])) < math.radians(0.5)
                )
                settled_at = (settled_at or time.monotonic()) if still else None
                if settled_at is not None and time.monotonic() - settled_at >= 0.35:
                    return
                previous = pose
            raise RuntimeError("base did not confirm stationary")

    rclpy.init()
    return PlacementController()


def run_legacy_return(path, state):
    child_state = path.with_name(path.stem + "_legacy_start.json")
    if child_state.exists():
        raise RuntimeError("legacy return checkpoint already exists; refuse replay")
    mark(path, state, "LEGACY_RETURN_RUNNING")
    command = [
        sys.executable,
        str(ROOT / "scripts/16_return_pickup_to_start.py"),
        "--execute",
        "--state-file",
        str(child_state),
    ]
    child = spawn_route_child(command)
    try:
        code = child.wait(timeout=150)
    except BaseException:
        if child.poll() is not None:
            raise
        signal_route_child(child, signal.SIGINT)
        try:
            child.wait(timeout=6)
        except subprocess.TimeoutExpired:
            signal_route_child(child, signal.SIGTERM)
            child.wait(timeout=3)
        raise
    report = (
        json.loads(child_state.read_text(encoding="utf-8"))
        if child_state.exists()
        else {}
    )
    state["legacy_return_report"] = report
    if (
        code != 0
        or report.get("status") != "complete"
        or report.get("zero_command_latched") is not True
    ):
        raise RuntimeError("existing script 16 did not complete")
    state.update(
        status="complete",
        phase="LEGACY_RETURN_COMPLETE",
        zero_command_latched=True,
        original_start_pose_verified=False,
    )
    write_state(path, state)


def execute(args):
    path = args.state_file.resolve()
    if args.phase == "outbound":
        if path.exists():
            raise RuntimeError(
                "placement checkpoint exists; refuse accidental outbound replay"
            )
        pose, frame = load_after15(args.after15_state)
        state = dict(
            version=2,
            status="starting",
            phase="CREATED",
            after15_pose=list(pose),
            odom_frame=frame,
            after15_state=str(args.after15_state.resolve()),
            reports=[],
            collision_guard="disabled",
            table_detection="dual_lidar_live_far_leg",
        )
        session = driver_session()
        state["driver_session"] = session
        state["leg_config"] = read_config(args.leg_config)
        # The ROI is referenced to wherever the base stands now, exactly like the
        # earlier table-leg stages. Filled in from the live pose once the node is
        # up, because no odometry is available before that.
        state["roi_origin"] = None
    else:
        if not args.placement_complete:
            raise ValueError(
                "return requires --placement-complete after the external placement"
            )
        state = json.loads(path.read_text(encoding="utf-8"))
        if state.get("version") not in (1, 2) or state.get("phase") not in (
            "WAITING_FOR_PLACEMENT",
            "AT_PICKUP_AFTER_PLACEMENT",
        ):
            raise RuntimeError(
                "checkpoint is not ready for return; refuse partial-stage replay"
            )
        for key in ("origin", "after_left", "before_turn", "placement_pose"):
            checked_pose(state[key])
        frame = state["odom_frame"]
        session = driver_session()
        if state.get("version") == 2 and state.get("driver_session") != session:
            raise RuntimeError(
                "base driver restarted after outbound; saved return poses invalid"
            )

    node = None
    try:
        node = create_node(frame, session)
        node.wait_ready()
        if not frame:
            state["odom_frame"] = node.odom_frame
        if args.phase == "outbound":
            outbound(node, state, path)
        elif state["phase"] == "WAITING_FOR_PLACEMENT":
            return_to_pickup(node, state, path)
        else:
            node.wait_stationary()
            assert_at_pose(node._fresh_pose(), state["origin"])
        node.stop(30)
        node.destroy_node()
        node = None
        import rclpy

        rclpy.shutdown()
        if args.phase == "return" and not args.stop_at_pickup:
            run_legacy_return(path, state)
        return state
    except BaseException as exc:
        state.update(status="failed", error=repr(exc), zero_command_latched=False)
        if node is not None:
            try:
                node.stop(30)
                state["zero_command_sent"] = True
            except BaseException:
                state["zero_command_sent"] = False
        # Preserve last phase; an interrupted motion is never silently resumed.
        state["failed_phase"] = state["phase"]
        state["phase"] = "FAILED"
        write_state(path, state)
        raise
    finally:
        if node is not None:
            node.destroy_node()
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("outbound", "return"))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--roi-origin-file", type=Path, default=ROOT / "state/leg_roi_origin.json"
    )
    parser.add_argument(
        "--leg-config", type=Path, default=ROOT / "config/post15_leg_approach.json"
    )
    parser.add_argument(
        "--after15-state", type=Path, default=ROOT / "state/return_from_letter.json"
    )
    parser.add_argument(
        "--state-file", type=Path, default=ROOT / "state/post15_placement.json"
    )
    parser.add_argument("--placement-complete", action="store_true")
    parser.add_argument(
        "--stop-at-pickup",
        action="store_true",
        help="return only to the script 15 endpoint",
    )
    args = parser.parse_args()
    if args.phase == "outbound" and (args.placement_complete or args.stop_at_pickup):
        parser.error("placement-complete/stop-at-pickup apply only to return")
    if not args.execute:
        print(
            json.dumps(
                dict(
                    status="dry_run",
                    motion_enabled=False,
                    collision_guard="disabled",
                    outbound=[
                        "left 0.85 m",
                        "forward until base front aligns with max-x live leg",
                        "CW 90 deg",
                        "STOP; await external placement",
                    ],
                    return_route=[
                        "restore preturn heading",
                        "retrace forward segment",
                        "retrace left segment",
                        "STOP at script 15 endpoint",
                    ]
                    + ([] if args.stop_at_pickup else ["run existing script 16"]),
                    legacy_start_pose_verified=False,
                    table_detection="dual_lidar_live_far_leg",
                    roi_reference="current pose at detour start",
                    roi={"x": [1.65, 3.65], "y": [-0.10, 0.60]},
                    front_offset_m=0.40,
                    front_alignment_tolerance_m=0.02,
                ),
                indent=2,
            )
        )
        return 0

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("operator signal")

    signal.signal(signal.SIGTERM, interrupt)
    try:
        with mission_lock():
            result = execute(args)
            print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc)}), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
