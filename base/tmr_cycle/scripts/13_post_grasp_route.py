#!/usr/bin/env python3
"""Run the confirmed post-grasp base route in one odometry-closed-loop process.

The controller deliberately has no LiDAR or map collision veto.  It keeps one
velocity publisher and one lease for the complete route so phase hand-offs
cannot replay a turn or leave a stale non-zero command behind.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass(frozen=True)
class Stage:
    name: str
    kind: str
    first: float
    second: float = 0.0


def confirmed_stages(
    retreat_m: float,
    turn_deg: float,
    backward_m: float,
    right_first_m: float,
    right_second_m: float,
    include_right: bool = True,
) -> list[Stage]:
    stages = [
        Stage("RETREAT_TO_PREDOOR", "translate", -retreat_m, 0.0),
        Stage("TURN_CCW180", "rotate", math.radians(turn_deg)),
        Stage("BACKWARD_AFTER_TURN", "translate", -backward_m, 0.0),
    ]
    if include_right:
        stages.extend([
            Stage("RIGHT_STAGE_1", "translate", 0.0, -right_first_m),
            Stage("RIGHT_STAGE_2", "translate", 0.0, -right_second_m),
        ])
    return stages


class RouteController(Node):
    def __init__(self, linear_speed: float, angular_speed: float) -> None:
        super().__init__("tmr_post_grasp_route")
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.pose: tuple[float, float, float] | None = None
        self.pose_at = 0.0
        self.last_yaw: float | None = None
        self.unwrapped_yaw: float | None = None
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(TwistStamped, COMMAND_TOPIC, 10)
        self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, message: Odometry) -> None:
        yaw = yaw_of(message.pose.pose.orientation)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            assert self.unwrapped_yaw is not None
            self.unwrapped_yaw += wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        p = message.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw)
        self.pose_at = time.monotonic()

    def publish(self, vx: float, vy: float, wz: float) -> None:
        lease = Bool()
        lease.data = True
        self.lease_pub.publish(lease)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.command_pub.publish(message)

    def stop(self, samples: int = 18) -> None:
        self.command[:] = (0.0, 0.0, 0.0)
        for _ in range(samples):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.pose is not None
                and time.monotonic() - self.pose_at <= 0.40
                and self.command_pub.get_subscription_count() == 1
            ):
                self.stop(8)
                return
        raise RuntimeError("fresh odometry or exclusive velocity adapter unavailable")

    def _tick(self, desired: tuple[float, float, float], last_tick: float) -> float:
        now = time.monotonic()
        dt = clamp(now - last_tick, 0.02, 0.10)
        for index, (target, accel) in enumerate(zip(desired, (0.16, 0.16, 0.28))):
            step = accel * dt
            self.command[index] += clamp(target - self.command[index], -step, step)
        self.publish(*self.command)
        return now

    def _fresh_pose(self) -> tuple[float, float, float]:
        if self.pose is None or time.monotonic() - self.pose_at > 0.40:
            raise RuntimeError("odometry became stale")
        return self.pose

    def translate(self, forward_m: float, left_m: float, timeout_s: float) -> dict:
        start_x, start_y, start_yaw = self._fresh_pose()
        forward = (math.cos(start_yaw), math.sin(start_yaw))
        left = (-math.sin(start_yaw), math.cos(start_yaw))
        target_x = start_x + forward_m * forward[0] + left_m * left[0]
        target_y = start_y + forward_m * forward[1] + left_m * left[1]
        deadline = time.monotonic() + timeout_s
        stable_since = None
        last_tick = time.monotonic()
        best_error = math.inf
        progress_at = time.monotonic()
        next_report = 0.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                x, y, yaw = self._fresh_pose()
                ex, ey = target_x - x, target_y - y
                position_error = math.hypot(ex, ey)
                yaw_error = wrap(start_yaw - yaw)
                if position_error <= 0.018 and abs(yaw_error) <= math.radians(1.0):
                    self.publish(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.20:
                        break
                    continue
                stable_since = None
                if position_error < best_error - 0.006:
                    best_error = position_error
                    progress_at = time.monotonic()
                elif time.monotonic() - progress_at > 4.0:
                    raise RuntimeError("translation made no odometry progress for 4 s")
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                desired = (
                    clamp(0.85 * body_x, -self.linear_speed, self.linear_speed),
                    clamp(0.85 * body_y, -self.linear_speed, self.linear_speed),
                    clamp(1.20 * yaw_error, -0.08, 0.08),
                )
                last_tick = self._tick(desired, last_tick)
                if last_tick >= next_report:
                    print(json.dumps({"event": "translation_progress", "remaining_m": position_error}), flush=True)
                    next_report = last_tick + 1.0
            else:
                raise TimeoutError("translation stage timed out")
        finally:
            self.stop()
        end_x, end_y, end_yaw = self._fresh_pose()
        actual_forward = (end_x - start_x) * forward[0] + (end_y - start_y) * forward[1]
        actual_left = (end_x - start_x) * left[0] + (end_y - start_y) * left[1]
        error = math.hypot(actual_forward - forward_m, actual_left - left_m)
        if error > 0.05:
            raise RuntimeError(f"translation endpoint error {error:.3f} m exceeds 0.05 m")
        return {
            "requested_forward_m": forward_m,
            "requested_left_m": left_m,
            "actual_forward_m": actual_forward,
            "actual_left_m": actual_left,
            "endpoint_error_m": error,
            "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
            "start_odom": [start_x, start_y, start_yaw],
            "end_odom": [end_x, end_y, end_yaw],
        }

    def rotate_ccw(self, radians: float, timeout_s: float) -> dict:
        start_x, start_y, start_yaw = self._fresh_pose()
        assert self.unwrapped_yaw is not None
        start_unwrapped = self.unwrapped_yaw
        target = start_unwrapped + radians
        deadline = time.monotonic() + timeout_s
        stable_since = None
        last_tick = time.monotonic()
        best_error = math.inf
        progress_at = time.monotonic()
        next_report = 0.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                x, y, yaw = self._fresh_pose()
                assert self.unwrapped_yaw is not None
                error = target - self.unwrapped_yaw
                if abs(error) <= math.radians(0.8):
                    self.publish(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.20:
                        break
                    continue
                stable_since = None
                if abs(error) < best_error - math.radians(0.8):
                    best_error = abs(error)
                    progress_at = time.monotonic()
                elif time.monotonic() - progress_at > 4.0:
                    raise RuntimeError("rotation made no odometry progress for 4 s")
                world_ex, world_ey = start_x - x, start_y - y
                desired = (
                    clamp(math.cos(yaw) * world_ex + math.sin(yaw) * world_ey, -0.018, 0.018),
                    clamp(-math.sin(yaw) * world_ex + math.cos(yaw) * world_ey, -0.018, 0.018),
                    clamp(1.15 * error, -self.angular_speed, self.angular_speed),
                )
                last_tick = self._tick(desired, last_tick)
                if last_tick >= next_report:
                    print(json.dumps({"event": "rotation_progress", "remaining_deg": math.degrees(error)}), flush=True)
                    next_report = last_tick + 1.0
            else:
                raise TimeoutError("rotation stage timed out")
        finally:
            self.stop()
        end_x, end_y, end_yaw = self._fresh_pose()
        assert self.unwrapped_yaw is not None
        actual = self.unwrapped_yaw - start_unwrapped
        angle_error = math.degrees(actual - radians)
        drift = math.hypot(end_x - start_x, end_y - start_y)
        if abs(angle_error) > 2.0 or drift > 0.06:
            raise RuntimeError(f"rotation endpoint error={angle_error:.2f} deg drift={drift:.3f} m")
        return {
            "requested_ccw_deg": math.degrees(radians),
            "actual_ccw_deg": math.degrees(actual),
            "error_deg": angle_error,
            "position_drift_m": drift,
            "start_odom": [start_x, start_y, start_yaw],
            "end_odom": [end_x, end_y, end_yaw],
        }


def write_state(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start", action="store_true")
    parser.add_argument("--retreat-m", type=float, default=1.70)
    parser.add_argument("--turn-deg", type=float, default=180.0)
    parser.add_argument("--backward-m", type=float, default=0.25)
    parser.add_argument("--right-first-m", type=float, default=0.80)
    parser.add_argument("--right-second-m", type=float, default=0.85)
    parser.add_argument(
        "--stop-before-right",
        action="store_true",
        help="run the verified retreat/turn/backward prefix and leave right motion to letter search",
    )
    parser.add_argument("--linear-speed-mps", type=float, default=0.08)
    parser.add_argument("--angular-speed-rps", type=float, default=0.18)
    parser.add_argument("--state-file", type=Path, default=Path("~/tmr_cycle/state/post_grasp_route.json").expanduser())
    args = parser.parse_args()
    stages = confirmed_stages(
        args.retreat_m,
        args.turn_deg,
        args.backward_m,
        args.right_first_m,
        args.right_second_m,
        include_right=not args.stop_before_right,
    )
    summary = {
        "collision_guard": "disabled_by_design",
        "stages": [{"name": item.name, "kind": item.kind, "values": [item.first, item.second]} for item in stages],
    }
    if not args.execute:
        print(json.dumps({"status": "dry_run", **summary}, indent=2), flush=True)
        return 0
    if args.state_file.exists() and not args.fresh_start:
        raise RuntimeError(f"state file exists; refuse accidental phase replay: {args.state_file}")

    state = {"status": "running", "next_stage": 0, "reports": [], **summary}
    write_state(args.state_file, state)
    rclpy.init()
    node = None
    try:
        node = RouteController(args.linear_speed_mps, args.angular_speed_rps)
        node.wait_ready()
        for index, stage in enumerate(stages):
            state["active_stage"] = stage.name
            write_state(args.state_file, state)
            print(json.dumps({"event": "stage_start", "index": index, "name": stage.name}), flush=True)
            if stage.kind == "translate":
                distance = math.hypot(stage.first, stage.second)
                timeout = max(15.0, distance / args.linear_speed_mps + 15.0)
                report = node.translate(stage.first, stage.second, timeout)
            else:
                timeout = max(20.0, abs(stage.first) / args.angular_speed_rps + 12.0)
                report = node.rotate_ccw(stage.first, timeout)
            state["reports"].append({"stage": stage.name, **report})
            state["next_stage"] = index + 1
            state.pop("active_stage", None)
            write_state(args.state_file, state)
            print(json.dumps({"event": "stage_complete", "index": index, "name": stage.name, "report": report}), flush=True)
        node.stop(30)
        state["status"] = "complete"
        state["zero_command_latched"] = True
        write_state(args.state_file, state)
        print(json.dumps(state, indent=2), flush=True)
        return 0
    except BaseException as exc:
        if node is not None:
            node.stop(30)
        state["status"] = "failed"
        state["error"] = repr(exc)
        state["zero_command_latched"] = True
        write_state(args.state_file, state)
        print(json.dumps(state, indent=2), flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
