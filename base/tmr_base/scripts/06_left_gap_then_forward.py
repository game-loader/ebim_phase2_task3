#!/usr/bin/env python3
"""Move left through the operator-confirmed wide opening, then forward."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("tmr_left_gap_then_forward")
        self.pose: tuple[float, float, float] | None = None
        self.odom_at = 0.0
        self.publisher = self.create_publisher(
            TwistStamped, "/swerve_drive_controller/cmd_vel", 10
        )
        self.create_subscription(
            Odometry,
            "/swerve_drive_controller/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (
            float(p.x),
            float(p.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.odom_at = time.monotonic()

    def send(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.publisher.publish(msg)

    def stop(self) -> None:
        for _ in range(20):
            self.send(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_pose(self, timeout: float = 5.0) -> tuple[float, float, float]:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.pose is not None:
                return self.pose
        raise RuntimeError("no odometry")

    def move_to(
        self,
        label: str,
        target_x: float,
        target_y: float,
        fixed_yaw: float,
        primary: str,
        speed: float,
        timeout: float,
    ) -> dict:
        start = self.wait_pose()
        deadline = time.monotonic() + timeout
        command = [0.0, 0.0, 0.0]
        last_tick = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.odom_at > 0.6:
                    raise RuntimeError("odometry stale")
                x, y, yaw = self.pose
                ex, ey = target_x - x, target_y - y
                yaw_error = wrap(fixed_yaw - yaw)
                if math.hypot(ex, ey) <= 0.018 and abs(yaw_error) <= math.radians(1.2):
                    break
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                if primary == "left":
                    desired = (
                        clamp(0.85 * body_x, -0.035, 0.035),
                        clamp(0.85 * body_y, -speed, speed),
                        clamp(1.1 * yaw_error, -0.08, 0.08),
                    )
                else:
                    desired = (
                        clamp(0.85 * body_x, -speed, speed),
                        clamp(0.85 * body_y, -0.035, 0.035),
                        clamp(1.1 * yaw_error, -0.08, 0.08),
                    )
                now = time.monotonic()
                dt = clamp(now - last_tick, 0.02, 0.10)
                last_tick = now
                for i, accel in enumerate((0.22, 0.22, 0.28)):
                    command[i] += clamp(desired[i] - command[i], -accel * dt, accel * dt)
                self.send(*command)
            else:
                raise TimeoutError(f"{label} timeout")
        finally:
            self.stop()
        assert self.pose is not None
        return {
            "label": label,
            "start_odom": list(start),
            "target_odom": [target_x, target_y, fixed_yaw],
            "end_odom": list(self.pose),
            "error_m": math.hypot(target_x - self.pose[0], target_y - self.pose[1]),
            "yaw_error_deg": math.degrees(wrap(self.pose[2] - fixed_yaw)),
        }

    def run(self, left_m: float, forward_m: float, speed: float) -> dict:
        start_x, start_y, yaw = self.wait_pose()
        left_target = (
            start_x - math.sin(yaw) * left_m,
            start_y + math.cos(yaw) * left_m,
        )
        first = self.move_to(
            "left_through_wide_opening",
            left_target[0], left_target[1], yaw, "left", speed,
            max(30.0, abs(left_m) / speed + 15.0),
        )
        forward_target = (
            left_target[0] + math.cos(yaw) * forward_m,
            left_target[1] + math.sin(yaw) * forward_m,
        )
        second = self.move_to(
            "forward_after_opening",
            forward_target[0], forward_target[1], yaw, "forward", speed,
            max(30.0, abs(forward_m) / speed + 15.0),
        )
        return {"status": "success", "segments": [first, second]}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-m", type=float, default=0.62)
    parser.add_argument("--forward-m", type=float, default=0.85)
    parser.add_argument("--speed-mps", type=float, default=0.11)
    args = parser.parse_args()
    rclpy.init()
    node = Controller()
    result: dict = {"status": "running"}
    code = 1
    try:
        result = node.run(args.left_m, args.forward_m, args.speed_mps)
        code = 0
    except BaseException as exc:
        result = {"status": "failed", "error": f"{type(exc).__name__}: {exc}"}
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
