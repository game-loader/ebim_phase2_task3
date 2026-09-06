#!/usr/bin/env python3
"""Continuously command the TMR base over a signed odometry-relative distance.

This program only uses existing ROS topics. It creates no files and starts no
robot-side services. Start the installed base controller first, then stream
this local file to ``python3 -`` over SSH if remote files must remain unchanged.
The default task distance is +0.85 m; a negative ``--distance-m`` moves backward.
"""

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


class ForwardController(Node):
    def __init__(self) -> None:
        super().__init__("tmr_relative_translation")
        self.pose: tuple[float, float, float] | None = None
        self.odom_received_at = 0.0
        self.publisher = self.create_publisher(
            TwistStamped,
            "/swerve_drive_controller/cmd_vel",
            10,
        )
        self.create_subscription(
            Odometry,
            "/swerve_drive_controller/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.pose = (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.odom_received_at = time.monotonic()

    def send(self, vx: float, vy: float, wz: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.publisher.publish(message)

    def stop(self, cycles: int = 30) -> None:
        for _ in range(cycles):
            self.send(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)

    def wait_for_odom(self, timeout_sec: float = 6.0) -> tuple[float, float, float]:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                return self.pose
        raise RuntimeError("no odometry received")

    def move_forward(
        self,
        distance_m: float,
        max_speed_mps: float,
        timeout_sec: float,
    ) -> dict:
        start_x, start_y, start_yaw = self.wait_for_odom()
        target_x = start_x + distance_m * math.cos(start_yaw)
        target_y = start_y + distance_m * math.sin(start_yaw)
        command = [0.0, 0.0, 0.0]
        last_tick = time.monotonic()
        next_report = last_tick
        deadline = last_tick + timeout_sec

        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.odom_received_at > 0.5:
                    raise RuntimeError("odometry became stale")

                x, y, yaw = self.pose
                error_x = target_x - x
                error_y = target_y - y
                body_x = math.cos(yaw) * error_x + math.sin(yaw) * error_y
                body_y = -math.sin(yaw) * error_x + math.cos(yaw) * error_y
                yaw_error = wrap(start_yaw - yaw)

                if math.hypot(error_x, error_y) <= 0.018 and abs(yaw_error) <= math.radians(1.0):
                    break

                desired = (
                    clamp(0.75 * body_x, -max_speed_mps, max_speed_mps),
                    clamp(0.75 * body_y, -0.035, 0.035),
                    clamp(1.20 * yaw_error, -0.10, 0.10),
                )
                now = time.monotonic()
                dt = clamp(now - last_tick, 0.02, 0.10)
                last_tick = now
                acceleration_steps = (0.15 * dt, 0.15 * dt, 0.28 * dt)
                for index in range(3):
                    command[index] += clamp(
                        desired[index] - command[index],
                        -acceleration_steps[index],
                        acceleration_steps[index],
                    )
                self.send(*command)

                if now >= next_report:
                    progress = (x - start_x) * math.cos(start_yaw) + (
                        y - start_y
                    ) * math.sin(start_yaw)
                    print(f"progress_m={progress:.3f}", flush=True)
                    next_report = now + 2.0
            else:
                raise TimeoutError("forward motion timed out")
        finally:
            self.stop()

        assert self.pose is not None
        end_x, end_y, end_yaw = self.pose
        delta_x, delta_y = end_x - start_x, end_y - start_y
        actual = delta_x * math.cos(start_yaw) + delta_y * math.sin(start_yaw)
        lateral = -delta_x * math.sin(start_yaw) + delta_y * math.cos(start_yaw)
        return {
            "status": "success",
            "requested_m": distance_m,
            "actual_m": actual,
            "error_m": actual - distance_m,
            "lateral_m": lateral,
            "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
            "start_odom": [start_x, start_y, start_yaw],
            "target_odom": [target_x, target_y, start_yaw],
            "end_odom": [end_x, end_y, end_yaw],
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance-m", type=float, default=0.85)
    parser.add_argument("--speed-mps", type=float, default=0.08)
    parser.add_argument("--timeout-s", type=float, default=35.0)
    args = parser.parse_args()
    if not 0.10 <= abs(args.distance_m) <= 3.0:
        parser.error("absolute --distance-m must be in [0.10, 3.0]")
    if not 0.02 <= args.speed_mps <= 0.15:
        parser.error("--speed-mps must be in [0.02, 0.15]")
    if args.timeout_s <= abs(args.distance_m) / args.speed_mps + 5.0:
        parser.error("--timeout-s is too short for the requested distance and speed")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ForwardController()
    exit_code = 1
    result: dict = {"status": "running", "requested_m": args.distance_m}
    try:
        result = node.move_forward(args.distance_m, args.speed_mps, args.timeout_s)
        exit_code = 0
    except BaseException as exc:
        result = {
            "status": "failed",
            "requested_m": args.distance_m,
            "error": repr(exc),
        }
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(result, indent=2), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
