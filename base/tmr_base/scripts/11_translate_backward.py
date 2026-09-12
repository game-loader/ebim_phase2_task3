#!/usr/bin/env python3
"""Move the TMR base backward in its current body frame using odometry closure."""

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
from std_msgs.msg import Bool


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BackwardController(Node):
    def __init__(self, distance: float, speed: float) -> None:
        super().__init__("tmr_translate_backward")
        self.distance = distance
        self.speed = speed
        self.pose = None
        self.pose_at = 0.0
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(TwistStamped, "/tmr_cycle/mission_cmd_vel", 10)
        self.lease_pub = self.create_publisher(Bool, "/tmr_cycle/mission_active", 10)
        self.create_subscription(Odometry, "/swerve_drive_controller/odom", self._odom, qos_profile_sensor_data)

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw_of(msg.pose.pose.orientation))
        self.pose_at = time.monotonic()

    def publish(self, vx: float, vy: float, wz: float) -> None:
        lease = Bool()
        lease.data = True
        self.lease_pub.publish(lease)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.command_pub.publish(msg)

    def stop(self) -> None:
        for _ in range(30):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def run(self, timeout: float) -> dict:
        deadline = time.monotonic() + 8.0
        while self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pose is None or self.command_pub.get_subscription_count() != 1:
            raise RuntimeError("odometry or exclusive velocity adapter unavailable")

        start_x, start_y, start_yaw = self.pose
        forward = (math.cos(start_yaw), math.sin(start_yaw))
        target_x = start_x - self.distance * forward[0]
        target_y = start_y - self.distance * forward[1]
        deadline = time.monotonic() + timeout
        last_tick = time.monotonic()
        next_report = 0.0
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.pose_at > 0.4:
                    raise RuntimeError("odometry became stale")
                x, y, yaw = self.pose
                ex, ey = target_x - x, target_y - y
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                yaw_error = wrap(start_yaw - yaw)
                if math.hypot(ex, ey) <= 0.018 and abs(yaw_error) <= math.radians(1.0):
                    break
                desired = (
                    clamp(0.8 * body_x, -self.speed, self.speed),
                    clamp(0.8 * body_y, -0.025, 0.025),
                    clamp(1.2 * yaw_error, -0.08, 0.08),
                )
                now = time.monotonic()
                dt = clamp(now - last_tick, 0.02, 0.10)
                last_tick = now
                for i, (target, accel) in enumerate(zip(desired, (0.15, 0.15, 0.28))):
                    step = accel * dt
                    self.command[i] += clamp(target - self.command[i], -step, step)
                self.publish(*self.command)
                if now >= next_report:
                    moved = -((x - start_x) * forward[0] + (y - start_y) * forward[1])
                    print(json.dumps({"event": "backward_progress", "m": moved}), flush=True)
                    next_report = now + 1.0
            else:
                raise TimeoutError("backward translation timed out")
        finally:
            self.stop()

        end_x, end_y, end_yaw = self.pose
        actual = -((end_x - start_x) * forward[0] + (end_y - start_y) * forward[1])
        return {
            "status": "success",
            "requested_backward_m": self.distance,
            "actual_backward_m": actual,
            "error_m": actual - self.distance,
            "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-m", type=float, default=0.25)
    parser.add_argument("--speed-mps", type=float, default=0.06)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    args = parser.parse_args()
    rclpy.init()
    node = BackwardController(args.distance_m, args.speed_mps)
    try:
        print(json.dumps(node.run(args.timeout_s), indent=2), flush=True)
        return 0
    except BaseException as exc:
        node.stop()
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
