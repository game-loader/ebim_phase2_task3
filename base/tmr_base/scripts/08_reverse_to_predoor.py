#!/usr/bin/env python3
"""Reverse from the confirmed post-door stop to a fixed pre-door standoff.

The normal entry ends 0.70 m beyond the frozen door plane.  Therefore the
default 1.70 m reverse translation returns the base centre to 1.00 m before
that plane.  Commands use the exclusive mission channel and retain a live
dual-LiDAR braking guard throughout the odometry-closed-loop translation.
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
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"
SCAN_TOPICS = ("/lidar_front/scan", "/lidar_rear/scan")


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ReverseController(Node):
    def __init__(self, distance_m: float, speed_mps: float) -> None:
        super().__init__("tmr_reverse_to_predoor")
        self.distance_m = distance_m
        self.speed_mps = speed_mps
        self.pose = None
        self.odom_at = 0.0
        self.scan_points: dict[str, tuple[float, list[tuple[float, float]]]] = {}
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(TwistStamped, COMMAND_TOPIC, 10)
        self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        for topic in SCAN_TOPICS:
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, source=topic: self._on_scan(source, message),
                qos_profile_sensor_data,
            )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _on_odom(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw_from_quaternion(message.pose.pose.orientation))
        self.odom_at = time.monotonic()

    def _on_scan(self, topic: str, message: LaserScan) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", message.header.frame_id.lstrip("/"), Time()
            )
        except Exception:
            return
        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        points = []
        angle = float(message.angle_min)
        for index, distance in enumerate(message.ranges):
            if index % 2 == 0 and math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                sx, sy = distance * math.cos(angle), distance * math.sin(angle)
                points.append((float(t.x) + c * sx - s * sy, float(t.y) + s * sx + c * sy))
            angle += float(message.angle_increment)
        self.scan_points[topic] = (time.monotonic(), points)

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

    def stop(self) -> None:
        self.command[:] = (0.0, 0.0, 0.0)
        for _ in range(25):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            fresh_scans = all(
                topic in self.scan_points and time.monotonic() - self.scan_points[topic][0] < 0.35
                for topic in SCAN_TOPICS
            )
            if self.pose is not None and fresh_scans and self.command_pub.get_subscription_count() == 1:
                for _ in range(5):
                    self.publish(0.0, 0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.03)
                return
        raise RuntimeError("odometry, dual LiDAR, or exclusive velocity adapter unavailable")

    def rear_speed_scale(self, speed: float) -> tuple[float, float | None]:
        now = time.monotonic()
        if any(topic not in self.scan_points or now - self.scan_points[topic][0] > 0.35 for topic in SCAN_TOPICS):
            raise RuntimeError("dual LiDAR became stale")
        rear_m, half_width, mask_pad, side_margin = 0.40, 0.29, 0.015, 0.055
        clearances = []
        for _stamp, points in self.scan_points.values():
            for x, y in points:
                if -rear_m - mask_pad <= x <= 0.40 + mask_pad and abs(y) <= half_width + mask_pad:
                    continue
                if x < -rear_m and abs(y) <= half_width + side_margin:
                    clearances.append(-rear_m - x)
        nearest = min(clearances) if clearances else None
        hard = speed * speed / (2.0 * 0.25) + 0.25 * speed + 0.08
        if nearest is not None and nearest <= hard:
            raise RuntimeError(
                f"rear braking corridor blocked: clearance={nearest:.3f}m required={hard:.3f}m"
            )
        if nearest is None or nearest >= hard + 0.18:
            return 1.0, nearest
        return clamp((nearest - hard) / 0.18, 0.18, 1.0), nearest

    def run(self, timeout_s: float) -> dict:
        self.wait_ready()
        assert self.pose is not None
        start_x, start_y, start_yaw = self.pose
        forward = (math.cos(start_yaw), math.sin(start_yaw))
        target_x = start_x - self.distance_m * forward[0]
        target_y = start_y - self.distance_m * forward[1]
        deadline = time.monotonic() + timeout_s
        last_progress = 0.0
        progress_at = time.monotonic()
        nearest_seen = math.inf
        next_report = 0.0
        last_tick = time.monotonic()
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.odom_at > 0.35:
                    raise RuntimeError("odometry became stale")
                x, y, yaw = self.pose
                dx, dy = x - start_x, y - start_y
                backward = -(dx * forward[0] + dy * forward[1])
                remaining = self.distance_m - backward
                yaw_error = wrap(start_yaw - yaw)
                if remaining <= 0.025 and abs(yaw_error) <= math.radians(1.2):
                    break
                if backward > last_progress + 0.01:
                    last_progress = backward
                    progress_at = time.monotonic()
                elif time.monotonic() - progress_at > 3.0:
                    raise RuntimeError("no odometry progress while reversing")

                world_ex, world_ey = target_x - x, target_y - y
                body_y = -math.sin(yaw) * world_ex + math.cos(yaw) * world_ey
                raw_speed = min(self.speed_mps, max(0.025, 0.70 * remaining))
                scale, nearest = self.rear_speed_scale(raw_speed)
                if nearest is not None:
                    nearest_seen = min(nearest_seen, nearest)
                desired = (
                    -raw_speed * scale,
                    clamp(0.75 * body_y, -0.025, 0.025),
                    clamp(1.10 * yaw_error, -0.07, 0.07),
                )
                now = time.monotonic()
                dt = clamp(now - last_tick, 0.02, 0.10)
                last_tick = now
                limits = (0.15 * dt, 0.15 * dt, 0.28 * dt)
                for index in range(3):
                    self.command[index] += clamp(desired[index] - self.command[index], -limits[index], limits[index])
                self.publish(*self.command)
                if now >= next_report:
                    print(json.dumps({"event":"reverse_progress","backward_m":backward,"remaining_m":remaining,"rear_clearance_m":nearest}), flush=True)
                    next_report = now + 1.0
            else:
                raise TimeoutError("reverse translation timed out")
        finally:
            self.stop()

        assert self.pose is not None
        end_x, end_y, end_yaw = self.pose
        actual = -((end_x - start_x) * forward[0] + (end_y - start_y) * forward[1])
        result = {
            "status": "success",
            "requested_backward_m": self.distance_m,
            "actual_backward_m": actual,
            "nominal_door_standoff_m": actual - 0.70,
            "error_m": actual - self.distance_m,
            "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
            "minimum_rear_clearance_m": None if math.isinf(nearest_seen) else nearest_seen,
            "start_odom": [start_x, start_y, start_yaw],
            "end_odom": [end_x, end_y, end_yaw],
        }
        if abs(result["error_m"]) > 0.04:
            raise RuntimeError("reverse endpoint outside 4 cm tolerance: " + json.dumps(result))
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-m", type=float, default=1.70)
    parser.add_argument("--speed-mps", type=float, default=0.06)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args()
    if not 0.10 <= args.distance_m <= 2.50:
        parser.error("distance must be in [0.10, 2.50] m")
    if not 0.02 <= args.speed_mps <= 0.10:
        parser.error("speed must be in [0.02, 0.10] m/s")
    rclpy.init()
    node = ReverseController(args.distance_m, args.speed_mps)
    try:
        result = node.run(args.timeout_s)
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        node.stop()
        print(json.dumps({"status":"failed","error":repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
