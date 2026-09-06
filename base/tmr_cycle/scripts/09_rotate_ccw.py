#!/usr/bin/env python3
"""Rotate the base counter-clockwise through an odometry-closed-loop angle."""

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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RotationController(Node):
    def __init__(self, degrees: float, max_speed: float) -> None:
        super().__init__("tmr_ccw_rotation")
        self.requested = math.radians(degrees)
        self.max_speed = max_speed
        self.pose = None
        self.odom_at = 0.0
        self.last_yaw = None
        self.unwrapped_yaw = None
        self.scan_points = {}
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(
            TwistStamped, "/tmr_cycle/mission_cmd_vel", 10
        )
        self.lease_pub = self.create_publisher(Bool, "/tmr_cycle/mission_active", 10)
        self.create_subscription(
            Odometry,
            "/swerve_drive_controller/odom",
            self._on_odom,
            qos_profile_sensor_data,
        )
        for topic in ("/lidar_front/scan", "/lidar_rear/scan"):
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, source=topic: self._on_scan(source, message),
                qos_profile_sensor_data,
            )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _on_odom(self, message):
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        p = message.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw)
        self.odom_at = time.monotonic()

    def _on_scan(self, topic, message):
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

    def publish(self, vx, vy, wz):
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

    def stop(self):
        self.command[:] = (0.0, 0.0, 0.0)
        for _ in range(25):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def wait_ready(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            scans_ready = all(
                topic in self.scan_points
                and time.monotonic() - self.scan_points[topic][0] < 0.35
                for topic in ("/lidar_front/scan", "/lidar_rear/scan")
            )
            if self.pose is not None and scans_ready and self.command_pub.get_subscription_count() == 1:
                for _ in range(5):
                    self.publish(0.0, 0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.03)
                return
        raise RuntimeError("odometry, dual LiDAR, or exclusive velocity adapter unavailable")

    def rotation_clearance(self):
        now = time.monotonic()
        distances = []
        for topic in ("/lidar_front/scan", "/lidar_rear/scan"):
            if topic not in self.scan_points or now - self.scan_points[topic][0] > 0.35:
                raise RuntimeError("dual LiDAR became stale")
            for x, y in self.scan_points[topic][1]:
                if -0.415 <= x <= 0.415 and abs(y) <= 0.305:
                    continue
                distances.append(math.hypot(x, y))
        if len(distances) < 8:
            raise RuntimeError("too few external LiDAR returns for rotation")
        distances.sort()
        robust_nearest = distances[7]
        if robust_nearest < 0.56:
            raise RuntimeError(
                f"rotation envelope blocked: robust clearance={robust_nearest:.3f}m"
            )
        return robust_nearest

    def run(self, timeout_s):
        self.wait_ready()
        assert self.pose is not None and self.unwrapped_yaw is not None
        start_x, start_y, start_wrapped = self.pose
        start_unwrapped = self.unwrapped_yaw
        target = start_unwrapped + self.requested
        deadline = time.monotonic() + timeout_s
        last_tick = time.monotonic()
        next_report = 0.0
        stable_since = None
        minimum_clearance = math.inf
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or self.unwrapped_yaw is None or time.monotonic() - self.odom_at > 0.35:
                    raise RuntimeError("odometry became stale")
                x, y, yaw = self.pose
                error = target - self.unwrapped_yaw
                clearance = self.rotation_clearance()
                minimum_clearance = min(minimum_clearance, clearance)
                if abs(error) <= math.radians(0.8):
                    self.publish(0.0, 0.0, 0.0)
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.25:
                        break
                    continue
                stable_since = None
                desired_wz = clamp(1.15 * error, -self.max_speed, self.max_speed)
                frame_dx, frame_dy = start_x - x, start_y - y
                desired_vx = clamp(math.cos(yaw) * frame_dx + math.sin(yaw) * frame_dy, -0.018, 0.018)
                desired_vy = clamp(-math.sin(yaw) * frame_dx + math.cos(yaw) * frame_dy, -0.018, 0.018)
                now = time.monotonic()
                dt = clamp(now - last_tick, 0.02, 0.10)
                last_tick = now
                for index, (desired, acceleration) in enumerate(
                    ((desired_vx, 0.12), (desired_vy, 0.12), (desired_wz, 0.28))
                ):
                    step = acceleration * dt
                    self.command[index] += clamp(desired - self.command[index], -step, step)
                self.publish(*self.command)
                if now >= next_report:
                    actual = self.unwrapped_yaw - start_unwrapped
                    print(json.dumps({"event":"rotation_progress","ccw_deg":math.degrees(actual),"remaining_deg":math.degrees(error),"clearance_m":clearance}), flush=True)
                    next_report = now + 1.0
            else:
                raise TimeoutError("rotation timed out")
        finally:
            self.stop()

        assert self.pose is not None and self.unwrapped_yaw is not None
        end_x, end_y, end_wrapped = self.pose
        actual = self.unwrapped_yaw - start_unwrapped
        result = {
            "status": "success",
            "requested_ccw_deg": math.degrees(self.requested),
            "actual_ccw_deg": math.degrees(actual),
            "error_deg": math.degrees(actual - self.requested),
            "position_drift_m": math.hypot(end_x - start_x, end_y - start_y),
            "minimum_rotation_clearance_m": minimum_clearance,
            "start_odom": [start_x, start_y, start_wrapped],
            "end_odom": [end_x, end_y, end_wrapped],
        }
        if abs(result["error_deg"]) > 1.5 or result["position_drift_m"] > 0.04:
            raise RuntimeError("rotation endpoint outside tolerance: " + json.dumps(result))
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--degrees", type=float, default=180.0)
    parser.add_argument("--speed-rps", type=float, default=0.18)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args()
    if not 1.0 <= args.degrees <= 180.0:
        parser.error("degrees must be in [1, 180]")
    rclpy.init()
    node = RotationController(args.degrees, args.speed_rps)
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
