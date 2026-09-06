#!/usr/bin/env python3
"""Low-speed left-shift and 360-degree scan gesture for live SLAM bootstrap."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import yaml


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BootstrapMapper(Node):
    def __init__(self, config: dict) -> None:
        super().__init__("tmr_bootstrap_mapping")
        self.config = config
        self.motion = config["bootstrap_mapping"]
        self.odom_frame = str(self.motion.get("odom_frame", "odom"))
        self.base_frame = str(self.motion.get("base_frame", "base_link"))
        self._latest_scan: LaserScan | None = None
        self._scan_received_at = 0.0
        self._last_command = (0.0, 0.0, 0.0)
        self._last_command_at = time.monotonic()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            TwistStamped, str(config["topics"]["manual_cmd"]), 10
        )
        self.create_subscription(
            LaserScan,
            str(config["topics"]["scan"]),
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._scan_received_at = time.monotonic()

    def _pose(self) -> tuple[float, float, float]:
        transform = self._tf_buffer.lookup_transform(
            self.odom_frame, self.base_frame, Time(), Duration(seconds=0.2)
        )
        t = transform.transform.translation
        return float(t.x), float(t.y), yaw_from_quaternion(transform.transform.rotation)

    def wait_ready(self, timeout_sec: float = 15.0) -> tuple[float, float, float]:
        deadline = time.monotonic() + timeout_sec
        pose = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                pose = self._pose()
            except Exception:
                continue
            if self._latest_scan is not None and self._publisher.get_subscription_count() > 0:
                return pose
        raise RuntimeError(
            "not ready: require odom->base_link TF, /navigation/scan, and cmd_vel_adapter"
        )

    def _publish_raw(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._publisher.publish(msg)

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        now = time.monotonic()
        dt = clamp(now - self._last_command_at, 0.02, 0.10)
        old_vx, old_vy, old_wz = self._last_command
        linear_step = float(self.motion["max_linear_accel"]) * dt
        angular_step = float(self.motion["max_angular_accel"]) * dt
        vx = old_vx + clamp(vx - old_vx, -linear_step, linear_step)
        vy = old_vy + clamp(vy - old_vy, -linear_step, linear_step)
        wz = old_wz + clamp(wz - old_wz, -angular_step, angular_step)
        self._last_command = (vx, vy, wz)
        self._last_command_at = now
        self._publish_raw(vx, vy, wz)

    def stop(self) -> None:
        for _ in range(20):
            self._publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.04)
            if max(map(abs, self._last_command)) < 1e-3:
                break
        self._last_command = (0.0, 0.0, 0.0)
        self._last_command_at = time.monotonic()
        self._publish_raw(0.0, 0.0, 0.0)

    def _scan_points_in_base(self) -> list[tuple[float, float]]:
        scan = self._latest_scan
        if scan is None or time.monotonic() - self._scan_received_at > 0.5:
            raise RuntimeError("LiDAR scan is missing or stale")
        source_frame = scan.header.frame_id or self.base_frame
        tx = ty = transform_yaw = 0.0
        if source_frame != self.base_frame:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, source_frame, Time(), Duration(seconds=0.15)
            )
            tx = float(transform.transform.translation.x)
            ty = float(transform.transform.translation.y)
            transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        points: list[tuple[float, float]] = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            sx, sy = distance * math.cos(angle), distance * math.sin(angle)
            points.append(
                (
                    tx + math.cos(transform_yaw) * sx - math.sin(transform_yaw) * sy,
                    ty + math.sin(transform_yaw) * sx + math.cos(transform_yaw) * sy,
                )
            )
        return points

    def _translation_blocked(self, vx: float, vy: float) -> bool:
        speed = math.hypot(vx, vy)
        if speed < 1e-5:
            return False
        ux, uy = vx / speed, vy / speed
        stop_distance = float(self.motion["translation_obstacle_distance"])
        half_width = float(self.motion["translation_corridor_half_width"])
        obstacle_hits = 0
        for x, y in self._scan_points_in_base():
            along = x * ux + y * uy
            across = abs(-uy * x + ux * y)
            if 0.0 < along < stop_distance and across < half_width:
                obstacle_hits += 1
                if obstacle_hits >= 4:
                    return True
        return False

    def _rotation_blocked(self) -> tuple[bool, float]:
        clearance = float(self.motion["rotation_clearance"])
        distances = sorted(math.hypot(x, y) for x, y in self._scan_points_in_base())
        nearest = distances[3] if len(distances) >= 4 else math.inf
        return nearest < clearance, nearest

    def translate_to(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        label: str,
    ) -> None:
        deadline = time.monotonic() + float(self.motion["translation_timeout_sec"])
        max_side_speed = float(self.motion["translate_speed"])
        max_forward_speed = float(self.motion["forward_correction_speed"])
        position_tolerance = float(self.motion["position_tolerance"])
        yaw_tolerance = math.radians(float(self.motion["yaw_tolerance_deg"]))
        self.get_logger().info(f"{label}: target odom=({target_x:.3f}, {target_y:.3f})")
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    x, y, yaw = self._pose()
                except Exception:
                    continue
                ex, ey = target_x - x, target_y - y
                yaw_error = wrap(target_yaw - yaw)
                if math.hypot(ex, ey) <= position_tolerance and abs(yaw_error) <= yaw_tolerance:
                    return
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                vx = clamp(0.8 * body_x, -max_forward_speed, max_forward_speed)
                vy = clamp(0.8 * body_y, -max_side_speed, max_side_speed)
                wz = clamp(1.2 * yaw_error, -0.12, 0.12)
                if self._translation_blocked(vx, vy):
                    raise RuntimeError(f"obstacle blocks {label}; stopped before motion corridor")
                self._publish(vx, vy, wz)
            raise TimeoutError(f"{label} timed out")
        finally:
            self.stop()

    def rotate_once(self, center_x: float, center_y: float, start_yaw: float) -> None:
        target_rotation = math.radians(float(self.motion["rotation_deg"]))
        direction = 1.0 if target_rotation >= 0.0 else -1.0
        target_rotation = abs(target_rotation)
        rotate_speed = abs(float(self.motion["rotate_speed"]))
        deadline = time.monotonic() + float(self.motion["rotation_timeout_sec"])
        x, y, last_yaw = self._pose()
        turned = 0.0
        self.get_logger().info(
            f"360-degree scan: speed={math.degrees(rotate_speed):.1f} deg/s, center=({center_x:.3f},{center_y:.3f})"
        )
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                blocked, nearest = self._rotation_blocked()
                if blocked:
                    raise RuntimeError(
                        f"rotation clearance insufficient: nearest={nearest:.3f} m, "
                        f"required={float(self.motion['rotation_clearance']):.3f} m"
                    )
                x, y, yaw = self._pose()
                delta = wrap(yaw - last_yaw)
                if abs(delta) < 0.5:
                    turned += direction * delta
                last_yaw = yaw
                if turned >= target_rotation - math.radians(2.0):
                    return
                ex, ey = center_x - x, center_y - y
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                vx = clamp(0.6 * body_x, -0.02, 0.02)
                vy = clamp(0.6 * body_y, -0.02, 0.02)
                self._publish(vx, vy, direction * rotate_speed)
            raise TimeoutError("360-degree scan timed out")
        finally:
            self.stop()

    def align_yaw(self, target_yaw: float) -> None:
        deadline = time.monotonic() + 8.0
        tolerance = math.radians(float(self.motion["yaw_tolerance_deg"]))
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                yaw_error = wrap(target_yaw - self._pose()[2])
                if abs(yaw_error) <= tolerance:
                    return
                blocked, nearest = self._rotation_blocked()
                if blocked:
                    raise RuntimeError(f"cannot align yaw: rotation clearance is {nearest:.3f} m")
                self._publish(0.0, 0.0, clamp(1.0 * yaw_error, -0.10, 0.10))
            raise TimeoutError("final yaw alignment timed out")
        finally:
            self.stop()

    def run(self, left_distance: float | None, no_return: bool, dry_run: bool) -> None:
        start_x, start_y, start_yaw = self.wait_ready()
        distance = float(self.motion["left_distance"] if left_distance is None else left_distance)
        scan_x = start_x - math.sin(start_yaw) * distance
        scan_y = start_y + math.cos(start_yaw) * distance
        blocked, nearest = self._rotation_blocked()
        self.get_logger().info(
            f"ready: start=({start_x:.3f},{start_y:.3f},{math.degrees(start_yaw):.1f} deg), "
            f"left={distance:.3f} m, current nearest obstacle={nearest:.3f} m"
        )
        if dry_run:
            self.get_logger().warning("dry-run: no velocity command was sent")
            return
        if distance <= 0.0:
            raise ValueError("left distance must be positive")
        self.translate_to(scan_x, scan_y, start_yaw, "shift left for mapping")
        self.rotate_once(scan_x, scan_y, start_yaw)
        self.align_yaw(start_yaw)
        if not no_return and bool(self.motion.get("return_to_start", True)):
            self.translate_to(start_x, start_y, start_yaw, "return to fixed start")
            self.align_yaw(start_yaw)
        self.get_logger().info("bootstrap mapping gesture completed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--left-distance", type=float, help="override configured left shift in metres")
    parser.add_argument("--no-return", action="store_true", help="leave the base at the scan position")
    parser.add_argument("--dry-run", action="store_true", help="check TF, scan and command adapter only")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))

    rclpy.init()
    node = BootstrapMapper(config)
    try:
        node.run(args.left_distance, args.no_return, args.dry_run)
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
