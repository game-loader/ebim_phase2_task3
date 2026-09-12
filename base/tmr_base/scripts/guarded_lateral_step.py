#!/usr/bin/env python3
"""Execute one signed lateral base step with odometry and dual-LiDAR guards."""

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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"
SCAN_TOPICS = ("/lidar_front/scan", "/lidar_rear/scan")
# Measured transforms used by the deployed navigation adapter.  The XY block
# is Rz(yaw) * Rx(pi), hence [[cos, sin], [sin, -cos]].
LIDAR_EXTRINSICS = {
    "lidar_front": (0.3275, 0.2175, 0.7846018366025517),
    "lidar_rear": (-0.3275, -0.2175, -2.3569908169872414),
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class GuardedLateralStep(Node):
    def __init__(self, right_m: float, speed_mps: float, forward_m: float = 0.0) -> None:
        super().__init__("tmr_guarded_lateral_step")
        self.right_m = right_m
        self.forward_m = forward_m
        self.axis_kind = "forward" if abs(forward_m) > 0.0 else "right"
        self.speed_mps = speed_mps
        self.pose = None
        self.velocity = None
        self.odom_at = 0.0
        self.scan_points: dict[str, tuple[float, list[tuple[float, float]]]] = {}
        self.command = [0.0, 0.0, 0.0]
        self.command_pub = self.create_publisher(TwistStamped, COMMAND_TOPIC, 10)
        self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, qos_profile_sensor_data)
        for topic in SCAN_TOPICS:
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, source=topic: self._scan(source, message),
                qos_profile_sensor_data,
            )

    def _odom(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw_of(message.pose.pose.orientation))
        twist = message.twist.twist
        self.velocity = (
            float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)
        )
        self.odom_at = time.monotonic()

    def _scan(self, topic: str, message: LaserScan) -> None:
        frame = message.header.frame_id.lstrip("/")
        if frame not in LIDAR_EXTRINSICS:
            return
        tx, ty, yaw = LIDAR_EXTRINSICS[frame]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = []
        angle = float(message.angle_min)
        for index, distance in enumerate(message.ranges):
            if (
                index % 2 == 0
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                scan_x = distance * math.cos(angle)
                scan_y = distance * math.sin(angle)
                points.append((
                    tx + cosine * scan_x + sine * scan_y,
                    ty + sine * scan_x - cosine * scan_y,
                ))
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
        for _ in range(30):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.025)

    def wait_ready(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            scans_ready = all(
                topic in self.scan_points and now - self.scan_points[topic][0] < 0.35
                for topic in SCAN_TOPICS
            )
            stopped = self.velocity is not None and (
                math.hypot(self.velocity[0], self.velocity[1]) <= 0.015
                and abs(self.velocity[2]) <= 0.03
            )
            if (
                self.pose is not None
                and now - self.odom_at < 0.35
                and scans_ready
                and stopped
                and self.command_pub.get_subscription_count() >= 1
            ):
                return
        raise RuntimeError(
            "fresh odometry, stationary base, dual LiDAR, or velocity adapter unavailable"
        )

    def clearance(self, direction: float, speed: float) -> float | None:
        now = time.monotonic()
        if any(
            topic not in self.scan_points or now - self.scan_points[topic][0] > 0.35
            for topic in SCAN_TOPICS
        ):
            raise RuntimeError("dual LiDAR became stale")
        # The verified chassis is x=[-0.40, +0.40] m.  At the required left
        # initial pose the tool reaches x=+0.422 m, so keep an asymmetric
        # padded sweep.  A persistent room return at x~-0.55 m is outside the
        # lateral swept volume and must not masquerade as side clearance.
        rear_limit, front_limit, lateral_extent = -0.45, 0.50, 0.55
        clearances = []
        for _, points in self.scan_points.values():
            for x, y in points:
                if -0.42 <= x <= 0.42 and -0.31 <= y <= 0.31:
                    continue
                if self.axis_kind == "right":
                    if not rear_limit <= x <= front_limit:
                        continue
                    if direction > 0 and y < -lateral_extent:
                        clearances.append(-lateral_extent - y)
                    elif direction < 0 and y > lateral_extent:
                        clearances.append(y - lateral_extent)
                else:
                    if not -lateral_extent <= y <= lateral_extent:
                        continue
                    if direction > 0 and x > front_limit:
                        clearances.append(x - front_limit)
                    elif direction < 0 and x < rear_limit:
                        clearances.append(rear_limit - x)
        nearest = min(clearances) if clearances else None
        required = speed * speed / (2.0 * 0.25) + 0.25 * speed + 0.10
        if nearest is not None and nearest <= required:
            side = (
                ("right" if direction > 0 else "left")
                if self.axis_kind == "right"
                else ("forward" if direction > 0 else "backward")
            )
            raise RuntimeError(
                f"{side} envelope blocked: clearance={nearest:.3f}m "
                f"required={required:.3f}m"
            )
        return nearest

    def run(self, timeout_s: float) -> dict:
        self.wait_ready()
        start_x, start_y, start_yaw = self.pose
        axis = (
            (math.cos(start_yaw), math.sin(start_yaw))
            if self.axis_kind == "forward"
            else (math.sin(start_yaw), -math.cos(start_yaw))
        )
        requested = self.forward_m if self.axis_kind == "forward" else self.right_m
        target_x = start_x + requested * axis[0]
        target_y = start_y + requested * axis[1]
        direction = 1.0 if requested > 0 else -1.0
        deadline = time.monotonic() + timeout_s
        last_tick = time.monotonic()
        nearest_seen = math.inf
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.025)
                if self.pose is None or time.monotonic() - self.odom_at > 0.35:
                    raise RuntimeError("odometry became stale")
                x, y, yaw = self.pose
                progress = (
                    (x - start_x) * axis[0] + (y - start_y) * axis[1]
                )
                remaining = requested - progress
                yaw_error = wrap(start_yaw - yaw)
                if abs(remaining) <= 0.005 and abs(yaw_error) <= math.radians(1.0):
                    break
                raw_speed = min(self.speed_mps, max(0.008, 0.9 * abs(remaining)))
                nearest = self.clearance(direction, raw_speed)
                if nearest is not None:
                    nearest_seen = min(nearest_seen, nearest)
                world_ex, world_ey = target_x - x, target_y - y
                body_x = math.cos(yaw) * world_ex + math.sin(yaw) * world_ey
                body_y = -math.sin(yaw) * world_ex + math.cos(yaw) * world_ey
                if self.axis_kind == "forward":
                    desired = (
                        clamp(0.9 * body_x, -raw_speed, raw_speed),
                        clamp(0.8 * body_y, -0.02, 0.02),
                        clamp(1.2 * yaw_error, -0.06, 0.06),
                    )
                else:
                    desired = (
                        clamp(0.8 * body_x, -0.02, 0.02),
                        clamp(0.9 * body_y, -raw_speed, raw_speed),
                        clamp(1.2 * yaw_error, -0.06, 0.06),
                    )
                tick = time.monotonic()
                delta = clamp(tick - last_tick, 0.02, 0.10)
                last_tick = tick
                for index, (target, acceleration) in enumerate(
                    zip(desired, (0.12, 0.12, 0.24))
                ):
                    step = acceleration * delta
                    self.command[index] += clamp(target - self.command[index], -step, step)
                self.publish(*self.command)
            else:
                raise TimeoutError("guarded lateral step timed out")
        finally:
            self.stop()

        end_x, end_y, end_yaw = self.pose
        actual = (
            (end_x - start_x) * axis[0] + (end_y - start_y) * axis[1]
        )
        result = {
            "status": "success",
            "error_m": actual - requested,
            "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
            "minimum_directional_clearance_m": (
                None if math.isinf(nearest_seen) else nearest_seen
            ),
        }
        result[f"requested_{self.axis_kind}_m"] = requested
        result[f"actual_{self.axis_kind}_m"] = actual
        return result

    def check_only(self) -> dict:
        self.wait_ready()
        requested = self.forward_m if self.axis_kind == "forward" else self.right_m
        direction = 1.0 if requested > 0 else -1.0
        return {
            "status": "ready",
            "direction": (
                ("right" if direction > 0 else "left")
                if self.axis_kind == "right"
                else ("forward" if direction > 0 else "backward")
            ),
            "directional_clearance_m": self.clearance(direction, self.speed_mps),
            "motion_commanded": False,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    direction = parser.add_mutually_exclusive_group(required=True)
    direction.add_argument("--right-m", type=float)
    direction.add_argument("--forward-m", type=float)
    parser.add_argument("--speed-mps", type=float, default=0.025)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    distance = args.forward_m if args.forward_m is not None else args.right_m
    if not 0.008 <= abs(distance) <= 2.0:
        parser.error("absolute motion distance must be in [0.008, 2.0] m")
    if not 0.008 <= args.speed_mps <= 0.04:
        parser.error("speed must be in [0.008, 0.04] m/s")
    rclpy.init()
    node = GuardedLateralStep(
        args.right_m or 0.0, args.speed_mps, forward_m=args.forward_m or 0.0
    )
    try:
        result = node.check_only() if args.check_only else node.run(args.timeout_s)
        print(json.dumps(result, indent=2), flush=True)
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
