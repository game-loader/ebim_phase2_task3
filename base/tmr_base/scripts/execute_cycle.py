#!/usr/bin/env python3
"""Run the start-relative mission and refine pickup from live LiDAR leg detections."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import yaml

from table_leg_detection import DetectionError, PairDetection, detect_pair


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_z(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CycleExecutor(Node):
    def __init__(self, config: dict, skip_arm: bool, allow_fallback: bool) -> None:
        super().__init__("tmr_cycle_executor")
        self.config = config
        self.points = config["points"]
        self.skip_arm = skip_arm
        self.allow_fallback = allow_fallback
        topics = config["topics"]
        self.map_frame = str(config.get("frame_id", "map"))
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._manual_pub = self.create_publisher(TwistStamped, topics["manual_cmd"], 10)
        self._arm_pub = self.create_publisher(String, topics["arm_command"], 10)
        self._perception_pub = self.create_publisher(String, topics["perception_command"], 10)
        self._legs_pub = self.create_publisher(PoseArray, topics["detected_legs"], 10)
        self._pickup_pose_pub = self.create_publisher(PoseStamped, topics["detected_pickup_pose"], 10)
        self._arm_done = False
        self._latest_scan: LaserScan | None = None
        self._mission_origin: tuple[float, float, float] | None = None
        self._detected_table_frame: tuple[float, float, float] | None = None
        self._collecting = False
        self._collected_points: list[tuple[float, float]] = []
        self._collected_frames = 0
        self._last_detection: PairDetection | None = None
        self.create_subscription(Bool, topics["arm_done"], self._on_arm_done, 10)
        self.create_subscription(LaserScan, topics["scan"], self._on_scan, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.2, self._broadcast_semantic_frames)

    def _broadcast_semantic_frames(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms: list[TransformStamped] = []
        if self._mission_origin is not None:
            ox, oy, origin_yaw = self._mission_origin
            mission = TransformStamped()
            mission.header.stamp = now
            mission.header.frame_id = self.map_frame
            mission.child_frame_id = "mission_start"
            mission.transform.translation.x = ox
            mission.transform.translation.y = oy
            mission.transform.rotation.z, mission.transform.rotation.w = quaternion_z(origin_yaw)
            transforms.append(mission)
        if self._detected_table_frame is not None:
            x, y, yaw = self._detected_table_frame
            table = TransformStamped()
            table.header.stamp = now
            table.header.frame_id = "mission_start"
            table.child_frame_id = "detected_table_edge"
            table.transform.translation.x = x
            table.transform.translation.y = y
            table.transform.rotation.z, table.transform.rotation.w = quaternion_z(yaw)
            transforms.append(table)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    def _on_arm_done(self, msg: Bool) -> None:
        if msg.data:
            self._arm_done = True

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        if self._collecting and self._mission_origin is not None:
            self._accumulate_scan(msg)

    def _lookup_frame_pose(self, child_frame: str) -> tuple[float, float, float]:
        transform = self._tf_buffer.lookup_transform(
            self.map_frame, child_frame, Time(), Duration(seconds=0.25)
        )
        t = transform.transform.translation
        return float(t.x), float(t.y), yaw_from_quaternion(transform.transform.rotation)

    def capture_mission_origin(self, timeout_sec: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_sec
        samples: list[tuple[float, float, float]] = []
        while rclpy.ok() and time.monotonic() < deadline and len(samples) < 8:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                samples.append(self._lookup_frame_pose("base_link"))
            except Exception:
                continue
        if len(samples) < 8:
            raise RuntimeError("map -> base_link is unavailable; finish SLAM startup first")
        xs = sorted(item[0] for item in samples)
        ys = sorted(item[1] for item in samples)
        sin_yaw = sum(math.sin(item[2]) for item in samples)
        cos_yaw = sum(math.cos(item[2]) for item in samples)
        self._mission_origin = (xs[len(xs) // 2], ys[len(ys) // 2], math.atan2(sin_yaw, cos_yaw))
        x, y, yaw = self._mission_origin
        self.get_logger().info(
            f"Mission origin captured automatically: map=({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f} deg)"
        )

    def _mission_to_map(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        if self._mission_origin is None:
            raise RuntimeError("mission origin has not been captured")
        ox, oy, origin_yaw = self._mission_origin
        return (
            ox + math.cos(origin_yaw) * x - math.sin(origin_yaw) * y,
            oy + math.sin(origin_yaw) * x + math.cos(origin_yaw) * y,
            wrap(origin_yaw + yaw),
        )

    def _map_to_mission(self, x: float, y: float, yaw: float = 0.0) -> tuple[float, float, float]:
        if self._mission_origin is None:
            raise RuntimeError("mission origin has not been captured")
        ox, oy, origin_yaw = self._mission_origin
        dx, dy = x - ox, y - oy
        return (
            math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy,
            -math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy,
            wrap(yaw - origin_yaw),
        )

    def _pose_stamped(self, mission_x: float, mission_y: float, mission_yaw: float) -> PoseStamped:
        x, y, yaw = self._mission_to_map(mission_x, mission_y, mission_yaw)
        z, w = quaternion_z(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def named_pose(self, name: str) -> PoseStamped:
        point = self.points[name]
        return self._pose_stamped(float(point["x"]), float(point["y"]), math.radians(float(point["yaw_deg"])))

    def navigate_pose(self, pose: PoseStamped, label: str, timeout_sec: float | None = None) -> None:
        self.get_logger().info(
            f"Navigate -> {label}: map=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )
        if not self._nav.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("Nav2 navigate_to_pose action is unavailable")
        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"navigation goal rejected: {label}")
        result_future = handle.get_result_async()
        timeout_sec = timeout_sec or float(self.config["cycle"]["navigation_timeout_sec"])
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            handle.cancel_goal_async()
            raise TimeoutError(f"navigation timed out: {label}")
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if result is None else result.status
            raise RuntimeError(f"navigation failed at {label}, status={status}")

    def navigate(self, point_name: str) -> None:
        self.navigate_pose(self.named_pose(point_name), point_name)

    def _accumulate_scan(self, scan: LaserScan) -> None:
        source_frame = scan.header.frame_id or "base_link"
        try:
            source_x, source_y, source_yaw = self._lookup_frame_pose(source_frame)
        except Exception:
            return
        detection_cfg = self.config["table_detection"]
        xmin, xmax = map(float, detection_cfg["roi"]["x"])
        ymin, ymax = map(float, detection_cfg["roi"]["y"])
        accepted = 0
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            local_x, local_y = distance * math.cos(angle), distance * math.sin(angle)
            map_x = source_x + math.cos(source_yaw) * local_x - math.sin(source_yaw) * local_y
            map_y = source_y + math.sin(source_yaw) * local_x + math.cos(source_yaw) * local_y
            mission_x, mission_y, _ = self._map_to_mission(map_x, map_y)
            if xmin <= mission_x <= xmax and ymin <= mission_y <= ymax:
                self._collected_points.append((mission_x, mission_y))
                accepted += 1
        if accepted:
            self._collected_frames += 1

    def _current_mission_pose(self) -> tuple[float, float, float]:
        return self._map_to_mission(*self._lookup_frame_pose("base_link"))

    def detect_pickup_pose(self) -> PoseStamped:
        cfg = self.config["table_detection"]
        self._collected_points = []
        self._collected_frames = 0
        self._collecting = True
        self.get_logger().info("Collecting stationary LiDAR frames for the near table-leg pair")
        deadline = time.monotonic() + float(cfg["collect_sec"])
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self._collecting = False
        if self._collected_frames < int(cfg["min_scan_frames"]):
            raise DetectionError(
                f"too few usable scan frames: {self._collected_frames}/{cfg['min_scan_frames']}"
            )
        observer_x, observer_y, _ = self._current_mission_pose()
        clusters, detection = detect_pair(self._collected_points, cfg, (observer_x, observer_y))
        self._last_detection = detection
        self._detected_table_frame = (
            detection.midpoint[0],
            detection.midpoint[1],
            detection.approach[2],
        )
        self._publish_detection(clusters, detection)
        first, second = detection.legs
        ax, ay, ayaw = detection.approach
        self.get_logger().info(
            "Detected target legs mission="
            f"({first.x:.3f},{first.y:.3f})/({second.x:.3f},{second.y:.3f}); "
            f"approach=({ax:.3f},{ay:.3f},{math.degrees(ayaw):.1f} deg), score={detection.score:.3f}"
        )
        return self._pose_stamped(ax, ay, ayaw)

    def _publish_detection(self, clusters, detection: PairDetection) -> None:
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for cluster in clusters:
            map_x, map_y, _ = self._mission_to_map(cluster.x, cluster.y, 0.0)
            pose = Pose()
            pose.position.x = map_x
            pose.position.y = map_y
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self._legs_pub.publish(pose_array)
        ax, ay, ayaw = detection.approach
        pickup = self._pose_stamped(ax, ay, ayaw)
        self._pickup_pose_pub.publish(pickup)

    def arm_pick(self, cycle_index: int) -> None:
        if self.skip_arm:
            self.get_logger().warning("Arm handshake skipped")
            return
        self._arm_done = False
        payload = {
            "command": "pick_from_table",
            "station": "detected_near_leg_pair",
            "table_frame": "detected_table_edge",
            "cycle": cycle_index,
        }
        if self._last_detection is not None:
            payload["leg_midpoint_mission"] = [round(v, 4) for v in self._last_detection.midpoint]
        command = String()
        command.data = json.dumps(payload, ensure_ascii=False)
        self._arm_pub.publish(command)
        timeout = float(self.config["cycle"]["arm_timeout_sec"])
        self.get_logger().info("Waiting for /tmr_cycle/arm_done=true")
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not self._arm_done and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._arm_done:
            raise TimeoutError("dual-arm pickup did not acknowledge completion")

    def _current_map_pose(self) -> tuple[float, float, float]:
        return self._lookup_frame_pose("base_link")

    def _obstacle_ahead(self, vx: float, vy: float) -> bool:
        scan = self._latest_scan
        speed = math.hypot(vx, vy)
        if scan is None or speed < 1e-3:
            return False
        ux, uy = vx / speed, vy / speed
        stop_distance = float(self.config["strafe"]["obstacle_stop_distance"])
        half_width = float(self.config["strafe"]["obstacle_corridor_half_width"])
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            along = x * ux + y * uy
            lateral = abs(-uy * x + ux * y)
            if 0.0 < along < stop_distance and lateral < half_width:
                return True
        return False

    def _publish_manual(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._manual_pub.publish(msg)

    def stop_manual(self) -> None:
        for _ in range(8):
            self._publish_manual(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def strafe_to_inspect_end(self) -> None:
        target = self.points["inspect_end"]
        target_x, target_y, target_yaw = self._mission_to_map(
            float(target["x"]), float(target["y"]), math.radians(float(target["yaw_deg"]))
        )
        settings = self.config["strafe"]
        deadline = time.monotonic() + float(self.config["cycle"]["strafe_timeout_sec"])
        self.get_logger().info("Start fixed-heading lateral table inspection")
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                x, y, yaw = self._current_map_pose()
                ex, ey = target_x - x, target_y - y
                yaw_error = wrap(target_yaw - yaw)
                if math.hypot(ex, ey) < float(settings["position_tolerance"]) and abs(yaw_error) < math.radians(float(settings["yaw_tolerance_deg"])):
                    return
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                vx = clamp(0.7 * body_x, -float(settings["max_forward_correction"]), float(settings["max_forward_correction"]))
                vy = clamp(0.7 * body_y, -float(settings["max_side_speed"]), float(settings["max_side_speed"]))
                wz = clamp(1.5 * yaw_error, -float(settings["max_yaw_speed"]), float(settings["max_yaw_speed"]))
                if self._obstacle_ahead(vx, vy):
                    raise RuntimeError("obstacle detected in lateral-motion corridor")
                self._publish_manual(vx, vy, wz)
            raise TimeoutError("lateral inspection segment timed out")
        finally:
            self.stop_manual()

    def perception(self, command_text: str) -> None:
        msg = String()
        msg.data = command_text
        self._perception_pub.publish(msg)

    def acquire_pickup_pose(self) -> PoseStamped:
        try:
            return self.detect_pickup_pose()
        except DetectionError as error:
            self.get_logger().error(f"Table-leg recognition rejected: {error}")
            if not self.allow_fallback:
                raise
            self.get_logger().warning("Using configured pickup_fallback because --allow-pickup-fallback was set")
            return self.named_pose("pickup_fallback")

    def verify_current_pair_standoff(
        self,
        expected_standoff: float,
        expected_midpoint: tuple[float, float],
    ) -> dict:
        """Re-detect after navigation and reject a geometrically wrong stop."""
        self.detect_pickup_pose()
        if self._last_detection is None:
            raise DetectionError("no table-leg detection is available for final verification")
        current_x, current_y, current_yaw = self._current_mission_pose()
        midpoint_x, midpoint_y = self._last_detection.midpoint
        midpoint_shift = math.hypot(
            midpoint_x - expected_midpoint[0], midpoint_y - expected_midpoint[1]
        )
        distance_to_midpoint = math.hypot(midpoint_x - current_x, midpoint_y - current_y)
        bearing = math.atan2(midpoint_y - current_y, midpoint_x - current_x)
        bearing_error = wrap(bearing - current_yaw)
        if abs(distance_to_midpoint - expected_standoff) > 0.10:
            raise DetectionError(
                f"final leg standoff is {distance_to_midpoint:.3f} m; "
                f"expected {expected_standoff:.3f} +/- 0.100 m"
            )
        if abs(bearing_error) > math.radians(8.0):
            raise DetectionError(
                f"base is not facing the leg midpoint; error={math.degrees(bearing_error):.1f} deg"
            )
        if midpoint_shift > 0.12:
            raise DetectionError(
                f"post-navigation detection switched or moved by {midpoint_shift:.3f} m"
            )
        report = {
            "distance_to_leg_midpoint_m": distance_to_midpoint,
            "heading_error_deg": math.degrees(bearing_error),
            "redetection_midpoint_shift_m": midpoint_shift,
        }
        self.get_logger().info(
            "Verified left-leg stop: "
            f"distance={distance_to_midpoint:.3f} m, "
            f"heading_error={math.degrees(bearing_error):.1f} deg"
        )
        return report

    def run_cycle(self, cycle_index: int) -> None:
        self.get_logger().info(f"=== cycle {cycle_index} ===")
        self.navigate("table_observe")
        pickup_pose = self.acquire_pickup_pose()
        self.navigate_pose(pickup_pose, "LiDAR-refined pickup")
        self.arm_pick(cycle_index)
        self.navigate("room_exit")
        self.perception("start_table_inspection")
        try:
            self.strafe_to_inspect_end()
        finally:
            self.perception("stop_table_inspection")
        if bool(self.config["cycle"].get("return_to_start", True)):
            self.navigate("p1_start")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=1, help="0 means repeat forever")
    parser.add_argument("--skip-arm", action="store_true", help="base-only dry integration test")
    parser.add_argument("--detect-only", action="store_true", help="navigate to observe point, detect and stop")
    parser.add_argument(
        "--current-left-pair",
        action="store_true",
        help="from the current pose, find the configured table-leg pair on the left and stop facing it",
    )
    parser.add_argument("--left-pair-spacing-m", type=float, default=0.74)
    parser.add_argument("--left-midpoint-forward-m", type=float, default=0.50)
    parser.add_argument("--left-midpoint-distance-m", type=float, default=2.30)
    parser.add_argument("--left-search-forward-half-m", type=float, default=0.70)
    parser.add_argument("--left-search-near-m", type=float, default=1.40)
    parser.add_argument("--left-search-far-m", type=float, default=2.90)
    parser.add_argument("--left-standoff-m", type=float, default=0.55)
    parser.add_argument("--left-pair-axis-deg", type=float, default=90.0)
    parser.add_argument(
        "--allow-pickup-fallback",
        action="store_true",
        help="debug only: use the approximate pickup point when leg recognition fails",
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))

    if args.current_left_pair:
        if args.allow_pickup_fallback:
            parser.error("--current-left-pair never permits --allow-pickup-fallback")
        if not 0.40 <= args.left_pair_spacing_m <= 1.20:
            parser.error("--left-pair-spacing-m must be in [0.40, 1.20]")
        if not 0.35 <= args.left_search_near_m < args.left_search_far_m <= 3.00:
            parser.error("left search distances must satisfy 0.35 <= near < far <= 3.00 m")
        if not 0.45 <= args.left_standoff_m <= 0.80:
            parser.error("--left-standoff-m must be in [0.45, 0.80]")
        if not 0.30 <= args.left_search_forward_half_m <= 1.50:
            parser.error("--left-search-forward-half-m must be in [0.30, 1.50]")
        cfg = config["table_detection"]
        cfg["roi"] = {
            "x": [
                args.left_midpoint_forward_m - args.left_search_forward_half_m,
                args.left_midpoint_forward_m + args.left_search_forward_half_m,
            ],
            "y": [args.left_search_near_m, args.left_search_far_m],
        }
        cfg["expected_pair_spacing"] = args.left_pair_spacing_m
        cfg["pair_spacing_tolerance"] = min(0.20, 0.30 * args.left_pair_spacing_m)
        cfg["expected_pair_axis_deg"] = args.left_pair_axis_deg
        cfg["pair_axis_tolerance_deg"] = 25.0
        cfg["expected_pair_midpoint"] = [
            args.left_midpoint_forward_m,
            args.left_midpoint_distance_m,
        ]
        cfg["max_midpoint_shift"] = min(
            0.80,
            0.5 * (args.left_search_far_m - args.left_search_near_m),
        )
        cfg["approach_standoff"] = args.left_standoff_m
        cfg["max_approach_shift"] = cfg["max_midpoint_shift"] + 0.15
        cfg["max_pair_score"] = 1.60

    rclpy.init()
    node = CycleExecutor(config, args.skip_arm, args.allow_pickup_fallback)
    try:
        node.capture_mission_origin()
        if args.current_left_pair:
            pickup_pose = node.acquire_pickup_pose()
            initial_midpoint = node._last_detection.midpoint
            if args.detect_only:
                return
            node.navigate_pose(pickup_pose, "current left table-leg pair")
            node.verify_current_pair_standoff(args.left_standoff_m, initial_midpoint)
            return
        if args.detect_only:
            node.navigate("table_observe")
            node.acquire_pickup_pose()
            return
        index = 1
        while rclpy.ok() and (args.cycles == 0 or index <= args.cycles):
            node.run_cycle(index)
            index += 1
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")
    finally:
        node.stop_manual()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
