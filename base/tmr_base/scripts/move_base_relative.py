#!/usr/bin/env python3
"""Execute guarded odometry-relative left/back translations."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import yaml


def load_bootstrap(path: Path):
    spec = importlib.util.spec_from_file_location("tmr_bootstrap_mapping", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_pose(node, timeout_sec: float = 3.0):
    deadline = time.monotonic() + timeout_sec
    while True:
        try:
            return node._pose()
        except Exception:
            if time.monotonic() >= deadline:
                raise
            rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bootstrap-script", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--left-m", type=float, default=0.0)
    parser.add_argument("--backward-m", type=float, default=0.0)
    parser.add_argument("--ccw-deg", type=float, default=0.0)
    args = parser.parse_args()
    if not 0.0 <= args.left_m <= 0.60:
        raise ValueError("left distance must be in [0, 0.60] m")
    if not 0.0 <= args.backward_m <= 1.20:
        raise ValueError("backward distance must be in [0, 1.20] m")
    if not -180.0 <= args.ccw_deg <= 180.0:
        raise ValueError("rotation must be in [-180, 180] degrees")
    if args.left_m == 0.0 and args.backward_m == 0.0 and args.ccw_deg == 0.0:
        raise ValueError("at least one motion must be non-zero")

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    motion = config["bootstrap_mapping"]
    motion["translate_speed"] = min(0.05, float(motion["translate_speed"]))
    motion["forward_correction_speed"] = 0.05
    motion["position_tolerance"] = 0.015
    motion["translation_timeout_sec"] = 45.0
    motion["translation_corridor_half_width"] = 0.30
    motion["translation_obstacle_distance"] = 0.42
    motion["rotation_deg"] = args.ccw_deg
    motion["rotation_clearance"] = 0.40
    motion["yaw_tolerance_deg"] = 0.30

    module = load_bootstrap(args.bootstrap_script)
    rclpy.init()
    node = module.BootstrapMapper(config)
    latest_odom = {"pose": None}

    def on_odom(msg: Odometry) -> None:
        p = msg.pose.pose.position
        latest_odom["pose"] = (
            float(p.x),
            float(p.y),
            module.yaw_from_quaternion(msg.pose.pose.orientation),
        )

    node.create_subscription(
        Odometry,
        "/swerve_drive_controller/odom",
        on_odom,
        qos_profile_sensor_data,
    )

    def direct_odom_pose():
        if latest_odom["pose"] is None:
            raise RuntimeError("odometry not received yet")
        return latest_odom["pose"]

    node._pose = direct_odom_pose
    try:
        start_x, start_y, start_yaw = node.wait_ready()
        left_x = -math.sin(start_yaw)
        left_y = math.cos(start_yaw)
        forward_x = math.cos(start_yaw)
        forward_y = math.sin(start_yaw)
        if args.left_m > 0.0:
            node.translate_to(
                start_x + left_x * args.left_m,
                start_y + left_y * args.left_m,
                start_yaw,
                "guarded left translation",
            )
        mid_x, mid_y, mid_yaw = read_pose(node)
        if args.backward_m > 0.0:
            node.translate_to(
                start_x + left_x * args.left_m - forward_x * args.backward_m,
                start_y + left_y * args.left_m - forward_y * args.backward_m,
                start_yaw,
                "guarded backward translation",
            )
        before_rotation_x, before_rotation_y, before_rotation_yaw = read_pose(node)
        if args.ccw_deg != 0.0:
            rotation_target = module.wrap(before_rotation_yaw + math.radians(args.ccw_deg))
            if abs(args.ccw_deg) > 5.0:
                node.rotate_once(before_rotation_x, before_rotation_y, before_rotation_yaw)
            node.align_yaw(rotation_target)
        node.stop()
        settle_deadline = time.monotonic() + 0.8
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        end_x, end_y, end_yaw = read_pose(node)
        backward_actual = -(
            (end_x - start_x) * forward_x + (end_y - start_y) * forward_y
        )
        left_actual = (end_x - start_x) * left_x + (end_y - start_y) * left_y
        result = {
            "requested_left_m": args.left_m,
            "actual_left_m": left_actual,
            "requested_backward_m": args.backward_m,
            "actual_backward_m": backward_actual,
            "requested_ccw_deg": args.ccw_deg,
            "actual_ccw_deg": math.degrees(module.wrap(end_yaw - before_rotation_yaw)),
            "yaw_error_deg": math.degrees(module.wrap(end_yaw - start_yaw)),
            "start_odom": [start_x, start_y, start_yaw],
            "after_left_odom": [mid_x, mid_y, mid_yaw],
            "end_odom": [end_x, end_y, end_yaw],
        }
        if abs(left_actual - args.left_m) > 0.03 or abs(backward_actual - args.backward_m) > 0.03:
            raise RuntimeError(f"translation error too large: {json.dumps(result)}")
        print(json.dumps(result, indent=2), flush=True)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
