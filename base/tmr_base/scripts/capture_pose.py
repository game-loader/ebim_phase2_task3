#!/usr/bin/env python3
"""One-time teaching of route points relative to the physical start pose."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument(
        "point",
        choices=["p1_start", "table_observe", "pickup_fallback", "room_exit", "inspect_end"],
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    data = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = Node("tmr_relative_route_pose_capture")
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841
    deadline = time.monotonic() + args.timeout
    transform = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                str(data.get("frame_id", "map")), "base_link", Time(), Duration(seconds=0.2)
            )
            break
        except Exception:
            continue
    if transform is None:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("No map -> base_link TF. Start live SLAM first.")

    t = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    if args.point == "p1_start":
        data["calibration_origin_map"] = {"x": float(t.x), "y": float(t.y), "yaw": yaw}
        recorded = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    else:
        if "calibration_origin_map" not in data:
            raise SystemExit("Record p1_start first in the same SLAM session.")
        origin = data["calibration_origin_map"]
        dx, dy = float(t.x) - float(origin["x"]), float(t.y) - float(origin["y"])
        origin_yaw = float(origin["yaw"])
        recorded = {
            "x": round(math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy, 4),
            "y": round(-math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy, 4),
            "yaw_deg": round(math.degrees(wrap(yaw - origin_yaw)), 2),
        }
    data["points"][args.point] = recorded
    args.config.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print(f"Recorded start-relative {args.point}: {recorded}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
