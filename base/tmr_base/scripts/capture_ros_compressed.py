#!/usr/bin/env python3
"""Capture the newest frames from a ROS 2 CompressedImage topic without OpenCV."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Capture(Node):
    def __init__(self, topic: str, requested_frames: int) -> None:
        super().__init__("capture_ros_compressed")
        self.requested_frames = requested_frames
        self.frames: list[bytes] = []
        self.create_subscription(
            CompressedImage,
            topic,
            self._on_frame,
            qos_profile_sensor_data,
        )

    def _on_frame(self, message: CompressedImage) -> None:
        payload = bytes(message.data)
        if len(payload) >= 1024:
            self.frames.append(payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    rclpy.init()
    node = Capture(args.topic, max(1, args.frames))
    try:
        deadline = time.monotonic() + args.timeout_s
        while len(node.frames) < node.requested_frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.frames:
            raise RuntimeError("no compressed image received")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(node.frames[-1])
        print(f"captured_frames={len(node.frames)} bytes={len(node.frames[-1])} output={args.output}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
