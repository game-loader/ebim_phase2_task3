#!/usr/bin/env python3
"""Export the newest compressed ZED RGB frame to an atomic JPEG file."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrameExporter(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__("tmr_zed_frame_export")
        self.output = output
        self.temporary = output.with_name(output.name + ".tmp")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.create_subscription(CompressedImage, topic, self.on_frame, qos_profile_sensor_data)

    def on_frame(self, message: CompressedImage) -> None:
        payload = bytes(message.data)
        if not payload:
            return
        self.temporary.write_bytes(payload)
        os.replace(self.temporary, self.output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/head_camera/zed/rgb/color/rect/image/compressed",
    )
    parser.add_argument("--output", type=Path, default=Path("/tmp/tmr_zed_latest.jpg"))
    args = parser.parse_args()
    rclpy.init()
    node = FrameExporter(args.topic, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
