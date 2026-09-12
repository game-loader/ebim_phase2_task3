#!/usr/bin/env python3
"""Passively record green START in the current base odometry session.

Place the base center at the preview's green START, facing its +x direction.
--capture-at-green-start confirms this placement. This command has no publishers.
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

from far_leg_target import driver_session, wrap

ROOT = Path(__file__).resolve().parents[1]


def stationary_window(samples):
    if len(samples) < 10 or samples[-1][0] - samples[0][0] < 0.9:
        return False
    first = samples[0][1]
    return all(
        math.hypot(p[0] - first[0], p[1] - first[1]) <= 0.005
        and abs(wrap(p[2] - first[2])) <= math.radians(0.5)
        and math.hypot(v[0], v[1]) < 0.01
        and abs(v[2]) < 0.02
        for _, p, v in samples
    )


def capture(path):
    if path.exists():
        raise RuntimeError("origin file exists; use a new run directory")
    session = driver_session()
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node("tmr_capture_green_start")
    samples = deque()
    frame = None
    fault = None

    def receive(msg):
        nonlocal frame, fault
        current_frame = msg.header.frame_id.lstrip("/")
        # Installed raw driver leaves child_frame_id empty; pose is base_link.
        if not current_frame or msg.child_frame_id.lstrip("/") not in ("", "base_link"):
            fault = "origin needs raw odometry for base_link"
            return
        if frame is not None and current_frame != frame:
            fault = "odometry frame changed"
        frame = current_frame
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = node.get_clock().now().nanoseconds * 1e-9
        q, p, v = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        pose = (
            p.x,
            p.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        )
        velocity = (v.linear.x, v.linear.y, v.angular.z)
        if (
            not all(math.isfinite(x) for x in (*pose, *velocity, norm))
            or abs(norm - 1) > 0.02
            or not 0 <= now - stamp <= 0.30
        ):
            fault = "invalid/stale raw odometry"
            return
        if samples and stamp <= samples[-1][0]:
            return  # Replayed samples cannot confirm stationarity.
        samples.append((stamp, pose, velocity))
        while len(samples) > 1 and stamp - samples[0][0] > 1.1:
            samples.popleft()

    node.create_subscription(
        Odometry, "/swerve_drive_controller/odom", receive, qos_profile_sensor_data
    )
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if fault:
                raise RuntimeError(fault)
            if stationary_window(samples):
                if driver_session() != session:
                    raise RuntimeError("base driver restarted during capture")
                result = dict(
                    version=1,
                    reference="green_start",
                    pose_odom=samples[-1][1],
                    odom_frame=frame,
                    base_frame="base_link",
                    driver_session=session,
                    stationary_confirmed=True,
                    stamp=samples[-1][0],
                    physical_start_confirmed_by_operator=True,
                )
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("x") as handle:
                    json.dump(result, handle, indent=2, allow_nan=False)
                print(json.dumps(result, indent=2))
                return
        raise TimeoutError("no fresh stationary odometry at green START")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-at-green-start", action="store_true")
    parser.add_argument(
        "--state-file", type=Path, default=ROOT / "state/leg_roi_origin.json"
    )
    args = parser.parse_args()
    if not args.capture_at_green_start:
        print(
            json.dumps(
                dict(
                    status="dry_run",
                    motion_enabled=False,
                    reference="green_start",
                    instruction="place base at green START with matching heading, then --capture-at-green-start",
                )
            )
        )
        return
    capture(args.state_file.resolve())


if __name__ == "__main__":
    main()
