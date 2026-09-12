"""Live dual-scan evidence and a bounded, front-edge-aligned forward approach.

ROS imports are deferred; the controller only uses the existing mission relay.
There is no collision guard or fixed-distance fallback in this module.
"""

from collections import deque
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

from far_leg_target import (
    DetectionError,
    EvidenceWindow,
    ScanFrame,
    StableLegTarget,
    alignment_error,
    forward_speed,
    in_reference,
    interpolate_pose,
    project_scan,
    wrap,
)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class LiveLegSensors:
    def __init__(self, node, origin, cfg):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.node, self.origin, self.cfg = node, origin, cfg
        self.window = EvidenceWindow(cfg)
        self.target = StableLegTarget(cfg)
        self.pending = deque(maxlen=120)
        self.received = {}
        self.frame_ids = {}
        self.extrinsics = {}
        self.fault = None
        self.last_detection = None
        self.reason = "collecting independent scans"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        spec = importlib.util.spec_from_file_location(
            "tmr_leg_laser_geometry",
            Path(__file__).with_name("05_right_turn_map_gap.py"),
        )
        self.geometry = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = self.geometry
        spec.loader.exec_module(self.geometry)
        self.subscriptions = [
            node.create_subscription(
                LaserScan,
                topic,
                lambda msg, source=topic: self.receive(source, msg),
                qos_profile_sensor_data,
            )
            for topic in cfg["scan_topics"]
        ]

    def ros_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def receive(self, source, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        frame = message.header.frame_id.lstrip("/")
        if not frame or (source in self.frame_ids and self.frame_ids[source] != frame):
            self.fault = "LiDAR frame changed or is empty"
            return
        if stamp < self.received.get(source, 0):
            self.fault = "LiDAR timestamp moved backwards"
            return
        if stamp == self.received.get(source):
            return
        if not 0 <= self.ros_now() - stamp <= self.cfg["fresh_s"]:
            return
        if (
            not all(
                math.isfinite(v)
                for v in (
                    message.angle_min,
                    message.angle_increment,
                    message.range_min,
                    message.range_max,
                )
            )
            or message.range_min < 0
            or message.range_max <= message.range_min
            or message.angle_increment == 0
            or not message.ranges
        ):
            self.fault = "invalid LiDAR scan geometry"
            return
        self.frame_ids[source] = frame
        self.received[source] = stamp
        self.pending.append((source, stamp, message))

    def mount(self, frame):
        from rclpy.time import Time

        if frame in self.extrinsics:
            return self.extrinsics[frame]
        if frame == "base_link":
            result = self.geometry.LaserExtrinsic(0, 0, 1, 0, 0, 1, "identity")
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", frame, Time()
                ).transform
            except Exception:
                result = self.geometry.measured_lidar_extrinsic(frame)
                if result is None:
                    raise RuntimeError(f"no verified base_link transform for {frame}")
            else:
                matrix = self.geometry.quaternion_xy_projection(tf.rotation)
                result = self.geometry.LaserExtrinsic(
                    tf.translation.x, tf.translation.y, *matrix, "tf_full_3d_projection"
                )
        if not all(
            math.isfinite(v)
            for v in (
                result.tx,
                result.ty,
                result.r00,
                result.r01,
                result.r10,
                result.r11,
            )
        ):
            raise RuntimeError("invalid LiDAR mounting transform")
        self.extrinsics[frame] = result
        return result

    def healthy(self):
        if self.fault:
            raise RuntimeError(self.fault)
        now = self.ros_now()
        for topic in self.cfg["scan_topics"]:
            if self.node.count_publishers(topic) != 1:
                raise DetectionError(f"need one raw scan publisher: {topic}")
            if not 0 <= now - self.received.get(topic, 0) <= self.cfg["fresh_s"]:
                raise DetectionError(f"waiting for fresh scan: {topic}")
            self.mount(self.frame_ids[topic])

    def reset_evidence(self):
        self.pending.clear()
        self.window = EvidenceWindow(self.cfg)
        self.target = StableLegTarget(self.cfg)
        self.last_detection = None

    def sample(self):
        try:
            self.healthy()
            now = self.ros_now()
            waiting = deque(maxlen=120)
            while self.pending:
                source, stamp, scan = self.pending.popleft()
                if now - stamp > self.cfg["fresh_s"]:
                    continue
                pose = interpolate_pose(self.node.odom_history, stamp)
                if pose is None:
                    waiting.append((source, stamp, scan))
                    continue
                points = project_scan(
                    scan,
                    self.mount(scan.header.frame_id.lstrip("/")),
                    pose,
                    self.origin["pose_odom"],
                    self.cfg,
                )
                self.window.add(ScanFrame(source, stamp, points))
            self.pending = waiting
            detection = self.window.detect(now)
            if detection["newest_stamp"] > self.target.last_stamp:
                self.last_detection = self.target.update(detection)
            if self.last_detection is None:
                self.reason = "waiting for stable leg identities"
            return self.last_detection
        except DetectionError as exc:
            self.target.missing()
            self.last_detection = None
            self.reason = str(exc)
            return None


def approach_far_leg(node, sensors, cfg, now=time.monotonic):
    """Hold zero whenever evidence is unavailable, and never change leg identity."""
    node.wait_stationary()
    start = node._fresh_pose()
    origin = sensors.origin["pose_odom"]
    start_roi = in_reference(start, origin)
    # Check heading before acquiring evidence, even when the target is hidden.
    alignment_error(start_roi, {"x": start_roi[0]}, cfg)
    sensors.reset_evidence()
    began = now()
    deadline = began + cfg["approach_timeout_s"]
    last_tick, missing_since, progress_at = began, began, began
    best_x = start_roi[0]
    acquired = False
    settled_at = None
    next_report = 0.0
    last_detection = None
    try:
        while now() < deadline:
            node.spin()
            pose = node._fresh_pose()
            current = in_reference(pose, origin)
            advance = current[0] - start_roi[0]
            if advance > cfg["maximum_forward_m"] or advance < -0.05:
                raise RuntimeError("far-leg approach exceeded allowed forward travel")
            if abs(current[1] - start_roi[1]) > 0.10:
                raise RuntimeError("far-leg approach left its lateral path")
            detection = sensors.sample()
            if detection is None:
                node.stop(1)  # Immediate zero, not the acceleration ramp.
                settled_at = None
                missing_since = missing_since if missing_since is not None else now()
                limit = (
                    cfg["lost_target_timeout_s"]
                    if acquired
                    else cfg["acquire_timeout_s"]
                )
                if now() - missing_since > limit:
                    raise TimeoutError(f"leg evidence unavailable: {sensors.reason}")
                progress_at, best_x, last_tick = now(), current[0], now()
                continue
            last_detection = detection
            acquired, missing_since = True, None
            error, front_x = alignment_error(current, detection["far"], cfg)
            speed = forward_speed(error, cfg)
            if advance + max(0.0, error) > cfg["maximum_forward_m"]:
                raise RuntimeError("detected far leg requires too much forward travel")
            yaw_error = wrap(start[2] - pose[2])
            lateral_error = start_roi[1] - current[1]
            if speed == 0:
                node.stop(1)
                if abs(lateral_error) > 0.025 or abs(yaw_error) > math.radians(1):
                    raise RuntimeError(
                        "front aligned but path/heading is outside tolerance"
                    )
                settled_at = settled_at or now()
                if now() - settled_at >= 0.4:
                    node.wait_stationary()
                    # Stationary confirmation takes time; require fresh scan evidence again.
                    final_detection = sensors.sample()
                    final_pose = node._fresh_pose()
                    if final_detection is None:
                        settled_at = None
                        continue
                    final_error, final_front = alignment_error(
                        in_reference(final_pose, origin), final_detection["far"], cfg
                    )
                    if abs(final_error) > cfg["front_alignment_tolerance_m"]:
                        settled_at = None
                        continue
                    return dict(
                        kind="live_far_leg_front_alignment",
                        target=final_detection,
                        actual_forward_m=in_reference(final_pose, origin)[0]
                        - start_roi[0],
                        front_x_roi_m=final_front,
                        alignment_error_m=final_error,
                        start_odom=list(start),
                        end_odom=list(final_pose),
                        stationary_confirmed=True,
                    )
                continue
            settled_at = None
            if current[0] > best_x + 0.005:
                best_x, progress_at = current[0], now()
            elif now() - progress_at > 4:
                raise RuntimeError("far-leg approach made no odometry progress for 4 s")
            # Travel along green START +x, hold the post-left y and original heading.
            vy_roi = clamp(0.8 * lateral_error, 0.025)
            c, s = math.cos(current[2]), math.sin(current[2])
            desired = (
                c * speed + s * vy_roi,
                -s * speed + c * vy_roi,
                clamp(1.2 * yaw_error, 0.06),
            )
            last_tick = node._tick(desired, last_tick)
            if now() >= next_report:
                print(
                    json.dumps(
                        dict(
                            event="far_leg_progress",
                            target=last_detection,
                            front_x_roi_m=front_x,
                            remaining_m=error,
                        )
                    ),
                    flush=True,
                )
                next_report = now() + 1
        raise TimeoutError("far-leg approach timed out")
    finally:
        node.stop()
