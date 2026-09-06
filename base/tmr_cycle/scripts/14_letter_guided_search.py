#!/usr/bin/env python3
"""Move robot-right, find a configured letter card, and center it in ZED RGB.

The base controller and vision loop run in one process on the base computer.
The controller learns the sign of image motion from a short odometry-measured
probe, so camera mounting skew does not need a trusted hand-derived transform.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
from typing import Iterable

from letter_card_vision import LetterCardRecognizer, LetterDetection, annotate


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
CONTROLLER_COMMAND_TOPIC = "/swerve_drive_controller/cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"
DEFAULT_CAMERA_TOPIC = "/head_camera/zed/rgb/color/rect/image/compressed"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def projected_center_right_m(
    observation: "TargetObservation | None",
    image_gain_per_m: float | None,
    maximum_projection_m: float = 0.35,
) -> float | None:
    """Project a verified card centre through a short arm/object occlusion."""
    if observation is None or image_gain_per_m is None or abs(image_gain_per_m) < 0.20:
        return None
    displacement = -(observation.center_x_norm - 0.5) / image_gain_per_m
    if not -0.08 <= displacement <= maximum_projection_m:
        return None
    return observation.right_m + displacement


@dataclass(frozen=True)
class TargetObservation:
    right_m: float
    center_x_norm: float
    row: str
    confidence: float
    members: int


class TargetTracker:
    """Temporal consensus for one physically continuous letter card."""

    def __init__(
        self,
        target_letter: str,
        requested_row: str = "auto",
        history_size: int = 7,
        center_tolerance_norm: float = 0.055,
        stable_frames: int = 3,
        minimum_tracking_confidence: float = 0.42,
    ) -> None:
        target = target_letter.strip().upper()
        if len(target) != 1 or not target.isalpha():
            raise ValueError("target_letter must be one A-Z character")
        if requested_row not in {"auto", "near", "far"}:
            raise ValueError("requested_row must be auto, near, or far")
        self.target_letter = target
        self.requested_row = requested_row
        self.center_tolerance_norm = float(center_tolerance_norm)
        self.stable_frames = int(stable_frames)
        self.minimum_tracking_confidence = float(minimum_tracking_confidence)
        self.history: deque[TargetObservation] = deque(maxlen=int(history_size))
        self.locked_row: str | None = None if requested_row == "auto" else requested_row
        self.consecutive_misses = 0

    def observe(
        self, detections: Iterable[LetterDetection], right_m: float
    ) -> TargetObservation | None:
        matches = [
            item
            for item in detections
            if item.letter == self.target_letter
            and item.confidence >= self.minimum_tracking_confidence
        ]
        if self.requested_row != "auto":
            matches = [item for item in matches if item.row == self.requested_row]
        if not matches:
            self.consecutive_misses += 1
            if self.consecutive_misses >= 2:
                self.history.clear()
            if self.consecutive_misses >= 5 and self.requested_row == "auto":
                # A brief false far/near proposal must not lock the whole scan
                # to the wrong row after that proposal has disappeared.
                self.locked_row = None
            return None
        self.consecutive_misses = 0
        groups: dict[str, list[LetterDetection]] = {"near": [], "far": []}
        for item in matches:
            groups[item.row].append(item)
        if self.locked_row is None:
            row = max(groups, key=lambda name: sum(item.confidence for item in groups[name]))
            recent_rows = [item.row for item in self.history]
            if recent_rows.count(row) >= 1:
                self.locked_row = row
        else:
            row = self.locked_row
        candidates = groups[row]
        if not candidates:
            return None
        # Never average unrelated same-letter proposals.  That previously
        # created a fictitious centred A from several floor/arm rectangles.
        if self.history:
            previous_x = self.history[-1].center_x_norm
            nearby = [
                item for item in candidates
                if abs(item.center_x_norm - previous_x) <= 0.16
            ]
            candidates = nearby or candidates
        selected = max(candidates, key=lambda item: item.confidence)
        observation = TargetObservation(
            right_m=float(right_m),
            center_x_norm=float(selected.center_x_norm),
            row=row,
            confidence=float(selected.confidence),
            members=1,
        )
        self.history.append(observation)
        return observation

    def control_observation(self) -> TargetObservation | None:
        """Return a two-frame, low-noise observation for motion control."""
        if self.consecutive_misses or len(self.history) < 2:
            return None
        recent = list(self.history)[-3:]
        row = recent[-1].row
        recent = [item for item in recent if item.row == row]
        if len(recent) < 2:
            return None
        centers = sorted(item.center_x_norm for item in recent)
        center = centers[len(centers) // 2]
        return TargetObservation(
            right_m=recent[-1].right_m,
            center_x_norm=float(center),
            row=row,
            confidence=float(sum(item.confidence for item in recent) / len(recent)),
            members=max(item.members for item in recent),
        )

    def centered(self) -> tuple[bool, float | None, float | None]:
        if self.consecutive_misses or len(self.history) < self.stable_frames:
            return False, None, None
        recent = list(self.history)[-self.stable_frames :]
        if self.locked_row is not None and any(item.row != self.locked_row for item in recent):
            return False, None, None
        errors = [item.center_x_norm - 0.5 for item in recent]
        mean = sum(errors) / len(errors)
        spread = max(errors) - min(errors)
        stable = abs(mean) <= self.center_tolerance_norm and spread <= 0.032
        return stable, mean, spread


class CenterHold:
    """Stop on a credible centre crossing before motion carries the card away."""

    def __init__(
        self,
        center_tolerance_norm: float,
        acquire_tolerance_norm: float = 0.080,
        grace_s: float = 0.70,
        single_frame_hold_s: float = 0.30,
        single_frame_confidence: float = 0.30,
    ) -> None:
        self.center_tolerance_norm = float(center_tolerance_norm)
        self.acquire_tolerance_norm = max(
            float(acquire_tolerance_norm), self.center_tolerance_norm
        )
        self.grace_s = float(grace_s)
        self.single_frame_hold_s = float(single_frame_hold_s)
        self.single_frame_confidence = float(single_frame_confidence)
        self.samples: deque[TargetObservation] = deque(maxlen=5)
        self.started_at: float | None = None
        self.last_seen_at: float | None = None

    def reset(self) -> None:
        self.samples.clear()
        self.started_at = None
        self.last_seen_at = None

    def update(
        self, observation: TargetObservation | None, now: float
    ) -> tuple[bool, bool, float | None, float | None]:
        if observation is not None:
            error = observation.center_x_norm - 0.5
            if abs(error) <= self.acquire_tolerance_norm:
                if self.started_at is None:
                    self.started_at = float(now)
                    self.samples.clear()
                self.samples.append(observation)
                self.last_seen_at = float(now)
            elif abs(error) > 1.6 * self.acquire_tolerance_norm:
                self.reset()
        return self.status(now)

    def status(self, now: float) -> tuple[bool, bool, float | None, float | None]:
        if self.started_at is None or self.last_seen_at is None or not self.samples:
            return False, False, None, None
        if float(now) - self.last_seen_at > self.grace_s:
            self.reset()
            return False, False, None, None
        errors = [item.center_x_norm - 0.5 for item in self.samples]
        mean = sum(errors) / len(errors)
        spread = max(errors) - min(errors)
        repeated = (
            len(errors) >= 2
            and abs(mean) <= self.center_tolerance_norm
            and spread <= 0.040
        )
        latest = self.samples[-1]
        # The wider acquisition band is only history/hysteresis.  Do not stop
        # the base until the latest observation is actually in the accepted
        # centre band; the old behavior could wait forever at x=0.57.
        holding = abs(latest.center_x_norm - 0.5) <= self.center_tolerance_norm
        credible_crossing = (
            len(errors) >= 2
            and
            float(now) - self.started_at >= self.single_frame_hold_s
            and abs(latest.center_x_norm - 0.5) <= self.center_tolerance_norm
            and latest.confidence >= self.single_frame_confidence
        )
        reported_error = mean if repeated else errors[-1]
        return (
            holding,
            bool(holding and (repeated or credible_crossing)),
            reported_error,
            spread,
        )


class AdaptiveCenterPolicy:
    """Learn d(image-x)/d(robot-right) and close the image-centering loop."""

    def __init__(
        self,
        search_speed_mps: float,
        refine_speed_mps: float,
        minimum_gain_confidence: float = 0.55,
        center_tolerance_norm: float = 0.055,
    ) -> None:
        self.search_speed_mps = float(search_speed_mps)
        self.refine_speed_mps = float(refine_speed_mps)
        self.gain_anchor: TargetObservation | None = None
        # Measured on both B and A passes: moving robot-right shifts a static
        # card left by roughly 0.6--0.8 normalized image-width per metre.
        # Starting with that sign avoids an unnecessary wrong-way probe.
        self.image_gain_per_m: float | None = -0.70
        self.last_direction = 1.0
        self.minimum_gain_confidence = float(minimum_gain_confidence)
        self.center_tolerance_norm = float(center_tolerance_norm)

    def reliable(self, observation: TargetObservation | None) -> bool:
        return (
            observation is not None
            and observation.confidence >= self.minimum_gain_confidence
        )

    def command(self, observation: TargetObservation | None) -> float:
        if observation is not None and not self.reliable(observation):
            # Never let a weak, possibly incorrect OCR result change motion.
            return 0.0
        if observation is None:
            # With no sighting, keep scanning.  After a card has been seen,
            # continue the last low-speed correction so a one-frame loss at
            # the image edge cannot leave the base parked until timeout.
            return (
                self.search_speed_mps
                if self.gain_anchor is None
                else self.last_direction * 0.55 * self.search_speed_mps
            )
        assert observation is not None
        if self.gain_anchor is not None and self.gain_anchor.members == observation.members:
            delta_m = observation.right_m - self.gain_anchor.right_m
            if abs(delta_m) >= 0.018:
                sample = (observation.center_x_norm - self.gain_anchor.center_x_norm) / delta_m
                if 0.04 <= abs(sample) <= 8.0:
                    self.image_gain_per_m = (
                        sample
                        if self.image_gain_per_m is None
                        else 0.65 * self.image_gain_per_m + 0.35 * sample
                    )
                # Keep a spatially separated anchor; updating it every video
                # frame prevented the 18 mm gain sample from ever forming.
                self.gain_anchor = observation
        else:
            self.gain_anchor = observation
        error = observation.center_x_norm - 0.5
        if abs(error) <= self.center_tolerance_norm:
            return 0.0
        if self.image_gain_per_m is None or abs(self.image_gain_per_m) < 0.04:
            # A stationary scene moves left in this forward-facing camera
            # when the robot translates right.  Use that nominal sign for
            # the short learning probe, including cards first seen left of
            # centre; measured odometry/image gain takes over after 18 mm.
            self.last_direction = math.copysign(1.0, error)
            return self.last_direction * 0.55 * self.search_speed_mps
        desired_displacement = -error / self.image_gain_per_m
        speed = clamp(0.65 * desired_displacement, -self.refine_speed_mps, self.refine_speed_mps)
        if 0.0 < abs(speed) < 0.018:
            speed = math.copysign(0.018, speed)
        self.last_direction = math.copysign(1.0, speed)
        return speed


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def run_ros(args: argparse.Namespace) -> dict:
    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool

    class SearchNode(Node):
        def __init__(self) -> None:
            super().__init__("tmr_letter_guided_search")
            self.pose = None
            self.odom_at = 0.0
            self.frame_bytes: bytes | None = None
            self.frame_sequence = 0
            self.frame_at = 0.0
            self.frame_content_at = 0.0
            self.frame_file_mtime_ns = -1
            self.command = [0.0, 0.0, 0.0]
            command_topic = CONTROLLER_COMMAND_TOPIC if args.direct_controller else COMMAND_TOPIC
            self.command_pub = self.create_publisher(TwistStamped, command_topic, 10)
            self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
            self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, qos_profile_sensor_data)
            if args.camera_file is None:
                self.create_subscription(
                    CompressedImage, args.camera_topic, self.on_frame, qos_profile_sensor_data
                )

        def on_odom(self, message) -> None:
            position = message.pose.pose.position
            self.pose = (
                float(position.x),
                float(position.y),
                yaw_from_quaternion(message.pose.pose.orientation),
            )
            self.odom_at = time.monotonic()

        def on_frame(self, message) -> None:
            payload = bytes(message.data)
            now = time.monotonic()
            self.frame_at = now
            if payload == self.frame_bytes:
                return
            self.frame_bytes = payload
            self.frame_sequence += 1
            self.frame_content_at = now

        def refresh_frame_file(self) -> None:
            if args.camera_file is None:
                return
            try:
                stat = args.camera_file.stat()
                if stat.st_mtime_ns == self.frame_file_mtime_ns:
                    return
                payload = args.camera_file.read_bytes()
            except (FileNotFoundError, OSError):
                return
            if payload:
                now = time.monotonic()
                self.frame_file_mtime_ns = stat.st_mtime_ns
                self.frame_at = now
                if payload != self.frame_bytes:
                    self.frame_bytes = payload
                    self.frame_sequence += 1
                    self.frame_content_at = now

        def publish(self, vx: float, vy: float, wz: float) -> None:
            if not args.direct_controller:
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

        def stop(self, samples: int = 24) -> None:
            self.command[:] = (0.0, 0.0, 0.0)
            for _ in range(samples):
                self.publish(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.025)

        def ready(self) -> None:
            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                self.refresh_frame_file()
                if (
                    self.pose is not None
                    and self.frame_bytes is not None
                    and self.frame_sequence >= 2
                    and time.monotonic() - self.odom_at <= 0.4
                    and time.monotonic() - self.frame_at <= 0.8
                    and self.command_pub.get_subscription_count() == 1
                ):
                    self.stop(6)
                    return
            raise RuntimeError("fresh odometry, ZED RGB, or exclusive velocity adapter unavailable")

        def fresh_pose(self):
            if self.pose is None or time.monotonic() - self.odom_at > 0.4:
                raise RuntimeError("odometry became stale")
            return self.pose

    rclpy.init()
    node = SearchNode()
    tracker = TargetTracker(
        args.target_letter,
        args.row,
        center_tolerance_norm=args.center_tolerance_norm,
        stable_frames=args.stable_frames,
        minimum_tracking_confidence=args.minimum_tracking_confidence,
    )
    center_hold = CenterHold(
        args.center_tolerance_norm,
        acquire_tolerance_norm=args.center_acquire_norm,
        single_frame_hold_s=args.center_hold_s,
    )
    policy = AdaptiveCenterPolicy(
        args.search_speed_mps,
        args.refine_speed_mps,
        minimum_gain_confidence=args.minimum_tracking_confidence,
        center_tolerance_norm=args.center_tolerance_norm,
    )
    recognizer = LetterCardRecognizer(
        alphabet=args.alphabet,
        minimum_confidence=args.minimum_confidence,
        row_split_y_norm=args.row_split_y_norm,
    )
    start = None
    latest_observation = None
    latest_frame = None
    latest_detections = []
    last_processed_sequence = -1
    decoded_frames = 0
    target_frames = 0
    last_target_at = None
    last_target_right_m = None
    next_report = 0.0
    last_tick = time.monotonic()
    max_right_seen = 0.0
    motion_anchor = None
    motion_anchor_at = time.monotonic()
    holding_center = False
    hold_centered = False
    hold_error = None
    hold_spread = None
    projection_target_right_m = None
    projection_source_right_m = None
    projection_source_x_norm = None
    projection_row = None
    projection_frame = None
    projection_detections = []
    projected_completion = False
    try:
        node.ready()
        start = node.fresh_pose()
        start_x, start_y, start_yaw = start
        right_axis = (math.sin(start_yaw), -math.cos(start_yaw))
        forward_axis = (math.cos(start_yaw), math.sin(start_yaw))
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.refresh_frame_file()
            x, y, yaw = node.fresh_pose()
            dx, dy = x - start_x, y - start_y
            right_m = dx * right_axis[0] + dy * right_axis[1]
            forward_drift = dx * forward_axis[0] + dy * forward_axis[1]
            max_right_seen = max(max_right_seen, right_m)
            if node.frame_bytes is None or time.monotonic() - node.frame_at > 0.9:
                raise RuntimeError("ZED RGB stream became stale")
            if time.monotonic() - node.frame_content_at > 1.2:
                node.stop(18)
                raise RuntimeError("ZED RGB content stopped advancing; refusing repeated old frames")
            if node.frame_sequence != last_processed_sequence:
                last_processed_sequence = node.frame_sequence
                array = np.frombuffer(node.frame_bytes, np.uint8)
                frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("ZED compressed frame decode failed")
                decoded_frames += 1
                latest_frame = frame
                detections = (
                    recognizer.detect(frame)
                    if right_m >= args.min_detection_right_m
                    else []
                )
                latest_detections = detections
                latest_observation = tracker.observe(detections, right_m)
                if right_m < args.min_detection_right_m:
                    center_hold.reset()
                holding_center, hold_centered, hold_error, hold_spread = center_hold.update(
                    latest_observation, time.monotonic()
                )
                if latest_observation is not None:
                    target_frames += 1
                    print(json.dumps({
                        "event": "target_observed",
                        "target": args.target_letter,
                        "row": latest_observation.row,
                        "center_x_norm": latest_observation.center_x_norm,
                        "confidence": latest_observation.confidence,
                        "members": latest_observation.members,
                        "right_m": right_m,
                    }), flush=True)
            holding_center, hold_centered, hold_error, hold_spread = center_hold.status(
                time.monotonic()
            )
            centered, mean_error, spread = tracker.centered()
            if not centered and hold_centered:
                centered, mean_error, spread = True, hold_error, hold_spread
            projected_completion = bool(
                not centered
                and latest_observation is None
                and projection_target_right_m is not None
                and projection_source_right_m is not None
                and abs(right_m - projection_source_right_m) <= 0.36
                and abs(projection_target_right_m - right_m) <= 0.012
            )
            if projected_completion:
                centered, mean_error, spread = True, 0.0, 0.0
            if centered:
                node.stop(30)
                evidence_saved = False
                evidence_frame = projection_frame if projected_completion else latest_frame
                evidence_detections = (
                    projection_detections if projected_completion else latest_detections
                )
                if args.evidence_image is not None and evidence_frame is not None:
                    args.evidence_image.parent.mkdir(parents=True, exist_ok=True)
                    temporary = args.evidence_image.with_name(
                        args.evidence_image.stem + ".tmp" + args.evidence_image.suffix
                    )
                    if not cv2.imwrite(
                        str(temporary), annotate(evidence_frame, evidence_detections)
                    ):
                        raise RuntimeError("failed to write letter authorization evidence")
                    os.replace(temporary, args.evidence_image)
                    evidence_saved = True
                # The task geometry requires the base to finish 8 cm to the
                # right of the visually centred card.  Perform this only after
                # the OCR/temporal authorization has completed, using odometry
                # so later image dropouts cannot cause oscillation.
                offset_start_x, offset_start_y, _ = node.fresh_pose()
                offset_start_right = (
                    (offset_start_x - start_x) * right_axis[0]
                    + (offset_start_y - start_y) * right_axis[1]
                )
                offset_target_right = offset_start_right + args.post_center_right_m
                if offset_target_right > args.max_right_m + 0.001:
                    raise RuntimeError("post-center right offset exceeds configured right limit")
                offset_deadline = time.monotonic() + max(
                    4.0, args.post_center_right_m / 0.035 + 3.0
                )
                offset_stable_since = None
                offset_last_tick = time.monotonic()
                while args.post_center_right_m > 0.001 and time.monotonic() < offset_deadline:
                    rclpy.spin_once(node, timeout_sec=0.025)
                    x, y, yaw = node.fresh_pose()
                    dx, dy = x - start_x, y - start_y
                    right_now = dx * right_axis[0] + dy * right_axis[1]
                    forward_now = dx * forward_axis[0] + dy * forward_axis[1]
                    remaining = offset_target_right - right_now
                    if abs(remaining) <= 0.012:
                        node.publish(0.0, 0.0, 0.0)
                        offset_stable_since = offset_stable_since or time.monotonic()
                        if time.monotonic() - offset_stable_since >= 0.20:
                            break
                        continue
                    offset_stable_since = None
                    desired = (
                        clamp(-0.85 * forward_now, -0.025, 0.025),
                        -clamp(0.90 * remaining, -0.035, 0.035),
                        clamp(1.15 * wrap(start_yaw - yaw), -0.07, 0.07),
                    )
                    now = time.monotonic()
                    dt = clamp(now - offset_last_tick, 0.02, 0.10)
                    offset_last_tick = now
                    for index, (target, acceleration) in enumerate(
                        zip(desired, (0.15, 0.15, 0.28))
                    ):
                        step = acceleration * dt
                        node.command[index] += clamp(target - node.command[index], -step, step)
                    node.publish(*node.command)
                else:
                    if args.post_center_right_m > 0.001:
                        raise RuntimeError("post-center right offset timed out")
                node.stop(30)
                end_x, end_y, end_yaw = node.fresh_pose()
                actual_right = (end_x - start_x) * right_axis[0] + (end_y - start_y) * right_axis[1]
                return {
                    "status": "success",
                    "target_centered": True,
                    "target_letter": args.target_letter,
                    "row": projection_row if projected_completion else tracker.locked_row,
                    "authorization_mode": (
                        "verified_card_projected_through_occlusion"
                        if projected_completion
                        else (
                            "plate_facing_d_direct"
                            if args.plate_direct_place_on_d
                            else "live_visual_center"
                        )
                    ),
                    "projection_source_right_m": projection_source_right_m,
                    "projection_source_x_norm": projection_source_x_norm,
                    "projected_center_right_m": projection_target_right_m,
                    "actual_right_m": actual_right,
                    "maximum_right_m": max_right_seen,
                    "center_error_norm": mean_error,
                    "center_spread_norm": spread,
                    "post_center_right_requested_m": args.post_center_right_m,
                    "post_center_right_actual_m": actual_right - offset_start_right,
                    "decoded_frames": decoded_frames,
                    "target_frames": target_frames,
                    "minimum_detection_right_m": args.min_detection_right_m,
                    "evidence_image": str(args.evidence_image) if args.evidence_image else None,
                    "evidence_saved": evidence_saved,
                    "image_gain_per_m": policy.image_gain_per_m,
                    "zero_command_latched": True,
                    "start_odom": [start_x, start_y, start_yaw],
                    "end_odom": [end_x, end_y, end_yaw],
                    "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
                }
            control_observation = tracker.control_observation()
            if policy.reliable(control_observation):
                last_target_at = time.monotonic()
                last_target_right_m = right_m
            right_speed = 0.0 if holding_center else policy.command(control_observation)
            if (
                policy.reliable(control_observation)
                and len(tracker.history) >= 3
            ):
                candidate = projected_center_right_m(
                    control_observation, policy.image_gain_per_m
                )
                if (
                    candidate is not None
                    and -args.max_left_m <= candidate <= args.max_right_m
                ):
                    projection_target_right_m = candidate
                    projection_source_right_m = control_observation.right_m
                    projection_source_x_norm = control_observation.center_x_norm
                    projection_row = control_observation.row
                    projection_frame = latest_frame.copy() if latest_frame is not None else None
                    projection_detections = list(latest_detections)
            if (
                not holding_center
                and last_target_at is not None
                and control_observation is None
            ):
                projection_remaining = (
                    None
                    if projection_target_right_m is None
                    else projection_target_right_m - right_m
                )
                if (
                    projection_remaining is not None
                    and projection_source_right_m is not None
                    and abs(right_m - projection_source_right_m) <= 0.36
                ):
                    # A carried object can hide the card just before it reaches
                    # image centre.  Preserve the multi-frame visual lock and
                    # close only the short learned displacement with odometry.
                    if abs(projection_remaining) <= 0.012:
                        right_speed = 0.0
                    else:
                        right_speed = clamp(
                            0.65 * projection_remaining,
                            -args.refine_speed_mps,
                            args.refine_speed_mps,
                        )
                        if 0.0 < abs(right_speed) < 0.018:
                            right_speed = math.copysign(0.018, right_speed)
                else:
                    lost_s = time.monotonic() - last_target_at
                    lost_m = abs(right_m - float(last_target_right_m))
                    if lost_s <= 0.45:
                    # Stop first; do not react to one OCR dropout.
                        right_speed = 0.0
                    elif lost_s <= 2.5 and lost_m <= 0.07:
                    # One bounded, slow continuation reacquires a card that is
                    # briefly hidden by glare or an arm cable.
                        right_speed = policy.last_direction * min(0.022, args.refine_speed_mps)
                    else:
                    # Never continue indefinitely in the last direction.  A
                    # bounded miss falls back to the normal rightward scan.
                        policy.gain_anchor = None
                        right_speed = args.search_speed_mps
            if right_speed > 0.0 and right_m >= args.max_right_m - 0.025:
                node.stop(30)
                raise RuntimeError(
                    f"target {args.target_letter} not centered before {args.max_right_m:.3f} m right limit"
                )
            if right_speed < 0.0 and right_m <= -args.max_left_m + 0.025:
                node.stop(30)
                raise RuntimeError(
                    f"target {args.target_letter} not centered before {args.max_left_m:.3f} m left limit"
                )
            if abs(right_speed) >= 0.018:
                if motion_anchor is None:
                    motion_anchor = (x, y)
                    motion_anchor_at = time.monotonic()
                elif math.hypot(x - motion_anchor[0], y - motion_anchor[1]) >= 0.008:
                    motion_anchor = (x, y)
                    motion_anchor_at = time.monotonic()
                elif time.monotonic() - motion_anchor_at > 3.0:
                    raise RuntimeError("no odometry progress during letter-guided motion")
            else:
                motion_anchor = (x, y)
                motion_anchor_at = time.monotonic()
            desired = (
                clamp(-0.85 * forward_drift, -0.025, 0.025),
                -right_speed,
                clamp(1.15 * wrap(start_yaw - yaw), -0.07, 0.07),
            )
            now = time.monotonic()
            dt = clamp(now - last_tick, 0.02, 0.10)
            last_tick = now
            for index, (target, acceleration) in enumerate(
                zip(desired, (0.15, 0.15, 0.28))
            ):
                step = acceleration * dt
                node.command[index] += clamp(target - node.command[index], -step, step)
            node.publish(*node.command)
            if now >= next_report:
                print(json.dumps({
                    "event": "search_progress",
                    "right_m": right_m,
                    "right_speed_mps": right_speed,
                    "target_seen": latest_observation is not None,
                    "gain": policy.image_gain_per_m,
                }), flush=True)
                next_report = now + 1.0
        raise TimeoutError("letter-guided search timed out")
    finally:
        try:
            node.stop(30)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--direct-controller",
        action="store_true",
        help="publish directly to the active swerve controller; the velocity adapter must be stopped",
    )
    parser.add_argument("--target-letter", default="B")
    parser.add_argument("--row", choices=("auto", "near", "far"), default="auto")
    parser.add_argument("--alphabet", default="ABE")
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument(
        "--camera-file",
        type=Path,
        help="read atomically refreshed JPEG frames from this file instead of subscribing in the control DDS domain",
    )
    parser.add_argument("--max-right-m", type=float, default=2.40)
    parser.add_argument("--max-left-m", type=float, default=0.80)
    parser.add_argument(
        "--min-detection-right-m",
        type=float,
        default=0.40,
        help="ignore OCR before reaching the empirically verified card-view region",
    )
    parser.add_argument("--search-speed-mps", type=float, default=0.055)
    parser.add_argument("--refine-speed-mps", type=float, default=0.040)
    parser.add_argument("--center-tolerance-norm", type=float, default=0.055)
    parser.add_argument("--center-acquire-norm", type=float, default=0.080)
    parser.add_argument("--center-hold-s", type=float, default=0.30)
    parser.add_argument("--post-center-right-m", type=float, default=0.08)
    parser.add_argument(
        "--plate-direct-place-on-d",
        action="store_true",
        help=(
            "while carrying the plate, accept two consecutive verified D-card "
            "frames in the plate-facing image band and do not add a right offset"
        ),
    )
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--minimum-confidence", type=float, default=0.22)
    parser.add_argument("--minimum-tracking-confidence", type=float, default=0.42)
    parser.add_argument("--row-split-y-norm", type=float, default=0.52)
    parser.add_argument("--timeout-s", type=float, default=75.0)
    parser.add_argument("--state-file", type=Path)
    parser.add_argument("--evidence-image", type=Path)
    args = parser.parse_args()
    args.target_letter = args.target_letter.strip().upper()
    if len(args.target_letter) != 1 or args.target_letter not in args.alphabet.upper():
        parser.error("target-letter must be one character included in --alphabet")
    if not 0.20 <= args.max_right_m <= 2.40:
        parser.error("max-right-m must be in [0.20, 2.40]")
    if not 0.05 <= args.max_left_m <= 1.20:
        parser.error("max-left-m must be in [0.05, 1.20]")
    if not 0.0 <= args.min_detection_right_m < args.max_right_m:
        parser.error("min-detection-right-m must be in [0, max-right-m)")
    if not args.minimum_confidence <= args.minimum_tracking_confidence <= 0.85:
        parser.error("minimum-tracking-confidence must be >= minimum-confidence and <= 0.85")
    if not 0.01 <= args.center_tolerance_norm <= 0.10:
        parser.error("center-tolerance-norm must be in [0.01, 0.10]")
    if not 0.0 <= args.post_center_right_m <= 0.20:
        parser.error("post-center-right-m must be in [0.00, 0.20]")
    if args.plate_direct_place_on_d:
        if args.target_letter != "D":
            parser.error("plate-direct-place-on-d is valid only for target D")
        # With the large plate held in front of the arm, D is physically under
        # the plate when its card centre lies within this wider camera band.
        # The white-card/corner/topology checks still run first, and two fresh
        # frames are required; only the camera-centre requirement is relaxed.
        args.center_tolerance_norm = 0.22
        args.center_acquire_norm = 0.22
        args.stable_frames = 2
        args.post_center_right_m = 0.0
    else:
        # Older mission configs used 0.070 for fast acceptance, which allowed a
        # visibly off-centre card to pass.  Keep compatibility with a coordinator
        # that already loaded that config, but never execute looser than 0.055.
        args.center_tolerance_norm = min(args.center_tolerance_norm, 0.055)
    return args


def dry_run(args: argparse.Namespace) -> dict:
    return {
        "status": "dry_run",
        "motion_enabled": False,
        "target_letter": args.target_letter,
        "requested_row": args.row,
        "maximum_right_m": args.max_right_m,
        "minimum_detection_right_m": args.min_detection_right_m,
        "post_center_right_m": args.post_center_right_m,
        "camera_topic": args.camera_topic,
        "camera_file": str(args.camera_file) if args.camera_file else None,
        "direct_controller": args.direct_controller,
        "vision": "white-card HSV -> perspective rectify -> glyph template -> temporal consensus",
        "centering": "online image-motion gain from odometry probe; signed low-speed refinement",
        "depth_required": False,
    }


def main() -> int:
    args = parse_args()
    if not args.execute:
        print(json.dumps(dry_run(args), indent=2), flush=True)
        return 0
    try:
        result = run_ros(args)
        if args.state_file:
            args.state_file.parent.mkdir(parents=True, exist_ok=True)
            args.state_file.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        report = {"status": "failed", "error": repr(exc), "zero_command_latched": True}
        if args.state_file:
            args.state_file.parent.mkdir(parents=True, exist_ok=True)
            args.state_file.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
