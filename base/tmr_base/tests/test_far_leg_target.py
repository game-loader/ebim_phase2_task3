"""Focused geometry/evidence tests; no ROS, map server, or robot motion."""

from collections import deque
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from far_leg_target import (  # noqa: E402
    DetectionError,
    EvidenceWindow,
    ScanFrame,
    StableLegTarget,
    alignment_error,
    driver_session,
    forward_speed,
    in_reference,
    interpolate_pose,
    load_roi_origin,
    project_scan,
    read_config,
    to_odom,
)
from live_leg_approach import LiveLegSensors, approach_far_leg  # noqa: E402

CFG = read_config(ROOT / "config/post15_leg_approach.json")


def points(x, y):
    return [(x + dx, y + dy) for dx in (-0.015, 0, 0.015) for dy in (-0.015, 0, 0.015)]


def evidence(centers=((2.0, 0.2), (3.2, 0.25)), count=12):
    result = EvidenceWindow(CFG)
    for i in range(count):
        result.add(
            ScanFrame(
                CFG["scan_topics"][i % 2],
                10 + i * 0.03,
                [p for x, y in centers for p in points(x, y)],
            )
        )
    return result


class LegTests(unittest.TestCase):
    def test_roi_two_legs_select_greatest_x_not_old_transverse_pair(self):
        result = evidence().detect(10.34)
        self.assertAlmostEqual(result["far"]["x"], 3.2, delta=0.025)
        self.assertLess(result["near"]["x"], result["far"]["x"])
        self.assertEqual(result["frames"], 12)

    def test_reject_missing_extra_ambiguous_or_broad_clusters(self):
        for centers in (
            [(2.0, 0.2)],
            [(2, 0.2), (2.6, 0.2), (3.2, 0.2)],
            [(2, 0), (2.05, 0.5)],
            [],
        ):
            with self.subTest(centers=centers), self.assertRaises(DetectionError):
                evidence(centers).detect(10.34)

    def test_no_single_frame_duplicate_or_stale_acceptance(self):
        window = evidence(count=1)
        frame = window.frames[0]
        for _ in range(100):
            self.assertFalse(window.add(frame))
        with self.assertRaises(DetectionError):
            window.detect(10.01)
        with self.assertRaises(DetectionError):
            evidence().detect(10.7)

    def test_each_leg_needs_multiple_fresh_frames(self):
        window = evidence(centers=[(2, 0.2)])
        window.add(ScanFrame(CFG["scan_topics"][0], 10.34, points(3.2, 0.25) * 30))
        with self.assertRaises(DetectionError):
            window.detect(10.35)
        window = evidence()
        for frame in window.frames:
            frame.source = CFG["scan_topics"][0]
        with self.assertRaises(DetectionError):
            window.detect(10.34)

    def test_target_stability_and_identity_never_switch(self):
        target = StableLegTarget(CFG)
        result = evidence().detect(10.34)
        for i in range(3):
            item = deepcopy(result)
            item["newest_stamp"] += i * 0.1
            ready = target.update(item)
            self.assertEqual(ready is not None, i == 2)
        changed = deepcopy(item)
        changed["newest_stamp"] += 0.1
        changed["far"]["x"] += 0.2
        with self.assertRaisesRegex(DetectionError, "identity"):
            target.update(changed)
        for i in range(3):
            item["newest_stamp"] += 0.2
            ready = target.update(deepcopy(item))
            self.assertEqual(ready is not None, i == 2)

    def test_green_start_transform_and_scan_time_interpolation(self):
        origin = (10, 5, math.pi / 2)
        self.assertEqual(to_odom((2, 0.5), origin), (9.5, 7.0))
        roi = in_reference((9.5, 7, math.pi / 2), origin)
        self.assertAlmostEqual(roi[0], 2)
        self.assertAlmostEqual(roi[1], 0.5)
        history = deque([(10, (10, 5, math.pi / 2)), (10.1, (10, 5.1, math.pi / 2))])
        pose = interpolate_pose(history, 10.05)
        self.assertAlmostEqual(pose[1], 5.05)
        self.assertIsNone(interpolate_pose(history, 10.2))
        scan = SimpleNamespace(
            ranges=[1.95],
            range_min=0.1,
            range_max=10,
            angle_min=0,
            angle_increment=0.01,
        )
        mount = SimpleNamespace(tx=0, ty=0.2, project_unit=lambda angle: (1, 0))
        projected = project_scan(scan, mount, pose, origin, CFG)
        self.assertAlmostEqual(projected[0][0], 2)
        self.assertAlmostEqual(projected[0][1], 0.2)
        # Using latest odometry would incorrectly project x=2.05.
        wrapped = interpolate_pose(
            [(0, (0, 0, math.radians(179))), (0.1, (0, 0, math.radians(-179)))], 0.05
        )
        self.assertAlmostEqual(abs(wrapped[2]), math.pi)

    def test_front_edge_is_point_four_ahead_without_old_standoff(self):
        error, front = alignment_error((2.8, 0.7, 0), {"x": 3.2}, CFG)
        self.assertAlmostEqual(front, 3.2)
        self.assertAlmostEqual(error, 0)
        self.assertEqual(forward_speed(error, CFG), 0)
        self.assertEqual(forward_speed(1, CFG), 0.05)
        for yaw in (-0.05, 0.05):
            _, front = alignment_error((2, 0, yaw), {"x": 3.2}, CFG)
            self.assertAlmostEqual(
                front, 2 + 0.4 * math.cos(yaw) + 0.29 * abs(math.sin(yaw))
            )
        with self.assertRaises(DetectionError):
            alignment_error((2, 0, math.pi / 2), {"x": 3.2}, CFG)
        with self.assertRaises(DetectionError):
            forward_speed(-0.03, CFG)

    def test_origin_cannot_be_rebound_to_stage15_or_other_session(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "origin.json"
            origin = dict(
                reference="green_start",
                stationary_confirmed=True,
                pose_odom=[5, 6, 0.7],
                driver_session="a",
                odom_frame="world",
            )
            path.write_text(json.dumps(origin))
            self.assertEqual(load_roi_origin(path, "a", "world"), origin)
            for session, frame in [("b", "world"), ("a", "odom")]:
                with self.assertRaises(ValueError):
                    load_roi_origin(path, session, frame)
            origin["reference"] = "stage15"
            path.write_text(json.dumps(origin))
            with self.assertRaises(ValueError):
                load_roi_origin(path, "a", "world")

    def test_driver_fingerprint_changes_on_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sys/kernel/random").mkdir(parents=True)
            (root / "sys/kernel/random/boot_id").write_text("boot")
            (root / "123").mkdir()
            (root / "123/cmdline").write_bytes(
                b"/opt/ros/humble/lib/controller_manager/ros2_control_node\0"
            )
            stat = root / "123/stat"
            stat.write_text("123 (ros2_control) " + " ".join(["0"] * 19 + ["100"]))
            before = driver_session(root)
            stat.write_text("123 (ros2_control) " + " ".join(["0"] * 19 + ["200"]))
            self.assertNotEqual(before, driver_session(root))

    def test_origin_capture_needs_real_stationarity(self):
        spec = importlib.util.spec_from_file_location(
            "capture_roi", ROOT / "scripts/20_capture_leg_roi_origin.py"
        )
        capture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(capture)
        samples = [(i * 0.05, (1, 2, 0), (0, 0, 0)) for i in range(21)]
        self.assertTrue(capture.stationary_window(samples))
        self.assertFalse(capture.stationary_window(samples[:5]))
        samples[-1] = (1, (1.01, 2, 0), (0, 0, 0))
        self.assertFalse(capture.stationary_window(samples))


class SensorIntegrationTests(unittest.TestCase):
    def test_callbacks_require_scan_time_odom_and_stop_on_stale_scanner(self):
        sensor = LiveLegSensors.__new__(LiveLegSensors)
        sensor.cfg = CFG
        sensor.origin = {"pose_odom": [0, 0, 0]}
        sensor.received, sensor.frame_ids = {}, {}
        sensor.pending = deque(maxlen=120)
        sensor.fault = None
        sensor.reset_evidence()
        sensor.node = SimpleNamespace(
            odom_history=deque([(9.9 + i * 0.05, (i * 0.01, 0, 0)) for i in range(20)]),
            count_publishers=lambda topic: 1,
        )
        sensor.mount = lambda frame: object()
        clock = [10.0]
        sensor.ros_now = lambda: clock[0]
        observed_poses = []

        def projection(scan, mount, pose, origin, cfg):
            observed_poses.append(pose)
            return points(2, 0.2) + points(3.2, 0.25)

        with patch("live_leg_approach.project_scan", side_effect=projection):
            result = None
            for i in range(14):
                stamp = 10 + i * 0.03
                clock[0] = stamp + 0.001
                message = SimpleNamespace(
                    header=SimpleNamespace(
                        stamp=SimpleNamespace(sec=10, nanosec=round(i * 0.03 * 1e9)),
                        frame_id="lidar_front" if i % 2 == 0 else "lidar_rear",
                    ),
                    ranges=[2.0],
                    range_min=0.1,
                    range_max=10.0,
                    angle_min=0.0,
                    angle_increment=0.01,
                )
                sensor.receive(CFG["scan_topics"][i % 2], message)
                result = sensor.sample()
            self.assertIsNotNone(result)
            self.assertAlmostEqual(observed_poses[0][0], 0.02)
            self.assertLess(observed_poses[0][0], sensor.node.odom_history[-1][1][0])
            # Both scans may stay in a one-second window, but stale streams veto motion.
            clock[0] += 0.35
            self.assertIsNone(sensor.sample())
            self.assertIn("fresh scan", sensor.reason)

    def test_no_bracketed_odometry_means_no_target(self):
        sensor = LiveLegSensors.__new__(LiveLegSensors)
        sensor.cfg = CFG
        sensor.origin = {"pose_odom": [0, 0, 0]}
        sensor.pending = deque()
        sensor.reset_evidence()
        sensor.node = SimpleNamespace(
            odom_history=deque([(9, (0, 0, 0)), (9.1, (0, 0, 0))])
        )
        sensor.healthy = lambda: None
        sensor.ros_now = lambda: 10.1
        sensor.pending.append((CFG["scan_topics"][0], 10.05, object()))
        with patch("live_leg_approach.project_scan") as project:
            self.assertIsNone(sensor.sample())
            project.assert_not_called()
        self.assertEqual(len(sensor.pending), 1)


class ApproachFixture:
    """Deterministic command integration to check controller branch behavior."""

    def __init__(self, missing=lambda t: False):
        self.t, self.x = 0.0, 2.0
        self.speed = 0.0
        self.calls = []
        self.origin = {"pose_odom": [0, 0, 0]}
        self.reason = "lost scans"
        self.missing = missing

    def wait_stationary(self):
        self.stop()

    def _fresh_pose(self):
        return (self.x, 0.7, 0)

    def spin(self):
        self.t += 0.05
        self.x += self.speed * 0.05

    def reset_evidence(self):
        pass

    def sample(self):
        if self.missing(self.t):
            return None
        return {"far": {"x": 3.2, "y": 0.25}, "near": {"x": 2, "y": 0.2}}

    def stop(self, samples=18):
        self.speed = 0
        self.calls.append((self.t, "zero"))

    def _tick(self, desired, last_tick):
        self.speed = desired[0]
        self.calls.append((self.t, "move"))
        return self.t


class ApproachTests(unittest.TestCase):
    def test_actual_distance_from_leg_and_zero_on_temporary_loss(self):
        fixture = ApproachFixture(lambda t: 3 < t < 4)
        report = approach_far_leg(fixture, fixture, CFG, now=lambda: fixture.t)
        self.assertAlmostEqual(report["actual_forward_m"], 0.8, delta=0.021)
        self.assertLessEqual(abs(report["alignment_error_m"]), 0.02)
        self.assertFalse(any(kind == "move" for t, kind in fixture.calls if 3 < t < 4))
        self.assertEqual(fixture.speed, 0)

    def test_absent_target_never_uses_nominal_distance(self):
        fixture = ApproachFixture(lambda t: True)
        with self.assertRaisesRegex(TimeoutError, "evidence unavailable"):
            approach_far_leg(fixture, fixture, CFG, now=lambda: fixture.t)
        self.assertEqual(fixture.x, 2)
        self.assertFalse(any(kind == "move" for _, kind in fixture.calls))

    def test_persistent_loss_stops_and_aborts(self):
        fixture = ApproachFixture(lambda t: t > 2)
        with self.assertRaises(TimeoutError):
            approach_far_leg(fixture, fixture, CFG, now=lambda: fixture.t)
        self.assertFalse(any(kind == "move" for t, kind in fixture.calls if t > 2))
        self.assertEqual(fixture.speed, 0)


if __name__ == "__main__":
    unittest.main()
