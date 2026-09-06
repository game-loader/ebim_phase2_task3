"""Exercise the detour geometry and handoff without publishing to ROS."""

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SCRIPT = ROOT / "scripts/20_after_return_placement.py"
SPEC = importlib.util.spec_from_file_location("post15", SCRIPT)
route = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(route)


class FakeMotion:
    """An algebraic odometry fixture, never a robot/sensor simulation runtime."""

    def __init__(self, pose, error_scale=1.0):
        self.pose = list(pose)
        self.linear_speed = 0.08
        self.error_scale = error_scale
        self.calls = []
        self.odom_frame = "world"
        self.detected_forward = 1.57

    def prepare_leg_approach(self, origin, cfg):
        pass

    def approach_far_leg(self):
        report = self.translate(self.detected_forward, 0, 90)
        return {"actual_forward_m": self.detected_forward, **report}

    def _fresh_pose(self):
        return tuple(self.pose)

    def wait_stationary(self):
        self.calls.append(("stop",))

    def wait_ready(self):
        pass

    def stop(self, samples):
        self.calls.append(("stop",))

    def destroy_node(self):
        pass

    def translate(self, forward, left, timeout):
        x, y, yaw = self.pose
        self.pose = [
            x + (math.cos(yaw) * forward - math.sin(yaw) * left) * self.error_scale,
            y + (math.sin(yaw) * forward + math.cos(yaw) * left) * self.error_scale,
            yaw,
        ]
        self.calls.append(("translate", forward, left))
        return {"end_odom": self.pose}

    def rotate_ccw(self, angle, timeout):
        self.pose[2] = route.wrap(self.pose[2] + angle * self.error_scale)
        self.calls.append(("rotate", angle))
        return {"end_odom": list(self.pose)}


class DetourTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "placement.json"

    def test_outbound_stops_for_placement_and_inverse_restores_pose(self):
        for yaw in (0.0, math.pi / 2, math.radians(-179), math.radians(179)):
            with self.subTest(yaw=yaw):
                start = [2.0, -3.0, yaw]
                state = {
                    "after15_pose": start,
                    "reports": [],
                    "roi_origin": {},
                    "leg_config": {},
                }
                node = FakeMotion(start)
                route.outbound(node, state, self.path)
                expected = [
                    start[0]
                    + node.detected_forward * math.cos(yaw)
                    - 0.85 * math.sin(yaw),
                    start[1]
                    + node.detected_forward * math.sin(yaw)
                    + 0.85 * math.cos(yaw),
                    route.wrap(yaw - math.pi / 2),
                ]
                route.assert_at_pose(node.pose, expected)
                self.assertEqual(state["phase"], "WAITING_FOR_PLACEMENT")
                self.assertFalse(state["placement_completed"])
                self.assertFalse(state["arm_policy_started"])
                self.assertTrue(state["zero_command_latched"])
                self.assertEqual(len([c for c in node.calls if c[0] != "stop"]), 3)
                saved = json.loads(self.path.read_text())
                route.return_to_pickup(node, saved, self.path)
                route.assert_at_pose(node.pose, start)
                self.assertEqual(saved["phase"], "AT_PICKUP_AFTER_PLACEMENT")
                self.assertTrue(saved["placement_completed"])

    def test_return_uses_recorded_waypoints_instead_of_blind_nominal_inverse(self):
        node = FakeMotion([1.0, 2.0, -0.7], error_scale=0.98)
        state = {
            "after15_pose": node.pose[:],
            "reports": [],
            "roi_origin": {},
            "leg_config": {},
        }
        route.outbound(node, state, self.path)
        # Arrival may drift slightly during placement. Retarget saved vertices.
        node.pose[0] += 0.01
        node.pose[1] -= 0.01
        node.error_scale = 1.0
        route.return_to_pickup(node, state, self.path)
        self.assertAlmostEqual(node.pose[0], state["origin"][0], places=8)
        self.assertAlmostEqual(node.pose[1], state["origin"][1], places=8)

    def test_base_moved_during_placement_cannot_start_return_motion(self):
        node = FakeMotion([0.0, 0.0, 0.0])
        state = {
            "after15_pose": node.pose[:],
            "reports": [],
            "roi_origin": {},
            "leg_config": {},
        }
        route.outbound(node, state, self.path)
        node.calls.clear()
        node.pose[0] += 0.30
        with self.assertRaisesRegex(RuntimeError, "checkpoint mismatch"):
            route.return_to_pickup(node, state, self.path)
        self.assertEqual(node.calls, [("stop",)])

    def test_after15_requires_confirmed_complete_result(self):
        good = {
            "status": "complete",
            "phase": "COMPLETE",
            "zero_command_latched": True,
            "door_report": {
                "status": "success",
                "final_state": "FINAL_STOP",
                "zero_command_latched": True,
                "interfaces": {"odom_frame": "world"},
                "final_stationary": {
                    "confirmed": True,
                    "x_m": 1,
                    "y_m": 2,
                    "yaw_rad": 0,
                },
            },
        }
        self.path.write_text(json.dumps(good))
        self.assertEqual(route.load_after15(self.path), ((1.0, 2.0, 0.0), "world"))
        good["phase"] = "DOOR_RETURN_FAILED"
        self.path.write_text(json.dumps(good))
        with self.assertRaises(ValueError):
            route.load_after15(self.path)

    def test_failed_checkpoint_and_unacknowledged_placement_cannot_connect_ros(self):
        args = SimpleNamespace(
            state_file=self.path, phase="return", placement_complete=False
        )
        self.path.write_text(json.dumps({"version": 1, "phase": "FAILED"}))
        with patch.object(route, "create_node") as create:
            with self.assertRaisesRegex(ValueError, "placement-complete"):
                route.execute(args)
            args.placement_complete = True
            with self.assertRaisesRegex(RuntimeError, "partial-stage replay"):
                route.execute(args)
            create.assert_not_called()

    def test_repeated_outbound_does_not_overwrite_checkpoint(self):
        text = '{"phase":"WAITING_FOR_PLACEMENT"}'
        self.path.write_text(text)
        with self.assertRaisesRegex(RuntimeError, "checkpoint exists"):
            route.execute(SimpleNamespace(state_file=self.path, phase="outbound"))
        self.assertEqual(self.path.read_text(), text)

    def test_nonfinite_pose_rejected(self):
        for pose in ([0, 0, float("nan")], [float("inf"), 0, 0], [0, 0]):
            with self.assertRaises(ValueError):
                route.checked_pose(pose)

    def test_preview_does_not_create_state_or_import_ros(self):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "return",
                "--state-file",
                str(self.path),
                "--stop-at-pickup",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        report = json.loads(result.stdout)
        self.assertFalse(report["motion_enabled"])
        self.assertEqual(report["return_route"][-1], "STOP at script 15 endpoint")
        self.assertFalse(self.path.exists())

    def test_full_handoff_waits_then_can_stop_at_pickup_then_continue_legacy_once(self):
        args = SimpleNamespace(
            phase="outbound",
            state_file=self.path,
            after15_state=Path(self.temporary.name) / "after15.json",
            placement_complete=False,
            stop_at_pickup=True,
            roi_origin_file=Path(self.temporary.name) / "green_start.json",
            leg_config=ROOT / "config/post15_leg_approach.json",
        )
        node = FakeMotion([1.0, -2.0, 0.8])
        # The real handoff owns ROS initialization. These tests exercise the
        # state transitions/dispatch using only algebraic odometry updates.
        ros = SimpleNamespace(shutdown=lambda: None, ok=lambda: False)
        with (
            patch.object(route, "load_after15", return_value=(node.pose[:], "world")),
            patch.object(route, "create_node", return_value=node),
            patch.object(route, "driver_session", return_value="session"),
            patch.object(route, "run_legacy_return") as legacy,
            patch.dict(sys.modules, {"rclpy": ros}),
        ):
            state = route.execute(args)
            self.assertEqual(state["phase"], "WAITING_FOR_PLACEMENT")
            legacy.assert_not_called()
            args.phase = "return"
            args.placement_complete = True
            state = route.execute(args)
            self.assertEqual(state["phase"], "AT_PICKUP_AFTER_PLACEMENT")
            legacy.assert_not_called()
            node.calls.clear()
            args.stop_at_pickup = False
            route.execute(args)
            legacy.assert_called_once()
            self.assertFalse(any(c[0] in ("translate", "rotate") for c in node.calls))

    def test_roi_origin_comes_from_the_live_pose(self):
        """The detour references wherever the base stands, not a saved START."""
        start = [1.0, -2.0, 0.8]
        node = FakeMotion(start)
        state = {
            "after15_pose": start[:],
            "reports": [],
            "roi_origin": None,
            "driver_session": "session",
            "leg_config": {},
        }
        route.outbound(node, state, self.path)
        origin = state["roi_origin"]
        self.assertEqual(origin["reference"], "current_pose")
        self.assertEqual(origin["driver_session"], "session")
        self.assertTrue(origin["stationary_confirmed"])
        # Captured before the first motion, so it is the detour start pose.
        route.assert_at_pose(origin["pose_odom"], start)

    def test_sensor_preflight_failure_prevents_left_motion(self):
        node = FakeMotion([0, 0, 0])
        state = {
            "after15_pose": node.pose[:],
            "reports": [],
            "roi_origin": {},
            "leg_config": {},
        }
        with patch.object(
            node, "prepare_leg_approach", side_effect=TimeoutError("scans")
        ):
            with self.assertRaises(TimeoutError):
                route.outbound(node, state, self.path)
        self.assertEqual(node.calls, [("stop",)])

    def test_detection_failure_prevents_turn_and_waiting_state(self):
        node = FakeMotion([0, 0, 0])
        state = {
            "after15_pose": node.pose[:],
            "reports": [],
            "roi_origin": {},
            "leg_config": {},
        }
        with patch.object(
            node, "approach_far_leg", side_effect=TimeoutError("no far leg")
        ):
            with self.assertRaises(TimeoutError):
                route.outbound(node, state, self.path)
        self.assertFalse(any(call[0] == "rotate" for call in node.calls))
        self.assertEqual(state["phase"], "APPROACH_FAR_LEG_FRONT_ALIGNMENT")

    def test_15_extension_is_opt_in_and_preview_does_not_move(self):
        command = [
            sys.executable,
            str(ROOT / "scripts/15_return_from_letter.py"),
            "--left-m",
            "0.5",
        ]
        old = json.loads(subprocess.check_output(command, text=True))
        new = json.loads(
            subprocess.check_output(command + ["--with-placement-detour"], text=True)
        )
        self.assertEqual(old["sequence"], new["sequence"][:-2])
        self.assertIn("STOP and save WAITING_FOR_PLACEMENT", new["sequence"][-1])
        self.assertFalse(new["motion_enabled"])


if __name__ == "__main__":
    unittest.main()
