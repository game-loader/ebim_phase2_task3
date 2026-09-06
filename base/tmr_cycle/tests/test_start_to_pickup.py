#!/usr/bin/env python3
"""Offline contract tests for the continuous mission wrapper."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
SOURCE = (SCRIPTS / "07_start_to_pickup.py").read_text(encoding="utf-8")
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location(
    "start_to_pickup_contract",
    SCRIPTS / "07_start_to_pickup.py",
)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


@dataclass(frozen=True)
class FakeGap:
    normal: tuple[float, float] = (1.0, 0.0)
    tangent: tuple[float, float] = (0.0, 1.0)
    midpoint: tuple[float, float] = (2.0, 0.2)
    right_edge: tuple[float, float] = (2.0, -0.25)
    left_edge: tuple[float, float] = (2.0, 0.65)
    width: float = 0.90


class ContinuousMissionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config_path = ROOT / "config" / "start_to_pickup.yaml"
        cls.config = mission._load_config(cls.config_path)

    def test_route_order_matches_confirmed_workflow(self) -> None:
        route = mission._dry_run_summary(self.config, self.config_path)["route"]
        expected = [
            "INITIAL_FORWARD",
            "TURN_CW90",
            "ACQUIRE_DOOR",
            "ALIGN_TO_MIDPOINT",
            "CROSS_DOOR",
            "FINAL_STOP",
        ]
        positions = [route.index(name) for name in expected]
        self.assertEqual(positions, sorted(positions))

    def test_frontal_door_targets_clear_complete_footprint(self) -> None:
        targets = mission.compute_door_targets(
            FakeGap(),
            front_m=0.40,
            rear_m=0.40,
            half_width_m=0.29,
            predoor_clearance_m=0.22,
            postdoor_clearance_m=0.18,
            side_clearance_m=0.09,
        )
        self.assertAlmostEqual(targets.predoor[0], 1.38)
        self.assertAlmostEqual(targets.postdoor[0], 2.58)
        self.assertAlmostEqual(targets.required_opening_width_m, 0.76)
        self.assertAlmostEqual(targets.centre_tangent_coordinate, 0.20)

    def test_fixed_postdoor_route_uses_door_perpendicular_bisector(self) -> None:
        targets = mission.compute_fixed_door_route_targets(
            midpoint=(2.0, 0.2),
            predoor=(1.38, 0.2),
            postdoor=(2.58, 0.2),
            before_door_m=0.50,
            forward_from_before_door_m=1.20,
        )
        self.assertEqual(targets.travel_normal, (1.0, 0.0))
        self.assertEqual(targets.half_metre_before_door, (1.5, 0.2))
        self.assertEqual(targets.final_after_forward, (2.7, 0.2))

    def test_direct_controller_topic_is_rejected(self) -> None:
        bad = copy.deepcopy(self.config)
        bad["interfaces"]["command_topic"] = "/swerve_drive_controller/cmd_vel"
        with self.assertRaisesRegex(ValueError, "mission_cmd_vel"):
            mission._validate_config(bad)

    def test_dangerously_large_self_mask_is_rejected(self) -> None:
        bad = copy.deepcopy(self.config)
        bad["safety"]["self_mask_padding_m"] = 0.50
        with self.assertRaisesRegex(ValueError, "self_mask_padding"):
            mission._validate_config(bad)

    def test_near_collision_requests_stop(self) -> None:
        decision = mission.evaluate_swept_corridor(
            [(0.46, 0.0)],
            0.10,
            0.0,
            self.config["safety"],
            self.config["footprint"],
        )
        self.assertTrue(decision.blocked)
        self.assertEqual(decision.speed_scale, 0.0)

    def test_self_return_mask_keeps_external_obstacle(self) -> None:
        filtered = mission.filter_self_returns(
            [(0.10, 0.10), (-0.20, -0.20), (0.43, 0.0), (0.0, 0.32)],
            self.config["footprint"],
            float(self.config["safety"]["self_mask_padding_m"]),
        )
        self.assertEqual(filtered, [(0.43, 0.0), (0.0, 0.32)])

    def test_forward_sweep_ignores_rear_corner_return(self) -> None:
        filtered = mission.filter_future_motion_returns(
            [(-0.335, 0.33), (0.46, 0.0), (0.0, -0.32)],
            0.10,
            0.0,
            self.config["footprint"],
        )
        self.assertEqual(filtered, [(0.46, 0.0)])

    def test_same_corner_return_is_checked_for_lateral_motion(self) -> None:
        point = (-0.335, 0.33)
        filtered = mission.filter_future_motion_returns(
            [point],
            0.0,
            0.10,
            self.config["footprint"],
        )
        self.assertEqual(filtered, [point])

    def test_reverse_sweep_keeps_obstacle_behind_rear_face(self) -> None:
        point = (-0.46, 0.0)
        filtered = mission.filter_future_motion_returns(
            [point],
            -0.10,
            0.0,
            self.config["footprint"],
        )
        self.assertEqual(filtered, [point])

    def test_sensor_timestamps_must_be_unique_and_monotonic(self) -> None:
        tracker = mission.StrictTimestampTracker()
        self.assertTrue(tracker.accept("front_scan", 100))
        self.assertFalse(tracker.accept("front_scan", 100))
        self.assertTrue(tracker.accept("rear_scan", 90))
        self.assertTrue(tracker.accept("front_scan", 101))
        with self.assertRaises(mission.SensorTimestampError):
            tracker.accept("front_scan", 99)
        with self.assertRaises(mission.SensorTimestampError):
            tracker.accept("odom", 0)

    def test_example_image_is_not_an_input(self) -> None:
        summary = mission._dry_run_summary(self.config, self.config_path)
        self.assertFalse(summary["example_image_used"])
        self.assertEqual(summary["command_topic"], "/tmr_cycle/mission_cmd_vel")

    def test_explicit_collision_disable_bypasses_every_runtime_collision_veto(self) -> None:
        self.assertIn('"--disable-collision-guard"', SOURCE)
        self.assertIn('"collision guard explicitly disabled"', SOURCE)
        self.assertIn('return math.inf', SOURCE)
        self.assertIn('"decision": "disabled"', SOURCE)


if __name__ == "__main__":
    unittest.main()
