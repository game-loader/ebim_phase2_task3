#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from continuous_route_geometry import (  # noqa: E402
    BrakingModel,
    GapObservation,
    SafetyConfig,
    StableGap,
    TemporalGapStabilizer,
    compute_gap_targets,
    evaluate_swept_corridor,
)


class SweptCorridorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.braking = BrakingModel(
            deceleration_mps2=1.0,
            reaction_time_s=0.20,
            margin_m=0.10,
        )

    def test_braking_formula(self) -> None:
        # 0.5^2/(2*1.0) + 0.2*0.5 + 0.1 = 0.325 m.
        self.assertAlmostEqual(self.braking.required_clearance(0.5), 0.325)

    def test_forward_hit_inside_braking_horizon_stops(self) -> None:
        boundary = 0.40 + self.braking.required_clearance(0.5)
        result = evaluate_swept_corridor(
            [(boundary - 0.001, 0.0)],
            (0.5, 0.0),
            braking=self.braking,
        )
        self.assertTrue(result.should_stop)
        self.assertEqual(result.hit_count, 1)
        self.assertAlmostEqual(result.footprint_leading_extent_m, 0.40)
        self.assertLess(result.nearest_clearance_m, result.required_clearance_m)

    def test_forward_hit_beyond_braking_horizon_does_not_stop(self) -> None:
        boundary = 0.40 + self.braking.required_clearance(0.5)
        result = evaluate_swept_corridor(
            [(boundary + 0.001, 0.0)],
            (0.5, 0.0),
            braking=self.braking,
        )
        self.assertFalse(result.should_stop)
        self.assertEqual(result.hit_count, 0)

    def test_lateral_motion_uses_body_length_as_corridor_half_width(self) -> None:
        required = self.braking.required_clearance(0.4)
        result = evaluate_swept_corridor(
            [
                (0.39, 0.29 + required - 0.001),  # within swept body length
                (0.45, 0.29 + required - 0.001),  # outside 0.40 + 0.04 corridor
            ],
            (0.0, 0.4),
            braking=self.braking,
        )
        self.assertTrue(result.should_stop)
        self.assertEqual(result.hit_count, 1)
        self.assertAlmostEqual(result.footprint_leading_extent_m, 0.29)
        self.assertAlmostEqual(result.corridor_half_width_m, 0.44)

    def test_obstacle_behind_trailing_edge_is_ignored(self) -> None:
        result = evaluate_swept_corridor(
            [(-0.41, 0.0)],
            (0.3, 0.0),
            braking=self.braking,
        )
        self.assertFalse(result.should_stop)

    def test_hit_inside_initial_footprint_is_not_future_collision_evidence(self) -> None:
        result = evaluate_swept_corridor(
            [(0.20, 0.0)],
            (0.3, 0.0),
            braking=self.braking,
        )
        self.assertFalse(result.should_stop)

    def test_initial_forward_ignores_rear_side_self_returns(self) -> None:
        result = evaluate_swept_corridor(
            [(-0.335, 0.33), (-0.34, -0.32), (-0.30, 0.31)],
            (0.10, 0.0),
            braking=self.braking,
        )
        self.assertFalse(result.should_stop)
        self.assertEqual(result.hit_count, 0)

    def test_directional_leading_plane_keeps_real_side_obstacle(self) -> None:
        result = evaluate_swept_corridor(
            [(0.0, 0.31)],
            (0.0, 0.10),
            braking=self.braking,
        )
        self.assertTrue(result.should_stop)
        self.assertEqual(result.hit_count, 1)

    def test_zero_command_has_no_swept_motion(self) -> None:
        result = evaluate_swept_corridor([(0.1, 0.0)], (0.0, 0.0))
        self.assertFalse(result.should_stop)
        self.assertEqual(result.hit_count, 0)

    def test_compact_controller_signature_and_aliases(self) -> None:
        config = SafetyConfig(braking=self.braking)
        result = evaluate_swept_corridor([(0.50, 0.0)], 0.5, 0.0, config)
        self.assertTrue(result.blocked)
        self.assertEqual(result.scale, 0.0)
        self.assertAlmostEqual(result.nearest_m, 0.10)


class TemporalGapStabilizerTests(unittest.TestCase):
    @staticmethod
    def observation(
        frame: int,
        *,
        x: float = 2.0,
        y: float = 0.1,
        width: float = 0.90,
        normal: tuple[float, float] = (1.0, 0.0),
        stamp: float | None = None,
    ) -> GapObservation:
        return GapObservation(
            frame_id=frame,
            midpoint=(x, y),
            width_m=width,
            normal=normal,
            stamp_s=stamp if stamp is not None else frame * 0.1,
        )

    def test_single_frame_never_confirms(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        self.assertIsNone(stabilizer.update(self.observation(1)))
        self.assertEqual(stabilizer.consecutive_count, 1)

    def test_three_consistent_unique_frames_confirm_average(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        self.assertIsNone(stabilizer.update(self.observation(1, x=1.98, width=0.88)))
        self.assertIsNone(stabilizer.update(self.observation(2, x=2.00, width=0.90)))
        stable = stabilizer.update(self.observation(3, x=2.02, width=0.92))
        self.assertIsNotNone(stable)
        assert stable is not None
        self.assertAlmostEqual(stable.midpoint[0], 2.0)
        self.assertAlmostEqual(stable.width_m, 0.90)
        self.assertEqual(stable.sample_count, 3)

    def test_duplicate_frame_does_not_add_evidence(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        self.assertIsNone(stabilizer.update(self.observation(1)))
        self.assertIsNone(stabilizer.update(self.observation(1)))
        self.assertEqual(stabilizer.consecutive_count, 1)
        self.assertIsNone(stabilizer.update(self.observation(2)))
        self.assertIsNotNone(stabilizer.update(self.observation(3)))

    def test_inconsistent_midpoint_restarts_consecutive_run(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        stabilizer.update(self.observation(1))
        stabilizer.update(self.observation(2, x=2.02))
        self.assertIsNone(stabilizer.update(self.observation(3, x=2.30)))
        self.assertEqual(stabilizer.consecutive_count, 1)
        self.assertIsNone(stabilizer.update(self.observation(4, x=2.31)))
        self.assertIsNotNone(stabilizer.update(self.observation(5, x=2.29)))

    def test_narrow_gap_breaks_run(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        stabilizer.update(self.observation(1))
        self.assertIsNone(stabilizer.update(self.observation(2, width=0.70)))
        self.assertEqual(stabilizer.consecutive_count, 0)

    def test_line_normal_sign_ambiguity_is_oriented_forward(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        stabilizer.update(self.observation(1, normal=(1.0, 0.0)))
        stabilizer.update(self.observation(2, normal=(-1.0, 0.0)))
        stable = stabilizer.update(self.observation(3, normal=(1.0, 0.0)))
        self.assertIsNotNone(stable)
        assert stable is not None
        self.assertGreater(stable.normal[0], 0.99)

    def test_large_time_gap_restarts_run(self) -> None:
        stabilizer = TemporalGapStabilizer(
            required_consecutive=3,
            maximum_interframe_s=0.25,
        )
        stabilizer.update(self.observation(1, stamp=0.0))
        stabilizer.update(self.observation(2, stamp=0.1))
        self.assertIsNone(stabilizer.update(self.observation(3, stamp=1.0)))
        self.assertEqual(stabilizer.consecutive_count, 1)

    def test_missing_frame_breaks_run(self) -> None:
        stabilizer = TemporalGapStabilizer(required_consecutive=3)
        stabilizer.update(self.observation(1))
        stabilizer.update(None)
        self.assertEqual(stabilizer.consecutive_count, 0)


class GapTargetTests(unittest.TestCase):
    @staticmethod
    def stable_gap(
        *,
        midpoint: tuple[float, float] = (2.0, 0.2),
        width: float = 1.0,
        normal: tuple[float, float] = (1.0, 0.0),
    ) -> StableGap:
        return StableGap(
            midpoint=midpoint,
            width_m=width,
            normal=normal,
            sample_count=4,
            first_frame_id=1,
            last_frame_id=4,
            first_stamp_s=0.1,
            last_stamp_s=0.4,
        )

    def test_axis_aligned_targets_clear_front_and_rear(self) -> None:
        targets = compute_gap_targets(
            self.stable_gap(),
            pre_door_clearance_m=0.20,
            post_door_clearance_m=0.25,
            side_clearance_m=0.08,
        )
        self.assertAlmostEqual(targets.pre_door[0], 1.40)
        self.assertAlmostEqual(targets.pre_door[1], 0.20)
        self.assertAlmostEqual(targets.post_door[0], 2.65)
        self.assertAlmostEqual(targets.post_door[1], 0.20)
        self.assertAlmostEqual(targets.traversal_distance_m, 1.25)
        self.assertAlmostEqual(targets.required_opening_width_m, 0.74)
        self.assertAlmostEqual(targets.available_side_clearance_m, 0.21)

    def test_skewed_gap_accounts_for_projected_rectangle(self) -> None:
        angle = math.radians(10.0)
        normal = (math.cos(angle), math.sin(angle))
        targets = compute_gap_targets(
            self.stable_gap(width=1.05, normal=normal),
            side_clearance_m=0.05,
        )
        # A skewed door sees some of the base length along its tangent, so the
        # required opening is wider than nominal 0.58 + margins.
        self.assertGreater(targets.required_opening_width_m, 0.68)
        self.assertAlmostEqual(
            math.hypot(
                targets.post_door[0] - targets.pre_door[0],
                targets.post_door[1] - targets.pre_door[1],
            ),
            targets.traversal_distance_m,
        )

    def test_too_narrow_gap_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "below required"):
            compute_gap_targets(self.stable_gap(width=0.70), side_clearance_m=0.08)


if __name__ == "__main__":
    unittest.main()
