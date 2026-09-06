#!/usr/bin/env python3
"""Static regression contracts for the post-grasp route."""

from pathlib import Path
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "scripts" / "13_post_grasp_route.py").read_text(encoding="utf-8")
RUNNER = (Path(__file__).resolve().parents[1] / "scripts" / "13_run_post_grasp_route.sh").read_text(encoding="utf-8")


class PostGraspRouteContracts(unittest.TestCase):
    def test_confirmed_order_and_values_are_frozen(self) -> None:
        names = [
            'Stage("RETREAT_TO_PREDOOR", "translate", -retreat_m, 0.0)',
            'Stage("TURN_CCW180", "rotate", math.radians(turn_deg))',
            'Stage("BACKWARD_AFTER_TURN", "translate", -backward_m, 0.0)',
            'Stage("RIGHT_STAGE_1", "translate", 0.0, -right_first_m)',
            'Stage("RIGHT_STAGE_2", "translate", 0.0, -right_second_m)',
        ]
        positions = [SOURCE.index(item) for item in names]
        self.assertEqual(positions, sorted(positions))
        for default in ("default=1.70", "default=180.0", "default=0.25", "default=0.80", "default=0.85"):
            self.assertIn(default, SOURCE)

    def test_no_perception_collision_veto_is_present(self) -> None:
        self.assertNotIn("LaserScan", SOURCE)
        self.assertNotIn("OccupancyGrid", SOURCE)
        self.assertIn('"collision_guard": "disabled_by_design"', SOURCE)

    def test_single_process_and_accidental_replay_guards(self) -> None:
        self.assertIn("for index, stage in enumerate(stages)", SOURCE)
        self.assertIn("refuse accidental phase replay", SOURCE)
        self.assertIn('state["next_stage"] = index + 1', SOURCE)
        self.assertIn('state["zero_command_latched"] = True', SOURCE)

    def test_runner_reuses_proven_ros_environment_before_nounset(self) -> None:
        self.assertIn('CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"', RUNNER)
        self.assertIn("rmw_cyclonedds_cpp", RUNNER)
        self.assertLess(RUNNER.index("source /opt/ros/humble/setup.bash"), RUNNER.index("set -u"))
        self.assertIn("flock -n /tmp/tmr_post_grasp_route.lock", RUNNER)


if __name__ == "__main__":
    unittest.main()
