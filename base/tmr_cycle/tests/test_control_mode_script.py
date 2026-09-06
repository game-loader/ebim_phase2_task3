#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "base" / "scripts" / "17_control_mode.sh").read_text(encoding="utf-8")


class ControlModeContracts(unittest.TestCase):
    def test_mode_switch_never_restarts_the_base_driver(self) -> None:
        self.assertNotIn("mobile_teleop.launch.py", SOURCE)
        self.assertNotIn("tmrv0_2.launch.py", SOURCE)
        self.assertNotIn("ros2_control_node", SOURCE)

    def test_mission_mode_stops_teleop_before_acquiring_lease(self) -> None:
        block = SOURCE.split("  mission)", 1)[1].split("    ;;", 1)[0]
        self.assertLess(block.index("stop_teleop_velocity_nodes"), block.index("publish_lease true"))

    def test_teleop_mode_releases_only_after_nodes_are_ready(self) -> None:
        block = SOURCE.split("  teleop)", 1)[1].split("    ;;", 1)[0]
        self.assertLess(block.index("ensure_joy_node"), block.index("publish_lease false"))
        self.assertLess(block.index("start_adapter_teleop"), block.index("publish_lease false"))

    def test_teleop_is_routed_through_the_single_adapter(self) -> None:
        self.assertIn("cmd_vel:=/tmr_cycle/cmd_vel", SOURCE)
        self.assertNotIn("cmd_vel:=/swerve_drive_controller/cmd_vel", SOURCE)

    def test_all_control_nodes_use_the_proven_cyclone_environment(self) -> None:
        self.assertIn("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", SOURCE)
        self.assertIn("RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}", SOURCE)
        self.assertIn("CYCLONEDDS_URI=${CYCLONEDDS_URI", SOURCE)
        self.assertIn("^ROS_DOMAIN_ID=${ROS_DOMAIN_ID}$", SOURCE)
        self.assertIn("^ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}$", SOURCE)
        self.assertIn('TMR_CYCLE_ROS_DOMAIN_ID:-97', SOURCE)
        self.assertIn('TMR_CYCLE_ROS_LOCALHOST_ONLY:-1', SOURCE)
        self.assertGreaterEqual(SOURCE.count('process_environment_matches'), 4)
        self.assertIn('if [[ "${#pids[@]}" == 1 ]]', SOURCE)
        self.assertIn('for pid in "${pids[@]}"', SOURCE)


if __name__ == "__main__":
    unittest.main()
