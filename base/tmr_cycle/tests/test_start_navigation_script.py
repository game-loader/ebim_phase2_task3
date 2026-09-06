#!/usr/bin/env python3
"""Static startup contracts; these tests never contact ROS or the robot."""

from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1] / "scripts" / "03_start_navigation.sh"
).read_text(encoding="utf-8")


def function_body(name: str) -> str:
    return SCRIPT.split(f"{name}() {{", 1)[1].split("\n}", 1)[0]


class StartNavigationScriptTests(unittest.TestCase):
    def test_default_graph_is_isolated_from_mixed_humble_jazzy_domain(self):
        self.assertIn('local_domain_id="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"', SCRIPT)
        self.assertIn('localhost_only="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"', SCRIPT)
        self.assertIn("ros2 daemon stop", SCRIPT)
        self.assertIn("unset CYCLONEDDS_URI", SCRIPT)
        self.assertIn("flock -n 9", SCRIPT)
        self.assertIn('/tmp/tmr_navigation_stack.ready', SCRIPT)
        self.assertIn('rm -f "${ready_file}"', SCRIPT)

    def test_controller_manager_readiness_is_real_rpc_not_graph_cache(self):
        body = function_body("controller_manager_rpc")
        self.assertIn("ros2 service call /controller_manager/list_controllers", body)
        self.assertIn("controller_manager_msgs/srv/ListControllers", body)
        self.assertIn("timeout 6", body)
        self.assertNotIn("ros2 service list", SCRIPT)

    def test_fci_handshake_has_short_bounded_failure(self):
        self.assertIn('base_ready_timeout_s="${TMR_CYCLE_BASE_READY_TIMEOUT_S:-15}"', SCRIPT)
        self.assertIn("FCI/TMR TCP connected but the protocol handshake did not answer", SCRIPT)
        self.assertIn("Successfully connected to robot", SCRIPT)
        self.assertNotIn("-X POST", function_body("check_tmr_state"))

    def test_stale_local_base_is_not_duplicated(self):
        self.assertIn("local_base_process_present", SCRIPT)
        self.assertIn("a local base/control process exists", SCRIPT)
        self.assertIn("controller_manager answered RPC; not launching a duplicate", SCRIPT)

    def test_spawner_retry_cannot_race_and_uses_same_parameters(self):
        body = function_body("ensure_swerve_active")
        grace = body.index('wait_for_controller_active "${builtin_spawner_grace_s}"')
        retry = body.index("ros2 run controller_manager spawner swerve_drive_controller")
        self.assertLess(grace, retry)
        self.assertIn('controller_params="${share_dir}/config/controllers.yaml"', body)
        self.assertIn('--ros-args --params-file "${controller_params}"', body)

    def test_upper_stack_waits_for_active_controller_odom_and_both_lidars(self):
        manager = SCRIPT.index('wait_for_controller_manager "${base_pid}"')
        active = SCRIPT.index("\n  ensure_swerve_active\n", manager)
        odom = SCRIPT.index("wait_for_topic_once /swerve_drive_controller/odom", active)
        lidars = SCRIPT.index("start_process lidars", odom)
        front = SCRIPT.index("wait_for_topic_once /lidar_front/scan", lidars)
        rear = SCRIPT.index("wait_for_topic_once /lidar_rear/scan", front)
        slam = SCRIPT.index("start_process slam", rear)
        self.assertEqual([manager, active, odom, lidars, front, rear, slam], sorted(
            [manager, active, odom, lidars, front, rear, slam]
        ))

    def test_cleanup_is_bounded_and_escalates(self):
        body = function_body("cleanup")
        self.assertIn('setsid "$@"', function_body("start_process"))
        self.assertIn('kill "-${signal}" -- "-${pid}"', function_body("signal_alive_children"))
        self.assertIn("signal_alive_children INT", body)
        self.assertIn("wait_for_children_exit 6", body)
        self.assertIn("signal_alive_children TERM", body)
        self.assertIn("signal_alive_children KILL", body)

    def test_motion_requires_explicit_one_command_flag(self):
        self.assertIn("run_mission=false", SCRIPT)
        self.assertIn("--run-mission) run_mission=true", SCRIPT)
        self.assertIn('if [[ "${run_mission}" == true ]]; then', SCRIPT)
        self.assertIn("--config \"${root_dir}/config/start_to_pickup.yaml\" --execute", SCRIPT)
        self.assertIn("--disable-collision-guard", SCRIPT)


if __name__ == "__main__":
    unittest.main()
