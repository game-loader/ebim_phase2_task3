"""Launch the laptop-side mobile teleop: foot pedals -> base + spine.

Reads both PCsensor foot switches and bridges them to the TMR swerve base
(swerve_drive_controller/cmd_vel) and the Franka spine. Assumes the robot's own
stack (controllers, spine server) is already running and reachable over DDS
(matching ROS_DOMAIN_ID / RMW).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("tmr_pedal_teleop"), "config", "pedal_map.yaml"
    )
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_cfg,
                description="Path to the pedal_map.yaml parameter file.",
            ),
            Node(
                package="pedal_state_publisher",
                executable="pedal_state_publisher",
                name="pedal_state_publisher",
                output="screen",
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="base_bridge",
                name="base_bridge",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="tmr_pedal_teleop",
                executable="spine_bridge",
                name="spine_bridge",
                parameters=[config],
                output="screen",
            ),
        ]
    )
