"""Driver bring-up for one hardware profile; never runs a mission or activates arms."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, **kwargs):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments={key: str(value) for key, value in kwargs.items()}.items())


def package_launch(package, filename, **kwargs):
    return include(Path(get_package_share_directory(package)) / "launch" / filename, **kwargs)


def generate_nodes(context):
    path = Path(os.environ["EBIM_HARDWARE_CONFIG"])
    config, root = json.loads(path.read_text()), path.parent
    role = LaunchConfiguration("role").perform(context)
    nodes = []
    if role == "servo":
        nodes.append(package_launch("franka_duo_joint_servo", "joint_servo.launch.py",
                                    playback_speed=config["runtime"]["playback_speed"], enable_gripper="true",
                                    commit_lead_steps=0, idle_follow_timeout_s=60.0))
        nodes.append(package_launch("franka_duo_joint_servo", "gello_target_relay.launch.py",
                                    enable_robot="true", enable_gripper="true"))
    elif role == "arm":
        if config["arms"]["mode"] == "managed":
            for side in ("left", "right"):
                # The single-arm launch starts broadcasters only. The historical
                # dual-arm launch also activates impedance and must not be used here.
                nodes.append(include(root / "scripts/hardware/franka.launch.py", arm_id="fr3v2",
                                     arm_prefix=side, namespace=side, robot_ip=config["arms"][side + "_ip"],
                                     load_gripper="false", joint_sources="joint_states"))
        if config["grippers"]["mode"] == "managed":
            for side in ("left", "right"):
                nodes.append(package_launch("franka_gripper_manager", "robotiq.launch.py",
                                            namespace=side + "/gripper", com_port=config["grippers"][side + "_port"]))
        if config["spine"]["mode"] == "managed":
            nodes.append(package_launch("franka_spine_server", "spine.launch.py", spine_ip=config["spine"]["ip"]))
    elif role == "camera":
        nodes.append(package_launch("zed_wrapper", "zed_camera.launch.py", camera_model="zedm",
                                    namespace="head_camera", camera_name="zed", publish_tf="false",
                                    serial_number=config["camera"]["serial"],
                                    ros_params_override_path=root / "zed.yaml"))
    elif role == "base":
        if config["base"]["mode"] == "managed":
            nodes.append(package_launch("franka_bringup", "tmrv0_2.launch.py",
                                        robot_config_file=root / "base_robot.yaml",
                                        controller_name="swerve_drive_controller"))
        if config["lidars"]["mode"] == "managed":
            for side in ("front", "rear"):
                nodes.append(Node(package="sick_safetyscanners2", executable="sick_safetyscanners2_node",
                                  name=f"lidar_{side}_node", namespace=f"lidar_{side}", output="screen",
                                  parameters=[{"sensor_ip": config["lidars"][side + "_ip"],
                                               "host_ip": config["lidars"]["host_ip"], "host_udp_port": 0,
                                               "interface_ip": "0.0.0.0", "frame_id": f"lidar_{side}",
                                               "channel": 0, "channel_enabled": True,
                                               "angle_start": -0.7854, "angle_end": 3.927,
                                               "use_persistent_config": True}]))
        for module in ("odom_frame_adapter", "dual_laser_merger"):
            nodes.append(ExecuteProcess(cmd=["/usr/bin/python3", str(root / "base/tmr_navigation/tmr_local_navigation" / (module + ".py"))], output="screen"))
        for side, values in {
            "front": (0.3275, 0.2175, 0.19065, -3.141592653589793, 0.0, 0.7846018366025517),
            "rear": (-0.3275, -0.2175, 0.19065, 3.141592653589793, 0.0, -2.3569908169872414),
        }.items():
            args = []
            for name, value in zip(("x", "y", "z", "roll", "pitch", "yaw"), values):
                args += ["--" + name, str(value)]
            nodes.append(Node(package="tf2_ros", executable="static_transform_publisher",
                              name=f"ebim_{side}_tf", arguments=args + ["--frame-id", "base_link", "--child-frame-id", f"lidar_{side}"]))
        nodes.append(package_launch("slam_toolbox", "online_async_launch.py", use_sim_time="false",
                                    slam_params_file=root / "base/tmr_navigation/config/slam_toolbox.yaml"))
        nodes.append(ExecuteProcess(cmd=["/usr/bin/python3", str(root / "base/tmr_cycle/scripts/cmd_vel_adapter.py")], output="screen"))
    else:
        raise ValueError(f"unknown host role: {role}")
    def exited(event, launch_context):
        if launch_context.is_shutdown:
            return []
        command = event.action.process_details.get("cmd", [])
        executable = Path(str(command[0])).name if command else "unknown"
        if executable == "spawner" and event.returncode == 0:
            return []
        # Observe included vendor processes too, while preserving surviving
        # streams. A mission cannot start with this fault marker present.
        (root / f"fault-{role}").write_text(f"{executable} exited with {event.returncode}")
        return []

    return [RegisterEventHandler(OnProcessExit(on_exit=exited)), *nodes]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("role"), OpaqueFunction(function=generate_nodes)])
