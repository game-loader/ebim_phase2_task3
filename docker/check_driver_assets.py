#!/usr/bin/env python3
"""Offline image check: resolve URDFs, plugins and executables without ROS nodes."""

import ctypes
import importlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xacro
import yaml


def share(package):
    return Path(get_package_share_directory(package))


def plugin(package, filename, name):
    root = ET.parse(share(package) / filename).getroot()
    libraries = [root] if root.tag == "library" else root.findall("library")
    for library in libraries:
        if any(entry.get("name") == name for entry in library.findall("class")):
            binary = Path(get_package_prefix(package)) / "lib" / ("lib" + library.get("path") + ".so")
            # Resolve dependencies/symbols only; do not instantiate hardware plugins.
            return ctypes.CDLL(str(binary), mode=os.RTLD_NOW)
    raise RuntimeError(f"missing plugin declaration: {name}")


def main():
    libraries = [plugin(*entry) for entry in (
        ("franka_hardware", "franka_hardware.xml", "franka_hardware/FrankaHardwareInterface"),
        ("franka_robot_state_broadcaster", "franka_robot_state_broadcaster.xml",
         "franka_robot_state_broadcaster/FrankaRobotStateBroadcaster"),
        ("franka_fr3_arm_controllers", "franka_fr3_arm_controllers.xml",
         "franka_fr3_arm_controllers/JointImpedanceController"),
        ("robotiq_driver", "hardware_interface_plugin.xml", "robotiq_driver/RobotiqGripperHardwareInterface"),
        ("robotiq_controllers", "controller_plugins.xml", "robotiq_controllers/RobotiqActivationController"),
    )]
    for side in ("left", "right"):
        xml = xacro.process_file(str(share("franka_bringup") / "urdf/franka_arm.urdf.xacro"), mappings={
            "robot_type": "fr3v2", "arm_prefix": side, "robot_ip": "192.0.2.1",
            "hand": "false", "use_fake_hardware": "false", "fake_sensor_commands": "false",
        }).toxml()
        root = ET.fromstring(xml)
        control = root.find("ros2_control")
        assert control is not None, "missing ros2_control"
        assert control.findtext("hardware/plugin") == "franka_hardware/FrankaHardwareInterface"
        expected = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
        joints = {joint.get("name"): joint for joint in control.findall("joint")}
        for name in expected:
            assert joints[name].find("command_interface[@name='effort']") is not None, name
    xml = xacro.process_file(str(share("franka_gripper_manager") / "urdf/robotiq_2f_85_gripper.urdf.xacro"),
                             mappings={"use_fake_hardware": "false", "com_port": "/dev/ebim-left-gripper"}).toxml()
    root = ET.fromstring(xml)
    assert root.findtext(".//hardware/plugin") == "robotiq_driver/RobotiqGripperHardwareInterface"
    assert root.findtext(".//hardware/param[@name='COM_port']") == "/dev/ebim-left-gripper"
    for package, filename, controller in (
        ("franka_fr3_arm_controllers", "controllers.yaml", "joint_impedance_controller"),
        ("franka_gripper_manager", "robotiq_controllers.yaml", "robotiq_gripper_controller"),
    ):
        config = yaml.safe_load((share(package) / "config" / filename).read_text())["/**"]
        assert controller in config["controller_manager"]["ros__parameters"]
        assert config[controller]["ros__parameters"]
        if controller == "joint_impedance_controller":
            assert config[controller]["ros__parameters"]["arm_id"] == "fr3v2"
    for package, executable in (("franka_spine_server", "spine_action_server_node.py"),
                                ("franka_gripper_manager", "robotiq_gripper_client"),
                                ("controller_manager", "ros2_control_node")):
        path = Path(get_package_prefix(package)) / "lib" / package / executable
        assert os.access(path, os.X_OK), path
    for module in ("franka_spine_server.spine_action_server", "franka_spine_msgs.action",
                   "franka_spine_msgs.srv", "franka_msgs.msg", "serial"):
        # serial is a C++ driver package, not a Python module.
        if module == "serial":
            share(module)
        else:
            importlib.import_module(module)
    print(f"Driver assets ready: {len(libraries)} plugins, dual FR3v2 URDFs, Robotiq and Spine")


if __name__ == "__main__":
    main()
