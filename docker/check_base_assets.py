"""Offline base image checks: package assets and xacro, never connect hardware."""

import importlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import xacro
import yaml


def main():
    shares = {name: Path(get_package_share_directory(name)) for name in (
        "franka_bringup", "franka_hardware", "franka_mobile", "franka_description", "olv_module_descriptions",
        "sick_safetyscanners2", "zed_wrapper", "zed_components", "slam_toolbox", "rmw_cyclonedds_cpp")}
    xml = xacro.process_file(str(shares["franka_bringup"] / "urdf/tmrv0_2.urdf.xacro"),
                             mappings={"robot_ip": "192.0.2.1", "use_fake_hardware": "false"}).toxml()
    model = ET.fromstring(xml)
    assert model.findtext(".//hardware/plugin") == "franka_hardware/FrankaHardwareInterface"
    assert model.findtext(".//hardware/param[@name='robot_ip']") == "192.0.2.1"
    parameters = yaml.safe_load((shares["franka_bringup"] / "config/controllers.yaml").read_text())["/**"]
    assert parameters["controller_manager"]["ros__parameters"]["swerve_drive_controller"]
    assert parameters["swerve_drive_controller"]["ros__parameters"]["state_interface_prefix"] == "tmrv0_2_"
    for package, executable in (("sick_safetyscanners2", "sick_safetyscanners2_node"),
                                ("controller_manager", "ros2_control_node"),
                                ("slam_toolbox", "async_slam_toolbox_node"),
                                ("rclcpp_components", "component_container_isolated")):
        assert os.access(Path(get_package_prefix(package)) / "lib" / package / executable, os.X_OK)
    for package, library in (("franka_hardware", "franka_hardware"), ("franka_mobile", "franka_mobile"),
                             ("zed_components", "zed_camera_component")):
        assert (Path(get_package_prefix(package)) / "lib" / f"lib{library}.so").is_file()
    for module in ("rclpy", "tf2_ros", "zed_msgs.msg", "sensor_msgs.msg", "cv2", "numpy"):
        importlib.import_module(module)
    assert '"5.1.2"' in Path("/usr/local/zed/zed-config-version.cmake").read_text()
    print("Base assets ready: TMR hardware/controller, SICK, ZED SDK/wrapper and SLAM")


if __name__ == "__main__":
    main()
