"""Run inside the isolated Hamburg ROS smoke runtime; never on a real robot."""
import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.srv import GetParameters
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from std_srvs.srv import Trigger

from franka_duo_tele_data.action_spec import matrix_to_rot6d
from franka_duo_tele_data.camera_input import camera_matrix
from franka_duo_tele_data.hardware import ROOT
from franka_duo_tele_data.joint_servo_client import chunk_payload
from franka_duo_tele_data.mcap_to_lerobot import quaternion_to_matrix
from franka_duo_tele_data.replay_rgb20d import check_joint_servo_controllers
from franka_duo_tele_data.ros_backend import ros


def forbidden(*args, **kwargs):
    raise AssertionError("phase attempted to initialize DDS")


def main():
    assert os.environ["ROS_DOMAIN_ID"] == "176"
    assert ros is not rclpy
    rclpy.init = forbidden
    ros.init()
    node = ros.create_node("phase_local_callbacks")
    latest = {}
    config_path = Path(os.environ["EBIM_HARDWARE_CONFIG"]).parent / "policy.yaml"
    config = yaml.safe_load(config_path.read_text())
    for key, kind in [("head", Image), ("camera_info", CameraInfo), ("left_pose", PoseStamped), ("right_pose", PoseStamped)]:
        node.create_subscription(kind, config["topics"][key], lambda msg, key=key: latest.__setitem__(key, msg), 10)
    deadline = time.monotonic() + 10
    while len(latest) < 4 and time.monotonic() < deadline:
        ros.spin_once(node)
    assert len(latest) == 4
    matrix = camera_matrix(latest["head"], latest["camera_info"], config, {})
    assert matrix.shape == (3, 3) and np.isfinite(matrix).all()
    assert abs(matrix[0, 0] - 386.9749396123877) < 1e-8
    assert latest["left_pose"].header.frame_id == "base"
    check_joint_servo_controllers(node, config, 0.1)
    hardware = json.loads(Path(os.environ["EBIM_HARDWARE_CONFIG"]).read_text())
    client = node.create_client(GetParameters, "/franka_duo_joint_servo/get_parameters")
    future = client.call_async(GetParameters.Request(names=["max_tracking_error_rad"]))
    ros.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done()
    assert future.result().values[0].double_value == hardware["runtime"]["max_tracking_error_rad"]
    # Exercise the real task -> socket -> DDS -> C++ IK/servo -> mapped relay
    # path. Stationary commands alone cannot reveal sign/activation errors.
    node.create_subscription(String, config["joint_servo_status_topic"],
                             lambda msg: latest.__setitem__("status", json.loads(msg.data)), 10)
    row = []
    for side in ("left", "right"):
        pose = latest[side + "_pose"].pose
        row += [pose.position.x, pose.position.y, pose.position.z]
        row += matrix_to_rot6d(quaternion_to_matrix(pose.orientation)).tolist()
    row += [0., 0.]
    rows = np.tile(row, (12, 1))
    rows[:, 0] += np.linspace(0, 0.01, len(rows))
    rows[:, 9] += np.linspace(0, -0.01, len(rows))
    values, dims, offset = chunk_payload(rows, 0)
    chunk = Float32MultiArray(data=values)
    chunk.layout.dim = [MultiArrayDimension(label="rows", size=dims[0], stride=dims[0] * dims[1]),
                        MultiArrayDimension(label="action", size=dims[1], stride=dims[1])]
    chunk.layout.data_offset = offset
    publisher = node.create_publisher(Float32MultiArray, config["joint_servo_chunk_topic"], 10)
    publisher.publish(chunk)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        ros.spin_once(node)
        status = latest.get("status", {})
        assert not status.get("fault"), status
        if status.get("chunks") == 1 and status.get("holding") and status.get("step", -1) >= len(rows) - 1:
            break
    else:
        raise AssertionError(f"local phase chunk did not complete on the real servo: {status}")
    for side in ("left", "right"):
        # A late request must not overwrite the activation anchor.
        client = node.create_client(Trigger, f"/{side}_gello_target_relay/prepare_mapping")
        future = client.call_async(Trigger.Request())
        ros.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and not future.result().success
    node.destroy_node()
    # Construct the route controller used by outbound/return, including its TF
    # listener. Every subscription is served by the gateway's fixed endpoints.
    scripts = ROOT / "base/tmr_base/scripts"
    sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location("gap_phase", scripts / "05_right_turn_map_gap.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    args = argparse.Namespace(base_frame="base_link", scan_topic=["/lidar_front/scan", "/lidar_rear/scan"],
                              cmd_topic="/tmr_cycle/mission_cmd_vel", odom_topic="/swerve_drive_controller/odom")
    route = module.GapApproachNode(args)
    for _ in range(20):
        ros.spin_once(route)
    assert route._odom is not None and len(route._latest_scans) == 2
    for side, x in (("front", 0.3275), ("rear", -0.3275)):
        transform = route._tf_buffer.lookup_transform("base_link", "lidar_" + side, Time())
        assert abs(transform.transform.translation.x - x) < 1e-8
    command = TwistStamped()
    command.header.stamp = route.get_clock().now().to_msg()
    command.header.frame_id = "base_link"
    command.twist.linear.x = 0.02
    route._cmd_publisher.publish(command)
    # Closing the phase must leave the existing adapter watchdog to send zero.
    route.destroy_node()
    ros.shutdown()
    print("GATEWAY_PHASE_SMOKE_PASSED", flush=True)


if __name__ == "__main__":
    main()
