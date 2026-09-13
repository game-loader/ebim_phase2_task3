"""ROS integration smoke test. Run only in a disposable --network none container."""

import json
import os
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetPosition
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Float32

from franka_duo_tele_data.hardware import ROOT, Orchestrator, load_hardware
from franka_duo_tele_data.mcap_to_lerobot import quaternion_to_matrix

sys.path.insert(0, str(ROOT / "scripts/hardware"))
from external import ExternalHost


def expected_tcp(side, positions):
    # Independent URDF chain evaluation catches omitted tool offsets and
    # offsets incorrectly applied along world Z instead of the rotated tip Z.
    robot = ET.parse(ROOT / "site/franka_duo_joint_servo/model/robot.urdf").getroot()
    pose = np.eye(4)
    for index in range(1, 9):
        joint = robot.find(f"joint[@name='{side}_fr3v2_joint{index}']")
        origin = joint.find("origin")
        transform = np.eye(4)
        rpy = np.fromstring(origin.get("rpy"), sep=" ")
        for axis, angle in zip(np.eye(3), rpy):
            transform[:3, :3] = cv2.Rodrigues(axis * angle)[0] @ transform[:3, :3]
        transform[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
        pose = pose @ transform
        if joint.get("type") == "revolute":
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            rotation = np.eye(4)
            rotation[:3, :3] = cv2.Rodrigues(axis * positions[index - 1])[0]
            pose = pose @ rotation
    tcp = pose.copy()
    tcp[:3, 3] += pose[:3, :3] @ np.array([0., 0., 0.174])
    return pose, tcp


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("mock graph did not reach expected state")


class MockHardware:
    def __init__(self):
        self.node = rclpy.create_node("hamburg_mock_hardware")
        self.active = True
        self.position = 0.4
        self.received = {}
        self.publishers = []
        self.previous = []
        self.services = []
        self.joints = {}
        for side in ("left", "right"):
            joints = JointState()
            joints.name = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
            joints.position = [0., -0.7, 0.1, -2., 0., 1.5, 0.1]
            if side == "right":
                reference = json.loads((ROOT / "configs/ptp_home_target.json").read_text())
                joints.position = reference["provenance"]["right"]["measured_q"]
            self.joints[side] = joints
            self.add(JointState, f"/{side}/franka_robot_state_broadcaster/measured_joint_states", joints)
            gripper = JointState()
            gripper.name, gripper.position = ["finger_joint"], [0.0]
            self.add(JointState, f"/{side}/gripper/joint_states", gripper)
            target = f"/{side}/gello/joint_states"
            self.previous.append((self.node.create_publisher(JointState, target, 10), joints))
            self.listen(JointState, target)
            self.listen(Float32, f"/{side}/gripper/gripper_client/target_gripper_width_percent")
            self.listen(PoseStamped, f"/franka_duo/measured/{side}_pose", qos_profile_sensor_data)
            self.services.append(self.node.create_service(
                ListControllers, f"/{side}/controller_manager/list_controllers", self.controllers))
        self.listen(TwistStamped, "/swerve_drive_controller/cmd_vel")
        self.add(Odometry, "/swerve_drive_controller/odom", Odometry())
        for side in ("front", "rear"):
            scan = LaserScan()
            scan.range_min, scan.range_max = 0.1, 20.
            scan.ranges = [5.] * 360
            self.add(LaserScan, f"/lidar_{side}/scan", scan)
        frame = Image()
        frame.header.frame_id = "head_left_optical"
        frame.width, frame.height, frame.encoding, frame.step = 640, 360, "bgr8", 1920
        frame.data = bytes(640 * 360 * 3)
        self.add(Image, "/head_camera/zed_node/rgb/color/rect/image", frame)
        info = CameraInfo()
        info.header.frame_id, info.width, info.height = frame.header.frame_id, 640, 360
        info.k = [320., 0., 310., 0., 325., 175., 0., 0., 1.]
        self.add(CameraInfo, "/head_camera/zed_node/rgb/color/rect/camera_info", info)
        self.services.append(self.node.create_service(GetPosition, "/franka_spine_node/get_position", self.get_position))
        self.action = ActionServer(self.node, MoveAbsolute, "/franka_spine_node/move_absolute", self.move)
        self.timer = self.node.create_timer(0.01, self.publish)

    def add(self, kind, topic, message):
        self.publishers.append((self.node.create_publisher(kind, topic, 10), message))

    def listen(self, kind, topic, qos=10):
        self.node.create_subscription(kind, topic, lambda message: self.received.__setitem__(topic, message), qos)

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        for publisher, message in self.publishers + self.previous:
            message.header.stamp = stamp
            publisher.publish(message)

    def controllers(self, request, response):
        response.controller = [ControllerState(name="joint_impedance_controller",
                                               state="active" if self.active else "inactive")]
        return response

    def get_position(self, request, response):
        response.position, response.success = self.position, True
        return response

    def move(self, handle):
        self.position = handle.request.position
        handle.succeed()
        return MoveAbsolute.Result(success=True, stop_by="target", error="")


def main():
    interfaces = json.loads(subprocess.check_output(["ip", "-j", "address"]))
    if any(i["ifname"] != "lo" and (i.get("addr_info") or "UP" in i["flags"]) for i in interfaces):
        raise RuntimeError("smoke test requires --network none; refusing an externally connected graph")
    config = load_hardware(ROOT / "hardware.hamburg.yaml")
    config["domains"] = {"arm": 176, "base": 176}
    for role in ("arm", "base"):
        config["hosts"][role]["dds_address"] = "127.0.0.1"
    config["runtime"]["ready_timeout_s"] = 10
    runner = Orchestrator(config)
    runner.deploy()
    host = ExternalHost(config, "arm", runner.release)
    os.environ.update(ROS_DOMAIN_ID="176", CYCLONEDDS_URI=f"file://{host.release}/dds_arm.xml")
    rclpy.init()
    hardware = MockHardware()
    executor = SingleThreadedExecutor()
    executor.add_node(hardware.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def reject(operation):
        try:
            operation()
        except subprocess.CalledProcessError:
            return
        raise AssertionError("expected handoff rejection")

    try:
        host.check()
        print("SMOKE: check accepted existing publishers without starting a task", flush=True)
        assert not host.state["processes"]
        reject(lambda: host.up(False))
        assert not host.state["processes"]
        hardware.active = False
        reject(lambda: host.up(False))
        assert not host.state["processes"]
        def handoff():
            for publisher, _ in hardware.previous:
                hardware.node.destroy_publisher(publisher)
            hardware.previous = []
        released = executor.create_task(handoff)
        wait_for(released.done)
        host.up(False)
        print("SMOKE: inactive controller handoff started servo and routes only", flush=True)
        assert set(host.state["processes"]) == {"external-servo", "routes"}
        for side in ("left", "right"):
            target = f"/{side}/gello/joint_states"
            wait_for(lambda target=target: target in hardware.received)
            np.testing.assert_allclose(hardware.received[target].position, hardware.joints[side].position)
            topic = f"/franka_duo/measured/{side}_pose"
            wait_for(lambda topic=topic: topic in hardware.received)
            message = hardware.received[topic]
            assert message.header.frame_id == f"{side}_fr3v2_link0"
            assert message.header.stamp.sec > 0
            position, orientation = message.pose.position, message.pose.orientation
            actual = np.eye(4)
            actual[:3, 3] = [position.x, position.y, position.z]
            actual[:3, :3] = quaternion_to_matrix(orientation)
            flange, tcp = expected_tcp(side, hardware.joints[side].position)
            np.testing.assert_allclose(actual, tcp, atol=1e-7, rtol=0)
            assert np.linalg.norm(actual[:3, 3] - flange[:3, 3]) > 0.17
        print("SMOKE: both measured poses are TCPs with the rotated 0.174 m tool offset", flush=True)
        hardware.active = True
        reject(host.down)
        assert all(host.alive(item) for item in host.state["processes"].values())
        print("SMOKE: active-controller shutdown refused and target streams retained", flush=True)
        # Simulated action verifies the client works without a SwitchOn server.
        host.native(["/app/entrypoint.sh", "spine", "--target-m", "0.468", "--execute"], timeout=20)
        assert hardware.position == 0.468
        host.probe("route-ready")
        route_pub = hardware.node.create_publisher(TwistStamped, "/tmr_cycle/mission_cmd_vel", 10)
        reject(lambda: host.probe("route-ready"))
        hardware.node.destroy_publisher(route_pub)
        host.probe("mission-ready")
        host.mission(False)
        print("SMOKE: mission readiness and dry plan passed with live-camera/local-route config", flush=True)
        hardware.active = False
        host.down()
        assert not host.state["processes"]
        print("HAMBURG_ROS_SMOKE_PASSED", flush=True)
    finally:
        hardware.active = False
        try:
            host.down()
        finally:
            executor.shutdown()
            thread.join(timeout=5)
            hardware.action.destroy()
            hardware.node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
