"""ROS integration smoke test. Run only in a disposable --network none container."""

import json
import os
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from functools import lru_cache, partial

import cv2
import numpy as np
import rclpy
import yaml
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ConfigureController, ListControllers, SwitchController
from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetPosition
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage

from franka_duo_tele_data.hardware import ROOT, Orchestrator, load_hardware
from franka_duo_tele_data.mcap_to_lerobot import quaternion_to_matrix

sys.path.insert(0, str(ROOT / "scripts/hardware"))
from external import ExternalHost


@lru_cache(maxsize=1)
def robot_model():
    return ET.parse(ROOT / "site/franka_duo_joint_servo/model/robot.urdf").getroot()


def expected_tcp(side, positions):
    # Independent URDF chain evaluation catches omitted tool offsets and
    # offsets incorrectly applied along world Z instead of the rotated tip Z.
    robot = robot_model()
    pose = np.eye(4)
    for index in range(1, 9):
        joint = robot.find(f"joint[@name='{side}_fr3v2_joint{index}']")
        origin = joint.find("origin")
        transform = np.eye(4)
        rpy = np.fromstring(origin.get("rpy"), sep=" ")
        for axis, angle in zip(np.eye(3), rpy, strict=True):
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
        self.active = dict.fromkeys(("left", "right"), True)
        self.configured = dict.fromkeys(("left", "right"), False)
        self.refuse_stop = False
        self.target_times = {"left": [], "right": []}
        self.position = 0.4
        self.received = {}
        self.robot_reference, self.gello_reference = {}, {}
        self.follow = True
        self.poses = {}
        self.base_commands = []
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
            pose = PoseStamped()
            self.poses[side] = pose
            pose.header.frame_id = "base"
            _, tcp = expected_tcp(side, joints.position)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tcp[:3, 3]
            # cv2 provides an axis-angle representation for a rotation matrix.
            vector = cv2.Rodrigues(tcp[:3, :3])[0].ravel()
            angle = np.linalg.norm(vector)
            xyz = vector / angle * np.sin(angle / 2)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = xyz
            pose.pose.orientation.w = float(np.cos(angle / 2))
            self.add(PoseStamped, f"/{side}/franka_robot_state_broadcaster/current_pose", pose)
            self.add(JointState, f"/{side}/franka_robot_state_broadcaster/measured_joint_states", joints)
            gripper = JointState()
            gripper.name, gripper.position = ["finger_joint"], [0.0]
            self.add(JointState, f"/{side}/gripper/joint_states", gripper)
            target = f"/{side}/gello/joint_states"
            self.previous.append((self.node.create_publisher(JointState, target, 10), joints))
            self.listen(JointState, target)
            self.listen(JointState, f"/franka_duo/joint_servo/{side}/target")
            self.listen(String, f"/{side}_gello_target_relay/mapping_status")
            self.listen(Float32, f"/{side}/gripper/gripper_client/target_gripper_width_percent")
            self.listen(PoseStamped, f"/franka_duo/measured/{side}_pose", qos_profile_sensor_data)
            self.services.append(self.node.create_service(
                ListControllers, f"/{side}/controller_manager/list_controllers", partial(self.controllers, side)))
            self.services.append(self.node.create_service(
                SwitchController, f"/{side}/controller_manager/switch_controller", partial(self.switch, side)))
            self.services.append(self.node.create_service(
                ConfigureController, f"/{side}/controller_manager/configure_controller", partial(self.configure, side)))
        self.listen(TwistStamped, "/swerve_drive_controller/cmd_vel")
        self.listen(String, "/franka_duo/joint_servo/status")
        self.add(Odometry, "/swerve_drive_controller/odom", Odometry())
        for side in ("front", "rear"):
            scan = LaserScan()
            scan.header.frame_id = "lidar_" + side
            scan.range_min, scan.range_max = 0.1, 20.
            scan.ranges = [5.] * 360
            self.add(LaserScan, f"/lidar_{side}/scan", scan)
        frame = Image()
        snapshot = next(yaml.safe_load_all((ROOT / "configs/hamburg/hamburg_head_camera_info_SN13024307.yaml").read_text()))
        frame.header.frame_id = snapshot["header"]["frame_id"]
        frame.width, frame.height, frame.encoding, frame.step = 640, 360, "bgr8", 1920
        frame.data = bytes(640 * 360 * 3)
        self.add(Image, "/head_camera/zed_node/rgb/color/rect/image", frame)
        info = CameraInfo()
        info.header.frame_id, info.width, info.height = frame.header.frame_id, 640, 360
        info.k, info.p = snapshot["k"], snapshot["p"]
        self.add(CameraInfo, "/head_camera/zed_node/rgb/color/rect/camera_info", info)
        self.static_tf = self.node.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transforms = []
        for side, x in (("front", 0.3275), ("rear", -0.3275)):
            transform = TransformStamped()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "lidar_" + side
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        self.static_tf.publish(TFMessage(transforms=transforms))
        self.services.append(self.node.create_service(GetPosition, "/franka_spine_node/get_position", self.get_position))
        self.action = ActionServer(self.node, MoveAbsolute, "/franka_spine_node/move_absolute", self.move)
        self.tick = 0
        self.timer = self.node.create_timer(0.02, self.publish)

    def add(self, kind, topic, message):
        self.publishers.append((self.node.create_publisher(kind, topic, 10), message))

    def listen(self, kind, topic, qos=10):
        def receive(message):
            self.received[topic] = message
            if topic == "/swerve_drive_controller/cmd_vel":
                self.base_commands.append(message.twist.linear.x)
            if topic in ("/left/gello/joint_states", "/right/gello/joint_states"):
                self.target_times[topic.split("/")[1]].append(time.monotonic())
        self.node.create_subscription(kind, topic, receive, qos)

    def publish(self):
        self.tick += 1
        # Model the organizer's controller, including independent activation
        # references and a bounded rate limiter. Never treat GELLO as absolute.
        for side, reference in self.robot_reference.items():
            if not self.active[side]:
                continue
            if time.monotonic() - self.target_times[side][-1] > 0.5:
                self.active[side] = False
                continue
            if not self.follow:
                continue
            message = self.received[f"/{side}/gello/joint_states"]
            by_name = dict(zip(message.name, message.position, strict=True))
            gello = np.array([by_name[n] for n in self.joints[side].name])
            goal = reference + np.array([-1, -1, 1, 1, 1, 1, -1]) * (gello - self.gello_reference[side])
            current = np.array(self.joints[side].position)
            self.joints[side].position = (current + np.clip(goal - current, -0.008, 0.008)).tolist()
            pose = self.poses[side]
            _, tcp = expected_tcp(side, self.joints[side].position)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tcp[:3, 3]
            vector = cv2.Rodrigues(tcp[:3, :3])[0].ravel()
            angle = np.linalg.norm(vector)
            xyz = vector / angle * np.sin(angle / 2)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = xyz
            pose.pose.orientation.w = float(np.cos(angle / 2))
        stamp = self.node.get_clock().now().to_msg()
        for publisher, message in self.publishers + self.previous:
            if isinstance(message, (Image, CameraInfo)) and self.tick % 3:
                continue
            message.header.stamp = stamp
            publisher.publish(message)

    def controllers(self, side, request, response):
        response.controller = [ControllerState(name="joint_impedance_controller",
                                               state="active" if self.active[side] else ("inactive" if self.configured[side] else "unconfigured"))]
        return response

    def configure(self, side, request, response):
        assert request.name == "joint_impedance_controller"
        self.configured[side] = True
        response.ok = True
        return response

    def switch(self, side, request, response):
        assert request.strictness == SwitchController.Request.STRICT
        if request.activate_controllers:
            assert self.configured[side]
            assert time.monotonic() - self.target_times[side][-1] < 0.5
            self.robot_reference[side] = np.array(self.joints[side].position)
            message = self.received[f"/{side}/gello/joint_states"]
            by_name = dict(zip(message.name, message.position, strict=True))
            self.gello_reference[side] = np.array([by_name[n] for n in self.joints[side].name])
            np.testing.assert_allclose(self.robot_reference[side], self.gello_reference[side], atol=1e-9)
            relay = json.loads(self.received[f"/{side}_gello_target_relay/mapping_status"].data)
            assert relay["state"] == "reference_held"
            self.active[side] = True
        elif self.refuse_stop:
            response.ok = False
            return response
        else:
            self.active[side] = False
        response.ok = True
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
    os.environ.update(ROS_DOMAIN_ID="176", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(host.release / "dds_arm.xml"))
    os.environ.pop("CYCLONEDDS_URI", None)
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
        try:
            host.up(False)
        except RuntimeError as error:
            assert "up --activate" in str(error)
        else:
            raise AssertionError("relative handoff allowed operator activation without reference synchronization")
        assert not host.state["processes"]
        reject(lambda: host.up(True))
        assert set(host.state["processes"]) == {"gateway"}
        def handoff():
            for publisher, _ in hardware.previous:
                hardware.node.destroy_publisher(publisher)
            hardware.previous = []
        released = executor.create_task(handoff)
        wait_for(released.done)
        host.up(True)
        assert all(hardware.configured.values()) and all(hardware.active.values())
        print("SMOKE: configured and activated impedance only after fresh aligned targets", flush=True)
        assert set(host.state["processes"]) == {"gateway", "external-servo", "routes"}
        nodes_before = sorted(hardware.node.get_node_names_and_namespaces())
        for side in ("left", "right"):
            intervals = np.diff(hardware.target_times[side][-40:])
            assert 0.03 < float(np.median(intervals)) < 0.08
            assert max(intervals) < 0.5
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
        hardware.refuse_stop = True
        reject(host.down)
        assert all(host.alive(item) for item in host.state["processes"].values())
        hardware.refuse_stop = False
        print("SMOKE: failed deactivation retained all target streams", flush=True)
        # Simulated action verifies the client works without a SwitchOn server.
        host.native(["/app/entrypoint.sh", "spine", "--target-m", "0.468", "--execute"], timeout=20)
        assert hardware.position == 0.468
        host.probe("route-ready")
        route_pub = hardware.node.create_publisher(TwistStamped, "/tmr_cycle/mission_cmd_vel", 10)
        reject(lambda: host.probe("route-ready"))
        hardware.node.destroy_publisher(route_pub)
        host.probe("mission-ready")
        host.native(["/app/.venv/bin/python", "/app/tests/hamburg_gateway_phase_smoke.py"], timeout=60)
        # The moving chunk must reach robot-space targets with the signed
        # relative mapping; a stationary-only smoke test cannot catch this bug.
        for side in ("left", "right"):
            target = np.array(hardware.received[f"/franka_duo/joint_servo/{side}/target"].position)
            wait_for(lambda side=side, target=target:
                     np.max(np.abs(np.array(hardware.joints[side].position) - target)) < 0.001)
            encoded = np.array(hardware.received[f"/{side}/gello/joint_states"].position)
            reconstructed = hardware.robot_reference[side] + np.array([-1, -1, 1, 1, 1, 1, -1]) * (
                encoded - hardware.gello_reference[side])
            np.testing.assert_allclose(reconstructed, target, atol=1e-6)
            assert np.max(np.abs(target - hardware.robot_reference[side])) > 0.003
            assert np.max(np.abs(encoded[[0, 1, 6]] - target[[0, 1, 6]])) > 0.003
        host.probe("mission-ready")
        print("SMOKE: moving TCP chunk reached both absolute targets through relative GELLO", flush=True)
        wait_for(lambda: any(abs(x - 0.02) < 1e-6 for x in hardware.base_commands))
        wait_for(lambda: hardware.base_commands[-1] == 0.)
        host.mission(False)
        assert sorted(hardware.node.get_node_names_and_namespaces()) == nodes_before
        print("SMOKE: stage processes used the gateway without adding ROS nodes", flush=True)
        print("SMOKE: mission readiness and dry plan passed with live-camera/local-route config", flush=True)
        # Retain the configurable guard regression independently of mapping.
        hardware.follow = False
        status_topic = "/franka_duo/joint_servo/status"
        original = hardware.joints["left"].position[0]
        limit = config["runtime"]["max_tracking_error_rad"]
        hardware.joints["left"].position[0] = original + limit - 0.02
        def status():
            return json.loads(hardware.received[status_topic].data)
        wait_for(lambda: limit - 0.03 < status()["tracking_error_rad"] < limit - 0.01)
        assert not status()["fault"]
        hardware.joints["left"].position[0] = original + limit + 0.02
        wait_for(lambda: status()["fault"])
        assert "tracking" in status()["fault_reason"]
        print(f"SMOKE: tracking guard {limit} rad remains enforced", flush=True)
        host.down()
        assert not any(hardware.active.values())
        assert not host.state["processes"]
        print("HAMBURG_ROS_SMOKE_PASSED", flush=True)
    finally:
        hardware.active = dict.fromkeys(hardware.active, False)
        hardware.refuse_stop = False
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
