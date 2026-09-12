#!/usr/bin/env python3
"""Bounded ROS interface checks and measured-pose impedance activation."""

import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import String

from alignment import alignment_error, joint_positions


SIDES = ("left", "right")


class Probe:
    def __init__(self, config):
        self.config = config
        self.node = rclpy.create_node("ebim_hardware_probe")
        self.latest = {}
        self.subscriptions = {}
        self.clients = {}
        self.timeout = float(config["runtime"]["ready_timeout_s"])

    def watch(self, topic, message_type, qos=qos_profile_sensor_data):
        if topic not in self.subscriptions:
            self.subscriptions[topic] = self.node.create_subscription(
                message_type, topic, lambda msg: self.latest.__setitem__(topic, (msg, time.monotonic())), qos)

    def fresh(self, topic, age=0.5):
        if topic not in self.latest:
            return False
        message, received = self.latest[topic]
        if time.monotonic() - received > age:
            return False
        if hasattr(message, "header"):
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            now = self.node.get_clock().now().nanoseconds * 1e-9
            return stamp > 0 and -0.1 <= now - stamp <= age
        return True

    def wait(self, predicate, label, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError("timeout waiting for " + label)

    def client(self, name, kind):
        if name not in self.clients:
            self.clients[name] = self.node.create_client(kind, name)
        return self.clients[name]

    def controllers(self, side):
        name = f"/{side}/controller_manager/list_controllers"
        client = self.client(name, ListControllers)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"{side}: controller manager unavailable")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{side}: controller query timed out")
        return {item.name: item.state for item in future.result().controller}

    def camera_ready(self):
        image, info = "/head_camera/zed/rgb/color/rect/image", "/head_camera/zed/rgb/color/rect/camera_info"
        self.watch(image, Image)
        self.watch(info, CameraInfo)
        self.wait(lambda: self.fresh(image) and self.fresh(info), "native camera Image and CameraInfo")
        frame, intrinsics = self.latest[image][0], self.latest[info][0]
        if not frame.data or frame.width != intrinsics.width or frame.height != intrinsics.height:
            raise RuntimeError("camera image and intrinsics have incompatible dimensions")
        if frame.header.frame_id != intrinsics.header.frame_id or intrinsics.k[0] <= 0 or intrinsics.k[4] <= 0:
            raise RuntimeError("camera frame/intrinsics invalid")

    def base_ready(self):
        topics = {"/swerve_drive_controller/odom": Odometry,
                  "/lidar_front/scan": LaserScan, "/lidar_rear/scan": LaserScan,
                  "/navigation/odom": Odometry}
        for topic, kind in topics.items():
            self.watch(topic, kind)
        self.watch("/map", OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                    reliability=ReliabilityPolicy.RELIABLE))
        self.wait(lambda: all(self.fresh(t) for t in topics) and self.fresh("/map", 5), "base odom, scans and live map")
        from tf2_ros import Buffer, TransformListener
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        self.wait(lambda: all(buffer.can_transform("map", frame, rclpy.time.Time())
                              for frame in ("base_link", "lidar_front", "lidar_rear")), "base map and sensor TF")
        if self.node.count_publishers("/swerve_drive_controller/cmd_vel") != 1:
            raise RuntimeError("base must have exactly one velocity adapter publisher")
        if self.node.count_subscribers("/swerve_drive_controller/cmd_vel") != 1:
            raise RuntimeError("base velocity controller subscriber missing or duplicated")
        del listener

    def arm_ready(self):
        from franka_spine_msgs.srv import GetPosition, SwitchOn
        for side in SIDES:
            self.watch(f"/{side}/franka_robot_state_broadcaster/current_pose", PoseStamped)
            self.watch(f"/{side}/franka_robot_state_broadcaster/measured_joint_states", JointState)
            self.watch(f"/{side}/gripper/joint_states", JointState)
        self.wait(lambda: all(self.fresh(t, 0.2) for t in self.subscriptions), "arm and gripper state")
        for side in SIDES:
            joint_positions(self.latest[f"/{side}/franka_robot_state_broadcaster/measured_joint_states"][0], side)
            controllers = self.controllers(side)
            for broadcaster in ("joint_state_broadcaster", "franka_robot_state_broadcaster"):
                if controllers.get(broadcaster) != "active":
                    raise RuntimeError(f"{side}: {broadcaster} not active")
        for name, kind in (("get_position", GetPosition), ("switch_on", SwitchOn)):
            self.wait(lambda: self.client("/franka_spine_node/" + name, kind).service_is_ready(), "spine " + name)
        from franka_spine_msgs.action import MoveAbsolute
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient
        for name, kind in [("/franka_spine_node/move_absolute", MoveAbsolute)] + [
            (f"/{side}/gripper/robotiq_gripper_controller/gripper_cmd", GripperCommand) for side in SIDES
        ]:
            client = ActionClient(self.node, kind, name)
            try:
                if not client.wait_for_server(timeout_sec=5):
                    raise RuntimeError(f"action server unavailable: {name}")
            finally:
                client.destroy()

    def servo_status(self):
        topic = "/franka_duo/joint_servo/status"
        self.watch(topic, String, 10)
        self.wait(lambda: self.fresh(topic, 0.2), "servo status", 15)
        value = json.loads(self.latest[topic][0].data)
        if value.get("schema") != "franka_duo_joint_servo_status_v1" or value.get("fault") is not False:
            raise RuntimeError("servo schema mismatch or fault")
        if abs(float(value["playback_speed"]) - self.config["runtime"]["playback_speed"]) > 1e-6:
            raise RuntimeError("servo playback speed does not match hardware.yaml")
        return value

    def runtime_ready(self):
        self.arm_ready()
        self.servo_status()
        for side in SIDES:
            for topic in (f"/{side}/gello/joint_states", f"/franka_duo/joint_servo/{side}/target"):
                self.watch(topic, JointState)
        self.wait(lambda: all(self.fresh(t, 0.2) for t in self.subscriptions), "servo target streams")
        for side in SIDES:
            topic = f"/{side}/gello/joint_states"
            if self.node.count_publishers(topic) != 1:
                raise RuntimeError("expected exactly one relay publisher on " + topic)
        if self.node.count_publishers("/franka_duo/joint_servo/status") != 1:
            raise RuntimeError("duplicate servo status publishers")

    def errors_and_mode(self, side, expected_mode):
        from franka_msgs.msg import FrankaRobotState
        topic = f"/{side}/franka_robot_state_broadcaster/robot_state"
        self.watch(topic, FrankaRobotState)
        def healthy_mode():
            if not self.fresh(topic, 0.2):
                return False
            state = self.latest[topic][0]
            errors = [key for key in state.current_errors.get_fields_and_field_types()
                      if getattr(state.current_errors, key)]
            if errors:
                raise RuntimeError(f"{side}: mode={state.robot_mode}, errors={errors}")
            return state.robot_mode == expected_mode
        self.wait(healthy_mode, f"{side} error-free mode {expected_mode}", 8)

    def inactive(self):
        for side in SIDES:
            active = [name for name, state in self.controllers(side).items()
                      if state == "active" and name not in ("joint_state_broadcaster", "franka_robot_state_broadcaster")]
            if active:
                raise RuntimeError(f"{side}: active command controllers {active}; refusing to stop/restart targets")

    def no_target_publishers(self):
        # Allow DDS discovery before deciding that a control channel is unowned.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for topic in ("/left/gello/joint_states", "/right/gello/joint_states",
                      "/franka_duo/joint_servo/status", "/franka_duo/joint_servo/action_chunk"):
            if self.node.count_publishers(topic):
                raise RuntimeError("existing target/policy publisher: " + topic)

    def vacant(self, topics, services):
        # Host network shares DDS with processes outside the PID namespace.
        # Process inspection alone cannot detect a host/other-container driver.
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        available = dict(self.node.get_service_names_and_types())
        for name in topics:
            if self.node.count_publishers(name):
                raise RuntimeError("existing unmanaged publisher: " + name)
        for name in services:
            if name in available:
                raise RuntimeError("existing unmanaged service: " + name)

    def vacant_arm(self):
        topics, services = [], []
        if self.config["arms"]["mode"] == "managed":
            for side in SIDES:
                topics.append(f"/{side}/franka_robot_state_broadcaster/measured_joint_states")
                services.append(f"/{side}/controller_manager/list_controllers")
        if self.config["grippers"]["mode"] == "managed":
            for side in SIDES:
                topics.append(f"/{side}/gripper/joint_states")
                services.append(f"/{side}/gripper/controller_manager/list_controllers")
        if self.config["spine"]["mode"] == "managed":
            services.append("/franka_spine_node/get_position")
        self.vacant(topics, services)

    def vacant_base(self):
        topics = ["/navigation/odom", "/map", "/swerve_drive_controller/cmd_vel"]
        if self.config["base"]["mode"] == "managed":
            topics.append("/swerve_drive_controller/odom")
        if self.config["lidars"]["mode"] == "managed":
            topics += ["/lidar_front/scan", "/lidar_rear/scan"]
        self.vacant(topics, [])

    def vacant_camera(self):
        self.vacant(["/head_camera/zed/rgb/color/rect/image"], [])

    def deactivate(self):
        # Only called for owned, managed arm drivers. Keep targets running until
        # both managers confirm all command controllers are inactive.
        states = {side: self.controllers(side) for side in SIDES}
        for side, controllers in states.items():
            unexpected = [name for name, state in controllers.items() if state == "active" and name not in
                          ("joint_state_broadcaster", "franka_robot_state_broadcaster", "joint_impedance_controller")]
            if unexpected:
                raise RuntimeError(f"{side}: unmanaged command controllers: {unexpected}")
        for side, controllers in states.items():
            if controllers.get("joint_impedance_controller") == "active":
                subprocess.run(["ros2", "control", "switch_controllers", "-c", f"/{side}/controller_manager",
                                "--deactivate", "joint_impedance_controller", "--strict"], check=True, timeout=30)
        self.inactive()

    def prepare_impedance(self):
        self.inactive()
        from ament_index_python.packages import get_package_share_directory
        parameters = str(Path(get_package_share_directory("franka_fr3_arm_controllers")) / "config/controllers.yaml")
        for side in SIDES:
            self.errors_and_mode(side, 1)
            state = self.controllers(side).get("joint_impedance_controller")
            if state is None:
                subprocess.run(["ros2", "run", "controller_manager", "spawner", "joint_impedance_controller",
                                "-c", f"/{side}/controller_manager", "--param-file", parameters, "--inactive"],
                               check=True, timeout=40)
            elif state == "unconfigured":
                subprocess.run(["ros2", "control", "set_controller_state", "joint_impedance_controller", "inactive",
                                "-c", f"/{side}/controller_manager"], check=True, timeout=20)
            if self.controllers(side).get("joint_impedance_controller") != "inactive":
                raise RuntimeError(side + " impedance did not configure as inactive")

    def activate(self):
        self.runtime_ready()
        states = [self.controllers(side).get("joint_impedance_controller") for side in SIDES]
        if states == ["active", "active"]:
            for side in SIDES:
                self.errors_and_mode(side, 2)
            print("both impedance controllers already active")
            return
        self.inactive()
        if states != ["inactive", "inactive"]:
            raise RuntimeError("impedance must be configured before starting the servo")
        for side in SIDES:
            self.errors_and_mode(side, 1)
            value = self.servo_status()
            if value.get("started") is not False or value.get("idle_latched") is not False:
                raise RuntimeError("servo is no longer idle-following; inspect and restart inactive runtime before activation")
            if self.node.count_publishers("/franka_duo/joint_servo/action_chunk") != 0:
                raise RuntimeError("a policy publisher is present during activation")
            target = f"/{side}/gello/joint_states"
            measured = f"/{side}/franka_robot_state_broadcaster/measured_joint_states"
            self.wait(lambda: self.fresh(target, 0.2) and self.fresh(measured, 0.2), "live activation alignment", 5)
            alignment_error(self.latest[target][0], self.latest[measured][0], side)
            subprocess.run(["ros2", "control", "switch_controllers", "-c", f"/{side}/controller_manager",
                            "--activate", "joint_impedance_controller"], check=True, timeout=30)
            self.errors_and_mode(side, 2)
        def latched():
            if not self.fresh("/franka_duo/joint_servo/status", 0.2):
                return False
            value = json.loads(self.latest["/franka_duo/joint_servo/status"][0].data)
            if value.get("fault"):
                raise RuntimeError("servo fault during activation")
            return value.get("idle_latched") is True
        self.wait(latched, "servo idle latch", 70)
        for side in SIDES:
            if self.controllers(side).get("joint_impedance_controller") != "active":
                raise RuntimeError(side + " impedance did not remain active")
            self.errors_and_mode(side, 2)

    def mission_ready(self):
        self.runtime_ready()
        self.camera_ready()
        value = self.servo_status()
        if not value.get("idle_latched"):
            raise RuntimeError("servo not latched")
        for side in SIDES:
            if self.controllers(side).get("joint_impedance_controller") != "active":
                raise RuntimeError("impedance not active: " + side)
            self.errors_and_mode(side, 2)


def main():
    operation, config_path = sys.argv[1:3]
    config = json.loads(Path(config_path).read_text())
    rclpy.init(args=[])
    probe = Probe(config)
    try:
        getattr(probe, operation.replace("-", "_"))()
        print(json.dumps({"check": operation, "status": "ready"}))
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
