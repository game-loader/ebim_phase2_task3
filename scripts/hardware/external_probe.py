#!/usr/bin/env python3
"""Hamburg interface checks and explicit controller lifecycle operations."""

import json
import sys
from pathlib import Path

from alignment import alignment_error, joint_positions
from controller_manager_msgs.srv import ConfigureController, SwitchController
from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetPosition
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from probe import SIDES, Probe
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan

from franka_duo_tele_data.camera_input import camera_matrix
from franka_duo_tele_data.ros_backend import ActionClient, ros as rclpy

COMMANDS = [f"/{side}/gello/joint_states" for side in SIDES] + [
    f"/{side}/gripper/gripper_client/target_gripper_width_percent" for side in SIDES
] + ["/swerve_drive_controller/cmd_vel"]


class ExternalProbe(Probe):
    def arm_ready(self):
        topics = []
        for side in SIDES:
            for suffix in ("franka_robot_state_broadcaster/measured_joint_states", "gripper/joint_states"):
                topic = f"/{side}/{suffix}"
                self.watch(topic, JointState)
                topics.append(topic)
            topic = f"/{side}/franka_robot_state_broadcaster/current_pose"
            self.watch(topic, PoseStamped)
            topics.append(topic)
        self.wait(lambda: all(self.fresh(t, 0.2) for t in topics), "arm and gripper JointState")
        for side in SIDES:
            joint_positions(self.latest[f"/{side}/franka_robot_state_broadcaster/measured_joint_states"][0], side)
        self.wait(lambda: self.client("/franka_spine_node/get_position", GetPosition).service_is_ready(),
                  "spine GetPosition")
        action = ActionClient(self.node, MoveAbsolute, "/franka_spine_node/move_absolute")
        try:
            if not action.wait_for_server(timeout_sec=5):
                raise RuntimeError("spine MoveAbsolute action unavailable")
        finally:
            action.destroy()

    def base_ready(self):
        topics = {"/swerve_drive_controller/odom": Odometry,
                  "/lidar_front/scan": LaserScan, "/lidar_rear/scan": LaserScan}
        for topic, kind in topics.items():
            self.watch(topic, kind)
        self.wait(lambda: all(self.fresh(t) for t in topics), "base odometry and dual LiDAR")

    def camera_ready(self):
        image = self.config["camera"]["image_topic"]
        info = self.config["camera"]["camera_info_topic"]
        self.watch(image, Image)
        self.watch(info, CameraInfo)
        self.wait(lambda: self.fresh(image) and self.fresh(info), "external Image and CameraInfo")
        frame, intrinsics = self.latest[image][0], self.latest[info][0]
        if not frame.data or frame.encoding not in ("bgr8", "rgb8", "bgra8", "rgba8", "mono8"):
            raise RuntimeError("unsupported or empty external camera image")
        camera_matrix(frame, intrinsics, {"camera_intrinsics": "live_rectified"}, {})

    def check(self):
        self.arm_ready()
        self.base_ready()
        self.camera_ready()
        self.wait(lambda: all(self.hardware_subscribers(t) >= 1 for t in COMMANDS),
                  "hardware command subscribers")
        controllers = {side: self.controllers(side) for side in SIDES}
        print(json.dumps({"hardware_interfaces": "ready", "controllers": controllers,
                          "command_publishers": {t: self.node.count_publishers(t) for t in COMMANDS},
                          "handoff": "before up: deactivate impedance, stop existing command publishers; "
                                     "after up: verify alignment and activate impedance"}))

    def hardware_subscribers(self, topic):
        # The fixed gateway also observes outgoing arm targets for alignment.
        return self.node.count_subscribers(topic) - int(topic.endswith("/gello/joint_states"))

    def unowned(self):
        # The pre-created gateway owns an idle action_chunk publisher.
        self.wait(lambda: self.node.count_publishers("/franka_duo/joint_servo/action_chunk") >= 1,
                  "gateway discovery")
        if self.node.count_publishers("/franka_duo/joint_servo/action_chunk") != 1:
            raise RuntimeError("another policy command publisher is present")
        for topic in COMMANDS:
            if self.node.count_publishers(topic):
                raise RuntimeError("command handoff required; existing publisher on " + topic)

    def route_ready(self):
        self.base_ready()
        topic = "/swerve_drive_controller/cmd_vel"
        if self.node.count_publishers(topic) != 1 or self.node.count_subscribers(topic) < 1:
            raise RuntimeError("route requires the task velocity adapter and external base subscriber")
        for topic in ("/tmr_cycle/mission_cmd_vel", "/tmr_cycle/mission_active"):
            if self.node.count_publishers(topic) != 1:
                raise RuntimeError("another route publisher is active on " + topic)

    def runtime_ready(self):
        self.arm_ready()
        self.base_ready()
        self.camera_ready()
        self.servo_status()
        topics = []
        for side in SIDES:
            pose = f"/{side}/franka_robot_state_broadcaster/current_pose"
            target = f"/{side}/gello/joint_states"
            self.watch(pose, PoseStamped)
            self.watch(target, JointState)
            topics += [pose, target]
        self.wait(lambda: all(self.fresh(t, 0.2) for t in topics), "TCP poses and joint targets")
        self.wait(lambda: all(self.node.count_publishers(t) >= 1
                             for t in COMMANDS + ["/franka_duo/joint_servo/status"]),
                  "task command publisher discovery")
        for topic in COMMANDS + ["/franka_duo/joint_servo/status"]:
            if self.node.count_publishers(topic) != 1:
                raise RuntimeError("expected exactly one task publisher on " + topic)
        self.wait(lambda: all(self.hardware_subscribers(t) >= 1 for t in COMMANDS),
                  "hardware command subscribers")

    def control_call(self, side, name, kind, request):
        client = self.client(f"/{side}/controller_manager/{name}", kind)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"{side}: {name} unavailable; organizer must pre-configure the controller if needed")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30)
        if not future.done() or future.result() is None or not future.result().ok:
            raise RuntimeError(f"{side}: {name} failed; target streams retained")

    def switch(self, side, activate):
        request = SwitchController.Request()
        request.activate_controllers = ["joint_impedance_controller"] if activate else []
        request.deactivate_controllers = [] if activate else ["joint_impedance_controller"]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 10
        self.control_call(side, "switch_controller", SwitchController, request)

    def deactivate_impedance(self):
        for side in SIDES:
            if self.controllers(side).get("joint_impedance_controller") == "active":
                self.switch(side, False)
        self.inactive()

    def configure_impedance(self):
        for side in SIDES:
            state = self.controllers(side).get("joint_impedance_controller")
            if state == "unconfigured":
                self.control_call(side, "configure_controller", ConfigureController,
                                  ConfigureController.Request(name="joint_impedance_controller"))
            if self.controllers(side).get("joint_impedance_controller") != "inactive":
                raise RuntimeError(side + " impedance must be configured and inactive")

    def activate_impedance(self):
        self.runtime_ready()
        self.wait(lambda: self.servo_status().get("idle_latched") is True, "servo idle latch", 70)
        try:
            for side in SIDES:
                target = f"/{side}/gello/joint_states"
                measured = f"/{side}/franka_robot_state_broadcaster/measured_joint_states"
                self.wait(lambda target=target, measured=measured:
                          self.fresh(target, 0.15) and self.fresh(measured, 0.15), "fresh activation target")
                alignment_error(self.latest[target][0], self.latest[measured][0], side)
                self.switch(side, True)
                if self.controllers(side).get("joint_impedance_controller") != "active":
                    raise RuntimeError(side + " activation did not reach active")
        except Exception:
            self.deactivate_impedance()
            raise

    def mission_ready(self):
        self.runtime_ready()
        self.wait(lambda: self.servo_status().get("idle_latched") is True, "servo idle latch", 70)
        for side in SIDES:
            if self.controllers(side).get("joint_impedance_controller") != "active":
                raise RuntimeError(side + " impedance must be activated before the mission")
            topics = [f"/{side}/gello/joint_states", f"/{side}/franka_robot_state_broadcaster/measured_joint_states"]
            self.wait(lambda topics=topics: all(self.fresh(t, 0.2) for t in topics), "fresh aligned joints")
            alignment_error(self.latest[f"/{side}/gello/joint_states"][0],
                            self.latest[f"/{side}/franka_robot_state_broadcaster/measured_joint_states"][0], side)


def main():
    operation, config_path = sys.argv[1:3]
    config = json.loads(Path(config_path).read_text())
    rclpy.init(args=[])
    probe = ExternalProbe(config)
    try:
        getattr(probe, operation.replace("-", "_"))()
        print(json.dumps({"check": operation, "status": "ready"}))
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
