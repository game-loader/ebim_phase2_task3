#!/usr/bin/env python3
"""Fixed DDS endpoint set for Hamburg; phase processes use a local Unix socket."""
import json
import os
import signal
import socketserver
import sys
import threading
import time
import uuid
from contextlib import suppress
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import ConfigureController, ListControllers, SwitchController
from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetPosition
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from franka_duo_tele_data.local_ros import decode, encode, type_name


class Gateway:
    def __init__(self, config):
        self.node = rclpy.create_node("hamburg_task_gateway")
        self.latest, self.publishers, self.subscriptions, self.services, self.goals = {}, {}, {}, {}, {}
        self.received_at = {}
        self.lock = threading.Lock()
        self.config = config
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for side in ("left", "right"):
            prefix = f"/{side}/franka_robot_state_broadcaster"
            for topic, kind in [(prefix + "/measured_joint_states", JointState),
                                (prefix + "/current_pose", PoseStamped),
                                (f"/{side}/gripper/joint_states", JointState),
                                (f"/{side}/gello/joint_states", JointState),
                                (f"/franka_duo/joint_servo/{side}/target", JointState),
                                (f"/{side}_gello_target_relay/mapping_status", String),
                                (f"/franka_duo/measured/{side}_pose", PoseStamped)]:
                self.subscribe(topic, kind)
            for name, kind in [("list_controllers", ListControllers), ("switch_controller", SwitchController),
                               ("configure_controller", ConfigureController)]:
                self.service(f"/{side}/controller_manager/{name}", kind)
            self.service(f"/{side}_gello_target_relay/get_parameters", GetParameters)
            for operation in ("prepare_mapping", "enable_mapping"):
                self.service(f"/{side}_gello_target_relay/{operation}", Trigger)
        self.service("/franka_duo_joint_servo/get_parameters", GetParameters)
        self.service("/franka_spine_node/get_position", GetPosition)
        self.action_name = "/franka_spine_node/move_absolute"
        self.action = ActionClient(self.node, MoveAbsolute, self.action_name)
        for topic, kind, qos in [
            (config["camera"]["image_topic"], Image, qos_profile_sensor_data),
            (config["camera"]["camera_info_topic"], CameraInfo, qos_profile_sensor_data),
            ("/swerve_drive_controller/odom", Odometry, qos_profile_sensor_data),
            ("/lidar_front/scan", LaserScan, qos_profile_sensor_data),
            ("/lidar_rear/scan", LaserScan, qos_profile_sensor_data),
            ("/map", OccupancyGrid, latched), ("/tf_static", TFMessage, latched),
            ("/tf", TFMessage, qos_profile_sensor_data),
            ("/navigate_to_pose/_action/status", GoalStatusArray, 10),
            ("/franka_duo/joint_servo/status", String, 10),
        ]:
            self.subscribe(topic, kind, qos)
        for topic, kind in [("/franka_duo/joint_servo/action_chunk", Float32MultiArray),
                            ("/tmr_cycle/mission_cmd_vel", TwistStamped),
                            ("/tmr_cycle/mission_active", Bool), ("/tmr_cycle/route_state", String)]:
            self.publishers[topic] = (kind, self.node.create_publisher(kind, topic, 10))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def subscribe(self, topic, kind, qos=qos_profile_sensor_data):
        def receive(message):
            with self.lock:
                old = self.latest.get(topic, (0, None))
                # Retain all static TF edges, not only the last publisher's island.
                if topic == "/tf_static" and old[1]:
                    edges = {t.child_frame_id: t for t in decode(old[1], TFMessage).transforms}
                    edges.update({t.child_frame_id: t for t in message.transforms})
                    message = TFMessage(transforms=list(edges.values()))
                self.latest[topic] = (old[0] + 1, encode(message))
                self.received_at[topic] = time.monotonic()
        self.subscriptions[topic] = (kind, self.node.create_subscription(kind, topic, receive, qos))

    def service(self, name, kind):
        self.services[name] = (kind, self.node.create_client(kind, name))

    @staticmethod
    def wait(future, timeout=30):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done() or future.result() is None:
            raise TimeoutError("ROS request timed out; state must be checked before retrying")
        return future.result()

    def dispatch(self, request):
        operation = request["operation"]
        if operation == "ping":
            return {"node": self.node.get_name(), "fixed_endpoints": True}
        if operation == "endpoint":
            direction, topic = request["direction"], request["topic"]
            if direction == "action":
                kind = MoveAbsolute if topic == self.action_name else None
            else:
                table = {"publish": self.publishers, "subscribe": self.subscriptions, "service": self.services}[direction]
                kind = table.get(topic, (None,))[0]
            if kind is None or type_name(kind) != request["kind"]:
                raise RuntimeError(f"endpoint was not provisioned before activation: {direction} {topic}")
            return True
        if operation == "read":
            with self.lock:
                return {t: self.latest[t] for t, seq in request["topics"].items()
                        if t in self.latest and self.latest[t][0] > seq
                        # String status has no header; do not let a new local
                        # phase mistake a dead servo's cached status for live data.
                        and (t != "/franka_duo/joint_servo/status" and not t.endswith("/mapping_status")
                             or time.monotonic() - self.received_at[t] <= 0.2)}
        if operation == "count":
            return getattr(self.node, "count_" + request["direction"])(request["topic"])
        if operation == "publish":
            kind, publisher = self.publishers[request["topic"]]
            publisher.publish(decode(request["data"], kind))
            return True
        if operation in ("service", "service-ready"):
            kind, client = self.services[request["name"]]
            if operation == "service-ready":
                return client.wait_for_service(timeout_sec=request["timeout"])
            return encode(self.wait(client.call_async(decode(request["data"], kind.Request))))
        if operation == "action-ready":
            if request["name"] != self.action_name:
                raise ValueError("unknown action")
            return self.action.wait_for_server(timeout_sec=request["timeout"])
        if operation == "goal":
            if request["name"] != self.action_name:
                raise ValueError("unknown action")
            goal = decode(request["data"], MoveAbsolute.Goal)
            if not 0 <= goal.position <= 0.770:
                raise ValueError("Hamburg spine range is 0–0.770 m")
            handle = self.wait(self.action.send_goal_async(goal), 5)
            identity = uuid.uuid4().hex
            if handle.accepted:
                self.goals[identity] = handle.get_result_async()
            return {"accepted": handle.accepted, "id": identity}
        if operation == "result":
            future = self.goals[request["goal"]]
            result = self.wait(future, 150)
            del self.goals[request["goal"]]
            return encode(result.result)
        raise ValueError("unknown local operation")


def main():
    config = json.loads(Path(sys.argv[1]).read_text())
    path = Path(sys.argv[2])
    rclpy.init()
    gateway = Gateway(config)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                request = json.loads(self.rfile.readline(16 * 1024 * 1024))
                response = {"result": gateway.dispatch(request)}
            except Exception as error:
                response = {"error": str(error)}
            with suppress(BrokenPipeError):
                self.wfile.write(json.dumps(response).encode() + b"\n")

    class Server(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True

    # A lifecycle lock in ExternalHost excludes another gateway startup.
    path.unlink(missing_ok=True)
    server = Server(str(path), Handler)
    os.chmod(path, 0o600)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    stop.wait()
    server.shutdown()
    server.server_close()
    gateway.executor.shutdown()
    gateway.thread.join()
    gateway.node.destroy_node()
    rclpy.shutdown()
    path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
