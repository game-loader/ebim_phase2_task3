"""Process-local callbacks backed by pre-created DDS endpoints over a Unix socket.

This module never initializes rclpy or creates a DDS node. ROS message classes,
CDR serialization and clocks are reused, while discovery belongs to the gateway.
"""
import base64
import concurrent.futures
import fcntl
import json
import logging
import os
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from rclpy.clock import Clock
from rclpy.serialization import deserialize_message, serialize_message

_running = False
_pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)


def rpc(operation, **values):
    with socket.socket(socket.AF_UNIX) as sock:
        sock.settimeout(180)
        sock.connect(os.environ["EBIM_ROS_GATEWAY"])
        stream = sock.makefile("rwb")
        stream.write(json.dumps({"operation": operation, **values}).encode() + b"\n")
        stream.flush()
        reply = json.loads(stream.readline())
        if "error" in reply:
            raise RuntimeError(reply["error"])
        return reply["result"]


def encode(message):
    return base64.b64encode(serialize_message(message)).decode()


def decode(data, kind):
    return deserialize_message(base64.b64decode(data), kind)


def type_name(kind):
    return kind.__module__.split(".")[0] + "/" + kind.__name__


def init(*args, **kwargs):
    global _running
    rpc("ping")
    _running = True


def ok(*args, **kwargs):
    return _running


def shutdown(*args, **kwargs):
    global _running
    _running = False


def create_node(name, **kwargs):
    return Node(name, **kwargs)


def spin_once(node, timeout_sec=0.05, **kwargs):
    node.poll(min(timeout_sec or 0.01, 0.05))


def spin(node, **kwargs):
    while ok() and not node.closed:
        spin_once(node)


def spin_until_future_complete(node, future, timeout_sec=None, **kwargs):
    deadline = time.monotonic() + (timeout_sec if timeout_sec is not None else 180)
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.005)


class Publisher:
    def __init__(self, kind, topic):
        rpc("endpoint", direction="publish", topic=topic, kind=type_name(kind))
        self.topic = topic
        self.lock = None
        if topic in ("/franka_duo/joint_servo/action_chunk", "/tmr_cycle/mission_cmd_vel"):
            path = Path(os.environ["EBIM_ROS_GATEWAY"]).parent / (topic.replace("/", "_") + ".lock")
            self.lock = path.open("a")
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def publish(self, message):
        rpc("publish", topic=self.topic, data=encode(message))

    def get_subscription_count(self):
        return rpc("count", topic=self.topic, direction="subscribers")

    def close(self):
        if self.lock:
            self.lock.close()


class Client:
    def __init__(self, kind, name):
        rpc("endpoint", direction="service", topic=name, kind=type_name(kind))
        self.kind, self.name = kind, name

    def wait_for_service(self, timeout_sec=3):
        return rpc("service-ready", name=self.name, timeout=timeout_sec)

    def service_is_ready(self):
        return self.wait_for_service(0)

    def call_async(self, request):
        return _pool.submit(lambda: decode(rpc("service", name=self.name, data=encode(request)), self.kind.Response))


class ActionClient:
    def __init__(self, node, kind, name):
        rpc("endpoint", direction="action", topic=name, kind=type_name(kind))
        self.name, self.kind = name, kind

    def wait_for_server(self, timeout_sec=5):
        return rpc("action-ready", name=self.name, timeout=timeout_sec)

    def send_goal_async(self, goal, **kwargs):
        def send():
            result = rpc("goal", name=self.name, data=encode(goal))
            return SimpleNamespace(accepted=result["accepted"], get_result_async=lambda: _pool.submit(
                lambda: SimpleNamespace(result=decode(rpc("result", goal=result["id"]), self.kind.Result))))
        return _pool.submit(send)

    def destroy(self):
        pass  # The underlying DDS client remains alive until runtime shutdown.


class Node:
    def __init__(self, name, **kwargs):
        self.name, self.closed = name, False
        self.subscriptions, self.publishers, self.timers = [], [], []
        self.clock = Clock()
        self.poll_lock = threading.Lock()

    def create_subscription(self, kind, topic, callback, qos, **kwargs):
        rpc("endpoint", direction="subscribe", topic=topic, kind=type_name(kind))
        item = SimpleNamespace(kind=kind, topic=topic, callback=callback, sequence=0)
        self.subscriptions.append(item)
        return item

    def create_publisher(self, kind, topic, qos, **kwargs):
        item = Publisher(kind, topic)
        self.publishers.append(item)
        return item

    def create_client(self, kind, name, **kwargs):
        return Client(kind, name)

    def destroy_client(self, client):
        pass

    def destroy_subscription(self, item):
        if item in self.subscriptions:
            self.subscriptions.remove(item)

    def create_timer(self, interval, callback, **kwargs):
        item = SimpleNamespace(interval=interval, callback=callback, due=time.monotonic() + interval)
        self.timers.append(item)
        return item

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return logging.getLogger(self.name)

    def count_publishers(self, topic):
        return rpc("count", topic=topic, direction="publishers")

    def count_subscribers(self, topic):
        return rpc("count", topic=topic, direction="subscribers")

    def get_publishers_info_by_topic(self, topic):
        return [None] * self.count_publishers(topic)

    def poll(self, timeout):
        with self.poll_lock:
            items = list(self.subscriptions)
            latest = rpc("read", topics={s.topic: s.sequence for s in items}) if items else {}
            for item in items:
                sample = latest.get(item.topic)
                if sample and sample[0] > item.sequence:
                    item.sequence = sample[0]
                    item.callback(decode(sample[1], item.kind))
            for timer in list(self.timers):
                if time.monotonic() >= timer.due:
                    timer.due = time.monotonic() + timer.interval
                    timer.callback()
        time.sleep(max(0, min(timeout, 0.01)))

    def destroy_node(self):
        self.closed = True
        for publisher in self.publishers:
            publisher.close()
        self.subscriptions.clear()
        self.timers.clear()
