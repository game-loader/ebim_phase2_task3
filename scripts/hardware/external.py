#!/usr/bin/env python3
"""Humble task lifecycle on an x86 station attached to an external robot graph."""

import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from host import Host, process_identity

from franka_duo_tele_data.hardware import fastdds_xml


class ExternalHost(Host):
    def environment(self, camera=False):
        env = super().environment(camera)
        env["EBIM_EXTERNAL_HARDWARE"] = "1"
        env["PYTHONPATH"] = "/app/src"
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(self.release / "dds_arm.xml")
        env["EBIM_ROS_GATEWAY"] = str(self.root / "ros.sock")
        for key in ("CYCLONEDDS_URI", "ROS_DISCOVERY_SERVER", "FASTDDS_DEFAULT_PROFILES_FILE"):
            env.pop(key, None)
        env.pop("LD_LIBRARY_PATH", None)
        return env

    def start_gateway(self):
        item = self.state["processes"].get("gateway")
        if item and self.alive(item):
            return
        if any(self.alive(p) for name, p in self.state["processes"].items() if name != "gateway"):
            raise RuntimeError("cannot recreate DDS gateway while runtime streams exist; operator recovery required")
        address = self.host["dds_address"]
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "address"]))
        addresses = {a["local"] for i in interfaces for a in i.get("addr_info", []) if a["family"] == "inet"}
        if address == "auto":
            candidates = addresses - {"127.0.0.1"}
            if len(candidates) != 1:
                raise RuntimeError("set hosts.arm.dds_address to the station robot-LAN IPv4 address; auto is ambiguous")
            address = candidates.pop()
        if address not in addresses:
            raise RuntimeError("DDS address is not assigned to this host: " + address)
        (self.release / "dds_arm.xml").write_text(fastdds_xml(address))
        (self.root / "logs").mkdir(exist_ok=True)
        command = ["/app/.venv/bin/python", "/app/scripts/hardware/ros_gateway.py",
                   str(self.release / "hardware.json"), str(self.root / "ros.sock")]
        (self.root / "ros.sock").unlink(missing_ok=True)
        with (self.root / "logs/gateway.log").open("ab") as log:
            child = subprocess.Popen(self.shell(command), env=self.environment(), stdout=log, stderr=log,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        self.state["release"] = str(self.release)
        self.state["processes"]["gateway"] = {"pid": child.pid, "start": process_identity(child.pid)}
        self.save()
        deadline = time.monotonic() + 30
        while not (self.root / "ros.sock").exists():
            if child.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("DDS gateway failed; inspect logs/gateway.log")
            time.sleep(0.1)

    def stop_gateway(self):
        item = self.state["processes"].get("gateway")
        if item and self.alive(item):
            os.killpg(item["pid"], signal.SIGINT)
            deadline = time.monotonic() + 15
            while self.alive(item) and time.monotonic() < deadline:
                time.sleep(0.1)
            if self.alive(item):
                raise RuntimeError("gateway failed to stop")
        self.state["processes"].pop("gateway", None)
        self.save()

    def probe(self, operation, camera=False):
        return self.native(["/app/.venv/bin/python", "/app/scripts/hardware/external_probe.py",
                            operation, str(self.release / "hardware.json")],
                           timeout=float(self.config["runtime"]["ready_timeout_s"]) + 90)

    def check(self):
        self.assert_no_conflicts()
        for setup in [self.host["ros_setup"], *self.host["overlays"]]:
            if not Path(setup).is_file():
                raise RuntimeError("missing Humble task overlay: " + setup)
        self.native(["/usr/bin/python3", "-c",
                     "import rclpy; from franka_spine_msgs.action import MoveAbsolute; "
                     "from franka_spine_msgs.srv import GetPosition; "
                     "from ament_index_python.packages import get_package_share_directory as p; "
                     "p('franka_duo_joint_servo'); p('rmw_fastrtps_cpp')"])
        running = any(self.alive(p) for p in self.state["processes"].values())
        self.start_gateway()
        try:
            self.probe("check")
        finally:
            if not running:
                self.stop_gateway()

    def up(self, activate):
        self.assert_no_conflicts()
        self.start_gateway()
        if activate:
            self.probe("deactivate-impedance")
        self.probe("inactive")
        # Old external command publishers must already be paused by their owner.
        # No process killing or assumed vendor-specific pause interface.
        self.probe("unowned")
        self.probe("check")
        self.probe("configure-impedance")
        self.start("external-servo")
        self.start("routes")
        self.probe("runtime-ready")
        if activate:
            self.probe("activate-impedance")
        self.health()

    def down(self):
        if any(self.alive(p) for p in self.state["processes"].values()):
            item = self.state["processes"].get("gateway")
            if not item or not self.alive(item):
                raise RuntimeError("gateway unavailable; operator must deactivate impedance before recovery")
            self.probe("deactivate-impedance")
        super().down()

    def health(self):
        if self.state.get("release") != str(self.release):
            raise RuntimeError("runtime release differs from this configuration")
        for name in ("gateway", "external-servo", "routes"):
            item = self.state["processes"].get(name)
            if not item or not self.alive(item):
                raise RuntimeError(name + " is not running; inspect logs")
            fault = self.release / ("fault-" + name)
            if fault.exists():
                raise RuntimeError(fault.read_text())

    def mission(self, execute):
        if execute:
            self.health()
            self.probe("mission-ready")
        command = ["/app/entrypoint.sh", "mission", "--base-local",
                   "--base-root", str(self.release / "base/tmr_base"),
                   "--base-env", str(self.release / "base_env.sh"),
                   "--config", str(self.release / "policy.yaml"),
                   "--calibration", str(self.release / "calibration.json"),
                   "--speed", str(self.config["runtime"]["playback_speed"])]
        if execute:
            command.append("--execute")
        self.native(command)


def main():
    operation, raw_config, release = sys.argv[1:4]
    host = ExternalHost(json.loads(raw_config), "arm", release)
    if operation in ("status", "health"):
        return getattr(host, operation)()
    if not (host.release / ".complete").is_file():
        if operation == "down" and not any(host.alive(p) for p in host.state["processes"].values()):
            return
        raise RuntimeError("local release is unavailable; run check/up first")
    with (host.root / ".orchestration.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        host.state = json.loads(host.state_path.read_text()) if host.state_path.exists() else {"processes": {}}
        if operation in ("check", "down"):
            getattr(host, operation)()
        elif operation == "up":
            host.up("--activate" in sys.argv)
        elif operation in ("mission", "mission-execute"):
            host.mission(operation == "mission-execute")
        else:
            raise ValueError("unknown external operation: " + operation)


if __name__ == "__main__":
    main()
