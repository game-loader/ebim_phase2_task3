#!/usr/bin/env python3
"""Humble task lifecycle on an x86 station attached to an external robot graph."""

import fcntl
import json
import sys
from pathlib import Path

from host import Host


class ExternalHost(Host):
    def environment(self, camera=False):
        env = super().environment(camera)
        env["EBIM_EXTERNAL_HARDWARE"] = "1"
        env["PYTHONPATH"] = "/app/src"
        env.pop("LD_LIBRARY_PATH", None)
        return env

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
                     "p('franka_duo_joint_servo'); p('rmw_cyclonedds_cpp')"])
        self.probe("check")

    def up(self, activate):
        if activate:
            raise RuntimeError("external mode never switches controllers; use up without --activate, "
                               "then have the operator activate impedance after target alignment")
        self.assert_no_conflicts()
        # Controller deactivation and old-publisher shutdown belong to the
        # operator. Never interrupt or race an existing hardware command stream.
        self.probe("inactive")
        self.probe("unowned")
        self.start("external-servo")
        self.start("routes")
        self.probe("runtime-ready")
        self.health()

    def health(self):
        if self.state.get("release") != str(self.release):
            raise RuntimeError("runtime release differs from this configuration")
        for name in ("external-servo", "routes"):
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
