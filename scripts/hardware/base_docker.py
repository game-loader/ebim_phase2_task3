#!/usr/bin/env python3
"""Sent over SSH to the base: manage Docker using only host Python's stdlib."""

import glob
import json
import fcntl
from pathlib import Path
import platform
import re
import subprocess
import sys


def docker(*args, **kwargs):
    return subprocess.run(["docker", *args], check=True, **kwargs)


class BaseDocker:
    def __init__(self, config, release):
        self.config, self.release = config, release
        self.host = config["hosts"]["base"]
        self.name, self.image = self.host["container"], self.host["image"]

    def info(self):
        # Distinguish an absent container from a broken/inaccessible Docker daemon.
        result = docker("container", "ls", "-a", "--filter", "name=^/" + self.name + "$",
                        "--format", "{{.ID}}", capture_output=True, text=True)
        if not result.stdout.strip():
            return None
        return json.loads(docker("inspect", self.name, capture_output=True, text=True).stdout)[0]

    def owned(self, info):
        labels = info["Config"].get("Labels") or {}
        if labels.get("io.ebim.runtime") != "base" or labels.get("io.ebim.release") != self.release:
            raise RuntimeError("base container belongs to another release; use its original profile to stop it")

    def image_check(self):
        info = json.loads(docker("image", "inspect", self.image, capture_output=True, text=True).stdout)[0]
        labels = info["Config"].get("Labels") or {}
        if labels.get("io.ebim.role") != "base" or labels.get("io.ebim.hardware.schema") != "3":
            raise RuntimeError("load the complete base image with hardware schema 3")
        if info.get("Architecture") != "arm64" or platform.machine() != "aarch64":
            raise RuntimeError("this base profile requires the matching ARM64 Jetson hardware")
        release = Path("/etc/nv_tegra_release").read_text()
        if not re.search(r"R36\b.*REVISION:\s*4\.", release):
            raise RuntimeError("base image targets the inspected Jetson L4T R36.4 system")
        if self.config["camera"]["mode"] == "managed":
            runtimes = json.loads(docker("info", "--format", "{{json .Runtimes}}",
                                         capture_output=True, text=True).stdout)
            if "nvidia" not in runtimes:
                raise RuntimeError("Docker's nvidia runtime is not configured on the base host")
        return info["Id"]

    def no_native_conflicts(self):
        patterns = [("base", ("tmrv0_2.launch.py", "ros2_control_node")),
                    ("lidars", ("sick_safetyscanners2_node", "sick_safetyscanners2_lifecycle_node")),
                    ("camera", ("zed_camera.launch.py", "zed_container"))]
        for path in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                args = path.read_bytes().split(b"\0")
            except OSError:
                continue
            if not args or b"-c" in args or b"-lc" in args:
                continue
            command = b" ".join(args).decode(errors="replace")
            navigation = any(name in command for name in
                             ("cmd_vel_adapter.py", "odom_frame_adapter.py", "dual_laser_merger.py",
                              "async_slam_toolbox_node"))
            if navigation or any(self.config[key]["mode"] == "managed" and any(p in command for p in names)
                                 for key, names in patterns):
                raise RuntimeError(f"unmanaged base/camera process {path.parent.name}; inspect before managed startup")

    def options(self, persistent=False):
        args = ["--pull", "never", "--network", "host", "--ipc", "private", "--shm-size", "512m",
                "--cap-add", "SYS_NICE", "--cap-add", "IPC_LOCK", "--ulimit", "rtprio=99:99",
                "--ulimit", "memlock=-1:-1"]
        if persistent:
            for suffix, target in (("state", "/app/runtime"), ("zed-settings", "/usr/local/zed/settings")):
                args += ["--mount", f"type=volume,src={self.name}-{suffix},dst={target}"]
        if self.config["camera"]["mode"] == "managed":
            args += ["--runtime", "nvidia", "--env", "NVIDIA_VISIBLE_DEVICES=all",
                     "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
                     "--mount", "type=bind,src=/dev/bus/usb,dst=/dev/bus/usb",
                     "--device-cgroup-rule", "c 189:* rmw",
                     "--mount", "type=bind,src=/run/udev,dst=/run/udev,readonly"]
            videos = sorted(glob.glob("/dev/video[0-9]*"))
            if not videos:
                raise RuntimeError("no ZED USB video devices found on the base host")
            for video in videos:
                args += ["--device", video]
            settings = self.config["camera"].get("sdk_settings_dir")
            if settings:
                path = Path(settings) / f"SN{self.config['camera']['serial']}.conf"
                if path.is_file():
                    if "," in str(path):
                        raise ValueError("SDK settings path cannot contain commas")
                    args += ["--mount", f"type=bind,src={path},dst=/run/ebim-zed-calibration.conf,readonly"]
        args += ["--env", "EBIM_BASE_CONFIG=" + json.dumps(self.config),
                 "--env", "EBIM_BASE_RELEASE=" + self.release]
        return args

    def inside(self, operation):
        # up has several sequential probes; budget all of them at the configured
        # timeout, rather than abandoning a still-running docker exec mid-startup.
        timeout = 4 * (float(self.config["runtime"]["ready_timeout_s"]) + 90) + 60
        return docker("exec", self.name, "/app/docker/base_entrypoint.sh", operation, timeout=timeout)

    def check(self):
        image_id = self.image_check()
        info = self.info()
        if info:
            self.owned(info)
            if info["Image"] != image_id or not info["State"]["Running"]:
                raise RuntimeError("base container stopped or image changed; inspect and stop the old runtime")
            self.inside("check")
        else:
            self.no_native_conflicts()
            docker("run", "--rm", *self.options(), self.image, "check")

    def deploy(self, payload):
        image_id = self.image_check()
        info = self.info()
        if info:
            self.owned(info)
            if info["Image"] != image_id or not info["State"]["Running"]:
                raise RuntimeError("refusing to replace/restart an existing base container")
        else:
            self.no_native_conflicts()
            docker("run", "--detach", "--name", self.name, "--restart", "no", "--stop-timeout", "60",
                   "--label", "io.ebim.runtime=base", "--label", "io.ebim.release=" + self.release,
                   *self.options(persistent=True), self.image, "keep")
        docker("exec", "-i", self.name, "/usr/bin/python3", "/app/scripts/hardware/deploy.py",
               "/app/runtime", self.release, input=payload)

    def run(self, operation):
        if operation == "check":
            return self.check()
        if operation == "deploy":
            return self.deploy(sys.stdin.buffer.read())
        info = self.info()
        if info is None:
            if operation in ("down", "status"):
                print(json.dumps({"host": "base", "container": "absent"}))
                return
            raise RuntimeError("base container absent; run up first")
        self.owned(info)
        if operation == "down":
            if info["State"]["Running"]:
                self.inside("down")
                docker("stop", "--time", "60", self.name)
            docker("rm", self.name)
        elif operation == "status":
            print(json.dumps({"host": "base", "container": self.name, "state": info["State"]}))
            if info["State"]["Running"]:
                self.inside("status")
        elif operation in ("up", "health", "ready"):
            self.inside(operation)
        else:
            raise ValueError("unsupported base operation: " + operation)


if __name__ == "__main__":
    try:
        manager = BaseDocker(json.loads(sys.argv[2]), sys.argv[3])
        if sys.argv[1] in ("health", "status"):
            # Read-only supervision must remain available while ready probes or
            # lifecycle commands hold the deployment lock.
            manager.run(sys.argv[1])
        else:
            with Path(f"/tmp/{manager.name}.docker.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                manager.run(sys.argv[1])
    except Exception as exc:
        print(f"base docker: {exc}", file=sys.stderr)
        raise SystemExit(1)
