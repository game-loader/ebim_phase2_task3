"""Hardware profiles, deployment and task startup orchestration.

This module is ROS-free. The external profile runs locally in a Humble task
container; the reference profile uses a Jazzy arm image and SSH to a Humble base.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import ipaddress
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import tarfile
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
COMPONENTS = ("arms", "grippers", "spine", "base", "lidars", "camera")
SSH_OPTIONS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8"]


def absolute_path(value: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or ".." in PurePosixPath(value).parts:
        raise ValueError(f"expected an absolute path without '..': {value!r}")
    if any(char in value for char in "\n\r\x00") or value == "/":
        raise ValueError("invalid path")
    return value


def load_hardware(path: Path, *, require_calibration=True) -> dict:
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get("version") != 1:
        raise ValueError("hardware.yaml requires version: 1")
    if config.get("profile") != "tmr_fr3v2_duo":
        raise ValueError("only tmr_fr3v2_duo with the original mounting geometry is supported")
    local = config.setdefault("deployment", "two-host") == "external"
    if config["deployment"] not in ("two-host", "external"):
        raise ValueError("deployment must be two-host or external")
    for component in COMPONENTS:
        if config[component]["mode"] not in ("managed", "external"):
            raise ValueError(f"{component}.mode must be managed or external")
        if local and config[component]["mode"] != "external":
            raise ValueError("external deployment requires all hardware modes to be external")
    if local:
        config.setdefault("hosts", {}).setdefault("arm", {"dds_address": "auto"})
        config["hosts"]["base"] = {"dds_address": config["hosts"]["arm"]["dds_address"]}
        for key in ("image_topic", "camera_info_topic"):
            if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", config["camera"].get(key, "")):
                raise ValueError(f"camera.{key} must be an absolute ROS topic")
    arm = config["hosts"]["arm"]
    if any(key in arm for key in ("runtime_root", "ros_setup", "overlays", "ssh_directory")):
        raise ValueError("arm runtime paths are supplied by the image; remove legacy host workspace settings")
    arm.update(runtime_root="/app/runtime", ros_setup="/opt/ros/jazzy/setup.bash",
               overlays=["/opt/ebim-drivers/install/setup.bash", "/app/site/install/setup.bash"],
               ssh_directory="/root/.ssh")
    if local:
        arm.update(ros_setup="/opt/ros/humble/setup.bash", overlays=["/app/site/install/setup.bash"])
    base = config["hosts"]["base"]
    if any(key in base for key in ("runtime_root", "ros_setup", "overlays")):
        raise ValueError("base runtime paths are supplied by the image; remove legacy host workspace settings")
    base.update(runtime_root="/app/runtime", ros_setup="/opt/ros/humble/setup.bash",
                overlays=["/opt/ebim-base/install/setup.bash"])
    if local:
        base.update(ros_setup=arm["ros_setup"], overlays=arm["overlays"])
    if not local and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", base["container"]):
        raise ValueError("invalid base container name")
    if not local and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_./:@-]*", base["image"]):
        raise ValueError("invalid base image reference")
    if config["camera"].get("sdk_settings_dir"):
        absolute_path(config["camera"]["sdk_settings_dir"])
    for role in ("arm", "base"):
        host = config["hosts"][role]
        if role == "arm" and "ssh" in host:
            raise ValueError("hosts.arm.ssh is obsolete: run on the arm host locally and remove this key")
        if role == "base" and not local and not re.fullmatch(r"[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+", host["ssh"]):
            raise ValueError("invalid base SSH target")
        for key in ("runtime_root", "ros_setup"):
            absolute_path(host[key])
        if not isinstance(host["overlays"], list):
            raise ValueError("overlays must be a list of setup files")
        for setup in host["overlays"]:
            absolute_path(setup)
        if host["dds_address"] != "auto":
            ipaddress.IPv4Address(host["dds_address"])
        domain = config["domains"][role]
        if type(domain) is not int or not 0 <= domain <= 232:
            raise ValueError("DDS domain must be an integer in [0, 232]")
    absolute_path(config["hosts"]["arm"]["ssh_directory"])
    if not local and config["domains"]["arm"] == config["domains"]["base"]:
        raise ValueError("arm and base control domains must be separate")
    if local and config["domains"]["arm"] != config["domains"]["base"]:
        raise ValueError("external deployment requires a shared DDS domain")
    for section, keys in {
        "arms": ("left_ip", "right_ip"), "spine": ("ip",), "base": ("ip",),
        "lidars": ("front_ip", "rear_ip", "host_ip"),
    }.items():
        if config[section]["mode"] == "external":
            continue
        for key in keys:
            ipaddress.IPv4Address(config[section][key])
    if config["arms"]["mode"] == "managed" and config["arms"]["left_ip"] == config["arms"]["right_ip"]:
        raise ValueError("left and right arm IPs must differ")
    for key in ("left_port", "right_port"):
        if config["grippers"]["mode"] == "external":
            continue
        if not absolute_path(config["grippers"][key]).startswith("/dev/"):
            raise ValueError("gripper port must be under /dev")
    if config["grippers"]["mode"] == "managed" and config["grippers"]["left_port"] == config["grippers"]["right_port"]:
        raise ValueError("left and right gripper ports must differ")
    if type(config["camera"]["serial"]) is not int or config["camera"]["serial"] <= 0:
        raise ValueError("camera serial must be a positive integer")
    if not 0 < float(config["runtime"]["playback_speed"]) <= 1:
        raise ValueError("playback_speed must be in (0, 1]")
    if not 10 <= float(config["runtime"]["ready_timeout_s"]) <= 300:
        raise ValueError("ready_timeout_s must be in [10, 300]")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_./:@-]*", config["image"]):
        raise ValueError("invalid Docker image reference")
    calibration = Path(os.environ.get("EBIM_CALIBRATION", config["camera"]["calibration"]))
    if not calibration.is_absolute():
        calibration = path.resolve().parent / calibration
    config["camera"]["calibration"] = str(calibration.resolve(strict=require_calibration))
    if os.environ.get("EBIM_CONTAINER_RUNTIME") == "1" and config["grippers"]["mode"] == "managed":
        config["grippers"].update(left_port="/dev/ebim-left-gripper", right_port="/dev/ebim-right-gripper")
    return config


def dds_xml(address: str) -> str:
    root = ET.Element("CycloneDDS", xmlns="https://cdds.io/config")
    domain = ET.SubElement(root, "Domain", Id="any")
    general = ET.SubElement(domain, "General")
    interfaces = ET.SubElement(general, "Interfaces")
    ET.SubElement(interfaces, "NetworkInterface", **({"autodetermine": "true"} if address == "auto" else {"address": address}))
    return ET.tostring(root, encoding="unicode")


def bundle_files(config: dict) -> dict[str, bytes]:
    files = {}
    for tree in ("base/tmr_base/scripts", "base/tmr_base/config", "base/tmr_navigation/tmr_local_navigation",
                 "base/tmr_navigation/config", "scripts/hardware"):
        for path in sorted((ROOT / tree).rglob("*")):
            if path.is_file() and not path.is_symlink() and "__pycache__" not in path.parts:
                files[path.relative_to(ROOT).as_posix()] = path.read_bytes()
    single_arm = "hosts/arm/teleoperation_overlay/src/franka_fr3_arm_controllers/launch/franka.launch.py"
    files["scripts/hardware/franka.launch.py"] = (ROOT / single_arm).read_bytes()
    files["hardware.json"] = json.dumps(config, sort_keys=True, indent=2).encode()
    files["calibration.json"] = Path(config["camera"]["calibration"]).read_bytes()
    for name in ("drivers.lock.json", "base_drivers.lock.json"):
        files["docker/" + name] = (ROOT / "docker" / name).read_bytes()
    for role in ("arm", "base"):
        files[f"dds_{role}.xml"] = dds_xml(config["hosts"][role]["dds_address"]).encode()
    files["base_robot.yaml"] = json.dumps({"ROBOT1": {
        "robot_type": "tmrv0_2", "namespace": "", "robot_ip": config["base"].get("ip", ""),
        "use_fake_hardware": "false", "use_rviz": "false",
    }}).encode()
    files["zed.yaml"] = json.dumps({"/**": {"ros__parameters": {
        "general": {"grab_resolution": "HD720", "pub_resolution": "CUSTOM",
                    "pub_downscale_factor": 2.0, "pub_frame_rate": 30.0},
        "depth": {"depth_mode": "NONE"},
    }}}).encode()
    base = config["hosts"]["base"]
    files["base_env.sh"] = ("\n".join([
        "#!/usr/bin/env bash", "unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH",
        *([] if config["deployment"] == "external" else [
            "export LD_LIBRARY_PATH=/opt/ebim-libfranka/lib:/usr/local/zed/lib:/usr/local/cuda/lib64"]),
        *[f"source {shlex.quote(p)}" for p in [base["ros_setup"], *base["overlays"]]],
        f"export ROS_DOMAIN_ID={config['domains']['base']}",
        "export ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        'export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/dds_base.xml"',
        "export PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1", "",
    ])).encode()
    if config["deployment"] == "external":
        mapping = yaml.safe_load((ROOT / "configs/tmr_rgb20d.yaml").read_text())
        mapping["topics"]["head"] = config["camera"]["image_topic"]
        mapping["topics"]["camera_info"] = config["camera"]["camera_info_topic"]
        mapping["camera_intrinsics"] = "live_rectified"
        for side in ("left", "right"):
            mapping["topics"][side + "_pose"] = f"/franka_duo/measured/{side}_pose"
        files["policy.yaml"] = yaml.safe_dump(mapping).encode()
    return files


def release_id(files: dict[str, bytes]) -> str:
    digest = hashlib.sha256()
    for name, content in sorted(files.items()):
        digest.update(name.encode() + b"\0" + hashlib.sha256(content).digest())
    return digest.hexdigest()[:16]


def archive(files: dict[str, bytes]) -> bytes:
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w:gz") as tar:
        for name, content in sorted(files.items()):
            info = tarfile.TarInfo(name)
            info.size = len(content)
            info.mode = 0o644
            tar.addfile(info, io.BytesIO(content))
    return stream.getvalue()


def host_argv(config: dict, role: str, command: list[str]) -> list[str]:
    if role == "arm":
        return list(command)
    if role == "base":
        return ["ssh", *SSH_OPTIONS, config["hosts"]["base"]["ssh"], shlex.join(command)]
    raise ValueError(f"unknown host role: {role}")


class Orchestrator:
    def __init__(self, config: dict):
        self.config = config
        self.files = bundle_files(config)
        self.release = release_id(self.files)

    def execute(self, role, args, *, payload=None, timeout=180):
        result = subprocess.run(host_argv(self.config, role, args), input=payload,
                                stdout=subprocess.PIPE, stderr=None, timeout=timeout, check=True)
        if result.stdout:
            print(result.stdout.decode(), end="", flush=True)
        return result.stdout

    def host(self, role, operation, *, activate=False):
        if self.config["deployment"] == "external":
            if role == "base":
                return
            command = ["/app/entrypoint.sh", "shell", "-c",
                       shlex.join(["/app/.venv/bin/python", "/app/scripts/hardware/external.py", operation,
                                   json.dumps(self.config), self.release] + (["--activate"] if activate else []))]
            return self.execute("arm", command, timeout=None if operation.startswith("mission") else 420)
        if role == "base":
            return self.base_docker(operation)
        # Pass the stdlib-only helper directly so check/status need no deployment.
        source = self.files["scripts/hardware/host.py"].decode()
        args = ["/usr/bin/python3", "-B", "-c", source, operation, role,
                json.dumps(self.config), self.release]
        if activate:
            args.append("--activate")
        return self.execute(role, args, timeout=None if operation.startswith("mission") else 420)

    def base_docker(self, operation, payload=None):
        source = self.files["scripts/hardware/base_docker.py"].decode()
        timeout = 4 * (float(self.config["runtime"]["ready_timeout_s"]) + 90) + 120
        return self.execute("base", ["/usr/bin/python3", "-B", "-c", source, operation,
                                    json.dumps(self.config), self.release], payload=payload, timeout=timeout)

    def plan(self):
        if self.config["deployment"] == "external":
            print(json.dumps({"release": self.release, "profile": self.config["profile"],
                              "execution": "single Humble task container; attach over DDS",
                              "domains": self.config["domains"], "image": self.config["image"],
                              "components": {key: self.config[key]["mode"] for key in COMPONENTS},
                              "camera": self.config["camera"], "base_container_required": False,
                              "steps": ["read-only hardware graph check", "deploy local routes and policy",
                                        "verify command ownership handoff", "start servo and route velocity adapter",
                                        "run mission only with --execute"]}, indent=2))
            return
        print(json.dumps({"release": self.release, "profile": self.config["profile"],
                          "execution": {"arm": "container-local", "base": "ssh-to-container"},
                          "hosts": self.config["hosts"], "image": self.config["image"],
                          "components": {key: self.config[key]["mode"] for key in COMPONENTS},
                          "steps": ["check host dependencies and ownership", "deploy bundled routes and helpers",
                                    "start base, SLAM and camera", "start arms, grippers and spine",
                                    "start image servo and relay", "check live interfaces",
                                    "activate impedance only with --activate"]}, indent=2))

    def deploy(self):
        payload = archive(self.files)
        if self.config["deployment"] != "external":
            self.base_docker("deploy", payload)
        for role in ("arm",):
            root = self.config["hosts"][role]["runtime_root"]
            # Extraction is into a new versioned directory, never the vendor workspace.
            code = """import io,os,sys,tarfile,tempfile
from pathlib import Path
root=Path(sys.argv[1]); release=sys.argv[2]
root.mkdir(parents=True,exist_ok=True)
target=root/'releases'/release
target.parent.mkdir(exist_ok=True)
data=sys.stdin.buffer.read()
if target.exists():
    if not (target/'.complete').is_file() or (target/'.complete').read_text()!=release:
        raise RuntimeError('incomplete existing release')
    print('release already deployed:',target)
else:
    temporary=Path(tempfile.mkdtemp(prefix='.staging-',dir=target.parent))
    with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as tar:
        for member in tar.getmembers():
            if not member.isfile() or member.name.startswith('/') or '..' in Path(member.name).parts:
                raise RuntimeError('invalid deployment archive member')
            path=temporary/member.name
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_bytes(tar.extractfile(member).read())
    (temporary/'.complete').write_text(release)
    os.rename(temporary,target)
    print('deployed:',target)
"""
            self.execute(role, ["/usr/bin/python3", "-B", "-c", code, root, self.release], payload=payload)

    def run(self, operation, activate=False, execute=False):
        if self.config["deployment"] == "external" and activate:
            raise ValueError("external deployment does not activate controllers; use up without --activate")
        if operation == "plan":
            return self.plan()
        if operation in ("check", "up"):
            if self.config["deployment"] == "external":
                self.deploy()
            for role in ("arm", "base"):
                self.host(role, "check")
            if operation == "check":
                return
        if operation == "up":
            if self.config["deployment"] != "external":
                self.deploy()
            self.host("base", "up")
            self.host("arm", "up", activate=activate)
        elif operation == "status":
            for role in ("arm", "base"):
                self.host(role, "status")
        elif operation == "down":
            # Refuse to stop a target stream under an active arm controller.
            self.host("arm", "down")
            self.host("base", "down")
        elif operation == "mission":
            if execute:
                self.host("base", "health")
                self.host("base", "ready")
            self.host("arm", "mission-execute" if execute else "mission")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("plan", "check", "up", "serve", "status", "down", "mission"))
    parser.add_argument("--hardware", type=Path, default=Path(os.environ.get("EBIM_HARDWARE", ROOT / "hardware.yaml")))
    parser.add_argument("--activate", action="store_true", help="activate impedance after live alignment (up only)")
    parser.add_argument("--execute", action="store_true", help="execute the cup/bowl mission (mission only)")
    args = parser.parse_args(argv)
    if args.activate and args.operation not in ("up", "serve"):
        parser.error("--activate is only valid with up/serve")
    if args.execute and args.operation != "mission":
        parser.error("--execute is only valid with mission")
    try:
        if args.operation != "plan" and os.environ.get("EBIM_CONTAINER_RUNTIME") != "1":
            raise ValueError("use bash scripts/docker_hardware.sh; hardware commands run inside the image")
        orchestrator = Orchestrator(load_hardware(args.hardware))
        if args.operation == "serve":
            from franka_duo_tele_data.hardware_service import serve
            return serve(orchestrator, args.activate)
        orchestrator.run(args.operation, args.activate, args.execute)
    except (ValueError, KeyError, TypeError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"hardware: {exc}")
        return 1
    return 0
