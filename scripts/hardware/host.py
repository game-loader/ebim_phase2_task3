#!/usr/bin/env python3
"""Process lifecycle helper used inside the arm and base containers."""

import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def process_identity(pid):
    try:
        # comm may contain spaces or parentheses; fields after its last ')' are stable.
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        namespace = Path(f"/proc/{pid}/ns/pid").stat().st_ino
        return None if fields[0] == "Z" else f"{boot}:{namespace}:{fields[19]}"
    except (OSError, IndexError):
        return None


def process_commands():
    rows = []
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().split(b"\0")
            # Exclude remote shells and Python -c payloads, which contain source text.
            if not args or b"-c" in args or b"-lc" in args:
                continue
            rows.append((int(path.parent.name), b" ".join(args).decode(errors="replace")))
        except OSError:
            pass
    return rows


class Host:
    def __init__(self, config, role, release):
        self.config, self.role = config, role
        self.host = config["hosts"][role]
        self.root = Path(self.host["runtime_root"])
        self.release = self.root / "releases" / release
        self.state_path = self.root / "state.json"
        self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else {"processes": {}}

    def environment(self, camera=False):
        env = os.environ.copy()
        # Do not inherit a previous shell's ROS domain, SDK environment or Pixi Python.
        for key in list(env):
            if key.startswith(("ROS_", "AMENT_", "COLCON_", "CMAKE_PREFIX", "PYTHONPATH", "LD_LIBRARY_PATH")):
                env.pop(key, None)
        env["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        env["LD_LIBRARY_PATH"] = "/opt/ebim-libfranka/lib:/usr/local/zed/lib:/usr/local/cuda/lib64"
        domain = "arm" if camera else self.role
        env.update(ROS_DOMAIN_ID=str(self.config["domains"][domain]),
                   RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", ROS_LOCALHOST_ONLY="0",
                   ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET",
                   CYCLONEDDS_URI=f"file://{self.release}/dds_{self.role}.xml",
                   PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1",
                   EBIM_HARDWARE_CONFIG=str(self.release / "hardware.json"))
        return env

    def shell(self, command):
        setups = [self.host["ros_setup"], *self.host["overlays"]]
        return ["/bin/bash", "--noprofile", "--norc", "-c",
                "set -e; " + " ".join(f"source {shlex.quote(p)};" for p in setups) +
                " exec " + shlex.join(command)]

    def native(self, command, camera=False, **kwargs):
        return run(self.shell(command), env=self.environment(camera), **kwargs)

    def save(self):
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.state, indent=2))
        temporary.replace(self.state_path)

    def alive(self, item):
        return item.get("start") is not None and process_identity(item["pid"]) == item["start"]

    def check(self):
        for setup in [self.host["ros_setup"], *self.host["overlays"]]:
            if not Path(setup).is_file():
                raise RuntimeError(f"missing ROS setup: {setup}")
        packages = ["rmw_cyclonedds_cpp", "controller_manager"]
        if self.role == "arm":
            packages += ["franka_fr3_arm_controllers", "franka_robot_state_broadcaster", "franka_msgs",
                         "franka_gripper_manager", "robotiq_description", "franka_spine_server", "franka_bringup"]
            if self.config["grippers"]["mode"] == "managed":
                for side in ("left", "right"):
                    port = self.config["grippers"][side + "_port"]
                    if not os.access(port, os.R_OK | os.W_OK):
                        raise RuntimeError(f"missing or inaccessible gripper device: {port}")
            if not Path("/opt/ebim-drivers/install/setup.bash").is_file():
                raise RuntimeError("run the hardware workflow inside the complete submission image")
            if not Path(self.host["ssh_directory"]).is_dir():
                raise RuntimeError("mount the base SSH credentials at /root/.ssh")
        else:
            packages += ["franka_bringup", "sick_safetyscanners2", "slam_toolbox", "tf2_ros", "zed_wrapper"]
            if not Path("/usr/local/zed/zed-config.cmake").is_file():
                raise RuntimeError("base image is missing the ZED SDK")
        code = "from ament_index_python.packages import get_package_share_directory as p; "
        code += "import rclpy,yaml; " + "; ".join(f"p({name!r})" for name in packages)
        # Package inspection does not initialize a ROS participant or daemon.
        self.native(["/usr/bin/python3", "-B", "-c", code])
        result = run(["ip", "-j", "address"], capture_output=True, text=True)
        addresses = {a["local"] for interface in json.loads(result.stdout) for a in interface.get("addr_info", [])}
        if self.host["dds_address"] not in addresses:
            raise RuntimeError(f"DDS address is not assigned to this host: {self.host['dds_address']}")
        self.assert_no_conflicts()
        print(json.dumps({"host": self.role, "dependencies": "ok", "live_readiness": "not checked"}))

    def assert_no_conflicts(self):
        # A previously managed process can be reused only with the same release.
        active = [item for item in self.state["processes"].values() if self.alive(item)]
        if active and self.state.get("release") != str(self.release):
            raise RuntimeError("another managed release is running; inspect status and stop it before upgrading")
        patterns = {
            "arm": [("arms", ("franka.launch.py", "franka_fr3_arm_controllers.launch.py", "ros2_control_node")),
                    ("grippers", ("robotiq.launch.py", "robotiq_gripper_client")),
                    ("spine", ("spine_action_server", "spine.launch.py"))],
            "base": [("base", ("tmrv0_2.launch.py", "ros2_control_node", "cmd_vel_adapter.py")),
                     ("lidars", ("sick_safetyscanners2_node",)),
                     ("camera", ("zed_camera.launch.py", "zed_container"))],
        }
        owned_groups = {item["pid"] for item in active}
        for pid, command in process_commands():
            try:
                if os.getpgid(pid) in owned_groups:
                    continue
            except ProcessLookupError:
                continue
            if self.role == "base" and any(fragment in command for fragment in
                    ("cmd_vel_adapter.py", "odom_frame_adapter", "dual_laser_merger", "async_slam_toolbox_node")):
                raise RuntimeError(f"existing unmanaged navigation process {pid}: {command}")
            for component, fragments in patterns[self.role]:
                if self.config[component]["mode"] == "managed" and any(f in command for f in fragments):
                    raise RuntimeError(f"existing unmanaged {component} process {pid}: {command}; "
                                       "use external mode only if its interfaces match, or stop it explicitly")

    def start(self, name, camera=False):
        item = self.state["processes"].get(name)
        if item and self.alive(item):
            return
        (self.release / f"fault-{name}").unlink(missing_ok=True)
        log_dir = self.root / "logs"
        log_dir.mkdir(exist_ok=True)
        command = ["ros2", "launch", str(self.release / "scripts/hardware/drivers.launch.py"),
                   f"role:={name}"]
        with (log_dir / f"{name}.log").open("ab") as log:
            child = subprocess.Popen(self.shell(command), env=self.environment(camera),
                                     stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        self.state["release"] = str(self.release)
        self.state["processes"][name] = {"pid": child.pid, "start": process_identity(child.pid)}
        self.save()
        time.sleep(0.5)
        if child.poll() is not None:
            raise RuntimeError(f"{name} exited; inspect {log_dir / (name + '.log')}")

    def probe(self, operation, camera=False):
        return self.native(["/usr/bin/python3", "-B", str(self.release / "scripts/hardware/probe.py"),
                            operation, str(self.release / "hardware.json")], camera=camera,
                           timeout=float(self.config["runtime"]["ready_timeout_s"]) + 90)

    def start_servo(self):
        item = self.state["processes"].get("servo")
        if item and self.alive(item):
            return
        self.probe("inactive")
        self.probe("no-target-publishers")
        self.probe("prepare-impedance")
        self.start("servo")

    def up(self, activate):
        self.assert_no_conflicts()
        group = "arm" if self.role == "arm" else "base"
        item = self.state["processes"].get(group)
        if not item or not self.alive(item):
            self.probe("vacant-" + group)
        if self.role == "base":
            self.start("base")
            if self.config["camera"]["mode"] == "managed":
                camera = self.state["processes"].get("camera")
                if not camera or not self.alive(camera):
                    self.probe("vacant-camera", camera=True)
                self.start("camera", camera=True)
            self.probe("base-ready")
            self.probe("camera-ready", camera=True)
        else:
            if any(self.config[key]["mode"] == "managed" for key in ("arms", "grippers", "spine")):
                self.start("arm")
            self.probe("arm-ready")
            self.start_servo()
            self.probe("runtime-ready")
            self.probe("camera-ready")
            if activate:
                self.probe("activate")
        self.health()
        print(json.dumps({"host": self.role, "status": "ready", "activated": bool(activate)}))

    def down(self):
        if any(self.alive(item) for item in self.state["processes"].values()) and self.state.get("release") != str(self.release):
            raise RuntimeError("refusing shutdown with a different release/configuration")
        if self.role == "arm" and any(self.alive(item) for item in self.state["processes"].values()):
            arm = self.state["processes"].get("arm")
            if self.config["arms"]["mode"] == "managed" and arm and self.alive(arm):
                self.probe("deactivate")
            self.probe("inactive")
        for name, item in reversed(list(self.state["processes"].items())):
            if self.alive(item):
                os.killpg(item["pid"], signal.SIGINT)
                deadline = time.monotonic() + 15
                while self.alive(item) and time.monotonic() < deadline:
                    time.sleep(0.2)
                if self.alive(item):
                    raise RuntimeError(f"{name} did not stop; left running for inspection")
            self.state["processes"].pop(name)
            self.save()

    def health(self):
        expected = ["servo"] if self.role == "arm" else ["base"]
        if self.role == "arm" and any(self.config[key]["mode"] == "managed" for key in ("arms", "grippers", "spine")):
            expected.append("arm")
        if self.role == "base" and self.config["camera"]["mode"] == "managed":
            expected.append("camera")
        if self.state.get("release") != str(self.release):
            raise RuntimeError("runtime release does not match this hardware configuration")
        for name in expected:
            item = self.state["processes"].get(name)
            if not item or not self.alive(item):
                raise RuntimeError(f"{name} is not running; inspect logs before restarting")
            fault = self.release / f"fault-{name}"
            if fault.exists():
                raise RuntimeError(f"{name} child process failed: {fault.read_text()}")

    def mission(self, execute):
        (self.root / "outputs").mkdir(exist_ok=True)
        if execute:
            self.health()
            self.probe("mission-ready")
        base_root = str(Path(self.config["hosts"]["base"]["runtime_root"]) / "releases" / self.release.name)
        env_file = base_root + "/base_env.sh"
        command = ["/app/entrypoint.sh", "mission",
                   "--calibration", str(self.release / "calibration.json"),
                   "--base-host", self.config["hosts"]["base"]["ssh"],
                   "--base-root", base_root + "/base/tmr_base", "--base-env", env_file,
                   "--base-container", self.config["hosts"]["base"]["container"],
                   "--base-release", self.release.name,
                   "--speed", str(self.config["runtime"]["playback_speed"])]
        if execute:
            command.append("--execute")
        self.native(command)

    def status(self):
        print(json.dumps({"host": self.role, "release": self.state.get("release"),
                          "processes": {name: {**item, "alive": self.alive(item)}
                                        for name, item in self.state["processes"].items()}}, indent=2))


def main():
    operation, role, raw_config, release = sys.argv[1:5]
    host = Host(json.loads(raw_config), role, release)
    if operation in ("check", "status", "health"):
        return getattr(host, operation)()
    if not (host.release / ".complete").is_file():
        if operation == "down" and not any(host.alive(item) for item in host.state["processes"].values()):
            return
        raise RuntimeError("this hardware/source release has not been deployed; run up")
    with (host.root / ".orchestration.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        host.state = json.loads(host.state_path.read_text()) if host.state_path.exists() else {"processes": {}}
        if operation == "up":
            host.up("--activate" in sys.argv)
        elif operation == "down":
            host.down()
        elif operation == "ready":
            host.probe("base-ready" if role == "base" else "runtime-ready")
        elif operation in ("mission", "mission-execute"):
            host.mission(operation == "mission-execute")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"hardware host: {error}", file=sys.stderr)
        raise SystemExit(1)
