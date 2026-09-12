import importlib.util
import io
import json
import os
import shlex
import subprocess
import sys
import tarfile
from unittest.mock import Mock

import pytest
import yaml

from franka_duo_tele_data.hardware import ROOT, Orchestrator, load_hardware
from franka_duo_tele_data.table_mission import MissionConfig, build_base_argv


def helper(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts/hardware" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def config():
    return load_hardware(ROOT / "hardware.yaml")


def container_info(release, *, running=True, image="image-id"):
    return {"Config": {"Labels": {"io.ebim.runtime": "base", "io.ebim.release": release}},
            "State": {"Running": running}, "Image": image}


def test_profile_has_no_native_base_workspace(config):
    assert config["hosts"]["base"]["runtime_root"] == "/app/runtime"
    assert config["hosts"]["base"]["overlays"] == ["/opt/ebim-base/install/setup.bash"]
    raw = yaml.safe_load((ROOT / "hardware.yaml").read_text())
    assert not {"runtime_root", "ros_setup", "overlays"} & raw["hosts"]["base"].keys()


def test_base_host_command_only_manages_docker(config):
    orchestrator = Orchestrator(config)
    orchestrator.execute = Mock()
    orchestrator.host("base", "up")
    role, args = orchestrator.execute.call_args.args
    assert role == "base" and args[:3] == ["/usr/bin/python3", "-B", "-c"]
    assert args[4] == "up"
    assert "class BaseDocker" in args[3] and "class Host:" not in args[3]


def test_base_container_has_gpu_usb_and_no_ros_workspace_mount(config, monkeypatch):
    module = helper("base_docker")
    monkeypatch.setattr(module.glob, "glob", lambda pattern: ["/dev/video0", "/dev/video1"])
    config["camera"]["sdk_settings_dir"] = ""
    args = module.BaseDocker(config, "release").options(persistent=True)
    assert args[args.index("--network") + 1] == "host"
    assert args[args.index("--runtime") + 1] == "nvidia"
    assert "c 189:* rmw" in args
    assert "/dev/video0" in args and "/dev/video1" in args
    assert not any("docker.sock" in arg or "ros2_ws" in arg or "--privileged" == arg for arg in args)
    assert any("dst=/app/runtime" in arg for arg in args)


def test_external_camera_needs_no_usb_devices(config):
    module = helper("base_docker")
    config["camera"]["mode"] = "external"
    args = module.BaseDocker(config, "release").options()
    assert "--device" not in args and "--runtime" not in args


@pytest.mark.parametrize("info", [container_info("other"), container_info("release", running=False),
                                 container_info("release", image="different-image")])
def test_deploy_cannot_replace_stopped_foreign_or_changed_container(config, monkeypatch, info):
    module = helper("base_docker")
    manager = module.BaseDocker(config, "release")
    manager.image_check = Mock(return_value="image-id")
    manager.info = Mock(return_value=info)
    run = Mock()
    monkeypatch.setattr(module, "docker", run)
    with pytest.raises(RuntimeError):
        manager.deploy(b"payload")
    run.assert_not_called()


def test_base_shutdown_failure_preserves_container(config, monkeypatch):
    module = helper("base_docker")
    manager = module.BaseDocker(config, "release")
    manager.info = Mock(return_value=container_info("release"))
    manager.inside = Mock(side_effect=RuntimeError("route holds lock"))
    run = Mock()
    monkeypatch.setattr(module, "docker", run)
    with pytest.raises(RuntimeError):
        manager.run("down")
    manager.inside.assert_called_once_with("down")
    run.assert_not_called()


def test_base_deploy_creates_keeper_before_sending_release(config, monkeypatch):
    module = helper("base_docker")
    manager = module.BaseDocker(config, "release")
    manager.image_check = Mock(return_value="image-id")
    manager.info = Mock(return_value=None)
    manager.no_native_conflicts = Mock()
    manager.options = Mock(return_value=["--network", "host"])
    run = Mock()
    monkeypatch.setattr(module, "docker", run)
    manager.deploy(b"tar-content")
    calls = run.call_args_list
    assert calls[0].args[0] == "run" and calls[0].args[-1] == "keep"
    assert calls[1].args[:2] == ("exec", "-i")
    assert calls[1].kwargs["input"] == b"tar-content"
    assert not any("build" in call.args or "pull" in call.args for call in calls)


def test_missing_container_differs_from_docker_daemon_failure(config, monkeypatch):
    module = helper("base_docker")
    monkeypatch.setattr(module, "docker", Mock(side_effect=subprocess.CalledProcessError(1, ["docker", "ls"])))
    with pytest.raises(subprocess.CalledProcessError):
        module.BaseDocker(config, "release").info()


def test_route_uses_heartbeat_client_and_container_environment():
    config = MissionConfig(base_host="tmr-user@base", base_root="/app/runtime/release/base/tmr_base",
                           arm_root="/app", arm_env="/opt/ros/jazzy/setup.bash", dataset="contract", speed=0.1,
                           init_timeout_s=1, outbound_timeout_s=1, stage_timeout_s=1, transition_settle_s=0,
                           base_env="/app/runtime/release/base_env.sh", base_container="base-container",
                           base_release="abcdef0123456789")
    command = build_base_argv(config, "run")
    assert command[1].endswith("base_route_client.py")
    assert command[2:5] == ["tmr-user@base", "base-container", "abcdef0123456789"]
    assert "source /app/runtime/release/base_env.sh" in command[-1]
    assert "ros2_ws" not in command[-1]


def test_route_client_ssh_command_preserves_shell_literal(monkeypatch):
    module = helper("base_route_client")
    literal = "printf '%s' 'literal $(never-execute)'; exit 0"
    monkeypatch.setattr(sys, "argv", ["client", "tmr-user@base", "base-container", "release", literal])
    client = Mock(return_value=0)
    monkeypatch.setattr(module, "client", client)
    assert module.main() == 0
    command = client.call_args.args[0]
    remote = shlex.split(command[-1])
    assert remote[:4] == ["docker", "exec", "-i", "base-container"]
    assert remote[-1] == literal


def test_no_heartbeat_means_no_route_start(monkeypatch):
    module = helper("base_route_watchdog")
    reader, writer = os.pipe()
    os.close(writer)
    run = Mock()
    monkeypatch.setattr(module.subprocess, "Popen", run)
    try:
        with pytest.raises(RuntimeError, match="client absent"):
            module.supervise(["must-not-run"], reader, heartbeat_timeout=0.1)
    finally:
        os.close(reader)
    run.assert_not_called()


@pytest.mark.parametrize("disconnect", ["eof", "timeout"])
def test_watchdog_stops_real_child_when_connection_lost(tmp_path, disconnect):
    child = tmp_path / "child.py"
    child.write_text("import signal,time,sys\nsignal.signal(signal.SIGINT, lambda *_: sys.exit(0))\nprint('ready', flush=True)\nwhile True: time.sleep(0.1)\n")
    code = "import sys; sys.path.insert(0,sys.argv[1]); from base_route_watchdog import supervise; sys.exit(supervise([sys.executable,sys.argv[2]],0,heartbeat_timeout=0.3))"
    process = subprocess.Popen([sys.executable, "-c", code, str(ROOT / "scripts/hardware"), str(child)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        process.stdin.write(b".\n")
        process.stdin.flush()
        assert process.stdout.readline().strip() == b"ready"
        if disconnect == "eof":
            process.stdin.close()
        assert process.wait(timeout=5) == 125
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if not process.stdin.closed:
            process.stdin.close()


def test_archive_deployment_rejects_path_traversal(tmp_path):
    payload = io.BytesIO()
    with tarfile.open(fileobj=payload, mode="w:gz") as tar:
        member = tarfile.TarInfo("../escape")
        member.size = 1
        tar.addfile(member, io.BytesIO(b"x"))
    with pytest.raises(RuntimeError, match="invalid deployment"):
        helper("deploy").deploy(tmp_path / "runtime", "abcdef0123456789", payload.getvalue())
    assert not (tmp_path / "escape").exists()


def test_watchdog_stops_descendant_after_shell_has_exited(tmp_path):
    stopped = tmp_path / "stopped"
    child = tmp_path / "descendant.py"
    child.write_text(
        "import signal,time,sys,pathlib\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "def stop(*_):\n"
        " pathlib.Path(sys.argv[1]).write_text('stopped'); sys.exit(0)\n"
        "signal.signal(signal.SIGTERM, stop)\n"
        "print('ready', flush=True)\n"
        "while True: time.sleep(0.1)\n")
    parent = tmp_path / "parent.py"
    parent.write_text(
        "import subprocess,sys\n"
        "child=subprocess.Popen([sys.executable,sys.argv[1],sys.argv[2]],stdout=subprocess.PIPE)\n"
        "assert child.stdout.readline().strip() == b'ready'\n")
    code = (
        "import sys; sys.path.insert(0,sys.argv[1]); from base_route_watchdog import supervise; "
        "sys.exit(supervise([sys.executable,*sys.argv[2:]],0,heartbeat_timeout=10))")
    process = subprocess.Popen([sys.executable, "-c", code, str(ROOT / "scripts/hardware"),
                                str(parent), str(child), str(stopped)], stdin=subprocess.PIPE)
    try:
        process.stdin.write(b".\n")
        process.stdin.flush()
        assert process.wait(timeout=7) == 0
        assert stopped.read_text() == "stopped"
    finally:
        process.stdin.close()
        if process.poll() is None:
            process.kill()
            process.wait()


def test_driver_lock_and_base_image_target_inspected_platform():
    sources = json.loads((ROOT / "docker/base_drivers.lock.json").read_text())
    assert sources["franka_ros2"]["commit"].startswith("1006036")
    assert sources["zed_ros2_wrapper"]["commit"].startswith("458c725")
    assert all(len(source["commit"]) == 40 for source in sources.values())
    dockerfile = (ROOT / "docker/base.Dockerfile").read_text()
    assert "5.1.2-devel-l4t-r36.4@sha256:" in dockerfile
    assert "ros-humble-slam-toolbox" in dockerfile and "ros-humble-pinocchio" in dockerfile


def test_duplicate_yaml_controller_blocks_are_merged():
    spec = importlib.util.spec_from_file_location("prepare", ROOT / "docker/prepare_base_sources.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    value = yaml.load('/**:\n  controller_manager: {rate: 1000}\n/**:\n  swerve: {speed: 0.35}\n', Loader=module.MergeLoader)
    assert value["/**"] == {"controller_manager": {"rate": 1000}, "swerve": {"speed": 0.35}}
    with pytest.raises(ValueError):
        yaml.load('/**:\n  swerve: 1\n/**:\n  swerve: 2\n', Loader=module.MergeLoader)
