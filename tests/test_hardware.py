import importlib.util
import ast
import io
import json
import os
import shlex
import subprocess
import sys
import tarfile
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from franka_duo_tele_data.hardware import ROOT, Orchestrator, archive, bundle_files, load_hardware, release_id, host_argv


def load_helper(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts/hardware" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def config():
    return load_hardware(ROOT / "hardware.yaml")


def test_invalid_profiles_rejected_before_any_remote_work(tmp_path):
    source = yaml.safe_load((ROOT / "hardware.yaml").read_text())
    source["camera"]["calibration"] = str(ROOT / "configs/zed_pnp_calibration.json")
    for section, key, value in [(None, "profile", "arbitrary_robot"), ("domains", "base", 0),
                                ("arms", "mode", "managd"), ("arms", "left_ip", "bad;command"),
                                ("runtime", "playback_speed", -1)]:
        candidate = json.loads(json.dumps(source))
        (candidate if section is None else candidate[section])[key] = value
        path = tmp_path / "hardware.yaml"
        path.write_text(yaml.safe_dump(candidate))
        with pytest.raises(ValueError):
            load_hardware(path)


def test_bundle_includes_executable_routes_and_no_host_navigation_install(config):
    files = bundle_files(config)
    payload = archive(files)
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as tar:
        names = tar.getnames()
        for name in ("07_start_to_pickup.py", "13_post_grasp_route.py", "15_return_from_letter.py", "20_after_return_placement.py"):
            assert "base/tmr_cycle/scripts/" + name in names
        assert "base/tmr_navigation/tmr_local_navigation/odom_frame_adapter.py" in names
        assert "scripts/hardware/franka.launch.py" in names
        assert not any("__pycache__" in name for name in names)
        env = tar.extractfile("base_env.sh").read().decode()
        assert "tmr_navigation/install" not in env
        assert "ROS_DOMAIN_ID=97" in env
        assert "dds_base.xml" in env
    assert release_id(files) == release_id(bundle_files(config))
    files["base/tmr_cycle/scripts/07_start_to_pickup.py"] += b"\n"
    assert release_id(files) != release_id(bundle_files(config))


def test_base_ssh_is_direct_and_preserves_literals(config):
    args = ["/usr/bin/python3", "-c", "print('$(touch /tmp/not-executed)')", "path with spaces"]
    outer = host_argv(config, "base", args)
    assert outer[0] == "ssh"
    assert outer[-2] == config["hosts"]["base"]["ssh"]
    assert shlex.split(outer[-1]) == args
    assert "StrictHostKeyChecking=yes" in outer


@pytest.mark.parametrize("operation", ["check", "up", "status", "down", "mission", "mission-execute"])
def test_arm_operations_execute_locally_without_ssh(config, monkeypatch, operation):
    assert "ssh" not in config["hosts"]["arm"]
    runner = Mock(return_value=SimpleNamespace(stdout=b""))
    monkeypatch.setattr(subprocess, "run", runner)
    Orchestrator(config).host("arm", operation)
    command = runner.call_args.args[0]
    assert command[:3] == ["/usr/bin/python3", "-B", "-c"]
    assert command[4:6] == [operation, "arm"]
    assert "100.79.180.52" not in shlex.join(command)


def test_local_execution_preserves_arguments_and_deployment_payload(config, monkeypatch):
    runner = Mock(return_value=SimpleNamespace(stdout=b""))
    monkeypatch.setattr(subprocess, "run", runner)
    args = ["/usr/bin/python3", "-B", "-c", "print('literal')", "path with spaces;$HOME"]
    Orchestrator(config).execute("arm", args, payload=b"archive-content")
    assert runner.call_args.args[0] == args
    assert runner.call_args.kwargs["input"] == b"archive-content"


def test_obsolete_arm_ssh_config_rejected(tmp_path):
    config = yaml.safe_load((ROOT / "hardware.yaml").read_text())
    config["hosts"]["arm"]["ssh"] = "aup@old-machine"
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="run on the arm host locally"):
        load_hardware(path)


def test_plan_never_connects(config, monkeypatch, capsys):
    monkeypatch.setattr(subprocess, "run", Mock(side_effect=AssertionError("network access")))
    Orchestrator(config).run("plan")
    assert json.loads(capsys.readouterr().out)["profile"] == "tmr_fr3v2_duo"


def test_failed_preflight_does_not_deploy(config):
    orchestrator = Orchestrator(config)
    orchestrator.host = Mock(side_effect=RuntimeError("missing driver"))
    orchestrator.deploy = Mock()
    with pytest.raises(RuntimeError):
        orchestrator.run("up", activate=True)
    orchestrator.deploy.assert_not_called()


def test_up_orders_base_before_arm_and_activation_is_explicit(config):
    orchestrator = Orchestrator(config)
    events = []
    orchestrator.host = lambda *args, **kwargs: events.append((args, kwargs))
    orchestrator.deploy = lambda: events.append("deploy")
    orchestrator.run("up")
    assert events == [(("arm", "check"), {}), (("base", "check"), {}), "deploy",
                      (("base", "up"), {}), (("arm", "up"), {"activate": False})]


def test_down_does_not_stop_base_when_arm_refuses(config):
    orchestrator = Orchestrator(config)
    orchestrator.host = Mock(side_effect=RuntimeError("active impedance"))
    with pytest.raises(RuntimeError):
        orchestrator.run("down")
    orchestrator.host.assert_called_once_with("arm", "down")


def test_activation_alignment_reorders_measured_but_rejects_reordered_target():
    alignment = load_helper("alignment")
    names = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
    target = SimpleNamespace(name=names, position=[i * 0.1 for i in range(7)])
    measured = SimpleNamespace(name=list(reversed(names)), position=list(reversed(target.position)))
    assert alignment.alignment_error(target, measured, "left") == 0
    with pytest.raises(RuntimeError, match="order"):
        alignment.alignment_error(measured, target, "left")
    measured.position[0] += 0.01
    with pytest.raises(RuntimeError, match="differs"):
        alignment.alignment_error(target, measured, "left")
    measured.position[0] = float("nan")
    with pytest.raises(RuntimeError, match="non-finite"):
        alignment.alignment_error(target, measured, "left")


def test_host_refuses_unmanaged_drivers_without_killing(config, tmp_path, monkeypatch):
    helper = load_helper("host")
    config["hosts"]["base"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "base", "release")
    monkeypatch.setattr(helper, "process_commands", lambda: [(123, "ros2 launch zed_wrapper zed_camera.launch.py")])
    monkeypatch.setattr(helper.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(helper.os, "killpg", Mock(side_effect=AssertionError("unexpected kill")))
    with pytest.raises(RuntimeError, match="unmanaged camera"):
        host.assert_no_conflicts()
    config["camera"]["mode"] = "external"
    host.assert_no_conflicts()


def test_pid_reuse_is_not_treated_as_owned(config, tmp_path, monkeypatch):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "arm", "release")
    monkeypatch.setattr(helper, "process_identity", lambda pid: "new-process")
    assert not host.alive({"pid": 123, "start": "old-process"})


def test_custom_base_environment_replaces_legacy_sources():
    from franka_duo_tele_data.table_mission import MissionConfig, build_remote_base_shell
    config = MissionConfig(base_host="base@host", base_root="/runtime/routes", arm_root="/app", arm_env="/ros",
                           dataset="/contract", speed=0.1, init_timeout_s=1, outbound_timeout_s=1,
                           stage_timeout_s=1, transition_settle_s=0, base_env="/runtime/profile env.sh")
    shell = build_remote_base_shell(config, "test")
    assert "source '/runtime/profile env.sh'" in shell
    assert "tmr_navigation/install" not in shell
    assert "/runtime/routes/scripts/07_start_to_pickup.py" in shell


def test_actual_archive_deployment_is_versioned_and_repeatable(config, tmp_path):
    for role in ("arm", "base"):
        config["hosts"][role]["runtime_root"] = str(tmp_path / role)
    orchestrator = Orchestrator(config)

    def local_remote(role, args, payload=None):
        import sys
        return subprocess.run([sys.executable, *args[1:]], input=payload, capture_output=True, check=True)

    orchestrator.execute = local_remote
    def local_base_deploy(operation, payload):
        assert operation == "deploy"
        load_helper("deploy").deploy(tmp_path / "base", orchestrator.release, payload)
    orchestrator.base_docker = local_base_deploy
    orchestrator.deploy()
    route = tmp_path / "base/releases" / orchestrator.release / "base/tmr_cycle/scripts/07_start_to_pickup.py"
    assert route.read_bytes() == (ROOT / "base/tmr_cycle/scripts/07_start_to_pickup.py").read_bytes()
    initial_mtime = route.stat().st_mtime_ns
    orchestrator.deploy()
    assert route.stat().st_mtime_ns == initial_mtime
    assert not (tmp_path / "base/tmr_cycle").exists()


def test_start_servo_checks_controller_state_before_starting_targets(config, tmp_path, monkeypatch):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "arm", "release")
    host.probe = Mock(side_effect=RuntimeError("active controller"))
    host.start = Mock(side_effect=AssertionError("servo started"))
    with pytest.raises(RuntimeError, match="active controller"):
        host.start_servo()
    host.probe.assert_called_once_with("inactive")
    host.start.assert_not_called()


def activation_probe():
    # Exercise the actual activation method without importing ROS binary modules.
    tree = ast.parse((ROOT / "scripts/hardware/probe.py").read_text())
    definition = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Probe")
    commands = Mock()
    namespace = {"SIDES": ("left", "right"), "qos_profile_sensor_data": None,
                 "subprocess": SimpleNamespace(run=commands),
                 "alignment_error": load_helper("alignment").alignment_error}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), "probe.py", "exec"), namespace)
    probe = namespace["Probe"].__new__(namespace["Probe"])
    probe.runtime_ready = Mock()
    probe.controllers = Mock(return_value={"joint_impedance_controller": "inactive"})
    probe.inactive = Mock()
    probe.errors_and_mode = Mock()
    probe.servo_status = Mock(return_value={"started": False, "idle_latched": False})
    probe.wait = Mock()
    probe.node = SimpleNamespace(count_publishers=lambda topic: 0)
    probe.latest = {}
    for side in ("left", "right"):
        for suffix in ("gello/joint_states", "franka_robot_state_broadcaster/measured_joint_states"):
            message = SimpleNamespace(name=[f"{side}_fr3v2_joint{i}" for i in range(1, 8)], position=[0.5] * 7)
            probe.latest[f"/{side}/{suffix}"] = (message, 0)
    return probe, commands


def test_latched_servo_never_activates_controllers():
    probe, commands = activation_probe()
    probe.servo_status.return_value["idle_latched"] = True
    with pytest.raises(RuntimeError, match="no longer idle-following"):
        probe.activate()
    commands.assert_not_called()


def test_partial_activation_failure_preserves_stream_and_does_not_activate_other_arm():
    probe, commands = activation_probe()
    probe.latest["/right/gello/joint_states"][0].position[0] = 0.7
    with pytest.raises(RuntimeError, match="differs"):
        probe.activate()
    assert commands.call_count == 1
    assert commands.call_args.args[0] == ["ros2", "control", "switch_controllers", "-c",
                                          "/left/controller_manager", "--activate", "joint_impedance_controller"]


def test_unknown_controller_state_cannot_activate():
    probe, commands = activation_probe()
    probe.controllers.return_value = {}
    with pytest.raises(RuntimeError, match="configured before"):
        probe.activate()
    commands.assert_not_called()


def docker_launcher():
    spec = importlib.util.spec_from_file_location("launch_args", ROOT / "docker/launch_args.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_docker_device_mapping_comes_from_profile_and_has_no_host_ros_mount(config):
    args = docker_launcher().launch_args(ROOT / "hardware.yaml", config["image"], "/home/operator/.ssh",
                                         "ebim-runtime", "up", activate=True)
    assert args[args.index("--network") + 1] == "host"
    assert args[-4:] == [config["image"], "hardware", "serve", "--activate"]
    for side in ("left", "right"):
        assert f"{config['grippers'][side + '_port']}:/dev/ebim-{side}-gripper:rw" in args
    assert "type=bind,src=/home/operator/.ssh,dst=/root/.ssh,readonly" in args
    assert not any("docker.sock" in arg or "site/install" in arg or "--privileged" == arg for arg in args)
    assert "rtprio=99:99" in args and "memlock=-1:-1" in args


def test_external_grippers_need_no_device_passthrough(tmp_path, config):
    profile = yaml.safe_load((ROOT / "hardware.yaml").read_text())
    profile["grippers"]["mode"] = "external"
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(profile))
    args = docker_launcher().launch_args(path, config["image"], "/home/operator/.ssh", "ebim", "up")
    assert "--device" not in args


def test_runtime_maps_serial_ports_and_calibration_override(config, monkeypatch):
    monkeypatch.setenv("EBIM_CONTAINER_RUNTIME", "1")
    monkeypatch.setenv("EBIM_CALIBRATION", config["camera"]["calibration"])
    loaded = load_hardware(ROOT / "hardware.yaml")
    assert loaded["grippers"]["left_port"] == "/dev/ebim-left-gripper"
    assert loaded["hosts"]["arm"]["ros_setup"] == "/opt/ros/jazzy/setup.bash"
    assert loaded["hosts"]["arm"]["overlays"][0] == "/opt/ebim-drivers/install/setup.bash"


def test_down_deactivates_before_stopping_streams(config, tmp_path, monkeypatch):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "arm", "release")
    host.state["processes"] = {"arm": {"pid": 10}, "servo": {"pid": 20}}
    host.state["release"] = str(host.release)
    running = {10, 20}
    events = []
    host.alive = lambda item: item["pid"] in running
    host.probe = lambda operation: events.append(operation)
    def kill(pid, sig):
        events.append(pid)
        running.remove(pid)
    monkeypatch.setattr(helper.os, "killpg", kill)
    host.down()
    assert events == ["deactivate", "inactive", 20, 10]


def test_deactivation_failure_preserves_all_processes(config, tmp_path, monkeypatch):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "arm", "release")
    host.state["processes"] = {"arm": {"pid": 10}, "servo": {"pid": 20}}
    host.state["release"] = str(host.release)
    host.alive = lambda item: True
    host.probe = Mock(side_effect=RuntimeError("deactivation failed"))
    kill = Mock()
    monkeypatch.setattr(helper.os, "killpg", kill)
    with pytest.raises(RuntimeError, match="deactivation failed"):
        host.down()
    kill.assert_not_called()
    assert len(host.state["processes"]) == 2


def test_external_arms_are_never_automatically_deactivated(config, tmp_path):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    config["arms"]["mode"] = "external"
    host = helper.Host(config, "arm", "release")
    host.state["processes"] = {"servo": {"pid": 20}}
    host.state["release"] = str(host.release)
    host.alive = lambda item: True
    host.probe = Mock(side_effect=RuntimeError("external controller active"))
    with pytest.raises(RuntimeError):
        host.down()
    host.probe.assert_called_once_with("inactive")


def test_healthy_launch_parent_cannot_hide_child_failure(config, tmp_path):
    helper = load_helper("host")
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = helper.Host(config, "arm", "release")
    host.release.mkdir(parents=True)
    host.state = {"release": str(host.release), "processes": {"arm": {"pid": 10}, "servo": {"pid": 20}}}
    host.alive = lambda item: True
    (host.release / "fault-servo").write_text("relay exited")
    with pytest.raises(RuntimeError, match="relay exited"):
        host.health()


def test_supervisor_preserves_runtime_after_partial_start_failure(config, tmp_path, monkeypatch):
    from franka_duo_tele_data import hardware_service
    config["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    orchestrator = SimpleNamespace(config=config, release="test", run=Mock(side_effect=RuntimeError("right arm")))
    monkeypatch.setattr(hardware_service, "runtime_instance", lambda: "new-container")
    monkeypatch.setattr(hardware_service.signal, "signal", Mock())
    # Exit the infinite keeper loop only from the test, after observing one tick.
    monkeypatch.setattr(hardware_service.time, "sleep", Mock(side_effect=KeyboardInterrupt))
    with pytest.raises(KeyboardInterrupt):
        hardware_service.serve(orchestrator, activate=True)
    orchestrator.run.assert_called_once_with("up", activate=True)
    status = json.loads((tmp_path / "service.json").read_text())
    assert status["state"] == "failed" and status["instance"] == "new-container"


def test_deactivate_refuses_other_active_command_controller():
    probe, commands = activation_probe()
    probe.controllers.return_value = {"joint_impedance_controller": "active", "another_controller": "active"}
    with pytest.raises(RuntimeError, match="unmanaged command controllers"):
        probe.deactivate()
    commands.assert_not_called()


def test_failed_base_readiness_prevents_mission(config):
    orchestrator = Orchestrator(config)
    orchestrator.host = Mock(side_effect=[None, RuntimeError("map unavailable")])
    with pytest.raises(RuntimeError):
        orchestrator.run("mission", execute=True)
    assert [call.args for call in orchestrator.host.call_args_list] == [("base", "health"), ("base", "ready")]


def test_controller_yaml_keeps_manager_and_controller_parameters():
    overlay = ROOT / "hosts/arm/teleoperation_overlay/src"
    for package, filename, controller in (("franka_fr3_arm_controllers", "controllers.yaml", "joint_impedance_controller"),
                                         ("franka_gripper_manager", "robotiq_controllers.yaml", "robotiq_gripper_controller")):
        settings = yaml.safe_load((overlay / package / "config" / filename).read_text())["/**"]
        assert settings["controller_manager"]["ros__parameters"][controller]["type"]
        assert settings[controller]["ros__parameters"]
        if controller == "joint_impedance_controller":
            assert settings[controller]["ros__parameters"]["arm_id"] == "fr3v2"


def test_shell_launcher_preserves_paths_without_evaluating_them(tmp_path, config):
    # Exercise Bash -> helper -> NUL argv -> Docker with no Docker daemon or ROS.
    profile_dir = tmp_path / "profile with spaces;$HOME"
    profile_dir.mkdir()
    source = yaml.safe_load((ROOT / "hardware.yaml").read_text())
    source["camera"]["calibration"] = config["camera"]["calibration"]
    profile = profile_dir / "hardware.yaml"
    profile.write_text(yaml.safe_dump(source))
    docker = tmp_path / "docker"
    docker.write_text(f"#!{sys.executable}\n" + '''import json, os, subprocess, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["DOCKER_TEST_LOG"], "a") as log:
    log.write(json.dumps(args) + "\\n")
if "/app/docker/launch_args.py" in args:
    tail = args[args.index("/app/docker/launch_args.py") + 1:]
    result = subprocess.run([sys.executable, os.environ["DOCKER_TEST_GENERATOR"], *tail])
    sys.exit(result.returncode)
''')
    docker.chmod(0o755)
    logfile = tmp_path / "docker.jsonl"
    env = {**os.environ, "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"],
           "DOCKER_TEST_LOG": str(logfile), "DOCKER_TEST_GENERATOR": str(ROOT / "docker/launch_args.py"),
           "EBIM_IMAGE": config["image"], "EBIM_SSH_DIR": "/home/operator/.ssh"}
    subprocess.run(["bash", str(ROOT / "scripts/docker_hardware.sh"), "plan", "--hardware", str(profile)],
                   check=True, env=env, capture_output=True)
    commands = [json.loads(line) for line in logfile.read_text().splitlines()]
    assert len(commands) == 2
    assert commands[0][commands[0].index("--network") + 1] == "none"
    assert commands[1][-3:] == [config["image"], "hardware", "plan"]
    assert f"type=bind,src={profile},dst=/hardware/hardware.yaml,readonly" in commands[1]
    assert not any(arg in ("build", "pull") for command in commands for arg in command)


def test_shell_down_never_stops_docker_when_controller_shutdown_refused(tmp_path):
    docker = tmp_path / "docker"
    docker.write_text(f"#!{sys.executable}\n" + '''import json, os, sys
args = sys.argv[1:]
with open(os.environ["DOCKER_TEST_LOG"], "a") as log:
    log.write(json.dumps(args) + "\\n")
if args[0] == "inspect":
    print("arm")
elif args[0] == "exec":
    sys.exit(1)
else:
    sys.exit(99)
''')
    docker.chmod(0o755)
    logfile = tmp_path / "docker.jsonl"
    result = subprocess.run(["bash", str(ROOT / "scripts/docker_hardware.sh"), "down"],
                            env={**os.environ, "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"],
                                 "DOCKER_TEST_LOG": str(logfile)}, capture_output=True)
    assert result.returncode != 0
    commands = [json.loads(line) for line in logfile.read_text().splitlines()]
    assert [command[0] for command in commands] == ["inspect", "exec"]
