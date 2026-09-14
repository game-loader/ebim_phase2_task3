import ast
import ast
import importlib.util
import json
import shlex
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

from franka_duo_tele_data.camera_input import camera_matrix, require_saved_image_intrinsics
from franka_duo_tele_data.hardware import ROOT, Orchestrator, bundle_files, fastdds_xml, load_hardware
from franka_duo_tele_data.table_mission import MissionConfig, build_base_argv


@pytest.fixture
def hamburg():
    return load_hardware(ROOT / "hardware.hamburg.yaml")


@pytest.mark.parametrize("key", ["alignment_tolerance_rad", "max_tracking_error_rad"])
@pytest.mark.parametrize("value", [0, -0.1, float("nan"), float("inf"), True, "0.2", None])
def test_invalid_motion_guard_rejected_before_ros(tmp_path, key, value):
    raw = yaml.safe_load((ROOT / "hardware.hamburg.yaml").read_text())
    raw["runtime"][key] = value
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match=key):
        load_hardware(path, require_calibration=False)


def test_motion_guard_defaults_and_overrides(tmp_path, hamburg):
    legacy = load_hardware(ROOT / "hardware.yaml")
    assert legacy["runtime"]["alignment_tolerance_rad"] == 0.003
    assert legacy["runtime"]["max_tracking_error_rad"] == 0.15
    assert hamburg["runtime"]["alignment_tolerance_rad"] == 0.01
    assert hamburg["runtime"]["max_tracking_error_rad"] == 0.2
    raw = yaml.safe_load((ROOT / "hardware.hamburg.yaml").read_text())
    raw["runtime"]["max_tracking_error_rad"] = 0.4
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump(raw))
    config = load_hardware(path, require_calibration=False)
    assert config["runtime"]["max_tracking_error_rad"] == 0.4
    config["camera"]["calibration"] = str(ROOT / "configs/zed_pnp_calibration.json")
    assert Orchestrator(config).release != Orchestrator(hamburg).release


@pytest.mark.parametrize("operation", ["activate_impedance", "mission_ready"])
@pytest.mark.parametrize("error, accepted", [(0.003076, True), (0.02, False)])
def test_external_alignment_uses_config_at_activation_and_mission(operation, error, accepted):
    from types import SimpleNamespace

    spec = importlib.util.spec_from_file_location("alignment", ROOT / "scripts/hardware/alignment.py")
    alignment = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(alignment)
    tree = ast.parse((ROOT / "scripts/hardware/external_probe.py").read_text())
    definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ExternalProbe")
    namespace = {"Probe": object, "SIDES": ("left", "right"), "alignment_error": alignment.alignment_error}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), "external_probe.py", "exec"), namespace)
    probe = namespace["ExternalProbe"]()
    probe.config = {"runtime": {"alignment_tolerance_rad": 0.01}}
    probe.runtime_ready = Mock()
    probe.servo_status = Mock(return_value={"idle_latched": True})
    probe.wait = lambda predicate, *args: predicate()
    probe.fresh = lambda *args: True
    probe.controllers = Mock(return_value={"joint_impedance_controller": "active"})
    probe.switch = Mock()
    probe.deactivate_impedance = Mock()
    probe.latest = {}
    for side in ("left", "right"):
        names = [f"{side}_fr3v2_joint{i}" for i in range(1, 8)]
        probe.latest[f"/{side}/gello/joint_states"] = (SimpleNamespace(name=names, position=[error] * 7), 0)
        probe.latest[f"/{side}/franka_robot_state_broadcaster/measured_joint_states"] = (
            SimpleNamespace(name=names, position=[0.] * 7), 0)
    if accepted:
        getattr(probe, operation)()
        if operation == "activate_impedance":
            assert probe.switch.call_count == 2
    else:
        with pytest.raises(RuntimeError, match="tolerance 0.010000"):
            getattr(probe, operation)()
        probe.switch.assert_not_called()


def test_external_plan_never_connects_and_needs_no_hardware_addresses(hamburg, monkeypatch, capsys):
    monkeypatch.setattr(subprocess, "run", Mock(side_effect=AssertionError("unexpected connection")))
    runner = Orchestrator(hamburg)
    runner.plan()
    plan = json.loads(capsys.readouterr().out)
    assert plan["domains"] == {"arm": 0, "base": 0}
    assert plan["base_container_required"] is False
    assert set(plan["components"].values()) == {"external"}
    runner.host("base", "up")
    assert hamburg["hosts"]["arm"]["ros_setup"] == "/opt/ros/humble/setup.bash"


def test_external_docker_has_no_ssh_devices_or_vendor_mounts(hamburg):
    spec = importlib.util.spec_from_file_location("launcher", ROOT / "docker/launch_args.py")
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    args = launcher.launch_args(ROOT / "hardware.hamburg.yaml", hamburg["image"], "/absent/ssh", "ebim", "check")
    assert "--device" not in args
    assert not any("ssh" in value or "docker.sock" in value for value in args)
    assert args[args.index("--network") + 1] == "host"


def test_external_policy_uses_live_camera_and_organizer_tcp(hamburg):
    files = bundle_files(hamburg)
    policy = yaml.safe_load(files["policy.yaml"])
    assert policy["camera_intrinsics"] == "live_rectified"
    assert policy["topics"]["head"] == "/head_camera/zed_node/rgb/color/rect/image"
    assert policy["topics"]["camera_info"] == "/head_camera/zed_node/rgb/color/rect/camera_info"
    assert policy["topics"]["left_pose"] == "/left/franka_robot_state_broadcaster/current_pose"
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in files["base_env.sh"].decode()
    assert "ROS_DOMAIN_ID=0" in files["base_env.sh"].decode()
    assert "ebim-base" not in files["base_env.sh"].decode()


def test_external_rejects_managed_component_or_split_domains(tmp_path):
    raw = yaml.safe_load((ROOT / "hardware.hamburg.yaml").read_text())
    raw["camera"]["calibration"] = str(ROOT / "configs/zed_pnp_calibration.json")
    for section, key, value in [("arms", "mode", "managed"), ("domains", "base", 97),
                                ("camera", "image_topic", "")]:
        candidate = json.loads(json.dumps(raw))
        candidate[section][key] = value
        path = tmp_path / "hardware.yaml"
        path.write_text(yaml.safe_dump(candidate))
        with pytest.raises(ValueError):
            load_hardware(path)


def test_external_activation_uses_only_local_host(hamburg):
    runner = Orchestrator(hamburg)
    runner.host = Mock()
    runner.deploy = Mock()
    runner.run("up", activate=True)
    runner.deploy.assert_called_once()
    assert runner.host.call_args.args == ("arm", "up")
    assert runner.host.call_args.kwargs == {"activate": True}


def test_local_route_runs_under_heartbeat_supervision():
    config = MissionConfig(base_host="unused", base_root="/app/runtime/base/tmr_base", arm_root="/app",
                           arm_env="/opt/ros/humble/setup.bash", dataset="/contract", speed=0.1,
                           init_timeout_s=1, outbound_timeout_s=1, stage_timeout_s=1, transition_settle_s=0,
                           base_local=True, base_env="/app/runtime/base_env.sh")
    command = build_base_argv(config, "test")
    assert command[2] == "--local"
    assert Path(command[1]).name == "base_route_client.py"
    assert "ssh" not in shlex.join(command)
    assert "07_start_to_pickup.py" in command[-1]
    assert "flock -n" in command[-1]


def frame_pair():
    header = SimpleNamespace(frame_id="head_left_optical")
    image = SimpleNamespace(width=640, height=360, header=header)
    info = SimpleNamespace(width=640, height=360, header=header,
                           k=[320., 0., 310., 0., 325., 175., 0., 0., 1.], d=[], p=[0.] * 12)
    return image, info


def test_external_uses_new_unit_intrinsics_without_reading_reference_k():
    image, info = frame_pair()
    np.testing.assert_array_equal(camera_matrix(image, info, {"camera_intrinsics": "live_rectified"}, {}),
                                  np.asarray(info.k).reshape(3, 3))
    info.p = [330., 0., 315., 0., 0., 331., 180., 0., 0., 0., 1., 0.]
    result = camera_matrix(image, info, {"camera_intrinsics": "live_rectified"}, {})
    assert result[0, 0] == 330. and result[1, 1] == 331.


def test_external_camera_rejects_mismatched_frames_dimensions_and_bad_projection():
    for change in ("frame", "size", "nan", "projection"):
        image, info = frame_pair()
        if change == "frame":
            image.header = SimpleNamespace(frame_id="other_camera")
        elif change == "size":
            info.width = 1280
        elif change == "nan":
            info.k[0] = float("nan")
        else:
            info.p[0] = 1.
        with pytest.raises(ValueError):
            camera_matrix(image, info, {"camera_intrinsics": "live_rectified"}, {})


def test_reference_mode_still_checks_intrinsics_and_external_saved_frame_is_rejected():
    image, info = frame_pair()
    with pytest.raises(ValueError, match="differ"):
        camera_matrix(image, info, {}, {"camera": {"K": np.eye(3).tolist()}})
    with pytest.raises(ValueError, match="requires live"):
        require_saved_image_intrinsics({"camera_intrinsics": "live_rectified"})


def external_host(hamburg, tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts/hardware"))
    spec = importlib.util.spec_from_file_location("external_host", ROOT / "scripts/hardware/external.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hamburg["hosts"]["arm"]["runtime_root"] = str(tmp_path)
    host = module.ExternalHost(hamburg, "arm", "test")
    host.assert_no_conflicts = Mock()
    host.health = Mock()
    host.start = Mock()
    host.start_gateway = Mock()
    return host


@pytest.mark.parametrize("failure", ["inactive", "unowned"])
def test_failed_handoff_starts_no_target_stream(hamburg, tmp_path, monkeypatch, failure):
    host = external_host(hamburg, tmp_path, monkeypatch)
    def probe(operation):
        if operation == failure:
            raise RuntimeError("handoff incomplete")
    host.probe = probe
    with pytest.raises(RuntimeError, match="handoff incomplete"):
        host.up(False)
    host.start.assert_not_called()


def test_external_start_never_loads_drivers_or_switches_controllers(hamburg, tmp_path, monkeypatch):
    host = external_host(hamburg, tmp_path, monkeypatch)
    host.probe = Mock()
    host.up(False)
    assert [c.args[0] for c in host.probe.call_args_list] == ["inactive", "unowned", "check", "configure-impedance", "runtime-ready"]
    assert [c.args[0] for c in host.start.call_args_list] == ["external-servo", "routes"]


def test_existing_command_publisher_blocks_handoff():
    tree = ast.parse((ROOT / "scripts/hardware/external_probe.py").read_text())
    definition = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    commands = ["/left/gello/joint_states", "/right/gello/joint_states", "/swerve_drive_controller/cmd_vel"]
    namespace = {"Probe": object, "COMMANDS": commands}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), "external_probe.py", "exec"), namespace)
    probe = namespace["ExternalProbe"]()
    probe.wait = Mock()
    for occupied in commands:
        probe.node = SimpleNamespace(count_publishers=lambda topic, occupied=occupied: int(topic == occupied or topic == "/franka_duo/joint_servo/action_chunk"))
        with pytest.raises(RuntimeError, match="handoff required"):
            probe.unowned()


def local_route_module(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts/hardware"))
    spec = importlib.util.spec_from_file_location("local_route", ROOT / "scripts/hardware/local_route.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_route_only_plan_never_starts_processes(monkeypatch, capsys):
    module = local_route_module(monkeypatch)
    monkeypatch.setattr(module.os, "execvpe", Mock(side_effect=AssertionError("unexpected process")))
    monkeypatch.setattr(module.ExternalHost, "health", Mock(side_effect=AssertionError("unexpected runtime check")))
    assert module.main(["outbound", "--hardware", str(ROOT / "hardware.hamburg.yaml")]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["motion_enabled"] is False
    assert "--local" in plan["command"]


def test_route_only_preserves_lock_across_exec(hamburg, tmp_path, monkeypatch):
    import fcntl

    module = local_route_module(monkeypatch)
    host = external_host(hamburg, tmp_path, monkeypatch)
    host.probe = Mock()
    monkeypatch.setattr(module, "ExternalHost", lambda *args: host)
    def execute(file, command, env):
        with (host.root / ".orchestration.lock").open("a") as second, pytest.raises(BlockingIOError):
            fcntl.flock(second, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert "base_route_client.py" in command[-1]
        assert env["ROS_DOMAIN_ID"] == "0"
    monkeypatch.setattr(module.os, "execvpe", execute)
    assert module.main(["outbound", "--hardware", str(ROOT / "hardware.hamburg.yaml"), "--execute"]) == 0
    host.probe.assert_called_once_with("route-ready")


def test_route_placement_requires_return_run_id(monkeypatch):
    module = local_route_module(monkeypatch)
    with pytest.raises(SystemExit):
        module.main(["placement", "--execute"])


def test_external_launcher_passes_explicit_activation(hamburg):
    spec = importlib.util.spec_from_file_location("launcher", ROOT / "docker/launch_args.py")
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    args = launcher.launch_args(ROOT / "hardware.hamburg.yaml", hamburg["image"], "/absent", "ebim", "up", True)
    assert args[-1] == "--activate"


def test_fastdds_preserves_organizer_transport_and_replaces_station_ip():
    root = ET.fromstring(fastdds_xml("192.168.50.4"))
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    assert [e.text for e in root.findall(".//d:interfaceWhiteList/d:address", ns)] == ["192.168.50.4", "127.0.0.1"]
    for tag, expected in [("type", "UDPv4"), ("maxMessageSize", "65500"),
                          ("sendBufferSize", "4194304"), ("receiveBufferSize", "16777216"),
                          ("useBuiltinTransports", "false")]:
        assert root.find(".//d:" + tag, ns).text == expected


def test_gateway_is_never_recreated_under_live_target_streams(hamburg, tmp_path, monkeypatch):
    host = external_host(hamburg, tmp_path, monkeypatch)
    del host.start_gateway  # Exercise the actual guard, not the startup stub.
    host.state["processes"] = {"external-servo": {"pid": 10}}
    host.alive = lambda _: True
    with pytest.raises(RuntimeError, match="cannot recreate DDS"):
        host.start_gateway()
