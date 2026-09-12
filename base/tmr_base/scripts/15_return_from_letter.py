#!/usr/bin/env python3
"""Return from a letter placement to the pickup-table side.

Order is fixed: translate robot-left by the measured outbound right distance
plus the configured 0.20 m post-placement margin,
rotate clockwise 180 degrees, then reuse the proven live doorway sequence with
its initial forward/turn stages explicitly skipped:
door centreline -> 0.50 m before door -> forward 1.20 m -> zero-speed hold.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

from route_process import spawn_route_child, signal_route_child


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR.parent / "config" / "start_to_pickup.yaml"


def atomic_write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


@contextmanager
def mission_lock(path: Path):
    if os.name != "posix":
        yield
        return
    import fcntl

    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError("another base mission already owns the motion lock") from exc
    try:
        yield
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def load_motion_module():
    path = SCRIPT_DIR / "13_post_grasp_route.py"
    spec = importlib.util.spec_from_file_location("tmr_post_grasp_motion", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import motion controller: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def extract_last_json(text: str) -> dict | None:
    decoder = json.JSONDecoder()
    best = None
    mission_report = None
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            best = value
            if "status" in value and "final_state" in value:
                mission_report = value
    return mission_report if mission_report is not None else best


def validate_door_config(path: Path) -> None:
    import yaml

    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    mission = value.get("mission", {}) if isinstance(value, dict) else {}
    before = float(mission.get("before_door_m", math.nan))
    forward = float(mission.get("forward_from_before_door_m", math.nan))
    if abs(before - 0.50) > 1e-9 or abs(forward - 1.20) > 1e-9:
        raise RuntimeError("door config must preserve before_door_m=0.50 and forward=1.20")


def run_door_child(args: argparse.Namespace) -> tuple[int, str]:
    environment = os.environ.copy()
    environment["TMR_CYCLE_SKIP_INITIAL_FORWARD"] = "1"
    environment["TMR_CYCLE_SKIP_TURN"] = "1"
    command = [
        sys.executable,
        str(SCRIPT_DIR / "07_start_to_pickup.py"),
        "--config",
        str(args.door_config),
        "--execute",
    ]
    if args.disable_collision_guard:
        command.append("--disable-collision-guard")
    process = spawn_route_child(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
        env=environment,
    )
    output = []
    started = time.monotonic()
    lines: queue.Queue[str | None] = queue.Queue()

    def read_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=read_output, daemon=True).start()
    reader_done = False
    try:
        while not reader_done or process.poll() is None:
            if time.monotonic() - started > args.door_timeout_s:
                signal_route_child(process, signal.SIGINT)
                raise TimeoutError("door return child timed out")
            try:
                line = lines.get(timeout=0.10)
            except queue.Empty:
                continue
            if line is None:
                reader_done = True
                continue
            output.append(line)
            print("[door-return] " + line, end="", flush=True)
        return process.wait(timeout=2.0), "".join(output)
    except BaseException:
        if process.poll() is None:
            signal_route_child(process, signal.SIGINT)
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                signal_route_child(process, signal.SIGTERM)
                process.wait(timeout=2.0)
        raise


def run(args: argparse.Namespace) -> dict:
    validate_door_config(args.door_config)
    previous = None
    if args.state_file.exists():
        previous = json.loads(args.state_file.read_text(encoding="utf-8"))
        if not args.fresh_start and not args.resume_door:
            raise RuntimeError("return state exists; use --resume-door or --fresh-start explicitly")
    total_left_m = args.left_m + args.extra_left_m
    state = {
        "status": "running",
        "phase": "CREATED",
        "requested_left_m": args.left_m,
        "extra_left_m": args.extra_left_m,
        "total_left_m": total_left_m,
        "turn_cw_deg": args.turn_cw_deg,
        "door_before_m": 0.50,
        "door_forward_m": 1.20,
        "reports": [],
    }
    if args.resume_door:
        resumable_phases = {"BASE_REALIGNED", "DOOR_RETURN_FAILED"}
        if not isinstance(previous, dict) or previous.get("phase") not in resumable_phases:
            raise RuntimeError(
                "--resume-door requires a BASE_REALIGNED or DOOR_RETURN_FAILED checkpoint"
            )
        completed = {item.get("stage") for item in previous.get("reports", [])}
        if not {"LEFT_BY_MEASURED_OUTBOUND", "TURN_CW_180"}.issubset(completed):
            raise RuntimeError("--resume-door checkpoint does not prove base realignment")
        state = previous
        state.pop("door_returncode", None)
        state.pop("door_report", None)
    atomic_write(args.state_file, state)

    if not args.resume_door:
        motion = load_motion_module()
        import rclpy

        rclpy.init()
        node = None
        try:
            node = motion.RouteController(args.linear_speed_mps, args.angular_speed_rps)
            node.wait_ready()
            state["phase"] = "LEFT_RETURN_RUNNING"
            atomic_write(args.state_file, state)
            if total_left_m > 0.018:
                timeout = max(18.0, total_left_m / args.linear_speed_mps + 15.0)
                left_report = node.translate(0.0, total_left_m, timeout)
            else:
                left_report = {"skipped": True, "reason": "target was already centered"}
            state["reports"].append({"stage": "LEFT_BY_MEASURED_OUTBOUND", **left_report})
            state["phase"] = "TURN_CW_180_RUNNING"
            atomic_write(args.state_file, state)
            turn_timeout = max(
                24.0, math.radians(args.turn_cw_deg) / args.angular_speed_rps + 15.0
            )
            turn_report = node.rotate_ccw(-math.radians(args.turn_cw_deg), turn_timeout)
            state["reports"].append({"stage": "TURN_CW_180", **turn_report})
            node.stop(30)
            state["phase"] = "BASE_REALIGNED"
            state["zero_command_latched"] = True
            atomic_write(args.state_file, state)
        finally:
            if node is not None:
                node.stop(30)
                node.destroy_node()
            rclpy.shutdown()

    state["phase"] = "DOOR_RETURN_RUNNING"
    atomic_write(args.state_file, state)
    returncode, output = run_door_child(args)
    report = extract_last_json(output)
    stable = bool(
        returncode == 0
        and isinstance(report, dict)
        and report.get("status") == "success"
        and report.get("final_state") == "FINAL_STOP"
        and report.get("zero_command_latched") is True
        and report.get("control_lease_held") is True
    )
    if not stable:
        state["phase"] = "DOOR_RETURN_FAILED"
        state["door_returncode"] = returncode
        state["door_report"] = report
        atomic_write(args.state_file, state)
        raise RuntimeError("door centreline/0.50/1.20 sequence did not prove FINAL_STOP")
    state["phase"] = "COMPLETE"
    state["status"] = "complete"
    state["door_report"] = report
    state["zero_command_latched"] = True
    atomic_write(args.state_file, state)
    return state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start", action="store_true")
    parser.add_argument("--resume-door", action="store_true")
    parser.add_argument("--left-m", type=float, required=True)
    parser.add_argument("--extra-left-m", type=float, default=0.20)
    parser.add_argument("--turn-cw-deg", type=float, default=180.0)
    parser.add_argument("--linear-speed-mps", type=float, default=0.065)
    parser.add_argument("--angular-speed-rps", type=float, default=0.18)
    parser.add_argument("--door-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--door-timeout-s", type=float, default=120.0)
    parser.add_argument("--disable-collision-guard", action="store_true")
    parser.add_argument(
        "--with-placement-detour", action="store_true",
        help="after COMPLETE: left 0.70, align base front with live far leg, CW90, stop",
    )
    parser.add_argument(
        "--placement-state-file", type=Path,
        help="script 20 checkpoint; default: <return-state-stem>_placement.json",
    )
    parser.add_argument(
        "--placement-roi-origin-file", type=Path,
        default=SCRIPT_DIR.parent / "state/leg_roi_origin.json",
        help="green START pose captured before departure in this odometry session",
    )
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path("~/tmr_base/state/return_from_letter.json").expanduser(),
    )
    args = parser.parse_args()
    if not 0.0 <= args.left_m <= 2.40:
        parser.error("left-m must be the measured search distance in [0.00, 2.40]")
    if not 0.0 <= args.extra_left_m <= 0.30:
        parser.error("extra-left-m must be in [0.00, 0.30]")
    if not 175.0 <= args.turn_cw_deg <= 185.0:
        parser.error("turn must remain within the bounded 180-degree return range")
    if args.fresh_start and args.resume_door:
        parser.error("--fresh-start and --resume-door are mutually exclusive")
    if args.placement_state_file is not None and not args.with_placement_detour:
        parser.error("--placement-state-file requires --with-placement-detour")
    return args


def run_placement_detour(args: argparse.Namespace) -> int:
    """Called after releasing script 15's lock; child 20 acquires the same lock."""
    state_file = args.placement_state_file or args.state_file.with_name(
        args.state_file.stem + "_placement.json"
    )
    command = [
        sys.executable, str(SCRIPT_DIR / "20_after_return_placement.py"), "outbound",
        "--execute", "--after15-state", str(args.state_file.resolve()),
        "--state-file", str(state_file.resolve()),
        "--roi-origin-file", str(args.placement_roi_origin_file.resolve()),
    ]
    child = spawn_route_child(command)
    try:
        return child.wait(timeout=210)
    except BaseException:
        if child.poll() is None:
            signal_route_child(child, signal.SIGINT)
            try:
                child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                signal_route_child(child, signal.SIGTERM)
                child.wait(timeout=3)
        raise


def main() -> int:
    args = parse_args()
    if not args.execute:
        print(json.dumps({
            "status": "dry_run",
            "motion_enabled": False,
            "sequence": [
                f"left {args.left_m + args.extra_left_m:.3f} m "
                f"(measured outbound + {args.extra_left_m:.3f} m placement margin)",
                f"CW {args.turn_cw_deg:.1f} deg",
                "live door midpoint alignment",
                "stop 0.50 m before door",
                "forward 1.20 m",
                "latched zero-speed hold",
            ] + ([
                "NEW script 20: left 0.70 m, base front aligns with max-x live leg in green START ROI, CW90",
                "STOP and save WAITING_FOR_PLACEMENT (no arm policy or automatic return)",
            ] if args.with_placement_detour else []),
        }, indent=2))
        return 0
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("operator signal")

    signal.signal(signal.SIGTERM, interrupt)
    try:
        if args.with_placement_detour:
            placement_path = args.placement_state_file or args.state_file.with_name(
                args.state_file.stem + "_placement.json"
            )
            if placement_path.exists():
                raise RuntimeError("placement checkpoint exists; refuse replay before starting script 15")
            from far_leg_target import driver_session, load_roi_origin
            load_roi_origin(args.placement_roi_origin_file, driver_session(), "")
        with mission_lock(Path("/tmp/tmr_letter_return.lock")):
            print(json.dumps(run(args), indent=2), flush=True)
        if args.with_placement_detour:
            return run_placement_detour(args)
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
