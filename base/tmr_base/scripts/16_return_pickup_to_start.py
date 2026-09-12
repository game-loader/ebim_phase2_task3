#!/usr/bin/env python3
"""Return from the pickup-table stop to the marked mission start.

This is the exact inverse of the fixed outbound prefix/final crossing:
reverse 1.20 m, rotate CCW 90 degrees, then reverse 0.85 m.  All stages use
one odometry-closed-loop controller and one mission lease.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent


def load_motion_module():
    path = SCRIPT_DIR / "13_post_grasp_route.py"
    spec = importlib.util.spec_from_file_location("tmr_return_motion", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import motion controller: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_state(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start", action="store_true")
    parser.add_argument("--door-reverse-m", type=float, default=1.20)
    parser.add_argument("--turn-ccw-deg", type=float, default=90.0)
    parser.add_argument("--final-reverse-m", type=float, default=0.85)
    parser.add_argument("--linear-speed-mps", type=float, default=0.08)
    parser.add_argument("--angular-speed-rps", type=float, default=0.18)
    parser.add_argument("--state-file", type=Path, default=Path("/tmp/tmr_return_pickup_to_start.json"))
    args = parser.parse_args()

    stages = [
        ("REVERSE_THROUGH_DOOR", "translate", -args.door_reverse_m),
        ("TURN_CCW_90", "rotate", math.radians(args.turn_ccw_deg)),
        ("REVERSE_TO_MARKED_START", "translate", -args.final_reverse_m),
    ]
    if not args.execute:
        print(json.dumps({"status": "dry_run", "stages": stages}, indent=2))
        return 0
    if args.state_file.exists() and not args.fresh_start:
        raise RuntimeError(f"state file exists; refuse accidental replay: {args.state_file}")

    motion = load_motion_module()
    import rclpy

    state = {"status": "running", "next_stage": 0, "reports": []}
    write_state(args.state_file, state)
    rclpy.init()
    node = None
    try:
        node = motion.RouteController(args.linear_speed_mps, args.angular_speed_rps)
        node.wait_ready()
        for index, (name, kind, value) in enumerate(stages):
            state["active_stage"] = name
            write_state(args.state_file, state)
            print(json.dumps({"event": "stage_start", "name": name}), flush=True)
            if kind == "translate":
                report = node.translate(value, 0.0, max(18.0, abs(value) / args.linear_speed_mps + 15.0))
            else:
                report = node.rotate_ccw(value, max(24.0, abs(value) / args.angular_speed_rps + 15.0))
            state["reports"].append({"stage": name, **report})
            state["next_stage"] = index + 1
            state.pop("active_stage", None)
            write_state(args.state_file, state)
        node.stop(30)
        state.update({"status": "complete", "phase": "AT_MARKED_START", "zero_command_latched": True})
        write_state(args.state_file, state)
        print(json.dumps(state, indent=2), flush=True)
        return 0
    except BaseException as exc:
        if node is not None:
            node.stop(30)
        state.update({"status": "failed", "error": repr(exc), "zero_command_latched": True})
        write_state(args.state_file, state)
        print(json.dumps(state, indent=2), flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
