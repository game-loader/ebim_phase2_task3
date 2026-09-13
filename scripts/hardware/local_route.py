#!/usr/bin/env python3
"""Run one bundled base leg in an already-started Hamburg task container."""

import argparse
import fcntl
import json
import os
import re
import uuid
from pathlib import Path

from external import ExternalHost

from franka_duo_tele_data.hardware import Orchestrator, load_hardware
from franka_duo_tele_data.table_mission import (
    MissionConfig,
    build_base_argv,
    build_placement_route_argv,
    build_post_grasp_argv,
    build_return_from_letter_argv,
)

BUILDERS = {"outbound": build_base_argv, "to-letter": build_post_grasp_argv,
            "return": build_return_from_letter_argv, "placement": build_placement_route_argv}


def route_command(host, route, run_id):
    config = MissionConfig(
        base_host="", base_root=str(host.release / "base/tmr_base"),
        arm_root="/app", arm_env=host.host["ros_setup"], dataset="/app/assets/contract",
        speed=host.config["runtime"]["playback_speed"], init_timeout_s=120,
        outbound_timeout_s=600, stage_timeout_s=600, transition_settle_s=0,
        base_local=True, base_env=str(host.release / "base_env.sh"))
    return BUILDERS[route](config, run_id)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route", choices=BUILDERS)
    parser.add_argument("--hardware", type=Path, default=Path(os.environ.get("EBIM_HARDWARE", "/app/hardware.hamburg.yaml")))
    parser.add_argument("--run-id", default=None, help="reuse the return run ID for placement")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    if args.route == "placement" and not args.run_id:
        parser.error("placement requires --run-id from the completed return leg")
    run_id = args.run_id or uuid.uuid4().hex[:8]
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", run_id):
        parser.error("run-id must contain 1-64 letters, digits, underscores or hyphens")
    config = load_hardware(args.hardware)
    if config["deployment"] != "external":
        parser.error("local routes require deployment: external")
    host = ExternalHost(config, "arm", Orchestrator(config).release)
    command = route_command(host, args.route, run_id)
    print(json.dumps({"route": args.route, "run_id": run_id,
                      "motion_enabled": args.execute, "command": command}), flush=True)
    if not args.execute:
        return 0
    # Share the mission/lifecycle lock so a standalone route excludes shutdown
    # and full missions. The route itself retains its heartbeat and base lock.
    with (host.root / ".orchestration.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        host.health()
        host.probe("route-ready")
        # Replace this process with the heartbeat client so its exit closes the
        # watchdog pipe. Keep the lifecycle lock across both exec calls.
        os.set_inheritable(lock.fileno(), True)
        command = host.shell(command)
        os.execvpe(command[0], command, host.environment())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
