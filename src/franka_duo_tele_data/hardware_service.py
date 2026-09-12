"""Keep the arm container alive independently of startup and mission success."""

import json
from pathlib import Path
import signal
import time


def runtime_instance():
    return str(Path("/proc/1/ns/pid").stat().st_ino)


def serve(orchestrator, activate=False):
    root = Path(orchestrator.config["hosts"]["arm"]["runtime_root"])
    root.mkdir(parents=True, exist_ok=True)
    status = root / "service.json"
    instance = runtime_instance()
    stopping = False

    def record(state, error=None):
        temporary = status.with_suffix(".tmp")
        temporary.write_text(json.dumps({"state": state, "release": orchestrator.release,
                                         "instance": instance, "error": error, "updated": time.time()}))
        temporary.replace(status)

    def stop_requested(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop_requested)
    signal.signal(signal.SIGINT, stop_requested)
    record("starting")
    try:
        orchestrator.run("up", activate=activate)
        record("ready")
    except Exception as exc:
        record("failed", str(exc))
        print(f"Startup failed; container and surviving target streams retained: {exc}", flush=True)
    # Never restart a failed driver, reset a robot fault, or kill target streams
    # merely because startup/mission/monitoring fails.
    while True:
        if stopping:
            record("stopping")
            try:
                orchestrator.run("down")
            except Exception as exc:
                record("stop-refused", str(exc))
                print(f"Shutdown refused; inspect runtime before stopping Docker: {exc}", flush=True)
                stopping = False
            else:
                record("stopped")
                return 0
        if json.loads(status.read_text())["state"] == "ready":
            try:
                # Process/fault checks only: do not take the mission's ROS lock.
                for role in ("arm", "base"):
                    orchestrator.host(role, "health")
            except Exception as exc:
                record("degraded", str(exc))
                print(f"Runtime degraded; streams retained: {exc}", flush=True)
        time.sleep(5)
