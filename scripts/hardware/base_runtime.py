#!/usr/bin/env python3
"""Container-side base lifecycle. The keeper never starts hardware on its own."""

import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import time

from host import Host


def sdk_calibration(config):
    if config["camera"]["mode"] != "managed":
        return
    serial = config["camera"]["serial"]
    target = Path(f"/usr/local/zed/settings/SN{serial}.conf")
    source = Path("/run/ebim-zed-calibration.conf")
    if not source.is_file():
        source = Path(f"/app/configs/zed_sdk/SN{serial}.conf")
    if source.is_file():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    if not target.is_file():
        raise RuntimeError(f"missing ZED factory calibration SN{serial}.conf; supply it via camera.sdk_settings_dir")


def operation(config, release, action):
    host = Host(config, "base", release)
    if action in ("check", "status", "health"):
        if action == "check":
            sdk_calibration(config)
        return getattr(host, action)()
    host.root.mkdir(parents=True, exist_ok=True)
    with (host.root / ".orchestration.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        host = Host(config, "base", release)
        if action == "down":
            return host.down()
        if not (host.release / ".complete").is_file():
            raise RuntimeError("base release is not deployed")
        if action == "up":
            sdk_calibration(config)
            return host.up(False)
        if action == "ready":
            return host.probe("base-ready")
        raise ValueError("unknown base operation: " + action)


def main():
    config = json.loads(os.environ["EBIM_BASE_CONFIG"])
    release = os.environ["EBIM_BASE_RELEASE"]
    action = sys.argv[1]
    if action != "keep":
        return operation(config, release, action)
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while True:
        if stopping:
            try:
                operation(config, release, "down")
            except Exception as exc:
                print(f"base shutdown refused; runtime retained: {exc}", flush=True)
                stopping = False
            else:
                return
        time.sleep(0.5)


if __name__ == "__main__":
    main()
