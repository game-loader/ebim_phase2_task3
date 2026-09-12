#!/usr/bin/env python3
"""Stop a container route when the arm client's heartbeat expires or closes."""

import fcntl
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time


def heartbeat(fd, timeout):
    readable, _, _ = select.select([fd], [], [], timeout)
    if not readable:
        return None
    return bool(os.read(fd, 4096))


def stop_group(child):
    # The shell can exit before its route descendants. Keep ownership until the
    # entire group has received bounded shutdown, even after reaping the shell.
    for sig, timeout in ((signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            child.poll()
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    child.wait(timeout=1)


def supervise(command, fd, heartbeat_timeout=3.0):
    # No motion before the controller of this session is known to be alive.
    if heartbeat(fd, heartbeat_timeout) is not True:
        raise RuntimeError("route client absent before startup")
    stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    environment = dict(os.environ, EBIM_BASE_ROUTE_SUPERVISED="1")
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True, env=environment)
    last = time.monotonic()
    interrupted = False
    try:
        while child.poll() is None:
            alive = heartbeat(fd, 0.1)
            if alive:
                last = time.monotonic()
            if stopped or alive is False or time.monotonic() - last > heartbeat_timeout:
                interrupted = True
                break
        if not interrupted:
            return child.returncode
    finally:
        stop_group(child)
    return 125


def main():
    release, shell = sys.argv[1:3]
    if release != os.environ["EBIM_BASE_RELEASE"]:
        raise RuntimeError("mission release differs from running base container")
    from host import Host
    config = json.loads(os.environ["EBIM_BASE_CONFIG"])
    root = Path(config["hosts"]["base"]["runtime_root"])
    # Prevent shutdown/restart while a route owns the base, including other clients.
    with (root / ".orchestration.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        Host(config, "base", release).health()
        return supervise(["bash", "--noprofile", "--norc", "-c", shell], sys.stdin.fileno())


if __name__ == "__main__":
    raise SystemExit(main())
