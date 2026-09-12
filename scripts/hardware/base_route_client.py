#!/usr/bin/env python3
"""Feed liveness bytes to a route watchdog through SSH and docker exec -i."""

import argparse
import shlex
import signal
import subprocess
import time


def client(command):
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        while process.poll() is None and not stopping:
            try:
                process.stdin.write(b".\n")
                process.stdin.flush()
            except BrokenPipeError:
                break
            time.sleep(0.25)
    finally:
        try:
            process.stdin.close()
        except BrokenPipeError:
            pass
    # EOF reaches the container even if the remote docker client survives.
    try:
        return process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("container")
    parser.add_argument("release")
    parser.add_argument("shell")
    args = parser.parse_args()
    remote = ["docker", "exec", "-i", args.container, "/usr/bin/python3",
              "/app/scripts/hardware/base_route_watchdog.py", args.release, args.shell]
    return client(["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                   "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3",
                   args.host, shlex.join(remote)])


if __name__ == "__main__":
    raise SystemExit(main())
