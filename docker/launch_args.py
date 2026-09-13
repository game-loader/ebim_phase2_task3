#!/usr/bin/env python3
"""Generate literal Docker arguments using image Python; host needs no Python."""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from franka_duo_tele_data.hardware import load_hardware


def mount(source, target, *, readonly=False, kind="bind"):
    if any(c in str(source) for c in (",", "\n", "\r", "\x00")):
        raise ValueError("Docker mount paths cannot contain commas or newlines")
    return ["--mount", f"type={kind},src={source},dst={target}" + (",readonly" if readonly else "")]


def launch_args(profile, image, ssh_dir, name, operation, activate=False):
    config = load_hardware(profile, require_calibration=False)
    if config["image"] != image:
        raise ValueError("set EBIM_IMAGE to the same image reference as hardware.yaml")
    if activate and config["deployment"] == "external":
        raise ValueError("external mode requires operator handoff; use up without --activate, then mission --execute")
    args = ["run", "--pull", "never", "--network", "none" if operation == "plan" else "host", "--ipc", "private",
            "--cap-add", "SYS_NICE", "--cap-add", "IPC_LOCK",
            "--ulimit", "rtprio=99:99", "--ulimit", "memlock=-1:-1", "--shm-size", "256m"]
    args += mount(profile, "/hardware/hardware.yaml", readonly=True)
    args += mount(config["camera"]["calibration"], "/hardware/calibration.json", readonly=True)
    args += ["--env", "EBIM_HARDWARE=/hardware/hardware.yaml",
             "--env", "EBIM_CALIBRATION=/hardware/calibration.json"]
    if operation != "plan":
        if config["deployment"] != "external":
            args += mount(ssh_dir, "/root/.ssh", readonly=True)
        args += mount(name + "-state", "/app/runtime", kind="volume")
        args += mount(name + "-outputs", "/app/outputs", kind="volume")
        if config["grippers"]["mode"] == "managed":
            for side in ("left", "right"):
                port = config["grippers"][side + "_port"]
                if ":" in port:
                    raise ValueError("device paths cannot contain ':'")
                args += ["--device", f"{port}:/dev/ebim-{side}-gripper:rw"]
    if operation == "up":
        args += ["--detach", "--name", name, "--label", "io.ebim.runtime=arm",
                 "--stop-timeout", "180", "--restart", "no"]
        operation = "serve"
    else:
        args += ["--rm"]
    args += [image, "hardware", operation]
    if activate:
        args.append("--activate")
    return args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("image")
    parser.add_argument("ssh_dir")
    parser.add_argument("name")
    parser.add_argument("operation", choices=("up", "check", "plan"))
    parser.add_argument("--activate", action="store_true")
    args = parser.parse_args()
    command = launch_args(args.profile, args.image, args.ssh_dir, args.name, args.operation, args.activate)
    sys.stdout.buffer.write(b"\0".join(arg.encode() for arg in command) + b"\0")


if __name__ == "__main__":
    main()
