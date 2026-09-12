import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest


@pytest.mark.parametrize("managed", [False, True])
def test_nested_route_group_and_targeted_stop(monkeypatch, managed):
    path = Path(__file__).resolve().parents[1] / "scripts/route_process.py"
    spec = importlib.util.spec_from_file_location("route_process", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("EBIM_BASE_ROUTE_SUPERVISED", "1" if managed else "0")
    child = module.spawn_route_child(
        [sys.executable, "-c", "import time; print('ready',flush=True); time.sleep(30)"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    try:
        assert child.stdout.readline().strip() == b"ready"
        expected_group = os.getpgrp() if managed else child.pid
        assert os.getpgid(child.pid) == expected_group
        module.signal_route_child(child, signal.SIGINT)
        assert child.wait(timeout=2) != 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
