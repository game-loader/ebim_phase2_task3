#!/usr/bin/env python3
"""Run the local orchestrator without installing the policy's perception stack."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from franka_duo_tele_data.hardware import main

if __name__ == "__main__":
    raise SystemExit(main())
