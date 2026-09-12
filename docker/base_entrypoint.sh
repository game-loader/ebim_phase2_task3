#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/ebim-base/install/setup.bash
export LD_LIBRARY_PATH="/opt/ebim-libfranka/lib:/usr/local/zed/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/app/scripts/hardware
exec /usr/bin/python3 /app/scripts/hardware/base_runtime.py "${1:-keep}"
