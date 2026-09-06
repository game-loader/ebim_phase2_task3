#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Readiness is owned by 19; never restart odometry during this saved-pose detour.
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
else
  unset CYCLONEDDS_URI
fi
cd "${root_dir}"
if [[ "${1:-}" == "capture-origin" ]]; then
  shift
  exec python3 scripts/20_capture_leg_roi_origin.py "$@"
fi
exec python3 scripts/20_after_return_placement.py "$@"
