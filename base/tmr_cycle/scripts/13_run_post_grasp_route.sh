#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts probe optional unset variables; enable nounset afterwards.
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
source "${HOME}/tmr_navigation/install/setup.bash"
source "${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
fi
set -u

cd "${HOME}/tmr_cycle"
exec flock -n /tmp/tmr_post_grasp_route.lock \
  python3 scripts/13_post_grasp_route.py "$@"
