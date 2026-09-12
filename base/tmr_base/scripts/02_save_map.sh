#!/usr/bin/env bash
set -Eeuo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
fi

name="${1:-tmr_map_$(date +%Y%m%d_%H%M%S)}"
map_dir="${HOME}/tmr_navigation/maps"
mkdir -p "${map_dir}"
ros2 run nav2_map_server map_saver_cli -f "${map_dir}/${name}" --ros-args -p save_map_timeout:=10.0
echo "[saved] ${map_dir}/${name}.yaml"
