#!/usr/bin/env bash
# Switch between exclusive autonomous mission control and Xbox teleoperation.
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
fi

mode="${1:-}"
teleop_config="${HOME}/ros2_ws/install/franka_bringup/share/franka_bringup/config/xbox.config.yaml"

publish_lease() {
  local value="$1"
  timeout 6 ros2 topic pub --once /tmr_cycle/mission_active \
    std_msgs/msg/Bool "{data: ${value}}" >/dev/null
}

process_environment_matches() {
  local pid="$1"
  local environ="/proc/${pid}/environ"
  [[ -r "${environ}" ]] || return 1
  tr '\0' '\n' <"${environ}" | grep -q '^RMW_IMPLEMENTATION=rmw_cyclonedds_cpp$' && \
    tr '\0' '\n' <"${environ}" | grep -q "^ROS_DOMAIN_ID=${ROS_DOMAIN_ID}$" && \
    tr '\0' '\n' <"${environ}" | grep -q "^ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}$"
}

ensure_cmd_adapter() {
  local pid=''
  local -a pids=()
  mapfile -t pids < <(pgrep -f '^python3 .*scripts/cmd_vel_adapter.py([[:space:]]|$)' || true)
  if [[ "${#pids[@]}" == 1 ]]; then
    pid="${pids[0]}"
    if process_environment_matches "${pid}"; then
      return
    fi
  fi
  if [[ "${#pids[@]}" -gt 0 ]]; then
    screen -S tmr_cmd_adapter -X quit >/dev/null 2>&1 || true
    for pid in "${pids[@]}"; do
      kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 0.5
  fi
  screen -S tmr_cmd_adapter -X quit >/dev/null 2>&1 || true
  screen -dmS tmr_cmd_adapter /bin/bash -c \
    "source /opt/ros/humble/setup.bash && source ${HOME}/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} CYCLONEDDS_URI=${CYCLONEDDS_URI:-} && cd ${HOME}/tmr_cycle && exec python3 scripts/cmd_vel_adapter.py"
  sleep 1
}

stop_teleop_velocity_nodes() {
  screen -S tmr_teleop_adapter -X quit >/dev/null 2>&1 || true
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] && kill -TERM "${pid}" 2>/dev/null || true
  done < <(pgrep -f '^/opt/ros/humble/lib/teleop_twist_joy/teleop_node([[:space:]]|$)' || true)
}

ensure_joy_node() {
  local pid=''
  local -a pids=()
  mapfile -t pids < <(pgrep -f '^/opt/ros/humble/lib/joy/joy_node([[:space:]]|$)' || true)
  if [[ "${#pids[@]}" == 1 ]] && process_environment_matches "${pids[0]}"; then
    return
  fi
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  if [[ "${#pids[@]}" -gt 0 ]]; then
    sleep 0.5
  fi
  screen -S tmr_joy_manual -X quit >/dev/null 2>&1 || true
  screen -dmS tmr_joy_manual /bin/bash -c \
    "source /opt/ros/humble/setup.bash && source ${HOME}/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} CYCLONEDDS_URI=${CYCLONEDDS_URI:-} && exec ros2 run joy joy_node"
}

start_adapter_teleop() {
  local pid=''
  local -a pids=()
  mapfile -t pids < <(pgrep -f '^/opt/ros/humble/lib/teleop_twist_joy/teleop_node.*cmd_vel:=/tmr_cycle/cmd_vel' || true)
  if [[ "${#pids[@]}" == 1 ]] && process_environment_matches "${pids[0]}"; then
    return
  fi
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  if [[ "${#pids[@]}" -gt 0 ]]; then
    sleep 0.5
  fi
  screen -S tmr_teleop_adapter -X quit >/dev/null 2>&1 || true
  screen -dmS tmr_teleop_adapter /bin/bash -c \
    "source /opt/ros/humble/setup.bash && source ${HOME}/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} CYCLONEDDS_URI=${CYCLONEDDS_URI:-} && exec ros2 run teleop_twist_joy teleop_node --ros-args -r __node:=teleop_twist_joy_node --params-file ${teleop_config} -r cmd_vel:=/tmr_cycle/cmd_vel"
}

case "${mode}" in
  mission)
    ensure_cmd_adapter
    stop_teleop_velocity_nodes
    publish_lease true
    printf '{"status":"success","mode":"mission","teleop_velocity_enabled":false,"mission_lease":true}\n'
    ;;
  teleop)
    ensure_cmd_adapter
    ensure_joy_node
    start_adapter_teleop
    sleep 1
    publish_lease false
    if ! pgrep -f '^/opt/ros/humble/lib/teleop_twist_joy/teleop_node.*cmd_vel:=/tmr_cycle/cmd_vel' >/dev/null; then
      echo '{"status":"failed","mode":"teleop","error":"teleop velocity node did not stay alive"}'
      exit 1
    fi
    printf '{"status":"success","mode":"teleop","teleop_velocity_enabled":true,"mission_lease":false}\n'
    ;;
  *)
    echo "usage: $0 {mission|teleop}" >&2
    exit 2
    ;;
esac
