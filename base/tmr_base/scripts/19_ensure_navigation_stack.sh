#!/usr/bin/env bash
# Idempotently ensure the base-local Humble runtime is healthy before a mission.
set -Eeo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ready_file="/tmp/tmr_navigation_stack.ready"
screen_name="tmr_navigation_stack"

source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
source "${HOME}/tmr_navigation/install/setup.bash"
source "${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
else
  unset CYCLONEDDS_URI || true
fi

ready_pid() {
  [[ -s "${ready_file}" ]] || return 1
  sed -n 's/^pid=\([0-9][0-9]*\)$/\1/p' "${ready_file}" | head -n 1
}

controller_rpc() {
  timeout 5 ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null | grep -q 'response:'
}

terminate_unresponsive_base_processes() {
  # A responsive controller can be safely reused by 03_start_navigation.sh.
  controller_rpc && return 0
  local -a stale=()
  mapfile -t stale < <(
    pgrep -f 'tmrv0_2\.launch\.py|^/opt/ros/humble/lib/controller_manager/ros2_control_node([[:space:]]|$)' || true
  )
  [[ "${#stale[@]}" == 0 ]] && return 0
  local pid
  for pid in "${stale[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  for _ in {1..60}; do
    local alive=false
    for pid in "${stale[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=true; done
    [[ "${alive}" == false ]] && return 0
    sleep 0.1
  done
  for pid in "${stale[@]}"; do kill -TERM "${pid}" 2>/dev/null || true; done
  sleep 1
}

runtime_healthy() {
  local pid domain adapters
  pid="$(ready_pid)" || return 1
  domain="$(sed -n 's/^domain=\([0-9][0-9]*\)$/\1/p' "${ready_file}" | head -n 1)"
  [[ "${domain}" == "${ROS_DOMAIN_ID}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  adapters="$(pgrep -fc '^python3 .*scripts/cmd_vel_adapter.py([[:space:]]|$)' || true)"
  [[ "${adapters}" == "1" ]] || return 1
  # The ready file can briefly outlive a child failure.  Require one current
  # odometry sample before reusing the stack so a mission never reaches its
  # first motion process with a dead base graph.
  timeout 3 ros2 topic echo --once --no-daemon \
    /swerve_drive_controller/odom >/dev/null 2>&1 || return 1
}

if runtime_healthy; then
  printf '{"status":"success","base_runtime":"reused","domain":%s}\n' "${ROS_DOMAIN_ID}"
  exit 0
fi

# Restart only a stack carrying this project's ready-file PID.  Its trap owns
# and shuts down its child process groups, avoiding partial duplicate graphs.
old_pid="$(ready_pid || true)"
if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
  kill -TERM "${old_pid}" 2>/dev/null || true
  for _ in {1..160}; do
    kill -0 "${old_pid}" 2>/dev/null || break
    sleep 0.1
  done
fi
screen -S "${screen_name}" -X quit >/dev/null 2>&1 || true
rm -f "${ready_file}"

# A command adapter left by an interrupted manual control-mode session must
# not coexist with the adapter managed by 03_start_navigation.sh.
while read -r pid; do
  [[ -n "${pid}" ]] && kill -TERM "${pid}" 2>/dev/null || true
done < <(pgrep -f '^python3 .*scripts/cmd_vel_adapter.py([[:space:]]|$)' || true)

terminate_unresponsive_base_processes

screen -dmS "${screen_name}" bash -lc \
  "exec ${root_dir}/scripts/03_start_navigation.sh > /tmp/tmr_navigation_bootstrap.log 2>&1"

for attempt in {1..100}; do
  if runtime_healthy; then
    printf '{"status":"success","base_runtime":"started","domain":%s}\n' "${ROS_DOMAIN_ID}"
    exit 0
  fi
  if (( attempt >= 3 )) && ! screen -ls 2>/dev/null | grep -q "[.]${screen_name}[[:space:]]"; then
    echo '{"status":"failed","error":"base runtime exited during startup"}' >&2
    tail -n 80 /tmp/tmr_navigation_bootstrap.log >&2 2>/dev/null || true
    exit 76
  fi
  sleep 1
done

echo '{"status":"failed","error":"base runtime did not become healthy"}' >&2
tail -n 80 /tmp/tmr_navigation_bootstrap.log >&2 2>/dev/null || true
exit 76
