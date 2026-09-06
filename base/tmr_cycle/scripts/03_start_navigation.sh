#!/usr/bin/env bash
# Start the complete base-local stack. Nothing moves unless --run-mission is
# supplied; even then the mission remains behind the exclusive zero-latching
# cmd_vel adapter.
set -Eeo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_dir=""
ready_file="/tmp/tmr_navigation_stack.ready"
pids=()
last_pid=""
base_pid=""
run_mission=false

readonly base_ready_timeout_s="${TMR_CYCLE_BASE_READY_TIMEOUT_S:-15}"
readonly builtin_spawner_grace_s="${TMR_CYCLE_SPAWNER_GRACE_S:-35}"
readonly local_domain_id="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
readonly localhost_only="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"

configure_environment() {
  source /opt/ros/humble/setup.bash
  source "${HOME}/ros2_ws/install/setup.bash"
  source "${HOME}/tmr_navigation/install/setup.bash"
  # The field workspace was once built as a catkin-marked isolated prefix, so
  # the top-level setup can omit its only ROS 2 package.
  source "${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash"

  # Keep the Humble base-local control graph away from the Jazzy hosts on
  # domain 0. Every process started here inherits these values.
  export ROS_DOMAIN_ID="${local_domain_id}"
  export ROS_LOCALHOST_ONLY="${localhost_only}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  if [[ "${TMR_CYCLE_USE_HOST_CYCLONEDDS:-1}" == "1" && -f "${HOME}/cyclonedds.xml" ]]; then
    export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
  else
    unset CYCLONEDDS_URI || true
  fi
  # A daemon created in another domain/RMW can return stale graph entries.
  ros2 daemon stop >/dev/null 2>&1 || true
  set -u
}

start_process() {
  local name="$1"
  shift
  # One process group per managed component lets cleanup stop launch children,
  # not only the top-level ros2/python process.
  setsid "$@" >"${log_dir}/${name}.log" 2>&1 &
  last_pid="$!"
  pids+=("${last_pid}")
  echo "[start] ${name}, PID ${last_pid}"
}

process_alive() {
  kill -0 "$1" 2>/dev/null
}

assert_process_alive() {
  local name="$1"
  local pid="$2"
  local settle_s="${3:-1}"
  sleep "${settle_s}"
  if ! process_alive "${pid}"; then
    echo "[error] ${name} exited during startup; see ${log_dir}/${name}.log" >&2
    return 1
  fi
}

local_base_process_present() {
  pgrep -f 'tmrv0_2\.launch\.py|/controller_manager/ros2_control_node|/ros2_control_node' >/dev/null 2>&1
}

existing_managed_stack_healthy() {
  local pid domain adapters
  [[ -s "${ready_file}" ]] || return 1
  pid="$(sed -n 's/^pid=\([0-9][0-9]*\)$/\1/p' "${ready_file}" | head -n 1)"
  domain="$(sed -n 's/^domain=\([0-9][0-9]*\)$/\1/p' "${ready_file}" | head -n 1)"
  [[ -n "${pid}" && "${domain}" == "${ROS_DOMAIN_ID}" ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  adapters="$(pgrep -fc '^python3 .*scripts/cmd_vel_adapter.py([[:space:]]|$)' || true)"
  [[ "${adapters}" == "1" ]] || return 1
  wait_for_topic_once /swerve_drive_controller/odom 4 || return 1
  wait_for_topic_once /lidar_front/scan 4 || return 1
  wait_for_topic_once /lidar_rear/scan 4 || return 1
}

# Prefer a completed RPC.  The Humble ros2control CLI can nevertheless spend
# longer than its outer timeout in graph discovery on this host; the launch
# log is a valid same-run fallback because log_dir is unique for every start.
base_log_reports_active_controller() {
  [[ -f "${log_dir}/base.log" ]] \
    && grep -Eq 'Configured and activated.*swerve_drive_controller' "${log_dir}/base.log"
}

controller_manager_rpc() {
  timeout 6 ros2 service call /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers '{}' 2>/dev/null \
    | grep -q 'response:' \
    || base_log_reports_active_controller
}

controller_active() {
  timeout 6 ros2 control list_controllers --spin-time 0.2 \
    -c /controller_manager 2>/dev/null \
    | grep -Eq '^swerve_drive_controller[[:space:]].*[[:space:]]active([[:space:]]|$)' \
    || base_log_reports_active_controller
}

check_tmr_state() {
  command -v curl >/dev/null 2>&1 || return 0
  local state
  if ! state="$(curl -kfsS --max-time 3 https://172.16.16.10/spine/api/state 2>/dev/null)"; then
    echo "[warn] TMR state endpoint did not answer; the bounded FCI handshake will decide readiness"
    return 0
  fi
  printf '%s\n' "${state}" >"${log_dir}/tmr_state_before_start.json"
  if [[ "${state}" != *SwitchedOn* ]]; then
    echo "[error] TMR reports a non-SwitchedOn state; no controller was started" >&2
    echo "[error] explicit fault-reset/switch-on recovery is required" >&2
    return 1
  fi
}

wait_for_controller_manager() {
  local owned_base_pid="${1:-}"
  local deadline=$((SECONDS + base_ready_timeout_s))
  while (( SECONDS < deadline )); do
    controller_manager_rpc && return 0
    if [[ -n "${owned_base_pid}" ]] && ! process_alive "${owned_base_pid}"; then
      echo "[error] base launch exited before controller_manager answered an RPC" >&2
      return 1
    fi
    sleep 0.5
  done

  if [[ -f "${log_dir}/base.log" ]] \
      && grep -q 'Connecting to robot at.*172\.16\.16\.10' "${log_dir}/base.log" \
      && ! grep -q 'Successfully connected to robot' "${log_dir}/base.log"; then
    echo "[error] FCI/TMR TCP connected but the protocol handshake did not answer within ${base_ready_timeout_s}s" >&2
    echo "[error] do not retry blindly; inspect TMR state and perform an explicit fault-reset/switch-on if authorized" >&2
  else
    echo "[error] controller_manager did not answer an RPC within ${base_ready_timeout_s}s" >&2
  fi
  return 1
}

wait_for_controller_active() {
  local wait_s="$1"
  local deadline=$((SECONDS + wait_s))
  while (( SECONDS < deadline )); do
    controller_active && return 0
    [[ -z "${base_pid}" ]] || process_alive "${base_pid}" || return 1
    # The launch parent can remain alive after its FCI child has already
    # reported a protocol failure.  Exit this wait immediately so the bounded
    # whole-stack retry can recover instead of wasting the spawner grace time.
    base_log_has_retryable_fci_error && return 1
    sleep 1
  done
  return 1
}

ensure_swerve_active() {
  if controller_active; then
    echo "[ready] swerve_drive_controller is active"
    return 0
  fi

  # The launch file already started its own spawner. Give that one its full
  # window before starting a retry, so two spawners never race each other.
  if wait_for_controller_active "${builtin_spawner_grace_s}"; then
    echo "[ready] launch spawner activated swerve_drive_controller"
    return 0
  fi

  if [[ -n "${base_pid}" ]] && ! process_alive "${base_pid}"; then
    echo "[error] base launch exited before its controller became active" >&2
    return 1
  fi
  if grep -Eq 'libfranka: incorrect object size|communication_constraints_violation' \
      "${log_dir}/base.log" 2>/dev/null; then
    echo "[error] transient FCI protocol failure interrupted this base launch" >&2
    return 1
  fi

  local share_dir controller_params
  share_dir="$(ros2 pkg prefix --share franka_bringup)"
  controller_params="${share_dir}/config/controllers.yaml"
  if [[ ! -f "${controller_params}" ]]; then
    echo "[error] controller parameters not found: ${controller_params}" >&2
    return 1
  fi

  echo "[retry] built-in spawner window elapsed; making one configured retry"
  if ! timeout 45 ros2 run controller_manager spawner swerve_drive_controller \
      --controller-manager /controller_manager \
      --controller-manager-timeout 20 \
      --service-call-timeout 10 \
      --ros-args --params-file "${controller_params}" \
      >"${log_dir}/swerve_spawner_retry.log" 2>&1; then
    echo "[error] configured swerve spawner retry failed; see ${log_dir}/swerve_spawner_retry.log" >&2
    return 1
  fi
  wait_for_controller_active 10 || {
    echo "[error] swerve_drive_controller did not become active after the configured retry" >&2
    return 1
  }
}

stop_process_group() {
  local pid="$1"
  process_alive "${pid}" || return 0
  kill -INT -- "-${pid}" 2>/dev/null || true
  local deadline=$((SECONDS + 7))
  while (( SECONDS < deadline )); do
    process_alive "${pid}" || return 0
    sleep 0.1
  done
  kill -TERM -- "-${pid}" 2>/dev/null || true
  deadline=$((SECONDS + 3))
  while (( SECONDS < deadline )); do
    process_alive "${pid}" || return 0
    sleep 0.1
  done
  kill -KILL -- "-${pid}" 2>/dev/null || true
}

base_log_has_retryable_fci_error() {
  grep -Eq \
    'libfranka: incorrect object size|communication_constraints_violation|Franka: NetworkException' \
    "${log_dir}/base.log" 2>/dev/null
}

start_base_with_fci_retry() {
  local attempt pid
  for attempt in 1 2 3; do
    echo "[base] launch attempt ${attempt}/3"
    start_process base ros2 launch franka_bringup tmrv0_2.launch.py controller_name:=swerve_drive_controller
    base_pid="${last_pid}"
    if wait_for_controller_manager "${base_pid}" \
        && ensure_swerve_active \
        && wait_for_topic_once /swerve_drive_controller/odom 8; then
      return 0
    fi
    if (( attempt == 3 )) || ! base_log_has_retryable_fci_error; then
      echo "[error] base startup failed without a retryable FCI signature" >&2
      return 1
    fi
    echo "[retry] transient FCI protocol failure; restarting the complete base launch"
    stop_process_group "${base_pid}"
    wait "${base_pid}" 2>/dev/null || true
    # Do not leave the reaped PID in the supervisor list.  `wait -n` treats a
    # PID from a failed retry as an already-finished child and immediately
    # tears down the healthy replacement stack.
    local -a live_pids=()
    for pid in "${pids[@]}"; do
      process_alive "${pid}" && live_pids+=("${pid}")
    done
    pids=("${live_pids[@]}")
    sleep 1
  done
  return 1
}

wait_for_topic_once() {
  local topic="$1"
  local wait_s="$2"
  local deadline=$((SECONDS + wait_s))
  # Immediately after a controller/driver starts, ros2 topic echo can exit
  # before DDS discovery knows the topic type.  Retry short reads inside the
  # overall bound instead of treating that first discovery miss as failure.
  while (( SECONDS < deadline )); do
    timeout 2 ros2 topic echo --once --no-daemon "${topic}" >/dev/null 2>&1 && return 0
    sleep 0.2
  done
  return 1
}

signal_alive_children() {
  local signal="$1"
  local pid
  for pid in "${pids[@]}"; do
    process_alive "${pid}" && kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  done
}

wait_for_children_exit() {
  local wait_s="$1"
  local deadline=$((SECONDS + wait_s))
  local pid any_alive
  while (( SECONDS < deadline )); do
    any_alive=false
    for pid in "${pids[@]}"; do
      process_alive "${pid}" && any_alive=true
    done
    [[ "${any_alive}" == false ]] && return 0
    sleep 0.2
  done
  return 1
}

cleanup() {
  trap - EXIT INT TERM
  set +e
  rm -f "${ready_file}"
  signal_alive_children INT
  wait_for_children_exit 6 || {
    signal_alive_children TERM
    wait_for_children_exit 3 || {
      signal_alive_children KILL
      wait_for_children_exit 2 || true
    }
  }
  local pid
  for pid in "${pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
  echo "[stop] live SLAM/base stack stopped"
}

main() {
  case "${1:-}" in
    "") ;;
    --run-mission) run_mission=true ;;
    *) echo "usage: $0 [--run-mission]" >&2; return 2 ;;
  esac

  configure_environment
  exec 9>/tmp/tmr_navigation_stack.lock
  if ! flock -n 9; then
    if existing_managed_stack_healthy; then
      echo "[reuse] managed base/navigation stack is already healthy"
      return 0
    fi
    echo "[error] the managed base/navigation stack is already running" >&2
    return 72
  fi
  rm -f "${ready_file}"
  log_dir="${HOME}/tmr_cycle/logs/live_slam_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${log_dir}"
  {
    echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
    echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
  } >"${log_dir}/environment.log"

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  check_tmr_state

  if controller_manager_rpc; then
    echo "[reuse] controller_manager answered RPC; not launching a duplicate"
    wait_for_controller_manager ""
    ensure_swerve_active
    if ! wait_for_topic_once /swerve_drive_controller/odom 8; then
      echo "[error] controller is active but no fresh odometry arrived" >&2
      return 1
    fi
  else
    if local_base_process_present; then
      echo "[error] a local base/control process exists but controller_manager RPC is unresponsive" >&2
      echo "[error] stop the stale process explicitly before retrying" >&2
      return 1
    fi
    start_base_with_fci_retry
  fi

  start_process lidars ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py start_cameras:=false start_lidars:=true start_rviz:=false
  assert_process_alive lidars "${last_pid}" 2
  wait_for_topic_once /lidar_front/scan 12 || { echo "[error] front LiDAR did not publish" >&2; return 1; }
  wait_for_topic_once /lidar_rear/scan 12 || { echo "[error] rear LiDAR did not publish" >&2; return 1; }

  start_process adapter ros2 launch tmr_local_navigation navigation_adapter.launch.py
  assert_process_alive adapter "${last_pid}" 2
  start_process slam ros2 launch tmr_local_navigation dual_lidar_slam.launch.py
  assert_process_alive slam "${last_pid}" 3
  # Own exactly one command adapter.  A prior standalone control-mode helper
  # can otherwise race this managed copy and create launch-time jitter.
  while read -r adapter_pid; do
    [[ -n "${adapter_pid}" ]] && kill -TERM "${adapter_pid}" 2>/dev/null || true
  done < <(pgrep -f '^python3 .*scripts/cmd_vel_adapter.py([[:space:]]|$)' || true)
  sleep 0.3
  start_process cmd_adapter python3 "${root_dir}/scripts/cmd_vel_adapter.py"
  assert_process_alive cmd_adapter "${last_pid}" 2

  echo "[ready] isolated base, controller, odometry, dual LiDAR, SLAM and zero-latching adapter are running"
  echo "[env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  echo "[logs] ${log_dir}"
  printf 'pid=%s\ndomain=%s\nready_unix_s=%s\n' "$$" "${ROS_DOMAIN_ID}" "$(date +%s)" >"${ready_file}"

  if [[ "${run_mission}" == true ]]; then
    start_process mission python3 "${root_dir}/scripts/07_start_to_pickup.py" \
      --config "${root_dir}/config/start_to_pickup.yaml" --execute --disable-collision-guard
    echo "[mission] complete start-to-pickup route launched in the same isolated ROS environment"
  else
    echo "[note] no motion mission was started; use --run-mission for the one-command workflow"
  fi

  set +e
  wait -n "${pids[@]}"
  local child_rc="$?"
  set -e
  echo "[error] a managed process exited (status ${child_rc}); inspect ${log_dir}" >&2
  return 1
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
