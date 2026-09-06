#!/usr/bin/env bash
# Own the single head-ZED instance, its atomic JPEG exporter and HTTP bridge.
set -Eeo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
frame_file="${TMR_CYCLE_ZED_FRAME_FILE:-/tmp/tmr_zed_latest.jpg}"
vision_domain="${TMR_CYCLE_VISION_DOMAIN_ID:-1}"
pids=()

source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${vision_domain}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
# The installed ZED stack has been validated with its default RMW.  Do not
# leak the base controller's CycloneDDS override into this high-bandwidth graph.
unset RMW_IMPLEMENTATION CYCLONEDDS_URI || true
ros2 daemon stop >/dev/null 2>&1 || true

exec 9>/tmp/tmr_zed_stream.lock
if ! flock -n 9; then
  echo "[error] the managed ZED stream is already running" >&2
  exit 73
fi

cleanup() {
  trap - EXIT INT TERM
  set +e
  local pid
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# Remove only the earlier letter-runner sessions owned by this project.  An
# unrelated camera process is never killed implicitly; the launch below will
# report the device-busy error instead.
screen -S tmr_zed_letters -X quit >/dev/null 2>&1 || true
screen -S tmr_zed_export -X quit >/dev/null 2>&1 || true

rm -f "${frame_file}"
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm namespace:=head_camera publish_tf:=false \
  serial_number:=17064700 \
  ros_params_override_path:="${root}/config/zed_letters_override.yaml" &
pids+=("$!")
python3 "${root}/scripts/zed_frame_export.py" --output "${frame_file}" &
pids+=("$!")
python3 -m http.server 18082 --bind 0.0.0.0 --directory "$(dirname "${frame_file}")" &
pids+=("$!")

ready=0
for _ in {1..40}; do
  if [[ -s "${frame_file}" ]]; then
    ready=1
    break
  fi
  for pid in "${pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null || {
      echo "[error] a managed ZED component exited during startup" >&2
      exit 74
    }
  done
  sleep 0.5
done
if [[ "${ready}" != 1 ]]; then
  echo "[error] no ZED RGB frame within 20 seconds" >&2
  exit 75
fi

echo "[ready] ZED serial 17064700, vision domain ${ROS_DOMAIN_ID}, HTTP :18082"
wait -n "${pids[@]}"
echo "[error] a managed ZED component stopped" >&2
exit 76
