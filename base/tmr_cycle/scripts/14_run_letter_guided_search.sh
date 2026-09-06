#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python=''
if [[ -x "${root}/.venv-letter/bin/python" ]] && \
  "${root}/.venv-letter/bin/python" -c 'import cv2, numpy, rclpy' 2>/dev/null; then
  python="${root}/.venv-letter/bin/python"
elif [[ -d "${root}/.letter-deps" ]]; then
  export PYTHONPATH="${root}/.letter-deps${PYTHONPATH:+:${PYTHONPATH}}"
  python=python3
else
  echo "letter vision environment is missing; run scripts/14_prepare_letter_vision.sh once" >&2
  exit 71
fi
"${python}" -c 'import cv2, numpy, rclpy' || {
  echo "letter vision environment failed its dependency preflight" >&2
  exit 72
}

control_domain="${ROS_DOMAIN_ID}"
vision_domain="${TMR_CYCLE_VISION_DOMAIN_ID:-1}"
frame_file="${TMR_CYCLE_ZED_FRAME_FILE:-/tmp/tmr_zed_latest.jpg}"

frame_is_fresh() {
  [[ -s "${frame_file}" ]] || return 1
  local modified now
  modified="$(stat -c %Y "${frame_file}" 2>/dev/null)" || return 1
  now="$(date +%s)"
  (( now - modified <= 2 ))
}

if ! frame_is_fresh; then
  screen -S tmr_zed_letters -X quit >/dev/null 2>&1 || true
  screen -S tmr_zed_export -X quit >/dev/null 2>&1 || true
  screen -dmS tmr_zed_letters bash -lc \
    "source /opt/ros/humble/setup.bash; source \"${HOME}/ros2_ws/install/setup.bash\"; unset RMW_IMPLEMENTATION CYCLONEDDS_URI; export ROS_DOMAIN_ID=${vision_domain} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm namespace:=head_camera publish_tf:=false serial_number:=17064700 ros_params_override_path:=${root}/config/zed_letters_override.yaml"
  screen -dmS tmr_zed_export bash -lc \
    "source /opt/ros/humble/setup.bash; source \"${HOME}/ros2_ws/install/setup.bash\"; unset RMW_IMPLEMENTATION CYCLONEDDS_URI; export ROS_DOMAIN_ID=${vision_domain} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; exec python3 ${root}/scripts/zed_frame_export.py --output ${frame_file}"
fi

ready=0
for _ in {1..65}; do
  if frame_is_fresh; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "${ready}" != 1 ]]; then
  echo "ZED RGB frame exporter did not become ready in vision domain ${vision_domain}" >&2
  exit 73
fi

camera_file_supplied=0
for argument in "$@"; do
  if [[ "${argument}" == "--camera-file" || "${argument}" == --camera-file=* ]]; then
    camera_file_supplied=1
    break
  fi
done
if [[ "${camera_file_supplied}" == 1 ]]; then
  exec "${python}" "${root}/scripts/14_letter_guided_search.py" "$@"
fi
export ROS_DOMAIN_ID="${control_domain}"
exec "${python}" "${root}/scripts/14_letter_guided_search.py" "$@" --camera-file "${frame_file}"
