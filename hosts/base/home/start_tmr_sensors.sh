#!/usr/bin/env bash

set -Eeo pipefail

LOCK_FILE=/tmp/tmr_sensor_stack.lock
ROS_SETUP=/opt/ros/humble/setup.bash
WORKSPACE_SETUP=/home/tmr-user/ros2_ws/install/setup.bash
ZED_OVERRIDE=/home/tmr-user/zed_override.yaml

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "[ERROR] TMR sensor stack is already running."
    echo "[INFO] Stop the existing process with Ctrl+C before restarting."
    exit 1
fi

for required_file in \
    "${ROS_SETUP}" \
    "${WORKSPACE_SETUP}" \
    "${ZED_OVERRIDE}"
do
    if [ ! -f "${required_file}" ]; then
        echo "[ERROR] Required file not found: ${required_file}"
        exit 1
    fi
done

source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

PIDS=()

cleanup()
{
    local exit_code=$?

    trap - EXIT INT TERM

    echo
    echo "[INFO] Stopping TMR sensor stack..."

    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -INT "${pid}" 2>/dev/null || true
        fi
    done

    for pid in "${PIDS[@]}"; do
        wait "${pid}" 2>/dev/null || true
    done

    echo "[INFO] TMR sensor stack stopped."
    exit "${exit_code}"
}

trap cleanup EXIT INT TERM

echo "[INFO] Starting ZED-M on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
(
    exec ros2 launch zed_wrapper zed_camera.launch.py \
        camera_model:=zedm \
        camera_name:=head_camera \
        publish_tf:=false \
        serial_number:=17064700 \
        ros_params_override_path:="${ZED_OVERRIDE}"
) &
PIDS+=("$!")

echo "[INFO] Starting front and rear LiDARs on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
(
    exec ros2 launch franka_mobile_sensors \
        franka_mobile_sensors.launch.py \
        start_cameras:=false \
        start_lidars:=true \
        start_rviz:=false
) &
PIDS+=("$!")

echo
echo "[INFO] Sensor processes started."
echo "[INFO] ZED PID:    ${PIDS[0]}"
echo "[INFO] LiDAR PID:  ${PIDS[1]}"
echo "[INFO] Press Ctrl+C to stop ZED and both LiDARs."

wait -n "${PIDS[@]}"

echo "[ERROR] A sensor process exited unexpectedly."
exit 1
