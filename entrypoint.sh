#!/usr/bin/env bash
# Container entrypoint for the Franka Duo Mobile cup/bowl mission.
#
# Modes:
#   mission             full pipeline, dry-run (prints the plan, starts nothing)
#   mission --execute   full pipeline, drives the base and both arms
#   grasp   ...         one table stage only (spine, detect, grasp)
#   place   ...         one placement only (--mode test | final)
#   spine   --target-m X [--execute]
#   selftest            offline tests, no robot required
#   model-check         offline model loading and dual-arm FK/IK checks
#   servo               joint target generator only; does not activate controllers
#   relay               relay, robot publication disabled unless explicitly enabled
#   shell               interactive shell with the environment ready
#   hardware ...        complete arm runtime and two-host orchestration
set -euo pipefail

# ROS setup files probe optional unset variables; enable nounset after them.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/ebim-drivers/install/setup.bash ]]; then
  source /opt/ebim-drivers/install/setup.bash
fi
# The image builds its own servo/model and Spine message overlay.
if [[ -f /app/site/install/setup.bash ]]; then
  source /app/site/install/setup.bash
fi
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONPATH="/app/src${PYTHONPATH:+:$PYTHONPATH}"

PY=/app/.venv/bin/python
OUT="${TABLE_MISSION_OUTPUT_DIR:-/app/outputs}"
mkdir -p "${OUT}/log"

mode="${1:-mission}"
if [[ $# -gt 0 ]]; then shift; fi

case "${mode}" in
  hardware)
    # Freeze this container's profile so editing host YAML cannot change the
    # addresses/devices used by an already-running mission or shutdown.
    if [[ -n "${EBIM_HARDWARE:-}" ]]; then
      if [[ ! -f /run/ebim/hardware.yaml ]]; then
        mkdir -p /run/ebim
        cp "${EBIM_CALIBRATION}" /run/ebim/calibration.json
        cp "${EBIM_HARDWARE}" /run/ebim/hardware.yaml
      fi
      export EBIM_HARDWARE=/run/ebim/hardware.yaml
      export EBIM_CALIBRATION=/run/ebim/calibration.json
    fi
    export EBIM_CONTAINER_RUNTIME=1
    exec "${PY}" /app/scripts/hardware.py "$@"
    ;;
  mission)
    # Asset paths are passed explicitly so the arm-local stages the mission
    # spawns resolve them inside the image rather than in a source checkout.
    exec "${PY}" -m franka_duo_tele_data.table_mission \
      --arm-root /app \
      --arm-env /opt/ros/jazzy/setup.bash \
      --arm-python "${PY}" \
      --arm-overlay "${TABLE_MISSION_OVERLAY:-}" \
      --dataset "${TABLE_MISSION_CONTRACT}" \
      --config "${TABLE_MISSION_CONFIG}" \
      --calibration "${TABLE_MISSION_CALIBRATION}" \
      --stage-poses "${TABLE_MISSION_STAGE_POSES}" \
      --weights "${TABLE_MISSION_WEIGHTS}" \
      --output-dir "${OUT}" \
      --checkpoint "${OUT}/table_mission.json" \
      --log-dir "${OUT}/log" \
      --lock "${OUT}/log/.lock" \
      "$@"
    ;;
  grasp)
    exec "${PY}" -m franka_duo_tele_data.table_grasp_stage \
      --dataset "${TABLE_MISSION_CONTRACT}" \
      --config "${TABLE_MISSION_CONFIG}" \
      --calibration "${TABLE_MISSION_CALIBRATION}" \
      --stage-poses "${TABLE_MISSION_STAGE_POSES}" \
      --weights "${TABLE_MISSION_WEIGHTS}" \
      --output "${OUT}/table_grasp_stage.json" \
      "$@"
    ;;
  place)
    exec "${PY}" -m franka_duo_tele_data.table_place_stage \
      --dataset "${TABLE_MISSION_CONTRACT}" \
      --config "${TABLE_MISSION_CONFIG}" \
      --stage-poses "${TABLE_MISSION_STAGE_POSES}" \
      --output "${OUT}/table_place_stage.json" \
      "$@"
    ;;
  spine)
    exec "${PY}" -m franka_duo_tele_data.spine_client "$@"
    ;;
  selftest)
    exec "${PY}" -m pytest /app/tests -q "$@"
    ;;
  model-check)
    exec /usr/bin/python3 -m franka_duo_joint_servo.check_model "$@"
    ;;
  servo)
    exec ros2 launch franka_duo_joint_servo joint_servo.launch.py "$@"
    ;;
  relay)
    exec ros2 launch franka_duo_joint_servo gello_target_relay.launch.py "$@"
    ;;
  shell)
    exec /bin/bash "$@"
    ;;
  --help | -h | help)
    sed -n '3,14p' "$0"
    exit 0
    ;;
  *)
    echo "unknown mode: ${mode}" >&2
    sed -n '3,14p' "$0" >&2
    exit 2
    ;;
esac
