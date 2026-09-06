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
#   shell               interactive shell with the environment ready
set -euo pipefail

# ROS setup files probe optional unset variables; enable nounset after them.
set +u
source /opt/ros/jazzy/setup.bash
# The site ROS 2 packages (joint servo + gello relay) are built on the robot
# host and bind-mounted in; source the overlay when it is present.
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
[[ $# -gt 0 ]] && shift || true

case "${mode}" in
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
  shell)
    exec /bin/bash "$@"
    ;;
  --help | -h | help)
    sed -n '3,11p' "$0"
    exit 0
    ;;
  *)
    echo "unknown mode: ${mode}" >&2
    sed -n '3,11p' "$0" >&2
    exit 2
    ;;
esac
