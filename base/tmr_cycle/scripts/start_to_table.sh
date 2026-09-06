#!/usr/bin/env bash
# Run the existing start -> doorway -> fixed pickup-side stop route.
# This route does not run table-leg refinement or arm grasping.
set -eo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
execute=false
mission_args=()
for arg in "$@"; do
  case "$arg" in
    --execute) execute=true ;;
    --disable-collision-guard) mission_args+=("$arg") ;;
    -h|--help)
      echo "Usage: bash scripts/start_to_table.sh [--execute] [--disable-collision-guard]"
      echo "Default: preview only. --execute starts services and moves from the CURRENT pose."
      echo "Use the marked starting position and heading. Collision guard is enabled by default."
      exit 0 ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

cd "${root_dir}"
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
source "${HOME}/tmr_navigation/install/setup.bash"
source "${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash"

export TMR_CYCLE_ROS_DOMAIN_ID=97
export TMR_CYCLE_ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1
if [[ -f "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
else
  unset CYCLONEDDS_URI
fi

# Start the complete route, never inherit a previous resume/guard override.
unset TMR_CYCLE_SKIP_INITIAL_FORWARD TMR_CYCLE_SKIP_TURN
unset TMR_CYCLE_RESUME_YAW_CORRECTION_DEG TMR_CYCLE_DISABLE_COLLISION_GUARD

if [[ "${execute}" != true ]]; then
  echo "[preview] No services or motion will be started."
  echo "[order] 19_ensure_navigation_stack.sh -> 17_control_mode.sh mission -> 07_start_to_pickup.py"
  exec python3 scripts/07_start_to_pickup.py --config config/start_to_pickup.yaml
fi

echo "[1/3] Start or reuse the managed base, LiDAR, odometry, SLAM and command adapter."
bash scripts/19_ensure_navigation_stack.sh
echo "[2/3] Select mission control and stop joystick velocity publication."
bash scripts/17_control_mode.sh mission
echo "[3/3] Run the complete route from the current pose."
exec python3 scripts/07_start_to_pickup.py \
  --config config/start_to_pickup.yaml --execute "${mission_args[@]}"
