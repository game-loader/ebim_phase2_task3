#!/usr/bin/env bash
# Preserve the caller errexit setting because this file is usually sourced.
_tmr_saved_errexit=
case $- in
  *e*) _tmr_saved_errexit=1 ;;
esac

set -e
source /opt/ros/jazzy/setup.bash
source /home/aup/recloned_sources/franka_ros2_jazzy_ws/install/setup.bash
if [ -f /home/aup/recloned_sources/teleoperation_overlay/install/setup.bash ]; then
  source /home/aup/recloned_sources/teleoperation_overlay/install/setup.bash
fi

if [ -n "$_tmr_saved_errexit" ]; then
  set -e
else
  set +e
fi
unset _tmr_saved_errexit
