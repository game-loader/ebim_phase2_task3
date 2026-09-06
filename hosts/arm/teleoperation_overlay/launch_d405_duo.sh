#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/recloned_sources/franka_ros2_jazzy_ws/install/setup.bash"
if [ -f "$HOME/recloned_sources/teleoperation_overlay/install/setup.bash" ]; then
  source "$HOME/recloned_sources/teleoperation_overlay/install/setup.bash"
fi

exec ros2 launch realsense2_camera rs_multi_camera_launch.py \
  camera_namespace1:='/' camera_name1:='wrist_camera_left' serial_no1:=_409122272639\
  enable_sync1:=false enable_depth1:=true \
  depth_module.color_profile1:=640x480x30 depth_module.depth_profile1:=640x480x30 \
  camera_namespace2:='/' camera_name2:='wrist_camera_right' serial_no2:=_409122274492 \
  enable_sync2:=false enable_depth2:=true \
  depth_module.color_profile2:=640x480x30 depth_module.depth_profile2:=640x480x30
