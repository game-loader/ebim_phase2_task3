# Teleoperation Overlay Migration

Created: 2026-08-12T16:36:27+08:00
Source: /home/aup/ros2_ws/overlays/teleoperation
Target: /home/aup/recloned_sources/teleoperation_overlay
Underlay: /home/aup/recloned_sources/franka_ros2_jazzy_ws

This overlay contains the previously working field packages/configuration for:
- GELLO state publisher
- Robotiq gripper manager
- FR3 arm teleoperation controllers
- D405 launch helper
- Spine/pedal teleoperation bridge and franka_spine_msgs

The new upstream franka_ros2 workspace is kept separate and should be sourced before this overlay.
