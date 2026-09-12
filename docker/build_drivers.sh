#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
vendor=/opt/ebim-vendor-src
python3 /app/docker/fetch_drivers.py /app/docker/drivers.lock.json "$vendor"
cmake -S "$vendor/libfranka" -B /tmp/ebim-libfranka-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ebim-libfranka \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF
cmake --build /tmp/ebim-libfranka-build --parallel 2
cmake --install /tmp/ebim-libfranka-build
export CMAKE_PREFIX_PATH="/opt/ebim-libfranka:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="/opt/ebim-libfranka/lib:${LD_LIBRARY_PATH:-}"
# Explicit base paths avoid optional GUI, simulation, mobile and teleoperation packages.
colcon --log-base /tmp/ebim-driver-log build \
  --base-paths \
    "$vendor/franka_description" "$vendor/serial" \
    "$vendor/ros2_robotiq_gripper/robotiq_description" \
    "$vendor/ros2_robotiq_gripper/robotiq_driver" \
    "$vendor/ros2_robotiq_gripper/robotiq_controllers" \
    "$vendor/franka_ros2/realtime_tools/realtime_tools" \
    "$vendor/franka_ros2/franka_msgs" \
    "$vendor/franka_ros2/franka_semantic_components" \
    "$vendor/franka_ros2/franka_hardware" \
    "$vendor/franka_ros2/franka_robot_state_broadcaster" \
    "$vendor/franka_ros2/franka_bringup" \
    "$vendor/franka_ros2/franka_gripper" \
    "$vendor/franka_ros2/franka_spine/franka_spine_msgs" \
    "$vendor/franka_ros2/franka_spine/franka_spine_server" \
    /app/hosts/arm/teleoperation_overlay/src/franka_fr3_arm_controllers \
    /app/hosts/arm/teleoperation_overlay/src/franka_gripper_manager \
  --build-base /tmp/ebim-driver-build --install-base /opt/ebim-drivers/install \
  --merge-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
rm -rf /tmp/ebim-libfranka-build /tmp/ebim-driver-build /tmp/ebim-driver-log
