#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
vendor=/opt/ebim-base-vendor-src
python3 /app/docker/fetch_drivers.py /app/docker/base_drivers.lock.json "$vendor"
cmake -S "$vendor/libfranka" -B /tmp/ebim-libfranka-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ebim-libfranka \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF
cmake --build /tmp/ebim-libfranka-build --parallel 2
cmake --install /tmp/ebim-libfranka-build
export CMAKE_PREFIX_PATH="/opt/ebim-libfranka:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="/opt/ebim-libfranka/lib:/usr/local/zed/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
# Preserve the inspected swerve parameters, but merge duplicate /** mappings.
# YAML parsers otherwise silently discard all but the final controller block.
python3 /app/docker/prepare_base_sources.py "$vendor"
colcon --log-base /tmp/ebim-base-log build \
  --base-paths \
    "$vendor/franka_description" "$vendor/olvx_descriptions_module" \
    "$vendor/franka_ros2/franka_msgs" "$vendor/franka_ros2/franka_hardware" \
    "$vendor/franka_ros2/franka_semantic_components" "$vendor/franka_ros2/franka_mobile" \
    "$vendor/franka_ros2/franka_bringup" "$vendor/sick_safetyscanners2" \
    "$vendor/zed_ros2_wrapper/zed_components" "$vendor/zed_ros2_wrapper/zed_wrapper" \
  --build-base /tmp/ebim-base-build --install-base /opt/ebim-base/install \
  --merge-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs
rm -rf /tmp/ebim-libfranka-build /tmp/ebim-base-build /tmp/ebim-base-log
