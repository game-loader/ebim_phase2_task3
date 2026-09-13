# Build on the x86 station: docker build --platform linux/amd64 \
#   -f docker/hamburg.Dockerfile -t franka-duo-table-mission:hamburg .
FROM ros:humble-ros-base
LABEL io.ebim.role="external" io.ebim.hardware.schema="3"
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1 PIP_NO_CACHE_DIR=1
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions python3-pip python3-venv \
      python3-yaml python3-numpy iproute2 util-linux tini libglib2.0-0 libgl1 \
      ros-humble-rmw-fastrtps-cpp ros-humble-ament-cmake-python \
      ros-humble-eigen3-cmake-module ros-humble-moveit-core \
      ros-humble-moveit-ros-planning ros-humble-moveit-kinematics ros-humble-ruckig \
      ros-humble-controller-manager-msgs ros-humble-control-msgs \
      ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs \
      ros-humble-tf2-ros ros-humble-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
RUN python3 -m venv --system-site-packages /app/.venv \
    && /app/.venv/bin/pip install --upgrade pip \
    && /app/.venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu \
      'torch==2.8.0+cpu' 'torchvision==0.23.0+cpu'
ARG PIP_INDEX_URL=https://pypi.org/simple
RUN /app/.venv/bin/pip install 'numpy>=1.23.5,<2' 'PyYAML>=6.0.2,<7' \
      'opencv-python-headless==4.11.0.86' 'opencv-python==4.11.0.86' \
      'ultralytics>=8.3.0,<9' 'pytest>=8.1,<9'
COPY site/franka_duo_joint_servo/ /app/site/franka_duo_joint_servo/
COPY hosts/arm/teleoperation_overlay/src/franka_spine_msgs/ /app/interfaces/franka_spine_msgs/
# Only the model, servo and wire interfaces are built. No hardware driver/SDK.
RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/ebim-log build \
      --base-paths /app/site/franka_duo_joint_servo /app/interfaces/franka_spine_msgs \
      --build-base /tmp/ebim-build --install-base /app/site/install --merge-install \
      --executor sequential --cmake-args -DBUILD_TESTING=ON \
    && source /app/site/install/setup.bash \
    && colcon --log-base /tmp/ebim-log test --build-base /tmp/ebim-build \
      --install-base /app/site/install --merge-install --base-paths /app/site/franka_duo_joint_servo /app/interfaces/franka_spine_msgs \
    && colcon test-result --test-result-base /tmp/ebim-build --verbose \
    && python3 -m franka_duo_joint_servo.check_model \
    && python3 -c 'from franka_spine_msgs.action import MoveAbsolute; from franka_spine_msgs.srv import GetPosition' \
    && rm -rf /tmp/ebim-build /tmp/ebim-log
COPY . /app/
ENV PYTHONPATH=/app/src \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    EBIM_ROS_GATEWAY=/app/runtime/ros.sock \
    EBIM_EXTERNAL_HARDWARE=1 \
    YOLO_OFFLINE=1 ULTRALYTICS_OFFLINE=1 YOLO_CONFIG_DIR=/app/.ultralytics \
    MPLCONFIGDIR=/tmp/mpl \
    TABLE_MISSION_WEIGHTS=/app/assets/weights/yolo11n-seg.pt \
    TABLE_MISSION_CALIBRATION=/app/configs/zed_pnp_calibration.json \
    TABLE_MISSION_CONTRACT=/app/assets/contract \
    TABLE_MISSION_CONFIG=/app/configs/tmr_rgb20d.yaml \
    TABLE_MISSION_STAGE_POSES=/app/configs/grasp_stage_poses.json \
    TABLE_MISSION_OUTPUT_DIR=/app/outputs
RUN chmod +x /app/entrypoint.sh \
    && /app/.venv/bin/python -m pytest /app/tests -q
ENTRYPOINT ["/usr/bin/tini", "--", "/app/entrypoint.sh"]
CMD ["hardware", "plan", "--hardware", "/app/hardware.hamburg.yaml"]
