# Franka Duo Mobile cup/bowl mission.
#
# The image supplies the complete Jazzy arm runtime, drivers and mission.
# Run on the arm machine with host networking and the two gripper devices.
# The separate Humble base/camera host is reached over SSH and DDS.
#
# Build: docker build -t franka-duo-table-mission:phase2 .

FROM ros:jazzy-ros-base
LABEL io.ebim.hardware.schema="3" io.ebim.role="arm"

ARG UBUNTU_APT_MIRROR=
ARG ROS_APT_MIRROR=
ARG GIT_PROXY=
ENV EBIM_GIT_PROXY=${GIT_PROXY}

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ros-jazzy-* packages supply the message and action types used on the wire.
# openssh-client is required because the base route runs on the mobile-base
# host and is driven over SSH. util-linux provides flock, which guards single
# ownership of the base velocity channel.
RUN if [[ -n "${UBUNTU_APT_MIRROR}" ]]; then \
      sed -i -E "s#http://(archive|security).ubuntu.com/ubuntu/?#${UBUNTU_APT_MIRROR}#g" \
        /etc/apt/sources.list.d/ubuntu.sources; \
    fi \
    && if [[ -n "${ROS_APT_MIRROR}" ]]; then \
      sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" \
        /etc/apt/sources.list.d/ros2.sources; \
      sed -i 's/^Types: deb deb-src$/Types: deb/' /etc/apt/sources.list.d/ros2.sources; \
    fi \
    && apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-venv \
      python3-colcon-common-extensions \
      openssh-client \
      util-linux \
      git build-essential cmake pkg-config iproute2 tini \
      python3-yaml python3-requests \
      libpoco-dev libeigen3-dev libfmt-dev libconsole-bridge-dev libtinyxml2-dev libcap-dev libboost-dev \
      ros-jazzy-pinocchio ros-jazzy-xacro \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-ros2-control-cmake \
      ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher \
      ros-jazzy-eigen3-cmake-module ros-jazzy-generate-parameter-library \
      ros-jazzy-angles ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-backward-ros ros-jazzy-visualization-msgs \
      libgl1 \
      libglib2.0-0 \
      ros-jazzy-sensor-msgs \
      ros-jazzy-geometry-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-nav-msgs \
      ros-jazzy-control-msgs \
      ros-jazzy-controller-manager-msgs \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-ament-cmake-python \
      ros-jazzy-moveit-core \
      ros-jazzy-moveit-ros-planning \
      ros-jazzy-moveit-kinematics \
      ros-jazzy-ruckig \
      ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
ENV LD_LIBRARY_PATH=/opt/ebim-libfranka/lib:${LD_LIBRARY_PATH}

# Sources are fetched only during build, pinned to the inspected model/driver revisions.
COPY docker/ /app/docker/
COPY hosts/arm/teleoperation_overlay/ /app/hosts/arm/teleoperation_overlay/
RUN CMAKE_BUILD_PARALLEL_LEVEL=2 bash /app/docker/build_drivers.sh

# Dependencies first, so edits to the policy do not invalidate this layer.
# --system-site-packages keeps the apt-installed rclpy and message modules
# importable; a clean venv would hide the entire ROS 2 Python API.
# Perception runs on CPU. Explicit local-version pins prevent pip from
# selecting CUDA wheels from the primary package index.
COPY pyproject.toml ./
ARG PIP_INDEX_URL=https://pypi.org/simple
RUN python3 -m venv --system-site-packages /app/.venv \
    && /app/.venv/bin/pip install --upgrade pip \
    && /app/.venv/bin/pip install \
         --extra-index-url https://download.pytorch.org/whl/cpu \
         "torch==2.8.0+cpu" \
         "torchvision==0.23.0+cpu" \
         "numpy>=1.23.5,<2.3.0" \
         "PyYAML>=6.0.2,<7.0.0" \
         "opencv-python-headless>=4.9.0,<5.0.0" \
         "ultralytics>=8.3.0,<9.0.0" \
         "pytest>=8.1.0,<9.0.0"

# Ultralytics must not reach the network at run time: the weights are baked in
# below and these settings keep it offline.
ENV YOLO_OFFLINE=1 \
    ULTRALYTICS_OFFLINE=1 \
    YOLO_CONFIG_DIR=/app/.ultralytics \
    MPLCONFIGDIR=/tmp/mpl
RUN mkdir -p /app/.ultralytics /tmp/mpl /app/outputs/log

COPY src/ /app/src/
COPY scripts/ /app/scripts/
COPY configs/ /app/configs/
COPY assets/ /app/assets/
COPY site/ /app/site/
COPY base/ /app/base/
COPY hosts/ /app/hosts/
COPY tests/ /app/tests/
COPY docs/ /app/docs/
COPY README.md LICENSE THIRD_PARTY_NOTICES.md /app/
COPY entrypoint.sh /app/entrypoint.sh
COPY hardware.yaml pixi.toml pixi.lock /app/
RUN chmod +x /app/entrypoint.sh /app/scripts/*.sh

# Build the servo, bundled host model and Spine interfaces in the image.
# No robot driver or controller manager is started by this build or check.
RUN source /opt/ros/jazzy/setup.bash \
    && source /opt/ebim-drivers/install/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/duo-log build \
         --base-paths /app/site/franka_duo_joint_servo \
         --build-base /tmp/duo-build \
         --install-base /app/site/install --merge-install \
         --executor sequential --cmake-args -DBUILD_TESTING=OFF \
    && source /app/site/install/setup.bash \
    && python3 -m franka_duo_joint_servo.check_model \
    && python3 -c 'from franka_spine_msgs.action import MoveAbsolute; from franka_spine_msgs.srv import GetPosition, SwitchOn' \
    && python3 /app/docker/check_driver_assets.py \
    && rm -rf /tmp/duo-build /tmp/duo-log

# PYTHONPATH is appended, never replaced: overwriting it hides the ROS 2
# packages that live in the system site-packages.
ENV PYTHONPATH=/app/src:${PYTHONPATH}

# Where the policy finds its baked artifacts. Override any of these to supply
# a different calibration, weight file or taught posture at run time.
ENV TABLE_MISSION_WEIGHTS=/app/assets/weights/yolo11n-seg.pt \
    TABLE_MISSION_CALIBRATION=/app/configs/zed_pnp_calibration.json \
    TABLE_MISSION_CONTRACT=/app/assets/contract \
    TABLE_MISSION_CONFIG=/app/configs/tmr_rgb20d.yaml \
    TABLE_MISSION_STAGE_POSES=/app/configs/grasp_stage_poses.json \
    TABLE_MISSION_OUTPUT_DIR=/app/outputs

# Offline self-check: proves the image is complete with no robot present.
RUN /app/.venv/bin/python -m pytest /app/tests -q

ENTRYPOINT ["/usr/bin/tini", "--", "/app/entrypoint.sh"]
CMD ["mission"]
