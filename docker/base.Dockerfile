# Jetson Orin base/camera image. Build for linux/arm64 on L4T R36.4.
# ZED SDK 5.1.2 + CUDA 12.6 match the inspected .50 host.
FROM stereolabs/zed:5.1.2-devel-l4t-r36.4@sha256:a194acb35508f920588d77c462cb127d199f337e9149349434cdf115e581bcda
LABEL io.ebim.hardware.schema="3" io.ebim.role="base"
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
ARG GIT_PROXY=
ARG UBUNTU_APT_MIRROR=
ARG ROS_APT_MIRROR=
ENV EBIM_GIT_PROXY=${GIT_PROXY}
USER root
RUN test "$(dpkg --print-architecture)" = arm64 \
    && if [[ -n "${UBUNTU_APT_MIRROR}" ]] && [[ -f /etc/apt/sources.list.d/ubuntu.sources ]]; then \
         sed -i -E "s#https?://(archive|security).ubuntu.com/ubuntu/?#${UBUNTU_APT_MIRROR}#g" /etc/apt/sources.list.d/ubuntu.sources; \
       fi \
    && apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main' \
       > /etc/apt/sources.list.d/ros2.list \
    && if [[ -n "${ROS_APT_MIRROR}" ]]; then \
         sed -i "s#https://packages.ros.org/ros2/ubuntu#${ROS_APT_MIRROR}#g" /etc/apt/sources.list.d/ros2.list; \
       fi \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-humble-ros-base python3-colcon-common-extensions python3-yaml python3-numpy \
       python3-opencv python3-requests python3-pytest python3-setuptools \
       build-essential cmake git pkg-config iproute2 util-linux tini \
       libpoco-dev libeigen3-dev libfmt-dev libconsole-bridge-dev libtinyxml2-dev libboost-all-dev \
       ros-humble-pinocchio ros-humble-xacro ros-humble-ament-cmake-python ros-humble-ament-cmake-auto \
       ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-realtime-tools \
       ros-humble-generate-parameter-library ros-humble-backward-ros \
       ros-humble-kdl-parser ros-humble-urdf ros-humble-joint-state-publisher \
       ros-humble-robot-state-publisher ros-humble-rmw-cyclonedds-cpp \
       ros-humble-slam-toolbox ros-humble-tf2-geometry-msgs \
       ros-humble-sick-safetyscanners-base ros-humble-sick-safetyscanners2-interfaces \
       ros-humble-zed-msgs ros-humble-nmea-msgs ros-humble-geographic-msgs \
       ros-humble-image-transport ros-humble-image-transport-plugins \
       ros-humble-diagnostic-updater ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY docker/ /app/docker/
RUN CMAKE_BUILD_PARALLEL_LEVEL=2 bash /app/docker/build_base_drivers.sh
ENV LD_LIBRARY_PATH=/opt/ebim-libfranka/lib:/usr/local/zed/lib:/usr/local/cuda/lib64
COPY scripts/hardware/ /app/scripts/hardware/
COPY base/ /app/base/
COPY configs/zed_sdk/ /app/configs/zed_sdk/
COPY THIRD_PARTY_NOTICES.md /app/
RUN chmod +x /app/docker/base_entrypoint.sh \
    && source /opt/ros/humble/setup.bash && source /opt/ebim-base/install/setup.bash \
    && python3 /app/docker/check_base_assets.py \
    && python3 -m pytest /app/base/tmr_base/tests -q
ENTRYPOINT ["/usr/bin/tini", "--", "/app/docker/base_entrypoint.sh"]
CMD ["keep"]
