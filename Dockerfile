# Franka Duo Mobile cup/bowl mission.
#
# The policy runs inside this image but drives real hardware through the ROS 2
# graph the robot host already publishes (Franka arm drivers, Robotiq grippers,
# ZED, and the spine action server). The image therefore ships the same ROS
# distribution as the arm host (Jazzy) and must be run with --network host so
# DDS discovery reaches those nodes. See README.md for the full run command.
#
# Build: docker build -t franka-duo-table-mission:phase2 .

FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ros-jazzy-* packages supply the message and action types used on the wire.
# openssh-client is required because the base route runs on the mobile-base
# host and is driven over SSH. util-linux provides flock, which guards single
# ownership of the base velocity channel.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-venv \
      openssh-client \
      util-linux \
      libgl1 \
      libglib2.0-0 \
      ros-jazzy-sensor-msgs \
      ros-jazzy-geometry-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-nav-msgs \
      ros-jazzy-control-msgs \
      ros-jazzy-controller-manager-msgs \
      ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Dependencies first, so edits to the policy do not invalidate this layer.
# --system-site-packages keeps the apt-installed rclpy and message modules
# importable; a clean venv would hide the entire ROS 2 Python API.
# Perception runs on CPU (~15 ms per frame after warm-up), so the CPU-only
# torch wheels are used and no CUDA runtime is needed.
COPY pyproject.toml ./
RUN python3 -m venv --system-site-packages /app/.venv \
    && /app/.venv/bin/pip install --upgrade pip \
    && /app/.venv/bin/pip install \
         --extra-index-url https://download.pytorch.org/whl/cpu \
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
COPY tests/ /app/tests/
COPY docs/ /app/docs/
COPY README.md LICENSE THIRD_PARTY_NOTICES.md /app/
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh /app/scripts/*.sh

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

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["mission"]
