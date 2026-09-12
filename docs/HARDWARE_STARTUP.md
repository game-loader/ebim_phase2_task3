# Hardware configuration and Docker startup

The `.100` arm machine runs one container with `--network host`: Franka drivers,
custom impedance controllers, Robotiq drivers, Spine server, joint servo,
relay, perception and mission. It needs no host ROS workspace, MoveIt install,
SSH self-login or Docker socket mount. The `.50` base/camera machine runs a
second container with host networking: Humble, TMR/swerve drivers, SICK drivers,
ZED SDK/wrapper, navigation adapters, SLAM and the bundled routes. The arm
container manages it over SSH and receives camera data over DDS. All arm
motion still uses impedance control. MoveIt/KDL provide
the bundled model and IK inside the image; no `move_group` is started.

## Configure another unit

Edit `hardware.yaml` for the same TMR FR3v2 Duo hardware and mounting geometry:

- Arm host DDS address, left/right arm IPs, Spine/base IPs.
- Left/right gripper `/dev/serial/by-id/...` paths. The launcher reads these
  once and maps them to `/dev/ebim-left-gripper` and `/dev/ebim-right-gripper`.
- Base SSH address, image reference and container name. Runtime paths are
  supplied by the images; no host ROS setup/overlay paths are needed.
- ZED Mini serial/calibration, scanner IPs and receiving interface IP.
- DDS domains and playback speed (default `0.1`).

This profile assumes the same hardware, mounting geometry and underlying
system configuration as the inspected machines. Serial/IP changes do not
cover changed mounting transforms, TCP, robot model or base kinematics.
Validate camera extrinsics and taught postures/routes for the physical cell. The default routes,
calibration, model, weights and taught poses are already in this submission.

The base image includes the original ZED Mini's `SN17064700.conf` factory
intrinsics. For another serial, put its matching `SN<serial>.conf` in a data
directory on `.50` and set `camera.sdk_settings_dir` to it. This reads only
calibration data, not a host SDK. Prepare the file before offline deployment;
startup refuses a missing serial-specific calibration. Factory intrinsics
are separate from `camera.calibration`, the policy's camera-to-robot extrinsics.

Keep the calibration file at the path named in YAML (relative to the YAML
directory, or absolute on the arm host). The launcher mounts the profile and
calibration read-only, then snapshots them inside the container. Editing the
host YAML does not retarget a running mission or its shutdown procedure.
Stop the runtime before applying a changed profile.

## Host prerequisites

| Machine | Must already provide |
| --- | --- |
| `.100` arm | Linux Docker Engine with the completed image loaded; Bash; hardware network routes; USB serial devices; kernel/RT scheduling support suitable for Franka FCI; robot FCI enabled |
| `.50` base/camera | Jetson Orin ARM64, matching L4T R36.4 kernel/NVIDIA drivers, Docker Engine with NVIDIA runtime and base image loaded, Python 3 standard library, SSH server, USB/video devices, matching ZED factory calibration |
| Both | Correct interface addresses and synchronized clocks |

Read-only inspection confirmed `.50` uses Ubuntu **22.04.5**, L4T **R36.4.0**,
CUDA **12.6**, ZED SDK **5.1.2**, and an already configured Docker `nvidia`
runtime. The base image is pinned to the official matching Stereolabs image.
The `.100` image uses Ubuntu 24.04 / ROS Jazzy; `.50` uses Ubuntu 22.04 / ROS
Humble inside its image. Neither host needs ROS, MoveIt, Python ROS modules,
vendor workspaces or a host ZED SDK installed for this workflow.

The arm container supplies SSH client software. Mount an SSH directory containing
a usable key and verified `known_hosts` entries for `.50`; encrypted keys need
an authentication setup usable in the container (no agent forwarding is added).
By default the launcher mounts `$HOME/.ssh` to `/root/.ssh` read-only; override
with `EBIM_SSH_DIR=/absolute/path`. SSH config entries must use paths valid
inside that mount. The `.50` SSH user must be able to run Docker without an
interactive sudo prompt. No credentials are stored in the image or hardware YAML.

Both containers use host networking, private IPC, `SYS_NICE`, `IPC_LOCK`, RT
priority and unlimited memlock limits. The arm container maps two explicit
serial devices and needs no GPU. The base container uses the NVIDIA runtime,
USB bus/video devices and read-only udev metadata for ZED. Neither uses
`--privileged`, host PID namespace, a Docker socket mount or nested Docker.
These options cannot supply a missing RT kernel or configure host networking.

## Commands on `.100`

Load both images separately; no command below builds or pulls them. Default
arm image: `franka-duo-table-mission:phase2`. If changed, set both `image` in
YAML and `EBIM_IMAGE` to that reference. Default base image:
`franka-duo-base:phase2`, configured in `hosts.base.image`.

After image delivery, on `.100`: `docker load -i arm-image.tar`; on `.50`:
`docker load -i base-image.tar`. Then configure `hardware.yaml` and SSH access.
All routine startup commands below run locally on `.100`.

```bash
# Validate profile/print plan without connecting to robot hosts.
bash scripts/docker_hardware.sh plan
# Inspect dependencies/devices and base SSH; starts no drivers.
bash scripts/docker_hardware.sh check
# Start runtime, align targets, activate impedance; no mission yet.
bash scripts/docker_hardware.sh up --activate
bash scripts/docker_hardware.sh status
bash scripts/docker_hardware.sh mission           # dry run
bash scripts/docker_hardware.sh mission --execute # physical mission
bash scripts/docker_hardware.sh down
```

One command for startup plus physical mission:

```bash
bash scripts/docker_hardware.sh run --execute
```

Pixi is optional and wraps the same launcher:

```bash
pixi run plan       # local Python validation; does not require a Docker image
pixi run check
pixi run up --activate
pixi run mission --execute
# Or: pixi run run --execute
pixi run down
```

`plan`, `check`, `up` and `run` accept `--hardware /path/to/hardware.yaml`.
`status`, `mission` and `down` use the running container's snapshot. The default
container name is `ebim-cup-bowl-runtime`; `EBIM_CONTAINER` overrides it.
The arm host requires no Python when using the Bash launcher: YAML is parsed by
a temporary network-isolated container from the same image.

## Startup, failures and shutdown

`up` checks dependencies and the base image/platform, creates the base
container, deploys a versioned release into its volume, starts base/navigation
and camera, then arm/gripper/Spine drivers and servo/relay. It checks live state,
image/intrinsics, map/TF and servers. Process checks and bounded DDS discovery
reject existing unmanaged publishers/managers; they do not kill them. DDS
checks only cover the configured domains/interfaces and canonical names.

Arm impedance starts inactive. `--activate` requires fresh target/measured
positions within `0.003 rad`, correct joint order and healthy robot states.
The servo has a 60-second idle-follow window. If `up` without activation
exceeds it, use `down`, then `up --activate`. `up` refuses an existing runtime
container instead of restarting active streams or silently changing config.

The containers remain alive after a mission finishes, a startup failure or a
child process failure. Failed children are recorded and block new missions;
they are not automatically restarted and robot faults are not reset. The
supervisor checks process ownership/child failures, not continuous physical
safety. Mission startup additionally checks live interfaces and robot state.

Each route runs via SSH → `docker exec -i` → a container-side watchdog. The
arm client sends heartbeats; EOF or a three-second heartbeat timeout stops
the route process group with bounded SIGINT/TERM/KILL escalation. Nested route
tasks inherit that group when supervised, including the return's door segment. The velocity
adapter's existing command timeout also stops stale motion commands. The base
orchestration lock prevents managed shutdown during a route; route completion
does not shut down the driver container.

`down` takes the orchestration lock, deactivates impedance on its managed arm
drivers, confirms command controllers are inactive, then stops servo and
drivers before the base processes. If any check fails or a mission owns the
lock, shutdown is refused and the container is retained. Active `external`
arm controllers must be deactivated by their owner. Use this command instead
of directly killing/stopping Docker: forced container/host termination cannot
guarantee controller deactivation or continued streaming.

`external` mode reuses matching canonical interfaces; it does not adapt
arbitrary topic names. Navigation adapters/SLAM remain managed. Base software
running in other domains or hardware processes started outside this workflow
must be reconciled explicitly before managed startup.

## Deployment and logs

Routes, navigation Python adapters, launch helpers, DDS profiles and calibration
are deployed under `/app/runtime/releases/<content-hash>/` inside `.50`'s container. Existing
vendor trees and shell startup files are not patched. No pre-existing
`~/tmr_base` or compiled `~/tmr_navigation` is required. Arm runtime code is
already in the image and its release/state lives in `/app/runtime`.

Docker volumes `<container>-state` and `<container>-outputs` persist logs and
mission checkpoints for the arm container. On `.50`, `<base-container>-state`
persists deployed routes/state/logs and `<base-container>-zed-settings` persists
SDK calibration. Both survive container removal. Process ownership includes
boot ID, PID namespace and process start time, so old container PIDs cannot be
reused as ownership evidence. A mission checkpoint intentionally prevents
replaying a completed drive; follow the README's reset-between-rounds procedure.

```bash
bash scripts/docker_hardware.sh logs
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/arm.log
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/servo.log
docker cp ebim-cup-bowl-runtime:/app/outputs ./mission-outputs
# On .50:
docker logs --tail 100 ebim-cup-bowl-base
docker exec ebim-cup-bowl-base tail -n 100 /app/runtime/logs/base.log
docker exec ebim-cup-bowl-base tail -n 100 /app/runtime/logs/camera.log
```

## Build and validation status

`docker/drivers.lock.json` pins vendor sources. Docker builds libfranka and the
required Jazzy packages, then the bundled servo/model. The build checks plugin
libraries, generated FR3v2/Robotiq URDFs, controller configuration, executable
installation and FK/IK without starting hardware. Source/license copies are
retained in `/opt/ebim-vendor-src`.

`docker/base_drivers.lock.json` pins the inspected Humble Franka, description,
Olive, SICK and ZED wrapper revisions; their sources/licenses remain in
`/opt/ebim-base-vendor-src`. `docker/base.Dockerfile` pins the ZED/CUDA base image
by digest. ROS/system packages are installed from apt and are not individually
version-locked. The build preserves the inspected swerve controller parameters,
merging upstream duplicate YAML root mappings without changing their values.
Its offline checks exercise the real TMR xacro, installed executables/libraries,
SDK version and route tests. No hardware nodes are started during the build.
The image uses Ubuntu's matching NumPy/OpenCV packages. The SDK base's optional
`pyzed` Python API and pip NumPy 2 are removed because Humble's OpenCV/cv_bridge
extensions require the NumPy 1 ABI. The ZED ROS wrapper uses the retained C++ SDK.

Build commands on the corresponding architectures:

```bash
# AMD64 builder
docker build --platform linux/amd64 -t franka-duo-table-mission:phase2 .
# ARM64 builder; official Jetson SDK development base, no camera needed to build
docker build --platform linux/arm64 -f docker/base.Dockerfile -t franka-duo-base:phase2 .
```

若构建机访问 GitHub 较慢，可临时增加
`--build-arg GIT_PROXY=https://gh-proxy.org`；锁定的仓库地址和提交不会改变。

On Jetson, the Ubuntu mirror must serve **ubuntu-ports** (ARM64):

```bash
docker build --network host --platform linux/arm64 -f docker/base.Dockerfile \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu-ports/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  -t franka-duo-base:phase2 .
```

`--network host` also avoids Docker bridge creation on Jetson kernels without
the iptables raw table. It changes build networking only, not host firewall rules.
For an offline transfer, pull the pinned ZED digest on an ARM64-capable builder,
tag it, and stream `docker save` into `ssh <base-host> docker load` (optionally
compressing the stream). This avoids storing an extra archive on the Jetson.
Docker save/load can lose the registry digest; after confirming the imported
image ID is `sha256:59421aba196373f6c32943d7b7aac2547559222d759269011d7d6385d7d0fc81`,
add `--build-arg BASE_IMAGE=stereolabs/zed:5.1.2-devel-l4t-r36.4` to use it locally.
The default build continues to pin the registry digest.

### Arm image verified on 2026-09-12

The AMD64 image was built on the `.100` arm host from commit `a33d17b` and
tagged `franka-duo-table-mission:a33d17b`. All 398 tracked source files in the
isolated remote build directory were checked against local SHA-256 hashes.
The image ID is
`sha256:c9d0b1e16d21957e6e6eb5b866c61f71489f6293a69ae6159dbeff676a771538`;
Docker reports 6.13 GB local disk usage. The default `:phase2` tag was not set;
set `hardware.yaml`'s top-level `image` to the verified tag when using it.

The build used these temporary arguments (host package sources were unchanged):

```bash
--build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu/ \
--build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
--build-arg GIT_PROXY=https://gh-proxy.org \
--build-arg PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
```

The 15 vendor/overlay packages and the joint servo compiled successfully.
`realtime_tools` comes from Jazzy alongside its `ros2_control` binaries; the
older vendor copy must not shadow that ABI. Torch `2.8.0+cpu` and torchvision
`0.23.0+cpu` are installed separately from the CPU wheel index, with their
transitive dependencies resolved through the normal PyPI index.

Verification ran in disposable containers with `--network none`, without
hardware device mappings or host workspaces:

- 255 policy and base-route tests passed.
- Both seven-joint KDL solvers passed three FK/IK round trips each (`model-check`).
- Five driver plugins resolved their shared-library symbols; dual FR3v2 and
  Robotiq xacros, controller parameters, executables and Spine imports passed.
- The bundled YOLO segmentation weights ran CPU inference on a synthetic RGB
  frame; Torch reported no CUDA runtime, and ROS image/message imports passed.
- `mission` and `hardware plan` printed their dry-run plans successfully.

Build and verification logs are in
`/home/aup/ebim_phase2_builds/f1ea686/` on `.100` (`build.log` and
`offline-verify.log`); the directory name predates the build fixes.

This recorded image predates the source directory rename to `base/tmr_base`.
Rebuild the image to include the renamed directory and updated startup paths.

The ARM64 base image was built on `.50` from commit `db0f066` as
`franka-duo-base:db0f066` (image ID
`sha256:62264216929c4858b1e48359909ea2a939adc1fbafa21dc5aae9ef3cedc0d47a`,
12.54 GB). All 396 tracked source files in the remote build directory matched
the commit's SHA-256 hashes. The build passed the package/asset checks,
OpenCV/cv_bridge image conversion and 130 base tests. An
offline container probe loaded the Franka, mobile, ZED, SICK, controller-manager
and SLAM libraries and detected one CUDA device. A synthetic CycloneDDS probe
between this image on `.50` and the existing arm image on `.100` transferred
228 acknowledged 640x360 RGB frames with matching CameraInfo out of 243 sent
on domain 0. A second run on isolated domain 42 acknowledged 230 of 245 frames.
Each run lasted 35 seconds and used sensor-data QoS for images/intrinsics and
reliable String acknowledgements; frame content was verified with CRC32.
Counts include discovery/startup time and are not a steady-state loss benchmark.
Both runs emitted CycloneDDS deserialization warnings on Humble, including
one on the isolated domain. Their cause remains unresolved despite successful
image transfer; sustained production traffic still needs validation.
No hardware nodes or motion were
started by these checks; live ZED capture, laser scans and physical control
remain untested in these containers.

Build, offline and DDS logs are retained on `.50` in
`/home/tmr-user/ebim_phase2_builds/39d9ee2/` (`build-hostnet.log`,
`offline-verify.log`, `dds-image-domain0.log`, `dds-image-verify.log`);
the directory name predates the build fixes. The corresponding arm DDS logs
are in `/home/aup/ebim_phase2_builds/f1ea686/`. Temporary validation containers
were removed. Set `hosts.base.image` to `franka-duo-base:db0f066` to select this
image; the default `:phase2` tag was not changed.
