# Franka Duo Mobile: Cup / Bowl Pick-and-Place

This project assumes the target has the same two-host baseline as the reference
system: an AMD64 arm host (`.100`) and a Jetson Orin base host (`.50`). Both
hosts build the images from this repository; no reference image or filesystem
is required.

## Hardware prerequisites

| Location | Required |
| --- | --- |
| Robot | Dual FR3v2 arms with FCI enabled, the original lifting Spine/TMR mobile base, two Robotiq 2F-85 grippers, front/rear SICK scanners and a ZED Mini; retain the original mounting geometry, TCPs and firmware baseline |
| `.100` arm host | AMD64 Linux, existing Franka real-time/network configuration, Docker Engine, Git, Bash, SSH client and both grippers exposed as USB-RS485 serial devices; no GPU |
| `.50` base host | Jetson Orin with matching L4T R36.4/NVIDIA driver, Docker Engine with the `nvidia` runtime, Git, Python 3, SSH server and ZED connected through USB 3 |
| Network/access | Existing hardware routes, DDS UDP/multicast between host interfaces, synchronized clocks, passwordless SSH from the arm container to `.50`, and Docker access for both users |

The hosts do not need ROS, MoveIt, kinematics libraries, vendor workspaces or an
HTTP image server. ROS drivers, the KDL/MoveIt model packages, ZED SDK, policy,
weights and routes are installed or copied into the images during the build.
Arm motion uses joint impedance control. Containers use the host kernel and
network; they cannot provide a missing real-time kernel, route or NVIDIA driver.

## `hardware.yaml`

Edit `hardware.yaml` in the checkout on `.100`:

| Field | Value |
| --- | --- |
| `image` | Arm image built below, normally `franka-duo-table-mission:phase2` |
| `hosts.base.image`, `hosts.base.ssh` | Base image built below, and the `.50` SSH user/address |
| `hosts.arm.dds_address`, `hosts.base.dds_address` | Actual DDS interface IP on each host |
| `arms.left_ip`, `arms.right_ip` | Left/right FR3 controller IPs |
| `spine.ip`, `base.ip` | Spine and mobile-base controller IPs |
| `grippers.left_port`, `grippers.right_port` | Correct `/dev/serial/by-id/...` paths on `.100` |
| `lidars.front_ip`, `lidars.rear_ip`, `lidars.host_ip` | Scanner IPs and receiving interface IP on `.50` |
| `camera.serial` | ZED serial; keep `17064700` to use the bundled sample |
| `camera.sdk_settings_dir` | Leave empty: the matching `SN17064700.conf` is bundled in `configs/zed_sdk/`; only set this for a different serial |
| `camera.calibration` | Leave the bundled `configs/zed_pnp_calibration.json` unchanged |
| `domains`, `runtime.playback_speed` | Keep domains 0/97 and speed 0.1 unless both sides change consistently |
| Component `mode` fields | Keep `managed` for complete startup |

The repository already contains the reference camera file
`configs/zed_sdk/SN17064700.conf`, and the base image copies it into the
container. For another serial, add its matching `SN<serial>.conf` to the
repository (or set `camera.sdk_settings_dir` to a directory on `.50`). Leave
the bundled `camera.calibration` path unchanged; it is the reference
camera-to-robot extrinsics used by the grasp policy. Changed mounting geometry
requires a new calibration and taught-pose/route validation.

## Build, deploy and start

Get the same commit on both hosts:

```bash
git clone https://github.com/game-loader/ebim_phase2_task3.git
cd ebim_phase2_task3
git rev-parse HEAD
```

On `.50`, build the base image:

```bash
docker build --network host --platform linux/arm64 -f docker/base.Dockerfile \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu-ports/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  -t franka-duo-base:phase2 .
```

On `.100`, build the arm image:

```bash
docker build --network host --platform linux/amd64 \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  --build-arg PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
  -t franka-duo-table-mission:phase2 .
```

Both builds need internet access and extra disk for caches. The mirror/proxy
arguments are optional. The ARM64 build uses the pinned Stereolabs ZED base
image and does not use a previously built application image.

Configure unattended SSH from `.100` to `.50` (reuse an existing key if present):

```bash
ssh-keygen -t ed25519
ssh-copy-id -i ~/.ssh/id_ed25519.pub tmr-user@172.16.0.50
ssh -o BatchMode=yes tmr-user@172.16.0.50 docker version
```

After editing the IPs/serials in `hardware.yaml` (the bundled ZED factory file
and camera extrinsics already need no preparation), run these commands from the
checkout root on `.100`:

```bash
export EBIM_IMAGE=franka-duo-table-mission:phase2
bash scripts/docker_hardware.sh plan
bash scripts/docker_hardware.sh check
bash scripts/docker_hardware.sh up --activate
bash scripts/docker_hardware.sh status
bash scripts/docker_hardware.sh mission --execute
bash scripts/docker_hardware.sh down
```

All routine commands run locally on `.100`; SSH is used only to manage `.50`.
`up --activate` deploys bundled routes, starts managed drivers and activates
impedance after position checks. `mission --execute` performs physical motion.
From a stopped state, `bash scripts/docker_hardware.sh run --execute` combines
startup and one mission. Pixi is optional (`pixi run run --execute`).

---

The sections below describe mission behavior, configuration and operational
details. They are not additional host setup steps.


## Mission behavior

```
CREATED
 -> INITIALIZING_ARMS            stow both arms inside the base envelope
 -> INITIALIZING_SPINE           spine 0.700 m, verified within 3 mm
 -> OUTBOUND_BASE_RUNNING        drive to the pick table (require FINAL_STOP)
 -> CUP_STAGE_RUNNING            spine 0.468 -> detect -> grasp cup (right arm)
 -> BOWL_STAGE_RUNNING           spine 0.468 -> detect -> grasp bowl (left arm)
 -> RAISING_SPINE_FOR_LETTER     spine 0.700
 -> POST_GRASP_ROUTE_RUNNING     carry both objects to the letter-side table
 -> LOWERING_SPINE_AT_LETTER     spine 0.468
 -> TEST_PLACE_RUNNING           lower, pause, lift; keep both grippers closed
 -> RAISING_SPINE_FOR_RETURN     spine 0.700
 -> RETURN_ROUTE_RUNNING         drive back to the pick side
 -> PLACEMENT_ROUTE_RUNNING      left 0.85 m, align to far table leg, turn 90 deg
 -> LOWERING_SPINE_AT_PLACEMENT  spine 0.468
 -> FINAL_PLACE_RUNNING          touch down, open, leave them
 -> COMPLETE
```

**Perception.** `yolo11n-seg` masks on the head frame. Cup = most confident
`cup`; bowl = the `bowl` with the most dark pixels (rejects the plate). A RANSAC
ellipse is fitted to the rim edge, its centre pixel is intersected with the
horizontal plane at the object's rim height, and that 3-D point is the grasp
target. No depth is used; a known table height replaces it.

**Frames.** Everything is in the **midpoint base frame**: the midpoint of the
two arms' `link0` origins, world-horizontal. It sits on the spine carriage and
moves with it. `--table-z -0.220` means 22 cm below the arm bases. The ZED
extrinsics in `configs/zed_pnp_calibration.json` were solved at **spine
0.468 m**, which is why grasping always happens there; changing it requires
re-calibration (`docs/ZED_PNP_CALIBRATION.md`).

**Placement is the grasp in reverse.** `--mode test` descends, pauses and lifts
while **keeping the gripper closed throughout**. `--mode final` descends, opens and lifts away
empty. X/Y always come from the arm's current pose, never the earlier grasp
point; the base has driven elsewhere by then.

---

## Configuration

| File | Purpose |
| --- | --- |
| `configs/tmr_rgb20d.yaml` | Topic names, workspace bounds, step limits, staleness |
| `hardware.yaml` | Hardware IPs/serials, DDS domains and base container settings |
| `configs/zed_sdk/` | Original ZED factory intrinsics; replacements need the matching serial file |
| `configs/zed_pnp_calibration.json` | Head-camera extrinsics, solved at spine 0.468 m |
| `configs/grasp_stage_poses.json` | Taught dual-arm postures |
| `assets/contract/` | 20-D action contract metadata |
| `assets/weights/yolo11n-seg.pt` | Segmentation weights, 6.2 MB |

`configs/grasp_stage_poses.json` holds up to three 18-D dual-arm EE postures
(left xyz+rot6d, right xyz+rot6d, midpoint frame):

- **`travel_stow` (required, shipped taught):** both arms folded inside the
  base envelope.
- `camera_clear` (optional): arms clear of the ZED view. `null` = skipped.
- `grasp_ready` (optional): grasp start posture. `null` = grasp starts from the
  current posture, which it does anyway.

To retune: Desk -> Programming mode, drag the arms, Desk -> Execution,
`bash scripts/reconnect_arm_state.sh both`, read both `current_pose` topics,
write the 18 values into the file.

---

## Safety and operational notes

- **Two gates for every motion.** Arm output needs `--publish` **and**
  `--enable-robot`; the mission needs `--execute`. Anything less is a dry run.
- **All validation precedes ROS init**: bad config, untaught posture or
  out-of-range spine target fails with no robot involvement.
- **Never restart the joint servo/relay while an impedance controller is
  active.** The site `JointImpedanceController` calls `rclcpp::shutdown()` on
  the whole `ros2_control_node` if its target stream gaps > 0.5 s, which takes
  the arm driver down. Deactivate first; `start_servo_and_activate.sh` enforces
  this. Kill only PIDs you started; never `pkill -f` by name.
- Spine moves are verified by read-back (3 mm). Detection failure aborts in
  place; the policy never descends on a guess.
- A placement refuses to start with an open gripper; a `test` placement fails
  if it ends with the gripper open.
- Every stage writes a JSON report to `outputs/`; `outputs/table_mission.json`
  is the resumable checkpoint.
- On `SIGINT` the accepted plan finishes, the servo holds its last pose, and
  the base adapter latches zero velocity. The operator may stop at any time.

**Known quirks.** The right arm has reflexed on impedance activation when an IK
solution put joint 6 on its lower limit (0.43982 rad); recover with
`reconnect_arm_state.sh right --no-restart`, `error_recovery`, reactivate. The
ZED may enumerate only its HID interface after a host reboot and need a USB 3
replug. Treat "no ZED image" as "check whether the camera host is up".

---

## Repository layout

```
src/franka_duo_tele_data/   policy
  table_mission.py            cross-host coordinator, phases, checkpointing
  table_grasp_stage.py        spine -> clear view -> detect -> pose -> grasp
  table_place_stage.py        placement (test / final)
  grasp_cup_bowl.py           YOLO perception, rim geometry, chunk construction
  spine_client.py             spine height with read-back verification
  zed_pnp_calib.py            extrinsic calibration, ray/plane intersection
  joint_servo_client.py       action-chunk payloads, servo status parsing
  rgb20d_io.py, action_spec.py  20-D observation/action contract
site/franka_duo_joint_servo/  ROS 2 pkg: KDL IK -> Ruckig -> impedance relay
site/franka_duo_ptp_step/     ROS 2 pkg: homing utility (not on the mission path)
base/tmr_base/             base-host routes (ROS 2 Humble): 07 outbound, 13 post-grasp,
                            15 return, 20 placement detour, velocity adapter, leg detection
base/tmr_navigation/        base-host adapter pkg: odom frame adapter, dual-LiDAR merger, SLAM launch
hosts/arm/teleoperation_overlay/  our arm-side ROS 2 overlay incl. the JointImpedanceController
hosts/arm/env/, hosts/base/home/  env scripts and DDS configs for each host
hosts/base/vendor_patches/  our two small diffs against upstream franka_ros2 / zed-ros2-wrapper
scripts/                    operator bring-up scripts
configs/, assets/           configuration, extrinsics, weights, contract
tests/                      offline tests, no robot required
docs/                       TABLE_MISSION.md, JOINT_SERVO_REPLAY.md, ZED_PNP_CALIBRATION.md
```

---

## Declarations

**Object poses.** No externally provided object poses. Cup and bowl positions
come from the policy's own perception (`yolo11n-seg` on the head camera,
intersected with a known table plane). No motion capture, no fiducials, no
operator input during a run. Fixed pre-measured quantities: camera extrinsics
from offline calibration, table height, object dimensions, the taught
`travel_stow` posture, base route distances. Table legs used by the base routes
are detected live from the dual LiDARs.

**Dataset.** The 20-D contract metadata in `assets/contract/` is derived from
our own teleoperated recordings on this platform. Two EE orientation constants
in `grasp_cup_bowl.py` come from those recordings.

**License.** Apache-2.0. See `LICENSE` and `THIRD_PARTY_NOTICES.md`.
