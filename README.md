# Franka Duo Mobile: Cup / Bowl Pick-and-Place

双 FR3 杯碗抓取与移动任务。以下假设目标也有相同硬件和底层配置的
`.100` 机械臂主机及 `.50` Jetson 底盘主机。两台机器都从本仓库源码构建，
不依赖预先交付的镜像或我们测试机器上的文件。

## 硬件前提

| 位置 | 需要具备 |
| --- | --- |
| 机器人 | 双 FR3v2（FCI 可用）、原有升降 Spine / TMR 底盘、两个 Robotiq 2F-85、前后 SICK 雷达、ZED Mini；安装位置与原机一致 |
| `.100` | AMD64 Linux、原有 Franka 实时内核/网络配置、Docker、Git、Bash、SSH 客户端、两个夹爪 USB-RS485 设备；不需要 GPU |
| `.50` | Jetson Orin、匹配的 L4T R36.4 / NVIDIA 驱动、Docker + NVIDIA runtime、Git、Python 3、SSH 服务、ZED USB 3 连接 |
| 网络与权限 | 原有硬件网段和路由可用；两机 DDS UDP/组播互通、时钟同步；`.100` 容器可免密 SSH 到 `.50`，对应用户可直接运行 Docker |

未安装的上述宿主机工具需先安装；两机首次构建需要联网，并预留镜像与编译缓存空间。
ROS、运动学库、驱动、ZED SDK 会在构建时安装到镜像，模型、权重和默认路线随源码打包，
不需要主机提前启动 ROS topic，也不需要 HTTP 图像服务。
机械臂使用阻抗控制。沿用两机现有底层配置即可；Docker 不提供宿主机内核或硬件网络配置。

## 填写 hardware.yaml

在 **`.100` 的本仓库根目录**修改 [hardware.yaml](hardware.yaml)：

| 字段 | 填写内容 |
| --- | --- |
| `image` | 保留 `franka-duo-table-mission:phase2`，对应下方在 `.100` 构建的镜像 |
| `hosts.base.image` / `hosts.base.ssh` | 保留 `franka-duo-base:phase2` / 填写 `.50` 的 SSH 用户和地址 |
| `hosts.arm.dds_address` / `hosts.base.dds_address` | 两台主机实际用于 DDS 通信的本机 IP |
| `arms.left_ip` / `arms.right_ip` | 左右机械臂 IP |
| `spine.ip` / `base.ip` | 升降机构 / 底盘控制器 IP |
| `grippers.left_port` / `grippers.right_port` | `.100` 上左右夹爪的 `/dev/serial/by-id/...` 路径 |
| `lidars.front_ip` / `lidars.rear_ip` / `lidars.host_ip` | 前后雷达 IP / `.50` 接收雷达数据的本机接口 IP |
| `camera.serial` / `camera.sdk_settings_dir` | ZED 序列号 / `.50` 上存放对应 `SN<序列号>.conf` 的目录 |
| `camera.calibration` | `.100` 上相机到机器人的外参文件，可填相对于 YAML 的路径 |
| 其余字段 | 相同硬件通常保留默认：各设备 `mode: managed`、DDS 域 0/97、回放速度 0.1 |

换相机序列号必须准备对应出厂标定文件；镜像内仅附带 `SN17064700.conf`。
相机外参与出厂标定是两份不同的数据，安装位置变化时需重新校准外参。

## 部署与启动

先在**两台机器各执行一次**，获取同一版本源码（也可直接解压同一份提交源码包）：

```bash
git clone https://github.com/game-loader/ebim_phase2_task3.git
cd ebim_phase2_task3
git rev-parse HEAD  # 两端应为同一提交；私有仓库需先配置访问权限
```

**在 `.50` 的仓库根目录构建底盘镜像：**

```bash
docker build --network host --platform linux/arm64 -f docker/base.Dockerfile \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu-ports/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  -t franka-duo-base:phase2 .
```

**在 `.100` 的仓库根目录构建机械臂镜像：**

```bash
docker build --network host --platform linux/amd64 \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  --build-arg PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
  -t franka-duo-table-mission:phase2 .
```

两条构建命令会下载各自的基础镜像及依赖。国内源和 GitHub 代理参数可按网络情况去掉。
底盘镜像使用 Dockerfile 中锁定的官方 ZED 基础镜像，无需先导入我们构建的镜像。

**在 `.100` 配置到 `.50` 的免密 SSH**（已有可用密钥则跳过生成）：

```bash
ssh-keygen -t ed25519
ssh-copy-id -i ~/.ssh/id_ed25519.pub tmr-user@172.16.0.50
ssh -o BatchMode=yes tmr-user@172.16.0.50 docker version
```

将用户/IP 替换为 `hosts.base.ssh` 的值，首次连接时核对主机指纹。
密钥必须支持容器无人值守使用；默认流程不转发 SSH agent、不提示输入密钥口令。

**两端构建成功，按上表填写 `.100` 的 YAML 和准备相机标定后，在 `.100` 启动：**

```bash
export EBIM_IMAGE=franka-duo-table-mission:phase2
bash scripts/docker_hardware.sh plan
bash scripts/docker_hardware.sh check
bash scripts/docker_hardware.sh up --activate
bash scripts/docker_hardware.sh status
bash scripts/docker_hardware.sh mission --execute
# 任务结束后有序关闭两端受管理的驱动：
bash scripts/docker_hardware.sh down
```

`.100` 的 `~/.ssh` 会只读挂入容器，需包含可无人值守使用的私钥和已确认的
`.50` 主机指纹 `known_hosts`；容器不会转发本机 SSH agent 或提示输入密码。
非默认目录用 `EBIM_SSH_DIR=/绝对路径` 指定。

所有日常命令都在 **`.100` 本地执行**，只对 `.50` 使用 SSH。
`up --activate` 自动启动两端驱动、部署路线并激活阻抗控制；
`mission --execute` 才执行移动和抓取。已有原生驱动会报冲突，需先处理。
从已停止的状态一键启动并执行可用 `bash scripts/docker_hardware.sh run --execute`；
不要在已经 `up` 后重复运行它。Pixi 可选，同等命令为 `pixi run run --execute`。

---

以下为构建方法、验证说明、原生部署参考及任务细节。使用上面的容器流程时，
不需要执行原生部署章节。

历史测试使用过底盘镜像 `db0f066` 和机械臂镜像 `a33d17b`，仅作为验证记录，
不作为目标机器的部署输入；目标两端均按上文从当前源码构建 `:phase2` 镜像。
镜像离线检查及两机合成图像 DDS 测试已通过，仍有 DDS 警告待排查；
真实相机采集及整机运动尚未完成容器验证。
详见 [构建与验证记录](docs/HARDWARE_STARTUP.md#build-and-validation-status)。

## 1. Hardware assumptions

| Item | Value |
| --- | --- |
| Platform | Franka Duo Mobile: 2x FR3 (`fr3v2`) on `franka_spine_v0_1`, TMR swerve base |
| Arms | Namespaces `/left` and `/right` |
| Grippers | 2x Robotiq 2F-85 (`0.0` open ... `0.8` closed) |
| Spine | Prismatic `franka_spine_vertical_joint`, range `[0.0, 0.85] m` |
| Head camera | ZED Mini, rectified RGB 640x360. **Depth is not used.** |
| Navigation sensors | Front and rear SICK safety laser scanners supported by `sick_safetyscanners2`, with the original mounting transforms |
| Computers | AMD64 arm host (`.100`) and Jetson Orin ARM64 base/camera host (`.50`); these addresses are configurable |
| Wrist cameras | Not used |
| GPU | Arm perception runs `yolo11n-seg` on CPU; ZED uses the base Jetson GPU through NVIDIA Container Runtime |

Both arms are mounted on the spine carriage (`fr3_duo
base_mount="franka_spine_mounting_point"`), so **moving the spine moves both
arms**. The pipeline therefore raises the spine to 0.700 m before every drive
and lowers it to 0.468 m on arrival.

Control rates: the policy publishes 20-D action chunks at `30 Hz x
playback_speed` (default `0.1`, i.e. 3 Hz). The hardware startup workflow runs
the 1 kHz joint tracker, impedance controllers and arm drivers in the same
container on the arm host.

### Cell assumptions

- Three tables at the **same** top height. Default `--table-z -0.220 m` in the
  *midpoint base frame* (Section 6). Override with `--letter-table-z` /
  `--placement-table-z` if they differ.
- Pick table: a tray with a **cup**, a **bowl containing dark beans**, and a
  **plate** as distractor. The bowl is told from the plate by the dark-pixel
  fraction inside its mask; its grasp point is the rim point opposite the
  spoon handle.
- Object geometry (overridable): cup 8 cm tall, bowl 4.5 cm tall, bowl rim
  11.5 cm diameter, tray 1 cm thick.
- Arms start stowed inside the base envelope (front/rear 0.40 m, width
  0.58 m). The shipped `travel_stow` posture satisfies this (Section 7).
- Ordinary indoor lighting. A segmentation network is used instead of colour
  thresholds because thresholds failed on shadows and the pale-green arm bodies.

### Reset between rounds

1. Return the base to its marked start pose and heading.
2. Put cup, bowl and plate back on the tray on the pick table.
3. Clear the letter-side and placement tables.
4. Open both grippers.
5. After confirming the physical reset, archive/clear the mission checkpoint.
   Managed deployment stores it at `/app/outputs/table_mission.json` in the arm
   container's persistent outputs volume, not the checkout's `outputs/`.
   Removing/recreating the runtime does not clear it. The standalone mission CLI
   also accepts `--fresh-start-confirmed`; the managed launcher does not forward
   this flag. The checkpoint refuses to replay a drive that already happened.

---

## 2. Software inventory and legacy host installation

The default hardware workflow packages both machines' software in separate
images and deploys the bundled base routes automatically. The inventory in Section 2.1
and manual commands in Sections 3–4 describe the legacy native installation;
they are not additional setup steps for the container workflow. Current host
requirements and commands are in [Hardware startup](docs/HARDWARE_STARTUP.md).

### 2.1 What is ours, and where it goes

| Tree in this repo | Deploy to | What it is |
| --- | --- | --- |
| `hosts/arm/teleoperation_overlay/` | arm host, colcon workspace overlaying `franka_ros2` | **Our arm-side ROS 2 overlay.** Contains `franka_fr3_arm_controllers` — despite its name this is where the **`JointImpedanceController`** lives (it subscribes `/{left,right}/gello/joint_states`, which our servo relay publishes). Carries our modifications: a `k_alpha` filter parameter in the controller, per-arm `franka_robot_state_broadcaster` parameters in `franka.launch.py`, and the `controllers.yaml` robot-state fix. Also `franka_gripper_manager` (Robotiq), `franka_gello_state_publisher`, `franka_spine_msgs`, and the pedal/keyboard teleop bridges (unused by this policy). |
| `site/franka_duo_joint_servo/` | arm host, colcon | **Our joint servo**: 20-D chunks → KDL IK → 1 kHz Ruckig tracker → gello relay → impedance controller. |
| `site/franka_duo_ptp_step/` | arm host, colcon | Our homing utility. Not on the mission path. |
| `src/franka_duo_tele_data/` | this container | The policy. |
| `base/tmr_base/` | base host, `~/tmr_base` | **Our navigation routes** (07 outbound, 13 post-grasp, 15 return, 20 placement detour), the exclusive velocity adapter, and live table-leg detection. |
| `base/tmr_navigation/` | base host, colcon | **Our local navigation adapter package**: odom frame adapter, dual-LiDAR merger, SLAM launch. |
| `hosts/arm/env/` | arm host `~/` | `tmr_env.sh` + `source_migrated_stack.sh` (sources Jazzy → `franka_ros2` → our overlay) and `cyclonedds.xml` (binds DDS to the arm host's interface). |
| `hosts/base/home/` | base host `~/` | `start_tmr_sensors.sh`, `zed_override.yaml`, `zed_relaunch.sh`, `cyclonedds.xml`. |
| `hosts/base/vendor_patches/` | apply to vendor checkouts on the base host | Two small diffs against upstream: `franka_bringup/config/tmr.config.yaml` (`use_rviz: false`) and `zed-ros2-wrapper` configs (30 Hz publish, `depth_mode: NONE`). |

Upstream vendor packages used by the legacy installation are listed below.
The images fetch dependencies at the exact revisions in
`docker/drivers.lock.json` (arm) and `docker/base_drivers.lock.json` (base)
during build and retain their sources/licenses.

| Package | Host | Origin | Revision |
| --- | --- | --- | --- |
| `franka_ros2` (incl. `franka_bringup`, `franka_mobile_sensors`, `franka_spine_server`, `franka_description`) | both | github.com/frankarobotics/franka_ros2 | base host: `1006036` (humble); arm host: Jazzy workspace |
| `libfranka` | both | github.com/frankarobotics/libfranka | `95b406f4` |
| `ros2_robotiq_gripper` | arm | github.com/PickNikRobotics/ros2_robotiq_gripper | `a29c69b` (built against Jazzy in the arm image) |
| `zed-ros2-wrapper` | base | github.com/stereolabs/zed-ros2-wrapper | `458c725` + our patch |
| `sick_safetyscanners2` | base | github.com/SICKAG/sick_safetyscanners2 | `886a81a` |
| `olvx_descriptions_module` | base | github.com/olive-robotics/olvx_descriptions_module | `c3444ed` |
| `gello_software` | both | github.com/wuphilipp/gello_software | `fa0407b` (our overlay forked from this) |

### 2.2 Container deployment requirements

The following dependencies must exist outside the images:

| Location | Required hardware / host setup |
| --- | --- |
| Robot | Dual FR3v2, compatible robot firmware/FCI enabled, original Spine/TMR swerve system, two Robotiq 2F-85 grippers, two SICK scanners and ZED Mini; original model, TCPs and mounting geometry |
| Arm host (`.100`) | AMD64 Linux with Docker Engine and Bash, suitable real-time kernel/scheduling and network setup for 1 kHz Franka FCI, access to both grippers' USB-RS485 serial devices; no GPU required |
| Base host (`.50`) | Jetson Orin ARM64 with matching JetPack/L4T R36.4 kernel and NVIDIA drivers, Docker Engine with `nvidia` runtime, SSH server and Python 3 standard library; ZED USB 3/video devices available |
| Network | Host interface addresses and routes to arms, Spine/base and scanners; scanner receiving IP assigned to the base host; DDS UDP/multicast between the two computers, SSH from arm to base, synchronized clocks |
| Access | Local arm user can run Docker; the base SSH user can run Docker without interactive sudo; an unattended SSH key and verified `known_hosts` available inside the arm container |
| Calibration | Matching `SN<serial>.conf` ZED factory intrinsics on `.50`; validated camera-to-robot extrinsics at `camera.calibration` on `.100` |
| Artifacts / storage | Complete AMD64 arm and ARM64 base images loaded on their respective hosts, this checkout on `.100` for the launcher/YAML/calibration, and space for images, build cache and persistent logs |

The inspected Jetson runs Ubuntu 22.04.5 / L4T R36.4.0. The base image supplies
CUDA 12.6 user-space libraries and ZED SDK 5.1.2. Installing generic Ubuntu
24.04 alone does not provide Jetson's matching kernel/drivers or Franka real-time
setup: containers share the host kernel. Keep the already configured system
baseline when deploying to an identical robot.

The recorded arm image is about 6.13 GB and the base image 12.54 GB (shared
layers affect actual disk use). Building and retaining import archives needs
additional space; these image sizes are not build-space requirements.

The arm container carries the policy, Python dependencies, Franka/Robotiq drivers,
custom impedance controllers, Spine server, joint servo and relay. The default
hardware workflow starts them locally in this container. The base image carries
Humble, TMR/swerve drivers, SICK, ZED SDK/wrapper, SLAM and navigation adapters.

The image also builds the joint servo, relay and Spine message interfaces.
The servo loads the exported default Duo model from `.100`, including both
KDL configurations and all referenced meshes. It no longer needs the host's
`franka_mobile_fr3_duo_moveit_config` package or a `site/install` bind mount.
MoveIt/KDL supply the robot model and IK inside the image; Ruckig tracks joint
targets. No `move_group` motion-planning server is started. The hardware remains
under joint impedance control, and no host kinematics installation is needed.
The default `mission` mode still expects a running servo/relay and active
impedance controllers; use the hardware workflow to start the complete stack.
See [bundled model provenance and validation](site/franka_duo_joint_servo/MODEL.md).

| Requirement | Notes |
| --- | --- |
| ROS 2 **Jazzy**, CycloneDDS | Installed inside the image |
| Franka FR3 arm drivers | Built inside the image, one per arm namespace |
| `JointImpedanceController` | Built inside the image; consumes `/{left,right}/gello/joint_states` |
| Robotiq gripper drivers | Built inside the image; serial paths come from YAML |
| ZED wrapper and SDK | Inside the base image; ROS rectified RGB + `camera_info`, no image HTTP server |
| `franka_spine_server` | Built inside the image; `/franka_spine_node/*` services and action |
| ROS 2 **Humble** | Inside the base image; navigation uses a separate DDS domain |
| Docker | Both images use `--network host`; `.50` uses NVIDIA runtime for ZED |

Default `managed` mode starts the required ROS publishers/services using the
drivers inside the images; pre-existing ROS topics, native ROS/MoveIt/vendor
workspaces and an image HTTP server are not deployment prerequisites.
`external` mode instead requires already-running compatible interfaces; it
does not adapt arbitrary topic names. Existing unmanaged drivers must be
reconciled before managed startup; the launcher reports conflicts.

**Network at run time:** no internet needed after both images, SSH access and
calibration are prepared. Weights, models, default routes, taught poses and the
action contract are bundled. The launcher mounts the YAML-selected policy
calibration from the arm host. Both containers use **host networking**; this
does not configure host interfaces/routes, supply a missing RT kernel or
install NVIDIA drivers. No `--privileged` or Docker socket mount is required.

### Required ROS 2 interfaces

Subscribed:

```
/left/franka_robot_state_broadcaster/current_pose    geometry_msgs/PoseStamped
/right/franka_robot_state_broadcaster/current_pose   geometry_msgs/PoseStamped
/left/gripper/joint_states                           sensor_msgs/JointState
/right/gripper/joint_states                          sensor_msgs/JointState
/head_camera/zed/rgb/color/rect/image                sensor_msgs/Image
/head_camera/zed/rgb/color/rect/camera_info          sensor_msgs/CameraInfo
/franka_duo/joint_servo/status                       std_msgs/String
```

Published:

```
/franka_duo/joint_servo/action_chunk                 std_msgs/Float32MultiArray
```

Services / actions called:

```
/franka_spine_node/switch_on                         franka_spine_msgs/srv/SwitchOn
/franka_spine_node/get_position                      franka_spine_msgs/srv/GetPosition
/franka_spine_node/move_absolute                     franka_spine_msgs/action/MoveAbsolute
/{left,right}/controller_manager/list_controllers    controller_manager_msgs/srv/ListControllers
/franka_duo_joint_servo/get_parameters               rcl_interfaces/srv/GetParameters
```

These are runtime interfaces created by managed startup, not topics the host
must publish in advance. Standalone policy settings include topic options in
`configs/tmr_rgb20d.yaml`; managed startup expects the canonical interfaces
listed here.

---

## 3. Two-host layout

```
 ARM HOST .100                        BASE HOST .50 (Jetson)
 Jazzy container                 --ssh-> Docker / Humble container
 perception, spine, arms                  routes, swerve, LiDAR, SLAM (domain 97)
 domain 0                       <--DDS-- ZED image + camera_info (domain 0)
```

The arm and base-control ROS domains stay separate. ZED publishes in the arm
domain, while navigation runs in the base domain. Routes run through SSH and
`docker exec` with connection heartbeats; each route returns a structured JSON
report. Every base leg takes the same `flock`, so two routes cannot own the
velocity channel at once. The default hardware workflow handles deployment
and startup; no manual copying or host workspace build is needed.

The following instructions describe the legacy native deployment only.

Set `--base-host user@host` and `--base-root /path/to/base/routes` for the
testbed. Passwordless SSH from the container to the base host is required:
mount a key with `-v ~/.ssh:/root/.ssh:ro`.

The base routes (`07_start_to_pickup.py`, `13_post_grasp_route.py`,
`15_return_from_letter.py`, `20_after_return_placement.py`) are shipped in
**`base/tmr_base/`**, together with the local navigation adapter package in
**`base/tmr_navigation/`**. They run on the base host under ROS 2 Humble, not in
this container. Deploy them once:

```bash
# From the checkout, copy our trees and home scripts to the base host.
rsync -a base/tmr_base/       <user>@<base-host>:~/tmr_base/
rsync -a base/tmr_navigation/  <user>@<base-host>:~/tmr_navigation/
rsync -a hosts/base/home/      <user>@<base-host>:~/
rsync -a hosts/base/vendor_patches/ <user>@<base-host>:~/vendor_patches/

# On the base host: apply the two vendor patches, build the adapter package.
cd ~/ros2_ws/src/franka_ros2    && git apply ~/vendor_patches/franka_bringup_tmr_config.patch
cd ~/ros2_ws/src/zed-ros2-wrapper && git apply ~/vendor_patches/zed_ros2_wrapper.patch
cd ~/tmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

The mission calls the routes by name under `--base-root` (default
`/home/tmr-user/tmr_base`). Two site-specific changes are already applied in
this copy: the step-20 detour shifts left **0.85 m**, and its table-leg ROI is
referenced to the pose the base is standing in when the detour starts, so no
saved START capture is needed.

---

## 4. Legacy native bring-up order

Skip this section when using `scripts/docker_hardware.sh` / Pixi. Do not run
these drivers alongside the managed container.

### Base host

```bash
cd ~/tmr_base            # the deployed copy of base/tmr_base (Section 3)
bash scripts/19_ensure_navigation_stack.sh   # controller, odometry, dual LiDAR, SLAM, velocity adapter
bash scripts/17_control_mode.sh mission      # take the exclusive mission velocity lease
```

Confirm `/swerve_drive_controller/odom`, `/lidar_front/scan`,
`/lidar_rear/scan`, `/map` publish and exactly **one** `cmd_vel_adapter.py`
runs. Note the stack runs with `ROS_LOCALHOST_ONLY=1`; probe it with the same
setting.

For the legacy native ZED wrapper, use a separate terminal on the **base host**
with its Humble workspace. The camera uses the arm DDS domain, unlike navigation:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
# Verify the DDS interface address and replace the serial for another camera.
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedm namespace:=head_camera publish_tf:=false serial_number:=17064700
```

### Arm host

```bash
# 0. One-time: deploy our overlay and env, then build it on top of franka_ros2.
rsync -a hosts/arm/teleoperation_overlay/ <user>@<arm-host>:~/recloned_sources/teleoperation_overlay/
rsync -a hosts/arm/env/tmr_env.sh hosts/arm/env/cyclonedds.xml <user>@<arm-host>:~/
rsync -a hosts/arm/env/source_migrated_stack.sh <user>@<arm-host>:~/recloned_sources/
# on the arm host (edit the address in ~/cyclonedds.xml to the host's own interface):
cd ~/recloned_sources/teleoperation_overlay && source ~/recloned_sources/franka_ros2_jazzy_ws/install/setup.bash \
  && colcon build --symlink-install
source ~/tmr_env.sh

# 1. Arm drivers, one per arm (skip if running). This launch file is ours.
ros2 launch franka_fr3_arm_controllers franka.launch.py \
  arm_id:=fr3v2 arm_prefix:=left namespace:=left robot_ip:=<LEFT_IP> \
  load_gripper:=false joint_sources:=joint_states
# same for arm_prefix:=right namespace:=right robot_ip:=<RIGHT_IP>

# 2. Grippers (ours) and spine server (upstream franka_ros2) on the arm host.
ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
  config_file:=example_fr3_duo_config_robotiq.yaml
ros2 launch franka_spine_server spine.launch.py spine_ip:=<SPINE_IP>
# The ZED wrapper runs separately on the base host as shown above.

# 3. Build our joint servo packages from this checkout.
colcon build --base-paths site --build-base site/build \
  --install-base site/install --merge-install \
  --packages-select franka_duo_joint_servo franka_duo_ptp_step
source site/install/setup.bash

# 4. Joint servo + gello relay, then activate both impedance controllers.
bash scripts/start_servo_and_activate.sh 0.1 --reconfigure
```

`start_servo_and_activate.sh` prints `READY:` when both impedance controllers
are active and the servo has latched. It refuses to restart the servo while a
controller is active (Section 8).

If `current_pose` is silent (controllers `inactive` after a Desk mode change):

```bash
bash scripts/reconnect_arm_state.sh both
```

---

## 5. Container deployment

### 5.1 Build or load both images

Build the current checkout on each matching architecture. The arm and base
are **separate images**. `plan/check/up/run` never builds or pulls either one.
On the AMD64 builder (for example `.100`), from the repository root:

```bash
docker build --network host --platform linux/amd64 \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  --build-arg PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
  -t franka-duo-table-mission:phase2 .
```

On the matching Jetson ARM64 builder (for example `.50`), with the same checkout:

```bash
docker build --network host --platform linux/arm64 -f docker/base.Dockerfile \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.ustc.edu.cn/ubuntu-ports/ \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  --build-arg GIT_PROXY=https://gh-proxy.org \
  -t franka-duo-base:phase2 .
```

The mirror/proxy arguments are optional and only affect the build. ARM64
Ubuntu packages require `ubuntu-ports`. The base Dockerfile defaults to the
pinned ZED image digest. When using a previously imported ZED base, see the
[offline base-image instructions](docs/HARDWARE_STARTUP.md#build-and-validation-status)
for image-ID verification and the `BASE_IMAGE` override.

Alternatively, export completed images on their builders and transfer the
archives to the corresponding hosts. These archives must contain the **complete
application images**; importing `stereolabs/zed` alone still requires a base build.

```bash
# On the respective builders:
docker save -o arm-image.tar franka-duo-table-mission:phase2
docker save -o base-image.tar franka-duo-base:phase2
# After transferring arm-image.tar to .100, run there:
docker load -i arm-image.tar
# After transferring base-image.tar to .50, run there:
docker load -i base-image.tar
```

An image built directly on its target needs no export/import. The deployment
above builds both images from the current source. Historical image tags are
validation records only; the old `a33d17b` arm image lacks the route rename and
closed-gripper test-placement update.

### 5.2 Configure hardware and SSH

On `.100`, edit `hardware.yaml` in this checkout:

| YAML field | Set to |
| --- | --- |
| `image` | Loaded AMD64 arm image tag; also set `EBIM_IMAGE` if different from `franka-duo-table-mission:phase2` |
| `hosts.base.image`, `hosts.base.ssh` | Loaded ARM64 base image tag and `.50` SSH user/address |
| `hosts.arm.dds_address`, `hosts.base.dds_address` | Actual local DDS interface IP on each host |
| `arms.left_ip`, `arms.right_ip`, `spine.ip`, `base.ip` | Hardware controller IPs |
| `grippers.left_port`, `grippers.right_port` | Actual `/dev/serial/by-id/...` paths on `.100`, assigned to the correct arm |
| `lidars.front_ip`, `lidars.rear_ip`, `lidars.host_ip` | Scanner addresses and receiving interface IP on `.50` |
| `camera.serial`, `camera.sdk_settings_dir` | ZED serial and directory on `.50` containing its `SN<serial>.conf` |
| `camera.calibration` | Policy camera-to-robot extrinsics file on `.100`, relative to YAML or absolute |
| `domains`, `runtime.playback_speed` | DDS domains (default arm/camera 0, navigation 97) and consistent policy/servo speed (default 0.1) |

Keep component modes `managed` for full startup. IPs/serials do not describe
changed geometry: verify mounting transforms, extrinsics and the taught routes
for the destination cell. Routes are already bundled; no `~/tmr_base` or native
ROS workspace needs to exist on either host. The supplied factory intrinsics
are for ZED serial `17064700`; a different serial needs its own factory file.

Prepare key-based SSH from the arm container to `.50`. The launcher mounts
`$HOME/.ssh` read-only as `/root/.ssh`; `EBIM_SSH_DIR` selects another absolute
directory. Include a usable unattended key and verified `known_hosts` entry.
Key/config paths must be valid inside the container; the launcher does not
forward a host SSH agent or prompt for passwords. Do not put credentials in YAML.

```bash
# On .100, from the checkout root. Keep this equal to hardware.yaml's image.
export EBIM_IMAGE=franka-duo-table-mission:phase2

# Test SSH and Docker access from the same container environment.
# Replace this user/address if hosts.base.ssh differs.
docker run --rm --pull never --network host \
  --mount "type=bind,src=${EBIM_SSH_DIR:-$HOME/.ssh},dst=/root/.ssh,readonly" \
  --entrypoint ssh "$EBIM_IMAGE" \
  -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=8 \
  tmr-user@172.16.0.50 docker version
```

### 5.3 Check, start and execute on .100

All commands below run **locally on the target arm host**, from this checkout.
They use SSH only for the separate base host. With the two images and hardware
configuration prepared:

```bash
# Offline arm software/model checks; no devices or network.
docker run --rm --pull never --network none "$EBIM_IMAGE" selftest
docker run --rm --pull never --network none "$EBIM_IMAGE" model-check

# Print configuration and startup plan; no host connections.
bash scripts/docker_hardware.sh plan
# Inspect dependencies, devices, base image and SSH; starts no drivers.
bash scripts/docker_hardware.sh check
# Start drivers/servo, align targets and activate impedance; no mission yet.
bash scripts/docker_hardware.sh up --activate
bash scripts/docker_hardware.sh status
# Print the mission plan using the running runtime's configuration.
bash scripts/docker_hardware.sh mission
# Execute physical base, Spine, arm and gripper actions.
bash scripts/docker_hardware.sh mission --execute
# After the mission: deactivate controllers and shut down managed drivers.
bash scripts/docker_hardware.sh down
```

For subsequent rounds, after reset and with the managed runtime stopped, startup
plus the physical mission can be invoked with one command:

```bash
bash scripts/docker_hardware.sh run --execute
# The runtime stays alive after the mission; shut it down explicitly.
bash scripts/docker_hardware.sh down
```

`up --activate` starts two containers, deploys routes into `.50`'s persistent
volume, starts the managed drivers and checks live interfaces before reporting
ready. It can activate hardware and is not an offline test. `run --execute`
includes this startup; do not run it after a separate `up`. On failures use
`status` / `logs`; existing runtimes are retained for inspection. Use `down`
for orderly shutdown before restarting or changing YAML.

`plan/check/up/run` accept `--hardware /absolute/path/hardware.yaml`.
`status/mission/down` use the running container's configuration snapshot.
`EBIM_CONTAINER` overrides the default arm container `ebim-cup-bowl-runtime`.

### 5.4 Optional Pixi and logs

Pixi wraps the same Bash launcher. Prepare its environment before an offline
deployment; it is needed only on `.100`, not inside the images or on `.50`.

```bash
pixi install
pixi run plan                 # Local Python plan; does not require an image.
pixi run check
pixi run up --activate
pixi run status
pixi run mission --execute
pixi run down
# Alternatively, from a stopped runtime: pixi run run --execute
```

With the default arm container name:

```bash
bash scripts/docker_hardware.sh logs
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/arm.log
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/servo.log
docker cp ebim-cup-bowl-runtime:/app/outputs ./mission-outputs
```

State, logs and mission checkpoints persist in Docker volumes across `down`.
Export outputs before removing the arm container. Follow the reset-between-rounds
procedure before replaying a completed mission. Standalone `mission/grasp/spine`
entrypoints assume the necessary drivers are already running; they are not
replacements for managed hardware startup.

---

## 6. What the mission does

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

## 7. Configuration

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

## 8. Safety and operational notes

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

## 9. Repository layout

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

## 10. Declarations

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
