# Hamburg deployment

Run the task on the **Linux x86 station** using `docker/hamburg.Dockerfile` and
`hardware.hamburg.yaml`. The companion already runs the complete ROS 2 Humble
hardware stack. The task attaches over DDS domain **0** using host networking.
No companion-side deployment, SSH credentials, serial devices, GPU, ZED SDK,
hardware driver startup or separate base container is required.

The station needs Bash, Docker, DDS connectivity to the companion (including
UDP discovery/multicast) and synchronized clocks. ROS is included in the image.
On a station with multiple network interfaces, set `hosts.arm.dds_address` to
the **station's** IP on the robot LAN. `auto` accepts exactly one non-loopback IPv4 address; with multiple addresses,
set the robot-LAN IPv4 address explicitly. The runtime verifies that it belongs
to the station before starting DDS. The attachment
`configs/hamburg/hamburg_fastdds_profile.xml` is the authoritative template: the
runtime replaces its station address, retains loopback, uses UDP-only Fast DDS
and native multicast discovery, with no discovery server. It preserves
`maxMessageSize=65500`, 4 MiB send and 16 MiB receive buffers. The host must allow
these socket buffer sizes. All task ROS processes use `rmw_fastrtps_cpp`.

## Interface mapping

`arms`, `grippers`, `spine`, `base`, `lidars` and `camera` are all `external`.
Both `domains.arm` and `domains.base` are `0`.

| Function | External interface | Type / semantics |
| --- | --- | --- |
| Arm commands | `/{left,right}/gello/joint_states` | `sensor_msgs/msg/JointState`, joint impedance targets |
| Arm state | `/{left,right}/franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/msg/JointState` |
| Gripper commands | `/{left,right}/gripper/gripper_client/target_gripper_width_percent` | `std_msgs/msg/Float32`, open `0.8`, closed `0.0` |
| Gripper state | `/{left,right}/gripper/joint_states` | `sensor_msgs/msg/JointState` |
| Base command | `/swerve_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| Base odometry | `/swerve_drive_controller/odom` | `nav_msgs/msg/Odometry` |
| Spine movement | `/franka_spine_node/move_absolute` | `franka_spine_msgs/action/MoveAbsolute` |
| Spine position | `/franka_spine_node/get_position` | `franka_spine_msgs/srv/GetPosition` |
| LiDARs | `/lidar_front/scan`, `/lidar_rear/scan` | `sensor_msgs/msg/LaserScan` |
| Head image | `/head_camera/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image`, rectified 640x360 `bgr8` |
| Head intrinsics | `/head_camera/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Controller status | `/{left,right}/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers`, confirmed; used by the resident gateway |

Arm joint names are `{left,right}_fr3v2_joint1` through `joint7`. Incoming
joint arrays are matched by name. Outgoing commands pair names and positions,
use RELIABLE/VOLATILE QoS and are published at **20 Hz**. The internal trajectory
servo remains at 1000 Hz. An independent relay maintains the last target on
input loss; no command gap may exceed the organizer's **0.5 s** timeout.

The mission now reads the organizer's
`/{left,right}/franka_robot_state_broadcaster/current_pose` directly as
`geometry_msgs/msg/PoseStamped`. The organizer and team confirmed that its
`base` frame means the corresponding arm's **link0**, and its payload is the
**TCP**, including +0.174 m along link8's local Z with no rotation. No offset is
added to this feedback. IK still removes that tool transform when solving for
link8. The internal `/franka_duo/measured/{left,right}_pose` FK outputs remain
available for comparison; no FR3 TF chain is required.

The organizer confirmed the complete bundled MoveAbsolute and GetPosition wire
definitions. Motion uses metres, metres/second and metres/second squared;
Hamburg's spine range is **0–0.770 m**. Mission heights 0.468 m and 0.700 m are
valid. External mode never calls `switch_on` and does not use the additional
`/spine/target_height` topic.

## Build and inspect

From the checkout root on the x86 station:

```bash
docker build --platform linux/amd64 -f docker/hamburg.Dockerfile \
  -t franka-duo-table-mission:hamburg .
export EBIM_IMAGE=franka-duo-table-mission:hamburg
bash scripts/docker_hardware.sh plan --hardware hardware.hamburg.yaml
bash scripts/docker_hardware.sh check --hardware hardware.hamburg.yaml
```

`plan` runs without network access. `check` connects to DDS, reads live state
and queries controller status; it publishes no commands, starts no hardware
drivers and does not switch controllers. Existing command publishers are
reported by `check` and are allowed at this inspection stage.

## Command handoff and mission

Start the container while the arms are out of Move mode. All task DDS
participants and endpoints are created before impedance activation: the servo,
two target relays, velocity adapter and resident ROS gateway. Subsequent phase
processes use a Unix socket to that gateway, with local callbacks and existing
ROS endpoints. They never initialize DDS. Status, readiness, controller
switching and spine action requests use the same gateway. No ROS CLI is used
for controller switching, and no gateway is recreated while target streams exist.

1. With impedance deactivated and the arms out of Move mode, the organizer
   stops existing arm, gripper and base command publishers.
   Their pause/resume interface remains unspecified; this container does not
   kill organizer processes. Occupied robot command topics block startup.
2. Start and activate through the existing controller managers:

   ```bash
   bash scripts/docker_hardware.sh up --hardware hardware.hamburg.yaml --activate
   bash scripts/docker_hardware.sh status
   ```

   The runtime deactivates impedance if needed, checks command ownership,
   configures an `unconfigured` impedance controller using
   `/{left,right}/controller_manager/configure_controller`, starts aligned
   targets, waits for the idle target latch and checks target freshness before
   activating. If configure is unavailable, the organizer must pre-configure
   both controllers as `inactive`. Both broadcasters stay active throughout.
   A partial activation failure triggers deactivation; streams remain alive.
   Without `--activate`, startup prepares the target streams and leaves
   impedance inactive for an operator-controlled handoff.
3. Run a dry plan, then explicitly execute:

   ```bash
   bash scripts/docker_hardware.sh mission
   bash scripts/docker_hardware.sh mission --execute
   ```

4. Shut down through the wrapper:

   ```bash
   bash scripts/docker_hardware.sh down
   ```

   The resident gateway first deactivates both impedance controllers and checks
   their state. Only after successful deactivation are target streams stopped.
   If deactivation fails, the container and streams remain running. If the
   gateway itself fails, organizer intervention is required; it is not restarted
   during active control. Do not bypass this sequence with `docker stop`.

`mission`, `status` and `down` use the profile retained in the running container.
`check` also reuses an existing runtime and gateway, avoiding new DDS discovery.
Before runtime startup, run `check` only with the arms out of Move mode.
Logs are available using `bash scripts/docker_hardware.sh logs` and
`/app/runtime/logs/` inside the runtime. Existing target streams remain alive
after mission errors; accepted trajectory chunks may finish before holding.

## Motion guard configuration

Hamburg's hold error and initial travel-stow transient can be accommodated in
`hardware.hamburg.yaml`, without editing Python or C++:

```yaml
runtime:
  playback_speed: 0.1
  alignment_tolerance_rad: 0.01
  max_tracking_error_rad: 0.2
  ready_timeout_s: 120
```

`alignment_tolerance_rad` limits the maximum absolute joint target/state
difference during both activation and mission readiness. Hamburg reported a
steady-state error of 0.003076 rad; the supplied profile uses 0.01 rad.
`max_tracking_error_rad` is the C++ servo's maximum joint tracking error during
motion. The supplied profile uses 0.2 rad, the lower end of Hamburg's requested
0.2–0.4 rad range; the operator can set another finite positive value after
evaluating the rig. These are thresholds, not speed controls. Errors above the
configured limits still reject alignment or latch a servo fault. Profiles that
omit these fields retain the original 0.003 and 0.15 rad defaults.

Use `playback_speed` to slow the trajectory. Reducing `max_joint_velocity_rad_s`
can reject a chunk that exceeds its velocity bound; it does not retime it.

The image must be rebuilt once after updating to this implementation. Later
threshold changes require only a YAML edit and a runtime restart. Shut down
the current runtime with `bash scripts/docker_hardware.sh down` first; ensure
the arms are out of Move mode, edit the YAML, then run:

```bash
bash scripts/docker_hardware.sh plan --hardware hardware.hamburg.yaml
bash scripts/docker_hardware.sh check --hardware hardware.hamburg.yaml
bash scripts/docker_hardware.sh up --hardware hardware.hamburg.yaml --activate
bash scripts/docker_hardware.sh mission --execute
```

`plan` shows the effective runtime values. Editing the host YAML does not change
an already running container or recover a latched servo fault.

## Camera and routes

SN13024307 uses its **live CameraInfo**. For the rectified image, a valid
projection matrix `P` supplies the intrinsics; if `P` is unset, the live `K`
matrix is used. Image and CameraInfo dimensions and frame IDs must match.
External mode refuses saved-image inference without live intrinsics and never
falls back to the bundled SN17064700 camera matrix or SDK calibration file.
Image and CameraInfo subscriptions use sensor-data BEST_EFFORT/VOLATILE QoS.

`configs/zed_pnp_calibration.json` still supplies the Shanghai camera-to-robot
extrinsics. The user confirmed identical camera mounting, table locations,
initial pose and routes, so the taught poses and route geometry are retained.

**The base container is not required.** `up` starts only the existing velocity
adapter for route execution, relaying leased `/tmr_cycle/mission_cmd_vel`
TwistStamped commands to the external `/swerve_drive_controller/cmd_vel`
interface, with a stop-on-timeout watchdog. It does not
start SLAM, navigation drivers, TF publishers, LiDARs or the base controller.
The mission starts each bundled route script locally in the same task container
as needed, under the existing heartbeat watchdog and exclusive route lock.
Routes read the external odometry and scans directly; an external map is
optional for the outbound route. No `/navigation/odom` or map/TF bring-up is
required. Existing navigation must relinquish command publishing before these
routes execute.

To run only a base leg after the task runtime is up, use the same container:

```bash
# Dry plan; no motion.
docker exec ebim-cup-bowl-runtime /app/entrypoint.sh shell -c \
  '/app/.venv/bin/python /app/scripts/hardware/local_route.py outbound'
# Physical outbound route only.
docker exec ebim-cup-bowl-runtime /app/entrypoint.sh shell -c \
  '/app/.venv/bin/python /app/scripts/hardware/local_route.py outbound --execute'
```

Available legs are `outbound`, `to-letter`, `return` and `placement`.
For `return`, specify `--run-id hamburg1`; the subsequent `placement` must use
the same ID so it can validate the return report. Each leg requires the robot
at that leg's Shanghai starting pose, arms already in their travel posture and
spine at 0.700 m. This diagnostic entry point does not reposition arms or spine.
It shares the full mission's lifecycle lock, route lock and heartbeat watchdog.
The runtime must already own the velocity adapter and command topics. Route
callbacks use the resident gateway, including latched `/tf_static` subscriptions.
Unknown ROS endpoints are rejected rather than created during a phase.

## Offline verification

The image build runs the Python tests, C++ tests and model checks. To exercise
handoff and shutdown against a simulated Humble graph in an isolated container:

```bash
docker run --rm --platform linux/amd64 --network none \
  franka-duo-table-mission:hamburg shell -c \
  '/app/.venv/bin/python /app/tests/hamburg_ros_smoke.py'
```

The complete physical mission still requires on-site verification after the
command publisher handoff is arranged. TCP transforms and spine definitions
are confirmed; the remaining organizer questions are in
[HAMBURG_QUESTIONS.md](HAMBURG_QUESTIONS.md).
