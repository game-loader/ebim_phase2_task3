# Hamburg deployment

Run the task on the **Linux x86 station** using `docker/hamburg.Dockerfile` and
`hardware.hamburg.yaml`. The companion already runs the complete ROS 2 Humble
hardware stack. The task attaches over DDS domain **0** using host networking.
No companion-side deployment, SSH credentials, serial devices, GPU, ZED SDK,
hardware driver startup or separate base container is required.

The station needs Bash, Docker, DDS connectivity to the companion (including
UDP discovery/multicast) and synchronized clocks. ROS is included in the image.
On a station with multiple network interfaces, set `hosts.arm.dds_address` to
the **station's** IP on the robot LAN. `auto` uses Cyclone DDS interface selection.

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
| Controller status | `/{left,right}/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers`, read only; availability pending organizer confirmation |

Arm joint names are `{left,right}_fr3v2_joint1` through `joint7`, with commands
ordered from joint 1 to 7. Measured poses are computed from these joint states
using the bundled FR3v2 model and published internally at
`/franka_duo/measured/{left,right}_pose` (link8 expressed in that arm's link0).
The organizer does not need to publish `current_pose` or `FrankaRobotState`.

The supplied MoveAbsolute example confirms the goal fields `position`,
`velocity`, `acceleration`, `deceleration`. The bundled definition uses metres,
metres/second and metres/second squared. Its result is `success`, `stop_by`,
`error`, and feedback is `current_position`. GetPosition has an empty request
and returns `float64 position`, `bool success`. Confirm these complete wire
definitions with the organizer. External mode never calls `switch_on`.

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

The organizer's existing continuous command publishers must be handed over.
The proposed sequence below requires organizer confirmation before a real run.
If the testbed uses a command multiplexer, obtain its switching procedure and
adapt the handoff to that interface first.

1. Organizer deactivates both `joint_impedance_controller` instances, then
   stops the previous arm command publishers. Also release any publishers on
   the gripper command topics and `/swerve_drive_controller/cmd_vel`.
2. Start the task runtime. It checks inactive arm controllers and vacant command
   topics before starting the servo, target relay and base velocity adapter:

   ```bash
   bash scripts/docker_hardware.sh up --hardware hardware.hamburg.yaml
   bash scripts/docker_hardware.sh status
   ```

3. Organizer verifies that joint targets match measured positions, then
   activates both impedance controllers. The task must remain stationary
   during handoff. The servo initially follows measured joints for up to
   60 seconds, then latches its idle target; if activation is delayed, verify
   alignment again. The mission checks alignment and active controller state.
4. Run a dry plan, then explicitly execute the mission:

   ```bash
   bash scripts/docker_hardware.sh mission
   bash scripts/docker_hardware.sh mission --execute
   ```

5. After the mission, organizer deactivates both impedance controllers before
   shutting down the task:

   ```bash
   bash scripts/docker_hardware.sh down
   ```

`up --activate` and the combined `run --execute` workflow are not supported for
external hardware. This image never switches organizer controllers. `down`
refuses to stop a running target stream while a command controller is active.
Do not bypass it using `docker stop` during active impedance control.

`mission`, `status` and `down` use the profile retained in the running container;
they do not require another `--hardware` argument. Logs are available using
`bash scripts/docker_hardware.sh logs` and `/app/runtime/logs/` inside the runtime.

## Camera and routes

SN13024307 uses its **live CameraInfo**. For the rectified image, a valid
projection matrix `P` supplies the intrinsics; if `P` is unset, the live `K`
matrix is used. Image and CameraInfo dimensions and frame IDs must match.
External mode refuses saved-image inference without live intrinsics and never
falls back to the bundled SN17064700 camera matrix or SDK calibration file.

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
The runtime must already own the velocity adapter and command topics.

## Offline verification

The image build runs the Python tests, C++ tests and model checks. To exercise
handoff and shutdown against a simulated Humble graph in an isolated container:

```bash
docker run --rm --platform linux/amd64 --network none \
  franka-duo-table-mission:hamburg shell -c \
  '/app/.venv/bin/python /app/tests/hamburg_ros_smoke.py'
```

The complete physical mission still requires on-site verification after the
controller handoff and custom spine interface definitions are confirmed.
