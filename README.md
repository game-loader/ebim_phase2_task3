# Franka Duo Mobile — Cup / Bowl Mobile Pick-and-Place

Mobile manipulation policy for a Franka Duo Mobile (dual FR3 on a lifting
spine, TMR swerve base). The robot drives to a table, finds a cup and a bowl
with the head camera, picks them with both arms, carries them to a second
table, sets them down and picks them straight back up, drives back, and finally
leaves them on a third table.

**All arm motion runs through joint impedance control.** No PTP or joint
position motion generator is used anywhere in the pipeline. The Franka
Cartesian/joint-position generators reflex on a single discontinuous sample;
the impedance controller does not, and the demonstrations this policy was tuned
against were recorded under it.

---

## 1. Hardware assumptions

| Item | Value |
| --- | --- |
| Platform | Franka Duo Mobile: 2x FR3 (`fr3v2`) on `franka_spine_v0_1`, TMR swerve base |
| Arms | Namespaces `/left` and `/right` |
| Grippers | 2x Robotiq 2F-85 (`0.0` open ... `0.8` closed) |
| Spine | Prismatic `franka_spine_vertical_joint`, range `[0.0, 0.85] m` |
| Head camera | ZED Mini, rectified RGB 640x360. **Depth is not used.** |
| Wrist cameras | Not used |
| GPU | **None required.** `yolo11n-seg` on CPU, ~15 ms/frame after warm-up |

Both arms are mounted on the spine carriage (`fr3_duo
base_mount="franka_spine_mounting_point"`), so **moving the spine moves both
arms**. The pipeline therefore raises the spine to 0.700 m before every drive
and lowers it to 0.468 m on arrival.

Control rates: the policy publishes 20-D action chunks at `30 Hz x
playback_speed` (default `0.1`, i.e. 3 Hz). The 1 kHz joint tracking and
impedance loops run in host-side ROS 2 nodes, not in this container.

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
5. Delete `outputs/table_mission.json` or pass `--fresh-start-confirmed`; the
   checkpoint refuses to replay a drive that already happened.

---

## 2. Software prerequisites on the host

The container carries the policy and its Python dependencies. It does **not**
carry robot drivers; those must already run on the host, because the policy is
a ROS 2 participant talking to them.

| Requirement | Notes |
| --- | --- |
| ROS 2 **Jazzy** on the arm host | Image is `ros:jazzy-ros-base`; RMW must match |
| `rmw_cyclonedds_cpp` | Default in the image; set `RMW_IMPLEMENTATION` to match the host |
| Franka FR3 arm drivers | `franka_fr3_arm_controllers`, one per arm namespace |
| `JointImpedanceController` | Site controller consuming `/{left,right}/gello/joint_states` |
| Robotiq gripper drivers | Publishing `/{left,right}/gripper/joint_states` |
| ZED wrapper | Publishing rectified head RGB + `camera_info` |
| `franka_spine_server` | Provides `/franka_spine_node/*` services and action |
| ROS 2 **Humble** on the base host | Navigation stack, separate DDS domain |
| Docker | With `--network host`; **no GPU runtime** |

**Network at run time:** no internet needed. Weights, extrinsics and the action
contract are baked into the image. The container does need **host networking**
for DDS and SSH to the base host.

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

Topic names are configurable in `configs/tmr_rgb20d.yaml`.

---

## 3. Two-host layout

```
 ARM HOST (ROS 2 Jazzy, domain 0)        BASE HOST (ROS 2 Humble, domain 97)
 this container                  --ssh->  navigation routes
 perception, spine, arms                  swerve controller, LiDAR, SLAM
```

The two ROS graphs are **never merged**. The base is driven over SSH and judged
only by the structured JSON report each route prints. Every base leg takes the
same `flock`, so two routes can never own the velocity channel at once.

Set `--base-host user@host` and `--base-root /path/to/base/routes` for the
testbed. Passwordless SSH from the container to the base host is required:
mount a key with `-v ~/.ssh:/root/.ssh:ro`.

The base routes (`07_start_to_pickup.py`, `13_post_grasp_route.py`,
`15_return_from_letter.py`, `20_after_return_placement.py`) live on the base
host in the `tmr_cycle` tree and are called by name; they are not part of this
image.

---

## 4. Bring-up order

### Base host

```bash
cd <base-routes>
bash scripts/19_ensure_navigation_stack.sh   # controller, odometry, dual LiDAR, SLAM, velocity adapter
bash scripts/17_control_mode.sh mission      # take the exclusive mission velocity lease
```

Confirm `/swerve_drive_controller/odom`, `/lidar_front/scan`,
`/lidar_rear/scan`, `/map` publish and exactly **one** `cmd_vel_adapter.py`
runs. Note the stack runs with `ROS_LOCALHOST_ONLY=1`; probe it with the same
setting.

### Arm host

```bash
# 1. Arm drivers, one per arm (skip if running).
ros2 launch franka_fr3_arm_controllers franka.launch.py \
  arm_id:=fr3v2 arm_prefix:=left namespace:=left robot_ip:=<LEFT_IP> \
  load_gripper:=false joint_sources:=joint_states
# same for arm_prefix:=right namespace:=right robot_ip:=<RIGHT_IP>

# 2. Robotiq grippers, ZED head camera, spine action server (site launch files).

# 3. Build the site ROS 2 packages shipped here.
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

## 5. Build and run

```bash
# Build
docker build -t franka-duo-table-mission:phase2 .

# Offline self-check: no robot, no network.
docker run --rm franka-duo-table-mission:phase2 selftest

# Dry run: prints the 12-step plan, starts nothing.
docker run --rm --network host franka-duo-table-mission:phase2 mission \
  --base-host <user>@<base-host> --base-root <base-routes-path>

# Full mission. Drives the base and both arms.
docker run --rm --network host \
  -v ~/.ssh:/root/.ssh:ro \
  -v "$PWD/site/install:/app/site/install:ro" \
  -v "$PWD/outputs:/app/outputs" \
  franka-duo-table-mission:phase2 mission \
    --base-host <user>@<base-host> --base-root <base-routes-path> \
    --speed 0.1 --execute
```

`--speed` **must** equal the servo `playback_speed`; the policy checks this and
refuses to publish otherwise. If your ROS domain or RMW differ, pass
`-e ROS_DOMAIN_ID=... -e RMW_IMPLEMENTATION=...`.

### Staged bring-up (recommended first)

```bash
# Spine only.
docker run --rm --network host franka-duo-table-mission:phase2 spine --target-m 0.468 --execute

# Stow the arms only.
docker run --rm --network host -v "$PWD/site/install:/app/site/install:ro" \
  franka-duo-table-mission:phase2 grasp --pose-only travel_stow --publish --enable-robot

# Perception only from a saved frame; never moves the robot.
docker run --rm -v "$PWD/frame.jpg:/frame.jpg:ro" \
  franka-duo-table-mission:phase2 grasp --image /frame.jpg

# Grasp both objects and hold; skip every placement leg.
docker run --rm --network host -v ~/.ssh:/root/.ssh:ro \
  -v "$PWD/site/install:/app/site/install:ro" \
  franka-duo-table-mission:phase2 mission \
    --base-host <user>@<base-host> --base-root <base-routes-path> \
    --stop-after-grasp --execute
```

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
 -> TEST_PLACE_RUNNING           touch down, open, close again, lift; keep them
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

**Placement is the grasp in reverse.** `--mode test` descends, opens,
**closes again** and lifts. `--mode final` descends, opens and lifts away
empty. X/Y always come from the arm's current pose, never the earlier grasp
point; the base has driven elsewhere by then.

---

## 7. Configuration

| File | Purpose |
| --- | --- |
| `configs/tmr_rgb20d.yaml` | Topic names, workspace bounds, step limits, staleness |
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
scripts/                    operator bring-up scripts
configs/, assets/           configuration, extrinsics, weights, contract
tests/                      66 offline tests, no robot required
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
