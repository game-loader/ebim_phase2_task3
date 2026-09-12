# Hardware configuration and Docker startup

The `.100` arm machine runs one container with `--network host`: Franka drivers,
custom impedance controllers, Robotiq drivers, Spine server, joint servo,
relay, perception and mission. It needs no host ROS workspace, MoveIt install,
SSH self-login or Docker socket mount. The `.50` base/camera machine remains
separate and runs its native Humble drivers; the container reaches it over
SSH and DDS. All arm motion still uses impedance control. MoveIt/KDL provide
the bundled model and IK inside the image; no `move_group` is started.

## Configure another unit

Edit `hardware.yaml` for the same TMR FR3v2 Duo hardware and mounting geometry:

- Arm host DDS address, left/right arm IPs, Spine/base IPs.
- Left/right gripper `/dev/serial/by-id/...` paths. The launcher reads these
  once and maps them to `/dev/ebim-left-gripper` and `/dev/ebim-right-gripper`.
- Base SSH address, Humble setup/overlay paths and writable runtime directory.
- ZED Mini serial/calibration, scanner IPs and receiving interface IP.
- DDS domains and playback speed (default `0.1`).

Serial/IP changes do not cover changed mounting transforms, camera intrinsics,
TCP, robot model or base kinematics. Recalibrate a replacement camera and
validate the taught postures/routes for the physical cell. The default routes,
calibration, model, weights and taught poses are already in this submission.

Keep the calibration file at the path named in YAML (relative to the YAML
directory, or absolute on the arm host). The launcher mounts the profile and
calibration read-only, then snapshots them inside the container. Editing the
host YAML does not retarget a running mission or its shutdown procedure.
Stop the runtime before applying a changed profile.

## Host prerequisites

| Machine | Must already provide |
| --- | --- |
| `.100` arm | Linux Docker Engine with the completed image loaded; Bash; hardware network routes; USB serial devices; kernel/RT scheduling support suitable for Franka FCI; robot FCI enabled |
| `.50` base/camera | ROS Humble and compatible TMR driver workspace, SICK drivers, SLAM Toolbox, TF2, rclpy, PyYAML, CycloneDDS, ZED wrapper and ZED SDK, SSH server |
| Both | Correct interface addresses and synchronized clocks |

The container supplies SSH client software. Mount an SSH directory containing
a usable key and verified `known_hosts` entries for `.50`; encrypted keys need
an authentication setup usable in the container (no agent forwarding is added).
By default the launcher mounts `$HOME/.ssh` to `/root/.ssh` read-only; override
with `EBIM_SSH_DIR=/absolute/path`. SSH config entries must use paths valid
inside that mount. No credentials are stored in the image or hardware YAML.

Docker uses host networking, private IPC, `SYS_NICE`, `IPC_LOCK`, RT priority
and unlimited memlock limits, plus the two explicit serial devices. It does
not use `--privileged`, host PID namespace, GPU runtime or nested Docker.
These options cannot supply a missing RT kernel or configure host networking.

## Commands on `.100`

Load/build the image separately; no command below builds or pulls it. Default
image: `franka-duo-table-mission:phase2`. If changed, set both `image` in YAML
and `EBIM_IMAGE` to that reference.

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
The host requires no Python when using the Bash launcher: YAML is parsed by
a temporary network-isolated container from the same image.

## Startup, failures and shutdown

`up` checks dependencies, deploys a versioned release, starts base/navigation
and camera, then arm/gripper/Spine drivers and servo/relay. It checks live state,
image/intrinsics, map/TF and servers. Process checks and bounded DDS discovery
reject existing unmanaged publishers/managers; they do not kill them. DDS
checks only cover the configured domains/interfaces and canonical names.

Arm impedance starts inactive. `--activate` requires fresh target/measured
positions within `0.003 rad`, correct joint order and healthy robot states.
The servo has a 60-second idle-follow window. If `up` without activation
exceeds it, use `down`, then `up --activate`. `up` refuses an existing runtime
container instead of restarting active streams or silently changing config.

The container remains alive after a mission finishes, a startup failure or a
child process failure. Failed children are recorded and block new missions;
they are not automatically restarted and robot faults are not reset. The
supervisor checks process ownership/child failures, not continuous physical
safety. Mission startup additionally checks live interfaces and robot state.

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
are deployed under `<runtime_root>/releases/<content-hash>/` on `.50`. Existing
vendor trees and shell startup files are not patched. No pre-existing
`~/tmr_cycle` or compiled `~/tmr_navigation` is required. Arm runtime code is
already in the image and its release/state lives in `/app/runtime`.

Docker volumes `<container>-state` and `<container>-outputs` persist logs and
mission checkpoints across container removal. Host process ownership includes
boot ID, PID namespace and process start time, so old container PIDs cannot be
reused as ownership evidence. A mission checkpoint intentionally prevents
replaying a completed drive; follow the README's reset-between-rounds procedure.

```bash
bash scripts/docker_hardware.sh logs
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/arm.log
docker exec ebim-cup-bowl-runtime tail -n 100 /app/runtime/logs/servo.log
docker cp ebim-cup-bowl-runtime:/app/outputs ./mission-outputs
```

## Build and validation status

`docker/drivers.lock.json` pins vendor sources. Docker builds libfranka and the
required Jazzy packages, then the bundled servo/model. The build checks plugin
libraries, generated FR3v2/Robotiq URDFs, controller configuration, executable
installation and FK/IK without starting hardware. Source/license copies are
retained in `/opt/ebim-vendor-src`.

The implementation has offline unit coverage. The complete image has **not yet
been built**, so these build checks and driver ABI compatibility remain
unverified. No deployment, hardware startup or target-host modification was
performed while implementing this workflow.
