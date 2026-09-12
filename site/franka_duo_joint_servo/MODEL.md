# Bundled Duo Model

Exported on 2026-09-11 from the installed default configuration on the arm
computer, using `get_robot_descriptions("mobile_fr3_duo_v0_2", "false")` and
the package's default parameter files. No robot node was started by the export.

- `franka_mobile_fr3_duo_moveit_config` 3.4.1 and `franka_hardware` 3.4.1:
  source commit `73a1501d76efa2bc4bf09cb2af9c2b72c2c642da`.
- `franka_description` 2.8.1:
  source commit `02afaae282d4a8e10d7d2f781b23b3515c303ce5`.
- Source worktrees were clean for these packages. The source snapshot,
  licenses, host dependency versions and per-file SHA-256 hashes are recorded
  under `model/source/` and `model/manifest.json`.

The exported URDF preserves all geometry, joints, limits and hardware
metadata. Only mesh resource URIs are relocated into this package and XML is
serialized again. All 26 referenced mesh resources are included. The original
expanded URDF/SRDF and the exact installed `description.py` / `parameters.py`
are retained for comparison. The servo loads the same default KDL and joint
limit configuration; it does not start the official full-body controller.

`joint_servo.launch.py` defaults to the installed model. A separately
validated snapshot can be selected with `model_directory:=/path/to/model`.
The original host MoveIt configuration package and its xacro include chain
are no longer runtime requirements for this joint servo. The separate PTP
utilities still use their original host configuration.

The image builds MoveIt/KDL/Ruckig clients, the joint servo, relay and Spine
interfaces. Host hardware drivers and impedance controllers remain separate.
`servo` and `relay` image modes launch only their named processes; they never
activate controllers. Relay publication requires `enable_robot:=true`, and
gripper publication additionally requires `enable_gripper:=true`.

```bash
docker run --rm franka-duo-table-mission:phase2 model-check
```

This loads the installed model and runs three FK/IK round trips per arm in an
isolated localhost ROS domain, without robot state or command publishers.
It validates model portability, not real-time servo performance or a physical
mission. Do not run a second servo/relay alongside an existing instance.

Builds can select signed package mirrors with `--build-arg UBUNTU_APT_MIRROR=...`
and `--build-arg ROS_APT_MIRROR=...`. Empty defaults retain the upstream sources.
`--build-arg PIP_INDEX_URL=...` optionally selects a PyPI mirror.

To refresh the snapshot on a machine with the original ROS overlay sourced:

```bash
python3 scripts/export_host_duo_model.py \
  --source-root /home/aup/recloned_sources/franka_ros2_jazzy_ws/src \
  --output /tmp/new-duo-model
```

The output directory must not already exist. Review its manifest and original
XML before replacing the submitted snapshot.
