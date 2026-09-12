# Third-Party Notices

This repository contains code adapted from the following project:

- **Franka ROS 2 drivers, descriptions, Spine interfaces/server and libfranka**.
  Docker downloads the exact revisions listed in `docker/drivers.lock.json`
  and retains their source trees and license files in `/opt/ebim-vendor-src`.
  Franka ROS 2 and descriptions are Apache 2.0; libfranka is Apache 2.0 and
  retains its bundled dependency notices. Franka ROS 2 also includes
  `realtime_tools` (BSD 3-Clause).
- **PickNik ROS 2 Robotiq gripper** (BSD 3-Clause) and **serial** by William
  Woodall and contributors (MIT), pinned in the same lock file. Full sources
  and notices are retained in `/opt/ebim-vendor-src` in the image.

- **Franka Robotics robot descriptions and Duo MoveIt configuration**, exported
  from the installed default model. Copyright Franka Robotics GmbH and
  contributors. Licensed under Apache 2.0. The expanded model, referenced
  meshes, original configuration helpers, licenses and source revisions are
  included in `site/franka_duo_joint_servo/model/`. See that directory's
  `manifest.json` and `../MODEL.md` for provenance.

- **Hugging Face LeRobot**, including adapted DP3 depth-to-point-cloud logic
  and optional pretrained-policy compatibility. Copyright the Hugging Face
  team and LeRobot contributors. Licensed under the Apache License, Version 2.0.
  Source: <https://github.com/huggingface/lerobot>

The recording and evaluation utilities were separated from
`game-loader/lerobot_droid`, which is also distributed under the Apache License,
Version 2.0. Source: <https://github.com/game-loader/lerobot_droid>

Optional runtime dependencies are installed separately and remain under their
own licenses. They include NumPy, PyYAML, PyTorch, ROS 2 and LeRobot. Their
inclusion in an environment does not change the license of
this repository. See each installed distribution for its full license text.
