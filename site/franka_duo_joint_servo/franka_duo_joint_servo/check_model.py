"""Check the installed model and both KDL solvers without any robot inputs."""

import os
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix

from .model import default_model_directory, load_model_parameters


def main() -> int:
    executable = Path(get_package_prefix("franka_duo_joint_servo")) / "lib/franka_duo_joint_servo/check_robot_model"
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID="185", ROS_LOCALHOST_ONLY="1")
    environment.pop("CYCLONEDDS_URI", None)
    with tempfile.TemporaryDirectory(prefix="duo-model-check-") as temporary:
        params = Path(temporary) / "parameters.yaml"
        params.write_text(yaml.safe_dump({"/**": {"ros__parameters": load_model_parameters(default_model_directory())}}))
        return subprocess.run(
            [str(executable), "--ros-args", "--params-file", str(params)],
            env=environment, check=False, timeout=45,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
