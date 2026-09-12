from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def default_model_directory() -> Path:
    return Path(get_package_share_directory("franka_duo_joint_servo")) / "model"


def load_model_parameters(directory: Path) -> dict:
    """Load the exported host defaults, including the original KDL parameters."""
    return {
        "robot_description": (directory / "robot.urdf").read_text(),
        "robot_description_semantic": (directory / "robot.srdf").read_text(),
        "robot_description_kinematics": yaml.safe_load((directory / "kinematics.yaml").read_text()),
        "robot_description_planning": yaml.safe_load((directory / "joint_limits.yaml").read_text()),
    }
