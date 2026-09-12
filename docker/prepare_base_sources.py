"""Normalize upstream controller YAML without changing the inspected parameters."""

from pathlib import Path
import sys
import yaml


class MergeLoader(yaml.SafeLoader):
    pass


def mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        value = loader.construct_object(value_node, deep=True)
        if key in result:
            if not isinstance(value, dict) or not isinstance(result[key], dict):
                raise ValueError(f"conflicting duplicate scalar: {key}")
            if result[key].keys() & value.keys():
                raise ValueError(f"conflicting duplicate controller: {key}")
            result[key].update(value)
        else:
            result[key] = value
    return result


MergeLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


def prepare(root):
    path = Path(root) / "franka_ros2/franka_bringup/config/controllers.yaml"
    settings = yaml.load(path.read_text(), Loader=MergeLoader)
    controllers = settings["/**"]
    assert controllers["controller_manager"]["ros__parameters"]["swerve_drive_controller"]["type"] == "franka_mobile/SwerveDriveController"
    assert controllers["swerve_drive_controller"]["ros__parameters"]["linear"]["x"]["max_velocity"] == 0.35
    path.write_text(yaml.safe_dump(settings, sort_keys=False))


if __name__ == "__main__":
    prepare(sys.argv[1])
