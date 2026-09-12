#!/usr/bin/env python3
"""Export the installed default Duo model without connecting to robot hardware."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    args = parser.parse_args()

    from ament_index_python.packages import get_package_share_directory
    from franka_mobile_fr3_duo_moveit_config import description, parameters

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    urdf, srdf = description.get_robot_descriptions("mobile_fr3_duo_v0_2", "false")
    parameters.get_parameters()
    original = output / "source"
    original.mkdir()
    (original / "robot.urdf").write_text(urdf)
    (original / "robot.srdf").write_text(srdf)
    for module in (description, parameters):
        shutil.copy2(inspect.getfile(module), original)

    config_share = Path(get_package_share_directory("franka_mobile_fr3_duo_moveit_config"))
    for name in ("kinematics.yaml", "joint_limits.yaml", "moveit_controllers.yaml", "moveit_defaults.json"):
        shutil.copy2(config_share / "config" / name, output / name)
    shutil.copy2(config_share / "package.xml", original / "moveit_config.package.xml")
    shutil.copytree(config_share / "urdf", original / "urdf")

    copied: dict[str, str] = {}

    def copy_resource(package: str, relative: str) -> str:
        share = Path(get_package_share_directory(package)).resolve()
        relative = os.path.normpath(relative)
        if Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError(f"resource outside package: {relative}")
        source = share / relative
        destination = output / "resources" / package / relative
        key = str(destination.relative_to(output))
        if key in copied:
            return f"package://franka_duo_joint_servo/model/{key}"
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied[key] = f"package://{package}/{relative}"
        # Collada visuals may reference textures relative to the mesh file.
        if source.suffix.lower() == ".dae":
            for element in ET.parse(source).getroot().iter():
                if element.tag.rsplit("}", 1)[-1] != "image":
                    continue
                for child in element:
                    if child.tag.rsplit("}", 1)[-1] == "init_from" and child.text:
                        texture = unquote(child.text.strip())
                        if urlparse(texture).scheme or Path(texture).is_absolute():
                            raise ValueError(f"non-relative Collada texture: {texture}")
                        texture_path = source.parent.relative_to(share) / texture
                        copy_resource(package, str(texture_path))
        return f"package://franka_duo_joint_servo/model/{key}"

    robot = ET.fromstring(urdf)
    for element in robot.iter():
        for attribute, value in list(element.attrib.items()):
            if value.startswith("package://"):
                parsed = urlparse(value)
                element.set(attribute, copy_resource(parsed.netloc, unquote(parsed.path.lstrip("/"))))
            elif value.startswith("file://"):
                raise ValueError(f"unbundled local resource: {value}")
    ET.ElementTree(robot).write(output / "robot.urdf", encoding="utf-8", xml_declaration=True)
    (output / "robot.srdf").write_text(srdf)

    packages = {}
    for name, relative in (
        ("franka_mobile_fr3_duo_moveit_config", "franka_mobile_fr3_duo_moveit_config"),
        ("franka_description", "franka_description"),
        ("franka_hardware", "franka_hardware"),
    ):
        source = args.source_root / relative
        commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
        status = subprocess.check_output(["git", "-C", str(source), "status", "--short", "--", "."], text=True).strip()
        share = Path(get_package_share_directory(name))
        packages[name] = {
            "version": ET.parse(share / "package.xml").getroot().findtext("version"),
            "source_commit": commit,
            "source_status": status,
        }
        for candidate in (source / "LICENSE", args.source_root / "LICENSE"):
            if candidate.is_file():
                shutil.copy2(candidate, original / f"{name}.LICENSE")
                break

    runtime = subprocess.check_output([
        "dpkg-query", "-W", "ros-jazzy-moveit-core", "ros-jazzy-moveit-ros-planning",
        "ros-jazzy-moveit-kinematics", "ros-jazzy-ruckig",
    ], text=True)
    manifest = {
        "schema_version": 1,
        "robot_name": "mobile_fr3_duo_v0_2",
        "simulate_in_gazebo": "false",
        "generator": "franka_mobile_fr3_duo_moveit_config.description.get_robot_descriptions",
        "transformation": "Only package resource URIs and XML serialization changed; original XML is in source/.",
        "packages": packages,
        "host_runtime_packages": dict(line.split("\t", 1) for line in runtime.splitlines()),
        "resources": copied,
        "sha256": {
            str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(output.rglob("*")) if path.is_file()
        },
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"output": str(output), "resources": len(copied), "packages": packages}, indent=2))


if __name__ == "__main__":
    main()
