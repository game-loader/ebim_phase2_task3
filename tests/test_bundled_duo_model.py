from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

MODEL = Path(__file__).resolve().parents[1] / "site/franka_duo_joint_servo/model"


def test_exported_model_preserves_host_geometry_and_semantics():
    manifest = json.loads((MODEL / "manifest.json").read_text())
    exported = ET.parse(MODEL / "robot.urdf").getroot()
    original = ET.parse(MODEL / "source/robot.urdf").getroot()
    for element in exported.iter():
        for attribute, value in list(element.attrib.items()):
            prefix = "package://franka_duo_joint_servo/model/"
            if value.startswith(prefix):
                element.set(attribute, manifest["resources"][value[len(prefix):]])
    assert ET.tostring(exported) == ET.tostring(original)
    assert (MODEL / "robot.srdf").read_bytes() == (MODEL / "source/robot.srdf").read_bytes()


def test_snapshot_checksums_and_resource_closure():
    manifest = json.loads((MODEL / "manifest.json").read_text())
    for relative, expected in manifest["sha256"].items():
        assert hashlib.sha256((MODEL / relative).read_bytes()).hexdigest() == expected, relative
    urdf = ET.parse(MODEL / "robot.urdf").getroot()
    resources = set()
    for element in urdf.iter():
        for value in element.attrib.values():
            assert not value.startswith("file://"), value
            if value.startswith("package://"):
                prefix = "package://franka_duo_joint_servo/model/"
                assert value.startswith(prefix), value
                relative = value[len(prefix):]
                assert (MODEL / relative).is_file(), relative
                resources.add(relative)
    assert resources <= manifest["resources"].keys()
    assert len(resources) > 0


def test_bundled_defaults_use_kdl_for_both_seven_joint_arms():
    kinematics = yaml.safe_load((MODEL / "kinematics.yaml").read_text())
    semantics = ET.parse(MODEL / "robot.srdf").getroot()
    joints = {joint.get("name") for joint in ET.parse(MODEL / "robot.urdf").getroot().findall("joint")}
    groups = {group.get("name") for group in semantics.findall("group")}
    for side in ("left", "right"):
        assert f"{side}_arm" in groups
        assert kinematics[f"{side}_arm"]["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
        assert {f"{side}_fr3v2_joint{i}" for i in range(1, 8)} <= joints
