#!/usr/bin/env python3
"""Synthetic tests for runtime OccupancyGrid doorway inference."""

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from runtime_map_door import (  # noqa: E402
    AmbiguousDoorwayError,
    DoorCandidate,
    DoorDetectorConfig,
    GridSnapshot,
    NoDoorwayFound,
    detect_doorway,
    validate_candidate,
)


def _ref_to_map(point, pose):
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return (
        pose[0] + cosine * point[0] - sine * point[1],
        pose[1] + sine * point[0] + cosine * point[1],
    )


def _map_to_ref(point, pose):
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return (cosine * dx + sine * dy, -sine * dx + cosine * dy)


def _synthetic_grid(
    *,
    resolution=0.05,
    origin_yaw=0.0,
    map_from_reference=(0.0, 0.0, 0.0),
    openings=((-0.45, 0.45),),
    unknown_corridor=False,
    wall_normal_angle=0.0,
):
    width = 220
    height = 180
    wall_x = 2.0
    centre_map = _ref_to_map((wall_x, 0.0), map_from_reference)
    half_x = width * resolution * 0.5
    half_y = height * resolution * 0.5
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    origin_x = centre_map[0] - cosine * half_x + sine * half_y
    origin_y = centre_map[1] - sine * half_x - cosine * half_y

    data = [0] * (width * height)
    for row in range(height):
        gy = (row + 0.5) * resolution
        for col in range(width):
            gx = (col + 0.5) * resolution
            map_point = (
                origin_x + cosine * gx - sine * gy,
                origin_y + sine * gx + cosine * gy,
            )
            ref_x, ref_y = _map_to_ref(map_point, map_from_reference)
            wall_normal = (math.cos(wall_normal_angle), math.sin(wall_normal_angle))
            wall_tangent = (-wall_normal[1], wall_normal[0])
            normal_value = ref_x * wall_normal[0] + ref_y * wall_normal[1]
            tangent_value = ref_x * wall_tangent[0] + ref_y * wall_tangent[1]
            in_opening = any(low < tangent_value < high for low, high in openings)
            if (
                unknown_corridor
                and wall_x - 0.42 < normal_value < wall_x + 0.62
                and any(
                    low + 0.06 < tangent_value < high - 0.06
                    for low, high in openings
                )
            ):
                value = -1
            else:
                value = 0
            if (
                abs(normal_value - wall_x) <= resolution * 0.82
                and -2.45 <= tangent_value <= 2.45
                and not in_opening
            ):
                value = 100
            data[row * width + col] = value
    return GridSnapshot(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        data=data,
    )


class RuntimeMapDoorTests(unittest.TestCase):
    def test_detects_frontal_door(self):
        grid = _synthetic_grid()
        candidate = detect_doorway(grid, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(candidate.midpoint[0], 2.0, delta=0.09)
        self.assertAlmostEqual(candidate.midpoint[1], 0.0, delta=0.09)
        self.assertAlmostEqual(candidate.width, 0.90, delta=0.12)
        self.assertGreater(candidate.normal[0], 0.97)
        self.assertGreater(candidate.corridor_free_ratio, 0.90)

    def test_origin_resolution_and_reference_pose_invariance(self):
        variants = (
            (0.04, 0.31, (1.20, -0.80, 0.53)),
            (0.06, -0.27, (-0.55, 1.10, -0.44)),
            (0.08, 0.16, (0.30, 0.70, 0.19)),
        )
        for resolution, origin_yaw, reference_pose in variants:
            with self.subTest(
                resolution=resolution,
                origin_yaw=origin_yaw,
                reference_pose=reference_pose,
            ):
                grid = _synthetic_grid(
                    resolution=resolution,
                    origin_yaw=origin_yaw,
                    map_from_reference=reference_pose,
                )
                candidate = detect_doorway(grid, reference_pose)
                self.assertAlmostEqual(candidate.midpoint[0], 2.0, delta=0.12)
                self.assertAlmostEqual(candidate.midpoint[1], 0.0, delta=0.12)
                self.assertAlmostEqual(candidate.width, 0.90, delta=0.16)

    def test_detects_a_nearly_frontal_slanted_wall(self):
        wall_angle = math.radians(10.0)
        grid = _synthetic_grid(
            origin_yaw=-0.21,
            map_from_reference=(0.8, -0.3, 0.37),
            wall_normal_angle=wall_angle,
        )
        candidate = detect_doorway(grid, (0.8, -0.3, 0.37))
        detected_angle = math.atan2(candidate.normal[1], candidate.normal[0])
        self.assertAlmostEqual(detected_angle, wall_angle, delta=math.radians(4.0))
        self.assertAlmostEqual(candidate.width, 0.90, delta=0.14)

    def test_unknown_corridor_is_not_treated_as_free(self):
        grid = _synthetic_grid(unknown_corridor=True)
        with self.assertRaises(NoDoorwayFound):
            detect_doorway(grid, (0.0, 0.0, 0.0))

    def test_similarly_scored_two_doors_are_ambiguous(self):
        grid = _synthetic_grid(openings=((-1.30, -0.50), (0.50, 1.30)))
        with self.assertRaises(AmbiguousDoorwayError) as context:
            detect_doorway(grid, (0.0, 0.0, 0.0))
        self.assertGreaterEqual(len(context.exception.candidates), 2)

    def test_validate_matching_lidar_candidate(self):
        grid = _synthetic_grid()
        candidate = detect_doorway(grid, (0.0, 0.0, 0.0))
        report = validate_candidate(grid, candidate, (0.0, 0.0, 0.0))
        self.assertEqual(report.status, "supported")
        self.assertTrue(report.supported)
        self.assertGreater(report.right_edge_support_ratio, 0.5)
        self.assertGreater(report.left_edge_support_ratio, 0.5)

    def test_validate_known_wall_as_conflict(self):
        grid = _synthetic_grid()
        lidar_candidate = DoorCandidate(
            midpoint=(2.0, 1.55),
            right_edge=(2.0, 1.10),
            left_edge=(2.0, 2.00),
            normal=(1.0, 0.0),
            tangent=(0.0, 1.0),
            width=0.90,
            score=1.0,
            plane_distance=2.0,
            right_wall_support_m=0.5,
            left_wall_support_m=0.5,
            right_edge_support_ratio=1.0,
            left_edge_support_ratio=1.0,
            corridor_free_ratio=1.0,
            corridor_known_ratio=1.0,
            corridor_unknown_ratio=0.0,
            corridor_occupied_ratio=0.0,
        )
        report = validate_candidate(grid, lidar_candidate, (0.0, 0.0, 0.0))
        self.assertEqual(report.status, "conflict")
        self.assertFalse(report.supported)
        self.assertGreater(report.corridor_occupied_ratio, 0.0)

    def test_validate_outside_map_as_unknown(self):
        grid = _synthetic_grid()
        lidar_candidate = DoorCandidate(
            midpoint=(20.0, 0.0),
            right_edge=(20.0, -0.45),
            left_edge=(20.0, 0.45),
            normal=(1.0, 0.0),
            tangent=(0.0, 1.0),
            width=0.90,
            score=1.0,
            plane_distance=20.0,
            right_wall_support_m=0.5,
            left_wall_support_m=0.5,
            right_edge_support_ratio=1.0,
            left_edge_support_ratio=1.0,
            corridor_free_ratio=1.0,
            corridor_known_ratio=1.0,
            corridor_unknown_ratio=0.0,
            corridor_occupied_ratio=0.0,
        )
        report = validate_candidate(grid, lidar_candidate, (0.0, 0.0, 0.0))
        self.assertEqual(report.status, "unknown")
        self.assertFalse(report.supported)
        self.assertAlmostEqual(report.corridor_unknown_ratio, 1.0)


if __name__ == "__main__":
    unittest.main()
