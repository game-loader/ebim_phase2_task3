#!/usr/bin/env python3
"""Synthetic tests for map-frame rectangular braking sweeps."""

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from runtime_grid_collision import (  # noqa: E402
    GridSweepConfig,
    RectangularFootprint,
    evaluate_occupancy_grid_sweep,
)
from runtime_map_door import GridSnapshot  # noqa: E402


def _transform(point, pose):
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    return (
        pose[0] + cosine * point[0] - sine * point[1],
        pose[1] + sine * point[0] + cosine * point[1],
    )


def _grid(*, resolution=0.05, origin=(-2.0, -2.0, 0.0), fill=0, marks=()):
    width = height = 120
    data = [fill] * (width * height)
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    for map_x, map_y, value in marks:
        dx = map_x - origin[0]
        dy = map_y - origin[1]
        gx = cosine * dx + sine * dy
        gy = -sine * dx + cosine * dy
        col = int(math.floor(gx / resolution))
        row = int(math.floor(gy / resolution))
        data[row * width + col] = value
    return GridSnapshot(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
        data=data,
    )


FOOTPRINT = RectangularFootprint(front_m=0.40, rear_m=0.40, width_m=0.58)
CONFIG = GridSweepConfig(
    brake_accel_mps2=0.25,
    reaction_time_s=0.25,
    hard_margin_m=0.08,
)


def _three_cell_cluster(x, y, spacing=0.05):
    return ((x, y - spacing, 100), (x, y, 100), (x, y + spacing, 100))


class RuntimeGridCollisionTests(unittest.TestCase):
    def test_forward_known_occupied_blocks(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=_three_cell_cluster(0.52, 0.0)),
            (0.0, 0.0, 0.0),
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertTrue(result.blocked)
        self.assertGreater(result.occupied_cells, 0)
        self.assertIsNotNone(result.collision_progress_m)
        self.assertGreaterEqual(result.largest_occupied_cluster_cells, 2)

    def test_isolated_map_speckle_does_not_stop_secondary_guard(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=((0.52, 0.0, 100),)),
            (0.0, 0.0, 0.0),
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertFalse(result.blocked)
        self.assertEqual(result.occupied_cells, 1)
        self.assertEqual(result.largest_occupied_cluster_cells, 1)
        self.assertFalse(result.map_can_confirm_clear)

    def test_occupied_cells_inside_starting_footprint_are_ignored(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=((-0.33, 0.0, 100), (0.20, 0.25, 100))),
            (0.0, 0.0, 0.0),
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertFalse(result.blocked)
        self.assertEqual(result.occupied_cells, 0)

    def test_side_obstacle_still_blocks_lateral_motion(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=((0.0, 0.36, 100), (0.05, 0.36, 100), (-0.05, 0.36, 100))),
            (0.0, 0.0, 0.0),
            (0.0, 0.10),
            FOOTPRINT,
            CONFIG,
        )
        self.assertTrue(result.blocked)

    def test_obstacle_beyond_braking_sweep_does_not_block(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=((0.90, 0.0, 100),)),
            (0.0, 0.0, 0.0),
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertFalse(result.blocked)
        self.assertTrue(result.map_can_confirm_clear)

    def test_reverse_motion_blocks_behind(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=_three_cell_cluster(-0.53, 0.0)),
            (0.0, 0.0, 0.0),
            (-0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertTrue(result.blocked)

    def test_lateral_motion_blocks_at_side(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=((0.0, 0.41, 100), (0.05, 0.41, 100), (-0.05, 0.41, 100))),
            (0.0, 0.0, 0.0),
            (0.0, 0.10),
            FOOTPRINT,
            CONFIG,
        )
        self.assertTrue(result.blocked)

    def test_unknown_is_not_obstacle_and_cannot_confirm_clear(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(fill=-1),
            (0.0, 0.0, 0.0),
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertFalse(result.blocked)
        self.assertGreater(result.unknown_cells, 0)
        self.assertFalse(result.map_can_confirm_clear)

    def test_map_from_base_rotation_is_used(self):
        pose = (1.0, -0.4, math.pi / 2.0)
        obstacles = [_transform(point, pose) for point in ((0.52, -0.05), (0.52, 0.0), (0.52, 0.05))]
        result = evaluate_occupancy_grid_sweep(
            _grid(marks=tuple((x, y, 100) for x, y in obstacles)),
            pose,
            (0.10, 0.0),
            FOOTPRINT,
            CONFIG,
        )
        self.assertTrue(result.blocked)

    def test_rotated_grid_origin_is_used(self):
        pose = (0.8, 0.3, -0.4)
        obstacles = [_transform(point, pose) for point in ((0.52, -0.04), (0.52, 0.0), (0.52, 0.04))]
        grid = _grid(
            resolution=0.04,
            origin=(-2.3, -1.7, 0.37),
            marks=tuple((x, y, 100) for x, y in obstacles),
        )
        result = evaluate_occupancy_grid_sweep(
            grid, pose, (0.10, 0.0), FOOTPRINT, CONFIG
        )
        self.assertTrue(result.blocked)

    def test_faster_command_has_longer_braking_sweep(self):
        grid = _grid(marks=_three_cell_cluster(0.72, 0.0))
        slow = evaluate_occupancy_grid_sweep(
            grid, (0.0, 0.0, 0.0), (0.05, 0.0), FOOTPRINT, CONFIG
        )
        fast = evaluate_occupancy_grid_sweep(
            grid, (0.0, 0.0, 0.0), (0.30, 0.0), FOOTPRINT, CONFIG
        )
        self.assertFalse(slow.blocked)
        self.assertTrue(fast.blocked)
        self.assertGreater(fast.swept_distance_m, slow.swept_distance_m)

    def test_stationary_does_not_claim_map_clear(self):
        result = evaluate_occupancy_grid_sweep(
            _grid(), (0.0, 0.0, 0.0), (0.0, 0.0), FOOTPRINT, CONFIG
        )
        self.assertFalse(result.blocked)
        self.assertFalse(result.map_can_confirm_clear)


if __name__ == "__main__":
    unittest.main()
