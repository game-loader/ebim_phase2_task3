#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]


class TmrControllerLimitContracts(unittest.TestCase):
    def test_route_acceleration_stays_below_installed_tmr_limit(self) -> None:
        config = yaml.safe_load((ROOT / "base" / "config" / "route.yaml").read_text())
        self.assertLessEqual(config["bootstrap_mapping"]["max_angular_accel"], 0.30)

    def test_active_route_helpers_use_the_same_angular_ramp(self) -> None:
        scripts = [
            "05_right_turn_map_gap.py",
            "08_reverse_to_predoor.py",
            "09_rotate_ccw.py",
            "10_translate_right.py",
            "11_translate_backward.py",
            "12_translate_right_odom_only.py",
            "13_post_grasp_route.py",
            "14_letter_guided_search.py",
        ]
        for name in scripts:
            source = (ROOT / "base" / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn("0.28", source, name)


if __name__ == "__main__":
    unittest.main()
