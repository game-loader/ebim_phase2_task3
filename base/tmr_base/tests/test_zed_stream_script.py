#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts" / "18_start_zed_stream.sh").read_text(encoding="utf-8")


class ZedStreamContracts(unittest.TestCase):
    def test_single_serial_bound_instance_uses_isolated_vision_domain(self) -> None:
        self.assertIn('TMR_CYCLE_VISION_DOMAIN_ID:-1', SOURCE)
        self.assertIn('ROS_LOCALHOST_ONLY', SOURCE)
        self.assertIn('serial_number:=17064700', SOURCE)
        self.assertIn('flock -n 9', SOURCE)

    def test_frame_export_stays_local(self) -> None:
        self.assertIn('zed_frame_export.py', SOURCE)
        self.assertNotIn('http.server', SOURCE)
        self.assertNotIn('18082', SOURCE)
        self.assertIn('rm -f "${frame_file}"', SOURCE)


if __name__ == "__main__":
    unittest.main()
