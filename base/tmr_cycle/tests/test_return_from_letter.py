#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts" / "15_return_from_letter.py").read_text(encoding="utf-8")


class ReturnFromLetterContracts(unittest.TestCase):
    def test_measured_left_precedes_clockwise_turn_and_door_child(self) -> None:
        points = [
            SOURCE.index("node.translate(0.0, total_left_m"),
            SOURCE.index("node.rotate_ccw(-math.radians(args.turn_cw_deg)"),
            SOURCE.index("run_door_child(args)"),
        ]
        self.assertEqual(points, sorted(points))
        self.assertIn('"TURN_CW_180"', SOURCE)

    def test_door_child_skips_old_prefix_only(self) -> None:
        self.assertIn('environment["TMR_CYCLE_SKIP_INITIAL_FORWARD"] = "1"', SOURCE)
        self.assertIn('environment["TMR_CYCLE_SKIP_TURN"] = "1"', SOURCE)
        self.assertIn('"door_before_m": 0.50', SOURCE)
        self.assertIn('"door_forward_m": 1.20', SOURCE)

    def test_resume_door_requires_completed_base_realign(self) -> None:
        self.assertIn('resumable_phases = {"BASE_REALIGNED", "DOOR_RETURN_FAILED"}', SOURCE)
        self.assertIn('{"LEFT_BY_MEASURED_OUTBOUND", "TURN_CW_180"}.issubset(completed)', SOURCE)
        self.assertIn('report.get("final_state") == "FINAL_STOP"', SOURCE)


if __name__ == "__main__":
    unittest.main()
