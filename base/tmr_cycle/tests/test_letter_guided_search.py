#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "base" / "scripts"
SEARCH_SOURCE = (SCRIPTS / "14_letter_guided_search.py").read_text(encoding="utf-8")
RUNNER_SOURCE = (SCRIPTS / "14_run_letter_guided_search.sh").read_text(encoding="utf-8")
EXPORT_SOURCE = (SCRIPTS / "zed_frame_export.py").read_text(encoding="utf-8")
sys.path.insert(0, str(SCRIPTS))

import letter_card_vision as vision  # noqa: E402

SEARCH_SPEC = importlib.util.spec_from_file_location(
    "letter_search", SCRIPTS / "14_letter_guided_search.py"
)
assert SEARCH_SPEC is not None and SEARCH_SPEC.loader is not None
search = importlib.util.module_from_spec(SEARCH_SPEC)
sys.modules[SEARCH_SPEC.name] = search
SEARCH_SPEC.loader.exec_module(search)


def detection(letter: str, x: float, row: str, confidence: float = 0.8):
    return vision.LetterDetection(
        letter=letter,
        confidence=confidence,
        center_x_norm=x,
        center_y_norm=0.72 if row == "near" else 0.30,
        area_norm=0.01,
        row=row,
        quad=((0.0, 0.0),) * 4,
    )


class LetterVisionContracts(unittest.TestCase):
    def test_camera_transport_is_isolated_from_control_dds(self) -> None:
        self.assertIn('vision_domain="${TMR_CYCLE_VISION_DOMAIN_ID:-1}"', RUNNER_SOURCE)
        self.assertIn('--camera-file "${frame_file}"', RUNNER_SOURCE)
        self.assertIn("os.replace(self.temporary, self.output)", EXPORT_SOURCE)
        self.assertIn("node.refresh_frame_file()", SEARCH_SOURCE)

    def test_direct_controller_is_explicit_recovery_mode(self) -> None:
        self.assertIn('parser.add_argument(\n        "--direct-controller"', SEARCH_SOURCE)
        self.assertIn("CONTROLLER_COMMAND_TOPIC", SEARCH_SOURCE)

    def test_white_cards_and_synthetic_letters_are_recognized(self) -> None:
        image = np.full((720, 1280, 3), (55, 70, 85), np.uint8)
        for letter, x, y in (("A", 150, 470), ("B", 560, 460), ("E", 930, 170)):
            cv2.rectangle(image, (x, y), (x + 90, y + 130), (245, 245, 245), -1)
            cv2.putText(image, letter, (x + 10, y + 100), cv2.FONT_HERSHEY_SIMPLEX,
                        2.3, (70, 70, 70), 4, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABE", minimum_confidence=0.20).detect(image)
        self.assertEqual({item.letter for item in found}, {"A", "B", "E"})
        self.assertEqual({item.row for item in found if item.letter in {"A", "B"}}, {"near"})
        self.assertEqual({item.row for item in found if item.letter == "E"}, {"far"})

    def test_letter_d_is_supported_when_configured(self) -> None:
        image = np.full((720, 1280, 3), (55, 70, 85), np.uint8)
        for letter, x, y in (("A", 150, 470), ("B", 560, 460), ("D", 930, 170)):
            cv2.rectangle(image, (x, y), (x + 90, y + 130), (245, 245, 245), -1)
            cv2.putText(image, letter, (x + 10, y + 100), cv2.FONT_HERSHEY_SIMPLEX,
                        2.3, (70, 70, 70), 4, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABD", minimum_confidence=0.20).detect(image)
        self.assertEqual({item.letter for item in found}, {"A", "B", "D"})

    def test_same_letter_cards_use_group_midpoint(self) -> None:
        tracker = search.TargetTracker("E", "far", stable_frames=3)
        observation = tracker.observe(
            [detection("E", 0.35, "far"), detection("E", 0.65, "far")], 0.7
        )
        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation.center_x_norm, 0.5)
        self.assertEqual(observation.members, 2)

    def test_center_requires_repeated_low_spread_observations(self) -> None:
        tracker = search.TargetTracker("B", "auto", stable_frames=3)
        for right_m, x in ((0.6, 0.53), (0.62, 0.51), (0.64, 0.52)):
            tracker.observe([detection("B", x, "near")], right_m)
        centered, error, spread = tracker.centered()
        self.assertTrue(centered)
        self.assertLess(abs(error), 0.055)
        self.assertLess(spread, 0.032)

    def test_adaptive_policy_learns_camera_motion_sign(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        first = search.TargetObservation(0.50, 0.70, "near", 0.8, 1)
        second = search.TargetObservation(0.55, 0.64, "near", 0.8, 1)
        self.assertGreater(policy.command(first), 0.0)
        command = policy.command(second)
        self.assertIsNotNone(policy.image_gain_per_m)
        self.assertGreater(command, 0.0)

    def test_gain_anchor_survives_high_frame_rate_small_steps(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        for right_m, x in ((0.50, 0.70), (0.506, 0.696), (0.512, 0.692), (0.519, 0.687)):
            policy.command(search.TargetObservation(right_m, x, "near", 0.86, 1))
        self.assertIsNotNone(policy.image_gain_per_m)
        self.assertLess(policy.image_gain_per_m, 0.0)

    def test_low_confidence_detection_cannot_reverse_gain(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        policy.command(search.TargetObservation(1.00, 0.70, "near", 0.86, 1))
        policy.command(search.TargetObservation(1.03, 0.68, "near", 0.86, 1))
        learned = policy.image_gain_per_m
        self.assertEqual(
            policy.command(search.TargetObservation(1.30, 0.90, "near", 0.29, 1)),
            0.0,
        )
        self.assertEqual(policy.image_gain_per_m, learned)

    def test_center_crossing_stops_then_accepts_a_credible_single_frame(self) -> None:
        hold = search.CenterHold(0.055, single_frame_hold_s=0.30)
        holding, centered, _, _ = hold.update(
            search.TargetObservation(1.76, 0.483, "near", 0.36, 1), 10.0
        )
        self.assertTrue(holding)
        self.assertFalse(centered)
        holding, centered, error, _ = hold.status(10.31)
        self.assertTrue(holding)
        self.assertTrue(centered)
        self.assertAlmostEqual(error, -0.017)

    def test_center_hold_rejects_a_low_confidence_single_frame(self) -> None:
        hold = search.CenterHold(0.055, single_frame_hold_s=0.30)
        hold.update(search.TargetObservation(1.2, 0.51, "near", 0.20, 1), 4.0)
        self.assertFalse(hold.status(4.31)[1])

    def test_acquisition_band_does_not_stop_outside_center_tolerance(self) -> None:
        hold = search.CenterHold(0.055, acquire_tolerance_norm=0.080)
        holding, centered, _, _ = hold.update(
            search.TargetObservation(1.2, 0.57, "near", 0.86, 1), 4.0
        )
        self.assertFalse(holding)
        self.assertFalse(centered)


if __name__ == "__main__":
    unittest.main()
