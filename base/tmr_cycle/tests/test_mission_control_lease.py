#!/usr/bin/env python3
"""Offline tests for the mission velocity lease state machine."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from mission_control_lease import MissionControlLease  # noqa: E402


class MissionControlLeaseTests(unittest.TestCase):
    TIMEOUT_NS = 500_000_000

    def make_lease(self) -> MissionControlLease:
        return MissionControlLease(self.TIMEOUT_NS)

    def test_initial_state_allows_background(self) -> None:
        state = self.make_lease().snapshot(0)
        self.assertFalse(state.latched)
        self.assertTrue(state.allow_background)
        self.assertFalse(state.require_zero)
        self.assertFalse(state.mission_fresh)

    def test_explicit_acquire_latches_and_holds_zero_until_command(self) -> None:
        state = self.make_lease().acquire(100)
        self.assertTrue(state.latched)
        self.assertFalse(state.allow_background)
        self.assertTrue(state.require_zero)
        self.assertIsNone(state.last_mission_command_ns)

    def test_mission_command_implicitly_acquires_fresh_lease(self) -> None:
        state = self.make_lease().mission_command(1_000)
        self.assertTrue(state.latched)
        self.assertTrue(state.mission_fresh)
        self.assertFalse(state.allow_background)
        self.assertFalse(state.require_zero)
        self.assertEqual(state.command_age_ns, 0)

    def test_command_remains_fresh_just_before_timeout(self) -> None:
        lease = self.make_lease()
        lease.mission_command(10)
        state = lease.snapshot(10 + self.TIMEOUT_NS - 1)
        self.assertTrue(state.mission_fresh)
        self.assertFalse(state.require_zero)

    def test_exact_timeout_requires_zero_but_stays_latched(self) -> None:
        lease = self.make_lease()
        lease.mission_command(10)
        state = lease.snapshot(10 + self.TIMEOUT_NS)
        self.assertTrue(state.latched)
        self.assertFalse(state.mission_fresh)
        self.assertTrue(state.require_zero)
        self.assertFalse(state.allow_background)

    def test_timeout_never_falls_through_to_background(self) -> None:
        lease = self.make_lease()
        lease.mission_command(0)
        late_ns = self.TIMEOUT_NS * 20
        self.assertFalse(lease.background_allowed(late_ns))
        self.assertTrue(lease.snapshot(late_ns).latched)

    def test_new_mission_command_recovers_from_timeout_without_release(self) -> None:
        lease = self.make_lease()
        lease.mission_command(0)
        self.assertTrue(lease.snapshot(self.TIMEOUT_NS).require_zero)
        state = lease.mission_command(self.TIMEOUT_NS + 1)
        self.assertTrue(state.latched)
        self.assertTrue(state.mission_fresh)
        self.assertFalse(state.require_zero)

    def test_only_explicit_release_restores_background(self) -> None:
        lease = self.make_lease()
        lease.acquire(0)
        state = lease.release(1)
        self.assertFalse(state.latched)
        self.assertTrue(state.allow_background)
        self.assertFalse(state.require_zero)
        self.assertIsNone(state.last_mission_command_ns)

    def test_release_clears_old_command_freshness_before_reacquire(self) -> None:
        lease = self.make_lease()
        lease.mission_command(0)
        lease.release(1)
        state = lease.acquire(2)
        self.assertFalse(state.mission_fresh)
        self.assertTrue(state.require_zero)
        self.assertIsNone(state.command_age_ns)

    def test_invalid_timeout_and_backwards_clock_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            MissionControlLease(0)
        lease = self.make_lease()
        lease.snapshot(100)
        with self.assertRaisesRegex(ValueError, "monotonic"):
            lease.snapshot(99)


if __name__ == "__main__":
    unittest.main()
