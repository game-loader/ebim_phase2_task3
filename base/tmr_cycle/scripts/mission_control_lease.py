#!/usr/bin/env python3
"""Pure state machine for an exclusive, latched mission velocity lease.

The class in this module deliberately has no ROS dependency.  Callers provide
``now_ns`` so watchdog and hand-over behaviour can be tested deterministically.

A mission command implicitly acquires the lease.  Once acquired, the lease is
never released by a command timeout: a stale (or not-yet-received) mission
command requires a zero output while manual and navigation inputs remain
blocked.  Only :meth:`MissionControlLease.release` restores background input.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class LeaseSnapshot:
    """Complete output decision for one instant.

    ``require_zero`` means the adapter must hold a zero velocity output.  It is
    intentionally independent of ``allow_background`` so a timed-out mission
    cannot silently fall through to an old Nav2 or manual command.
    """

    latched: bool
    mission_fresh: bool
    allow_background: bool
    require_zero: bool
    last_mission_command_ns: Optional[int]
    command_age_ns: Optional[int]


class MissionControlLease:
    """Latched arbitration state for mission versus background velocity input."""

    def __init__(self, command_timeout_ns: int) -> None:
        if isinstance(command_timeout_ns, bool) or not isinstance(command_timeout_ns, int):
            raise TypeError("command_timeout_ns must be an integer")
        if command_timeout_ns <= 0:
            raise ValueError("command_timeout_ns must be positive")
        self._command_timeout_ns = command_timeout_ns
        self._latched = False
        self._last_mission_command_ns: Optional[int] = None
        self._last_now_ns: Optional[int] = None

    @property
    def command_timeout_ns(self) -> int:
        return self._command_timeout_ns

    @property
    def latched(self) -> bool:
        return self._latched

    @property
    def last_mission_command_ns(self) -> Optional[int]:
        return self._last_mission_command_ns

    def acquire(self, now_ns: int) -> LeaseSnapshot:
        """Explicitly acquire the lease without claiming a fresh command."""

        self._check_now(now_ns)
        self._latched = True
        return self._snapshot_unchecked(now_ns)

    def release(self, now_ns: int) -> LeaseSnapshot:
        """Explicitly release the lease and restore background arbitration."""

        self._check_now(now_ns)
        self._latched = False
        self._last_mission_command_ns = None
        return self._snapshot_unchecked(now_ns)

    def mission_command(self, now_ns: int) -> LeaseSnapshot:
        """Record an accepted mission command, implicitly acquiring the lease."""

        self._check_now(now_ns)
        self._latched = True
        self._last_mission_command_ns = now_ns
        return self._snapshot_unchecked(now_ns)

    def snapshot(self, now_ns: int) -> LeaseSnapshot:
        """Return the arbitration decision at ``now_ns`` without changing lease state."""

        self._check_now(now_ns)
        return self._snapshot_unchecked(now_ns)

    def background_allowed(self, now_ns: int) -> bool:
        """Return whether manual/Nav2 input may be forwarded at ``now_ns``."""

        return self.snapshot(now_ns).allow_background

    def _check_now(self, now_ns: int) -> None:
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            raise TypeError("now_ns must be an integer")
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        if self._last_now_ns is not None and now_ns < self._last_now_ns:
            raise ValueError("now_ns must be monotonic")
        self._last_now_ns = now_ns

    def _snapshot_unchecked(self, now_ns: int) -> LeaseSnapshot:
        if self._last_mission_command_ns is None:
            command_age_ns = None
            mission_fresh = False
        else:
            command_age_ns = now_ns - self._last_mission_command_ns
            mission_fresh = command_age_ns < self._command_timeout_ns

        allow_background = not self._latched
        require_zero = self._latched and not mission_fresh
        return LeaseSnapshot(
            latched=self._latched,
            mission_fresh=mission_fresh,
            allow_background=allow_background,
            require_zero=require_zero,
            last_mission_command_ns=self._last_mission_command_ns,
            command_age_ns=command_age_ns,
        )


__all__ = ["LeaseSnapshot", "MissionControlLease"]
