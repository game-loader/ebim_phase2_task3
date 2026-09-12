#!/usr/bin/env python3
"""Arbitrate TMR velocity inputs with a fail-closed exclusive mission lease."""

from __future__ import annotations

import copy
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool

from mission_control_lease import MissionControlLease


class CmdVelAdapter(Node):
    def __init__(self) -> None:
        super().__init__("tmr_cmd_vel_adapter")
        self.declare_parameter("output_topic", "/swerve_drive_controller/cmd_vel")
        self.declare_parameter("manual_topic", "/tmr_cycle/cmd_vel")
        self.declare_parameter("mission_topic", "/tmr_cycle/mission_cmd_vel")
        self.declare_parameter("mission_lease_topic", "/tmr_cycle/mission_active")
        self.declare_parameter("startup_locked", True)
        self.declare_parameter("manual_priority_sec", 0.20)
        self.declare_parameter("command_timeout_sec", 0.50)
        self.declare_parameter("mission_max_age_sec", 0.35)
        self.declare_parameter("mission_future_tolerance_sec", 0.10)

        self._publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter("output_topic").value), 10
        )
        self._last_manual_ns = -1
        self._last_nav_ns = -1
        self._last_smoothed_ns = -1
        self._mission_lease = MissionControlLease(
            max(
                1,
                int(float(self.get_parameter("command_timeout_sec").value) * 1e9),
            )
        )
        self._zero_sent = True
        if bool(self.get_parameter("startup_locked").value):
            self._mission_lease.acquire(self._now_ns())
            # Repeat once from the first watchdog tick after graph discovery.
            self._publish_twist(Twist())

        self.create_subscription(Twist, "/cmd_vel", self._on_raw_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_smoothed_nav, 10)
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("manual_topic").value),
            self._on_manual,
            10,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("mission_topic").value),
            self._on_mission,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("mission_lease_topic").value),
            self._on_mission_lease,
            10,
        )
        self.create_timer(0.05, self._watchdog)

    def _now_ns(self) -> int:
        return time.monotonic_ns()

    def _manual_is_recent(self) -> bool:
        window = float(self.get_parameter("manual_priority_sec").value)
        return self._last_manual_ns >= 0 and (self._now_ns() - self._last_manual_ns) < window * 1e9

    def _publish_twist(self, twist: Twist) -> None:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist = copy.deepcopy(twist)
        self._publisher.publish(out)
        self._zero_sent = False

    @staticmethod
    def _twist_is_zero(twist: Twist) -> bool:
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        return all(math.isfinite(float(value)) and abs(float(value)) <= 1e-9 for value in values)

    @staticmethod
    def _twist_is_finite(twist: Twist) -> bool:
        return all(
            math.isfinite(float(value))
            for value in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )

    def _on_manual(self, msg: TwistStamped) -> None:
        now_ns = self._now_ns()
        self._last_manual_ns = now_ns
        if self._mission_lease.background_allowed(now_ns):
            self._publish_twist(msg.twist)

    def _on_mission(self, msg: TwistStamped) -> None:
        # A mission command acquires a latched exclusive lease.  If the
        # mission process crashes, the watchdog holds zero instead of falling
        # back to an old Nav2 goal, teleop process, or legacy manual script.
        now_monotonic_ns = self._now_ns()
        if not self._twist_is_finite(msg.twist):
            self._mission_lease.acquire(now_monotonic_ns)
            self._publish_twist(Twist())
            self._zero_sent = True
            self.get_logger().error("rejected non-finite mission velocity and held zero")
            return
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._twist_is_zero(msg.twist):
            # A stop is always honored, but an unstamped/replayed stop does not
            # claim freshness for a later nonzero command.
            if stamp_ns > 0:
                self._mission_lease.mission_command(now_monotonic_ns)
            else:
                self._mission_lease.acquire(now_monotonic_ns)
            self._publish_twist(msg.twist)
            return
        ros_now_ns = self.get_clock().now().nanoseconds
        age_ns = ros_now_ns - stamp_ns
        maximum_age_ns = int(float(self.get_parameter("mission_max_age_sec").value) * 1e9)
        future_tolerance_ns = int(
            float(self.get_parameter("mission_future_tolerance_sec").value) * 1e9
        )
        if (
            stamp_ns <= 0
            or msg.header.frame_id.lstrip("/") != "base_link"
            or age_ns > maximum_age_ns
            or age_ns < -future_tolerance_ns
        ):
            self._mission_lease.acquire(now_monotonic_ns)
            self._publish_twist(Twist())
            self._zero_sent = True
            self.get_logger().error(
                f"rejected stale/future/unframed mission velocity (age={age_ns / 1e9:.3f}s) and held zero"
            )
            return
        self._mission_lease.mission_command(now_monotonic_ns)
        self._publish_twist(msg.twist)

    def _on_mission_lease(self, msg: Bool) -> None:
        if msg.data:
            state = self._mission_lease.acquire(self._now_ns())
            if state.require_zero:
                self._publish_twist(Twist())
                self._zero_sent = True
            return
        self._mission_lease.release(self._now_ns())
        self._publish_twist(Twist())
        self._zero_sent = True

    def _on_smoothed_nav(self, msg: Twist) -> None:
        self._last_smoothed_ns = self._now_ns()
        self._last_nav_ns = self._last_smoothed_ns
        if self._mission_lease.background_allowed(self._now_ns()) and not self._manual_is_recent():
            self._publish_twist(msg)

    def _on_raw_nav(self, msg: Twist) -> None:
        now_ns = self._now_ns()
        self._last_nav_ns = now_ns
        smoothed_recent = self._last_smoothed_ns >= 0 and now_ns - self._last_smoothed_ns < int(0.20e9)
        if (
            self._mission_lease.background_allowed(now_ns)
            and not smoothed_recent
            and not self._manual_is_recent()
        ):
            self._publish_twist(msg)

    def _watchdog(self) -> None:
        now_ns = self._now_ns()
        lease = self._mission_lease.snapshot(now_ns)
        if lease.latched:
            if lease.require_zero:
                # Hold zero continuously so a late DDS match or controller
                # reconnect cannot retain a command sent before adapter restart.
                self._publish_twist(Twist())
                self._zero_sent = True
            return
        newest = max(self._last_manual_ns, self._last_nav_ns)
        timeout = float(self.get_parameter("command_timeout_sec").value)
        if newest >= 0 and now_ns - newest > timeout * 1e9:
            self._publish_twist(Twist())
            self._zero_sent = True

    def hold_zero(self, repetitions: int = 20) -> None:
        for _ in range(max(1, repetitions)):
            self._publish_twist(Twist())
            time.sleep(0.01)
        self._zero_sent = True


def main() -> None:
    rclpy.init()
    node = CmdVelAdapter()

    def stop_on_signal(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_signal)
    signal.signal(signal.SIGINT, stop_on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.hold_zero()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
