"""Bridge /pedal/state to the TMR swerve base as a TwistStamped stream.

Single pedals map to base motion (x+/x-, y+/y-, CW/CCW). When a spine combo is
fully pressed the base is held still so the spine bridge can jog instead. A
TwistStamped is published continuously at ``publish_rate`` (zero when nothing is
pressed) so the swerve controller's watchdog is fed and the base stops on release.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class BaseBridge(Node):
    def __init__(self):
        super().__init__("base_bridge")

        self.declare_parameter("cmd_vel_topic", "/swerve_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("publish_rate", 20.0)
        # SwerveDriveController clamps x/y to 0.1 m/s and yaw to 0.1 rad/s
        # (franka_bringup/config/controllers.yaml), so anything above that is
        # silently limited. Keep the bare-`ros2 run` defaults conservative.
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.05)
        self.declare_parameter("pedal_timeout", 0.5)
        # token -> (axis, sign); axis in {x, y, yaw}
        self.declare_parameter("map_1A_x", 1.0)
        self.declare_parameter("map_2A_x", -1.0)
        self.declare_parameter("map_1B_y", 1.0)
        self.declare_parameter("map_2B_y", -1.0)
        self.declare_parameter("map_1C_yaw", -1.0)
        self.declare_parameter("map_2C_yaw", 1.0)
        # combos that belong to the spine (base is suppressed while held)
        self.declare_parameter("up_combo", ["1A", "2C"])
        self.declare_parameter("down_combo", ["1C", "2A"])

        gp = self.get_parameter
        self.frame_id = gp("frame_id").value
        self.linear_speed = gp("linear_speed").value
        self.angular_speed = gp("angular_speed").value
        self.pedal_timeout = gp("pedal_timeout").value
        self.axis_map = {
            "1A": ("x", gp("map_1A_x").value),
            "2A": ("x", gp("map_2A_x").value),
            "1B": ("y", gp("map_1B_y").value),
            "2B": ("y", gp("map_2B_y").value),
            "1C": ("yaw", gp("map_1C_yaw").value),
            "2C": ("yaw", gp("map_2C_yaw").value),
        }
        self.combos = [set(gp("up_combo").value), set(gp("down_combo").value)]

        self.pressed = set()
        self.last_msg_time = self.get_clock().now()

        self.create_subscription(String, "/pedal/state", self._on_pedal, 10)
        self.pub = self.create_publisher(TwistStamped, gp("cmd_vel_topic").value, 10)
        rate = gp("publish_rate").value
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f"base_bridge -> {gp('cmd_vel_topic').value} "
            f"(v={self.linear_speed} m/s, w={self.angular_speed} rad/s)"
        )

    def _on_pedal(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.pressed = set() if msg.data == "NONE" else set(msg.data.split("+"))

    def _combo_active(self):
        return any(combo and combo.issubset(self.pressed) for combo in self.combos)

    def _publish(self):
        x = y = yaw = 0.0
        # Stop if the pedal publisher went silent, or a spine combo is held.
        stale = (self.get_clock().now() - self.last_msg_time).nanoseconds > (
            self.pedal_timeout * 1e9
        )
        if not stale and not self._combo_active():
            for token in self.pressed:
                axis_sign = self.axis_map.get(token)
                if axis_sign is None:
                    continue
                axis, sign = axis_sign
                if axis == "x":
                    x += sign * self.linear_speed
                elif axis == "y":
                    y += sign * self.linear_speed
                elif axis == "yaw":
                    yaw += sign * self.angular_speed

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = yaw
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
