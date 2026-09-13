"""Native ROS for standalone routes, fixed DDS gateway for Hamburg phases."""
import os

__all__ = ["ros", "Node"]

if os.environ.get("EBIM_ROS_GATEWAY"):
    from franka_duo_tele_data.local_ros import Node
    from franka_duo_tele_data import local_ros as ros
else:
    import rclpy as ros

    def __getattr__(name):
        if name == "Node":
            from rclpy.node import Node
            return Node
        raise AttributeError(name)
