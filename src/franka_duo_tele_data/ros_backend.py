"""Use the fixed Hamburg DDS gateway or the ordinary native ROS runtime."""
import os

__all__ = ["ros", "Node", "ActionClient"]

if os.environ.get("EBIM_ROS_GATEWAY"):
    from . import local_ros as ros
    from .local_ros import ActionClient, Node
else:
    import rclpy as ros
    from rclpy.action import ActionClient
    from rclpy.node import Node
