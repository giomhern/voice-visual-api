from __future__ import annotations 
import math
import yaml 
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

def yaw_to_quat(yaw: float):
    # planar quaternion
    import math
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Goal:
    x: float
    y: float
    yaw: float

class Nav2Navigator:
    def __init__(self, node: Node, action_name: str = "/navigate_to_pose", goals_yaml: str = ""):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)

        with open(goals_yaml, "r") as f:
            cfg = yaml.safe_load(f)
        self.frame_id = cfg.get("frame_id", "map")
        self.goals = {
            k: Goal(**v) for k, v in cfg["goals"].items()
        }

    def wait_ready(self, timeout_s: float = 10.0) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout_s)

    def _pose_stamped(self, g: Goal) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.pose.position.x = float(g.x)
        p.pose.position.y = float(g.y)
        qx, qy, qz, qw = yaw_to_quat(float(g.yaw))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def goto(self, name: str) -> bool:
        if name not in self.goals:
            self.node.get_logger().error(f"[NAV] Unknown goal '{name}'")
            return False

        if not self.wait_ready():
            self.node.get_logger().error("[NAV] NavigateToPose action not available.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(self.goals[name])

        self.node.get_logger().info(f"[NAV] Sending goal '{name}'")
        send_future = self.client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("[NAV] Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        res = result_future.result()
        if res is None:
            self.node.get_logger().error("[NAV] No result")
            return False

        self.node.get_logger().info("[NAV] Goal finished")
        return True