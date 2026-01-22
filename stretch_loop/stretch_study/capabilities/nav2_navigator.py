from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def yaw_to_quat(yaw: float):
    # planar quaternion
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Goal:
    x: float
    y: float
    yaw: float


class Nav2Navigator:
    """
    Thin wrapper around nav2_simple_commander.BasicNavigator that loads named goals from YAML.
    """

    def __init__(self, node, goals_yaml: str):
        self.node = node
        self.nav = BasicNavigator()

        with open(goals_yaml, "r") as f:
            cfg = yaml.safe_load(f)

        self.frame_id = cfg.get("frame_id", "map")
        self.goals: Dict[str, Goal] = {k: Goal(**v) for k, v in cfg["goals"].items()}

        # Optional: initial pose in YAML (recommended if you’re using AMCL)
        self.initial_pose: Optional[Goal] = None
        if "initial_pose" in cfg:
            self.initial_pose = Goal(**cfg["initial_pose"])

        self._activated = False

    def _pose_stamped(self, g: Goal) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x = float(g.x)
        p.pose.position.y = float(g.y)
        qx, qy, qz, qw = yaw_to_quat(float(g.yaw))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def activate(self, timeout_s: float = 30.0) -> bool:
        """
        Sets initial pose (if provided) and waits for Nav2 to become active.
        """
        if self._activated:
            return True

        if self.initial_pose is not None:
            ip = self._pose_stamped(self.initial_pose)
            self.node.get_logger().info("[NAV] Setting initial pose from YAML")
            self.nav.setInitialPose(ip)

        self.node.get_logger().info("[NAV] Waiting for Nav2 to become active...")
        try:
            # This blocks until nav2 lifecycle nodes are active
            self.nav.waitUntilNav2Active()
        except Exception as e:
            self.node.get_logger().error(f"[NAV] waitUntilNav2Active failed: {e}")
            return False

        self._activated = True
        self.node.get_logger().info("[NAV] Nav2 is active.")
        return True

    def goto(self, name: str, timeout_s: float = 120.0, feedback_every_s: float = 2.0) -> bool:
        if name not in self.goals:
            self.node.get_logger().error(f"[NAV] Unknown goal '{name}'. Known: {list(self.goals.keys())}")
            return False

        if not self.activate():
            return False

        goal_pose = self._pose_stamped(self.goals[name])
        self.node.get_logger().info(f"[NAV] goToPose('{name}') x={goal_pose.pose.position.x:.2f} y={goal_pose.pose.position.y:.2f}")

        task = self.nav.goToPose(goal_pose)

        last_feedback_t = time.time()
        start_t = time.time()

        while not self.nav.isTaskComplete(task=task):
            feedback = self.nav.getFeedback(task=task)
            now = time.time()

            if feedback and (now - last_feedback_t) >= feedback_every_s:
                last_feedback_t = now
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.node.get_logger().info(
                    f"[NAV] {name}: dist={feedback.distance_remaining:.2f}m "
                    f"track_err={feedback.tracking_error:.2f}m eta={eta:.0f}s"
                )

            if now - start_t > timeout_s:
                self.node.get_logger().warn(f"[NAV] {name}: timeout, canceling task")
                self.nav.cancelTask()
                break

            time.sleep(0.05)

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info(f"[NAV] {name}: SUCCEEDED")
            return True
        elif result == TaskResult.CANCELED:
            self.node.get_logger().warn(f"[NAV] {name}: CANCELED")
            return False
        elif result == TaskResult.FAILED:
            try:
                (code, msg) = self.nav.getTaskError()
                self.node.get_logger().error(f"[NAV] {name}: FAILED code={code} msg={msg}")
            except Exception:
                self.node.get_logger().error(f"[NAV] {name}: FAILED (no error detail)")
            return False
        else:
            self.node.get_logger().error(f"[NAV] {name}: INVALID result={result}")
            return False