from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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

    Fixes / additions:
      - Supports stand-off distance (social distance) via stand_off_m argument in goto()
      - Logs the effective (offset) goal pose
      - Keeps behavior identical when stand_off_m == 0
    """

    def __init__(self, node, goals_yaml: str):
        self.node = node
        self.nav = BasicNavigator()

        with open(goals_yaml, "r") as f:
            cfg = yaml.safe_load(f) or {}

        self.frame_id = cfg.get("frame_id", "map")
        raw_goals = cfg.get("goals", {}) or {}
        self.goals: Dict[str, Goal] = {k: Goal(**v) for k, v in raw_goals.items()}

        # Optional: initial pose in YAML (recommended if you’re using AMCL)
        self.initial_pose: Optional[Goal] = None
        if "initial_pose" in cfg and isinstance(cfg["initial_pose"], dict):
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
            self.node.get_logger().info(
                f"[NAV] Setting initial pose from YAML x={ip.pose.position.x:.2f} y={ip.pose.position.y:.2f}"
            )
            self.nav.setInitialPose(ip)

        self.node.get_logger().info("[NAV] Waiting for Nav2 to become active...")
        try:
            # nav2_simple_commander blocks until nav2 lifecycle nodes are active
            self.nav.waitUntilNav2Active()
        except Exception as e:
            self.node.get_logger().error(f"[NAV] waitUntilNav2Active failed: {e}")
            return False

        self._activated = True
        self.node.get_logger().info("[NAV] Nav2 is active.")
        return True

    @staticmethod
    def _apply_stand_off(g: Goal, stand_off_m: float) -> Goal:
        """
        Offset goal backwards along goal yaw by stand_off_m.
        If yaw is the desired final facing direction, this makes the robot stop stand_off_m away.
        """
        if not stand_off_m or stand_off_m <= 0.0:
            return g
        ox = float(g.x) - float(stand_off_m) * math.cos(float(g.yaw))
        oy = float(g.y) - float(stand_off_m) * math.sin(float(g.yaw))
        return Goal(x=ox, y=oy, yaw=float(g.yaw))

    def goto(
        self,
        name: str,
        timeout_s: float = 120.0,
        feedback_every_s: float = 2.0,
        stand_off_m: float = 0.0,
    ) -> bool:
        """
        Navigate to a named goal.

        Args:
          name: key in YAML goals map
          timeout_s: cancel after timeout
          feedback_every_s: log feedback interval
          stand_off_m: social-distance stand-off (meters). If >0, offset goal position.

        Returns:
          True on success, False otherwise
        """
        if name not in self.goals:
            self.node.get_logger().error(f"[NAV] Unknown goal '{name}'. Known: {list(self.goals.keys())}")
            return False

        if not self.activate(timeout_s=30.0):
            return False

        base_goal = self.goals[name]
        eff_goal = self._apply_stand_off(base_goal, float(stand_off_m))

        if stand_off_m and stand_off_m > 0.0:
            self.node.get_logger().info(
                f"[NAV] stand_off_m={stand_off_m:.2f} applied for '{name}': "
                f"({base_goal.x:.2f},{base_goal.y:.2f}) -> ({eff_goal.x:.2f},{eff_goal.y:.2f}) yaw={eff_goal.yaw:.2f}"
            )

        goal_pose = self._pose_stamped(eff_goal)
        self.node.get_logger().info(
            f"[NAV] goToPose('{name}') x={goal_pose.pose.position.x:.2f} y={goal_pose.pose.position.y:.2f} frame={self.frame_id}"
        )

        task = self.nav.goToPose(goal_pose)

        last_feedback_t = time.time()
        start_t = time.time()

        while not self.nav.isTaskComplete(task=task):
            feedback = self.nav.getFeedback(task=task)
            now = time.time()

            if feedback and (now - last_feedback_t) >= feedback_every_s:
                last_feedback_t = now
                try:
                    eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                except Exception:
                    eta = float("nan")
                self.node.get_logger().info(
                    f"[NAV] {name}: dist={getattr(feedback, 'distance_remaining', float('nan')):.2f}m "
                    f"track_err={getattr(feedback, 'tracking_error', float('nan')):.2f}m eta={eta:.0f}s"
                )

            if now - start_t > timeout_s:
                self.node.get_logger().warn(f"[NAV] {name}: timeout after {timeout_s:.1f}s, canceling task")
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