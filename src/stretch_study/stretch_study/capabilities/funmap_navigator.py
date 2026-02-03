from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Goal:
    x: float
    y: float
    yaw: float


class FunmapNavigator:
    """
    FUNMAP navigator wrapper.

    - Publishes PoseStamped goals to /goal_pose (same as RViz "2D Goal Pose")
    - Waits for "arrival" by measuring TF distance map->base_link (or base_footprint)
    - Loads named goals from YAML just like your old Nav2Navigator
    - Supports stand_off_m (social_distance)
    """

    def __init__(
        self,
        node,
        goals_yaml: str,
        goal_topic: str = "/goal_pose",
        base_frame: str = "base_link",
    ):
        self.node = node
        self.goal_topic = goal_topic
        self.base_frame = base_frame

        # publisher for RViz-style goals
        self.pub_goal = node.create_publisher(PoseStamped, goal_topic, 10)

        # TF listener for arrival checks
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        with open(goals_yaml, "r") as f:
            cfg = yaml.safe_load(f) or {}

        self.frame_id = cfg.get("frame_id", "map")
        raw_goals = cfg.get("goals", {}) or {}
        self.goals: Dict[str, Goal] = {k: Goal(**v) for k, v in raw_goals.items()}

        # FUNMAP doesn't need nav2 lifecycle activation, but we keep API parity.
        self._ready = True

    def activate(self, timeout_s: float = 3.0) -> bool:
        # Optional: verify TF exists so we can compute arrival
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                _ = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    self.base_frame,
                    rclpy.time.Time(),
                )
                return True
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().warn(
            f"[FUNMAP] TF not ready for {self.frame_id} -> {self.base_frame}. "
            "If your base frame is base_footprint, set nav.base_frame:=base_footprint"
        )
        return False

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

    @staticmethod
    def _apply_stand_off(g: Goal, stand_off_m: float) -> Goal:
        if not stand_off_m or stand_off_m <= 0.0:
            return g
        ox = float(g.x) - float(stand_off_m) * math.cos(float(g.yaw))
        oy = float(g.y) - float(stand_off_m) * math.sin(float(g.yaw))
        return Goal(x=ox, y=oy, yaw=float(g.yaw))

    def _get_robot_xy(self) -> Optional[tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                rclpy.time.Time(),
            )
            return (float(tf.transform.translation.x), float(tf.transform.translation.y))
        except Exception:
            return None

    def goto(
        self,
        name: str,
        timeout_s: float = 120.0,
        feedback_every_s: float = 2.0,
        stand_off_m: float = 0.0,
        arrive_dist_m: float = 0.35,
    ) -> bool:
        if name not in self.goals:
            self.node.get_logger().error(f"[FUNMAP] Unknown goal '{name}'. Known: {list(self.goals.keys())}")
            return False

        if not self.activate(timeout_s=3.0):
            # still allow goal publish, but arrival detection may not work
            self.node.get_logger().warn("[FUNMAP] Continuing anyway (goal will be published).")

        base_goal = self.goals[name]
        eff_goal = self._apply_stand_off(base_goal, float(stand_off_m))

        if stand_off_m and stand_off_m > 0.0:
            self.node.get_logger().info(
                f"[FUNMAP] stand_off_m={stand_off_m:.2f} applied for '{name}': "
                f"({base_goal.x:.2f},{base_goal.y:.2f}) -> ({eff_goal.x:.2f},{eff_goal.y:.2f}) yaw={eff_goal.yaw:.2f}"
            )

        goal_pose = self._pose_stamped(eff_goal)
        self.node.get_logger().info(
            f"[FUNMAP] Publishing goal '{name}' to {self.goal_topic}: "
            f"x={goal_pose.pose.position.x:.2f} y={goal_pose.pose.position.y:.2f} frame={self.frame_id}"
        )
        self.pub_goal.publish(goal_pose)

        # Wait for arrival by distance in map frame
        start_t = time.time()
        last_feedback_t = 0.0

        while time.time() - start_t < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            robot_xy = self._get_robot_xy()
            if robot_xy is None:
                continue

            dx = robot_xy[0] - float(eff_goal.x)
            dy = robot_xy[1] - float(eff_goal.y)
            dist = math.hypot(dx, dy)

            now = time.time()
            if now - last_feedback_t >= feedback_every_s:
                last_feedback_t = now
                self.node.get_logger().info(f"[FUNMAP] {name}: dist={dist:.2f}m")

            if dist <= float(arrive_dist_m):
                self.node.get_logger().info(f"[FUNMAP] {name}: ARRIVED (dist={dist:.2f}m)")
                return True

        self.node.get_logger().warn(f"[FUNMAP] {name}: TIMEOUT after {timeout_s:.1f}s")
        return False
    

    def _get_current_pose_map(self, timeout_s: float = 1.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                t = self.tf_buffer.lookup_transform("map", self.base_frame, rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                return x, y, yaw
            except Exception:
                time.sleep(0.05)
        raise RuntimeError("Could not lookup map->base transform")

    def rotate_relative(self, delta_yaw_rad: float, timeout_s: float = 20.0) -> bool:
        x, y, yaw = self._get_current_pose_map(timeout_s=2.0)
        target_yaw = yaw + float(delta_yaw_rad)

        q = quaternion_from_euler(0.0, 0.0, target_yaw)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.node.get_logger().info(f"[FUNMAP] rotate_relative: yaw {yaw:.2f} -> {target_yaw:.2f}")
        self.goal_pub.publish(msg)

        # Then just reuse your existing "wait until arrived" logic if you have one.
        # If your goto() already waits for arrival via TF distance, call it here:
        return self._wait_until_arrived_xy(x, y, timeout_s=timeout_s)