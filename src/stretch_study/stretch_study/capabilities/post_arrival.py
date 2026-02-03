# stretch_study/capabilities/post_arrival.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


@dataclass
class FunmapTriggerConfig:
    head_scan_srv: str = "/funmap/trigger_head_scan"
    local_loc_srv: str = "/funmap/trigger_local_localization"
    global_loc_srv: str = "/funmap/trigger_global_localization"
    default_timeout_s: float = 10.0


class FunmapTriggers:
    """Small helper to call FUNMAP Trigger services."""

    def __init__(self, node: Node, cfg: Optional[FunmapTriggerConfig] = None):
        self.node = node
        self.cfg = cfg or FunmapTriggerConfig()

        self._head_scan = node.create_client(Trigger, self.cfg.head_scan_srv)
        self._local_loc = node.create_client(Trigger, self.cfg.local_loc_srv)
        self._global_loc = node.create_client(Trigger, self.cfg.global_loc_srv)

    def _call(self, client, name: str, timeout_s: float) -> bool:
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            self.node.get_logger().warn(f"[FUNMAP] service not available: {name}")
            return False

        req = Trigger.Request()
        fut = client.call_async(req)

        t0 = time.time()
        while time.time() - t0 < float(timeout_s):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if fut.done():
                try:
                    resp = fut.result()
                    ok = bool(resp.success)
                    self.node.get_logger().info(f"[FUNMAP] {name}: success={ok} msg='{resp.message}'")
                    return ok
                except Exception as e:
                    self.node.get_logger().warn(f"[FUNMAP] {name}: call failed: {e}")
                    return False

        self.node.get_logger().warn(f"[FUNMAP] {name}: timed out after {timeout_s:.1f}s")
        return False

    def head_scan(self, timeout_s: float = 15.0) -> bool:
        return self._call(self._head_scan, "trigger_head_scan", timeout_s)

    def local_localize(self, timeout_s: float = 8.0) -> bool:
        return self._call(self._local_loc, "trigger_local_localization", timeout_s)

    def global_localize(self, timeout_s: float = 10.0) -> bool:
        return self._call(self._global_loc, "trigger_global_localization", timeout_s)


@dataclass
class FunmapRotateConfig:
    goal_topic: str = "/goal_pose"        # RViz 2D goal topic you already used
    map_frame: str = "map"
    base_frame: str = "base_footprint"    # can be base_link if that’s what FUNMAP publishes
    arrive_yaw_tol_deg: float = 5.0
    arrive_xy_tol_m: float = 0.25         # rotation goal keeps same x/y, but allow some drift
    tf_timeout_s: float = 2.0


class FunmapRotate:
    """
    Rotate the robot by publishing a goal pose with the same x/y and a new yaw.
    This avoids cmd_vel fighting: FUNMAP controller handles the motion.
    """

    def __init__(self, node: Node, cfg: Optional[FunmapRotateConfig] = None):
        self.node = node
        self.cfg = cfg or FunmapRotateConfig()

        self.goal_pub = node.create_publisher(PoseStamped, self.cfg.goal_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def _lookup_map_to_base(self, timeout_s: float) -> Tuple[float, float, float]:
        """
        Returns (x, y, yaw) of base frame in map frame.
        """
        t0 = time.time()
        last_err = None
        while time.time() - t0 < float(timeout_s):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.cfg.map_frame,
                    self.cfg.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                x = float(t.transform.translation.x)
                y = float(t.transform.translation.y)
                q = t.transform.rotation
                yaw = float(euler_from_quaternion([q.x, q.y, q.z, q.w])[2])
                return x, y, yaw
            except Exception as e:
                last_err = e
                rclpy.spin_once(self.node, timeout_sec=0.05)

        raise RuntimeError(f"TF lookup failed {self.cfg.map_frame}->{self.cfg.base_frame}: {last_err}")

    @staticmethod
    def _wrap_to_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def rotate_relative(self, delta_yaw_rad: float, timeout_s: float = 20.0) -> bool:
        """
        Publish a goal with the same x/y and yaw + delta. Wait until yaw is within tolerance.
        """
        x, y, yaw = self._lookup_map_to_base(timeout_s=self.cfg.tf_timeout_s)
        target_yaw = self._wrap_to_pi(yaw + float(delta_yaw_rad))

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, target_yaw)

        goal = PoseStamped()
        goal.header.frame_id = self.cfg.map_frame
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = float(qx)
        goal.pose.orientation.y = float(qy)
        goal.pose.orientation.z = float(qz)
        goal.pose.orientation.w = float(qw)

        self.node.get_logger().info(
            f"[FUNMAP] rotate_relative: yaw {yaw:.2f} -> {target_yaw:.2f} (delta={delta_yaw_rad:.2f}rad)"
        )
        self.goal_pub.publish(goal)

        yaw_tol = math.radians(float(self.cfg.arrive_yaw_tol_deg))
        xy_tol = float(self.cfg.arrive_xy_tol_m)

        t0 = time.time()
        while time.time() - t0 < float(timeout_s):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                cx, cy, cyaw = self._lookup_map_to_base(timeout_s=0.5)
            except Exception:
                continue

            dyaw = abs(self._wrap_to_pi(cyaw - target_yaw))
            dist = math.hypot(cx - x, cy - y)

            if dyaw <= yaw_tol and dist <= xy_tol:
                self.node.get_logger().info(f"[FUNMAP] rotate_relative done (dyaw={dyaw:.3f} rad, dist={dist:.3f} m)")
                return True

        self.node.get_logger().warn("[FUNMAP] rotate_relative timed out")
        return False