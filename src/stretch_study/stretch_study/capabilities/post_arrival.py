# stretch_study/capabilities/post_arrival.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def _yaw_from_quat(q) -> float:
    """Quaternion -> yaw (rotation about Z)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FunmapTriggers:
    """Small helper to call FUNMAP Trigger services."""

    def __init__(self, node: Node):
        self.node = node
        self._head_scan = node.create_client(Trigger, "/funmap/trigger_head_scan")
        self._local_loc = node.create_client(Trigger, "/funmap/trigger_local_localization")
        self._global_loc = node.create_client(Trigger, "/funmap/trigger_global_localization")

    def _call(self, client, name: str, timeout_s: float) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.node.get_logger().warn(f"[FUNMAP] service not available: {name}")
            return False

        req = Trigger.Request()
        fut = client.call_async(req)

        t0 = time.time()
        while time.time() - t0 < timeout_s:
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


class TurnInPlace:
    """Odom-based in-place turning for repeatable 90-degree rotations."""

    def __init__(
        self,
        node: Node,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
    ):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)

        self._have_odom = False
        self._yaw = 0.0
        self._odom_sub = node.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self._have_odom = True
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _wait_for_odom(self, timeout_s: float = 2.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._have_odom:
                return True
        return False

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def turn_left_90(
        self,
        speed_rad_s: float = 0.5,
        timeout_s: float = 10.0,
        tol_deg: float = 3.0,
    ) -> bool:
        """Turn left by +90 degrees (pi/2) using odom yaw delta."""
        if not self._wait_for_odom():
            self.node.get_logger().warn("[TURN] No odom received; cannot do accurate 90deg turn.")
            return False

        start_yaw = float(self._yaw)
        target = math.pi / 2.0
        tol = math.radians(float(tol_deg))

        self.node.get_logger().info("[TURN] Turning left 90 degrees...")

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            delta = _wrap_to_pi(float(self._yaw) - start_yaw)

            if delta >= (target - tol):
                break

            cmd = Twist()
            cmd.angular.z = float(abs(speed_rad_s))  # positive = left
            self.cmd_pub.publish(cmd)

        self.stop()
        self.node.get_logger().info("[TURN] Done.")
        return True