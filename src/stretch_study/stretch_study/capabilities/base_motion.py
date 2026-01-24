from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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


@dataclass
class MotionLimits:
    linear_speed: float = 0.15
    angular_speed: float = 0.6
    linear_tol: float = 0.03
    angular_tol: float = math.radians(3.0)


class BaseMotion:
    """Odom-based relative base motion (drive distance, turn angle).

    This is deterministic and does not require mapping/Nav2.
    """

    def __init__(
        self,
        node: Node,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        limits: Optional[MotionLimits] = None,
    ):
        self.node = node
        self.limits = limits or MotionLimits()
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = node.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._have_odom = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def _on_odom(self, msg: Odometry) -> None:
        self._have_odom = True
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        self._yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def wait_for_odom(self, timeout_s: float = 2.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._have_odom:
                return True
        return False

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def set_speed_profile(self, movement_speed: str) -> None:
        """Update limits based on a symbolic profile."""
        ms = (movement_speed or "medium").lower()
        if ms == "slow":
            self.limits.linear_speed = 0.10
            self.limits.angular_speed = 0.45
        elif ms == "fast":
            self.limits.linear_speed = 0.22
            self.limits.angular_speed = 0.75
        else:
            self.limits.linear_speed = 0.15
            self.limits.angular_speed = 0.60

    def drive_distance(self, dist_m: float, timeout_s: float = 20.0) -> None:
        if abs(dist_m) < 1e-6:
            return
        if not self.wait_for_odom():
            raise RuntimeError("No odometry received; cannot drive distance.")

        start_x, start_y = self._x, self._y
        direction = 1.0 if dist_m >= 0 else -1.0
        speed = direction * abs(self.limits.linear_speed)

        t0 = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            dx = self._x - start_x
            dy = self._y - start_y
            traveled = math.sqrt(dx * dx + dy * dy)

            if traveled >= abs(dist_m) - self.limits.linear_tol:
                break
            if time.time() - t0 > timeout_s:
                self.node.get_logger().warn("drive_distance timeout; stopping")
                break

            msg = Twist()
            msg.linear.x = speed
            self.cmd_pub.publish(msg)

        self.stop()

    def turn_angle(self, angle_rad: float, timeout_s: float = 12.0) -> None:
        if abs(angle_rad) < 1e-6:
            return
        if not self.wait_for_odom():
            raise RuntimeError("No odometry received; cannot turn.")

        start_yaw = self._yaw
        direction = 1.0 if angle_rad >= 0 else -1.0
        target = direction * abs(angle_rad)
        speed = direction * abs(self.limits.angular_speed)

        t0 = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            delta = _wrap_to_pi(self._yaw - start_yaw)

            if abs(delta - target) <= self.limits.angular_tol:
                break
            if time.time() - t0 > timeout_s:
                self.node.get_logger().warn("turn_angle timeout; stopping")
                break

            msg = Twist()
            msg.angular.z = speed
            self.cmd_pub.publish(msg)

        self.stop()
