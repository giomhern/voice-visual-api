from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class BaseMotion:
    def __init__(self, node, cmd_vel_topic="/stretch/cmd_vel", odom_topic="/odom"):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = node.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )

        self._lock = threading.Lock()
        self._have_odom = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.linear_speed = 0.12
        self.angular_speed = 0.6
        self.linear_tol = 0.01
        self.angular_tol = math.radians(2.0)

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._have_odom = True
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = yaw_from_quat(msg.pose.pose.orientation)

    def stop(self):
        self.cmd_pub.publish(Twist())

    # -----------------------------
    # PUBLIC ASYNC API
    # -----------------------------

    def drive_distance_async(self, dist_m: float):
        threading.Thread(
            target=self._drive_distance_blocking,
            args=(dist_m,),
            daemon=True,
        ).start()

    def turn_left_90_async(self):
        threading.Thread(
            target=self._turn_angle_blocking,
            args=(math.pi / 2,),
            daemon=True,
        ).start()

    # -----------------------------
    # INTERNAL BLOCKING WORKERS
    # -----------------------------

    def _drive_distance_blocking(self, dist_m: float):
        if dist_m <= 0:
            return

        while not self._have_odom:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        with self._lock:
            sx, sy = self._x, self._y

        start = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            with self._lock:
                dx = self._x - sx
                dy = self._y - sy
                traveled = math.hypot(dx, dy)

            if traveled >= dist_m - self.linear_tol:
                break

            if time.time() - start > 10.0:
                break

            msg = Twist()
            msg.linear.x = self.linear_speed
            self.cmd_pub.publish(msg)

        self.stop()

    def _turn_angle_blocking(self, angle_rad: float):
        while not self._have_odom:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        with self._lock:
            start_yaw = self._yaw

        start = time.time()
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            with self._lock:
                delta = wrap_to_pi(self._yaw - start_yaw)

            if abs(delta - angle_rad) <= self.angular_tol:
                break

            if time.time() - start > 8.0:
                break

            msg = Twist()
            msg.angular.z = self.angular_speed
            self.cmd_pub.publish(msg)

        self.stop()