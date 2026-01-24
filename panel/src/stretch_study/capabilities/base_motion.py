from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class BaseMotion:
    """
    BaseMotion with serialized motion execution.

    - Odom updates come from ROS callback (main executor thread).
    - Motion runs in ONE background worker thread at a time.
    - No rclpy.spin_once() inside motion loops.
    """

    def __init__(self, node, cmd_vel_topic="/stretch/cmd_vel", odom_topic="/odom"):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = node.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        # Pose state
        self._lock = threading.Lock()
        self._odom_ready = threading.Event()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Motion params (tune later)
        self.linear_speed = 0.12
        self.angular_speed = 0.6
        self.linear_tol = 0.01
        self.angular_tol = math.radians(2.0)
        self.control_hz = 20.0

        # Serialize motion commands
        self._motion_lock = threading.Lock()

        self._busy = threading.Event()
        self._busy.clear()

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._odom_ready.set()

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _wait_for_odom(self, timeout_s: float = 2.0) -> bool:
        return self._odom_ready.wait(timeout=timeout_s)

    def _get_pose(self):
        with self._lock:
            return self._x, self._y, self._yaw

    # -----------------------------
    # Public API
    # -----------------------------

    def run_sequence_async(self, fn: Callable[[], None]) -> None:
        """
        Run a motion sequence in a single background thread.
        Ensures sequences don't overlap.
        """
        threading.Thread(target=self._run_exclusive, args=(fn,), daemon=True).start()

    def _run_exclusive(self, fn: Callable[[], None]) -> None:
        with self._motion_lock:
            self._busy.set()
            try:
                fn()
            finally:
                self.stop()
                self._busy.clear()


    def is_busy(self) -> bool:
        return self._busy.is_set()

    def wait_until_idle(self, timeout_s: float = 10.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            if not self.is_busy():
                return True
            time.sleep(0.05)
        return not self.is_busy()

    # -----------------------------
    # Blocking primitives (used only inside _run_exclusive)
    # -----------------------------

    def drive_distance(self, dist_m: float, timeout_s: float = 10.0) -> None:
        if dist_m <= 0:
            return
        if not self._wait_for_odom():
            self.node.get_logger().error("[MOTION] No odom received; cannot drive_distance.")
            return

        sx, sy, _ = self._get_pose()
        start_t = time.time()
        dt = 1.0 / self.control_hz

        while True:
            x, y, _ = self._get_pose()
            traveled = math.hypot(x - sx, y - sy)

            if traveled >= dist_m - self.linear_tol:
                self.node.get_logger().info(f"[MOTION] drive_distance done traveled={traveled:.3f} target={dist_m:.3f}")
                break
            if time.time() - start_t > timeout_s:
                self.node.get_logger().warn(f"[MOTION] drive_distance timeout traveled={traveled:.3f} target={dist_m:.3f}")
                break

            msg = Twist()
            msg.linear.x = self.linear_speed
            self.cmd_pub.publish(msg)
            time.sleep(dt)

        self.stop()

    def turn_left_90(self, timeout_s: float = 8.0) -> None:
        self.turn_angle(+math.pi / 2, timeout_s=timeout_s)

    def turn_angle(self, angle_rad: float, timeout_s: float = 8.0) -> None:
        if not self._wait_for_odom():
            self.node.get_logger().error("[MOTION] No odom received; cannot turn_angle.")
            return

        _, _, start_yaw = self._get_pose()
        start_t = time.time()
        dt = 1.0 / self.control_hz

        while True:
            _, _, yaw = self._get_pose()
            delta = wrap_to_pi(yaw - start_yaw)

            if abs(delta - angle_rad) <= self.angular_tol:
                self.node.get_logger().info(f"[MOTION] turn_angle done delta={delta:.3f} target={angle_rad:.3f}")
                break
            if time.time() - start_t > timeout_s:
                self.node.get_logger().warn(f"[MOTION] turn_angle timeout delta={delta:.3f} target={angle_rad:.3f}")
                break

            msg = Twist()
            # Positive angular.z should be CCW in ROS (left turn)
            msg.angular.z = self.angular_speed if angle_rad >= 0 else -self.angular_speed
            self.cmd_pub.publish(msg)
            time.sleep(dt)

        self.stop()