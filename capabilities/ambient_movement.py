#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SmoothAmbientMotion(Node):
    """
    Smooth continuous sway:
      - gentle forward/back motion only (no rotation)
    No pauses. No randomness. Pure fluid motion.
    """

    SWITCH_TO_NAV_SRV = "/switch_to_navigation_mode"

    def __init__(self, skip_nav_switch: bool = False):
        super().__init__("smooth_ambient_motion")

        self.skip_nav_switch = skip_nav_switch
        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.srv_nav = self.create_client(Trigger, self.SWITCH_TO_NAV_SRV)

        # === Tunable Parameters ===
        self.fb_amp_m = 0.02         # 2 cm forward/back
        self.fb_period_s = 5.0

        self.update_hz = 30.0
        self.dt = 1.0 / self.update_hz

        # Safety caps
        self.lin_cap = 0.06
        self.ang_cap = 0.12

        self.max_lin_acc = 0.15
        self.max_ang_acc = 0.25
        self.kp_yaw = 1.5

        # Internal
        self.t0 = time.monotonic()
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.current_yaw = None
        self.target_yaw = None

        # Switch to navigation mode before publishing cmd_vel
        if not self.skip_nav_switch:
            self._ensure_nav_mode()
        self._wait_for_odom()

        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info("Smooth ambient motion running.")

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

    def _ensure_nav_mode(self):
        """Switch to navigation mode (required for cmd_vel to work)."""
        if not self.srv_nav.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"{self.SWITCH_TO_NAV_SRV} not available — cmd_vel may not work"
            )
            return

        fut = self.srv_nav.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

        if fut.done():
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"Navigation mode: success={resp.success} msg='{resp.message}'"
                )
            except Exception as e:
                self.get_logger().error(f"Navigation mode switch failed: {e}")
        else:
            self.get_logger().error("Navigation mode switch timed out")

    def _wait_for_odom(self):
        start = time.monotonic()
        while rclpy.ok() and self.current_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > 5.0:
                self.get_logger().warn("No odometry received yet; proceeding without heading lock.")
                return

    def _rate_limit(self, v_cmd, w_cmd):
        dv_max = self.max_lin_acc * self.dt
        dw_max = self.max_ang_acc * self.dt

        v = self.v_prev + clamp(v_cmd - self.v_prev, -dv_max, dv_max)
        w = self.w_prev + clamp(w_cmd - self.w_prev, -dw_max, dw_max)

        v = clamp(v, -self.lin_cap, self.lin_cap)
        w = clamp(w, -self.ang_cap, self.ang_cap)

        self.v_prev, self.w_prev = v, w
        return v, w

    def _tick(self):
        t = time.monotonic() - self.t0

        fb_A = self.fb_amp_m
        fb_w = 2 * math.pi / self.fb_period_s

        # Oscillate forward/back and hold the initial heading to avoid drift.
        v_cmd = fb_A * fb_w * math.cos(fb_w * t)
        if self.current_yaw is not None and self.target_yaw is not None:
            yaw_error = wrap_pi(self.target_yaw - self.current_yaw)
            w_cmd = clamp(self.kp_yaw * yaw_error, -self.ang_cap, self.ang_cap)
        else:
            w_cmd = 0.0

        v, w = self._rate_limit(v_cmd, w_cmd)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        try:
            self.pub.publish(Twist())
            self.get_logger().info("Ambient motion stopped.")
        except Exception:
            pass  # Context may already be shut down after SIGINT


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-nav-switch", action="store_true",
        help="Skip the /switch_to_navigation_mode call (use when driver is already in nav mode)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SmoothAmbientMotion(skip_nav_switch=args.skip_nav_switch)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
