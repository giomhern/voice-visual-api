#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SmoothAmbientMotion(Node):
    """
    Smooth continuous sway:
      - gentle forward/back motion only (no rotation)
    No pauses. No randomness. Pure fluid motion.
    """

    SWITCH_TO_NAV_SRV = "/switch_to_navigation_mode"

    def __init__(self):
        super().__init__("smooth_ambient_motion")

        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.srv_nav = self.create_client(Trigger, self.SWITCH_TO_NAV_SRV)

        # === Tunable Parameters ===
        self.yaw_amp_deg = 6.0       # how far left/right (degrees)
        self.yaw_period_s = 5.0      # shorter = more lively

        self.fb_amp_m = 0.02         # 2 cm forward/back
        self.fb_period_s = 5.0

        self.update_hz = 30.0
        self.dt = 1.0 / self.update_hz

        # Safety caps
        self.lin_cap = 0.06
        self.ang_cap = 0.25

        self.max_lin_acc = 0.15
        self.max_ang_acc = 0.35

        # Internal
        self.t0 = time.monotonic()
        self.v_prev = 0.0
        self.w_prev = 0.0

        # Switch to navigation mode before publishing cmd_vel
        self._ensure_nav_mode()

        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info("Smooth ambient motion running.")

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

        # Forward/back only — no rotation
        v_cmd = fb_A * fb_w * math.cos(fb_w * t)
        w_cmd = 0.0

        v, w = self._rate_limit(v_cmd, w_cmd)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())
        self.get_logger().info("Ambient motion stopped.")


def main():
    rclpy.init()
    node = SmoothAmbientMotion()

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