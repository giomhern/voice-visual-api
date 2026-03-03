#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SmoothAmbientMotion(Node):
    """
    Smooth continuous sway:
      - left <-> right base rotation
      - gentle forward/back motion
    No pauses. No randomness. Pure fluid motion.
    """

    def __init__(self):
        super().__init__("smooth_ambient_motion")

        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

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

        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info("Smooth ambient motion running.")

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

        # Convert position amplitude -> velocity amplitude
        yaw_A = math.radians(self.yaw_amp_deg)
        yaw_w = 2 * math.pi / self.yaw_period_s

        fb_A = self.fb_amp_m
        fb_w = 2 * math.pi / self.fb_period_s

        # Smooth velocity commands
        w_cmd = yaw_A * yaw_w * math.cos(yaw_w * t)
        v_cmd = fb_A * fb_w * math.cos(fb_w * t + math.pi / 2)

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