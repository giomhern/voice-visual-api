#!/usr/bin/env python3
from __future__ import annotations

import math
import time
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class OrganicAmbientMotion(Node):
    """
    Smooth + organic ambient motion.

    Adds:
      - sinusoidal sway
      - slow random amplitude modulation
      - random bias drift
      - tiny spontaneous micro-turns
    """

    def __init__(self):
        super().__init__("organic_ambient_motion")

        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        # === BASE PARAMETERS (safe defaults) ===
        self.yaw_amp_deg = 5.0
        self.yaw_period = 6.0

        self.fb_amp_m = 0.02
        self.fb_period = 5.0

        self.update_hz = 30.0
        self.dt = 1.0 / self.update_hz

        self.lin_cap = 0.06
        self.ang_cap = 0.25

        self.max_lin_acc = 0.15
        self.max_ang_acc = 0.35

        # === Internal state ===
        self.t0 = time.monotonic()
        self.v_prev = 0.0
        self.w_prev = 0.0

        # Organic modifiers
        self.bias = 0.0
        self.bias_target = 0.0
        self.micro_burst_timer = 0.0
        self.micro_burst_strength = 0.0

        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info("Organic ambient motion started.")

    def _rate_limit(self, v_cmd, w_cmd):
        dv_max = self.max_lin_acc * self.dt
        dw_max = self.max_ang_acc * self.dt

        v = self.v_prev + clamp(v_cmd - self.v_prev, -dv_max, dv_max)
        w = self.w_prev + clamp(w_cmd - self.w_prev, -dw_max, dw_max)

        v = clamp(v, -self.lin_cap, self.lin_cap)
        w = clamp(w, -self.ang_cap, self.ang_cap)

        self.v_prev, self.w_prev = v, w
        return v, w

    def _update_bias(self):
        # Occasionally choose new drift target
        if random.random() < 0.005:
            self.bias_target = random.uniform(-0.05, 0.05)

        # Smoothly move toward target
        self.bias += (self.bias_target - self.bias) * 0.01

    def _maybe_trigger_micro_burst(self):
        if self.micro_burst_timer <= 0 and random.random() < 0.003:
            self.micro_burst_timer = random.uniform(0.3, 0.8)
            self.micro_burst_strength = random.uniform(-0.15, 0.15)

    def _update_micro_burst(self):
        if self.micro_burst_timer > 0:
            self.micro_burst_timer -= self.dt
            return self.micro_burst_strength
        return 0.0

    def _tick(self):
        t = time.monotonic() - self.t0

        # Slight random modulation of period
        yaw_period_mod = self.yaw_period + random.uniform(-0.3, 0.3)
        fb_period_mod = self.fb_period + random.uniform(-0.3, 0.3)

        yaw_A = math.radians(self.yaw_amp_deg)
        yaw_w = 2 * math.pi / yaw_period_mod

        fb_A = self.fb_amp_m
        fb_w = 2 * math.pi / fb_period_mod

        # Base smooth motion
        w_cmd = yaw_A * yaw_w * math.cos(yaw_w * t)
        v_cmd = fb_A * fb_w * math.cos(fb_w * t + math.pi / 2)

        # Add organic drift
        self._update_bias()
        w_cmd += self.bias

        # Add spontaneous micro-expression
        self._maybe_trigger_micro_burst()
        w_cmd += self._update_micro_burst()

        # Rate limit + cap
        v, w = self._rate_limit(v_cmd, w_cmd)

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())
        self.get_logger().info("Organic ambient motion stopped.")


def main():
    rclpy.init()
    node = OrganicAmbientMotion()

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