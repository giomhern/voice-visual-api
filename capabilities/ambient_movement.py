#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class AmbientLivelyBase(Node):
    """
    Smooth ambient motion for Stretch base using sinusoids:
      - gentle yaw sway (left-right)
      - gentle forward-back "breathing"
    Publishes Twist to /stretch/cmd_vel.

    Safety:
      - very small amplitudes by default
      - acceleration limiting to avoid jerks
    """

    def __init__(
        self,
        cmd_vel_topic: str = "/stretch/cmd_vel",

        # --- Yaw sway ---
        yaw_amp_deg: float = 6.0,     # +/- degrees of sway (small)
        yaw_period_s: float = 6.0,    # smaller = more lively, larger = calmer

        # --- Forward/back ---
        fb_amp_m: float = 0.03,       # +/- meters of "breathing" (3 cm)
        fb_period_s: float = 5.0,
        fb_phase_deg: float = 90.0,   # phase offset vs yaw; 90° feels natural

        # --- Update + smoothing ---
        update_hz: float = 30.0,
        max_lin_acc: float = 0.15,    # m/s^2
        max_ang_acc: float = 0.35,    # rad/s^2

        # --- Hard safety caps ---
        lin_speed_cap: float = 0.06,  # m/s
        ang_speed_cap: float = 0.25,  # rad/s
    ):
        super().__init__("ambient_lively_base")

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.dt = 1.0 / float(update_hz)

        # Convert “amplitude in position” to “amplitude in velocity”
        # If theta(t) = A * sin(ωt), then theta_dot(t) = A*ω * cos(ωt)
        self.yaw_A = math.radians(yaw_amp_deg)
        self.yaw_w = 2.0 * math.pi / float(yaw_period_s)

        self.fb_A = float(fb_amp_m)
        self.fb_w = 2.0 * math.pi / float(fb_period_s)
        self.fb_phase = math.radians(fb_phase_deg)

        self.max_lin_acc = float(max_lin_acc)
        self.max_ang_acc = float(max_ang_acc)

        self.lin_cap = float(lin_speed_cap)
        self.ang_cap = float(ang_speed_cap)

        self.v_prev = 0.0
        self.w_prev = 0.0

        self.t0 = time.monotonic()

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"Ambient motion: topic={cmd_vel_topic} | "
            f"yaw_amp={yaw_amp_deg}deg period={yaw_period_s}s | "
            f"fb_amp={fb_amp_m}m period={fb_period_s}s | hz={update_hz}"
        )

    def _rate_limit(self, v_cmd: float, w_cmd: float) -> tuple[float, float]:
        # limit acceleration per timestep
        dv_max = self.max_lin_acc * self.dt
        dw_max = self.max_ang_acc * self.dt

        v = self.v_prev + clamp(v_cmd - self.v_prev, -dv_max, dv_max)
        w = self.w_prev + clamp(w_cmd - self.w_prev, -dw_max, dw_max)

        # hard caps
        v = clamp(v, -self.lin_cap, self.lin_cap)
        w = clamp(w, -self.ang_cap, self.ang_cap)

        self.v_prev, self.w_prev = v, w
        return v, w

    def _tick(self):
        t = time.monotonic() - self.t0

        # Smooth commands (velocity space)
        w_cmd = self.yaw_A * self.yaw_w * math.cos(self.yaw_w * t)
        v_cmd = self.fb_A * self.fb_w * math.cos(self.fb_w * t + self.fb_phase)

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
    node = AmbientLivelyBase()

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