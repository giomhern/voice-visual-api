#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class Step:
    kind: str  # "turn" or "drive"
    value: float  # radians for turn, meters for drive


class PrecisePath(Node):
    def __init__(self):
        super().__init__("precise_path")

        # Change this if your base cmd_vel topic differs
        self.cmd_vel_topic = "/stretch/cmd_vel"
        self.odom_topic = "/odom"

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 50)

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None

        # --- tuning knobs ---
        self.rate_hz = 30.0
        self.max_wz = 0.6          # rad/s cap
        self.max_vx = 0.25         # m/s cap
        self.kp_yaw = 1.8          # proportional gain for turning
        self.kp_dist = 0.8         # proportional gain for driving
        self.yaw_tol = math.radians(1.5)
        self.dist_tol = 0.01       # 1 cm
        self.timeout_s = 20.0      # per-step timeout

        # Example hardcoded script:
        # - rotate left 30 degrees
        # - drive forward 0.75 m
        # - rotate right 30 degrees (back to original heading)
        self.steps: List[Step] = [
            Step("turn", math.radians(+30.0)),
            Step("drive", 0.75),
            Step("turn", math.radians(-30.0)),
        ]

        self.get_logger().info("Waiting for /odom...")
        while rclpy.ok() and (self._pose_xy is None or self._yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Got odom. Executing steps.")
        self.run_steps()
        self.stop()
        self.get_logger().info("Done.")
        raise SystemExit

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def stop(self):
        self.pub.publish(Twist())

    def run_steps(self):
        for i, s in enumerate(self.steps):
            self.get_logger().info(f"Step {i+1}/{len(self.steps)}: {s.kind} {s.value:.3f}")
            if s.kind == "turn":
                self.turn_relative(s.value)
            elif s.kind == "drive":
                self.drive_forward(s.value)
            else:
                raise ValueError(f"Unknown step kind: {s.kind}")
            self.stop()
            time.sleep(0.2)  # small settle

    def turn_relative(self, delta_yaw: float):
        assert self._yaw is not None
        start = time.time()
        target = wrap_pi(self._yaw + delta_yaw)

        dt = 1.0 / self.rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._yaw is not None

            err = wrap_pi(target - self._yaw)
            if abs(err) <= self.yaw_tol:
                break
            if time.time() - start > self.timeout_s:
                self.get_logger().warn("Turn timeout; stopping.")
                break

            wz = max(-self.max_wz, min(self.max_wz, self.kp_yaw * err))
            cmd = Twist()
            cmd.angular.z = float(wz)
            self.pub.publish(cmd)
            time.sleep(dt)

    def drive_forward(self, distance_m: float):
        assert self._pose_xy is not None and self._yaw is not None
        start = time.time()
        x0, y0 = self._pose_xy
        target_dist = abs(distance_m)
        direction = 1.0 if distance_m >= 0.0 else -1.0

        dt = 1.0 / self.rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._pose_xy is not None

            x, y = self._pose_xy
            traveled = math.hypot(x - x0, y - y0)
            remaining = target_dist - traveled

            if remaining <= self.dist_tol:
                break
            if time.time() - start > self.timeout_s:
                self.get_logger().warn("Drive timeout; stopping.")
                break

            vx = direction * max(-self.max_vx, min(self.max_vx, self.kp_dist * remaining))
            cmd = Twist()
            cmd.linear.x = float(vx)
            self.pub.publish(cmd)
            time.sleep(dt)


def main():
    rclpy.init()
    try:
        PrecisePath()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()