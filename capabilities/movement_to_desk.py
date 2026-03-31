#!/usr/bin/env python3
import argparse
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Speed presets: (max_vx m/s, max_wz rad/s)
SPEED_PRESETS = {
    "slow": (0.08, 0.3),
    "medium": (0.15, 0.6),
    "fast": (0.25, 0.8),
}


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
    def __init__(self, speed: str = "medium"):
        super().__init__("precise_path")

        # Change this if your base cmd_vel topic differs
        self.cmd_vel_topic = "/stretch/cmd_vel"
        self.odom_topic = "/odom"

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 5)

        # Service client to reset odom via mode switch
        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None

        # --- tuning knobs (from speed preset) ---
        vx, wz = SPEED_PRESETS.get(speed, SPEED_PRESETS["medium"])
        self.rate_hz = 30.0
        self.max_wz = wz             # rad/s cap
        self.max_vx = vx             # m/s cap
        self.kp_yaw = 1.8            # proportional gain for turning
        self.kp_dist = 0.8           # proportional gain for driving
        self.yaw_tol = math.radians(1.5)
        self.dist_tol = 0.01         # 1 cm
        self.timeout_s = 30.0        # per-step timeout

        self.steps: List[Step] = [
            Step("turn", math.radians(-39.0)),
            Step("drive", 2.1),
            Step("turn", math.radians(-42.0)),
        ]

        self.get_logger().info(f"Speed: {speed} (vx={vx}, wz={wz})")

        # Reset odom by (re-)entering navigation mode so yaw starts at ~0
        self.get_logger().info("Resetting odom via switch_to_navigation_mode...")
        self._reset_odom()

        self.get_logger().info("Waiting for /odom...")
        self._pose_xy = None
        self._yaw = None
        while rclpy.ok() and (self._pose_xy is None or self._yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Got odom (yaw={math.degrees(self._yaw):.1f}°). Executing steps.")
        self.run_steps()
        self.stop()
        self.get_logger().info("Done.")
        raise SystemExit

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _reset_odom(self):
        """Call switch_to_navigation_mode to reset odom to (0, 0, yaw=0)."""
        if not self.srv_nav.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("switch_to_navigation_mode service not available, skipping odom reset")
            return
        fut = self.srv_nav.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.done():
            resp = fut.result()
            self.get_logger().info(f"Odom reset: success={resp.success} msg='{resp.message}'")
        else:
            self.get_logger().warn("Odom reset timed out")
        time.sleep(0.5)  # let odom republish after reset

    def _drain_odom(self):
        """Drain all pending odom messages to get the freshest reading."""
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.0)
        # One final spin with a short wait to catch the latest publish
        rclpy.spin_once(self, timeout_sec=0.05)

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
        self._drain_odom()
        start = time.time()
        target = wrap_pi(self._yaw + delta_yaw)
        self.get_logger().info(f"Turn: yaw={math.degrees(self._yaw):.1f}° target={math.degrees(target):.1f}° delta={math.degrees(delta_yaw):.1f}°")

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
        self._drain_odom()
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
    parser = argparse.ArgumentParser(description="Precise path movement to desk")
    parser.add_argument(
        "--speed", choices=["slow", "medium", "fast"], default="medium",
        help="Movement speed preset (default: medium)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    try:
        PrecisePath(speed=args.speed)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()