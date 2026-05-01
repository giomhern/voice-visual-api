#!/usr/bin/env python3
import argparse
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


SPEED_PRESETS = {
    "slow": 0.25,
    "medium": 0.4,
    "fast": 0.6,
}


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class LeftTurn90(Node):
    def __init__(self, speed: str = "medium"):
        super().__init__("left_turn_90")

        self.rate_hz = 30.0
        self.max_wz = SPEED_PRESETS.get(speed, SPEED_PRESETS["medium"])
        self.min_wz = 0.05
        self.kp_yaw = 1.2
        self.yaw_tol = math.radians(1.5)
        self.timeout_s = 20.0

        self._yaw: Optional[float] = None

        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")

        self.get_logger().info(
            f"Ready to turn left 92.0 deg with speed preset '{speed}' "
            f"(max_wz={self.max_wz:.2f} rad/s)"
        )

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _ensure_navigation_mode(self):
        if not self.srv_nav.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/switch_to_navigation_mode service is not available")

        future = self.srv_nav.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError("Timed out switching to navigation mode")

        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            raise RuntimeError(f"Failed to switch to navigation mode: {message}")

        self.get_logger().info(f"Navigation mode ready: {response.message}")
        time.sleep(0.5)

    def _wait_for_odom(self):
        self.get_logger().info("Waiting for odometry...")
        while rclpy.ok() and self._yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _drain_odom(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self, settle_s: float = 0.25):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.05)
        time.sleep(settle_s)

    def turn_left_90(self):
        assert self._yaw is not None

        self._drain_odom()
        assert self._yaw is not None

        delta_yaw = math.radians(92.0)
        start_yaw = self._yaw
        target_yaw = wrap_pi(start_yaw + delta_yaw)
        start_time = time.time()
        dt = 1.0 / self.rate_hz

        self.get_logger().info(
            f"Turning left from {math.degrees(start_yaw):.1f} deg "
            f"to {math.degrees(target_yaw):.1f} deg"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._yaw is not None

            error = wrap_pi(target_yaw - self._yaw)
            if abs(error) <= self.yaw_tol:
                break

            if time.time() - start_time > self.timeout_s:
                self.get_logger().warn("Turn timed out; stopping early.")
                break

            wz = self.kp_yaw * error
            wz = max(-self.max_wz, min(self.max_wz, wz))
            if 0.0 < abs(wz) < self.min_wz:
                wz = math.copysign(self.min_wz, wz)

            cmd = Twist()
            cmd.angular.z = float(wz)
            self.pub.publish(cmd)
            time.sleep(dt)

        self.stop()
        self.get_logger().info("Left turn complete.")

    def run(self) -> int:
        self._ensure_navigation_mode()
        self._wait_for_odom()
        self.turn_left_90()
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Turn the Stretch base left 92 degrees relative to its current orientation."
    )
    parser.add_argument(
        "--speed",
        choices=["slow", "medium", "fast"],
        default="medium",
        help="Turn speed preset",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LeftTurn90(speed=args.speed)
    try:
        rc = node.run()
    finally:
        node.stop(settle_s=0.0)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
