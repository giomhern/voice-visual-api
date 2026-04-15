#!/usr/bin/env python3
import argparse
import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


SPEED_PRESETS = {
    "slow": 0.08,
    "medium": 0.15,
    "fast": 0.25,
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


class DriveStraight(Node):
    def __init__(self, distance_m: float, speed: str = "medium"):
        super().__init__("drive_straight")

        self.distance_m = float(distance_m)
        self.max_vx = SPEED_PRESETS.get(speed, SPEED_PRESETS["medium"])
        self.max_wz = 0.3
        self.rate_hz = 30.0
        self.kp_dist = 0.8
        self.kp_yaw = 1.2
        self.dist_tol = 0.01
        self.yaw_tol = math.radians(1.0)
        self.timeout_s = max(10.0, abs(self.distance_m) / max(self.max_vx, 1e-3) + 10.0)

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None

        self.pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")

        self.get_logger().info(
            f"Driving {self.distance_m:.3f} m with speed preset '{speed}' (vx={self.max_vx:.2f} m/s), "
            "then turning left 90.0 deg"
        )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
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
        while rclpy.ok() and (self._pose_xy is None or self._yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _drain_odom(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.pub.publish(Twist())

    def _turn_relative(self, delta_yaw: float):
        assert self._yaw is not None
        self._drain_odom()
        start = time.time()
        target = wrap_pi(self._yaw + delta_yaw)
        dt = 1.0 / self.rate_hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._yaw is not None

            error = wrap_pi(target - self._yaw)
            if abs(error) <= self.yaw_tol:
                return
            if time.time() - start > self.timeout_s:
                self.get_logger().warn("Turn timed out; stopping early.")
                return

            cmd = Twist()
            cmd.angular.z = max(-self.max_wz, min(self.max_wz, self.kp_yaw * error))
            self.pub.publish(cmd)
            time.sleep(dt)

    def run(self) -> int:
        self._ensure_navigation_mode()
        self._wait_for_odom()

        if abs(self.distance_m) <= self.dist_tol:
            self.get_logger().info("Requested distance is near zero; nothing to do.")
        else:
            self._drain_odom()
            assert self._pose_xy is not None
            x0, y0 = self._pose_xy
            target_dist = abs(self.distance_m)
            direction = 1.0 if self.distance_m >= 0.0 else -1.0
            dt = 1.0 / self.rate_hz
            start = time.time()

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                assert self._pose_xy is not None

                x, y = self._pose_xy
                traveled = math.hypot(x - x0, y - y0)
                remaining = target_dist - traveled

                if remaining <= self.dist_tol:
                    break
                if time.time() - start > self.timeout_s:
                    self.get_logger().warn("Drive timed out; stopping early.")
                    break

                cmd = Twist()
                cmd.linear.x = direction * min(self.max_vx, max(0.03, self.kp_dist * remaining))
                self.pub.publish(cmd)
                time.sleep(dt)

        self.stop()
        time.sleep(0.2)
        self.get_logger().info("Drive complete. Turning left 90 degrees.")
        self._turn_relative(math.pi / 2.0)

        self.stop()
        self.get_logger().info("Drive and turn complete.")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Drive straight for a requested distance in meters, then turn left 90 degrees."
    )
    parser.add_argument("distance_m", type=float, help="Distance to drive in meters. Negative drives backward.", default=2.65)
    parser.add_argument(
        "--speed",
        choices=["slow", "medium", "fast"],
        default="medium",
        help="Movement speed preset",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DriveStraight(distance_m=args.distance_m, speed=args.speed)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
