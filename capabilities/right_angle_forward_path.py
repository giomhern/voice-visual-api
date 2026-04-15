#!/usr/bin/env python3
import argparse
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


SPEED_PRESETS = {
    "slow": (0.08, 0.3),
    "medium": (0.15, 0.6),
    "fast": (0.25, 0.8),
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


@dataclass
class Step:
    kind: str
    value: float


class RightAngleForwardPath(Node):
    def __init__(self, speed: str = "medium", turn_degrees: float = -83.0):
        super().__init__("right_angle_forward_path")

        self.cmd_vel_topic = "/stretch/cmd_vel"
        self.odom_topic = "/odom"

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")
        self.srv_stow = self.create_client(Trigger, "/stow_the_robot")

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None

        vx, wz = SPEED_PRESETS.get(speed, SPEED_PRESETS["medium"])
        self.rate_hz = 30.0
        self.max_vx = vx
        self.max_wz = wz
        self.kp_dist = 0.8
        self.kp_yaw = 1.2            # reduced from 1.8 to slow approach
        self.dist_tol = 0.01
        self.yaw_tol = math.radians(1.5)
        self.max_turn_wz = 0.3       # cap angular velocity during turns
        self.timeout_s = 30.0

        self.steps: List[Step] = [
            Step("drive", 0.9144),
            Step("turn", math.radians(turn_degrees)),
            Step("drive", 1.4),
        ]

        self.get_logger().info(
            f"Using speed preset '{speed}' (vx={vx}, wz={wz}), turn={turn_degrees:.1f} deg"
        )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _reset_odom(self):
        if not self.srv_nav.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Navigation mode service unavailable; skipping odom reset")
            return

        future = self.srv_nav.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            response = future.result()
            self.get_logger().info(
                f"Navigation mode: success={response.success} msg='{response.message}'"
            )
        else:
            self.get_logger().warn("Navigation mode request timed out")
        time.sleep(0.5)

    def _stow_robot(self):
        if not self.srv_stow.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Stow service unavailable; skipping stow")
            return

        future = self.srv_stow.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=40.0)
        if future.done():
            response = future.result()
            self.get_logger().info(
                f"Stow: success={response.success} msg='{response.message}'"
            )
        else:
            self.get_logger().warn("Stow request timed out")
        time.sleep(0.5)

    def _wait_for_odom(self):
        self.get_logger().info("Waiting for odometry...")
        while rclpy.ok() and (self._pose_xy is None or self._yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _drain_odom(self):
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.pub.publish(Twist())

    def _run_steps(self):
        for index, step in enumerate(self.steps, start=1):
            self.get_logger().info(
                f"Step {index}/{len(self.steps)}: {step.kind} {step.value:.4f}"
            )
            if step.kind == "drive":
                self._drive_forward(step.value)
            elif step.kind == "turn":
                self._turn_relative(step.value)
            else:
                raise ValueError(f"Unknown step kind: {step.kind}")
            self.stop()
            time.sleep(0.2)

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
            cmd.angular.z = max(-self.max_turn_wz, min(self.max_turn_wz, self.kp_yaw * error))
            self.pub.publish(cmd)
            time.sleep(dt)

    def _drive_forward(self, distance_m: float):
        assert self._pose_xy is not None
        self._drain_odom()
        start = time.time()
        x0, y0 = self._pose_xy
        dt = 1.0 / self.rate_hz

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._pose_xy is not None

            x, y = self._pose_xy
            traveled = math.hypot(x - x0, y - y0)
            remaining = distance_m - traveled

            if remaining <= self.dist_tol:
                return
            if time.time() - start > self.timeout_s:
                self.get_logger().warn("Drive timed out; stopping early.")
                return

            cmd = Twist()
            cmd.linear.x = max(0.0, min(self.max_vx, self.kp_dist * remaining))
            self.pub.publish(cmd)
            time.sleep(dt)

    def run(self) -> int:
        self._stow_robot()
        self._reset_odom()
        self._wait_for_odom()
        self._run_steps()
        self.stop()
        self.get_logger().info("Path complete.")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Drive 0.9144 m, turn right 90 degrees, then drive 1.3969 m."
    )
    parser.add_argument(
        "--speed",
        choices=["slow", "medium", "fast"],
        default="medium",
        help="Movement speed preset",
    )
    parser.add_argument(
        "--turn-deg",
        type=float,
        default=-83.0,
        help="Relative turn in degrees. Negative is right turn. Default: -83.0",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RightAngleForwardPath(speed=args.speed, turn_degrees=args.turn_deg)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
