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
    def __init__(self, speed: str = "medium", turn_degrees: float = -70.0,
                 social_offset: float = 0.0):
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
        self.kp_yaw = 0.9            # gentler so we decelerate earlier
        self.dist_tol = 0.01
        self.yaw_tol = math.radians(1.5)
        self.max_turn_wz = 0.25      # cap angular velocity during turns
        self.min_turn_wz = 0.05      # floor to avoid stalling near target
        self.timeout_s = 30.0

        # Second drive leg: full 1.5m minus the social distance offset,
        # so the controller can stop short of the demo position.
        second_leg = max(0.3, 1.5 - max(0.0, social_offset))

        self.steps: List[Step] = [
            Step("drive", 0.9144),
            Step("turn", math.radians(turn_degrees)),
            Step("drive", second_leg),
        ]

        self.get_logger().info(
            f"social_offset={social_offset:.2f}m → second drive leg {second_leg:.2f}m"
        )

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

        # Slow-zone: start decelerating when this close to target (radians).
        # Prevents overshoot from rotational momentum.
        slow_zone = math.radians(12.0)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._yaw is not None

            error = wrap_pi(target - self._yaw)
            if abs(error) <= self.yaw_tol:
                break
            if time.time() - start > self.timeout_s:
                self.get_logger().warn("Turn timed out; stopping early.")
                break

            # Cap by proportional term, then further taper inside slow zone.
            raw = self.kp_yaw * error
            if abs(error) < slow_zone:
                raw *= max(0.35, abs(error) / slow_zone)
            cmd_wz = max(-self.max_turn_wz, min(self.max_turn_wz, raw))
            # Keep at least min speed so we don't stall right at the edge
            if 0 < abs(cmd_wz) < self.min_turn_wz:
                cmd_wz = math.copysign(self.min_turn_wz, cmd_wz)

            cmd = Twist()
            cmd.angular.z = cmd_wz
            self.pub.publish(cmd)
            time.sleep(dt)

        # Active brake: publish zero repeatedly and monitor for overshoot.
        # If momentum carries us past target, apply a short counter-torque.
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.03)
            rclpy.spin_once(self, timeout_sec=0.0)

        # One-shot counter-torque if we drifted past target
        err = wrap_pi(target - self._yaw)
        if abs(err) > self.yaw_tol * 2:
            direction = 1.0 if err > 0 else -1.0
            cmd = Twist()
            cmd.angular.z = direction * self.min_turn_wz * 1.5
            brake_end = time.time() + min(0.5, abs(err) / self.min_turn_wz)
            while time.time() < brake_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                err = wrap_pi(target - self._yaw)
                if abs(err) <= self.yaw_tol:
                    break
                self.pub.publish(cmd)
                time.sleep(dt)
            self.pub.publish(Twist())

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
        description="Drive 0.9144 m, turn by --turn-deg, then drive 1.3969 m."
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
        default=-85.0,
        help="Relative turn in degrees. Negative is right turn. Default: -85.0",
    )
    parser.add_argument(
        "--social-offset",
        type=float,
        default=0.0,
        help="Stop the final drive leg this many meters short of the nominal "
             "demo position (used to honor social distance). Default: 0.0",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RightAngleForwardPath(
        speed=args.speed,
        turn_degrees=args.turn_deg,
        social_offset=args.social_offset,
    )
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
