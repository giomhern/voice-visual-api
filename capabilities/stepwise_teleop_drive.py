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


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StepwiseTeleopDrive(Node):
    def __init__(
        self,
        step_distance_m: float,
        steps: int,
        speed_mps: float,
        teleop_mode: str,
        final_mode: str,
    ):
        super().__init__("stepwise_teleop_drive")

        self.step_distance_m = float(step_distance_m)
        self.steps = int(steps)
        self.speed_mps = max(0.02, float(speed_mps))
        self.teleop_mode = teleop_mode
        self.final_mode = final_mode

        self.rate_hz = 30.0
        self.dist_tol = 0.01
        self.kp_dist = 0.8
        self.step_timeout_s = max(8.0, abs(self.step_distance_m) / self.speed_mps + 8.0)

        self._pose_xy: Optional[Tuple[float, float]] = None
        self._yaw: Optional[float] = None

        self.cmd_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.srv_nav = self.create_client(Trigger, "/switch_to_navigation_mode")
        self.srv_pos = self.create_client(Trigger, "/switch_to_position_mode")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _call_trigger(self, client, name: str, timeout_s: float = 12.0) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} service is not available")
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            self.get_logger().error(f"{name} timed out")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(f"{name} returned no response")
            return False

        self.get_logger().info(f"{name}: success={response.success} msg='{response.message}'")
        return bool(response.success)

    def _ensure_navigation_mode(self) -> bool:
        ok = self._call_trigger(self.srv_nav, "/switch_to_navigation_mode")
        if ok:
            time.sleep(0.3)
        return ok

    def _ensure_position_mode(self) -> bool:
        ok = self._call_trigger(self.srv_pos, "/switch_to_position_mode")
        if ok:
            time.sleep(0.3)
        return ok

    def _wait_for_odom(self) -> None:
        self.get_logger().info("Waiting for odometry...")
        start = time.time()
        while rclpy.ok() and (self._pose_xy is None or self._yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                raise RuntimeError("Timed out waiting for /odom")

    def _drain_odom(self) -> None:
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_stop(self, repeats: int = 6, sleep_s: float = 0.05) -> None:
        stop_msg = Twist()
        for _ in range(repeats):
            self.cmd_pub.publish(stop_msg)
            time.sleep(sleep_s)

    def _drive_step(self, step_index: int) -> bool:
        if not self._ensure_navigation_mode():
            return False

        self._drain_odom()
        if self._pose_xy is None:
            self.get_logger().error("No odometry available before driving")
            return False

        x0, y0 = self._pose_xy
        direction = 1.0 if self.step_distance_m >= 0.0 else -1.0
        target_dist = abs(self.step_distance_m)
        dt = 1.0 / self.rate_hz
        start = time.time()

        self.get_logger().info(
            f"Step {step_index}/{self.steps}: driving {self.step_distance_m:.3f} m"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._pose_xy is None:
                continue

            x, y = self._pose_xy
            traveled = math.hypot(x - x0, y - y0)
            remaining = target_dist - traveled

            if remaining <= self.dist_tol:
                break
            if time.time() - start > self.step_timeout_s:
                self.get_logger().warn("Drive step timed out; stopping early.")
                break

            cmd = Twist()
            cmd.linear.x = direction * min(self.speed_mps, max(0.03, self.kp_dist * remaining))
            self.cmd_pub.publish(cmd)
            time.sleep(dt)

        self._publish_stop()
        time.sleep(0.2)
        return True

    def _prompt_for_manual_teleop(self, step_index: int) -> bool:
        self._publish_stop()
        if self.teleop_mode == "position":
            if not self._ensure_position_mode():
                return False
        elif self.teleop_mode == "navigation":
            if not self._ensure_navigation_mode():
                return False

        print("")
        print(f"Step {step_index} complete.")
        print(f"Robot is in {self.teleop_mode} mode for teleop.")
        print("Use teleop in another terminal if needed, for example:")
        if self.teleop_mode == "position":
            print("  ros2 run stretch_core keyboard_teleop")
        else:
            print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/stretch/cmd_vel")
        print("Press Enter here when you are ready for the next automatic 0.1 m move.")
        input()

        if not self._ensure_navigation_mode():
            return False
        self._publish_stop()
        return True

    def _set_final_mode(self) -> None:
        if self.final_mode == "position":
            self._publish_stop()
            self._ensure_position_mode()
        elif self.final_mode == "navigation":
            self._publish_stop()
            self._ensure_navigation_mode()
        else:
            self._publish_stop()

    def run(self) -> int:
        self._wait_for_odom()

        for step_index in range(1, self.steps + 1):
            if not self._drive_step(step_index):
                self._set_final_mode()
                return 1

            if step_index < self.steps:
                if not self._prompt_for_manual_teleop(step_index):
                    self._set_final_mode()
                    return 1

        self.get_logger().info("Sequence complete.")
        self._set_final_mode()
        return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Drive the Stretch base forward in fixed increments, pausing between moves "
            "so the operator can use keyboard teleop and press Enter to continue."
        )
    )
    parser.add_argument(
        "--step-distance",
        type=float,
        default=0.1,
        help="Distance in meters for each automatic move. Default: 0.1",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=2,
        help="Number of automatic move steps to run. Default: 2",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.08,
        help="Linear speed in m/s during automatic moves. Default: 0.08",
    )
    parser.add_argument(
        "--teleop-mode",
        choices=["position", "navigation"],
        default="position",
        help="Mode to switch to during the manual teleop pause. Default: position",
    )
    parser.add_argument(
        "--final-mode",
        choices=["leave", "navigation", "position"],
        default="leave",
        help="Mode to leave Stretch in after the final step. Default: leave",
    )
    args, ros_args = parser.parse_known_args()

    if args.steps < 1:
        raise SystemExit("--steps must be at least 1")

    rclpy.init(args=ros_args)
    node = StepwiseTeleopDrive(
        step_distance_m=args.step_distance,
        steps=args.steps,
        speed_mps=args.speed,
        teleop_mode=args.teleop_mode,
        final_mode=args.final_mode,
    )
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        node._publish_stop()
        node._set_final_mode()
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
