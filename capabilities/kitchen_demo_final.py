#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def duration_msg(duration_s: float) -> Duration:
    sec = int(duration_s)
    nanosec = int((duration_s - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Config:
    cmd_vel_topic: str = "/stretch/cmd_vel"
    switch_to_nav_srv: str = "/switch_to_navigation_mode"
    switch_to_pos_srv: str = "/switch_to_position_mode"
    stow_srv: str = "/stow_the_robot"
    traj_action_name: str = "/stretch_controller/follow_joint_trajectory"

    traj_server_wait_s: float = 5.0
    traj_timeout_s: float = 35.0

    first_lift_m: float = 0.9244683262494358
    first_wrist_yaw_rad: float = 0.09139968861151945
    first_wrist_pitch_rad: float = -0.6273981422452273
    first_wrist_roll_rad: float = -0.0015339807878856412
    head_pan_rad: float = -0.026346416025597612
    head_tilt_rad: float = 0.051695294389011916

    arm_extension_m: float = 4.0 * 0.04140732084546477
    gripper_open_pos: float = 0.5339577172721249
    gripper_closed_pos: float = 0.0
    second_lift_m: float = 0.844288316664387


class KitchenDemoFinal(Node):
    def __init__(self, cfg: Config):
        super().__init__("kitchen_demo_final")
        self.cfg = cfg

        self.cmd_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)
        self.srv_nav = self.create_client(Trigger, cfg.switch_to_nav_srv)
        self.srv_pos = self.create_client(Trigger, cfg.switch_to_pos_srv)
        self.srv_stow = self.create_client(Trigger, cfg.stow_srv)
        self.traj_client = ActionClient(self, FollowJointTrajectory, cfg.traj_action_name)

        # Odom subscription — used by the in-place 92° turn at the start
        # of the kitchen demo (after the operator finishes teleop, the
        # robot is still pointed in the bed→kitchen approach direction
        # and needs to face the counter before grasping).
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._yaw: Optional[float] = None

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

    def _call_trigger(self, client, name: str, wait_s: float = 5.0, timeout_s: float = 12.0) -> bool:
        if not client.wait_for_service(timeout_sec=float(wait_s)):
            self.get_logger().error(f"[SRV] {name} not available")
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(timeout_s))
        if not future.done():
            self.get_logger().error(f"[SRV] {name} timed out after {timeout_s:.1f}s")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(f"[SRV] {name} returned no response")
            return False

        ok = bool(response.success)
        self.get_logger().info(f"[SRV] {name}: success={ok} msg='{response.message}'")
        time.sleep(0.25)
        return ok

    def _ensure_position_mode(self) -> bool:
        return self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, timeout_s=12.0)

    def _ensure_navigation_mode(self) -> bool:
        return self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, timeout_s=12.0)

    def _stow_robot(self) -> bool:
        return self._call_trigger(self.srv_stow, self.cfg.stow_srv, wait_s=5.0, timeout_s=45.0)

    def _stop_base(self):
        stop_msg = Twist()
        for _ in range(6):
            self.cmd_pub.publish(stop_msg)
            time.sleep(0.05)

    def _turn_left_92(self,
                      max_wz: float = 0.4,
                      min_wz: float = 0.05,
                      kp_yaw: float = 1.2,
                      yaw_tol_deg: float = 1.5,
                      timeout_s: float = 20.0) -> bool:
        """Turn the base 92° left, closed-loop on /odom yaw."""
        if not self._ensure_navigation_mode():
            self.get_logger().error("[TURN] Cannot switch to navigation mode")
            return False

        # Wait for odom to arrive
        t0 = time.time()
        while self._yaw is None and rclpy.ok() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._yaw is None:
            self.get_logger().error("[TURN] No odometry received; cannot turn")
            return False

        # Drain any backlog so start_yaw reflects current pose
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.0)
        rclpy.spin_once(self, timeout_sec=0.05)

        start_yaw = self._yaw
        target_yaw = _wrap_pi(start_yaw + math.radians(92.0))
        yaw_tol = math.radians(yaw_tol_deg)
        rate_dt = 1.0 / 30.0

        self.get_logger().info(
            f"[TURN] Turning left 92° ({math.degrees(start_yaw):.1f}° → "
            f"{math.degrees(target_yaw):.1f}°)"
        )

        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self._yaw is None:
                time.sleep(rate_dt)
                continue

            error = _wrap_pi(target_yaw - self._yaw)
            if abs(error) <= yaw_tol:
                break
            if time.time() - t0 > timeout_s:
                self.get_logger().warn("[TURN] Timed out; stopping early.")
                break

            wz = max(-max_wz, min(max_wz, kp_yaw * error))
            if 0.0 < abs(wz) < min_wz:
                wz = math.copysign(min_wz, wz)

            cmd = Twist()
            cmd.angular.z = float(wz)
            self.cmd_pub.publish(cmd)
            time.sleep(rate_dt)

        # Stop
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)
        time.sleep(0.2)
        self.get_logger().info("[TURN] Left turn complete.")
        return True

    def _send_traj(self, joint_names: List[str], positions: List[float], duration_s: float) -> bool:
        if len(joint_names) != len(positions):
            self.get_logger().error("[TRAJ] joint_names/positions mismatch")
            return False

        if not self.traj_client.wait_for_server(timeout_sec=float(self.cfg.traj_server_wait_s)):
            self.get_logger().error(f"[TRAJ] Action not available: {self.cfg.traj_action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = duration_msg(duration_s)
        goal.trajectory.points = [point]

        self.get_logger().info(f"[TRAJ] Sending {joint_names} -> {positions}")
        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=float(self.cfg.traj_timeout_s))
        if not send_future.done():
            self.get_logger().error("[TRAJ] send_goal timed out")
            return False

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("[TRAJ] goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=float(self.cfg.traj_timeout_s))
        if not result_future.done():
            self.get_logger().error("[TRAJ] result timed out")
            return False

        result = result_future.result().result
        ok = int(result.error_code) == 0
        err_str = getattr(result, "error_string", "")
        self.get_logger().info(f"[TRAJ] done ok={ok} error_code={result.error_code} err='{err_str}'")
        time.sleep(0.25)
        return ok

    def _go_to_first_pose(self) -> bool:
        self.get_logger().info("[STEP 1A] Lift to first measured height")
        if not self._send_traj(["joint_lift"], [self.cfg.first_lift_m], duration_s=2.5):
            return False

        self.get_logger().info("[STEP 1B] Position wrist and head from measured configuration")
        return self._send_traj(
            [
                "joint_wrist_yaw",
                "joint_wrist_pitch",
                "joint_wrist_roll",
                "joint_head_pan",
                "joint_head_tilt",
            ],
            [
                self.cfg.first_wrist_yaw_rad,
                self.cfg.first_wrist_pitch_rad,
                self.cfg.first_wrist_roll_rad,
                self.cfg.head_pan_rad,
                self.cfg.head_tilt_rad,
            ],
            duration_s=2.0,
        )

    def _extend_arm(self) -> bool:
        self.get_logger().info("[STEP 2] Extend arm to measured extension")
        return self._send_traj(["wrist_extension"], [self.cfg.arm_extension_m], duration_s=1.8)

    def _open_gripper(self) -> bool:
        self.get_logger().info("[STEP 3] Open gripper fully")
        return self._send_traj(
            ["joint_gripper_finger_left"],
            [self.cfg.gripper_open_pos],
            duration_s=1.2,
        )

    def _lower_lift(self) -> bool:
        self.get_logger().info("[STEP 4] Move lift to second measured height")
        return self._send_traj(["joint_lift"], [self.cfg.second_lift_m], duration_s=1.8)

    def _close_gripper_and_retract(self) -> bool:
        self.get_logger().info("[STEP 5A] Close gripper")
        if not self._send_traj(
            ["joint_gripper_finger_left"],
            [self.cfg.gripper_closed_pos],
            duration_s=1.2,
        ):
            return False

        self.get_logger().info("[STEP 5B] Retract arm fully")
        return self._send_traj(["wrist_extension"], [0.0], duration_s=2.0)

    def _teleop_takeover(self):
        self.get_logger().info("[STEP 6] Handing over to teleop in position mode")
        self._stop_base()
        if not self._ensure_position_mode():
            return False

        print("")
        print("Kitchen demo automatic sequence complete.")
        print("Robot is in position mode for teleop.")
        print("Use keyboard teleop in another terminal if needed:")
        print("  ros2 run stretch_core keyboard_teleop")
        print("Press Enter here when teleop is done.")
        input()
        return True

    def run(self) -> int:
        self.get_logger().info("[PIPE] 0) Turn 92° left (post-teleop) toward the kitchen counter")
        if not self._turn_left_92():
            return 1

        self.get_logger().info("[PIPE] 1) stow robot first")
        if not self._stow_robot():
            return 2

        self.get_logger().info("[PIPE] switch_to_position_mode for arm/wrist/gripper commands")
        if not self._ensure_position_mode():
            return 3

        if not self._go_to_first_pose():
            return 4

        if not self._extend_arm():
            return 5

        if not self._open_gripper():
            return 6

        if not self._lower_lift():
            return 7

        if not self._close_gripper_and_retract():
            return 8

        if not self._teleop_takeover():
            return 9

        self.get_logger().info("[DONE] Kitchen demo sequence complete")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Kitchen demo final sequence: stow, pose, extend, open, lower, close/retract, teleop handoff."
    )
    _, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = KitchenDemoFinal(Config())
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        node._stop_base()
        node._ensure_position_mode()
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
