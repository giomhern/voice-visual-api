#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


@dataclass
class Config:
    cmd_vel_topic: str = "/stretch/cmd_vel"

    # Mode switch services
    switch_to_nav_srv: str = "/switch_to_navigation_mode"
    switch_to_pos_srv: str = "/switch_to_position_mode"

    # Trajectory action
    traj_action_name: str = "/stretch_controller/follow_joint_trajectory"

    traj_server_wait_s: float = 5.0
    traj_timeout_s: float = 30.0


@dataclass
class Pose:
    wrist_extension: float = 0.20
    joint_lift: float = 0.35
    joint_wrist_yaw: float = 0.70     # start at RIGHT by default
    joint_wrist_pitch: float = -0.30
    joint_wrist_roll: float = 0.0
    joint_gripper_finger_left: float = 0.05  # open-ish


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class AirGraspDrop(Node):
    def __init__(self, cfg: Config):
        super().__init__("air_grasp_drop")
        self.cfg = cfg

        self.cmd_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)

        self.srv_nav = self.create_client(Trigger, cfg.switch_to_nav_srv)
        self.srv_pos = self.create_client(Trigger, cfg.switch_to_pos_srv)

        self.traj_client = ActionClient(self, FollowJointTrajectory, cfg.traj_action_name)

        self.joint_names = [
            "wrist_extension",
            "joint_lift",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_gripper_finger_left",  # ONLY ONE finger joint
        ]

    # ---------- helpers ----------
    def _stop_base(self, settle_s: float = 0.15):
        z = Twist()
        for _ in range(5):
            self.cmd_pub.publish(z)
            time.sleep(0.05)
        time.sleep(settle_s)

    def _call_trigger(self, client, name: str, wait_s: float = 5.0, timeout_s: float = 12.0) -> bool:
        if not client.wait_for_service(timeout_sec=float(wait_s)):
            self.get_logger().error(f"[SRV] {name} not available")
            return False

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=float(timeout_s))

        if not fut.done():
            self.get_logger().error(f"[SRV] {name} timed out after {timeout_s:.1f}s")
            return False

        try:
            resp = fut.result()
            ok = bool(resp.success)
            self.get_logger().info(f"[SRV] {name}: success={ok} msg='{resp.message}'")
            time.sleep(0.25)
            return ok
        except Exception as e:
            self.get_logger().error(f"[SRV] {name} failed: {e}")
            return False

    def _drive_forward_distance(self, meters: float, speed_mps: float):
        """
        Simple time-based drive. If you want odom-precise distance like your PrecisePath node,
        we can swap this to odom-based control.
        """
        if meters <= 0:
            return
        speed_mps = max(0.01, float(speed_mps))
        duration_s = meters / speed_mps

        msg = Twist()
        msg.linear.x = float(speed_mps)
        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self._stop_base()

    def _send_pose(self, start: Pose, goal: Pose, duration_s: float) -> bool:
        if not self.traj_client.wait_for_server(timeout_sec=float(self.cfg.traj_server_wait_s)):
            self.get_logger().error(f"[TRAJ] Action not available: {self.cfg.traj_action_name}")
            return False

        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(self.joint_names)

        # Important: do NOT set a future stamp
        g.trajectory.header.stamp.sec = 0
        g.trajectory.header.stamp.nanosec = 0

        p0 = JointTrajectoryPoint()
        p0.positions = [
            start.wrist_extension,
            start.joint_lift,
            start.joint_wrist_yaw,
            start.joint_wrist_pitch,
            start.joint_wrist_roll,
            start.joint_gripper_finger_left,
        ]
        p0.time_from_start = dur(0.0)

        p1 = JointTrajectoryPoint()
        p1.positions = [
            goal.wrist_extension,
            goal.joint_lift,
            goal.joint_wrist_yaw,
            goal.joint_wrist_pitch,
            goal.joint_wrist_roll,
            goal.joint_gripper_finger_left,
        ]
        p1.time_from_start = dur(duration_s)

        g.trajectory.points = [p0, p1]

        self.get_logger().info(
            f"[TRAJ] -> yaw={goal.joint_wrist_yaw:.2f} fingerL={goal.joint_gripper_finger_left:.3f}"
        )
        send_fut = self.traj_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=float(self.cfg.traj_timeout_s))
        if not send_fut.done():
            self.get_logger().error("[TRAJ] send_goal timed out")
            return False

        gh = send_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("[TRAJ] goal rejected")
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=float(self.cfg.traj_timeout_s))
        if not res_fut.done():
            self.get_logger().error("[TRAJ] result timed out")
            return False

        result = res_fut.result().result
        ok = int(result.error_code) == 0
        err_str = getattr(result, "error_string", "")
        self.get_logger().info(f"[TRAJ] done ok={ok} error_code={result.error_code} err='{err_str}'")
        time.sleep(0.2)
        return ok

    # ---------- main behavior ----------
    def run(self, drop: str, step_m: float, speed_mps: float) -> int:
        # Tuning
        grip_open = 0.05
        grip_closed = 0.01

        yaw_right = +0.70
        yaw_mid = 0.00
        yaw_left = -0.70

        # How many forward "steps" to reach the requested drop zone
        steps_needed = {"right": 0, "middle": 1, "left": 2}[drop]
        dist_total = steps_needed * float(step_m)

        self.get_logger().info(f"[PLAN] drop='{drop}' steps={steps_needed} step_m={step_m:.3f} total_m={dist_total:.3f}")

        # Start pose: RIGHT, open
        cur = Pose(
            wrist_extension=0.20,
            joint_lift=0.35,
            joint_wrist_yaw=yaw_right,
            joint_wrist_pitch=-0.30,
            joint_wrist_roll=0.0,
            joint_gripper_finger_left=grip_open,
        )

        # 1) Go to position mode and pinch at RIGHT
        if not self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, wait_s=5.0, timeout_s=12.0):
            return 2

        self.get_logger().info("[STEP] Right: ensure open")
        if not self._send_pose(cur, Pose(**{**cur.__dict__, "joint_gripper_finger_left": grip_open}), 0.6):
            return 3

        self.get_logger().info("[STEP] Right: close (pinch)")
        pinch = Pose(**{**cur.__dict__, "joint_gripper_finger_left": grip_closed})
        if not self._send_pose(cur, pinch, 0.8):
            return 3
        cur = pinch

        # 2) Navigate forward to drop zone while staying closed
        if dist_total > 0.0:
            if not self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, wait_s=5.0, timeout_s=12.0):
                return 4
            self.get_logger().info(f"[STEP] Drive forward {dist_total:.3f} m (time-based)")
            self._drive_forward_distance(dist_total, speed_mps)

        # 3) Switch back to position mode, orient wrist to target zone, then release
        if not self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, wait_s=5.0, timeout_s=12.0):
            return 5

        target_yaw = {"right": yaw_right, "middle": yaw_mid, "left": yaw_left}[drop]

        self.get_logger().info(f"[STEP] Rotate wrist to {drop.upper()} (yaw={target_yaw:.2f}) while holding closed")
        hold_at_drop = Pose(**{**cur.__dict__, "joint_wrist_yaw": target_yaw, "joint_gripper_finger_left": grip_closed})
        if not self._send_pose(cur, hold_at_drop, 1.0):
            return 6
        cur = hold_at_drop

        self.get_logger().info(f"[STEP] Release at {drop.upper()} (open)")
        released = Pose(**{**cur.__dict__, "joint_gripper_finger_left": grip_open})
        if not self._send_pose(cur, released, 0.7):
            return 7
        cur = released

        # Optional: return wrist yaw neutral
        self.get_logger().info("[STEP] Return yaw neutral (optional)")
        neutral = Pose(**{**cur.__dict__, "joint_wrist_yaw": 0.0})
        self._send_pose(cur, neutral, 1.0)

        self.get_logger().info("[DONE] Air-grasp drop complete")
        return 0


def main():
    parser = argparse.ArgumentParser(description="Air-grasp then drop at right/middle/left")
    parser.add_argument("--drop", choices=["right", "middle", "left"], default="middle",
                        help="Where to release the pinch (default: middle)")
    parser.add_argument("--step-m", type=float, default=0.45,
                        help="Forward distance (meters) between right->middle and middle->left (default: 0.45)")
    parser.add_argument("--speed", type=float, default=0.05,
                        help="Forward driving speed in m/s (default: 0.05)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AirGraspDrop(Config())
    try:
        rc = node.run(drop=args.drop, step_m=args.step_m, speed_mps=args.speed)
    finally:
        node._stop_base()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()