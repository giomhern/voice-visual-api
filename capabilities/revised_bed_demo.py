#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass
class Config:
    cmd_vel_topic: str = "/stretch/cmd_vel"
    joint_states_topic: str = "/joint_states"

    switch_to_nav_srv: str = "/switch_to_navigation_mode"
    switch_to_pos_srv: str = "/switch_to_position_mode"
    stow_srv: str = "/stow_the_robot"

    traj_action_name: str = "/stretch_controller/follow_joint_trajectory"

    traj_server_wait_s: float = 5.0
    traj_timeout_s: float = 30.0


@dataclass
class Pose:
    wrist_extension: float = 0.20
    joint_lift: float = 0.35
    joint_wrist_yaw: float = 0.0
    joint_wrist_pitch: float = -0.30
    joint_wrist_roll: float = 0.0
    joint_gripper_finger_left: float = 0.05


def dur(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class RevisedBedDemo(Node):
    def __init__(self, cfg: Config):
        super().__init__("revised_bed_demo")
        self.cfg = cfg
        self._joint_state: JointState | None = None

        self.cmd_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)
        self.create_subscription(JointState, cfg.joint_states_topic, self._on_joint_state, 10)

        self.srv_nav = self.create_client(Trigger, cfg.switch_to_nav_srv)
        self.srv_pos = self.create_client(Trigger, cfg.switch_to_pos_srv)
        self.srv_stow = self.create_client(Trigger, cfg.stow_srv)

        self.traj_client = ActionClient(self, FollowJointTrajectory, cfg.traj_action_name)

        self.joint_names = [
            "wrist_extension",
            "joint_lift",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "joint_gripper_finger_left",
        ]

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _wait_for_joint_state(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state is not None:
                return True
        self.get_logger().error(f"[JOINTS] No joint states received on {self.cfg.joint_states_topic}")
        return False

    def _joint_map(self) -> dict[str, float]:
        if self._joint_state is None:
            return {}
        joints = {name: float(pos) for name, pos in zip(self._joint_state.name, self._joint_state.position)}
        # `wrist_extension` is a virtual joint accepted by the trajectory
        # controller but not published on /joint_states — it's the sum of
        # the four telescoping arm segments.
        if "wrist_extension" not in joints:
            arm_segments = ("joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3")
            if all(seg in joints for seg in arm_segments):
                joints["wrist_extension"] = sum(joints[seg] for seg in arm_segments)
        return joints

    def _current_pose(self) -> Pose | None:
        if not self._wait_for_joint_state():
            return None

        joints = self._joint_map()
        missing = [name for name in self.joint_names if name not in joints]
        if missing:
            self.get_logger().error(f"[JOINTS] Missing joint states for: {missing}")
            return None

        return Pose(
            wrist_extension=joints["wrist_extension"],
            joint_lift=joints["joint_lift"],
            joint_wrist_yaw=joints["joint_wrist_yaw"],
            joint_wrist_pitch=joints["joint_wrist_pitch"],
            joint_wrist_roll=joints["joint_wrist_roll"],
            joint_gripper_finger_left=joints["joint_gripper_finger_left"],
        )

    def _call_trigger(self, client, name: str, wait_s: float = 5.0, timeout_s: float = 20.0) -> bool:
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
            time.sleep(0.2)
            return ok
        except Exception as exc:
            self.get_logger().error(f"[SRV] {name} failed: {exc}")
            return False

    def _ensure_nav_mode(self) -> bool:
        return self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, wait_s=5.0, timeout_s=12.0)

    def _ensure_pos_mode(self) -> bool:
        return self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, wait_s=5.0, timeout_s=12.0)

    def _stow_robot(self) -> bool:
        return self._call_trigger(self.srv_stow, self.cfg.stow_srv, wait_s=5.0, timeout_s=40.0)

    def _stop_base_nav(self, settle_s: float = 0.15) -> None:
        if not self._ensure_nav_mode():
            self.get_logger().warn("[STOP] Could not switch to nav mode; skipping cmd_vel stop")
            return

        z = Twist()
        for _ in range(6):
            self.cmd_pub.publish(z)
            time.sleep(0.05)
        time.sleep(settle_s)

    def _drive_forward_distance_nav(self, meters: float, speed_mps: float) -> bool:
        if meters <= 0.0:
            self._stop_base_nav()
            return True

        if not self._ensure_nav_mode():
            self.get_logger().error("[DRIVE] Failed to switch to navigation mode before driving")
            return False

        speed_mps = max(0.01, float(speed_mps))
        duration_s = meters / speed_mps

        msg = Twist()
        msg.linear.x = float(speed_mps)
        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self._stop_base_nav()
        return True

    def _send_pose(self, start: Pose, goal: Pose, duration_s: float) -> bool:
        if not self.traj_client.wait_for_server(timeout_sec=float(self.cfg.traj_server_wait_s)):
            self.get_logger().error(f"[TRAJ] Action not available: {self.cfg.traj_action_name}")
            return False

        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(self.joint_names)
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

        self.get_logger().info(f"[TRAJ] Sending pose -> {goal}")
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

    def _move_lift_first(self, cur: Pose, lift_target: float) -> Pose | None:
        self.get_logger().info("[STEP] Raise lift first")
        lifted = Pose(**{**cur.__dict__, "joint_lift": lift_target})
        if not self._send_pose(cur, lifted, 1.5):
            return None
        return lifted

    def _apparent_grasp(self, cur: Pose, open_wide: float, closed: float) -> Pose | None:
        self.get_logger().info("[STEP] Open gripper wide")
        wide = Pose(**{**cur.__dict__, "joint_gripper_finger_left": open_wide})
        if not self._send_pose(cur, wide, 0.7):
            return None
        time.sleep(0.2)

        self.get_logger().info("[STEP] Close gripper slowly")
        pinch = Pose(**{**wide.__dict__, "joint_gripper_finger_left": closed})
        if not self._send_pose(wide, pinch, 1.2):
            return None

        time.sleep(0.5)
        return pinch

    def _apparent_release(self, cur: Pose, open_wide: float, closed: float) -> Pose | None:
        self.get_logger().info("[STEP] Open gripper wide to release")
        released = Pose(**{**cur.__dict__, "joint_gripper_finger_left": open_wide})
        if not self._send_pose(cur, released, 0.9):
            return None

        time.sleep(0.2)
        half_close = Pose(**{**released.__dict__, "joint_gripper_finger_left": max(closed, open_wide - 0.02)})
        if not self._send_pose(released, half_close, 0.35):
            return None
        if not self._send_pose(half_close, released, 0.35):
            return None

        time.sleep(0.2)
        return released

    def run(self, drop: str, step_m: float, speed_mps: float) -> int:
        grip_open_wide = 0.08
        grip_closed = 0.01

        steps_needed = {"right": 0, "middle": 1, "left": 2}[drop]
        dist_total = steps_needed * float(step_m)

        cur = self._current_pose()
        if cur is None:
            return 1

        self.get_logger().info("[PIPE] 1) stow robot before starting")
        if not self._stow_robot():
            return 2
        cur = self._current_pose()
        if cur is None:
            return 3

        self.get_logger().info("[PIPE] 2) switch to position mode")
        if not self._ensure_pos_mode():
            return 4

        lifted = self._move_lift_first(cur, lift_target=0.60)
        if lifted is None:
            self.get_logger().info("[PIPE] recovery: stow after failed lift step")
            self._stow_robot()
            return 5
        cur = lifted

        self.get_logger().info("[PIPE] 3) do gripper motions after lift")
        grasped = self._apparent_grasp(cur, open_wide=grip_open_wide, closed=grip_closed)
        if grasped is None:
            self.get_logger().info("[PIPE] recovery: stow after failed grasp step")
            self._stow_robot()
            return 6
        cur = grasped

        self.get_logger().info(f"[PIPE] 4) carry forward by {dist_total:.3f} m")
        if not self._drive_forward_distance_nav(dist_total, speed_mps):
            self.get_logger().info("[PIPE] recovery: stow after failed drive step")
            self._stow_robot()
            return 7

        self.get_logger().info("[PIPE] 5) switch to position mode for release")
        if not self._ensure_pos_mode():
            self._stow_robot()
            return 8

        released = self._apparent_release(cur, open_wide=grip_open_wide, closed=grip_closed)
        if released is None:
            self.get_logger().info("[PIPE] recovery: stow after failed release step")
            self._stow_robot()
            return 9

        self.get_logger().info("[PIPE] 6) stow robot after demo")
        if not self._stow_robot():
            return 10

        self.get_logger().info("[DONE] Revised bed demo complete")
        return 0


def main():
    parser = argparse.ArgumentParser(description="Revised bed demo with safety stow before and after")
    parser.add_argument("--drop", choices=["right", "middle", "left"], default="middle")
    parser.add_argument("--step-m", type=float, default=0.4)
    parser.add_argument("--speed", type=float, default=0.05)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RevisedBedDemo(Config())
    try:
        rc = node.run(drop=args.drop, step_m=args.step_m, speed_mps=args.speed)
    finally:
        node._stop_base_nav()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
