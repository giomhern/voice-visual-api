#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
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
class Config:
    cmd_vel_topic: str = "/stretch/cmd_vel"
    odom_topic: str = "/odom"
    switch_to_nav_srv: str = "/switch_to_navigation_mode"
    switch_to_pos_srv: str = "/switch_to_position_mode"
    traj_action_name: str = "/stretch_controller/follow_joint_trajectory"

    turn_left_rad: float = 83 * math.pi / 180
    yaw_tol_rad: float = math.radians(2.0)
    max_wz: float = 0.6
    kp_yaw: float = 1.8
    rate_hz: float = 30.0
    turn_timeout_s: float = 20.0
    post_turn_settle_s: float = 1.0

    traj_server_wait_s: float = 5.0
    traj_timeout_s: float = 30.0

    prewipe_lift_m: float = 0.9
    prewipe_wrist_yaw_rad: float = 0.10929613113685194
    prewipe_head_pan_rad: float = -1.7996282068213987
    prewipe_head_tilt_rad: float = -0.799664042887519
    prewipe_extension_m: float = 0.15


class PrewipePoseTest(Node):
    def __init__(self, cfg: Config):
        super().__init__("prewipe_pose_test")
        self.cfg = cfg

        self._yaw: Optional[float] = None
        self._pose_xy: Optional[Tuple[float, float]] = None
        self._mode: Optional[str] = None

        self.cmd_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)
        self.create_subscription(Odometry, cfg.odom_topic, self._on_odom, 50)

        self.srv_nav = self.create_client(Trigger, cfg.switch_to_nav_srv)
        self.srv_pos = self.create_client(Trigger, cfg.switch_to_pos_srv)
        self.traj_client = ActionClient(self, FollowJointTrajectory, cfg.traj_action_name)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _stop_base(self, settle_s: float = 0.25):
        if self._mode != "navigation":
            time.sleep(settle_s)
            return

        z = Twist()
        for _ in range(5):
            self.cmd_pub.publish(z)
            time.sleep(0.05)
        time.sleep(settle_s)

    def _call_trigger(self, client, name: str, wait_s: float = 5.0, timeout_s: float = 10.0) -> bool:
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
            if ok:
                if name == self.cfg.switch_to_nav_srv:
                    self._mode = "navigation"
                elif name == self.cfg.switch_to_pos_srv:
                    self._mode = "position"
            return ok
        except Exception as e:
            self.get_logger().error(f"[SRV] {name} failed: {e}")
            return False

    def _turn_relative(self, delta_rad: float) -> bool:
        t0 = time.time()
        while rclpy.ok() and self._yaw is None and (time.time() - t0) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._yaw is None:
            self.get_logger().error("[TURN] No /odom yaw received")
            return False

        target = wrap_pi(self._yaw + delta_rad)
        start = time.time()
        dt = 1.0 / float(self.cfg.rate_hz)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self._yaw is not None

            err = wrap_pi(target - self._yaw)
            if abs(err) <= self.cfg.yaw_tol_rad:
                break

            if (time.time() - start) > self.cfg.turn_timeout_s:
                self.get_logger().error("[TURN] Timeout")
                self._stop_base()
                return False

            wz = self.cfg.kp_yaw * err
            wz = max(-self.cfg.max_wz, min(self.cfg.max_wz, wz))

            cmd = Twist()
            cmd.angular.z = float(wz)
            self.cmd_pub.publish(cmd)
            time.sleep(dt)

        self._stop_base()
        self.get_logger().info("[TURN] Reached target yaw")
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

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec_i = int(duration_s)
        nsec_i = int((duration_s - sec_i) * 1e9)
        pt.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)
        goal.trajectory.points = [pt]

        self.get_logger().info(f"[TRAJ] Sending {joint_names} -> {positions}")
        send_fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=float(self.cfg.traj_timeout_s))

        if not send_fut.done():
            self.get_logger().error("[TRAJ] send_goal timed out")
            return False

        goal_handle = send_fut.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("[TRAJ] goal rejected")
            return False

        res_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=float(self.cfg.traj_timeout_s))
        if not res_fut.done():
            self.get_logger().error("[TRAJ] result timed out")
            return False

        result = res_fut.result().result
        err_str = getattr(result, "error_string", "")
        ok = int(result.error_code) == 0
        self.get_logger().info(f"[TRAJ] done ok={ok} error_code={result.error_code} err='{err_str}'")
        time.sleep(0.25)
        return ok

    def _prewipe_pose(self) -> bool:
        ok = self._send_traj(
            joint_names=["joint_lift"],
            positions=[self.cfg.prewipe_lift_m],
            duration_s=2.5,
        )
        if not ok:
            return False

        ok = self._send_traj(
            joint_names=["joint_wrist_yaw", "joint_head_pan", "joint_head_tilt"],
            positions=[
                self.cfg.prewipe_wrist_yaw_rad,
                self.cfg.prewipe_head_pan_rad,
                self.cfg.prewipe_head_tilt_rad,
            ],
            duration_s=2.0,
        )
        if not ok:
            return False

        ok = self._send_traj(
            joint_names=["wrist_extension"],
            positions=[self.cfg.prewipe_extension_m],
            duration_s=2.0,
        )
        return ok

    def run(self) -> int:
        self.get_logger().info("[PIPE] Starting standalone pre-wipe pose test")

        self.get_logger().info("[PIPE] 1) switch_to_navigation_mode")
        if not self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, wait_s=5.0, timeout_s=12.0):
            return 2
        time.sleep(0.25)

        self.get_logger().info("[PIPE] 2) turn left precisely (odom-based)")
        if not self._turn_relative(self.cfg.turn_left_rad):
            return 3
        self._stop_base(settle_s=self.cfg.post_turn_settle_s)

        self.get_logger().info("[PIPE] 3) switch_to_position_mode")
        if not self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, wait_s=5.0, timeout_s=12.0):
            return 4
        time.sleep(0.25)

        self.get_logger().info("[PIPE] 4) pre-wipe pose")
        if not self._prewipe_pose():
            self.get_logger().info("[PIPE] recovery: switch_to_navigation_mode after pre-wipe failure")
            self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, wait_s=2.0, timeout_s=8.0)
            time.sleep(0.25)
            return 5

        self.get_logger().info("[PIPE] DONE (pre-wipe pose reached)")
        return 0


def main():
    rclpy.init()
    node = PrewipePoseTest(Config())
    try:
        rc = node.run()
    finally:
        node._stop_base()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
