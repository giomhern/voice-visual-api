#!/usr/bin/env python3
from __future__ import annotations

import math
import time
import os
import signal
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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

    # Mode switch services (Trigger)
    switch_to_nav_srv: str = "/switch_to_navigation_mode"
    switch_to_pos_srv: str = "/switch_to_position_mode"

    # Surface cleaning Trigger service (Hello Robot demo)
    clean_surface_srv: str = "/clean_surface/trigger_clean_surface"

    # OPTIONAL: command to start the clean_surface demo node/launch from THIS script.
    # Set this to what you normally run in a terminal, e.g.:
    #   ["ros2", "launch", "stretch_demos", "clean_surface.launch.py"]
    # or:
    #   ["ros2", "run", "stretch_demos", "clean_surface_node"]
    #
    # If left as None, this script assumes the clean_surface service already exists.
    clean_surface_cmd: Optional[List[str]] = None

    # How long to wait for the service to appear after launching clean_surface
    demo_start_wait_s: float = 20.0

    # If True, stop the demo process after triggering
    stop_demo_after_trigger: bool = True

    # Trajectory action
    traj_action_name: str = "/stretch_controller/follow_joint_trajectory"

    # Turn parameters
    turn_left_rad: float = math.pi / 2.0
    yaw_tol_rad: float = math.radians(2.0)
    max_wz: float = 0.6
    kp_yaw: float = 1.8
    rate_hz: float = 30.0
    turn_timeout_s: float = 20.0

    # Trajectory timing
    traj_server_wait_s: float = 5.0
    traj_timeout_s: float = 30.0


class SurfaceCleanDesk(Node):
    def __init__(self, cfg: Config):
        super().__init__("surface_clean_desk")
        self.cfg = cfg

        self._yaw: Optional[float] = None
        self._pose_xy: Optional[Tuple[float, float]] = None

        self.cmd_pub = self.create_publisher(Twist, cfg.cmd_vel_topic, 10)
        self.create_subscription(Odometry, cfg.odom_topic, self._on_odom, 50)

        # Trigger clients
        self.srv_nav = self.create_client(Trigger, cfg.switch_to_nav_srv)
        self.srv_pos = self.create_client(Trigger, cfg.switch_to_pos_srv)
        self.srv_clean = self.create_client(Trigger, cfg.clean_surface_srv)

        # Trajectory action
        self.traj_client = ActionClient(self, FollowJointTrajectory, cfg.traj_action_name)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose_xy = (p.x, p.y)
        self._yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    # ---------- process helpers ----------
    def _start_process(self, cmd: List[str], name: str) -> Optional[subprocess.Popen]:
        if not cmd:
            return None
        self.get_logger().info(f"[PROC] starting {name}: {' '.join(cmd)}")

        # Start in its own process group so we can stop the entire launch tree
        p = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
        )
        return p

    def _stop_process(self, p: Optional[subprocess.Popen], name: str, timeout_s: float = 5.0) -> None:
        if not p or p.poll() is not None:
            return

        self.get_logger().info(f"[PROC] stopping {name} (SIGINT)")
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if p.poll() is not None:
                return
            time.sleep(0.1)

        self.get_logger().warn(f"[PROC] {name} did not exit; sending SIGKILL")
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

    def _wait_for_service(self, client, name: str, total_wait_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < total_wait_s:
            if client.wait_for_service(timeout_sec=0.5):
                return True
        self.get_logger().error(f"[SRV] {name} not available after {total_wait_s:.1f}s")
        return False

    # ---------- motion helpers ----------
    def _stop_base(self, settle_s: float = 0.25):
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
            return ok
        except Exception as e:
            self.get_logger().error(f"[SRV] {name} failed: {e}")
            return False

    def _turn_relative(self, delta_rad: float) -> bool:
        # Wait for odom
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

    # ---------- trajectory helpers ----------
    def _send_traj(self, joint_names: List[str], positions: List[float], duration_s: float) -> bool:
        if len(joint_names) != len(positions):
            self.get_logger().error("[TRAJ] joint_names/positions mismatch")
            return False

        if not self.traj_client.wait_for_server(timeout_sec=float(self.cfg.traj_server_wait_s)):
            self.get_logger().error(f"[TRAJ] Action not available: {self.cfg.traj_action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # Important: don't set a future stamp
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
        # Goal A: everything EXCEPT extension
        ok = self._send_traj(
            joint_names=["joint_lift", "joint_wrist_yaw", "joint_head_pan", "joint_head_tilt"],
            positions=[
                0.906026669779699,          # lift
                0.10929613113685194,        # wrist_yaw
                -1.7996282068213987,        # head_pan
                -0.799664042887519,         # head_tilt
            ],
            duration_s=2.0,
        )
        if not ok:
            return False

        # Goal B: extension last
        ok = self._send_traj(
            joint_names=["wrist_extension"],
            positions=[0.10261581340392491],
            duration_s=2.0,
        )
        return ok

    # ---------- main pipeline ----------
    def run(self) -> int:
        self.get_logger().info("[PIPE] 1) switch_to_navigation_mode")
        if not self._call_trigger(self.srv_nav, self.cfg.switch_to_nav_srv, wait_s=5.0, timeout_s=12.0):
            return 2
        time.sleep(0.25)

        self.get_logger().info("[PIPE] 2) turn left precisely (odom-based)")
        if not self._turn_relative(self.cfg.turn_left_rad):
            return 3

        self.get_logger().info("[PIPE] 3) switch_to_position_mode")
        if not self._call_trigger(self.srv_pos, self.cfg.switch_to_pos_srv, wait_s=5.0, timeout_s=12.0):
            return 4
        time.sleep(0.25)

        self.get_logger().info("[PIPE] 4) pre-wipe pose")
        if not self._prewipe_pose():
            return 5

        # Start clean_surface demo process if configured, otherwise assume it's already running
        self.get_logger().info("[PIPE] 5) ensure clean_surface service exists (optionally launch demo)")
        clean_proc = None
        if self.cfg.clean_surface_cmd:
            clean_proc = self._start_process(self.cfg.clean_surface_cmd, "clean_surface")
            if not self._wait_for_service(self.srv_clean, self.cfg.clean_surface_srv, self.cfg.demo_start_wait_s):
                self._stop_process(clean_proc, "clean_surface")
                return 6
        else:
            if not self._wait_for_service(self.srv_clean, self.cfg.clean_surface_srv, 2.0):
                self.get_logger().error(
                    "[PIPE] clean_surface service not found. Start the demo separately, "
                    "or set Config.clean_surface_cmd to launch it."
                )
                return 6

        self.get_logger().info("[PIPE] 6) trigger clean_surface")
        if not self._call_trigger(self.srv_clean, self.cfg.clean_surface_srv, wait_s=0.5, timeout_s=90.0):
            self._stop_process(clean_proc, "clean_surface")
            return 7

        if self.cfg.stop_demo_after_trigger:
            self._stop_process(clean_proc, "clean_surface")

        self.get_logger().info("[PIPE] DONE ✅")
        return 0


def main():
    rclpy.init()

    # IMPORTANT:
    # If you want THIS script to start the clean_surface node, set clean_surface_cmd below.
    # Example:
    # cfg = Config(clean_surface_cmd=["ros2", "launch", "stretch_demos", "clean_surface.launch.py"])
    cfg = Config(
        clean_surface_cmd="ros2 launch stretch_demos clean_surface.launch.py",   # <-- set this to your real launch/run command if desired
        stop_demo_after_trigger=True,
    )

    node = SurfaceCleanDesk(cfg)
    try:
        rc = node.run()
    finally:
        node._stop_base()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()