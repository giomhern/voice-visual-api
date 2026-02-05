# stretch_study/capabilities/deterministic_demos.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .base_motion import BaseMotion


# -----------------------------
# Clean surface Trigger client
# -----------------------------
@dataclass
class CleanSurfaceConfig:
    service_name: str = "/clean_surface/trigger_clean_surface"
    wait_for_service_s: float = 10.0


class CleanSurfaceClient:
    """
    Minimal client to trigger Hello Robot's clean_surface demo.
    """

    def __init__(self, node, cfg: Optional[CleanSurfaceConfig] = None):
        self.node = node
        self.cfg = cfg or CleanSurfaceConfig()
        self._client = node.create_client(Trigger, self.cfg.service_name)

    def _try_autodiscover_service(self) -> Optional[str]:
        names_and_types = self.node.get_service_names_and_types()
        candidates = []
        for (name, types) in names_and_types:
            if "clean_surface" in name:
                candidates.append((name, types))

        if not candidates:
            return None

        for (name, types) in candidates:
            if any(t == "std_srvs/srv/Trigger" for t in types) and ("trigger" in name.lower()):
                return name

        for (name, types) in candidates:
            if any(t == "std_srvs/srv/Trigger" for t in types):
                return name

        return candidates[0][0]

    def _ensure_client(self) -> bool:
        if self._client.wait_for_service(timeout_sec=float(self.cfg.wait_for_service_s)):
            return True

        alt = self._try_autodiscover_service()
        if alt and alt != self.cfg.service_name:
            self.node.get_logger().warn(
                f"[CLEAN_SURFACE] '{self.cfg.service_name}' not available. "
                f"Autodiscovered '{alt}'. Rebinding client."
            )
            self.cfg.service_name = alt
            self._client = self.node.create_client(Trigger, self.cfg.service_name)

        if not self._client.wait_for_service(timeout_sec=float(self.cfg.wait_for_service_s)):
            self.node.get_logger().error(
                f"[CLEAN_SURFACE] No Trigger service available (tried '{self.cfg.service_name}'). "
                "Make sure the clean_surface node is running."
            )
            return False

        return True

    def trigger_sync(self, timeout_s: float = 30.0) -> bool:
        if not self._ensure_client():
            return False

        fut = self._client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=float(timeout_s))

        if not fut.done():
            self.node.get_logger().error(f"[CLEAN_SURFACE] Trigger timed out after {timeout_s:.1f}s")
            return False

        try:
            resp = fut.result()
            self.node.get_logger().info(
                f"[CLEAN_SURFACE] Trigger response: success={resp.success} message='{resp.message}'"
            )
            return bool(resp.success)
        except Exception as e:
            self.node.get_logger().error(f"[CLEAN_SURFACE] Trigger call failed: {e}")
            return False


# -----------------------------
# Pre-wipe posture (your recorded joint state)
# -----------------------------
@dataclass
class PreWipePose:
    # This is EXACTLY the ordering and values you pasted from /stretch/joint_states
    joint_names: Tuple[str, ...] = (
        "wrist_extension",
        "joint_lift",
        "joint_arm_l3",
        "joint_arm_l2",
        "joint_arm_l1",
        "joint_arm_l0",
        "joint_head_pan",
        "joint_head_tilt",
        "joint_wrist_yaw",
        "joint_wrist_pitch",
        "joint_wrist_roll",
        "joint_gripper_finger_left",
        "joint_gripper_finger_right",
    )
    positions: Tuple[float, ...] = (
        0.1110089077092974,
        0.8838716489173609,
        0.02775222692732435,
        0.02775222692732435,
        0.02775222692732435,
        0.02775222692732435,
        -1.7996282068213987,
        -0.7950621005238621,
        -0.042184471666855135,
        -1.165825398793087,
        -0.0015339807878856412,
        0.003029408718051971,
        0.003029408718051971,
    )
    move_time_s: float = 4.0
    wait_for_action_s: float = 5.0


class JointTrajectoryMover:
    """
    Sends a FollowJointTrajectory to the Stretch driver (autodiscovers action name).
    """

    def __init__(self, node):
        self.node = node
        self._action_name = self._autodiscover_fjt_action_name()
        self._client = ActionClient(self.node, FollowJointTrajectory, self._action_name)
        self.node.get_logger().info(f"[POSTURE] Using FollowJointTrajectory action: {self._action_name}")

    def _autodiscover_fjt_action_name(self) -> str:
        # Try to find any FollowJointTrajectory action server.
        # rclpy: node.get_action_names_and_types() returns List[Tuple[str, List[str]]]
        try:
            names_and_types = self.node.get_action_names_and_types()
        except Exception:
            names_and_types = []

        fjt_type = "control_msgs/action/FollowJointTrajectory"
        candidates: List[str] = []

        for (name, types) in names_and_types:
            if any(t == fjt_type for t in types):
                candidates.append(name)

        # Prefer names that look like Stretch's driver action
        preferred = []
        for n in candidates:
            ln = n.lower()
            score = 0
            if "joint" in ln:
                score += 1
            if "traj" in ln:
                score += 2
            if "stretch" in ln or "driver" in ln:
                score += 2
            preferred.append((score, n))

        if preferred:
            preferred.sort(reverse=True, key=lambda x: x[0])
            return preferred[0][1]

        # Common fallbacks if discovery failed (safe guesses)
        # You can change this if your system uses a different name.
        return "/stretch_driver/joint_traj"

    def move_to(self, joint_names: List[str], positions: List[float], move_time_s: float = 4.0,
                wait_for_action_s: float = 5.0, timeout_s: float = 30.0) -> bool:
        if len(joint_names) != len(positions):
            self.node.get_logger().error("[POSTURE] joint_names and positions length mismatch")
            return False

        if not self._client.wait_for_server(timeout_sec=float(wait_for_action_s)):
            self.node.get_logger().error(f"[POSTURE] Action server not available: {self._action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, positions))
        pt.time_from_start.sec = int(move_time_s)
        pt.time_from_start.nanosec = int((move_time_s - int(move_time_s)) * 1e9)
        goal.trajectory.points = [pt]

        send_fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_fut, timeout_sec=10.0)
        if not send_fut.done():
            self.node.get_logger().error("[POSTURE] Timed out sending trajectory goal")
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[POSTURE] Trajectory goal rejected")
            return False

        self.node.get_logger().info("[POSTURE] Trajectory goal accepted, waiting for result...")
        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=float(timeout_s))
        if not result_fut.done():
            self.node.get_logger().error("[POSTURE] Timed out waiting for trajectory result")
            return False

        status = result_fut.result().status
        # 4 is SUCCEEDED in action_msgs/GoalStatus
        ok = (status == 4)
        self.node.get_logger().info(f"[POSTURE] Result status={status} ok={ok}")
        return ok


# -----------------------------
# Demos
# -----------------------------
class DeterministicDemos:
    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        funmap_head_scan_srv: str = "/funmap/trigger_head_scan",
        funmap_local_loc_srv: str = "/funmap/trigger_local_localization",
        switch_to_traj_srv: str = "/switch_to_trajectory_mode",
        switch_to_nav_srv: str = "/switch_to_navigation_mode",
        **_ignored_kwargs,
    ):
        self.node = node
        self.motion_enabled = bool(motion_enabled)
        self.distances: Dict[str, float] = dict(distances or {})
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.odom_topic = str(odom_topic)

        self.motion = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)
        self.turn_left_rad = math.pi / 2.0

        self.clean_surface = CleanSurfaceClient(
            node=self.node,
            cfg=CleanSurfaceConfig(service_name=str(clean_surface_service)),
        )

        self._srv_head_scan = self.node.create_client(Trigger, funmap_head_scan_srv)
        self._srv_local_loc = self.node.create_client(Trigger, funmap_local_loc_srv)
        self._srv_traj_mode = self.node.create_client(Trigger, switch_to_traj_srv)
        self._srv_nav_mode = self.node.create_client(Trigger, switch_to_nav_srv)

        # NEW: posture mover + config
        self._posture = JointTrajectoryMover(self.node)
        self._pre_wipe_pose = PreWipePose()

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"head_scan_srv={funmap_head_scan_srv} local_loc_srv={funmap_local_loc_srv} "
            f"traj_srv={switch_to_traj_srv} nav_srv={switch_to_nav_srv}"
        )

    def _call_trigger_sync(self, client, name: str, timeout_s: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            self.node.get_logger().warn(f"[SETUP] service not available: {name}")
            return False

        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=float(timeout_s))

        if not fut.done():
            self.node.get_logger().warn(f"[SETUP] {name}: timed out after {timeout_s:.1f}s")
            return False

        try:
            resp = fut.result()
            ok = bool(resp.success)
            self.node.get_logger().info(f"[SETUP] {name}: success={ok} msg='{resp.message}'")
            return ok
        except Exception as e:
            self.node.get_logger().warn(f"[SETUP] {name}: call failed: {e}")
            return False

    # -----------------------------
    # NEW: move joints to your recorded pre-wipe posture
    # -----------------------------
    def _go_to_pre_wipe_pose(self) -> bool:
        cfg = self._pre_wipe_pose
        self.node.get_logger().info(
            f"[POSTURE] Moving to pre-wipe pose (lift={cfg.positions[cfg.joint_names.index('joint_lift')]:.3f} m)"
        )
        return self._posture.move_to(
            joint_names=list(cfg.joint_names),
            positions=list(cfg.positions),
            move_time_s=float(cfg.move_time_s),
            wait_for_action_s=float(cfg.wait_for_action_s),
            timeout_s=30.0,
        )

    # -----------------------------
    # Surface cleaning demo pipeline
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(
            f"[DEMO] Desk surface-clean requested thoroughness='{thoroughness}'"
        )

        # (Optional) best-effort scans / localization
        self._call_trigger_sync(self._srv_head_scan, "funmap/trigger_head_scan", timeout_s=20.0)
        self._call_trigger_sync(self._srv_local_loc, "funmap/trigger_local_localization", timeout_s=10.0)

        # Switch to trajectory mode FIRST (so joint trajectory action works)
        self._call_trigger_sync(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=5.0)

        # NEW: move to the exact posture you recorded
        if not self._go_to_pre_wipe_pose():
            self.node.get_logger().error(
                "[DEMO] Could not reach pre-wipe posture. "
                "Check FollowJointTrajectory action name and controller mode."
            )
            return

        # Trigger clean_surface
        ok = self.clean_surface.trigger_sync(timeout_s=30.0)
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered; it will scan/plan/execute.")
    # Placeholders
    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")