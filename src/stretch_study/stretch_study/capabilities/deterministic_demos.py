# stretch_study/capabilities/deterministic_demos.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .base_motion import BaseMotion


# -----------------------------
# Clean surface Trigger client
# -----------------------------
@dataclass
class CleanSurfaceConfig:
    service_name: str = "/clean_surface/trigger_clean_surface"
    wait_for_service_s: float = 10.0


class CleanSurfaceClient:
    """Minimal client to trigger Hello Robot's clean_surface demo (std_srvs/srv/Trigger)."""

    def __init__(self, node: Node, cfg: Optional[CleanSurfaceConfig] = None):
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

    def trigger_sync(self, timeout_s: float = 60.0) -> bool:
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
# Demos
# -----------------------------
class DeterministicDemos:
    """
    - desk_demo(): (optional) funmap setup + switch to trajectory mode + PRE-WIPE ACTION GOAL + trigger clean_surface
    - transit(): legacy deterministic base motion (not used by desk_demo)
    """

    def __init__(
        self,
        node: Node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        # Setup services (best-effort)
        funmap_head_scan_srv: str = "/funmap/trigger_head_scan",
        funmap_local_loc_srv: str = "/funmap/trigger_local_localization",
        switch_to_traj_srv: str = "/switch_to_trajectory_mode",
        switch_to_nav_srv: str = "/switch_to_navigation_mode",
        # Trajectory action (you verified this works):
        traj_action_name: str = "/stretch_controller/follow_joint_trajectory",
        **_ignored_kwargs,
    ):
        self.node = node
        self.motion_enabled = bool(motion_enabled)
        self.distances: Dict[str, float] = dict(distances or {})
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.odom_topic = str(odom_topic)

        # Legacy base motion helper (NOT used in desk_demo)
        self.motion = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)
        self.turn_left_rad = math.pi / 2.0

        # Clean surface trigger client
        self.clean_surface = CleanSurfaceClient(
            node=self.node,
            cfg=CleanSurfaceConfig(service_name=str(clean_surface_service)),
        )

        # Trigger clients for setup
        self._srv_head_scan = self.node.create_client(Trigger, funmap_head_scan_srv)
        self._srv_local_loc = self.node.create_client(Trigger, funmap_local_loc_srv)
        self._srv_traj_mode = self.node.create_client(Trigger, switch_to_traj_srv)
        self._srv_nav_mode = self.node.create_client(Trigger, switch_to_nav_srv)

        # Trajectory action client for pre-wipe pose
        self._traj_action_name = str(traj_action_name)
        self._traj_client = ActionClient(self.node, FollowJointTrajectory, self._traj_action_name)

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"traj_action={self._traj_action_name}"
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

    def _send_traj(
        self,
        joint_names: List[str],
        positions: List[float],
        duration_sec: float = 2.0,
        server_wait_sec: float = 5.0,
        timeout_sec: float = 30.0,
    ) -> bool:
        """Send one FollowJointTrajectory goal and wait for result. Returns True if error_code==0."""
        if len(joint_names) != len(positions):
            self.node.get_logger().error("[PREP] joint_names and positions length mismatch")
            return False

        if not self._traj_client.wait_for_server(timeout_sec=float(server_wait_sec)):
            self.node.get_logger().error(f"[PREP] Trajectory action not available: {self._traj_action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=int(duration_sec))
        goal.trajectory.points = [pt]

        self.node.get_logger().info(f"[PREP] Sending goal {joint_names} -> {positions}")
        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_fut, timeout_sec=float(timeout_sec))

        if not send_fut.done():
            self.node.get_logger().error("[PREP] send_goal timed out")
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[PREP] Goal rejected")
            return False

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=float(timeout_sec))

        if not result_fut.done():
            self.node.get_logger().error("[PREP] result timed out")
            return False

        result = result_fut.result().result
        err_str = getattr(result, "error_string", "")
        self.node.get_logger().info(f"[PREP] Result error_code={result.error_code} error_string='{err_str}'")
        return int(result.error_code) == 0

    def _preprocess_pose_exact(self) -> bool:
        """
        This matches your WORKING test script exactly.
        """
        joint_names = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]
        positions = [0.906026669779699, -8.377152024940286e-06, 0.0051132692929521375]
        return self._send_traj(joint_names, positions, duration_sec=2.0, timeout_sec=30.0)

    # -----------------------------
    # Deterministic base transit (legacy / optional)
    # -----------------------------
    def transit(self, from_loc: str, to_loc: str) -> None:
        if not self.motion_enabled:
            self.node.get_logger().info("[MOTION] transit skipped (motion disabled)")
            return
        self.node.get_logger().warn("[MOTION] transit not used for desk_demo; only legacy paths implemented.")

    # -----------------------------
    # Surface cleaning demo pipeline
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        """
        IMPORTANT: This function does NOT rotate or drive the base.
        It only:
          - (best-effort) funmap head scan + localization
          - switches to trajectory mode
          - runs your exact preprocessing pose (action goal)
          - triggers clean_surface
        """
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(f"[DEMO] Desk clean requested thoroughness='{thoroughness}'")

        # Optional / best-effort setup (comment these out if you want *zero* funmap activity)
        # self._call_trigger_sync(self._srv_head_scan, "funmap/trigger_head_scan", timeout_s=30.0)
        # self._call_trigger_sync(self._srv_local_loc, "funmap/trigger_local_localization", timeout_s=15.0)

        # Switch to trajectory mode (important)
        self._call_trigger_sync(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=10.0)

    
        # Preprocessing posture (your working pose)
        if not self._preprocess_pose_exact():
            self.node.get_logger().error("[DEMO] Preprocessing pose failed; not triggering clean_surface.")
            return

        # Trigger clean_surface
        ok = self.clean_surface.trigger_sync(timeout_s=60.0)
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered.")

    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")