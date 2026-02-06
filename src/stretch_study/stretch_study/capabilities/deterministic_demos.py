# stretch_study/capabilities/deterministic_demos.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, List

import rclpy
from rclpy.action import ActionClient
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
    # Correct default for your system (you confirmed):
    service_name: str = "/clean_surface/trigger_clean_surface"
    # Give bringup time; 2s is often too short on real launches
    wait_for_service_s: float = 10.0


class CleanSurfaceClient:
    """
    Minimal client to trigger Hello Robot's clean_surface demo.
    Triggers via std_srvs/srv/Trigger.
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
        """
        Synchronous trigger: blocks until response or timeout.
        """
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
    Demo behaviors for StudyEngine.

    - desk_demo(): setup pipeline + (optional) move to a pre-wipe pose + trigger clean_surface.
    - transit(): legacy deterministic base motion fallback (only if you enable it).
    """

    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        # Setup services
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

        # Base motion helper (legacy / fallback ONLY — do not use for turning while FUNMAP is active)
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

        # Trajectory action client for pre-wipe positioning
        self._traj_action_name = str(traj_action_name)
        self._traj_client = ActionClient(self.node, FollowJointTrajectory, self._traj_action_name)

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"traj_action={self._traj_action_name} "
            f"head_scan_srv={funmap_head_scan_srv} local_loc_srv={funmap_local_loc_srv} "
            f"traj_srv={switch_to_traj_srv} nav_srv={switch_to_nav_srv}"
        )

    # -----------------------------
    # Small helpers
    # -----------------------------
    def _call_trigger_sync(self, client, name: str, timeout_s: float = 5.0) -> bool:
        """
        Call a std_srvs/Trigger service and block until completion or timeout.
        """
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
        duration_sec: float = 3.0,
        server_wait_sec: float = 5.0,
        timeout_sec: float = 30.0,
    ) -> bool:
        """
        Send a single-point FollowJointTrajectory goal synchronously.
        Returns True on success (error_code==0).
        """
        if len(joint_names) != len(positions):
            self.node.get_logger().error("[POSTURE] joint_names and positions length mismatch")
            return False

        if not self._traj_client.wait_for_server(timeout_sec=float(server_wait_sec)):
            self.node.get_logger().error(f"[POSTURE] Trajectory action not available: {self._traj_action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)

        sec_i = int(duration_sec)
        nsec_i = int(max(0.0, duration_sec - sec_i) * 1e9)
        pt.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)

        goal.trajectory.points = [pt]

        self.node.get_logger().info(f"[POSTURE] Sending {joint_names} -> {positions} ({duration_sec:.1f}s)")
        send_fut = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_fut, timeout_sec=float(timeout_sec))

        if not send_fut.done():
            self.node.get_logger().error("[POSTURE] send_goal timed out")
            return False

        goal_handle = send_fut.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("[POSTURE] Goal rejected")
            return False

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=float(timeout_sec))

        if not result_fut.done():
            self.node.get_logger().error("[POSTURE] result timed out")
            return False

        result = result_fut.result().result
        # Many controllers also include error_string; guard if missing
        err_str = getattr(result, "error_string", "")
        self.node.get_logger().info(f"[POSTURE] Result error_code={result.error_code} error_string='{err_str}'")
        return int(result.error_code) == 0

    def _go_to_pre_wipe_pose(self) -> bool:
        """
        Move robot into a known-good "pre-wipe" starting posture.

        Uses values you pasted from /stretch/joint_states:
          joint_lift: 0.8838716489
          joint_arm_l0..l3: 0.0277522269
          joint_wrist_yaw: -0.0421844717

        NOTE:
        - We command arm segments (joint_arm_l0..l3) instead of 'wrist_extension'
          because some controllers don't accept a synthetic 'wrist_extension' joint.
        """
        lift = 0.8838716489173609
        arm_seg = 0.02775222692732435
        wrist_yaw = -0.042184471666855135

        # Do it in stages (easier to debug + less likely to be rejected)
        ok = self._send_traj(["joint_lift"], [lift], duration_sec=3.0, timeout_sec=30.0)
        if not ok:
            return False

        ok = self._send_traj(
            ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"],
            [arm_seg, arm_seg, arm_seg, arm_seg],
            duration_sec=4.0,
            timeout_sec=30.0,
        )
        if not ok:
            return False

        ok = self._send_traj(["joint_wrist_yaw"], [wrist_yaw], duration_sec=2.0, timeout_sec=20.0)
        return ok

    # -----------------------------
    # Deterministic base transit (legacy / fallback)
    # -----------------------------
    def transit(self, from_loc: str, to_loc: str) -> None:
        if not self.motion_enabled:
            self.node.get_logger().info("[MOTION] transit skipped (motion disabled)")
            return

        from_loc = (from_loc or "").lower().strip()
        to_loc = (to_loc or "").lower().strip()
        self.node.get_logger().info(f"[MOTION] transit requested {from_loc} -> {to_loc}")

        def _turn_left():
            # NOTE: uses cmd_vel under the hood; only safe if FUNMAP is NOT driving.
            if hasattr(self.motion, "turn_angle"):
                self.motion.turn_angle(self.turn_left_rad)
            else:
                raise RuntimeError("BaseMotion has no turn_angle")

        if from_loc == "door" and to_loc == "desk":
            d = float(self.distances.get("door_to_desk", 0.0))
            self.node.get_logger().info(f"[MOTION] door->desk drive {d:.2f}m")
            if d > 0:
                self.motion.drive_distance(d)

        elif from_loc == "desk" and to_loc == "bed":
            d = float(self.distances.get("desk_to_bed", 0.0))
            self.node.get_logger().info("[MOTION] desk->bed turn left 90 then drive")
            _turn_left()
            if d > 0:
                self.motion.drive_distance(d)

        elif from_loc == "bed" and to_loc == "kitchen":
            d = float(self.distances.get("bed_to_kitchen", 0.0))
            self.node.get_logger().info("[MOTION] bed->kitchen turn left 90 then drive")
            _turn_left()
            if d > 0:
                self.motion.drive_distance(d)

        else:
            self.node.get_logger().warn(f"[MOTION] no deterministic route for {from_loc} -> {to_loc}")

    # -----------------------------
    # Surface cleaning demo pipeline
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        """
        Automated setup + optional pre-wipe positioning + trigger clean_surface.

        Pipeline:
          1) FUNMAP head_scan (best-effort)
          2) FUNMAP local localization (best-effort)
          3) Switch to trajectory mode
          4) Move to pre-wipe pose (so the demo doesn't fail "pre wipe gesture")
          5) Trigger clean_surface

        IMPORTANT:
        - Do NOT publish cmd_vel turns here if FUNMAP/nav is active.
        """
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(f"[DEMO] Desk surface-clean requested thoroughness='{thoroughness}'")

        # 1) Head scan (best effort)
        self._call_trigger_sync(self._srv_head_scan, "funmap/trigger_head_scan", timeout_s=30.0)

        # 2) Local localization (best effort)
        self._call_trigger_sync(self._srv_local_loc, "funmap/trigger_local_localization", timeout_s=15.0)

        # 3) Switch to trajectory mode
        self._call_trigger_sync(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=10.0)

        # Small settle time sometimes helps joint_states/action server stabilize after mode switch
        t0 = time.time()
        while time.time() - t0 < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # 4) Pre-wipe posture
        if not self._go_to_pre_wipe_pose():
            self.node.get_logger().error("[DEMO] cannot perform pre wipe gesture (pre-wipe posture failed)")
            return

        # 5) Trigger clean_surface (sync)
        ok = self.clean_surface.trigger_sync(timeout_s=60.0)
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered; it will scan/plan/execute.")

        # Optional: switch back later (do NOT immediately switch; can interrupt trajectories)
        # self._call_trigger_sync(self._srv_nav_mode, "switch_to_navigation_mode", timeout_s=10.0)

    # Placeholders
    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")