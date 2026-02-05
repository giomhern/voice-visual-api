# stretch_study/capabilities/deterministic_demos.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from std_srvs.srv import Trigger

from .base_motion import BaseMotion


# -----------------------------
# Clean surface Trigger client
# -----------------------------
@dataclass
class CleanSurfaceConfig:
    # Correct default for your system (you confirmed):
    # /clean_surface/trigger_clean_surface
    service_name: str = "/clean_surface/trigger_clean_surface"
    # Give bringup time; 2s is often too short on real launches
    wait_for_service_s: float = 10.0


class CleanSurfaceClient:
    """
    Minimal client to trigger Hello Robot's clean_surface demo.

    This class triggers the demo via a ROS service call (std_srvs/srv/Trigger).
    """

    def __init__(self, node, cfg: Optional[CleanSurfaceConfig] = None):
        self.node = node
        self.cfg = cfg or CleanSurfaceConfig()
        self._client = node.create_client(Trigger, self.cfg.service_name)

    def _try_autodiscover_service(self) -> Optional[str]:
        """
        If the configured service isn't present, look for a likely clean_surface Trigger service.

        Preference:
          - name contains 'clean_surface'
          - supports std_srvs/srv/Trigger
          - name contains 'trigger'
        """
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
        """
        Ensure the Trigger service exists; if not, attempt autodiscovery and recreate client.
        """
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
        This avoids the 'async but nothing is spinning' issue.
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

    - desk_demo(): runs a SAFE setup pipeline, then triggers the official clean_surface node.
    - transit(): legacy deterministic base motion fallback (only if you enable it).
    """

    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        # NOTE: if your Stretch does NOT publish /odom, change this to the correct topic
        # (e.g., "/stretch/odom") or pass it in when constructing DeterministicDemos.
        odom_topic: str = "/odom",
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        # Setup services
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

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"head_scan_srv={funmap_head_scan_srv} local_loc_srv={funmap_local_loc_srv} "
            f"traj_srv={switch_to_traj_srv} nav_srv={switch_to_nav_srv}"
        )

    def _call_trigger_sync(self, client, name: str, timeout_s: float = 5.0) -> bool:
        """
        Call a std_srvs/Trigger service and block until completion or timeout.
        This is robust even if your app doesn't spin elsewhere.
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
        Automated setup + trigger for clean_surface.

        Safe pipeline:
          1) FUNMAP head_scan (best-effort)
          2) FUNMAP local localization (best-effort)
          3) Switch to trajectory mode (important for follow_joint_trajectory)
          4) Trigger clean_surface

        IMPORTANT:
        - Do NOT publish cmd_vel turns here if FUNMAP/nav is active.
          That causes fighting controllers and infinite spins.
        """
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(
            f"[DEMO] Desk surface-clean requested thoroughness='{thoroughness}'"
        )

        # 1) Head scan (best effort)
        self._call_trigger_sync(self._srv_head_scan, "funmap/trigger_head_scan", timeout_s=20.0)

        # 2) Local localization (best effort)
        self._call_trigger_sync(self._srv_local_loc, "funmap/trigger_local_localization", timeout_s=10.0)

        # 3) Switch to trajectory mode (helps clean_surface execute its trajectory)
        self._call_trigger_sync(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=5.0)

        # 4) Trigger clean_surface (sync so it doesn't "do nothing" if nothing else is spinning)
        ok = self.clean_surface.trigger_sync(timeout_s=30.0)
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered; it will scan/plan/execute.")

        # Optional:
        # Do NOT immediately switch back to nav mode; that can interrupt the arm trajectory.
        # If you want, do this later (e.g., next step / on user confirm):
        # self._call_trigger_sync(self._srv_nav_mode, "switch_to_navigation_mode", timeout_s=5.0)

    # Placeholders
    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")