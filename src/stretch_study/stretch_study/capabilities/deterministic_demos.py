from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from std_srvs.srv import Trigger

from .base_motion import BaseMotion


@dataclass
class CleanSurfaceConfig:
    # Correct default based on your system:
    # ros2 node info /clean_surface  -> /clean_surface/trigger_clean_surface
    service_name: str = "/clean_surface/trigger_clean_surface"
    wait_for_service_s: float = 2.0


class CleanSurfaceClient:
    """
    Minimal client to trigger Hello Robot's clean_surface demo.

    This class only triggers the demo via a ROS service call.
    """

    def __init__(self, node, cfg: Optional[CleanSurfaceConfig] = None):
        self.node = node
        self.cfg = cfg or CleanSurfaceConfig()
        self._client = node.create_client(Trigger, self.cfg.service_name)

    def _try_autodiscover_service(self) -> Optional[str]:
        """
        If the configured service isn't present, look for a likely clean_surface Trigger service.

        We prefer:
          - service name contains 'trigger'
          - service supports std_srvs/srv/Trigger
        """
        names_and_types = self.node.get_service_names_and_types()
        candidates = []

        for (name, types) in names_and_types:
            # types is usually a list like ['std_srvs/srv/Trigger']
            if "clean_surface" in name:
                candidates.append((name, types))

        if not candidates:
            return None

        # Prefer a Trigger service with 'trigger' in the name
        for (name, types) in candidates:
            if any(t == "std_srvs/srv/Trigger" for t in types) and ("trigger" in name.lower()):
                return name

        # Otherwise any Trigger service
        for (name, types) in candidates:
            if any(t == "std_srvs/srv/Trigger" for t in types):
                return name

        # Last resort: return first candidate name (we'll still attempt Trigger)
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

    def trigger_async(self) -> bool:
        """
        Fire-and-forget: sends the trigger request asynchronously.
        Returns True if we successfully dispatched the request (not whether cleaning succeeded).
        """
        if not self._ensure_client():
            return False

        req = Trigger.Request()
        future = self._client.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                self.node.get_logger().info(
                    f"[CLEAN_SURFACE] Trigger response: success={resp.success} message='{resp.message}'"
                )
            except Exception as e:
                self.node.get_logger().error(f"[CLEAN_SURFACE] Trigger call failed: {e}")

        future.add_done_callback(_done_cb)
        self.node.get_logger().info(f"[CLEAN_SURFACE] Trigger sent to {self.cfg.service_name}")
        return True


class DeterministicDemos:
    """
    Demo behaviors for StudyEngine.

    desk_demo() triggers Hello Robot's clean_surface node.
    """

    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        # IMPORTANT: correct default service name:
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        **_ignored_kwargs,
    ):
        self.node = node
        self.motion_enabled = bool(motion_enabled)
        self.distances: Dict[str, float] = dict(distances or {})
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.odom_topic = str(odom_topic)

        # Base motion helper (legacy / fallback ONLY)
        self.motion = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)

        # Turn left 90 degrees (kept for deterministic transit ONLY)
        self.turn_left_rad = math.pi / 2.0

        # Clean surface trigger client
        self.clean_surface = CleanSurfaceClient(
            node=self.node,
            cfg=CleanSurfaceConfig(service_name=str(clean_surface_service)),
        )

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"distances={self.distances} clean_surface_service={clean_surface_service}"
        )

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
    # Surface cleaning (real clean_surface demo)
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        """
        Trigger the official clean_surface node.

        NOTE:
        - Do NOT publish cmd_vel turns here if FUNMAP/nav is active.
          That causes fighting controllers and infinite spins.
        """
        self.node.get_logger().info(
            f"[DEMO] Desk surface-clean demo requested thoroughness='{thoroughness}'. Triggering clean_surface..."
        )

        ok = self.clean_surface.trigger_async()
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered; robot will scan/plan/execute.")

    # Placeholders
    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")