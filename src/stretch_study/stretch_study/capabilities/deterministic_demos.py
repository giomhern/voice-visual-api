import time
from std_srvs.srv import Trigger

class DeterministicDemos:
    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        clean_surface_service: str = "/clean_surface/trigger_clean_surface",
        # NEW: setup services (defaults match your system)
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

        # Clean surface trigger client
        self.clean_surface = CleanSurfaceClient(
            node=self.node,
            cfg=CleanSurfaceConfig(service_name=str(clean_surface_service)),
        )

        # NEW: Trigger clients for setup
        self._srv_head_scan = self.node.create_client(Trigger, funmap_head_scan_srv)
        self._srv_local_loc = self.node.create_client(Trigger, funmap_local_loc_srv)
        self._srv_traj_mode = self.node.create_client(Trigger, switch_to_traj_srv)
        self._srv_nav_mode = self.node.create_client(Trigger, switch_to_nav_srv)

        self.node.get_logger().info(
            "[DEMOS] init "
            f"clean_surface_service={clean_surface_service} "
            f"head_scan_srv={funmap_head_scan_srv} local_loc_srv={funmap_local_loc_srv} "
            f"traj_srv={switch_to_traj_srv} nav_srv={switch_to_nav_srv}"
        )

    def _call_trigger(self, client, name: str, timeout_s: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            self.node.get_logger().warn(f"[SETUP] service not available: {name}")
            return False
        fut = client.call_async(Trigger.Request())
        t0 = time.time()
        while time.time() - t0 < float(timeout_s):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if fut.done():
                try:
                    resp = fut.result()
                    ok = bool(resp.success)
                    self.node.get_logger().info(f"[SETUP] {name}: success={ok} msg='{resp.message}'")
                    return ok
                except Exception as e:
                    self.node.get_logger().warn(f"[SETUP] {name}: call failed: {e}")
                    return False
        self.node.get_logger().warn(f"[SETUP] {name}: timed out after {timeout_s:.1f}s")
        return False

    # -----------------------------
    # Surface cleaning demo pipeline
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        """
        Automated setup + trigger for clean_surface.

        Safe pipeline:
          1) head_scan (optional but helpful)
          2) local localization (optional but helpful)
          3) switch to trajectory mode (important for follow_joint_trajectory)
          4) trigger clean_surface
          5) optionally switch back to navigation mode (optional)
        """
        self.node.get_logger().info(
            f"[DEMO] Desk surface-clean requested thoroughness='{thoroughness}'"
        )

        # 1) Head scan (best effort)
        self._call_trigger(self._srv_head_scan, "funmap/trigger_head_scan", timeout_s=20.0)

        # 2) Local localization (best effort)
        self._call_trigger(self._srv_local_loc, "funmap/trigger_local_localization", timeout_s=10.0)

        # 3) Switch to trajectory mode (this matters for execution)
        self._call_trigger(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=5.0)

        # 4) Trigger clean_surface
        ok = self.clean_surface.trigger_async()
        if not ok:
            self.node.get_logger().error(
                "[DEMO] Could not trigger clean_surface. "
                "Check service exists: ros2 service list | grep trigger_clean_surface"
            )
            return

        self.node.get_logger().info("[DEMO] clean_surface triggered. Waiting for it to run...")

        # OPTIONAL:
        # Do NOT immediately switch back to nav mode, because it can interrupt the arm trajectory.
        # If you want, switch back later (e.g., after user confirmation or next step).
        # self._call_trigger(self._srv_nav_mode, "switch_to_navigation_mode", timeout_s=5.0)