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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

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
    desk_demo(): switch to nav -> turn left -> switch to trajectory -> PRE-WIPE ACTION GOAL -> trigger clean_surface
    transit(): legacy deterministic base motion (not used by desk_demo)
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
        # Trajectory action:
        traj_action_name: str = "/stretch_controller/follow_joint_trajectory",
        joint_states_topic: str = "/stretch/joint_states",
        **_ignored_kwargs,
    ):
        self.node = node
        self.motion_enabled = bool(motion_enabled)
        self.distances: Dict[str, float] = dict(distances or {})
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.odom_topic = str(odom_topic)

        # Legacy base motion helper (not required for desk_demo, but kept)
        self.motion = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)
        self.turn_left_rad = math.pi / 2.0

        # cmd_vel publisher for turn
        self._cmd_vel_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Cache joint states (CRITICAL for 2-point trajectories)
        self._latest_joint_state: Optional[JointState] = None

        def _on_js(msg: JointState):
            self._latest_joint_state = msg

        self._js_sub = self.node.create_subscription(JointState, joint_states_topic, _on_js, 10)

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
            f"joint_states={joint_states_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"traj_action={self._traj_action_name}"
        )

    # -----------------------------
    # Service helpers
    # -----------------------------
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

    def _switch_mode(self, client, name: str, timeout_s: float = 10.0) -> bool:
        ok = self._call_trigger_sync(client, name, timeout_s=timeout_s)
        time.sleep(0.25)  # settle
        return ok

    # -----------------------------
    # Base helpers
    # -----------------------------
    def _stop_base(self, settle_s: float = 0.2) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(5):
            self._cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        time.sleep(settle_s)

    def _turn_left_cmd_vel(
        self,
        angle_rad: float,
        ang_vel_rad_s: float = 0.35,
        timeout_s: float = 12.0,
    ) -> bool:
        """Simple time-based rotation."""
        if abs(angle_rad) < 1e-3:
            return True

        duration = abs(angle_rad) / max(1e-3, abs(ang_vel_rad_s))
        duration = min(duration, timeout_s)

        twist = Twist()
        twist.angular.z = abs(ang_vel_rad_s) * (1.0 if angle_rad > 0 else -1.0)

        t0 = time.time()
        while (time.time() - t0) < duration:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self._stop_base(settle_s=0.3)
        return True

    # -----------------------------
    # Trajectory helpers
    # -----------------------------
    def _get_current_joint_positions(self, joint_names: List[str], timeout_s: float = 2.0) -> Optional[List[float]]:
        """Read latest JointState and return positions for joint_names in order."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            js = self._latest_joint_state
            if js and js.name and js.position and len(js.name) == len(js.position):
                try:
                    out = []
                    for j in joint_names:
                        if j not in js.name:
                            return None
                        idx = js.name.index(j)
                        out.append(float(js.position[idx]))
                    return out
                except Exception:
                    return None
        return None

    def _send_traj(
        self,
        joint_names: List[str],
        positions: List[float],
        duration_sec: float = 2.0,
        server_wait_sec: float = 5.0,
        timeout_sec: float = 30.0,
    ) -> bool:
        """Send FollowJointTrajectory with >=2 points, stamp=0 (Stretch trajectory mode requirement)."""
        if len(joint_names) != len(positions):
            self.node.get_logger().error("[PREP] joint_names and positions length mismatch")
            return False

        if not self._traj_client.wait_for_server(timeout_sec=float(server_wait_sec)):
            self.node.get_logger().error(f"[PREP] Trajectory action not available: {self._traj_action_name}")
            return False

        current = self._get_current_joint_positions(joint_names, timeout_s=2.0)
        if current is None:
            self.node.get_logger().warn("[PREP] Could not read joint_states for start point; using target as start.")
            current = list(positions)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        # ✅ IMPORTANT: Stretch trajectory mode rejects future header stamps
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        # Point 0: start at current position
        p0 = JointTrajectoryPoint()
        p0.positions = list(current)
        p0.time_from_start = Duration(sec=0, nanosec=0)

        # Point 1: target at t=duration
        p1 = JointTrajectoryPoint()
        p1.positions = list(positions)

        if duration_sec <= 0.05:
            duration_sec = 0.5  # safety minimum

        sec_i = int(duration_sec)
        nsec_i = int((duration_sec - sec_i) * 1e9)
        p1.time_from_start = Duration(sec=sec_i, nanosec=nsec_i)

        # Guarantee strictly increasing time
        if p1.time_from_start.sec == 0 and p1.time_from_start.nanosec == 0:
            p1.time_from_start = Duration(sec=1, nanosec=0)

        goal.trajectory.points = [p0, p1]

        self.node.get_logger().info(f"[PREP] Sending traj {joint_names} -> {positions}")
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
        """Your preprocessing pose."""
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
        Pipeline:
          1) switch to navigation mode
          2) turn left (cmd_vel)
          3) switch to trajectory mode
          4) preprocessing arm pose
          5) trigger clean_surface
        """
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(f"[DEMO] Desk clean requested thoroughness='{thoroughness}'")

        # 1) NAV mode for base motion
        self._switch_mode(self._srv_nav_mode, "switch_to_navigation_mode", timeout_s=10.0)

        # 2) Turn left
        if not self._turn_left_cmd_vel(self.turn_left_rad, ang_vel_rad_s=0.35, timeout_s=12.0):
            self.node.get_logger().error("[DEMO] Turn-left failed; aborting.")
            return

        # 3) TRAJ mode for arm
        self._switch_mode(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=10.0)

        # 4) Preprocessing posture
        if not self._preprocess_pose_exact():
            self.node.get_logger().error("[DEMO] Preprocessing pose failed; not triggering clean_surface.")
            return

        # 5) Trigger clean_surface
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