# stretch_study/capabilities/deterministic_demos.py
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple, Callable

import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from action_msgs.msg import GoalStatus
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

    IMPORTANT:
    - In a larger app (StudyEngine), your node is likely already being spun by an executor.
      So we avoid spin_until_future_complete and use async callbacks.
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

    def trigger_async(self, on_done: Optional[Callable[[bool], None]] = None) -> bool:
        """
        Fire-and-log trigger. Returns True if request was dispatched.
        Calls on_done(success: bool) when response arrives.
        """
        if not self._ensure_client():
            return False

        fut = self._client.call_async(Trigger.Request())

        def _done_cb(f):
            ok = False
            try:
                resp = f.result()
                ok = bool(resp.success)
                self.node.get_logger().info(
                    f"[CLEAN_SURFACE] Trigger response: success={resp.success} message='{resp.message}'"
                )
            except Exception as e:
                self.node.get_logger().error(f"[CLEAN_SURFACE] Trigger call failed: {e}")

            if on_done:
                try:
                    on_done(ok)
                except Exception as e:
                    self.node.get_logger().error(f"[CLEAN_SURFACE] on_done callback error: {e}")

        fut.add_done_callback(_done_cb)
        self.node.get_logger().info(f"[CLEAN_SURFACE] Trigger sent to {self.cfg.service_name}")
        return True


# -----------------------------
# Pre-wipe posture (your recorded joint state)
# -----------------------------
@dataclass
class PreWipePose:
    # Your recorded snapshot ordering
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
    Sends a FollowJointTrajectory goal to Stretch trajectory controller.

    IMPORTANT:
    - We do NOT spin in here. Your app's executor should already be spinning.
    - We use callbacks on futures.
    """

    def __init__(self, node, action_name: str = "/stretch_controller/follow_joint_trajectory"):
        self.node = node
        self._action_name = str(action_name)
        self._client = ActionClient(self.node, FollowJointTrajectory, self._action_name)
        self.node.get_logger().info(f"[POSTURE] Using FollowJointTrajectory action: {self._action_name}")

    def move_to_async(
        self,
        joint_names: List[str],
        positions: List[float],
        move_time_s: float = 4.0,
        wait_for_action_s: float = 5.0,
        on_done: Optional[Callable[[bool], None]] = None,
    ) -> bool:
        if len(joint_names) != len(positions):
            self.node.get_logger().error("[POSTURE] joint_names and positions length mismatch")
            return False

        if not self._client.wait_for_server(timeout_sec=float(wait_for_action_s)):
            self.node.get_logger().error(f"[POSTURE] Action server not available: {self._action_name}")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = int(move_time_s)
        pt.time_from_start.nanosec = int((move_time_s - int(move_time_s)) * 1e9)
        goal.trajectory.points = [pt]

        self.node.get_logger().info(f"[POSTURE] Sending trajectory goal ({len(joint_names)} joints)...")
        send_fut = self._client.send_goal_async(goal)

        def _on_goal_sent(f):
            try:
                goal_handle = f.result()
            except Exception as e:
                self.node.get_logger().error(f"[POSTURE] send_goal failed: {e}")
                if on_done:
                    on_done(False)
                return

            if not goal_handle.accepted:
                self.node.get_logger().error("[POSTURE] Trajectory goal rejected")
                if on_done:
                    on_done(False)
                return

            self.node.get_logger().info("[POSTURE] Goal accepted; waiting for result...")
            result_fut = goal_handle.get_result_async()

            def _on_result(rf):
                ok = False
                try:
                    res = rf.result()
                    status = res.status
                    ok = (status == GoalStatus.STATUS_SUCCEEDED)
                    self.node.get_logger().info(f"[POSTURE] Result status={status} ok={ok}")
                except Exception as e:
                    self.node.get_logger().error(f"[POSTURE] get_result failed: {e}")

                if on_done:
                    on_done(ok)

            result_fut.add_done_callback(_on_result)

        send_fut.add_done_callback(_on_goal_sent)
        return True


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
        # Setup services
        funmap_head_scan_srv: str = "/funmap/trigger_head_scan",
        funmap_local_loc_srv: str = "/funmap/trigger_local_localization",
        switch_to_traj_srv: str = "/switch_to_trajectory_mode",
        switch_to_nav_srv: str = "/switch_to_navigation_mode",
        # Trajectory action
        follow_joint_traj_action: str = "/stretch_controller/follow_joint_trajectory",
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

        # Setup trigger clients
        self._srv_head_scan = self.node.create_client(Trigger, funmap_head_scan_srv)
        self._srv_local_loc = self.node.create_client(Trigger, funmap_local_loc_srv)
        self._srv_traj_mode = self.node.create_client(Trigger, switch_to_traj_srv)
        self._srv_nav_mode = self.node.create_client(Trigger, switch_to_nav_srv)

        # Posture mover + config
        self._posture = JointTrajectoryMover(self.node, action_name=follow_joint_traj_action)
        self._pre_wipe_pose = PreWipePose()

        self.node.get_logger().info(
            "[DEMOS] init "
            f"motion_enabled={self.motion_enabled} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"clean_surface_service={clean_surface_service} "
            f"traj_action={follow_joint_traj_action} "
            f"traj_srv={switch_to_traj_srv} nav_srv={switch_to_nav_srv}"
        )

    # -----------------------------
    # Async Trigger helper (no internal spinning)
    # -----------------------------
    def _call_trigger_async(
        self,
        client,
        name: str,
        timeout_s: float = 5.0,
        on_done: Optional[Callable[[bool], None]] = None,
    ) -> bool:
        if not client.wait_for_service(timeout_sec=float(timeout_s)):
            self.node.get_logger().warn(f"[SETUP] service not available: {name}")
            if on_done:
                on_done(False)
            return False

        fut = client.call_async(Trigger.Request())

        def _done_cb(f):
            ok = False
            try:
                resp = f.result()
                ok = bool(resp.success)
                self.node.get_logger().info(f"[SETUP] {name}: success={ok} msg='{resp.message}'")
            except Exception as e:
                self.node.get_logger().warn(f"[SETUP] {name}: call failed: {e}")

            if on_done:
                on_done(ok)

        fut.add_done_callback(_done_cb)
        self.node.get_logger().info(f"[SETUP] {name}: request sent (async)")
        return True

    # -----------------------------
    # Filter your recorded pose down to joints the trajectory controller typically owns
    # -----------------------------
    def _filtered_pre_wipe(self) -> Tuple[List[str], List[float]]:
        cfg = self._pre_wipe_pose
        allowed = {
            "joint_lift",
            "joint_arm_l0",
            "joint_arm_l1",
            "joint_arm_l2",
            "joint_arm_l3",
            "joint_wrist_yaw",
            "joint_wrist_pitch",
            "joint_wrist_roll",
        }
        names: List[str] = []
        pos: List[float] = []
        for j, p in zip(cfg.joint_names, cfg.positions):
            if j in allowed:
                names.append(j)
                pos.append(float(p))

        if not names:
            self.node.get_logger().error("[POSTURE] Filtered pre-wipe pose is empty. Check joint names.")
        return names, pos

    # -----------------------------
    # Optional deterministic base transit (legacy / fallback)
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
    # Surface cleaning demo pipeline
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        """
        Sequence:
          0) (optional) switch_to_trajectory_mode
          1) move to pre-wipe posture via FollowJointTrajectory
          2) trigger clean_surface

        NOTE:
        - This is fully async (no spin_until_future_complete) and assumes your StudyEngine/executor is spinning.
        """
        thoroughness = (thoroughness or "").lower().strip()
        self.node.get_logger().info(f"[DEMO] Desk surface-clean requested thoroughness='{thoroughness}'")

        # Build filtered goal
        names, pos = self._filtered_pre_wipe()
        cfg = self._pre_wipe_pose
        if not names:
            self.node.get_logger().error("[DEMO] Pre-wipe pose empty after filtering; aborting.")
            return

        def _after_clean_surface(ok: bool):
            # clean_surface Trigger returns quickly; the behavior continues internally
            if ok:
                self.node.get_logger().info("[DEMO] clean_surface triggered; it will scan/plan/execute.")
            else:
                self.node.get_logger().error("[DEMO] clean_surface trigger failed.")

        def _after_pose(ok: bool):
            if not ok:
                self.node.get_logger().error(
                    "[DEMO] Could not reach pre-wipe posture. "
                    "If this persists, verify the controller owns these joints."
                )
                return
            self.node.get_logger().info("[DEMO] Pre-wipe reached; triggering clean_surface...")
            if not self.clean_surface.trigger_async(on_done=_after_clean_surface):
                self.node.get_logger().error(
                    "[DEMO] Could not dispatch clean_surface trigger (service missing?)."
                )

        def _after_traj_mode(_ok: bool):
            # Even if switching fails, we still attempt to move (sometimes already in traj mode).
            self.node.get_logger().info("[DEMO] Sending pre-wipe trajectory...")
            if not self._posture.move_to_async(
                joint_names=names,
                positions=pos,
                move_time_s=float(cfg.move_time_s),
                wait_for_action_s=float(cfg.wait_for_action_s),
                on_done=_after_pose,
            ):
                self.node.get_logger().error("[DEMO] Failed to dispatch trajectory goal (action server missing?).")

        # Optional but recommended for Stretch controller ownership
        self._call_trigger_async(self._srv_traj_mode, "switch_to_trajectory_mode", timeout_s=10.0, on_done=_after_traj_mode)

    # Placeholders
    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo placeholder arrangement={arrangement}")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo placeholder snack={snack}")