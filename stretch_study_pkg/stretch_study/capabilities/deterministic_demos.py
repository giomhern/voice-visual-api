import threading
import time

from stretch_study.capabilities.base_motion import BaseMotion
from stretch_study.capabilities.arm_commander import ArmCommander


class DeterministicDemos:
    def __init__(self, node, motion_enabled, distances, cmd_vel_topic, odom_topic):
        self.node = node
        self.motion_enabled = motion_enabled
        self.distances = distances
        self.motion = BaseMotion(node, cmd_vel_topic, odom_topic)

        # Serialize arm demos so you don't stack goals
        self._arm_lock = threading.Lock()

        # Parameters (you can later make these ROS params)
        self.trajectory_action = "/stretch_controller/follow_joint_trajectory"  # override if different
        self.joint_states_topic = "/joint_states"

        self.arm = ArmCommander(node, action_name=self.trajectory_action, joint_states_topic=self.joint_states_topic)
        self.arm_ready = self.arm.wait_ready(timeout_s=5.0)

        # Joint set used in the doc snippet
        self.demo_joints = ["joint_lift", "joint_arm", "joint_wrist_yaw"]]

        # --- Deterministic demo "poses" (TUNE THESE NUMBERS) ---
        self.pose_stow = [0.20, 0.00, 3.14]

        # Desk wipe (gesture, no contact)
        self.pose_desk_ready = [0.50, 0.10, 0.00]
        self.pose_wipe_left  = [0.50, 0.18, +0.60]
        self.pose_wipe_right = [0.50, 0.18, -0.60]

        # Bed “pillow” gesture
        self.pose_bed_ready = [0.75, 0.10, 0.00]
        self.pose_pillow_center = [0.75, 0.20, 0.00]
        self.pose_pillow_top    = [0.85, 0.20, 0.00]

        # Kitchen snack “present”
        self.pose_kitchen_ready = [0.55, 0.10, 0.00]
        self.pose_present_snack = [0.55, 0.25, 0.00]

    def _wait_for_base_idle(self, timeout_s: float = 10.0) -> bool:
        if hasattr(self.motion, "is_busy") and self.motion.is_busy():
            self.node.get_logger().info("[DEMO] waiting for base motion to finish...")
            ok = self.motion.wait_until_idle(timeout_s=timeout_s)
            if not ok:
                self.node.get_logger().warn("[DEMO] base still moving; skipping demo.")
            return ok
        return True

    def _arm_pose(self, pose, duration_s=2.0):
        if not self.arm_ready:
            self.node.get_logger().error("[ARM] ArmCommander not ready (action server or joint_states missing).")
            return False
        return self.arm.send_pose(self.demo_joints, pose, duration_s=duration_s, wait=True)
    
    def transit(self, from_loc: str, to_loc: str):
        if not self.motion_enabled:
            self.node.get_logger().info("[MOTION] transit skipped (motion disabled)")
            return

        self.node.get_logger().info(f"[MOTION] transit requested {from_loc} -> {to_loc}")

        def seq():
            if from_loc == "door" and to_loc == "desk":
                self.node.get_logger().info("[MOTION] executing door -> desk")
                self.motion.drive_distance(self.distances["door_to_desk"])

            elif from_loc == "desk" and to_loc == "bed":
                self.node.get_logger().info("[MOTION] executing desk -> bed (turn left 90, drive)")
                self.motion.turn_left_90()
                self.motion.drive_distance(self.distances["desk_to_bed"])

            elif from_loc == "bed" and to_loc == "kitchen":
                self.node.get_logger().info("[MOTION] executing bed -> kitchen (turn left 90, drive)")
                self.motion.turn_left_90()
                self.motion.drive_distance(self.distances["bed_to_kitchen"])

            else:
                self.node.get_logger().warn(f"[MOTION] no route for {from_loc} -> {to_loc}")

        self.motion.run_sequence_async(seq)

    # -----------------------------
    # Demo safety: wait for base to stop
    # -----------------------------
    def _wait_for_base_idle(self, timeout_s: float = 10.0) -> bool:
        # If your BaseMotion doesn't yet expose these, add the small patch below.
        if not hasattr(self.motion, "is_busy"):
            # Fallback: just sleep a moment (won't be as safe, but prevents crashes)
            self.node.get_logger().warn("[DEMO] BaseMotion missing is_busy(); sleeping 0.5s fallback")
            time.sleep(0.5)
            return True

        if not self.motion.is_busy():
            return True

        self.node.get_logger().info("[DEMO] waiting for base motion to finish before starting demo...")
        ok = self.motion.wait_until_idle(timeout_s=timeout_s)
        if not ok:
            self.node.get_logger().warn("[DEMO] base motion still busy; skipping demo for safety.")
        return ok

    def desk_demo(self, thoroughness: str):
        if not self._wait_for_base_idle():
            return

        passes = {"once": 1, "twice": 2, "thorough": 3, "none": 0}.get(thoroughness, 0)
        self.node.get_logger().info(f"[DEMO] Desk demo start (thoroughness={thoroughness}, passes={passes})")

        if passes == 0:
            self.node.get_logger().info("[DEMO] Desk demo skipped (none).")
            return

        with self._arm_lock:
            self._arm_pose(self.pose_desk_ready, duration_s=2.0)
            for i in range(passes):
                self.node.get_logger().info(f"[DEMO] Wipe pass {i+1}/{passes}")
                self._arm_pose(self.pose_wipe_left, duration_s=1.0)
                self._arm_pose(self.pose_wipe_right, duration_s=1.0)
            self._arm_pose(self.pose_stow, duration_s=2.5)

        self.node.get_logger().info("[DEMO] Desk demo complete")


    def bed_demo(self, arrangement: str):
        if not self._wait_for_base_idle():
            return

        target = self.pose_pillow_top if arrangement == "top" else self.pose_pillow_center
        self.node.get_logger().info(f"[DEMO] Bed demo start (arrangement={arrangement})")

        with self._arm_lock:
            self._arm_pose(self.pose_bed_ready, duration_s=2.0)
            self._arm_pose(target, duration_s=2.0)
            time.sleep(0.5)
            self._arm_pose(self.pose_stow, duration_s=2.5)

        self.node.get_logger().info("[DEMO] Bed demo complete")


    def kitchen_demo(self, snack: str):
        if not self._wait_for_base_idle():
            return

        self.node.get_logger().info(f"[DEMO] Kitchen demo start (snack={snack})")

        with self._arm_lock:
            self._arm_pose(self.pose_kitchen_ready, duration_s=2.0)
            self._arm_pose(self.pose_present_snack, duration_s=2.0)
            time.sleep(0.5)
            self._arm_pose(self.pose_stow, duration_s=2.5)

        self.node.get_logger().info("[DEMO] Kitchen demo complete")
