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
        self.joint_states_topic = "/stretch/joint_states"

        self.arm = ArmCommander(
            node,
            action_name=self.trajectory_action,
            joint_states_topic=self.joint_states_topic,
        )
        self.arm_ready = self.arm.wait_ready(timeout_s=5.0)

        # Joint set for desk demo (lift + extension + pitch + yaw)
        self.desk_joints = ["joint_lift", "wrist_extension", "joint_wrist_pitch", "joint_wrist_yaw"]

        # Joint sets for simpler demos (lift + extension + yaw)
        self.simple_joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]

        # --- Simple (existing) poses (kept intact for bed/kitchen) ---
        self.pose_stow = [0.20, 0.00, 0.00]  # simple joints
        self.pose_bed_ready = [0.75, 0.10, 0.00]
        self.pose_pillow_center = [0.75, 0.20, 0.00]
        self.pose_pillow_top = [0.85, 0.20, 0.00]
        self.pose_kitchen_ready = [0.55, 0.10, 0.00]
        self.pose_present_snack = [0.55, 0.25, 0.00]

    def set_speed_mode(self, speed: str):
        """
        Apply user-facing speed labels to real motion speeds.
        This affects deterministic transit + any BaseMotion usage.
        """
        s = (speed or "").lower().strip()
        if s == "slow":
            self.motion.linear_speed = 0.08
            self.motion.angular_speed = 0.4
        elif s == "fast":
            self.motion.linear_speed = 0.18
            self.motion.angular_speed = 0.9
        else:
            # medium/default
            self.motion.linear_speed = 0.12
            self.motion.angular_speed = 0.6

        self.node.get_logger().info(
            f"[MOTION] set_speed_mode={s} -> linear_speed={self.motion.linear_speed:.2f} m/s "
            f"angular_speed={self.motion.angular_speed:.2f} rad/s"
        )

    def _arm_pose(self, joint_names, pose, duration_s=2.0):
        if not self.arm_ready:
            self.node.get_logger().error("[ARM] ArmCommander not ready (action server or joint_states missing).")
            return False
        return self.arm.send_pose(joint_names, pose, duration_s=duration_s, wait=True)

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

    def _wait_for_base_idle(self, timeout_s: float = 10.0) -> bool:
        if not hasattr(self.motion, "is_busy"):
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

    # -----------------------------
    # UPDATED DESK DEMO ONLY
    # -----------------------------
    def desk_demo(self, thoroughness: str):
        if not self._wait_for_base_idle():
            return

        passes = {"once": 1, "twice": 2, "thorough": 3, "none": 0}.get(thoroughness, 0)
        self.node.get_logger().info(f"[DEMO] Desk demo start (thoroughness={thoroughness}, passes={passes})")

        if passes == 0:
            self.node.get_logger().info("[DEMO] Desk demo skipped (none).")
            return

        if not self.arm_ready:
            self.node.get_logger().error("[DEMO] Arm not ready; cannot run desk demo.")
            return

        # ---- Calibration knobs (tune safely) ----
        # If gripper faces right when yaw=0, set straight_yaw to +/-1.57
        straight_yaw = 0.0
        lift_high = 0.80
        ext_retracted = 0.05
        ext_reach = 0.22
        pitch_hover = -0.20
        pitch_wipe = -0.95
        yaw_span = 0.80
        sweeps_per_pass = 3
        # ----------------------------------------

        stow = [0.20, 0.00, 0.00, 0.00]
        high_hover = [lift_high, ext_retracted, pitch_hover, straight_yaw]
        reach_pose = [lift_high, ext_reach, pitch_hover, straight_yaw]
        wipe_ready = [lift_high, ext_reach, pitch_wipe, straight_yaw]

        left = straight_yaw + yaw_span
        right = straight_yaw - yaw_span

        with self._arm_lock:
            self.node.get_logger().info("[DEMO] Phase 1: lift high + align yaw straight")
            self.arm.send_pose(self.desk_joints, high_hover, duration_s=2.0, wait=True)

            self.node.get_logger().info("[DEMO] Phase 2: extend straight once")
            self.arm.send_pose(self.desk_joints, reach_pose, duration_s=1.5, wait=True)

            self.node.get_logger().info("[DEMO] Phase 3: pitch down for wiping")
            self.arm.send_pose(self.desk_joints, wipe_ready, duration_s=1.0, wait=True)

            for p in range(passes):
                self.node.get_logger().info(f"[DEMO] Phase 4: wiping pass {p+1}/{passes}")

                points = []
                t = 0.8
                points.append((wipe_ready, t))

                for _ in range(sweeps_per_pass):
                    t += 0.8
                    points.append(([wipe_ready[0], wipe_ready[1], wipe_ready[2], left], t))
                    t += 0.8
                    points.append(([wipe_ready[0], wipe_ready[1], wipe_ready[2], right], t))

                t += 0.6
                points.append(([wipe_ready[0], wipe_ready[1], wipe_ready[2], straight_yaw], t))

                self.arm.send_trajectory(self.desk_joints, points, wait=True)

            self.node.get_logger().info("[DEMO] Phase 5: retract + stow")
            self.arm.send_pose(self.desk_joints, high_hover, duration_s=1.5, wait=True)
            self.arm.send_pose(self.desk_joints, stow, duration_s=2.5, wait=True)

        self.node.get_logger().info("[DEMO] Desk demo complete")

    def bed_demo(self, arrangement: str):
        if not self._wait_for_base_idle():
            return

        target = self.pose_pillow_top if arrangement == "top" else self.pose_pillow_center
        self.node.get_logger().info(f"[DEMO] Bed demo start (arrangement={arrangement})")

        with self._arm_lock:
            self._arm_pose(self.simple_joints, self.pose_bed_ready, duration_s=2.0)
            self._arm_pose(self.simple_joints, target, duration_s=2.0)
            time.sleep(0.5)
            self._arm_pose(self.simple_joints, self.pose_stow, duration_s=2.5)

        self.node.get_logger().info("[DEMO] Bed demo complete")

    def kitchen_demo(self, snack: str):
        if not self._wait_for_base_idle():
            return

        self.node.get_logger().info(f"[DEMO] Kitchen demo start (snack={snack})")

        with self._arm_lock:
            self._arm_pose(self.simple_joints, self.pose_kitchen_ready, duration_s=2.0)
            self._arm_pose(self.simple_joints, self.pose_present_snack, duration_s=2.0)
            time.sleep(0.5)
            self._arm_pose(self.simple_joints, self.pose_stow, duration_s=2.5)

        self.node.get_logger().info("[DEMO] Kitchen demo complete")