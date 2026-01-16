from __future__ import annotations

import time

from stretch_study.capabilities.base_motion import BaseMotion


class DeterministicDemos:
    def __init__(self, node, motion_enabled, distances, cmd_vel_topic, odom_topic):
        self.node = node
        self.motion_enabled = motion_enabled
        self.distances = distances
        self.motion = BaseMotion(node, cmd_vel_topic, odom_topic)

    # -----------------------------
    # Transit between corners
    # -----------------------------
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

    # -----------------------------
    # Demos (deterministic placeholders)
    # -----------------------------
    def desk_demo(self, thoroughness: str):
        if not self._wait_for_base_idle():
            return

        passes = {"once": 1, "twice": 2, "thorough": 3, "none": 0}.get(thoroughness, 0)
        self.node.get_logger().info(f"[DEMO] Desk demo start (thoroughness={thoroughness}, passes={passes})")

        for i in range(passes):
            self.node.get_logger().info(f"[DEMO] Desk wipe pass {i+1}/{passes}")
            time.sleep(1.0)

        self.node.get_logger().info("[DEMO] Desk demo complete")

    def bed_demo(self, arrangement: str):
        if not self._wait_for_base_idle():
            return

        self.node.get_logger().info(f"[DEMO] Bed demo start (arrangement={arrangement})")
        time.sleep(2.0)
        self.node.get_logger().info("[DEMO] Bed demo complete")

    def kitchen_demo(self, snack: str):
        if not self._wait_for_base_idle():
            return

        self.node.get_logger().info(f"[DEMO] Kitchen demo start (snack={snack})")
        time.sleep(2.0)
        self.node.get_logger().info("[DEMO] Kitchen demo complete")