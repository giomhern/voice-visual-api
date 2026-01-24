from __future__ import annotations

import math
import time
from typing import Dict, Optional

from .base_motion import BaseMotion


class DeterministicDemos:
    """
    Deterministic, low-variance demo behaviors.

    Compatible with StudyEngine calls:
      - transit(from_loc, to_loc)
      - desk_demo(thoroughness)
      - bed_demo(arrangement)
      - kitchen_demo(snack)
      - set_speed_mode(speed)   (for movement_speed)
    """

    def __init__(
        self,
        node,
        motion_enabled: bool = False,
        distances: Optional[Dict[str, float]] = None,
        cmd_vel_topic: str = "/stretch/cmd_vel",
        odom_topic: str = "/odom",
        **_ignored_kwargs,  # swallow unexpected keyword args safely
    ):
        self.node = node
        self.motion_enabled = bool(motion_enabled)
        self.distances: Dict[str, float] = dict(distances or {})
        self.cmd_vel_topic = str(cmd_vel_topic)
        self.odom_topic = str(odom_topic)

        # Create the base motion helper
        self.motion = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)

        # Default turn left (90 degrees)
        self.turn_left_rad = math.pi / 2.0

        self.node.get_logger().info(
            f"[DEMOS] init motion_enabled={self.motion_enabled} cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} distances={self.distances}"
        )

    # -----------------------------
    # Real motion tuning hook
    # -----------------------------
    def set_speed_mode(self, speed: str):
        """
        Apply user-facing speed labels to real deterministic motion speeds.
        This affects BaseMotion linear/angular speed.
        """
        s = (speed or "").lower().strip()

        # These are conservative defaults; tune as needed
        if s == "slow":
            self.motion.linear_speed = 0.08
            self.motion.angular_speed = 0.4
        elif s == "fast":
            self.motion.linear_speed = 0.18
            self.motion.angular_speed = 0.9
        else:
            self.motion.linear_speed = 0.12
            self.motion.angular_speed = 0.6

        self.node.get_logger().info(
            f"[MOTION] set_speed_mode={s} -> linear_speed={self.motion.linear_speed:.2f} m/s "
            f"angular_speed={self.motion.angular_speed:.2f} rad/s"
        )

    # -----------------------------
    # Deterministic base transit (rectangle route)
    # -----------------------------
    def transit(self, from_loc: str, to_loc: str) -> None:
        """
        Move the base along the assumed rectangle route.

        Route assumed:
          door (bottom-right) -> desk (top-right) -> bed (top-left) -> kitchen (bottom-left)
        """
        if not self.motion_enabled:
            self.node.get_logger().info("[MOTION] transit skipped (motion disabled)")
            return

        from_loc = (from_loc or "").lower().strip()
        to_loc = (to_loc or "").lower().strip()

        self.node.get_logger().info(f"[MOTION] transit requested {from_loc} -> {to_loc}")

        # Helper that turns left by 90 in a compatible way
        def _turn_left():
            if hasattr(self.motion, "turn_left_90"):
                self.motion.turn_left_90()
            elif hasattr(self.motion, "turn_angle"):
                self.motion.turn_angle(self.turn_left_rad)
            else:
                raise RuntimeError("BaseMotion has no turn_left_90 or turn_angle")

        # Build sequential motion
        def seq():
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

        # If BaseMotion supports serialized async sequences, use it.
        if hasattr(self.motion, "run_sequence_async"):
            self.motion.run_sequence_async(seq)
        else:
            # Fallback: run inline (blocking)
            seq()

    # -----------------------------
    # Demos (gesture placeholders)
    # -----------------------------
    def desk_demo(self, thoroughness: str) -> None:
        passes = {"once": 1, "twice": 2, "thorough": 3, "none": 0}.get((thoroughness or "").lower().strip(), 2)
        self.node.get_logger().info(f"[DEMO] Desk demo start thoroughness={thoroughness} passes={passes}")

        if passes == 0:
            self.node.get_logger().info("[DEMO] Desk demo skipped (none)")
            return

        for i in range(passes):
            self.node.get_logger().info(f"[DEMO] Desk wipe pass {i+1}/{passes}")
            time.sleep(1.0)

        self.node.get_logger().info("[DEMO] Desk demo complete")

    def bed_demo(self, arrangement: str) -> None:
        arrangement = (arrangement or "top").lower().strip()
        self.node.get_logger().info(f"[DEMO] Bed demo start arrangement={arrangement}")
        time.sleep(2.0)
        self.node.get_logger().info("[DEMO] Bed demo complete")

    def kitchen_demo(self, snack: str) -> None:
        snack = (snack or "doritos").lower().strip()
        self.node.get_logger().info(f"[DEMO] Kitchen demo start snack={snack}")
        time.sleep(2.0)
        self.node.get_logger().info("[DEMO] Kitchen demo complete")