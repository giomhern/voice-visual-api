from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional

from rclpy.node import Node

from .base_motion import BaseMotion


class DeterministicDemos:
    """Deterministic, low-variance demo behaviors.

    These are *gesture demos* by default (safe & repeatable).
    Swap implementation with real Stretch APIs incrementally.
    """

    def __init__(
        self,
        node: Optional[Node] = None,
        logger=None,
        say_fn=None,
        motion_cfg: Optional[Dict[str, Any]] = None,
    ):
        self.log = logger
        self.say = say_fn or (lambda text: None)
        self.node = node

        motion_cfg = motion_cfg or {}
        self.motion_enabled = bool(motion_cfg.get('enable_transit', False))
        self.cmd_vel_topic = str(motion_cfg.get('cmd_vel_topic', '/stretch/cmd_vel'))
        self.odom_topic = str(motion_cfg.get('odom_topic', '/odom'))
        self.turn_left_rad = float(motion_cfg.get('turn_left_rad', math.pi / 2))
        self.distances = dict(motion_cfg.get('distances_m', {}))

        self._mover: Optional[BaseMotion] = None
        if self.node is not None:
            # We always create the mover if a node is provided; callers can still disable motion.
            self._mover = BaseMotion(self.node, cmd_vel_topic=self.cmd_vel_topic, odom_topic=self.odom_topic)

    def _sleep(self, seconds: float):
        time.sleep(seconds)

    # ----------------------
    # Deterministic base transit (optional)
    # ----------------------
    def transit(self, from_loc: str, to_room: str, movement_speed: str = 'medium') -> None:
        """Move the base along the assumed rectangle route.

        Intended for controlled demo testing. Enable with motion.enable_transit.

        Route assumed:
          door (bottom-right) -> desk (top-right) -> bed (top-left) -> kitchen (bottom-left)
        """
        if not self.motion_enabled:
            return
        if self._mover is None or self.node is None:
            raise RuntimeError('BaseMotion not initialized (node missing).')

        from_loc = (from_loc or '').lower()
        to_room = (to_room or '').lower()
        self._mover.set_speed_profile(movement_speed)

        # Determine the leg to execute
        # legs are configured by keys in defaults.yaml
        if from_loc == 'door' and to_room == 'desk':
            d = float(self.distances.get('door_to_desk', 0.0))
            self.say('Moving to the desk area.')
            self._mover.drive_distance(d)
            return
        if from_loc == 'desk' and to_room == 'bed':
            d = float(self.distances.get('desk_to_bed', 0.0))
            self.say('Moving to the bed area.')
            self._mover.turn_angle(self.turn_left_rad)
            self._mover.drive_distance(d)
            return
        if from_loc == 'bed' and to_room == 'kitchen':
            d = float(self.distances.get('bed_to_kitchen', 0.0))
            self.say('Moving to the kitchen area.')
            self._mover.turn_angle(self.turn_left_rad)
            self._mover.drive_distance(d)
            return

        # Unknown transition: do nothing but log
        if self.log:
            self.log.log_event({'type': 'transit_skipped', 'from': from_loc, 'to': to_room})

    def desk_demo(self, thoroughness: str, eff: Dict[str, Any]) -> None:
        passes = {'once': 1, 'twice': 2, 'thorough': 3, 'none': 0}.get(thoroughness, 1)
        self.say(f"Here is how I will wipe your desk: {thoroughness}.")
        if self.log:
            self.log.log_event({'type': 'demo_start', 'room': 'desk', 'thoroughness': thoroughness, 'effective': eff})

        if passes == 0:
            self.say("I will not wipe the desk.")
            self._sleep(0.5)
        else:
            for i in range(passes):
                # placeholder gesture timing
                self.say(f"Wipe pass {i+1}.")
                self._sleep(1.0)

        if self.log:
            self.log.log_event({'type': 'demo_end', 'room': 'desk'})

    def bed_demo(self, arrangement: str, eff: Dict[str, Any]) -> None:
        self.say(f"I will move the pillow to the {arrangement} of the bed.")
        if self.log:
            self.log.log_event({'type': 'demo_start', 'room': 'bed', 'pillow_arrangement': arrangement, 'effective': eff})

        # placeholder gesture
        self._sleep(1.5)

        if self.log:
            self.log.log_event({'type': 'demo_end', 'room': 'bed'})

    def kitchen_demo(self, snack: str, eff: Dict[str, Any]) -> None:
        self.say(f"I will retrieve {snack} and present it.")
        if self.log:
            self.log.log_event({'type': 'demo_start', 'room': 'kitchen', 'snack_preference': snack, 'effective': eff})

        # placeholder: reach -> grasp -> present
        self._sleep(2.0)
        self.say(f"Here you go: {snack}.")
        self._sleep(0.5)

        if self.log:
            self.log.log_event({'type': 'demo_end', 'room': 'kitchen'})
