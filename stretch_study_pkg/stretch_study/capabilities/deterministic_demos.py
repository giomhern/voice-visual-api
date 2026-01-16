import time
from stretch_study.capabilities.base_motion import BaseMotion


class DeterministicDemos:
    def __init__(self, node, motion_enabled, distances, cmd_vel_topic, odom_topic):
        self.motion_enabled = motion_enabled
        self.distances = distances
        self.motion = BaseMotion(node, cmd_vel_topic, odom_topic)

    def transit(self, from_loc: str, to_loc: str):
        if not self.motion_enabled:
            return

        if from_loc == "door" and to_loc == "desk":
            self.motion.drive_distance_async(
                self.distances["door_to_desk"]
            )

        elif from_loc == "desk" and to_loc == "bed":
            self.motion.turn_left_90_async()
            time.sleep(1.0)
            self.motion.drive_distance_async(
                self.distances["desk_to_bed"]
            )

        elif from_loc == "bed" and to_loc == "kitchen":
            self.motion.turn_left_90_async()
            time.sleep(1.0)
            self.motion.drive_distance_async(
                self.distances["bed_to_kitchen"]
            )

    def desk_demo(self, thoroughness: str):
        passes = {"once": 1, "twice": 2, "thorough": 3}.get(thoroughness, 0)
        print("[DEMO] Desk demo start")
        for i in range(passes):
            print(f"[DEMO] Wipe pass {i+1}")
            time.sleep(1.0)
        print("[DEMO] Desk demo complete")

    def bed_demo(self, arrangement: str):
        print(f"[DEMO] Bed demo start – pillow to {arrangement}")
        time.sleep(2.0)
        print("[DEMO] Bed demo complete")

    def kitchen_demo(self, snack: str):
        print(f"[DEMO] Kitchen demo start – presenting {snack}")
        time.sleep(2.0)
        print("[DEMO] Kitchen demo complete")