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

        def seq():
            # door -> desk: straight
            if from_loc == "door" and to_loc == "desk":
                self.motion.drive_distance(self.distances["door_to_desk"])

            # desk -> bed: turn left 90, then straight
            elif from_loc == "desk" and to_loc == "bed":
                self.motion.turn_left_90()
                self.motion.drive_distance(self.distances["desk_to_bed"])

            # bed -> kitchen: turn left 90, then straight
            elif from_loc == "bed" and to_loc == "kitchen":
                self.motion.turn_left_90()
                self.motion.drive_distance(self.distances["bed_to_kitchen"])

        # Run sequentially in ONE exclusive motion worker
        self.motion.run_sequence_async(seq)