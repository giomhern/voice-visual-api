import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stretch_study.core.script_steps import build_script
from stretch_study.core.logger import StudyLogger
from stretch_study.capabilities.deterministic_demos import DeterministicDemos


class StudyEngine(Node):
    def __init__(self):
        super().__init__("stretch_study_engine")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("motion.enable_transit", False)
        self.declare_parameter("motion.cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("motion.odom_topic", "/odom")
        self.declare_parameter("motion.distances_m.door_to_desk", 0.0)
        self.declare_parameter("motion.distances_m.desk_to_bed", 0.0)
        self.declare_parameter("motion.distances_m.bed_to_kitchen", 0.0)

        self.declare_parameter("study.session_id", "debug_session")
        self.declare_parameter("study.participant_id", "debug_participant")
        # Default log dir: ~/stretch_study_logs
        self.declare_parameter("study.log_dir", "~/stretch_study_logs")

        # -------------------------
        # Read parameters
        # -------------------------
        self.motion_enabled = bool(self.get_parameter("motion.enable_transit").value)
        self.cmd_vel_topic = str(self.get_parameter("motion.cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("motion.odom_topic").value)

        self.study_session_id = str(self.get_parameter("study.session_id").value)
        self.study_participant_id = str(self.get_parameter("study.participant_id").value)
        self.study_log_dir = str(self.get_parameter("study.log_dir").value)

        self.distances = {
            "door_to_desk": float(self.get_parameter("motion.distances_m.door_to_desk").value),
            "desk_to_bed": float(self.get_parameter("motion.distances_m.desk_to_bed").value),
            "bed_to_kitchen": float(self.get_parameter("motion.distances_m.bed_to_kitchen").value),
        }

        # -------------------------
        # ROS Pub/Sub
        # -------------------------
        self.pub_prompt = self.create_publisher(String, "/study_prompt", 10)
        self.sub_event = self.create_subscription(
            String, "/study_event", self.on_event, 10
        )

        # -------------------------
        # Logger (matches your StudyLogger signature)
        # -------------------------
        self.logger = StudyLogger(
            output_dir=self.study_log_dir,
            session_id=self.study_session_id,
            participant_id=self.study_participant_id,
        )

        # -------------------------
        # Script + capabilities
        # -------------------------
        self.script = build_script()
        self.step_idx = 0
        self.location = "door"

        self.demos = DeterministicDemos(
            self,
            self.motion_enabled,
            self.distances,
            self.cmd_vel_topic,
            self.odom_topic,
        )

        self.publish_prompt()

    def publish_prompt(self):
        step = self.script[self.step_idx]
        msg = String()
        msg.data = json.dumps({"step": step["id"], "prompt": step["prompt"]})
        self.pub_prompt.publish(msg)

        # Optional: log prompts too (handy for debugging)
        self.logger.log_event({
            "type": "prompt",
            "step_id": step["id"],
            "prompt": step["prompt"],
            "step_idx": self.step_idx,
            "location": self.location,
        })

    def on_event(self, msg: String):
        ev = json.loads(msg.data)
        step = self.script[self.step_idx]

        if ev.get("type") == "advance":
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.publish_prompt()

        elif ev.get("type") == "arrive" and step.get("expect") == "arrive":
            room = ev.get("room", "")
            self.demos.transit(self.location, room)
            self.location = room

            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.publish_prompt()

        elif ev.get("type") == "demo_confirm":
            room = ev.get("room", "")
            yes = bool(ev.get("yes", False))
            if yes:
                if room == "desk":
                    self.demos.desk_demo("twice")
                elif room == "bed":
                    self.demos.bed_demo("top")
                elif room == "kitchen":
                    self.demos.kitchen_demo("doritos")

        # Log the incoming event
        self.logger.log_event({"type": "event", "event": ev})


def main():
    rclpy.init()
    node = StudyEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()