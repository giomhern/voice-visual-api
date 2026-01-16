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

        self.declare_parameter("motion.enable_transit", False)
        self.declare_parameter("motion.cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("motion.odom_topic", "/odom")
        self.declare_parameter("motion.distances_m.door_to_desk", 0.0)
        self.declare_parameter("motion.distances_m.desk_to_bed", 0.0)
        self.declare_parameter("motion.distances_m.bed_to_kitchen", 0.0)

        self.motion_enabled = self.get_parameter("motion.enable_transit").value
        self.cmd_vel_topic = self.get_parameter("motion.cmd_vel_topic").value
        self.odom_topic = self.get_parameter("motion.odom_topic").value

        self.distances = {
            "door_to_desk": self.get_parameter("motion.distances_m.door_to_desk").value,
            "desk_to_bed": self.get_parameter("motion.distances_m.desk_to_bed").value,
            "bed_to_kitchen": self.get_parameter("motion.distances_m.bed_to_kitchen").value,
        }

        self.pub_prompt = self.create_publisher(String, "/study_prompt", 10)
        self.sub_event = self.create_subscription(String, "/study_event", self.on_event, 10)

        self.logger = StudyLogger()
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

    def on_event(self, msg: String):
        ev = json.loads(msg.data)
        step = self.script[self.step_idx]

        if ev["type"] == "advance":
            self.step_idx += 1
            self.publish_prompt()

        elif ev["type"] == "arrive" and step["expect"] == "arrive":
            self.demos.transit(self.location, ev["room"])
            self.location = ev["room"]
            self.step_idx += 1
            self.publish_prompt()
            self.publish_prompt()

        elif ev["type"] == "demo_confirm" and ev["yes"]:
            if ev["room"] == "desk":
                self.demos.desk_demo("twice")
            elif ev["room"] == "bed":
                self.demos.bed_demo("top")
            elif ev["room"] == "kitchen":
                self.demos.kitchen_demo("doritos")

        self.logger.log(ev)


def main():
    rclpy.init()
    node = StudyEngine()
    rclpy.spin(node)
    rclpy.shutdown()