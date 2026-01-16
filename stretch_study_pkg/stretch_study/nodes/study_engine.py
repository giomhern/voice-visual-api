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
        self.sub_event = self.create_subscription(String, "/study_event", self.on_event, 10)

        # -------------------------
        # Logger
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

        # -------------------------
        # Console visibility
        # -------------------------
        self.get_logger().info(
            "StudyEngine started | "
            f"session_id={self.study_session_id} participant_id={self.study_participant_id} "
            f"motion_enabled={self.motion_enabled} cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"distances={self.distances} log_dir={self.study_log_dir}"
        )

        self.publish_prompt()

    def publish_prompt(self):
        step = self.script[self.step_idx]
        payload = {"step": step["id"], "prompt": step["prompt"]}

        msg = String()
        msg.data = json.dumps(payload)
        self.pub_prompt.publish(msg)

        # Visible console output
        self.get_logger().info(
            f"[PROMPT] step_idx={self.step_idx} id={step['id']} location={self.location} :: {step['prompt']}"
        )

        # File log prompt
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

        # Visible console output for every event
        self.get_logger().info(f"[EVENT] step_idx={self.step_idx} expecting={step.get('expect')} ev={ev}")

        if ev.get("type") == "advance":
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] advance {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif ev.get("type") == "arrive" and step.get("expect") == "arrive":
            room = ev.get("room", "")
            self.get_logger().info(f"[ARRIVE] from={self.location} to={room} (motion_enabled={self.motion_enabled})")

            # Non-blocking transit (movement happens in capability layer)
            try:
                self.demos.transit(self.location, room)
            except Exception as e:
                self.get_logger().error(f"[MOTION_ERROR] transit failed: {e}")

            self.location = room

            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] arrive {prev} -> {self.step_idx} (location={self.location})")
            self.publish_prompt()

        elif ev.get("type") == "demo_confirm":
            room = ev.get("room", "")
            yes = bool(ev.get("yes", False))
            self.get_logger().info(f"[DEMO_CONFIRM] room={room} yes={yes}")

            if yes:
                try:
                    if room == "desk":
                        self.get_logger().info("[DEMO] running desk_demo(thoroughness=twice)")
                        self.demos.desk_demo("twice")
                    elif room == "bed":
                        self.get_logger().info("[DEMO] running bed_demo(arrangement=top)")
                        self.demos.bed_demo("top")
                    elif room == "kitchen":
                        self.get_logger().info("[DEMO] running kitchen_demo(snack=doritos)")
                        self.demos.kitchen_demo("doritos")
                    else:
                        self.get_logger().warn(f"[DEMO] unknown room '{room}'")
                except Exception as e:
                    self.get_logger().error(f"[DEMO_ERROR] demo failed: {e}")

        else:
            # Event not applicable to this step (still logged)
            self.get_logger().info("[INFO] event ignored for current step")

        # File log the incoming event
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