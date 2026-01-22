import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stretch_study.core.script_steps import build_script
from stretch_study.core.logger import StudyLogger
from stretch_study.capabilities.deterministic_demos import DeterministicDemos
from stretch_study.capabilities.nav2_navigator import Nav2Navigator


class StudyEngine(Node):
    def __init__(self):
        super().__init__("stretch_study_engine")

        # -------------------------
        # Parameters: motion (legacy / fallback)
        # -------------------------
        self.declare_parameter("motion.enable_transit", False)
        self.declare_parameter("motion.cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("motion.odom_topic", "/odom")
        self.declare_parameter("motion.distances_m.door_to_desk", 0.0)
        self.declare_parameter("motion.distances_m.desk_to_bed", 0.0)
        self.declare_parameter("motion.distances_m.bed_to_kitchen", 0.0)

        # -------------------------
        # Parameters: study meta
        # -------------------------
        self.declare_parameter("study.session_id", "debug_session")
        self.declare_parameter("study.participant_id", "debug_participant")
        self.declare_parameter("study.log_dir", "~/stretch_study_logs")

        # -------------------------
        # Parameters: navigation (Nav2)
        # -------------------------
        self.declare_parameter("nav.enable", False)
        self.declare_parameter("nav.goals_yaml", "")
        self.declare_parameter("nav.action_name", "/navigate_to_pose")
        self.declare_parameter("nav.timeout_s", 120.0)
        self.declare_parameter("nav.advance_on_fail", False)

        self.pub_speech = self.create_publisher(String, "/speech_request", 10)
        self.declare_parameter("speech.enable", True)
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "auto")
        self.speech_enable = bool(self.get_parameter("speech.enable").value)
        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_rate = int(self.get_parameter("speech.rate").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)

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

        self.nav_enabled = bool(self.get_parameter("nav.enable").value)
        self.nav_goals_yaml = str(self.get_parameter("nav.goals_yaml").value)
        self.nav_action_name = str(self.get_parameter("nav.action_name").value)
        self.nav_timeout_s = float(self.get_parameter("nav.timeout_s").value)
        self.nav_advance_on_fail = bool(self.get_parameter("nav.advance_on_fail").value)

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

        # Always keep deterministic demos available (also used for transit fallback)
        self.demos = DeterministicDemos(
            node=self,
            motion_enabled=self.motion_enabled,
            distances=self.distances,
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        # Nav2 navigator (optional)
        self.navigator = None
        if self.nav_enabled:
            if not self.nav_goals_yaml:
                self.get_logger().error("[NAV] nav.enable is true but nav.goals_yaml is empty. Disabling nav.")
                self.nav_enabled = False
            else:
                try:
                    self.navigator = Nav2Navigator(
                        node=self,
                        action_name=self.nav_action_name,
                        goals_yaml=self.nav_goals_yaml,
                    )
                    ok = self.navigator.wait_ready(timeout_s=10.0)
                    self.get_logger().info(f"[NAV] Navigator ready={ok} action={self.nav_action_name} goals={self.nav_goals_yaml}")
                except Exception as e:
                    self.get_logger().error(f"[NAV] Failed to init Nav2Navigator: {e}")
                    self.nav_enabled = False
                    self.navigator = None

        # -------------------------
        # Console start banner
        # -------------------------
        self.get_logger().info(
            "StudyEngine started | "
            f"session_id={self.study_session_id} participant_id={self.study_participant_id} "
            f"motion_enabled={self.motion_enabled} cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"distances={self.distances} "
            f"nav_enabled={self.nav_enabled} nav_action={self.nav_action_name} nav_goals_yaml={self.nav_goals_yaml} "
            f"log_dir={self.study_log_dir}"
        )

        self.publish_prompt()

    def publish_prompt(self):
        step = self.script[self.step_idx]
        payload = {"step": step["id"], "prompt": step["prompt"]}

        msg = String()
        msg.data = json.dumps(payload)
        self.pub_prompt.publish(msg)

        # Console visibility
        self.get_logger().info(
            f"[PROMPT] step_idx={self.step_idx} id={step['id']} expect={step.get('expect')} "
            f"location={self.location} :: {step['prompt']}"
        )

        # File log
        self.logger.log_event({
            "type": "prompt",
            "step_id": step["id"],
            "prompt": step["prompt"],
            "step_idx": self.step_idx,
            "location": self.location,
        })

        self.speak(step["prompt"])

    def speak(self, text: str, interrupt: bool = False):
        if not getattr(self, "speech_enable", False):
            return
        msg = String()
        msg.data = json.dumps({
            "text": text,
            "volume": self.speech_volume,
            "rate": self.speech_rate,
            "voice": self.speech_voice,
            "interrupt": interrupt,
        })
        self.pub_speech.publish(msg)
        self.logger.log_event({"type": "speech_request", "text": text})

    def _handle_arrive(self, room: str) -> None:
        """
        Handle an arrive(room) event at a step that expects arrival.
        Uses Nav2 if enabled, otherwise falls back to deterministic transit.
        Advances the script only on success unless nav.advance_on_fail is true.
        """
        self.get_logger().info(f"[ARRIVE] request room={room} from={self.location}")

        success = True

        # Prefer Nav2 when enabled
        if self.nav_enabled and self.navigator is not None:
            self.get_logger().info(f"[NAV] Navigating to '{room}' via Nav2...")
            try:
                # If your Nav2Navigator.goto does not accept timeout_s, remove that arg.
                success = self.navigator.goto(room, timeout_s=self.nav_timeout_s)
            except TypeError:
                # Backward-compatible: goto(name) without timeout
                success = self.navigator.goto(room)
            except Exception as e:
                success = False
                self.get_logger().error(f"[NAV] goto('{room}') raised: {e}")

            self.get_logger().info(f"[NAV] goal '{room}' success={success}")

        else:
            # Fallback: deterministic transit (your previous corner-walk)
            self.get_logger().info(f"[MOTION] Nav disabled; using deterministic transit {self.location} -> {room}")
            try:
                self.demos.transit(self.location, room)
                # Transit is async; for the study flow we treat it as initiated successfully.
                success = True
            except Exception as e:
                success = False
                self.get_logger().error(f"[MOTION_ERROR] transit failed: {e}")

        # Decide whether to advance
        if success or self.nav_advance_on_fail:
            self.location = room
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] arrive {prev} -> {self.step_idx} (location={self.location})")
            self.publish_prompt()
        else:
            self.get_logger().warn(f"[ARRIVE] Failed to reach '{room}'. Staying on step {self.step_idx}.")
            self.publish_prompt()

    def on_event(self, msg: String):
        ev = json.loads(msg.data)
        step = self.script[self.step_idx]

        # Console visibility for every event
        self.get_logger().info(f"[EVENT] step_idx={self.step_idx} step_id={step['id']} ev={ev}")

        et = ev.get("type")

        if et == "advance":
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] advance {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif et == "arrive" and step.get("expect") == "arrive":
            room = ev.get("room", "")
            self._handle_arrive(room)

        elif et == "demo_confirm":
            room = ev.get("room", "")
            yes = bool(ev.get("yes", False))
            self.get_logger().info(f"[DEMO_CONFIRM] room={room} yes={yes}")

            if yes:
                try:
                    if room == "desk":
                        self.demos.desk_demo("twice")
                    elif room == "bed":
                        self.demos.bed_demo("top")
                    elif room == "kitchen":
                        self.demos.kitchen_demo("doritos")
                    else:
                        self.get_logger().warn(f"[DEMO] unknown room '{room}'")
                except Exception as e:
                    self.get_logger().error(f"[DEMO_ERROR] demo failed: {e}")

        else:
            self.get_logger().info("[INFO] event ignored for current step")

        # File log the incoming event
        self.logger.log_event({
            "type": "event",
            "event": ev,
            "step_idx": self.step_idx,
            "step_id": step["id"],
            "location": self.location,
        })


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