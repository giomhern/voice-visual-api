# study_engine.py
from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from stretch_study.core.script_steps import build_script
from stretch_study.core.logger import StudyLogger
from stretch_study.capabilities.deterministic_demos import DeterministicDemos
from stretch_study.capabilities.funmap_navigator import FunmapNavigator
from stretch_study.capabilities.post_arrival import FunmapTriggers


ALLOWED_GLOBAL_KEYS = {
    "movement_speed",
    "voice_volume",
    "voice_profile",
    "explainability",
    "confirmation",
    "social_distance",
}

ALLOWED_ROOMS = {"desk", "bed", "kitchen"}

ALLOWED_ROOM_KEYS_COMMON = {
    "movement_speed",
    "voice_volume",
    "explainability",
    "social_distance",
}

ALLOWED_ROOM_KEYS_BY_ROOM = {
    "desk": {"cleaning_thoroughness"},
    "bed": {"pillow_arrangement"},
    "kitchen": {"preferred_snack"},
}


class StudyEngine(Node):
    def __init__(self):
        super().__init__("stretch_study_engine")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("motion.enable_transit", False)
        self.declare_parameter("motion.cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("motion.odom_topic", "/odom")
        self.declare_parameter("motion.scaler_node", "/cmd_vel_scaler")

        self.declare_parameter("study.session_id", "debug_session")
        self.declare_parameter("study.participant_id", "debug_participant")
        self.declare_parameter("study.log_dir", "~/stretch_study_logs")

        # FUNMAP navigation
        self.declare_parameter("nav.enable", False)
        self.declare_parameter("nav.goals_yaml", "")
        self.declare_parameter("nav.timeout_s", 120.0)
        self.declare_parameter("nav.advance_on_fail", False)
        self.declare_parameter("nav.goal_topic", "/goal_pose")
        self.declare_parameter("nav.base_frame", "base_footprint")
        self.declare_parameter("nav.arrive_dist_m", 0.30)

        # Speech
        self.declare_parameter("speech.enable", True)
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "auto")

        # -------------------------
        # Read parameters
        # -------------------------
        self.motion_enabled = bool(self.get_parameter("motion.enable_transit").value)
        self.cmd_vel_topic = str(self.get_parameter("motion.cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("motion.odom_topic").value)
        self.scaler_node = str(self.get_parameter("motion.scaler_node").value)

        self.study_session_id = str(self.get_parameter("study.session_id").value)
        self.study_participant_id = str(self.get_parameter("study.participant_id").value)
        self.study_log_dir = str(self.get_parameter("study.log_dir").value)

        self.nav_enabled = bool(self.get_parameter("nav.enable").value)
        self.nav_goals_yaml = str(self.get_parameter("nav.goals_yaml").value)
        self.nav_timeout_s = float(self.get_parameter("nav.timeout_s").value)
        self.nav_advance_on_fail = bool(self.get_parameter("nav.advance_on_fail").value)
        self.nav_goal_topic = str(self.get_parameter("nav.goal_topic").value)
        self.nav_base_frame = str(self.get_parameter("nav.base_frame").value)
        self.nav_arrive_dist_m = float(self.get_parameter("nav.arrive_dist_m").value)

        self.speech_enable = bool(self.get_parameter("speech.enable").value)
        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_rate = int(self.get_parameter("speech.rate").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)

        # -------------------------
        # ROS Pub/Sub
        # -------------------------
        self.pub_prompt = self.create_publisher(String, "/study_prompt", 10)
        self.sub_event = self.create_subscription(String, "/study_event", self.on_event, 10)
        self.pub_speech = self.create_publisher(String, "/speech_request", 10)

        self._scaler_client = self.create_client(
            SetParameters, f"{self.scaler_node}/set_parameters"
        )

        # -------------------------
        # Logger
        # -------------------------
        self.logger = StudyLogger(
            output_dir=self.study_log_dir,
            session_id=self.study_session_id,
            participant_id=self.study_participant_id,
        )

        # -------------------------
        # State
        # -------------------------
        self.global_settings: Dict[str, Any] = {}
        self.room_settings: Dict[str, Dict[str, Any]] = {r: {} for r in ALLOWED_ROOMS}
        self.rules: Dict[str, Any] = {}

        self.script = build_script()
        self.step_idx = 0
        self.location = "door"

        # -------------------------
        # Capabilities
        # -------------------------
        self.demos = DeterministicDemos(
            node=self,
            motion_enabled=self.motion_enabled,
            distances={},
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        self.funmap = FunmapTriggers(self)

        # FUNMAP navigator
        self.navigator: Optional[FunmapNavigator] = None
        if self.nav_enabled and self.nav_goals_yaml:
            self.navigator = FunmapNavigator(
                node=self,
                goals_yaml=self.nav_goals_yaml,
                goal_topic=self.nav_goal_topic,
                base_frame=self.nav_base_frame,
            )
            self.navigator.activate(timeout_s=3.0)

        self.get_logger().info("[STUDY] StudyEngine ready")
        self.publish_prompt()

    # -------------------------
    # Speech
    # -------------------------
    def speak(self, text: str, interrupt: bool = False):
        if not self.speech_enable:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "text": text,
                "volume": self.speech_volume,
                "rate": self.speech_rate,
                "voice": self.speech_voice,
                "interrupt": interrupt,
            },
            ensure_ascii=False,
        )
        self.pub_speech.publish(msg)

    # -------------------------
    # Prompt
    # -------------------------
    def publish_prompt(self):
        if not self.script:
            self.get_logger().error("[PROMPT] script is empty")
            return

        step = self.script[self.step_idx]
        msg = String()
        msg.data = json.dumps(
            {"step": step.get("id", ""), "prompt": step.get("prompt", "")},
            ensure_ascii=False,
        )
        self.pub_prompt.publish(msg)

        self.get_logger().info(
            f"[PROMPT] step_idx={self.step_idx} :: {step.get('prompt','')}"
        )
        self.speak(step.get("prompt", ""), interrupt=True)

    # -------------------------
    # Arrival
    # -------------------------
    def _handle_arrive(self, room: str):
        self.get_logger().info(f"[ARRIVE] {room}")

        success = False
        if self.nav_enabled and self.navigator:
            success = self.navigator.goto(
                room,
                timeout_s=self.nav_timeout_s,
                stand_off_m=0.0,
                arrive_dist_m=self.nav_arrive_dist_m,
            )

        if success or self.nav_advance_on_fail:
            self.location = room

            if success and room == "desk":
                self.get_logger().info("[DESK] post-arrival routine")

                try:
                    self.funmap.head_scan(timeout_s=15.0)
                except Exception as e:
                    self.get_logger().warn(f"[DESK] head_scan failed: {e}")

                try:
                    self.funmap.local_localize(timeout_s=8.0)
                except Exception as e:
                    self.get_logger().warn(f"[DESK] local_localize failed: {e}")

                thoroughness = self.room_settings.get("desk", {}).get(
                    "cleaning_thoroughness", "twice"
                )
                self.demos.desk_demo(str(thoroughness))

            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] {prev} -> {self.step_idx}")

        self.publish_prompt()

    # -------------------------
    # Settings
    # -------------------------
    def _apply_set(self, ev: Dict[str, Any]):
        scope = ev.get("scope")

        if scope == "global":
            key = ev.get("key")
            if key in ALLOWED_GLOBAL_KEYS:
                self.global_settings[key] = ev.get("value")

        elif scope == "room":
            room = ev.get("room")
            key = ev.get("key")
            allowed = set(ALLOWED_ROOM_KEYS_COMMON) | set(
                ALLOWED_ROOM_KEYS_BY_ROOM.get(room, set())
            )
            if room in ALLOWED_ROOMS and key in allowed:
                self.room_settings.setdefault(room, {})[key] = ev.get("value")

    # -------------------------
    # Events
    # -------------------------
    def on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except Exception:
            return

        et = ev.get("type")

        if et == "arrive":
            room = ev.get("room", "").lower()
            if room in ALLOWED_ROOMS:
                self._handle_arrive(room)

        elif et == "advance":
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[ADVANCE] {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif et == "skip":
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.publish_prompt()

        elif et == "demo_confirm":
            if ev.get("yes", False) and ev.get("room") == "desk":
                thoroughness = self.room_settings.get("desk", {}).get(
                    "cleaning_thoroughness", "twice"
                )
                self.demos.desk_demo(str(thoroughness))

        elif et == "set":
            self._apply_set(ev)


def main(args=None):
    rclpy.init(args=args)
    node = StudyEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()