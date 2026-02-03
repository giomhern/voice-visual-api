# study_engine.py
from __future__ import annotations

import json
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from stretch_study.core.script_steps import build_script
from stretch_study.core.logger import StudyLogger
from stretch_study.capabilities.deterministic_demos import DeterministicDemos
from stretch_study.capabilities.funmap_navigator import FunmapNavigator
from stretch_study.capabilities.post_arrival import FunmapTriggers, TurnInPlace


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

        self.declare_parameter("nav.enable", False)
        self.declare_parameter("nav.goals_yaml", "")
        self.declare_parameter("nav.timeout_s", 120.0)
        self.declare_parameter("nav.advance_on_fail", False)
        self.declare_parameter("nav.goal_topic", "/goal_pose")
        self.declare_parameter("nav.base_frame", "base_footprint")
        self.declare_parameter("nav.arrive_dist_m", 0.30)

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

        self._scaler_client = self.create_client(SetParameters, f"{self.scaler_node}/set_parameters")

        # -------------------------
        # Logger
        # -------------------------
        self.logger = StudyLogger(
            output_dir=self.study_log_dir,
            session_id=self.study_session_id,
            participant_id=self.study_participant_id,
        )

        # -------------------------
        # Settings store
        # -------------------------
        self.global_settings: Dict[str, Any] = {}
        self.room_settings: Dict[str, Dict[str, Any]] = {r: {} for r in ALLOWED_ROOMS}
        self.rules: Dict[str, Any] = {}

        # -------------------------
        # Script + capabilities
        # -------------------------
        self.script = build_script()
        self.step_idx = 0
        self.location = "door"

        self.demos = DeterministicDemos(
            node=self,
            motion_enabled=self.motion_enabled,
            distances={},
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        # FUNMAP helpers
        self.funmap = FunmapTriggers(self)
        self.turner = TurnInPlace(
            node=self,
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        # -------------------------
        # FUNMAP navigator
        # -------------------------
        self.navigator: Optional[FunmapNavigator] = None
        if self.nav_enabled and self.nav_goals_yaml:
            self.navigator = FunmapNavigator(
                node=self,
                goals_yaml=self.nav_goals_yaml,
                goal_topic=self.nav_goal_topic,
                base_frame=self.nav_base_frame,
            )
            self.navigator.activate()

        self.get_logger().info("[STUDY] StudyEngine ready.")
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
            }
        )
        self.pub_speech.publish(msg)

    # -------------------------
    # Prompts
    # -------------------------
    def publish_prompt(self):
        step = self.script[self.step_idx]
        msg = String()
        msg.data = json.dumps({"step": step["id"], "prompt": step["prompt"]})
        self.pub_prompt.publish(msg)
        self.speak(step["prompt"], interrupt=True)

    # -------------------------
    # Arrival handler
    # -------------------------
    def _handle_arrive(self, room: str):
        self.get_logger().info(f"[ARRIVE] Navigating to {room}")

        success = False
        if self.nav_enabled and self.navigator:
            success = self.navigator.goto(
                room,
                timeout_s=self.nav_timeout_s,
                stand_off_m=0.0,
                arrive_dist_m=self.nav_arrive_dist_m,
            )

        if success:
            self.location = room

            # ✅ DESK ROUTINE
            if room == "desk":
                self.get_logger().info("[DESK] Running post-arrival routine")

                self.funmap.head_scan()
                self.funmap.local_localize()
                self.turner.turn_left_90(speed_rad_s=0.5)

                thoroughness = self.room_settings.get("desk", {}).get("cleaning_thoroughness", "twice")
                self.demos.desk_demo(str(thoroughness))

            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)

        self.publish_prompt()

    # -------------------------
    # Event handler
    # -------------------------
    def on_event(self, msg: String):
        ev = json.loads(msg.data)
        et = ev.get("type")

        if et == "arrive":
            room = ev.get("room", "").strip().lower()
            if room in ALLOWED_ROOMS:
                self._handle_arrive(room)

        elif et == "set":
            scope = ev.get("scope")
            key = ev.get("key")
            val = ev.get("value")

            if scope == "room" and room := ev.get("room"):
                self.room_settings.setdefault(room, {})[key] = val
            elif scope == "global":
                self.global_settings[key] = val


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