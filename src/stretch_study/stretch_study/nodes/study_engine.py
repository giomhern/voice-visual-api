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

        # Speed scaler service client (optional)
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
        # Script + state
        # -------------------------
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
        self.turner = TurnInPlace(
            node=self,
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        # FUNMAP navigator
        self.navigator: Optional[FunmapNavigator] = None
        if self.nav_enabled:
            if not self.nav_goals_yaml:
                self.get_logger().error("[NAV] nav.enable is true but nav.goals_yaml is empty. Disabling nav.")
                self.nav_enabled = False
            else:
                try:
                    self.navigator = FunmapNavigator(
                        node=self,
                        goals_yaml=self.nav_goals_yaml,
                        goal_topic=self.nav_goal_topic,
                        base_frame=self.nav_base_frame,
                    )
                    ok = self.navigator.activate(timeout_s=3.0)
                    self.get_logger().info(
                        f"[NAV] FunmapNavigator ready={ok} goals_yaml={self.nav_goals_yaml} "
                        f"goal_topic={self.nav_goal_topic} base_frame={self.nav_base_frame}"
                    )
                except Exception as e:
                    self.get_logger().error(f"[NAV] Failed to init FunmapNavigator: {e}")
                    self.nav_enabled = False
                    self.navigator = None

        self.get_logger().info(
            "[STUDY] started | "
            f"session_id={self.study_session_id} participant_id={self.study_participant_id} "
            f"nav_enabled={self.nav_enabled} goals_yaml={self.nav_goals_yaml} "
            f"cmd_vel={self.cmd_vel_topic} odom={self.odom_topic}"
        )

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
                "volume": int(self.speech_volume),
                "rate": int(self.speech_rate),
                "voice": str(self.speech_voice),
                "interrupt": bool(interrupt),
            },
            ensure_ascii=False,
        )
        self.pub_speech.publish(msg)
        self.logger.log_event({"type": "speech_request", "text": text})

    # -------------------------
    # Prompt publishing
    # -------------------------
    def publish_prompt(self):
        if not self.script:
            self.get_logger().error("[PROMPT] script is empty! build_script() returned nothing.")
            return

        step = self.script[self.step_idx]
        payload = {"step": step.get("id", ""), "prompt": step.get("prompt", "")}

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_prompt.publish(msg)

        self.get_logger().info(f"[PROMPT] step_idx={self.step_idx} id={payload['step']} :: {payload['prompt']}")
        self.speak(payload["prompt"], interrupt=True)

        self.logger.log_event(
            {
                "type": "prompt",
                "step_idx": self.step_idx,
                "step_id": payload["step"],
                "prompt": payload["prompt"],
                "location": self.location,
            }
        )

    # -------------------------
    # Arrival handler
    # -------------------------
    def _handle_arrive(self, room: str):
        self.get_logger().info(f"[ARRIVE] request room={room} from={self.location}")

        success = False

        if self.nav_enabled and self.navigator is not None:
            try:
                success = self.navigator.goto(
                    room,
                    timeout_s=self.nav_timeout_s,
                    stand_off_m=0.0,  # goals are stop poses
                    arrive_dist_m=self.nav_arrive_dist_m,
                )
            except Exception as e:
                success = False
                self.get_logger().error(f"[NAV] goto('{room}') raised: {e}")

        else:
            # fallback deterministic motion if enabled
            try:
                self.demos.transit(self.location, room)
                success = True
            except Exception as e:
                success = False
                self.get_logger().error(f"[MOTION] transit failed: {e}")

        self.get_logger().info(f"[ARRIVE] room={room} success={success}")

        if success or self.nav_advance_on_fail:
            self.location = room

            # Desk post-arrival routine
            if success and room == "desk":
                self.get_logger().info("[DESK] post-arrival routine: head_scan -> local_localize -> turn_left_90 -> desk_demo")

                # Best-effort (won't crash if services missing)
                try:
                    self.funmap.head_scan(timeout_s=15.0)
                except Exception as e:
                    self.get_logger().warn(f"[DESK] head_scan failed: {e}")

                try:
                    self.funmap.local_localize(timeout_s=8.0)
                except Exception as e:
                    self.get_logger().warn(f"[DESK] local_localize failed: {e}")

                try:
                    self.turner.turn_left_90(speed_rad_s=0.5)
                except Exception as e:
                    self.get_logger().warn(f"[DESK] turn_left_90 failed: {e}")

                thoroughness = self.room_settings.get("desk", {}).get("cleaning_thoroughness", "twice")
                try:
                    self.demos.desk_demo(str(thoroughness))
                except Exception as e:
                    self.get_logger().error(f"[DESK] desk_demo failed: {e}")

            # advance step
            if self.script:
                prev = self.step_idx
                self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
                self.get_logger().info(f"[TRANSITION] arrive {prev} -> {self.step_idx}")

        self.publish_prompt()

    # -------------------------
    # Settings application
    # -------------------------
    def _apply_set(self, ev: Dict[str, Any]):
        scope = (ev.get("scope") or "").strip().lower()

        if scope == "global":
            key = (ev.get("key") or "").strip()
            val = ev.get("value")

            if key and key in ALLOWED_GLOBAL_KEYS:
                self.global_settings[key] = val
                self.get_logger().info(f"[SET] global {key}={val}")
            else:
                self.get_logger().warn(f"[SET] invalid global key '{key}'")

        elif scope == "room":
            room = (ev.get("room") or "").strip().lower()
            key = (ev.get("key") or "").strip()
            val = ev.get("value")

            allowed = set(ALLOWED_ROOM_KEYS_COMMON) | set(ALLOWED_ROOM_KEYS_BY_ROOM.get(room, set()))
            if room in ALLOWED_ROOMS and key in allowed:
                self.room_settings.setdefault(room, {})
                self.room_settings[room][key] = val
                self.get_logger().info(f"[SET] room {room} {key}={val}")
            else:
                self.get_logger().warn(f"[SET] invalid room/key room='{room}' key='{key}'")

        elif scope == "rules":
            val = ev.get("value")
            if isinstance(val, dict):
                self.rules = val
                self.get_logger().info(f"[SET] rules updated keys={list(self.rules.keys())}")
            else:
                self.get_logger().warn("[SET] rules value must be a JSON object")

        else:
            self.get_logger().warn(f"[SET] unknown scope '{scope}'")

        self.logger.log_event({"type": "settings", "event": ev, "global": self.global_settings, "rooms": self.room_settings})

    # -------------------------
    # Event handler (keyboard teleop compatible)
    # -------------------------
    def on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[EVENT] invalid JSON: {e} raw={msg.data}")
            return

        et = ev.get("type")
        self.get_logger().info(f"[EVENT] et={et} ev={ev}")

        if et == "arrive":
            room = (ev.get("room") or "").strip().lower()
            if room in ALLOWED_ROOMS:
                self._handle_arrive(room)
            else:
                self.get_logger().warn(f"[EVENT] invalid room '{room}'")

        elif et == "advance":
            if not self.script:
                self.get_logger().error("[ADVANCE] script is empty.")
                return
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] advance {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif et == "skip":
            if not self.script:
                self.get_logger().error("[SKIP] script is empty.")
                return
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] skip {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif et == "demo_confirm":
            room = (ev.get("room") or "").strip().lower()
            yes = bool(ev.get("yes", False))
            self.get_logger().info(f"[DEMO_CONFIRM] room={room} yes={yes}")

            if yes:
                try:
                    if room == "desk":
                        thoroughness = self.room_settings.get("desk", {}).get("cleaning_thoroughness", "twice")
                        self.demos.desk_demo(str(thoroughness))
                    elif room == "bed":
                        arrangement = self.room_settings.get("bed", {}).get("pillow_arrangement", "top")
                        self.demos.bed_demo(str(arrangement))
                    elif room == "kitchen":
                        snack = self.room_settings.get("kitchen", {}).get("preferred_snack", "doritos")
                        self.demos.kitchen_demo(str(snack))
                    else:
                        self.get_logger().warn(f"[DEMO_CONFIRM] unknown room '{room}'")
                except Exception as e:
                    self.get_logger().error(f"[DEMO_CONFIRM] demo failed: {e}")
            else:
                self.get_logger().info("[DEMO_CONFIRM] demo declined")

        elif et == "set":
            self._apply_set(ev)

        else:
            self.get_logger().info("[EVENT] ignored")

        self.logger.log_event({"type": "event", "event": ev, "step_idx": self.step_idx, "location": self.location})


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