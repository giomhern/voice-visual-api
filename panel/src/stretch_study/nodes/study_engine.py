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
from stretch_study.capabilities.nav2_navigator import Nav2Navigator


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
        # Parameters: motion (legacy / fallback)
        # -------------------------
        self.declare_parameter("motion.enable_transit", False)
        self.declare_parameter("motion.cmd_vel_topic", "/stretch/cmd_vel")
        self.declare_parameter("motion.odom_topic", "/odom")
        self.declare_parameter("motion.distances_m.door_to_desk", 0.0)
        self.declare_parameter("motion.distances_m.desk_to_bed", 0.0)
        self.declare_parameter("motion.distances_m.bed_to_kitchen", 0.0)

        # cmd_vel scaler (Nav2 speed scaling)
        self.declare_parameter("motion.scaler_node", "/cmd_vel_scaler")  # node name running CmdVelScaler

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
        self.declare_parameter("nav.timeout_s", 120.0)
        self.declare_parameter("nav.advance_on_fail", False)

        # -------------------------
        # Parameters: speech
        # -------------------------
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

        self.distances = {
            "door_to_desk": float(self.get_parameter("motion.distances_m.door_to_desk").value),
            "desk_to_bed": float(self.get_parameter("motion.distances_m.desk_to_bed").value),
            "bed_to_kitchen": float(self.get_parameter("motion.distances_m.bed_to_kitchen").value),
        }

        self.nav_enabled = bool(self.get_parameter("nav.enable").value)
        self.nav_goals_yaml = str(self.get_parameter("nav.goals_yaml").value)
        self.nav_timeout_s = float(self.get_parameter("nav.timeout_s").value)
        self.nav_advance_on_fail = bool(self.get_parameter("nav.advance_on_fail").value)

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

        # -------------------------
        # Scaler param client (Nav2 speed)
        # -------------------------
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
            distances=self.distances,
            cmd_vel_topic=self.cmd_vel_topic,
            odom_topic=self.odom_topic,
        )

        # Nav2 navigator (optional)
        self.navigator: Optional[Nav2Navigator] = None
        if self.nav_enabled:
            if not self.nav_goals_yaml:
                self.get_logger().error("[NAV] nav.enable is true but nav.goals_yaml is empty. Disabling nav.")
                self.nav_enabled = False
            else:
                try:
                    self.navigator = Nav2Navigator(node=self, goals_yaml=self.nav_goals_yaml)
                    ok = self.navigator.activate(timeout_s=30.0)
                    self.get_logger().info(f"[NAV] Navigator active={ok} goals={self.nav_goals_yaml}")
                except Exception as e:
                    self.get_logger().error(f"[NAV] Failed to init Nav2Navigator: {e}")
                    self.nav_enabled = False
                    self.navigator = None

        # -------------------------
        # Start banner
        # -------------------------
        self.get_logger().info(
            "StudyEngine started | "
            f"session_id={self.study_session_id} participant_id={self.study_participant_id} "
            f"motion_enabled={self.motion_enabled} cmd_vel={self.cmd_vel_topic} odom={self.odom_topic} "
            f"distances={self.distances} "
            f"nav_enabled={self.nav_enabled} nav_goals_yaml={self.nav_goals_yaml} "
            f"scaler_node={self.scaler_node} "
            f"log_dir={self.study_log_dir} speech_enable={self.speech_enable}"
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
    # Settings helpers
    # -------------------------
    def _snapshot_settings(self):
        self.logger.snapshot(
            "settings",
            {"global": self.global_settings, "rooms": self.room_settings, "rules": self.rules},
        )

    def _validate_room_key(self, room: str, key: str) -> bool:
        if room not in ALLOWED_ROOMS:
            return False
        allowed = set(ALLOWED_ROOM_KEYS_COMMON) | set(ALLOWED_ROOM_KEYS_BY_ROOM.get(room, set()))
        return key in allowed

    def _get_effective_setting(self, key: str, room: Optional[str] = None, default: Any = None) -> Any:
        if room and room in self.room_settings and key in self.room_settings[room]:
            return self.room_settings[room][key]
        if key in self.global_settings:
            return self.global_settings[key]
        return default

    # --- real behavior mappings ---
    def _speed_to_scales(self, speed: str) -> Tuple[float, float]:
        s = (speed or "").lower().strip()
        if s == "slow":
            return 0.4, 0.5
        if s == "fast":
            return 1.0, 1.0
        return 0.7, 0.8  # medium/default

    def _social_distance_to_m(self, sd: Any) -> float:
        s = str(sd or "").lower().strip()
        return {"close": 0.8, "medium": 1.2, "far": 1.7}.get(s, 1.2)

    def _set_cmd_vel_scaler(self, lin: float, ang: float):
        # This requires cmd_vel_scaler node to be running.
        if not self._scaler_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("[SPEED] cmd_vel_scaler set_parameters not available (is cmd_vel_scaler running?)")
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(name="linear_scale", value=float(lin)).to_parameter_msg(),
            Parameter(name="angular_scale", value=float(ang)).to_parameter_msg(),
        ]
        self._scaler_client.call_async(req)
        self.get_logger().info(f"[SPEED] Updated cmd_vel_scaler lin={lin:.2f} ang={ang:.2f}")

    def _apply_real_behavior_side_effects(self, scope: str, key: str, value: Any, room: Optional[str] = None):
        """
        Convert settings into actual behavior changes:
          - voice volume/profile -> speech behavior
          - movement speed -> deterministic + nav2 cmd_vel scaling
          - social distance -> applied when calling navigator.goto (handled in _handle_arrive)
        """
        if scope == "global":
            if key == "voice_volume":
                try:
                    self.speech_volume = int(value)
                except Exception:
                    self.get_logger().warn(f"[SPEECH] voice_volume not an int: {value}")
                return

            if key == "voice_profile":
                prof = str(value).lower().strip()
                # Map your profiles to speech_node voice selector (best-effort)
                # If you use pyttsx3/espeak, voice strings may differ; keep simple:
                if prof == "neutral":
                    self.speech_voice = "auto"
                else:
                    self.speech_voice = prof
                return

            if key == "movement_speed":
                # 1) deterministic motion speeds
                if hasattr(self.demos, "set_speed_mode"):
                    self.demos.set_speed_mode(str(value))
                # 2) Nav2 speeds (via cmd_vel_scaler)
                lin, ang = self._speed_to_scales(str(value))
                self._set_cmd_vel_scaler(lin, ang)
                return

        # Room-scoped movement speed (optional): apply deterministic speed immediately
        # (Nav2 room-specific speed would require more logic; keep global as the main real-time control)
        if scope == "room" and room:
            if key == "movement_speed" and hasattr(self.demos, "set_speed_mode"):
                self.demos.set_speed_mode(str(value))

    def _apply_set_event(self, ev: Dict[str, Any]):
        scope = (ev.get("scope") or "").strip().lower()

        if scope == "global":
            key = (ev.get("key") or "").strip()
            if key not in ALLOWED_GLOBAL_KEYS:
                self.get_logger().warn(f"[SET] unknown global key '{key}' (ignored)")
                return

            value = ev.get("value")
            self.global_settings[key] = value
            self.get_logger().info(f"[SET] global {key}={self.global_settings[key]}")

            # Apply side effects to real robot behavior
            self._apply_real_behavior_side_effects("global", key, value)

            self.logger.log_event({"type": "settings", "scope": "global", "key": key, "value": value})
            self._snapshot_settings()
            return

        if scope == "room":
            room = (ev.get("room") or "").strip().lower()
            key = (ev.get("key") or "").strip()
            if not self._validate_room_key(room, key):
                self.get_logger().warn(f"[SET] invalid room/key room='{room}' key='{key}' (ignored)")
                return

            value = ev.get("value")
            self.room_settings.setdefault(room, {})
            self.room_settings[room][key] = value
            self.get_logger().info(f"[SET] room {room} {key}={value}")

            # Apply side effects (limited)
            self._apply_real_behavior_side_effects("room", key, value, room=room)

            self.logger.log_event({"type": "settings", "scope": "room", "room": room, "key": key, "value": value})
            self._snapshot_settings()
            return

        if scope == "rules":
            val = ev.get("value")
            if not isinstance(val, dict):
                self.get_logger().warn("[SET] rules value must be a JSON object; ignored")
                return
            self.rules = val
            self.get_logger().info(f"[SET] rules updated keys={list(self.rules.keys())}")
            self.logger.log_event({"type": "settings", "scope": "rules", "value": self.rules})
            self._snapshot_settings()
            return

        self.get_logger().warn(f"[SET] unknown scope '{scope}' (ignored)")

    # -------------------------
    # Prompts
    # -------------------------
    def publish_prompt(self):
        step = self.script[self.step_idx]
        payload = {"step": step["id"], "prompt": step["prompt"]}

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_prompt.publish(msg)

        self.get_logger().info(
            f"[PROMPT] step_idx={self.step_idx} id={step['id']} expect={step.get('expect')} "
            f"location={self.location} :: {step['prompt']}"
        )

        self.logger.log_event(
            {"type": "prompt", "step_id": step["id"], "prompt": step["prompt"], "step_idx": self.step_idx, "location": self.location}
        )

        self.speak(step["prompt"], interrupt=True)

    # -------------------------
    # Movement / arrival
    # -------------------------
    def _handle_arrive(self, room: str) -> None:
        self.get_logger().info(f"[ARRIVE] request room={room} from={self.location}")

        success = True

        if self.nav_enabled and self.navigator is not None:
            self.get_logger().info(f"[NAV] Navigating to '{room}' via Nav2...")

            # Social distance stand-off (room override -> global -> default)
            sd = self._get_effective_setting("social_distance", room=room, default="medium")
            stand_off_m = self._social_distance_to_m(sd)

            try:
                success = self.navigator.goto(
                    room,
                    timeout_s=self.nav_timeout_s,
                    stand_off_m=stand_off_m,
                )
            except TypeError:
                # Backward compatibility if goto doesn't accept stand_off_m
                success = self.navigator.goto(room, timeout_s=self.nav_timeout_s)
            except Exception as e:
                success = False
                self.get_logger().error(f"[NAV] goto('{room}') raised: {e}")

            self.get_logger().info(f"[NAV] goal '{room}' success={success}")

        else:
            self.get_logger().info(f"[MOTION] Nav disabled; using deterministic transit {self.location} -> {room}")
            try:
                self.demos.transit(self.location, room)
                success = True
            except Exception as e:
                success = False
                self.get_logger().error(f"[MOTION_ERROR] transit failed: {e}")

        if success or self.nav_advance_on_fail:
            self.location = room
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] arrive {prev} -> {self.step_idx} (location={self.location})")
            self.publish_prompt()
        else:
            self.get_logger().warn(f"[ARRIVE] Failed to reach '{room}'. Staying on step {self.step_idx}.")
            self.publish_prompt()

    # -------------------------
    # Event handler
    # -------------------------
    def on_event(self, msg: String):
        ev = json.loads(msg.data)
        step = self.script[self.step_idx]

        self.get_logger().info(f"[EVENT] step_idx={self.step_idx} step_id={step['id']} ev={ev}")

        et = ev.get("type")

        if et == "set":
            try:
                self._apply_set_event(ev)
            except Exception as e:
                self.get_logger().error(f"[SET] failed to apply set event: {e}")

        elif et == "advance":
            prev = self.step_idx
            self.step_idx = min(self.step_idx + 1, len(self.script) - 1)
            self.get_logger().info(f"[TRANSITION] advance {prev} -> {self.step_idx}")
            self.publish_prompt()

        elif et == "arrive" and step.get("expect") == "arrive":
            room = (ev.get("room") or "").strip().lower()
            self._handle_arrive(room)

        elif et == "demo_confirm":
            room = (ev.get("room") or "").strip().lower()
            yes = bool(ev.get("yes", False))
            self.get_logger().info(f"[DEMO_CONFIRM] room={room} yes={yes}")

            if yes:
                try:
                    if room == "desk":
                        thoroughness = self._get_effective_setting("cleaning_thoroughness", room="desk", default="twice")
                        self.demos.desk_demo(str(thoroughness))
                    elif room == "bed":
                        arrangement = self._get_effective_setting("pillow_arrangement", room="bed", default="top")
                        self.demos.bed_demo(str(arrangement))
                    elif room == "kitchen":
                        snack = self._get_effective_setting("preferred_snack", room="kitchen", default="doritos")
                        self.demos.kitchen_demo(str(snack))
                    else:
                        self.get_logger().warn(f"[DEMO] unknown room '{room}'")
                except Exception as e:
                    self.get_logger().error(f"[DEMO_ERROR] demo failed: {e}")

        else:
            self.get_logger().info("[INFO] event ignored for current step")

        self.logger.log_event(
            {
                "type": "event",
                "event": ev,
                "step_idx": self.step_idx,
                "step_id": step["id"],
                "location": self.location,
                "global_settings": self.global_settings,
                "room_settings": self.room_settings,
            }
        )


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