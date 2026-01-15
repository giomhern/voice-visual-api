#!/usr/bin/env python3
"""
stretch_study_controller.py  (ROS 2, Python 3.8+ compatible)

One-file “central control panel + study orchestrator” for your Stretch study:
- Finite-state machine that walks through:
  Intro -> Global settings wizard -> Walk/Follow -> Desk config -> Desk demo prompt/run
  -> Bed config -> Bed demo prompt/run -> Kitchen config -> Kitchen demo prompt/run
  -> Situational rules -> Round 2 switch -> repeat -> End
- Profiles: "user" (Round 1) and "jackie" (Round 2)
- Settings storage + timestamped archiving to JSON
- Terminal hotkeys for RA override (advance, follow toggle, station set, demo confirm/run, estop)
- ROS topic input for Voice/Visual/Hybrid UI: /study_event (std_msgs/String JSON)
- Base motion via geometry_msgs/Twist published to cmd_vel topic (default: stretch/cmd_vel)
- “Follow me” placeholder that creeps forward so the flow works immediately

Python compatibility:
- Avoids Python 3.10 union types (X | None) and uses typing.Optional/Dict/List/Callable

How to run (on robot):
  source /opt/ros/humble/setup.bash
  python3 stretch_study_controller.py --ros-args -p cmd_vel_topic:=stretch/cmd_vel

UI event format (publish JSON to /study_event):
  {"type":"set","scope":"global","key":"movement_speed","value":"slow"}
  {"type":"set","scope":"room","room":"desk","key":"cleaning_thoroughness","value":"twice"}
  {"type":"arrive","room":"desk"}
  {"type":"demo_confirm","room":"desk","yes":true}
  {"type":"set","scope":"rules","value":{"no_go":[...],"time_rules":[...]}}
  {"type":"profile","value":"jackie"}
  {"type":"advance"}

Terminal hotkeys:
  ENTER : advance script (when waiting)
  f     : toggle FOLLOW mode (walk together placeholder)
  1/2/3 : mark arrival at desk/bed/kitchen
  d     : confirm demo YES for current room (or run demo immediately for debug)
  s     : stop base (IDLE)
  e     : toggle ESTOP latch
  p     : print current effective settings for active room
  q     : quit safely (stops base)
"""

import json
import os
import sys
import time
import threading
from dataclasses import dataclass, field
from enum import Enum, auto
from datetime import datetime
from typing import Optional, Callable, Any, Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ----------------------------
# Data model
# ----------------------------

class Explainability(str, Enum):
    NONE = "none"
    SHORT = "short"
    FULL = "full"

class Speed(str, Enum):
    SLOW = "slow"
    MEDIUM = "medium"
    FAST = "fast"

class SocialDistance(str, Enum):
    CLOSE = "close"
    MEDIUM = "medium"
    FAR = "far"

class VoiceProfile(str, Enum):
    NEUTRAL = "neutral"
    FRIENDLY = "friendly"
    PLAYFUL = "playful"

@dataclass
class ProfileData:
    global_defaults: Dict[str, Any] = field(default_factory=lambda: {
        "movement_speed": Speed.MEDIUM.value,
        "voice_volume": 60,
        "voice_profile": VoiceProfile.NEUTRAL.value,
        "explainability": Explainability.SHORT.value,
        "confirmation": True,
        "social_distance": SocialDistance.MEDIUM.value,
    })
    rooms: Dict[str, Dict[str, Any]] = field(default_factory=lambda: {
        "desk": {},
        "bed": {},
        "kitchen": {},
    })
    rules: Dict[str, Any] = field(default_factory=lambda: {
        "no_go": [],
        "time_rules": [],
    })

@dataclass
class StudyState:
    active_profile: str = "user"   # "user" or "jackie"
    current_room: str = "desk"     # "desk" | "bed" | "kitchen"
    demo_confirm: Dict[str, Optional[bool]] = field(default_factory=lambda: {"desk": None, "bed": None, "kitchen": None})
    waiting_for: str = ""          # debug string


class Phase(Enum):
    INTRO = auto()
    GLOBAL_WIZARD = auto()
    WALK_AND_STATIONS = auto()
    STATION_CONFIG = auto()
    STATION_DEMO_PROMPT = auto()
    STATION_DEMO_RUN = auto()
    RULES = auto()
    ROUND_SWITCH = auto()
    END = auto()


# ----------------------------
# Step system for wizard/config
# ----------------------------

@dataclass
class Step:
    step_id: str
    prompt: str
    scope: str  # "global" or "room" or "rules" or "meta"
    key: Optional[str] = None
    room: Optional[str] = None
    valid: Optional[Callable[[Any], bool]] = None


# ----------------------------
# Main controller node
# ----------------------------

class StretchStudyController(Node):
    def __init__(self):
        super().__init__("stretch_study_controller")

        # Parameters
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "stretch/cmd_vel").value
        self.say_topic = self.declare_parameter("say_topic", "stretch_say").value
        self.event_topic = self.declare_parameter("event_topic", "study_event").value
        self.data_dir = self.declare_parameter("data_dir", os.path.join(os.getcwd(), "study_data")).value

        os.makedirs(self.data_dir, exist_ok=True)
        self.state_path = os.path.join(self.data_dir, "study_state.json")
        self.archive_dir = os.path.join(self.data_dir, "archives")
        os.makedirs(self.archive_dir, exist_ok=True)

        # ROS pubs/subs
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.say_pub = self.create_publisher(String, self.say_topic, 10)
        self.event_sub = self.create_subscription(String, self.event_topic, self.on_event_msg, 10)

        # Study data
        self.profiles: Dict[str, ProfileData] = {
            "user": ProfileData(),
            "jackie": ProfileData(),
        }
        self.study = StudyState()

        # FSM control
        self.phase = Phase.INTRO
        self.step_index = 0
        self.pending_steps: List[Step] = []
        self._lock = threading.Lock()

        # Base motion control
        self.mode_follow = False
        self.estop = False
        self.last_cmd_time = time.time()

        # Follow placeholder params
        self.follow_v = 0.05 # m/s creep
        self.follow_max_v_by_speed = {"slow": 0.15, "medium": 0.25, "fast": 0.35}

        # Control timers
        self.create_timer(0.05, self.control_tick)   # 20 Hz
        self.create_timer(0.25, self.autosave_tick)  # periodic save

        # Build initial INTRO steps
        self.enter_intro()

        self.log_info("Publishing cmd_vel to: %s" % self.cmd_vel_topic)
        self.log_info("Listening for UI events on: %s" % self.event_topic)
        self.log_info("Publishing speech lines on: %s" % self.say_topic)
        self.say("System ready. Press ENTER to begin, or send {\"type\":\"advance\"} on /%s." % self.event_topic)

    # -------- Logging / speech --------

    def log_info(self, msg: str):
        self.get_logger().info(msg)

    def log_warn(self, msg: str):
        self.get_logger().warn(msg)

    def say(self, text: str):
        self.log_info("[SAY] %s" % text)
        self.say_pub.publish(String(data=text))

    # -------- Persistence --------

    def autosave_tick(self):
        self.save_state()

    def save_state(self):
        with self._lock:
            blob = {
                "timestamp": datetime.now().isoformat(),
                "phase": self.phase.name,
                "step_index": self.step_index,
                "study": {
                    "active_profile": self.study.active_profile,
                    "current_room": self.study.current_room,
                    "demo_confirm": self.study.demo_confirm,
                    "waiting_for": self.study.waiting_for,
                },
                "profiles": {
                    name: {
                        "global_defaults": pd.global_defaults,
                        "rooms": pd.rooms,
                        "rules": pd.rules,
                    }
                    for name, pd in self.profiles.items()
                },
            }
        try:
            with open(self.state_path, "w") as f:
                json.dump(blob, f, indent=2)
        except Exception as e:
            self.log_warn("Failed to save state: %s" % e)

    def archive_snapshot(self, label: str):
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        path = os.path.join(self.archive_dir, "%s_%s.json" % (ts, label))
        try:
            try:
                with open(self.state_path, "r") as f:
                    current = json.load(f)
            except Exception:
                current = {}
            with open(path, "w") as f:
                json.dump(current, f, indent=2)
            self.log_info("Archived snapshot: %s" % path)
        except Exception as e:
            self.log_warn("Failed to archive snapshot: %s" % e)

    # -------- Effective settings --------

    def profile(self) -> ProfileData:
        return self.profiles[self.study.active_profile]

    def effective_settings(self, room: Optional[str] = None) -> Dict[str, Any]:
        pd = self.profile()
        eff = dict(pd.global_defaults)
        if room:
            eff.update(pd.rooms.get(room, {}))
        return eff

    # -------- Base motion control --------

    def publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.last_cmd_time = time.time()

    def set_estop(self, enabled: bool):
        self.estop = enabled
        self.mode_follow = False
        self.publish_stop()
        self.say("Emergency stop enabled." if enabled else "Emergency stop released.")

    def control_tick(self):
        if self.estop:
            self.publish_stop()
            return

        if self.mode_follow and self.phase in (Phase.WALK_AND_STATIONS, Phase.STATION_CONFIG):
            eff = self.effective_settings(self.study.current_room)
            max_v = self.follow_max_v_by_speed.get(str(eff.get("movement_speed", "medium")), 0.25)
            v = min(self.follow_v, max_v)

            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.last_cmd_time = time.time()
            return

        self.publish_stop()

    # -------- Event handling (UI) --------

    def on_event_msg(self, msg: String):
        try:
            evt = json.loads(msg.data)
        except Exception as e:
            self.log_warn("Bad JSON on /%s: %s" % (self.event_topic, e))
            return
        self.handle_event(evt)

    def handle_event(self, evt: Dict[str, Any]):
        etype = evt.get("type", "")

        if etype == "advance":
            self.advance()
            return

        if etype == "profile":
            val = evt.get("value")
            if val in self.profiles:
                self.study.active_profile = val
                self.say("Profile set to %s." % val)
                self.archive_snapshot("profile_%s" % val)
            else:
                self.say("Unknown profile.")
            return

        if etype == "arrive":
            room = evt.get("room")
            if room in ("desk", "bed", "kitchen"):
                self.on_arrive(room)
            return

        if etype == "demo_confirm":
            room = evt.get("room", self.study.current_room)
            yes = bool(evt.get("yes", False))
            if room in ("desk", "bed", "kitchen"):
                self.study.demo_confirm[room] = yes
                if yes:
                    self.say("Okay. I will perform the %s demo." % room)
                else:
                    self.say("Okay. I will skip the %s demo." % room)
                if self.phase == Phase.STATION_DEMO_PROMPT and room == self.study.current_room:
                    self.advance()
            return

        if etype == "set":
            scope = evt.get("scope")

            if scope == "global":
                key = evt.get("key")
                value = evt.get("value")
                if key:
                    self.apply_setting_global(str(key), value)
                    self.maybe_confirm()
                    self.advance_if_waiting_for(str(key))
                return

            if scope == "room":
                room = evt.get("room", self.study.current_room)
                key = evt.get("key")
                value = evt.get("value")
                if room in ("desk", "bed", "kitchen") and key:
                    self.apply_setting_room(room, str(key), value)
                    self.maybe_confirm()
                    self.advance_if_waiting_for("%s.%s" % (room, str(key)))
                return

            if scope == "rules":
                val = evt.get("value", {})
                if isinstance(val, dict):
                    self.profile().rules = val
                    self.say("Situational rules saved.")
                    self.archive_snapshot("rules_%s" % self.study.active_profile)
                    self.advance_if_waiting_for("rules")
                return

        self.log_warn("Unhandled event: %s" % evt)

    def maybe_confirm(self):
        eff = self.effective_settings(None)
        if bool(eff.get("confirmation", True)):
            self.say("Confirmed.")

    def advance_if_waiting_for(self, token: str):
        if self.study.waiting_for == token:
            self.advance()

    # -------- Apply settings --------

    def apply_setting_global(self, key: str, value: Any):
        pd = self.profile()
        # Normalize some common fields
        if key == "voice_volume":
            try:
                value = int(value)
            except Exception:
                pass
        if key == "confirmation" and not isinstance(value, bool):
            if str(value).lower() in ("yes", "true", "1"):
                value = True
            elif str(value).lower() in ("no", "false", "0"):
                value = False

        pd.global_defaults[key] = value
        self.log_info("Set GLOBAL %s=%s (profile=%s)" % (key, value, self.study.active_profile))
        self.archive_snapshot("global_%s" % self.study.active_profile)

    def apply_setting_room(self, room: str, key: str, value: Any):
        pd = self.profile()
        pd.rooms.setdefault(room, {})
        pd.rooms[room][key] = value
        self.log_info("Set ROOM[%s] %s=%s (profile=%s)" % (room, key, value, self.study.active_profile))
        self.archive_snapshot("room_%s_%s" % (room, self.study.active_profile))

    # ----------------------------
    # FSM: entering phases
    # ----------------------------

    def enter_intro(self):
        self.phase = Phase.INTRO
        self.step_index = 0
        self.pending_steps = [
            Step(
                step_id="intro_ra",
                prompt=("Welcome and thank you for participating. Today, you’ll be interacting with Stretch. "
                        "You will configure my behavior for the desk, bed, and kitchen. "
                        "You will guide me to each area and tell me where we are."),
                scope="meta",
            ),
            Step(
                step_id="intro_stretch",
                prompt=("Hello. I’m Stretch. Thanks for unboxing me. Before we walk around your home, "
                        "we need to set my general behavior settings. These will be my defaults unless overridden per room."),
                scope="meta",
            ),
        ]
        self.study.waiting_for = ""
        self.present_current_step()

    def enter_global_wizard(self):
        self.phase = Phase.GLOBAL_WIZARD
        self.step_index = 0
        self.pending_steps = [
            Step(
                step_id="global_movement_speed",
                prompt="First: Movement Speed. I can move slowly, medium, or fast. Please set my default movement speed.",
                scope="global",
                key="movement_speed",
                valid=lambda v: v in ("slow", "medium", "fast"),
            ),
            Step(
                step_id="global_voice_volume",
                prompt="Next: Voice Volume (0 to 100). Please set my default speaking volume.",
                scope="global",
                key="voice_volume",
                valid=lambda v: isinstance(v, (int, float)) and 0 <= int(v) <= 100,
            ),
            Step(
                step_id="global_voice_profile",
                prompt="Next: Voice Profile. Please choose: neutral, friendly, or playful.",
                scope="global",
                key="voice_profile",
                valid=lambda v: v in ("neutral", "friendly", "playful"),
            ),
            Step(
                step_id="global_explainability",
                prompt="Next: Explainability. Please choose: none, short, or full.",
                scope="global",
                key="explainability",
                valid=lambda v: v in ("none", "short", "full"),
            ),
            Step(
                step_id="global_confirmation",
                prompt="Next: Confirmation style. Should I verbally confirm every change? Please choose: yes or no.",
                scope="global",
                key="confirmation",
                valid=lambda v: isinstance(v, bool) or str(v).lower() in ("yes", "no", "true", "false", "1", "0"),
            ),
            Step(
                step_id="global_social_distance",
                prompt="Finally: Social distance. Please choose: close, medium, or far.",
                scope="global",
                key="social_distance",
                valid=lambda v: v in ("close", "medium", "far"),
            ),
        ]
        self.present_current_step()

    def enter_walk(self):
        self.phase = Phase.WALK_AND_STATIONS
        self.step_index = 0
        self.pending_steps = []
        self.study.waiting_for = ""
        self.say("Great. My default behavior has been set.")
        self.say("Please walk with me and guide me. When we reach a station, tell me where we are: desk, bed, or kitchen.")
        self.say("Press 'f' to toggle follow mode. Press 1/2/3 to mark arrival at desk/bed/kitchen.")

    def enter_station_config(self, room: str):
        self.phase = Phase.STATION_CONFIG
        self.step_index = 0
        self.study.current_room = room
        self.study.demo_confirm[room] = None

        demo_key = {"desk": "cleaning_thoroughness", "bed": "pillow_arrangement", "kitchen": "snack_preference"}[room]
        demo_prompt = {
            "desk": "Desk demo setting: Cleaning thoroughness. Choose: once, twice, thorough, or none.",
            "bed": "Bed demo setting: Pillow arrangement. Choose: center or top.",
            "kitchen": "Kitchen demo setting: Preferred snack. Choose: doritos or cheetos.",
        }[room]

        self.pending_steps = [
            Step(
                step_id="%s_movement_speed" % room,
                prompt="In the %s area, please set movement speed: slow, medium, or fast." % room,
                scope="room", room=room, key="movement_speed",
                valid=lambda v: v in ("slow", "medium", "fast"),
            ),
            Step(
                step_id="%s_voice_volume" % room,
                prompt="In the %s area, please set voice volume (0 to 100)." % room,
                scope="room", room=room, key="voice_volume",
                valid=lambda v: isinstance(v, (int, float)) and 0 <= int(v) <= 100,
            ),
            Step(
                step_id="%s_explainability" % room,
                prompt="In the %s area, please set explainability: none, short, or full." % room,
                scope="room", room=room, key="explainability",
                valid=lambda v: v in ("none", "short", "full"),
            ),
            Step(
                step_id="%s_social_distance" % room,
                prompt="In the %s area, please set social distance: close, medium, or far." % room,
                scope="room", room=room, key="social_distance",
                valid=lambda v: v in ("close", "medium", "far"),
            ),
            Step(
                step_id="%s_%s" % (room, demo_key),
                prompt=demo_prompt,
                scope="room", room=room, key=demo_key,
                valid=lambda v: True,
            ),
        ]

        self.say("Affirmative — I have registered this area as the %s." % room)
        self.say("In this room, you can set preferences that override my general defaults.")
        self.present_current_step()

    def enter_demo_prompt(self, room: str):
        self.phase = Phase.STATION_DEMO_PROMPT
        self.step_index = 0
        self.pending_steps = []
        self.study.waiting_for = ""
        self.say("I have received your %s customizations. Would you like me to perform the %s demo?" % (room, room))
        self.say("Send {\"type\":\"demo_confirm\",\"room\":\"%s\",\"yes\":true/false} or press 'd' for yes." % room)

    def enter_demo_run(self, room: str):
        self.phase = Phase.STATION_DEMO_RUN
        self.step_index = 0
        self.pending_steps = []
        self.say("Performing %s demo now." % room)

        self.mode_follow = False
        self.publish_stop()

        # Placeholder safe demo
        if room == "desk":
            passes = self.effective_settings("desk").get("cleaning_thoroughness", "once")
            self.say("Here is how I will wipe your desk: %s. (Demo placeholder)" % passes)
            self.base_gesture_wiggle()
        elif room == "bed":
            arrangement = self.effective_settings("bed").get("pillow_arrangement", "center")
            self.say("Here is how I will arrange the pillow: %s. (Demo placeholder)" % arrangement)
            self.base_gesture_wiggle()
        elif room == "kitchen":
            snack = self.effective_settings("kitchen").get("snack_preference", "cheetos")
            self.say("Here is the snack I will retrieve: %s. (Demo placeholder)" % snack)
            self.base_gesture_wiggle()

        self.say("Demo complete.")
        self.enter_walk()

    def enter_rules(self):
        self.phase = Phase.RULES
        self.step_index = 0
        self.pending_steps = [
            Step(
                step_id="rules",
                prompt=("Before we end this round, you may set home-wide situational settings like no-go zones and time-based rules. "
                        "Please submit your situational rules now."),
                scope="rules",
            )
        ]
        self.present_current_step()

    def enter_round_switch(self):
        self.phase = Phase.ROUND_SWITCH
        self.step_index = 0
        self.pending_steps = [
            Step(
                step_id="round2_intro",
                prompt=("Welcome back. I understand Jackie will be staying with you for a week. "
                        "We’ll now walk through the apartment again. This time, please configure my behavior to best support Jackie’s needs."),
                scope="meta",
            )
        ]
        self.present_current_step()

    def enter_end(self):
        self.phase = Phase.END
        self.mode_follow = False
        self.publish_stop()
        self.say("Thank you. This completes the session.")
        self.archive_snapshot("end")

    # ----------------------------
    # FSM: present step and advance
    # ----------------------------

    def present_current_step(self):
        if self.step_index >= len(self.pending_steps):
            return

        step = self.pending_steps[self.step_index]
        self.say(step.prompt)

        if step.scope == "global" and step.key:
            self.study.waiting_for = step.key
            self.say("(Waiting for global setting '%s' via /%s)" % (step.key, self.event_topic))
        elif step.scope == "room" and step.room and step.key:
            self.study.waiting_for = "%s.%s" % (step.room, step.key)
            self.say("(Waiting for room setting '%s.%s' via /%s)" % (step.room, step.key, self.event_topic))
        elif step.scope == "rules":
            self.study.waiting_for = "rules"
            self.say("(Waiting for situational rules via /%s)" % self.event_topic)
        else:
            self.study.waiting_for = ""

    def advance(self):
        with self._lock:
            if self.step_index < len(self.pending_steps):
                step = self.pending_steps[self.step_index]
                if step.scope in ("global", "room", "rules") and self.study.waiting_for:
                    self.say("Still waiting for input for this step. Press ENTER again to force skip.")
                    self.study.waiting_for = ""
                    return

                self.step_index += 1
                if self.step_index < len(self.pending_steps):
                    self.present_current_step()
                    return

            # Phase transitions
            if self.phase == Phase.INTRO:
                self.enter_global_wizard()
                return

            if self.phase == Phase.GLOBAL_WIZARD:
                self.archive_snapshot("global_done_%s" % self.study.active_profile)
                self.enter_walk()
                return

            if self.phase == Phase.STATION_CONFIG:
                room = self.study.current_room
                self.archive_snapshot("%s_config_done_%s" % (room, self.study.active_profile))
                self.enter_demo_prompt(room)
                return

            if self.phase == Phase.STATION_DEMO_PROMPT:
                room = self.study.current_room
                yes = self.study.demo_confirm.get(room)
                if yes is None:
                    self.say("Please confirm yes or no for the demo.")
                    return
                if yes:
                    self.enter_demo_run(room)
                else:
                    self.say("Okay. Skipping demo.")
                    self.enter_walk()
                return

            if self.phase == Phase.WALK_AND_STATIONS:
                self.say("Waiting for you to arrive at a station: desk, bed, or kitchen.")
                return

            if self.phase == Phase.RULES:
                if self.study.active_profile == "user":
                    self.say("Thank you for onboarding me in your home.")
                    self.archive_snapshot("round1_done")
                    self.study.active_profile = "jackie"
                    self.enter_round_switch()
                else:
                    self.archive_snapshot("round2_done")
                    self.enter_end()
                return

            if self.phase == Phase.ROUND_SWITCH:
                self.say("Just like before, please guide me to each room and tell me where we are.")
                self.enter_walk()
                return

            if self.phase == Phase.END:
                self.say("Session already complete.")
                return

    # ----------------------------
    # Station arrival and flow
    # ----------------------------

    def on_arrive(self, room: str):
        self.mode_follow = False
        self.publish_stop()

        if self.phase == Phase.WALK_AND_STATIONS:
            self.enter_station_config(room)
            return

        self.say("Arrival at %s received, but I'm not currently walking between stations." % room)

    # ----------------------------
    # Demo placeholder motion
    # ----------------------------

    def base_gesture_wiggle(self):
        if self.estop:
            return

        def _run():
            for w in (0.6, -0.6, 0.6, 0.0):
                if self.estop:
                    break
                msg = Twist()
                msg.angular.z = float(w)
                self.cmd_pub.publish(msg)
                time.sleep(0.35)
            self.publish_stop()

        t = threading.Thread(target=_run, daemon=True)
        t.start()

    # ----------------------------
    # Debug helpers
    # ----------------------------

    def print_effective(self):
        eff_global = self.effective_settings(None)
        eff_room = self.effective_settings(self.study.current_room)
        self.log_info("Active profile: %s" % self.study.active_profile)
        self.log_info("Current room: %s" % self.study.current_room)
        self.log_info("Global defaults: %s" % json.dumps(eff_global, indent=2))
        self.log_info("Effective in room: %s" % json.dumps(eff_room, indent=2))


# ----------------------------
# Terminal key loop
# ----------------------------

def terminal_key_loop(node: StretchStudyController):
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    node.log_info("Terminal hotkeys: ENTER advance | f follow | 1/2/3 arrive | d demo yes/run | e estop | p print | s stop | q quit")

    try:
        while rclpy.ok():
            ch = sys.stdin.read(1)
            if not ch:
                continue

            if ch == "q":
                node.say("Quitting.")
                node.publish_stop()
                break

            if ch in ("\r", "\n"):
                node.advance()
                continue

            if ch == "e":
                node.set_estop(not node.estop)
                continue

            if ch == "s":
                node.mode_follow = False
                node.publish_stop()
                node.say("Stopped.")
                continue

            if ch == "f":
                if node.estop:
                    node.say("Cannot follow while emergency stop is enabled.")
                else:
                    node.mode_follow = not node.mode_follow
                    node.say("Follow mode on." if node.mode_follow else "Follow mode off.")
                continue

            if ch == "1":
                node.handle_event({"type": "arrive", "room": "desk"})
                continue
            if ch == "2":
                node.handle_event({"type": "arrive", "room": "bed"})
                continue
            if ch == "3":
                node.handle_event({"type": "arrive", "room": "kitchen"})
                continue

            if ch == "d":
                room = node.study.current_room
                node.handle_event({"type": "demo_confirm", "room": room, "yes": True})
                if node.phase != Phase.STATION_DEMO_PROMPT:
                    node.enter_demo_run(room)
                continue

            if ch == "p":
                node.print_effective()
                continue
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ----------------------------
# Main
# ----------------------------

def main():
    rclpy.init()
    node = StretchStudyController()

    t = threading.Thread(target=terminal_key_loop, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
