#!/usr/bin/env python3
"""
stretch_main_controller_v2.py
Event-driven control script for Stretch robot study.

The voice assistant pushes events (setting confirmations, ACTION tags,
section completions) via HTTP POST to this controller's Flask server.
The controller reacts by navigating, applying settings, and running demos.

Flow:
    voice assistant completes section 1  →  robot navigates to desk
    voice assistant triggers ACTION tag   →  robot performs demo
    voice assistant completes section 2  →  robot navigates to bed
    ... etc.

Usage:
    # Make sure stretch_core is running in navigation mode
    ros2 launch stretch_core stretch_driver.launch.py mode:=navigation

    # Event-driven mode (voice assistant pushes events):
    python3 stretch_main_controller_v2.py --voice-url http://192.168.0.23:5050

    # Manual mode (operator presses Enter, like v1):
    python3 stretch_main_controller_v2.py --voice-url http://192.168.0.23:5050 --manual

    # Standalone mode (no voice assistant — load most recent saved configs
    # from experiment_record/ and wait for action events / manual triggers):
    python3 stretch_main_controller_v2.py --standalone

    # Start standalone mode at a specific section (e.g. skip hello, assume
    # sections 1-2 are already configured, fill gaps with defaults):
    python3 stretch_main_controller_v2.py --standalone 3
"""

import argparse
import json
import json as _json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import urllib.request

import rclpy
from flask import Flask, jsonify, request as flask_request
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Import config loader from same repo
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from stretch_config_loader import StretchConfigLoader

# Hardcoded navigation/demo scripts live in capabilities/
_SCRIPTS_DIR = os.path.join(_HERE, "capabilities")

CAPABILITIES_DIR = os.path.join(_HERE, "capabilities")

# Social distance → offset in meters (shortens last drive to desk)
SOCIAL_DISTANCE_OFFSET = {"close": 0.0, "medium": 0.1, "far": 0.2}


def _speed_label(value):
    """Normalize a movement-speed setting to a label.

    Voice/hybrid send strings ("slow"/"medium"/"fast"); the visual tablet
    sends a float (e.g. 0.5). Nav scripts expect the label form.
    """
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if value <= 0.15:
            return "slow"
        if value <= 0.30:
            return "medium"
        return "fast"
    s = str(value).strip().lower()
    if s in ("slow", "medium", "fast"):
        return s
    return "medium"


def _bed_drop_label(value) -> str:
    """Normalize bed demo variants to the drop labels bed_demo.py expects."""
    alias_map = {
        "left": "left",
        "right": "right",
        "middle": "middle",
        "center": "middle",
        "top": "middle",
    }
    return alias_map.get(str(value).strip().lower(), "middle")

# Bed demo spacing and post-demo drive target.
BED_DEMO_STEP_M = 0.6
BED_POSITION_FORWARD_OFFSET_M = {
    "right": 0.0,
    "middle": BED_DEMO_STEP_M,
    "left": 2.0 * BED_DEMO_STEP_M,
}
BED_POST_DEMO_TOTAL_DRIVE_M = 3.048

# Kitchen approach distance after any deferred bed-to-kitchen repositioning.
# The bed demo leaves the robot facing toward the kitchen; the kitchen flow
# drives straight in that same direction (NO additional turn) after the user
# submits kitchen settings.
KITCHEN_APPROACH_DIST_M = 2.0

# Map ACTION tags to (room, demo_type) for dispatch
ACTION_MAP = {
    "PERFORM_DESK_DEMO_1": ("desk", 1),
    "PERFORM_DESK_DEMO_2": ("desk", 2),
    "PERFORM_DESK_DEMO_3": ("desk", 3),
    "PERFORM_BED_DEMO_LEFT": ("bed", "left"),
    "PERFORM_BED_DEMO_CENTER": ("bed", "center"),
    "PERFORM_BED_DEMO_MIDDLE": ("bed", "middle"),
    "PERFORM_BED_DEMO_RIGHT": ("bed", "right"),
    "PERFORM_BED_DEMO_TOP": ("bed", "top"),
    # One kitchen demo: the operator teleops the robot into position for
    # whichever item (milk or ketchup), and kitchen_demo_final.py runs the
    # same grasp sequence either way. The chosen item is only relevant to
    # the LLM's TTS announcement.
    "PERFORM_KITCHEN_DEMO": ("kitchen", None),
}

ROOM_TO_SECTION = {"desk": 2, "bed": 3, "kitchen": 4}


# =============================================================================
# Main Controller Node
# =============================================================================

class StretchMainController(Node):

    def __init__(self, voice_url=None, flask_port=5060, manual=False, start_section=1,
                 standalone=False):
        super().__init__("stretch_main_controller")

        self.voice_url = voice_url.rstrip("/") if voice_url else None
        self.flask_port = flask_port
        self.manual = manual
        self.start_section = start_section
        self.standalone = standalone

        # --- Config loader (remote or local) ---
        # In standalone mode, skip the remote URL so the loader reads the most
        # recent JSON files from experiment_record/ without hitting the voice
        # server (which isn't running during robot-only testing).
        self.loader = StretchConfigLoader(remote_url=None if standalone else voice_url)

        # --- Event queue (voice assistant pushes events here) ---
        self.event_queue = queue.Queue()

        # --- Live settings (updated by setting_confirmed events) ---
        self.live_settings = {}

        # --- Station settings cache (loaded at section completion) ---
        self.desk = None
        self.bed = None
        self.kitchen = None
        self.general = None
        self._deferred_bed_to_kitchen_drive_m = None

        # --- Section/action gating ---
        # Visual mode sends section_complete and action as separate HTTP
        # requests. Those requests are posted from background threads, so
        # delivery order is not guaranteed. Keep actions pending until the
        # matching section is explicitly complete and its config has been
        # loaded by the controller.
        self.completed_sections = set()
        self._pending_actions = []
        self._handled_section_actions = set()
        self._awaiting_section = None

        # --- Driver mode switching ---
        self.srv_navigation_mode = self.create_client(
            Trigger, "/switch_to_navigation_mode"
        )
        self.srv_position_mode = self.create_client(
            Trigger, "/switch_to_position_mode"
        )

        self._cmd_vel_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        # --- Ambient movement state ---
        self._ambient_proc = None
        self._ambient_enabled = False

        # True while _handle_action is running its nav/teleop/demo pipeline.
        # Used to suppress concurrent capabilities (nod.py, ambient restarts)
        # that would fight the demo for the trajectory action server / cmd_vel.
        self._demo_in_progress = False

        # --- Keyboard input for manual mode ---
        # `_key_wait_active` gates the key thread so stdin is only consumed
        # when we're explicitly waiting for operator input. Otherwise stray
        # Enter presses during autonomous operation (e.g. between nav and
        # teleop) would pre-set `_key_event` and skip the next wait.
        self._key_event = threading.Event()
        self._key_wait_active = threading.Event()
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

    # -----------------------------------------------------------------
    # Flask event receiver
    # -----------------------------------------------------------------

    def start_flask_server(self):
        """Start Flask server in background thread to receive events from voice assistant."""
        app = Flask(__name__)

        @app.route("/event", methods=["POST"])
        def receive_event():
            data = flask_request.get_json(silent=True) or {}
            event_type = data.get("event", "unknown")
            callback_url = data.get("callback_url") or data.get("voice_url")
            if callback_url:
                self.voice_url = str(callback_url).rstrip("/")
                self.get_logger().info(
                    f"[Event] Demo callback URL set to: {self.voice_url}"
                )
            self.get_logger().info(f"[Event] Received: {event_type} | {json.dumps(data)}")
            self.event_queue.put(data)
            return jsonify({"ok": True, "event": event_type})

        @app.route("/health", methods=["GET"])
        def health():
            return jsonify({
                "status": "ok",
                "mode": "manual" if self.manual else "event-driven",
                "queue_size": self.event_queue.qsize(),
            })

        thread = threading.Thread(
            target=lambda: app.run(
                host="0.0.0.0", port=self.flask_port,
                debug=False, use_reloader=False, threaded=True,
            ),
            daemon=True,
        )
        thread.start()
        self.get_logger().info(f"[Init] Event server on port {self.flask_port}")

    # -----------------------------------------------------------------
    # Event helpers
    # -----------------------------------------------------------------

    def wait_for_event(self, event_type: str, timeout: float = 600.0,
                       section: int = None) -> dict:
        """Block until an event of the given type arrives. Returns the event dict."""
        label = f"{event_type}"
        if section is not None:
            label += f" section={section}"
        self.get_logger().info(f"[Wait] Waiting for event: {label}")
        if event_type == "section_complete" and section in self.completed_sections:
            self.get_logger().info(
                f"[Wait] Section {section} was already completed; continuing"
            )
            return {"event": "section_complete", "section": section}
        start = time.time()
        previous_awaiting = self._awaiting_section
        if event_type == "section_complete":
            self._awaiting_section = section
        try:
            while time.time() - start < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                try:
                    ev = self.event_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if ev.get("event") == event_type:
                    if section is not None and ev.get("section") != section:
                        self.get_logger().info(
                            f"[Wait] Saw {event_type} for section {ev.get('section')} "
                            f"while waiting for section {section}; handling without unblocking"
                        )
                        self._handle_event(ev)
                        continue
                    self._handle_event(ev)
                    return ev
                self._handle_event(ev)
        finally:
            self._awaiting_section = previous_awaiting
        self.get_logger().warn(f"[Wait] Timeout waiting for {label}")
        return {}

    def drain_events(self):
        """Process all queued events without blocking."""
        while not self.event_queue.empty():
            try:
                ev = self.event_queue.get_nowait()
                self._handle_event(ev)
            except queue.Empty:
                break

    # Events the assistant emits as informational signals but that have no
    # controller-side behavior. We acknowledge them silently so they don't
    # show up as "Unhandled" in the log.
    _SILENTLY_IGNORED_EVENTS = frozenset({"user_profile", "enter_room"})

    def _handle_event(self, ev: dict):
        """Handle a single event that arrived while waiting for something else."""
        event_type = ev.get("event")
        if event_type == "setting_confirmed":
            key = ev.get("key", "")
            value = ev.get("value")
            self.live_settings[key] = value
            self.get_logger().info(f"[Setting] {key} = {value}")
            if key == "ambient_movement" and str(value).lower() in ("off", "false", "no"):
                self.stop_ambient()
        elif event_type == "user_confirmed":
            if self._demo_in_progress:
                self.get_logger().info(
                    "[Event] user_confirmed ignored — demo in progress "
                    "(nod.py would conflict with demo trajectory action)"
                )
                return
            was_ambient = self._ambient_enabled
            self.stop_ambient()
            self.run_capability("nod.py", wait=True)
            if was_ambient:
                self.start_ambient()
        elif event_type == "action":
            self._handle_action(ev.get("action", ""))
        elif event_type == "section_complete":
            section = ev.get("section")
            if section is not None:
                if self._awaiting_section != section:
                    self.get_logger().warn(
                        f"[Section] Ignoring section_complete for section {section}; "
                        f"currently waiting for {self._awaiting_section}"
                    )
                    return
                self.completed_sections.add(section)
                self.get_logger().info(f"[Section] Completed: {section}")
        elif event_type in self._SILENTLY_IGNORED_EVENTS:
            return
        else:
            self.get_logger().info(f"[Event] Unhandled: {ev}")

    def _section_for_action(self, action: str):
        mapping = ACTION_MAP.get(action)
        if not mapping:
            return None
        room, _ = mapping
        return ROOM_TO_SECTION.get(room)

    def _queue_action_until_ready(self, action: str, reason: str):
        if action not in self._pending_actions:
            self._pending_actions.append(action)
        self.get_logger().info(f"[Action] Deferred {action}: {reason}")

    def _room_config_loaded(self, room: str) -> bool:
        return getattr(self, room, None) is not None

    def _run_pending_action_for_section(self, section: int):
        for action in list(self._pending_actions):
            if self._section_for_action(action) != section:
                continue
            self._pending_actions.remove(action)
            self.get_logger().info(
                f"[Action] Running deferred action for section {section}: {action}"
            )
            self._handle_action(action)
            return

    # -----------------------------------------------------------------
    # Mode switching + teleop handoff
    # -----------------------------------------------------------------

    def _switch_mode(self, mode: str) -> bool:
        """Call /switch_to_navigation_mode or /switch_to_position_mode.

        mode: 'navigation' or 'position'.
        Returns True if the switch succeeded.
        """
        client = self.srv_navigation_mode if mode == "navigation" else self.srv_position_mode
        name = f"/switch_to_{mode}_mode"
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} service unavailable")
            return False
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done():
            self.get_logger().error(f"{name} timed out")
            return False
        resp = fut.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(f"{name}: success={ok}")
        time.sleep(0.3)
        return ok

    def _teleop_pause(self, room: str):
        """Hand control to a human teleoperator between hardcoded nav and the demo.

        The robot is stopped and switched into position mode so that
        `ros2 run stretch_core keyboard_teleop` works from another terminal.
        Blocks on ENTER; then switches back to navigation mode so the demo
        scripts can drive cmd_vel as needed.
        """
        # Stop any residual motion
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.3)

        # Switch into position mode for keyboard_teleop
        self._switch_mode("position")

        print("\n" + "=" * 60)
        print(f"[TELEOP] Handoff to human operator for {room} station")
        print("In another terminal, run one of the teleop tools:")
        print("  Keyboard (joint/arm control, position mode):")
        print("    ros2 run stretch_core keyboard_teleop")
        print("Drive the robot as needed, then press ENTER here to resume.")
        print("=" * 60 + "\n")

        self.wait_for_operator("Press ENTER when teleop is complete to run the demo...")

        # Ensure the base is stopped before demo takes over
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.2)

        # Switch back to navigation mode so demo cmd_vel works
        self._switch_mode("navigation")
        self.get_logger().info(f"[TELEOP] Resuming automated demo for {room}")

    def _handle_action(self, action: str):
        """Navigate to the station, apply social distance, then run the demo."""
        if action not in ACTION_MAP:
            self.get_logger().warn(f"[Action] Unknown action: {action}")
            return
        if self._demo_in_progress:
            self._queue_action_until_ready(
                action, "another demo is already in progress"
            )
            return
        room, param = ACTION_MAP[action]
        section = ROOM_TO_SECTION.get(room)
        if section not in self.completed_sections:
            if self._awaiting_section == section:
                self._queue_action_until_ready(
                    action, f"section {section} has not completed yet"
                )
            else:
                self.get_logger().warn(
                    f"[Action] Ignoring premature {action}; section {section} "
                    f"is not the active section"
                )
            return
        if not self._room_config_loaded(room):
            self._queue_action_until_ready(
                action, f"{room} config has not been loaded yet"
            )
            return
        if section in self._handled_section_actions:
            self.get_logger().warn(
                f"[Action] Ignoring duplicate {action} for completed section {section}"
            )
            return
        self.get_logger().info(f"[Action] Dispatching: {action} -> {room}({param})")
        self._handled_section_actions.add(section)
        self._demo_in_progress = True
        was_ambient = self._ambient_enabled
        self.stop_ambient()
        try:
            self._run_action_pipeline(room, param)
        finally:
            self._demo_in_progress = False

        # Notify voice assistant that demo is done (guard already cleared)
        self._notify_demo_complete(room)

        if was_ambient:
            self.start_ambient()

    def _room_setting(self, room: str, key: str, default):
        """Read a room setting from live_settings, falling back to the loaded
        station config (so standalone mode works without setting_confirmed
        events from the voice assistant)."""
        val = self.live_settings.get(key)
        if val is not None:
            return val
        cfg = getattr(self, room, None)
        if cfg is None:
            return default
        attr = {"room_movement_speed": "movement_speed",
                "room_social_distance": "social_distance",
                "preferred_snack": "preferred_snack"}.get(key)
        if attr and hasattr(cfg, attr):
            return getattr(cfg, attr)
        return default

    def _run_action_pipeline(self, room: str, param):
        """The actual nav/teleop/demo pipeline, wrapped so the caller can
        guarantee _demo_in_progress is cleared even on exception."""
        speed = _speed_label(self._room_setting(room, "room_movement_speed", "medium"))
        social = self._room_setting(room, "room_social_distance", "close")
        social_offset = SOCIAL_DISTANCE_OFFSET.get(social, 0.0)

        if room == "desk":
            level_map = {1: "low", 2: "medium", 3: "high"}
            level = level_map.get(int(param), "low")
            self.get_logger().info(
                f"[Action] Desk (speed={speed}, level={level}, social={social}, offset={social_offset}m)"
            )

            # Step 1: Navigate with social offset (stops short of demo position)
            nav_script = os.path.join(_SCRIPTS_DIR, "right_angle_forward_path.py")
            self.run_script(nav_script, args=[
                "--speed", speed,
                "--social-offset", str(social_offset),
            ], wait=True)

            # Step 2: Social distance pause → drive to demo position
            self._social_distance_approach(social, social_offset)

            # Step 3: Handoff to human teleop (operator drives base/arm manually)
            self._teleop_pause(room)

            # Step 4: Wipe demo. Skip the script's prewipe pose since the
            # operator already positioned the arm via teleop — overriding it
            # would conflict with the desk and trip joint_lift contact safety.
            demo_script = os.path.join(_SCRIPTS_DIR, "prewipe_wipe_test.py")
            self.run_script(demo_script, args=["--level", level, "--skip-prewipe"], wait=True)

        elif room == "bed":
            # Step 1: Navigate with social offset to the fixed bed demo
            # staging position. drive_straight.py handles the social-distance
            # pause internally, then closes the final offset on odom. The
            # 92° turn is deferred to bed_demo.py so teleop starts facing
            # forward, matching the desk flow's "teleop before demo" handoff.
            total_dist = 2.7  # fixed target distance from bed approach start
            self.get_logger().info(
                f"[Action] Bed nav (speed={speed}, dist={total_dist}m, "
                f"social={social}, offset={social_offset:.2f}m)"
            )
            nav_script = os.path.join(_SCRIPTS_DIR, "drive_straight.py")
            self.run_script(
                nav_script,
                args=[
                    str(total_dist),
                    "--speed", speed,
                    "--skip-end-turn",
                    "--social-offset", str(social_offset),
                ],
                wait=True,
            )

            # Step 2: Handoff to human teleop (robot at the fixed target,
            # still facing forward).
            self._teleop_pause(room)

            # Step 3: bed_demo.py — turn 92° left toward the bed, then the
            # grasp / carry / release sequence. Handles all drop values
            # including aliases from older bed section configs.
            drop = _bed_drop_label(param)

            self.get_logger().info(f"[Action] Bed demo (pillow={param}, drop={drop})")
            demo_script = os.path.join(_SCRIPTS_DIR, "bed_demo.py")
            self.run_script(demo_script, args=["--drop", drop], wait=True)

            # Defer the deterministic post-drop repositioning drive until
            # the kitchen section is submitted. This matches the office ->
            # bedroom pattern: finish the current demo, wait for the next
            # room settings, then navigate.
            already_traveled_m = BED_POSITION_FORWARD_OFFSET_M.get(drop, BED_DEMO_STEP_M)
            remaining_dist_m = max(0.0, BED_POST_DEMO_TOTAL_DRIVE_M - already_traveled_m)
            self._deferred_bed_to_kitchen_drive_m = remaining_dist_m
            self.get_logger().info(
                f"[Action] Deferred bed-to-kitchen reposition drive until kitchen section "
                f"(target={BED_POST_DEMO_TOTAL_DRIVE_M:.3f}m, "
                f"drop_offset={already_traveled_m:.3f}m, "
                f"remaining={remaining_dist_m:.3f}m)"
            )

        elif room == "kitchen":
            # Step 1: Drive forward toward the kitchen. The bed demo already
            # turned the robot 92° left, so the robot is pointed toward the
            # kitchen. Any bed-to-kitchen repositioning that used to happen
            # at the end of the bed demo is now included here, after the
            # kitchen settings are complete.
            deferred_dist_m = self._deferred_bed_to_kitchen_drive_m or 0.0
            total_kitchen_drive_m = deferred_dist_m + KITCHEN_APPROACH_DIST_M
            self.get_logger().info(
                f"[Action] Kitchen nav (drive_straight, speed={speed}, "
                f"deferred={deferred_dist_m:.2f}m, approach={KITCHEN_APPROACH_DIST_M:.2f}m, "
                f"total={total_kitchen_drive_m:.2f}m, social={social}, offset={social_offset:.2f}m)"
            )
            nav_script = os.path.join(_SCRIPTS_DIR, "drive_straight.py")
            self.run_script(
                nav_script,
                args=[
                    str(total_kitchen_drive_m),
                    "--speed", speed,
                    "--skip-end-turn",
                    "--social-offset", str(social_offset),
                ],
                wait=True,
            )
            self._deferred_bed_to_kitchen_drive_m = None

            # Step 2: Handoff to human teleop — operator drives the robot
            # into position for whichever item (milk or ketchup) the user
            # chose. kitchen_demo_final.py runs the same grasp sequence.
            self._teleop_pause(room)

            # Step 3: Run the kitchen demo (single demo for both items).
            item = self._room_setting("kitchen", "preferred_snack", "milk")
            self.get_logger().info(f"[Action] Kitchen demo (item={item})")
            demo_script = os.path.join(_SCRIPTS_DIR, "kitchen_demo_final.py")
            self.run_script(demo_script, wait=True)

    def _notify_demo_complete(self, room: str):
        """POST demo_complete to the voice assistant so it continues the conversation."""
        if not self.voice_url:
            return
        url = f"{self.voice_url}/api/demo_complete"
        data = _json.dumps({"result": f"{room} demo complete"}).encode()
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                self.get_logger().info(f"[DemoNotify] Notified voice assistant: {resp.status}")
        except Exception as e:
            self.get_logger().warn(f"[DemoNotify] Failed to notify voice assistant: {e}")

    def _social_distance_approach(self, social: str, offset: float):
        """Pause at the social distance position, then drive forward to the
        fixed demo position. Ensures every demo starts from the exact same spot."""
        if offset > 0:
            self.get_logger().info(
                f"[Social] Stopped at '{social}' distance ({offset:.2f}m from demo position). Pausing 2s..."
            )
            time.sleep(2.0)
            # Drive forward to the demo position
            self.get_logger().info(f"[Social] Driving {offset:.2f}m forward to demo position")
            self._drive_forward_cmd_vel(offset)
        else:
            self.get_logger().info("[Social] Already at demo position (close distance)")

    def _drive_forward_cmd_vel(self, distance_m: float, speed: float = 0.08):
        """Drive forward a short distance using cmd_vel (no turn, no mode switch)."""
        if distance_m <= 0:
            return
        duration = distance_m / speed
        self.get_logger().info(f"[Drive] Forward {distance_m:.2f}m at {speed} m/s for {duration:.1f}s")

        msg = Twist()
        msg.linear.x = speed
        start = time.time()
        while time.time() - start < duration:
            self._cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.3)

    def run_script(self, script_path: str, args: list = None, wait: bool = False):
        """Run an arbitrary Python script as a subprocess."""
        cmd = ["python3", script_path]
        if args:
            cmd.extend(str(a) for a in args)
        self.get_logger().info(f"[Script] Running: {' '.join(cmd)}")
        proc = subprocess.Popen(cmd)
        if wait:
            while proc.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.drain_events()
            self.get_logger().info(f"[Script] Completed (rc={proc.returncode})")
        return proc

    # -----------------------------------------------------------------
    # Ambient movement
    # -----------------------------------------------------------------

    def start_ambient(self):
        """Start ambient movement background process (respects user preference)."""
        if self._demo_in_progress:
            self.get_logger().info("[Ambient] Skipped — demo in progress")
            return
        if str(self.live_settings.get("ambient_movement", "on")).lower() in ("off", "false", "no"):
            self.get_logger().info("[Ambient] Skipped — user set ambient movement to off")
            return
        self._ambient_enabled = True
        if self._ambient_proc is not None and self._ambient_proc.poll() is None:
            return
        self._ambient_proc = subprocess.Popen(
            [
                "python3",
                os.path.join(CAPABILITIES_DIR, "ambient_movement.py"),
                "--skip-nav-switch",
            ]
        )
        self._ambient_start_time = time.time()
        self.get_logger().info("[Ambient] Started")

    def stop_ambient(self):
        """Stop ambient movement immediately and publish zero cmd_vel."""
        self._ambient_enabled = False
        if self._ambient_proc is None or self._ambient_proc.poll() is not None:
            self._ambient_proc = None
            self._cmd_vel_pub.publish(Twist())
            return

        self._ambient_proc.send_signal(signal.SIGINT)
        try:
            self._ambient_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._ambient_proc.kill()
            self._ambient_proc.wait()
        self._ambient_proc = None
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info("[Ambient] Stopped")

    # -----------------------------------------------------------------
    # Capability scripts (subprocess)
    # -----------------------------------------------------------------

    def run_capability(self, script: str, args: list = None, wait: bool = False):
        """Run a capability script from capabilities/ as a subprocess."""
        cmd = ["python3", os.path.join(CAPABILITIES_DIR, script)]
        if args:
            cmd.extend(str(a) for a in args)
        self.get_logger().info(f"[Capability] Running: {' '.join(cmd)}")
        proc = subprocess.Popen(cmd)
        if wait:
            while proc.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.drain_events()
            self.get_logger().info(
                f"[Capability] {script} completed (rc={proc.returncode})"
            )
        return proc

    # -----------------------------------------------------------------
    # Operator keyboard control (manual mode)
    # -----------------------------------------------------------------

    def _key_loop(self):
        """Continuously read stdin lines. Only set `_key_event` when a
        `wait_for_operator` call has activated `_key_wait_active` — otherwise
        the Enter is discarded. This prevents stray Enters pressed during
        autonomous operation from unblocking future waits."""
        while True:
            try:
                input()
            except EOFError:
                break
            if self._key_wait_active.is_set():
                self._key_event.set()
            # else: discard the keypress silently

    def wait_for_operator(self, prompt: str = "Press ENTER to continue..."):
        # Enable the key loop and clear any latent state
        self._key_event.clear()
        self._key_wait_active.set()
        try:
            self.get_logger().info(f"[Operator] {prompt}")
            print("\n" + "=" * 60)
            print(f">>> {prompt}")
            print("=" * 60 + "\n", flush=True)
            while not self._key_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
                self.drain_events()
        finally:
            self._key_wait_active.clear()
            self._key_event.clear()

    # -----------------------------------------------------------------
    # Main sequence — event-driven
    # -----------------------------------------------------------------

    def _mode_label(self) -> str:
        if self.standalone:
            return "standalone"
        return "manual" if self.manual else "event-driven"

    def run(self):
        """Execute the full study sequence."""
        self.get_logger().info("=" * 55)
        self.get_logger().info("  STRETCH MAIN CONTROLLER v2 (hardcoded nav)")
        self.get_logger().info(f"  Mode: {self._mode_label()}")
        self.get_logger().info("=" * 55)

        # Wait for odom
        self.get_logger().info("[Init] Waiting for odometry...")
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=1.0)

        if self.standalone:
            self._run_standalone()
            return

        ss = self.start_section
        if ss > 1:
            self.get_logger().info(f"[Init] Skipping to section {ss}")
            self.start_ambient()

        # =============================================================
        # SESSION START: hello greeting
        # =============================================================
        if ss <= 1:
            if self.manual:
                self.wait_for_operator("Press ENTER to start hello greeting...")
            else:
                # Wait for session_start from voice/hybrid assistant.
                # If using viz assistant, start ambient + hello immediately
                # since session_start may arrive before we're listening.
                ev = self.wait_for_event("session_start", timeout=10.0)
                if not ev:
                    self.get_logger().info("[Init] No session_start received, proceeding anyway")

            self.run_capability("hello.py", wait=True)
            self.start_ambient()

            # PHASE 1: Wait for section 1 complete (general settings)
            if self.manual:
                self.wait_for_operator("Section 1 done. Press ENTER to continue...")
            else:
                if not self.wait_for_event("section_complete", section=1):
                    self.get_logger().error(
                        "[Wait] Section 1 did not complete; stopping before loading defaults"
                    )
                    return

            self.general = self.loader.load_general_settings()
            self.get_logger().info(
                f"[Config] General: personality={self.general.personality}, "
                f"explainability={self.general.explainability_level}"
            )

        # =============================================================
        # PHASE 2: Wait for section 2 (desk configs + demo).
        #          ACTION event triggers navigate + demo automatically.
        # =============================================================
        if ss <= 2:
            if self.manual:
                self.wait_for_operator("Section 2 done. Press ENTER...")
            else:
                if not self.wait_for_event("section_complete", section=2):
                    self.get_logger().error(
                        "[Wait] Section 2 did not complete; stopping before running demo"
                    )
                    return

            self.desk = self.loader.load_desk_settings()
            self.get_logger().info(
                f"[Config] Desk: speed={self.desk.movement_speed}, "
                f"cleaning={self.desk.cleaning_thoroughness}"
            )
            self._run_pending_action_for_section(2)

        # =============================================================
        # PHASE 3: Wait for section 3 (bed configs + demo).
        # =============================================================
        if ss <= 3:
            if self.manual:
                self.wait_for_operator("Section 3 done. Press ENTER...")
            else:
                if not self.wait_for_event("section_complete", section=3):
                    self.get_logger().error(
                        "[Wait] Section 3 did not complete; stopping before running demo"
                    )
                    return

            self.bed = self.loader.load_bed_settings()
            self.get_logger().info(
                f"[Config] Bed: speed={self.bed.movement_speed}, "
                f"pillow={self.bed.pillow_arrangement}"
            )
            self._run_pending_action_for_section(3)

        # =============================================================
        # PHASE 4: Wait for section 4 (kitchen configs + demo).
        # =============================================================
        if self.manual:
            self.wait_for_operator("Section 4 done. Press ENTER...")
        else:
            if not self.wait_for_event("section_complete", section=4):
                self.get_logger().error(
                    "[Wait] Section 4 did not complete; stopping before running kitchen demo"
                )
                return

        self.kitchen = self.loader.load_kitchen_settings()
        self.get_logger().info(
            f"[Config] Kitchen: speed={self.kitchen.movement_speed}, "
            f"snack={self.kitchen.preferred_snack}"
        )
        self._run_pending_action_for_section(4)

        # =============================================================
        self.get_logger().info("=" * 55)
        self.get_logger().info("  Study sequence complete!")
        self.get_logger().info("=" * 55)

    def _standalone_action_for_section(self, ss: int) -> str:
        """Pick the ACTION tag that matches section `ss`, using the saved
        station config to choose the specific variant (wipe level / pillow
        drop / snack). Returns '' if there's no demo for this section."""
        if ss == 2 and self.desk:
            return f"PERFORM_DESK_DEMO_{self.desk.get_wipe_count()}"
        if ss == 3 and self.bed:
            drop = _bed_drop_label(self.bed.pillow_arrangement)
            variant = "CENTER" if drop == "middle" else drop.upper()
            return f"PERFORM_BED_DEMO_{variant}"
        if ss == 4 and self.kitchen:
            # Kitchen demo is the same regardless of milk vs ketchup —
            # the operator teleops into position for the chosen item.
            return "PERFORM_KITCHEN_DEMO"
        return ""

    def _run_standalone(self):
        """Robot-only testing: load the most recent saved configs from
        experiment_record/ and immediately run the demo for the chosen
        section (nav → teleop handoff → demo) without waiting for voice-
        assistant events.

        `self.start_section` (from --standalone N) picks which section's
        demo to auto-dispatch. Sections before it are assumed already
        configured (saved records loaded; defaults fill any gaps). After
        the demo, ambient runs and the event loop stays alive so additional
        action events can be POSTed to /event."""
        ss = self.start_section
        self.get_logger().info(
            f"[Standalone] Starting from section {ss} — loading saved configs"
        )

        self.general = self.loader.load_general_settings()
        self.desk = self.loader.load_desk_settings()
        self.bed = self.loader.load_bed_settings()
        self.kitchen = self.loader.load_kitchen_settings()
        self.completed_sections.update(range(1, ss + 1))

        self.get_logger().info(
            f"[Config] General: personality={self.general.personality}, "
            f"explainability={self.general.explainability_level}"
        )
        self.get_logger().info(
            f"[Config] Desk: speed={self.desk.movement_speed}, "
            f"social={self.desk.social_distance}, "
            f"cleaning={self.desk.cleaning_thoroughness}"
        )
        self.get_logger().info(
            f"[Config] Bed: speed={self.bed.movement_speed}, "
            f"social={self.bed.social_distance}, "
            f"pillow={self.bed.pillow_arrangement}"
        )
        self.get_logger().info(
            f"[Config] Kitchen: speed={self.kitchen.movement_speed}, "
            f"social={self.kitchen.social_distance}, "
            f"snack={self.kitchen.preferred_snack}"
        )

        if ss <= 1:
            # Section 1 has no demo — just do the hello greeting.
            self.run_capability("hello.py", wait=True)
        else:
            # Auto-dispatch the section's demo: nav → teleop → demo.
            # _handle_action manages ambient stop/restart on its own, so
            # don't start ambient beforehand.
            action = self._standalone_action_for_section(ss)
            if action:
                self.get_logger().info(
                    f"[Standalone] Auto-dispatching {action} for section {ss}"
                )
                self._handle_action(action)
            else:
                self.get_logger().warn(
                    f"[Standalone] No demo mapped for section {ss}"
                )
        self.start_ambient()

        self.get_logger().info(
            "[Standalone] Ready — POST action events to /event "
            f"on port {self.flask_port} to trigger more demos. Ctrl-C to exit."
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.drain_events()


# =============================================================================
# Entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Stretch Main Controller v2")
    parser.add_argument(
        "--voice-url", type=str, default="http://192.168.0.23:5050",
        help="URL of the voice assistant Flask server",
    )
    parser.add_argument(
        "--port", type=int, default=5060,
        help="Port for the event receiver Flask server",
    )
    parser.add_argument(
        "--manual", action="store_true",
        help="Use manual operator mode (Enter key) instead of event-driven",
    )
    parser.add_argument(
        "--start-section", type=int, default=1, choices=[1, 2, 3, 4],
        help="Skip ahead to this section (default: 1)",
    )
    parser.add_argument(
        "--standalone",
        type=int, nargs="?", const=1, default=None,
        choices=[1, 2, 3, 4], metavar="SECTION",
        help=(
            "Robot-only testing: load the most recent saved configs from "
            "experiment_record/ (defaults fill any missing records) and skip "
            "the voice-assistant config phases. Optional SECTION (1-4) is the "
            "section to start from; sections before it are assumed already "
            "configured. Defaults to 1 when the flag is passed alone. "
            "Action events can still be dispatched by POSTing to /event."
        ),
    )
    args, ros_args = parser.parse_known_args()

    standalone = args.standalone is not None
    start_section = args.standalone if standalone else args.start_section

    rclpy.init(args=ros_args)
    node = StretchMainController(
        voice_url=args.voice_url,
        flask_port=args.port,
        manual=args.manual,
        start_section=start_section,
        standalone=standalone,
    )

    node.start_flask_server()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("[Controller] Interrupted by user")
    finally:
        node.stop_ambient()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
