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

import yaml
import rclpy
from flask import Flask, jsonify, request as flask_request
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger

# Import config loader and FUNMAP navigator from same repo
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from stretch_config_loader import StretchConfigLoader

_STUDY_PKG = os.path.join(_HERE, "src", "stretch_study", "stretch_study")
sys.path.insert(0, _STUDY_PKG)
from capabilities.funmap_navigator import FunmapNavigator

GOALS_YAML = os.path.join(_STUDY_PKG, "config", "goals.yml")


# =============================================================================
# Helpers
# =============================================================================

FUNMAP_SPEED = {"slow": 0.2, "medium": 0.25, "fast": 0.3}

CAPABILITIES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "capabilities",
)

# Map ACTION tags to (room, demo_type) for dispatch
ACTION_MAP = {
    "PERFORM_DESK_DEMO_1": ("desk", 1),
    "PERFORM_DESK_DEMO_2": ("desk", 2),
    "PERFORM_DESK_DEMO_3": ("desk", 3),
    "PERFORM_BED_DEMO_CENTER": ("bed", "center"),
    "PERFORM_BED_DEMO_TOP": ("bed", "top"),
    "PERFORM_KITCHEN_DEMO_DORITOS": ("kitchen", "Doritos"),
    "PERFORM_KITCHEN_DEMO_CHEETOS": ("kitchen", "Cheetos"),
}


# =============================================================================
# Main Controller Node
# =============================================================================

class StretchMainController(Node):

    def __init__(self, voice_url=None, flask_port=5060, manual=False, start_section=1):
        super().__init__("stretch_main_controller")

        self.voice_url = voice_url.rstrip("/") if voice_url else None
        self.flask_port = flask_port
        self.manual = manual
        self.start_section = start_section

        # --- Config loader (remote or local) ---
        self.loader = StretchConfigLoader(remote_url=voice_url)

        # --- Event queue (voice assistant pushes events here) ---
        self.event_queue = queue.Queue()

        # --- Live settings (updated by setting_confirmed events) ---
        self.live_settings = {}

        # --- Station settings cache (loaded at section completion) ---
        self.desk = None
        self.bed = None
        self.kitchen = None
        self.general = None
        # --- FUNMAP navigator (publishes goal_pose, monitors TF for arrival) ---
        self.nav = FunmapNavigator(self, goals_yaml=GOALS_YAML)

        # --- FUNMAP velocity parameter client ---
        self.funmap_param_client = self.create_client(
            SetParameters, "/funmap/set_parameters"
        )

        # --- Driver mode switching ---
        self.srv_position_mode = self.create_client(
            Trigger, "/switch_to_position_mode"
        )
        self.srv_navigation_mode = self.create_client(
            Trigger, "/switch_to_navigation_mode"
        )

        # --- Ambient movement state ---
        self._ambient_proc = None
        self._ambient_enabled = False
        self._cmd_vel_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        # --- Keyboard input for manual mode ---
        self._key_event = threading.Event()
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

    def wait_for_event(self, event_type: str, timeout: float = 600.0) -> dict:
        """Block until an event of the given type arrives. Returns the event dict."""
        self.get_logger().info(f"[Wait] Waiting for event: {event_type}")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                ev = self.event_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if ev.get("event") == event_type:
                return ev
            # Not the event we're looking for — handle it anyway
            self._handle_event(ev)
        self.get_logger().warn(f"[Wait] Timeout waiting for {event_type}")
        return {}

    def drain_events(self):
        """Process all queued events without blocking."""
        while not self.event_queue.empty():
            try:
                ev = self.event_queue.get_nowait()
                self._handle_event(ev)
            except queue.Empty:
                break

    def _handle_event(self, ev: dict):
        """Handle a single event that arrived while waiting for something else."""
        event_type = ev.get("event")
        if event_type == "setting_confirmed":
            key = ev.get("key", "")
            value = ev.get("value")
            self.live_settings[key] = value
            self.get_logger().info(f"[Setting] {key} = {value}")
            # Immediately stop ambient if user turned it off
            if key == "ambient_movement" and value == "off":
                self.stop_ambient()
        elif event_type == "user_confirmed":
            was_ambient = self._ambient_enabled
            self.stop_ambient()
            self.run_capability("nod.py", wait=True)
            if was_ambient:
                self.start_ambient()
        elif event_type == "action":
            self._handle_action(ev.get("action", ""))
        else:
            self.get_logger().info(f"[Event] Unhandled: {ev}")

    def _handle_action(self, action: str):
        """Navigate to the station, then run the demo."""
        if action not in ACTION_MAP:
            self.get_logger().warn(f"[Action] Unknown action: {action}")
            return
        room, param = ACTION_MAP[action]
        self.get_logger().info(f"[Action] Dispatching: {action} -> {room}({param})")
        was_ambient = self._ambient_enabled
        self.stop_ambient()

        # Use speed from live_settings (already pushed before ACTION)
        speed = self.live_settings.get("room_movement_speed", "medium")
        self.get_logger().info(f"[Action] Navigating to {room} (speed={speed}) before demo")
        self.navigate_to(room, speed, social_distance_m=0.0)

        if room == "desk":
            self.desk_demo(param)
        elif room == "bed":
            self.bed_demo(param)
        elif room == "kitchen":
            self.kitchen_demo(param)

        # Notify voice assistant that demo is done so it continues
        self._notify_demo_complete(room)

        if was_ambient:
            self.start_ambient()

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

    # -----------------------------------------------------------------
    # Ambient movement
    # -----------------------------------------------------------------

    def start_ambient(self):
        """Start ambient movement background process (respects user preference)."""
        if self.live_settings.get("ambient_movement", "on") == "off":
            self.get_logger().info("[Ambient] Skipped — user set ambient movement to off")
            return
        self._ambient_enabled = True
        if self._ambient_proc is not None and self._ambient_proc.poll() is None:
            return  # already running
        self._ambient_proc = subprocess.Popen(
            ["python3", os.path.join(CAPABILITIES_DIR, "ambient_movement.py")]
        )
        self.get_logger().info("[Ambient] Started")

    def stop_ambient(self):
        """Stop ambient movement background process."""
        self._ambient_enabled = False
        if self._ambient_proc is None or self._ambient_proc.poll() is not None:
            self._ambient_proc = None
            return
        self._ambient_proc.send_signal(signal.SIGINT)
        try:
            self._ambient_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._ambient_proc.kill()
            self._ambient_proc.wait()
        self._ambient_proc = None
        # Zero out base velocity
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info("[Ambient] Stopped")

    # -----------------------------------------------------------------
    # Capability scripts (subprocess)
    # -----------------------------------------------------------------

    def run_capability(self, script: str, args: list = None, wait: bool = False):
        """Run a capability script from voice-visual-api/capabilities/ as a subprocess.

        Args:
            script: Script filename (e.g. "hello.py")
            args: Extra CLI arguments
            wait: If True, block until done (while still processing events)
        Returns:
            The Popen process object
        """
        cmd = ["python3", os.path.join(CAPABILITIES_DIR, script)]
        if args:
            cmd.extend(args)
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
        """Background thread: waits for Enter key presses."""
        while True:
            try:
                input()
                self._key_event.set()
            except EOFError:
                break

    def wait_for_operator(self, prompt: str = "Press ENTER to continue..."):
        """Block until the operator presses Enter."""
        self._key_event.clear()
        self.get_logger().info(f"[Operator] {prompt}")
        print(f"\n>>> {prompt}")
        while not self._key_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    # -----------------------------------------------------------------
    # Speed scaling
    # -----------------------------------------------------------------

    def set_speed(self, speed: str):
        """Set FUNMAP base_translate_velocity and base_rotate_velocity."""
        vel = FUNMAP_SPEED.get(speed, 0.25)
        self.get_logger().info(f"[Speed] Setting FUNMAP velocities to {vel} m/s (speed={speed})")

        if not self.funmap_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[Speed] /funmap/set_parameters service not available, skipping")
            return

        params = []
        for name in ("base_translate_velocity", "base_rotate_velocity"):
            p = Parameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(vel)
            params.append(p)

        req = SetParameters.Request()
        req.parameters = params
        future = self.funmap_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().info(f"[Speed] FUNMAP velocities set to {vel}")
        else:
            self.get_logger().warn("[Speed] Failed to set FUNMAP velocities")

    # -----------------------------------------------------------------
    # Driver mode switching
    # -----------------------------------------------------------------

    def _switch_mode(self, client, name: str):
        """Call a switch_to_*_mode Trigger service."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"[Mode] {name} service not available")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            self.get_logger().info(f"[Mode] {name}: {future.result().message}")
            return future.result().success
        self.get_logger().warn(f"[Mode] {name} call failed")
        return False

    def switch_to_position_mode(self):
        return self._switch_mode(self.srv_position_mode, "switch_to_position_mode")

    def switch_to_navigation_mode(self):
        return self._switch_mode(self.srv_navigation_mode, "switch_to_navigation_mode")

    # -----------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------

    def navigate_to(
        self,
        room: str,
        speed: str,
        social_distance_m: float = 0.0,
        timeout_s: float = 120.0,
        arrive_dist_m: float = 0.35,
    ) -> bool:
        self.get_logger().info(f"[Nav] -> {room} | speed={speed}")
        self.set_speed(speed)
        self.switch_to_position_mode()
        arrived = self.nav.goto(
            name=room,
            timeout_s=timeout_s,
            arrive_dist_m=arrive_dist_m,
            stand_off_m=social_distance_m,
        )
        self.switch_to_navigation_mode()
        return arrived

    # -----------------------------------------------------------------
    # Demo placeholders
    # -----------------------------------------------------------------

    def desk_demo(self, wipe_count):
        self.get_logger().info(f"[Demo] DESK: cleaning surface ({wipe_count} wipe(s))")
        self.run_capability("desk_demo.py", args=["--wipes", str(wipe_count)], wait=True)

    def bed_demo(self, pillow_position):
        self.get_logger().info(f"[Demo] BED: arranging pillow ({pillow_position})")
        # TODO: use follow_joint_trajectory for pillow arrangement

    def kitchen_demo(self, snack):
        self.get_logger().info(f"[Demo] KITCHEN: fetching snack ({snack})")
        # TODO: navigate to snack, pick up with gripper, deliver

    # -----------------------------------------------------------------
    # Main sequence — event-driven
    # -----------------------------------------------------------------

    def run(self):
        """Execute the full study sequence."""
        self.get_logger().info("=" * 55)
        self.get_logger().info("  STRETCH MAIN CONTROLLER v2 (event-driven)")
        self.get_logger().info(f"  Mode: {'manual' if self.manual else 'event-driven'}")
        self.get_logger().info("=" * 55)

        # Wait for TF
        self.get_logger().info("[Init] Waiting for TF...")
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=1.0)

        ss = self.start_section
        if ss > 1:
            self.get_logger().info(f"[Init] Skipping to section {ss}")
            self.start_ambient()

        # =============================================================
        # SESSION START: hello greeting (runs while voice assistant speaks)
        # =============================================================
        if ss <= 1:
            if self.manual:
                self.wait_for_operator("Press ENTER to start hello greeting...")
            else:
                self.wait_for_event("session_start")

            self.run_capability("hello.py", wait=True)
            self.start_ambient()

            # =============================================================
            # PHASE 1: Wait for section 1 complete (general settings)
            # =============================================================
            if self.manual:
                self.wait_for_operator("Section 1 done. Press ENTER to continue...")
            else:
                self.wait_for_event("section_complete")  # section 1

            self.general = self.loader.load_general_settings()
            self.get_logger().info(
                f"[Config] General: personality={self.general.personality}, "
                f"explainability={self.general.explainability_level}"
            )

        # =============================================================
        # PHASE 2: Wait for section 2 (desk configs + demo).
        #          ACTION event triggers navigate_to + demo automatically.
        # =============================================================
        if ss <= 2:
            if self.manual:
                self.wait_for_operator("Section 2 done. Press ENTER...")
            else:
                self.wait_for_event("section_complete")  # section 2

            self.desk = self.loader.load_desk_settings()
            self.get_logger().info(
                f"[Config] Desk: speed={self.desk.movement_speed}, "
                f"cleaning={self.desk.cleaning_thoroughness}"
            )

        # =============================================================
        # PHASE 3: Wait for section 3 (bed configs + demo).
        #          ACTION event triggers navigate_to + demo automatically.
        # =============================================================
        if ss <= 3:
            if self.manual:
                self.wait_for_operator("Section 3 done. Press ENTER...")
            else:
                self.wait_for_event("section_complete")  # section 3

            self.bed = self.loader.load_bed_settings()
            self.get_logger().info(
                f"[Config] Bed: speed={self.bed.movement_speed}, "
                f"pillow={self.bed.pillow_arrangement}"
            )

        # =============================================================
        # PHASE 4: Wait for section 4 (kitchen configs + demo).
        #          ACTION event triggers navigate_to + demo automatically.
        # =============================================================
        if self.manual:
            self.wait_for_operator("Section 4 done. Press ENTER...")
        else:
            self.wait_for_event("section_complete")  # section 4

        self.kitchen = self.loader.load_kitchen_settings()
        self.get_logger().info(
            f"[Config] Kitchen: speed={self.kitchen.movement_speed}, "
            f"snack={self.kitchen.preferred_snack}"
        )

        # =============================================================
        self.get_logger().info("=" * 55)
        self.get_logger().info("  Study sequence complete!")
        self.get_logger().info("=" * 55)


# =============================================================================
# Entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Stretch Main Controller v2 (event-driven)")
    parser.add_argument(
        "--voice-url",
        type=str,
        default="http://192.168.0.23:5050",
        help="URL of the voice assistant Flask server (default: http://192.168.0.23:5050)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5060,
        help="Port for the event receiver Flask server (default: 5060)",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Use manual operator mode (Enter key) instead of event-driven",
    )
    parser.add_argument(
        "--start-section",
        type=int,
        default=1,
        choices=[1, 2, 3, 4],
        help="Skip ahead to this section (default: 1)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = StretchMainController(
        voice_url=args.voice_url,
        flask_port=args.port,
        manual=args.manual,
        start_section=args.start_section,
    )

    # Start event receiver
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
