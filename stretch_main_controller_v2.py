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
import math
import os
import queue
import subprocess
import sys
import threading
import time

import yaml
import rclpy
from flask import Flask, jsonify, request as flask_request
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import Buffer, TransformListener

# Import config loader from same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stretch_config_loader import StretchConfigLoader, ValueMappings


# =============================================================================
# Helpers
# =============================================================================

SPEED_TO_SCALE = {"slow": 0.3, "medium": 1.0, "fast": 1.5}

CAPABILITIES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "voice-visual-api", "capabilities",
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


def yaw_to_quat(yaw: float):
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def apply_stand_off(x, y, yaw, stand_off_m):
    """Offset goal position backwards along its facing direction."""
    if stand_off_m <= 0:
        return x, y
    ox = x - stand_off_m * math.cos(yaw)
    oy = y - stand_off_m * math.sin(yaw)
    return ox, oy


# =============================================================================
# Main Controller Node
# =============================================================================

class StretchMainController(Node):

    def __init__(self, voice_url=None, flask_port=5060, manual=False):
        super().__init__("stretch_main_controller")

        self.voice_url = voice_url.rstrip("/") if voice_url else None
        self.flask_port = flask_port
        self.manual = manual

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
        # --- ROS2 interfaces ---
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- cmd_vel scaler parameter client ---
        self.scaler_client = self.create_client(
            SetParameters, "/cmd_vel_scaler/set_parameters"
        )

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
        elif event_type == "user_confirmed":
            self.run_capability("nod.py")
        elif event_type == "action":
            self._handle_action(ev.get("action", ""))
        else:
            self.get_logger().info(f"[Event] Unhandled: {ev}")

    def _handle_action(self, action: str):
        """Dispatch an ACTION tag to the appropriate demo."""
        if action not in ACTION_MAP:
            self.get_logger().warn(f"[Action] Unknown action: {action}")
            return
        room, param = ACTION_MAP[action]
        self.get_logger().info(f"[Action] Dispatching: {action} -> {room}({param})")
        if room == "desk":
            self.desk_demo(param)
        elif room == "bed":
            self.bed_demo(param)
        elif room == "kitchen":
            self.kitchen_demo(param)

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
        scale = SPEED_TO_SCALE.get(speed, 1.0)
        self.get_logger().info(f"[Speed] Setting cmd_vel_scaler scale={scale}")

        if not self.scaler_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[Speed] cmd_vel_scaler service not available, skipping")
            return

        param = Parameter()
        param.name = "scale"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(scale)

        req = SetParameters.Request()
        req.parameters = [param]
        future = self.scaler_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.get_logger().info(f"[Speed] Scale set to {scale}")
        else:
            self.get_logger().warn("[Speed] Failed to set scale")

    # -----------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------

    def _get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, "base_link", rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def navigate_to(
        self,
        room: str,
        speed: str,
        social_distance_m: float,
        timeout_s: float = 120.0,
        arrive_dist_m: float = 0.35,
    ) -> bool:
        if room not in self.goals:
            self.get_logger().error(f"[Nav] Unknown room: {room}")
            return False

        goal = self.goals[room]
        gx, gy, gyaw = float(goal["x"]), float(goal["y"]), float(goal["yaw"])
        gx, gy = apply_stand_off(gx, gy, gyaw, social_distance_m)

        velocity = ValueMappings.get_velocity(speed)
        self.get_logger().info(
            f"[Nav] -> {room} | speed={speed} ({velocity} m/s) | "
            f"social_distance={social_distance_m:.2f}m | "
            f"target=({gx:.2f}, {gy:.2f})"
        )

        self.set_speed(speed)

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        qx, qy, qz, qw = yaw_to_quat(gyaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)

        start = time.time()
        last_log = 0.0
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            # Process any queued events while navigating
            self.drain_events()
            pos = self._get_robot_xy()
            if pos is None:
                continue
            dist = math.hypot(pos[0] - gx, pos[1] - gy)
            now = time.time()
            if now - last_log >= 3.0:
                last_log = now
                self.get_logger().info(f"[Nav] {room}: distance={dist:.2f}m")
            if dist <= arrive_dist_m:
                self.get_logger().info(f"[Nav] Arrived at {room} (dist={dist:.2f}m)")
                return True

        self.get_logger().warn(f"[Nav] Timeout reaching {room} after {timeout_s:.0f}s")
        return False

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

        # =============================================================
        # SESSION START: hello greeting (runs while voice assistant speaks)
        # =============================================================
        if self.manual:
            self.wait_for_operator("Press ENTER to start hello greeting...")
        else:
            self.wait_for_event("session_start")

        self.run_capability("hello.py")  # non-blocking: robot waves while assistant speaks

        # =============================================================
        # PHASE 1: Wait for section 1 complete → move to desk
        # =============================================================
        if self.manual:
            self.wait_for_operator("Section 1 done. Press ENTER to move to DESK...")
        else:
            self.wait_for_event("section_complete")  # section 1

        self.general = self.loader.load_general_settings()
        self.get_logger().info(
            f"[Config] General: personality={self.general.personality}, "
            f"explainability={self.general.explainability_level}"
        )

        # Move to desk using precise path (blocking — processes events while moving)
        self.run_capability(
            "movement_to_desk.py", args=["--speed", "medium"], wait=True,
        )

        # =============================================================
        # PHASE 2: At desk — voice assistant configures, triggers demo,
        #          then completes section 2 → navigate to bed
        # =============================================================
        if self.manual:
            self.wait_for_operator("At desk. Press ENTER when desk section is done...")
        else:
            # Wait for section 2 completion.
            # ACTION events and setting_confirmed events will be handled
            # automatically by wait_for_event's internal drain loop.
            self.wait_for_event("section_complete")  # section 2

        self.desk = self.loader.load_desk_settings()
        desk_social_m = ValueMappings.get_distance_meters(self.desk.social_distance)
        self.get_logger().info(
            f"[Config] Desk: speed={self.desk.movement_speed}, "
            f"social_distance={self.desk.social_distance} ({desk_social_m:.2f}m), "
            f"cleaning={self.desk.cleaning_thoroughness}"
        )

        # Load bed settings for social distance at destination
        self.bed = self.loader.load_bed_settings()
        bed_social_m = ValueMappings.get_distance_meters(self.bed.social_distance)

        # Navigate to bed using desk speed + bed social distance
        self.navigate_to("bed", self.desk.movement_speed, bed_social_m)

        # =============================================================
        # PHASE 3: At bed — voice assistant configures, triggers demo,
        #          then completes section 3 → navigate to kitchen
        # =============================================================
        if self.manual:
            self.wait_for_operator("At bed. Press ENTER when bed section is done...")
        else:
            self.wait_for_event("section_complete")  # section 3

        self.bed = self.loader.load_bed_settings()
        bed_social_m = ValueMappings.get_distance_meters(self.bed.social_distance)
        self.get_logger().info(
            f"[Config] Bed: speed={self.bed.movement_speed}, "
            f"social_distance={self.bed.social_distance} ({bed_social_m:.2f}m), "
            f"pillow={self.bed.pillow_arrangement}"
        )

        # Load kitchen settings for social distance at destination
        self.kitchen = self.loader.load_kitchen_settings()
        kitchen_social_m = ValueMappings.get_distance_meters(self.kitchen.social_distance)

        # Navigate to kitchen using bed speed + kitchen social distance
        self.navigate_to("kitchen", self.bed.movement_speed, kitchen_social_m)

        # =============================================================
        # PHASE 4: At kitchen — voice assistant configures, triggers demo,
        #          then completes section 4 → done
        # =============================================================
        if self.manual:
            self.wait_for_operator("At kitchen. Press ENTER when kitchen section is done...")
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
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = StretchMainController(
        voice_url=args.voice_url,
        flask_port=args.port,
        manual=args.manual,
    )

    # Start event receiver
    node.start_flask_server()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("[Controller] Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
