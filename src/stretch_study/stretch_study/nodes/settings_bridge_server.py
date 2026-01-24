import json
import threading
from typing import Any, Dict

from flask import Flask, jsonify, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def clamp_int(x: Any, lo: int, hi: int, default: int) -> int:
    try:
        v = int(x)
    except Exception:
        return default
    return max(lo, min(hi, v))


def coerce_speed(value: Any) -> str:
    """
    Accepts:
      - "slow" | "medium" | "fast"
      - numeric scale 0.0..1.0 (float/int)
    Returns one of: "slow" | "medium" | "fast"
    """
    if value is None:
        return "medium"

    # string case
    if isinstance(value, str):
        s = value.strip().lower()
        if s in ("slow", "medium", "fast"):
            return s
        if s in ("med", "normal", "default"):
            return "medium"

    # numeric case
    try:
        x = float(value)
        # Map scale -> bucket
        # 0.0..0.45 => slow
        # 0.45..0.8 => medium
        # 0.8..1.0 => fast
        if x < 0.45:
            return "slow"
        if x < 0.8:
            return "medium"
        return "fast"
    except Exception:
        return "medium"


class SettingsBridge(Node):
    """
    Flask -> ROS bridge.
    Kotlin app calls HTTP endpoints; we publish StudyEvents into ROS.

    Implemented:
      - POST /settings/volume { "volume": 0..100 }
      - POST /settings/movement_speed { "speed": "slow"|"medium"|"fast" } OR { "speed": 0.0..1.0 }
    """

    def __init__(self):
        super().__init__("stretch_settings_bridge")

        # Params
        self.declare_parameter("http.host", "0.0.0.0")
        self.declare_parameter("http.port", 5000)
        self.declare_parameter("topics.study_event", "/study_event")
        self.declare_parameter("topics.speech_request", "/speech_request")
        self.declare_parameter("speech.confirm", True)

        self.host = str(self.get_parameter("http.host").value)
        self.port = int(self.get_parameter("http.port").value)
        self.topic_event = str(self.get_parameter("topics.study_event").value)
        self.topic_speech = str(self.get_parameter("topics.speech_request").value)
        self.speech_confirm = bool(self.get_parameter("speech.confirm").value)

        # ROS pubs
        self.pub_event = self.create_publisher(String, self.topic_event, 10)
        self.pub_speech = self.create_publisher(String, self.topic_speech, 10)

        # Flask app
        self.app = Flask(__name__)
        self._setup_routes()

        self.get_logger().info(f"[BRIDGE] Flask will listen on http://{self.host}:{self.port}")

    def _publish_event(self, ev: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(ev, ensure_ascii=False)
        self.pub_event.publish(msg)
        self.get_logger().info(f"[BRIDGE] published event: {msg.data}")

    def _publish_speech(self, text: str, volume: int):
        msg = String()
        msg.data = json.dumps(
            {"text": text, "volume": volume, "rate": 170, "voice": "auto", "interrupt": False},
            ensure_ascii=False,
        )
        self.pub_speech.publish(msg)

    def _setup_routes(self):
        @self.app.get("/health")
        def health():
            return jsonify({"status": "ok"})

        @self.app.post("/settings/volume")
        def set_volume():
            body = request.get_json(silent=True) or {}
            volume = clamp_int(body.get("volume"), 0, 100, default=60)

            # publish set event to StudyEngine
            self._publish_event(
                {"type": "set", "scope": "global", "key": "voice_volume", "value": volume}
            )

            if self.speech_confirm:
                self._publish_speech(f"Volume set to {volume} percent.", volume=volume)

            return jsonify({"ok": True, "volume": volume})

        @self.app.post("/settings/movement_speed")
        def set_movement_speed():
            body = request.get_json(silent=True) or {}
            speed = coerce_speed(body.get("speed"))

            # publish set event to StudyEngine
            # StudyEngine will apply deterministic speed + cmd_vel_scaler if running
            self._publish_event(
                {"type": "set", "scope": "global", "key": "movement_speed", "value": speed}
            )

            if self.speech_confirm:
                # use current speech volume
                self._publish_speech(f"Movement speed set to {speed}.", volume=clamp_int(body.get("volume"), 0, 100, default=60))

            return jsonify({"ok": True, "movement_speed": speed})

    def run_http(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)


def main():
    rclpy.init()
    node = SettingsBridge()

    t = threading.Thread(target=node.run_http, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()