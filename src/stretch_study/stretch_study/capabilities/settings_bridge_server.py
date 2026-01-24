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


class SettingsBridge(Node):
    """
    Flask -> ROS bridge.
    Kotlin app calls HTTP endpoints; we publish StudyEvents into ROS.

    Current implemented capability:
      - POST /settings/volume { "volume": 0..100 }
        -> publishes /study_event set global voice_volume
        -> (optional) publishes /speech_request to confirm
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

            # 1) publish set event to StudyEngine (logged + stored)
            self._publish_event(
                {"type": "set", "scope": "global", "key": "voice_volume", "value": volume}
            )

            # 2) optional spoken confirmation (through speech_node)
            if self.speech_confirm:
                self._publish_speech(f"Volume set to {volume} percent.", volume=volume)

            return jsonify({"ok": True, "volume": volume})

    def run_http(self):
        # Flask must run in a thread so rclpy can spin too
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