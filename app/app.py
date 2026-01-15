#!/usr/bin/env python3
"""
Flask API for Stretch
- Audio / TTS / Gemini intent parsing
- Direct robot control (RobotManager)
- Study control via ROS event publishing (/study_event)

IMPORTANT:
  - Flask never modifies study state directly
  - Study FSM lives in stretch_study_controller.py
"""

import json
import os

from flask import Flask, request, jsonify
from flask_cors import CORS

# ---- Audio / Robot / LLM ----
from audio import set_volume, say_text, list_voices
from robot_control import RobotManager
from gemini import transcribe_audio, parse_intent

# ---- ROS event bridge ----
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =========================================================
# ROS → Study Event Publisher
# =========================================================

class StudyEventPublisher(Node):
    def __init__(self):
        super().__init__("study_event_publisher")
        self.pub = self.create_publisher(String, "study_event", 10)

    def send(self, event: dict):
        msg = String()
        msg.data = json.dumps(event)
        self.pub.publish(msg)


# Initialize ROS once (no spinning required)
rclpy.init(args=None)
ros_node = StudyEventPublisher()


# =========================================================
# Flask app setup
# =========================================================

app = Flask(__name__)
CORS(app)

robot = RobotManager()


# =========================================================
# Health
# =========================================================

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"}), 200


# =========================================================
# Audio / TTS
# =========================================================

@app.route("/settings/volume", methods=["POST"])
def api_set_volume():
    data = request.get_json(silent=True) or {}
    level = data.get("level")

    try:
        level = int(level)
        assert 0 <= level <= 100
    except Exception:
        return jsonify({"ok": False, "error": "Volume must be integer 0–100"}), 400

    ok, msg, meta = set_volume(level)
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/tts/say", methods=["POST"])
def api_tts_say():
    data = request.get_json(silent=True) or {}
    text = (data.get("text") or "").strip()
    if not text:
        return jsonify({"ok": False, "error": "Missing 'text'"}), 400

    ok, msg, meta = say_text(
        text=text,
        rate=data.get("rate"),
        voice=data.get("voice"),
        local_gain=data.get("volume"),
    )
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/tts/voices", methods=["GET"])
def api_voices():
    return jsonify(list_voices()), 200


# =========================================================
# Robot lifecycle + motion (direct control)
# =========================================================

@app.route("/robot/start", methods=["POST"])
def robot_start():
    ok, msg, meta = robot.startup()
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/robot/stop", methods=["POST"])
def robot_stop():
    ok, msg, meta = robot.shutdown()
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/robot/home", methods=["POST"])
def robot_home():
    ok, msg, meta = robot.home()
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/robot/diagnostics", methods=["GET"])
def robot_diag():
    return jsonify(robot.diagnostics()), 200


@app.route("/robot/move", methods=["POST"])
def robot_move():
    data = request.get_json(silent=True) or {}
    distance = data.get("distance_m")
    velocity = data.get("velocity_mps", 0.10)

    if distance is None:
        return jsonify({"ok": False, "error": "Missing distance_m"}), 400

    ok, msg, meta = robot.move_linear(distance_m=distance, velocity_mps=velocity)
    if ok:
        try:
            say_text(f"Moving {abs(float(distance)):.2f} meters.")
        except Exception:
            pass
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/robot/rotate", methods=["POST"])
def robot_rotate():
    data = request.get_json(silent=True) or {}
    angle = data.get("angle_rad")
    rate = data.get("angular_rate_rps", 0.5)

    if angle is None:
        return jsonify({"ok": False, "error": "Missing angle_rad"}), 400

    ok, msg, meta = robot.move_angular(angle, rate)
    if ok:
        try:
            say_text(f"Rotating {abs(float(angle)):.2f} radians.")
        except Exception:
            pass
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


@app.route("/robot/halt", methods=["POST"])
def robot_halt():
    if not hasattr(robot, "halt_motion"):
        return jsonify({"ok": False, "error": "halt_motion not implemented"}), 501
    ok, msg, meta = robot.halt_motion()
    return jsonify({"ok": ok, "message": msg, "meta": meta}), (200 if ok else 500)


# =========================================================
# Study control API (publishes ROS events)
# =========================================================

@app.route("/study/advance", methods=["POST"])
def study_advance():
    ros_node.send({"type": "advance"})
    return jsonify({"ok": True})


@app.route("/study/arrive", methods=["POST"])
def study_arrive():
    data = request.get_json(silent=True) or {}
    room = data.get("room")
    if room not in ("desk", "bed", "kitchen"):
        return jsonify({"ok": False, "error": "Invalid room"}), 400

    ros_node.send({"type": "arrive", "room": room})
    return jsonify({"ok": True})


@app.route("/study/set", methods=["POST"])
def study_set():
    data = request.get_json(silent=True) or {}
    required = {"scope", "key", "value"}
    if not required.issubset(data):
        return jsonify({"ok": False, "error": "Missing fields"}), 400

    event = {
        "type": "set",
        "scope": data["scope"],
        "key": data["key"],
        "value": data["value"],
    }
    if data["scope"] == "room":
        event["room"] = data.get("room")

    ros_node.send(event)
    return jsonify({"ok": True})


@app.route("/study/demo", methods=["POST"])
def study_demo():
    data = request.get_json(silent=True) or {}
    ros_node.send({
        "type": "demo_confirm",
        "room": data.get("room"),
        "yes": bool(data.get("yes", False)),
    })
    return jsonify({"ok": True})


# =========================================================
# Voice command → intent → action / study event
# =========================================================

@app.route("/voice/command", methods=["POST"])
def voice_command():
    # --- get text or audio ---
    if request.content_type and request.content_type.startswith("multipart/form-data"):
        file = request.files.get("audio")
        if not file:
            return jsonify({"ok": False, "error": "Missing audio"}), 400
        audio_bytes = file.read()
        ok, transcript, _ = transcribe_audio(audio_bytes, "audio/wav")
        if not ok:
            return jsonify({"ok": False, "error": transcript}), 500
        text = transcript
    else:
        data = request.get_json(silent=True) or {}
        text = (data.get("text") or "").strip()
        if not text:
            return jsonify({"ok": False, "error": "Missing text"}), 400

    # --- parse intent ---
    ok, intent, meta = parse_intent(text)
    if not ok:
        say_text("Sorry, I didn't understand.")
        return jsonify({"ok": False, "intent": intent}), 400

    action = intent.get("action")
    params = intent.get("params", {})

    # --- study-related intents ---
    if action == "study_advance":
        ros_node.send({"type": "advance"})
        say_text("Okay, let's continue.")
        return jsonify({"ok": True})

    if action == "study_arrive":
        ros_node.send({"type": "arrive", "room": params.get("room")})
        say_text(f"We are now at the {params.get('room')}.")
        return jsonify({"ok": True})

    if action == "study_set":
        ros_node.send({"type": "set", **params})
        say_text("Okay, I've updated that.")
        return jsonify({"ok": True})

    # --- fallback to existing actions ---
    say_text("Sorry, that command isn't supported yet.")
    return jsonify({"ok": False, "intent": intent}), 400


# =========================================================
# Debug helpers
# =========================================================

@app.route("/intent/debug", methods=["POST"])
def intent_debug():
    data = request.get_json(silent=True) or {}
    text = data.get("text", "")
    ok, intent, meta = parse_intent(text)
    return jsonify({"ok": ok, "intent": intent, "meta": meta}), (200 if ok else 500)


# =========================================================
# Main
# =========================================================

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False,
        threaded=False,
    )
