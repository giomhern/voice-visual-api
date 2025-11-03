# app.py
from flask import Flask, request, jsonify
from flask_cors import CORS
from audio import set_volume, say_text, list_voices
from robot_control import RobotManager
from gemini import transcribe_audio, parse_intent

app = Flask(__name__)
CORS(app)

robot = RobotManager()

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"}), 200

@app.route("/settings/volume", methods=["POST"])
def api_set_volume():
    data = request.get_json(silent=True) or {}
    level = data.get("level")
    if level is None:
        return jsonify({"error": "Missing 'level' (0-100)."}), 400
    try:
        level = int(level)
        if not (0 <= level <= 100):
            raise ValueError
    except Exception:
        return jsonify({"error": "Volume 'level' must be an integer 0-100."}), 400

    ok, msg, meta = set_volume(level)
    return (jsonify({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}), 200 if ok else 500)

@app.route("/tts/say", methods=["POST"])
def api_tts_say():
    data = request.get_json(silent=True) or {}
    text = (data.get("text") or "").strip()
    if not text:
        return jsonify({"error": "Missing or empty 'text'."}), 400

    rate = data.get("rate")          # int-like (words/min for pyttsx3, -s for espeak)
    voice = data.get("voice")        # substring to match
    local_gain = data.get("volume")  # 0.0-1.0 for pyttsx3 only

    ok, msg, meta = say_text(text=text, rate=rate, voice=voice, local_gain=local_gain)
    return (jsonify({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}), 200 if ok else 500)

@app.route("/tts/voices", methods=["GET"])
def api_voices():
    return jsonify(list_voices()), 200


@app.route("/robot/start", methods=["POST"])
def robot_start():
    ok, msg, meta = robot.startup()
    return ({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}, 200 if ok else 500)

@app.route("/robot/stop", methods=["POST"])
def robot_stop():
    ok, msg, meta = robot.shutdown()
    return ({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}, 200 if ok else 500)

# Optional (motion): uncomment home() in RobotManager first
@app.route("/robot/home", methods=["POST"])
def robot_home():
    ok, msg, meta = robot.home()
    return ({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}, 200 if ok else 500)

@app.route("/robot/diagnostics", methods=["GET"])
def robot_diag():
    return jsonify(robot.diagnostics()), 200


@app.route("/voice/command", methods=["POST"])
def voice_command():
    """
    Accepts either:
      - JSON: {"text": "..."}  OR
      - multipart/form-data with file field "audio" and optional "mime_type"

    Pipeline:
      audio -> Gemini transcription -> intent parse -> execute -> speak confirmation
      text  -> intent parse -> execute -> speak confirmation
    """
    # 1) Get text or audio
    if request.content_type and request.content_type.startswith("multipart/form-data"):
        # Audio path
        file = request.files.get("audio")
        mime_type = request.form.get("mime_type", "audio/wav")
        if not file:
            return jsonify({"ok": False, "error": "No 'audio' file uploaded."}), 400
        audio_bytes = file.read()
        ok_t, transcript, meta_t = transcribe_audio(audio_bytes, mime_type)
        if not ok_t:
            return jsonify({"ok": False, "error": transcript, "stage": "transcription"}), 500
        user_text = transcript
    else:
        # JSON path
        data = request.get_json(silent=True) or {}
        user_text = (data.get("text") or "").strip()
        if not user_text:
            return jsonify({"ok": False, "error": "Provide 'text' or upload 'audio'."}), 400

    # 2) Parse intent with Gemini
    ok_p, intent, meta_p = parse_intent(user_text)
    if not ok_p:
        say_text(f"Sorry, I couldn't understand that.")
        return jsonify({"ok": False, "error": intent.get("error", "parse failed"), "transcript": user_text}), 400

    action = intent.get("action")
    params = intent.get("params", {})

    # 3) Execute intent
    #    (you can expand this mapping over time)
    if action == "set_volume":
        level = params.get("level")
        try:
            level = int(level)
        except Exception:
            say_text("I couldn't read the volume level.")
            return jsonify({"ok": False, "error": "Invalid volume level", "intent": intent}), 400
        if not (0 <= level <= 100):
            say_text("Volume must be between zero and one hundred.")
            return jsonify({"ok": False, "error": "Level out of range", "intent": intent}), 400
        ok, msg, meta = set_volume(level)
        if ok:
            say_text(f"Okay. Volume set to {level} percent.")
            return jsonify({"ok": True, "action": action, "message": msg, "params": {"level": level}}), 200
        else:
            say_text("Sorry, I could not change the volume.")
            return jsonify({"ok": False, "action": action, "error": msg}), 500

    elif action == "speak":
        text = (params.get("text") or "").strip()
        if not text:
            say_text("I didn't catch what to say.")
            return jsonify({"ok": False, "error": "Missing text for speak", "intent": intent}), 400
        ok, msg, meta = say_text(text)
        return jsonify({"ok": ok, "action": action, "message": msg, "spoken": text}), (200 if ok else 500)

    elif action == "robot_start":
        ok, msg, meta = robot.startup()
        if ok:
            say_text("Robot started.")
        else:
            say_text("Sorry, I couldn't start the robot.")
        return jsonify({"ok": ok, "action": action, "message": msg, "meta": meta}), (200 if ok else 500)

    elif action == "robot_stop":
        ok, msg, meta = robot.shutdown()
        if ok:
            say_text("Robot stopped.")
        else:
            say_text("Sorry, I couldn't stop the robot.")
        return jsonify({"ok": ok, "action": action, "message": msg, "meta": meta}), (200 if ok else 500)

    else:
        say_text("Sorry, that command isn't supported yet.")
        return jsonify({"ok": False, "error": f"Unknown action '{action}'", "intent": intent}), 400

@app.route("/models", methods=["GET"])
def models():
    import google.generativeai as genai, os
    try:
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        data = []
        for m in genai.list_models():
            data.append({
                "name": m.name,
                "id": m.name.split("/")[-1],
                "methods": getattr(m, "supported_generation_methods", []),
            })
        return jsonify({"ok": True, "models": data}), 200
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

# app.py
@app.route("/intent/debug", methods=["POST"])
def intent_debug():
    data = request.get_json(silent=True) or {}
    text = (data.get("text") or "").strip()
    if not text:
        return jsonify({"ok": False, "error": "Missing text"}), 400
    from gemini import parse_intent
    ok, intent, meta = parse_intent(text)
    # include meta.raw if present to inspect
    return jsonify({"ok": ok, "intent": intent, "meta": meta}), 200 if ok else 500


# Move forward/backward by distance (meters)
@app.route("/robot/move", methods=["POST"])
def robot_move():
    data = request.get_json(silent=True) or {}
    distance_m = data.get("distance_m", None)
    velocity_mps = data.get("velocity_mps", 0.10)  # optional

    if distance_m is None:
        return jsonify({"ok": False, "error": "Missing distance_m (meters). Positive=forward, negative=backward."}), 400

    ok, msg, meta = robot.move_linear(distance_m=distance_m, velocity_mps=velocity_mps)
    if ok:
        # optional audible confirmation
        try:
            say_text(f"Moving {abs(float(distance_m)):.2f} meters {'forward' if float(distance_m) >= 0 else 'backward'}.")
        except Exception:
            pass
    return jsonify({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}), (200 if ok else 500)


# (Optional) emergency halt
@app.route("/robot/halt", methods=["POST"])
def robot_halt():
    if not hasattr(robot, "halt_motion"):
        return jsonify({"ok": False, "error": "halt_motion not implemented"}), 501
    ok, msg, meta = robot.halt_motion()
    return jsonify({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}), (200 if ok else 500)

@app.route("/robot/rotate", methods=["POST"])
def robot_rotate():
    data = request.get_json(silent=True) or {}
    angle_rad = data.get("angle_rad")
    rate = data.get("angular_rate_rps", 0.5)
    if angle_rad is None:
        return jsonify({"ok": False, "error": "Missing angle_rad (radians). Positive=CCW, negative=CW."}), 400
    ok, msg, meta = robot.move_angular(angle_rad, rate)
    if ok:
        try:
            say_text(f"Rotating {abs(float(angle_rad)):.2f} radians.")
        except Exception:
            pass
    return jsonify({"ok": ok, "message": msg, **({"meta": meta} if meta else {})}), (200 if ok else 500)


if __name__ == "__main__":
    # run single-process, single-thread, no reloader
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=False)