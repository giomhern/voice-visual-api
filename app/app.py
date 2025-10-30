# app.py
from flask import Flask, request, jsonify
from flask_cors import CORS
from audio import set_volume, say_text, list_voices
from robot_control import RobotManager

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

if __name__ == "__main__":
    # run single-process, single-thread, no reloader
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=False)