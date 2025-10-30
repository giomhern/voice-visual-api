#!/usr/bin/env python3
"""
Minimal Stretch control API (no API key, two-file version)
"""

import os
import subprocess
import threading
import time
from typing import Optional
from flask import Flask, jsonify, request
from flask_cors import CORS
import stretch_body.robot as stretch_robot


# ---------- Robot control ----------
class RobotControl:
    def __init__(self, max_lin: float, max_ang: float, mixer_control: str):
        self.max_lin = float(max_lin)
        self.max_ang = float(max_ang)
        self.mixer_control = mixer_control
        self._lock = threading.Lock()
        self._duration_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        self.robot = stretch_robot.Robot()
        self.robot.startup()

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def set_speed(self, linear: float, angular: float, duration_s: Optional[float] = None):
        linear = self._clamp(linear, -self.max_lin, self.max_lin)
        angular = self._clamp(angular, -self.max_ang, self.max_ang)

        self._stop_flag.set()
        if self._duration_thread and self._duration_thread.is_alive():
            self._duration_thread.join(timeout=0.2)
        self._stop_flag.clear()

        with self._lock:
            self.robot.base.set_velocity(linear, angular)
            self.robot.push_command()

        if duration_s and duration_s > 0:
            self._duration_thread = threading.Thread(
                target=self._auto_stop_after, args=(duration_s,), daemon=True
            )
            self._duration_thread.start()
        return True

    def _auto_stop_after(self, secs: float):
        time.sleep(secs)
        self.stop()

    def stop(self):
        self._stop_flag.set()
        with self._lock:
            self.robot.base.stop()
            self.robot.push_command()
        return True

    def set_volume(self, level: int):
        level = int(self._clamp(level, 0, 100))
        try:
            subprocess.run(
                ["amixer", "sset", self.mixer_control, f"{level}%"],
                check=False,
                capture_output=True,
            )
            return True
        except Exception:
            return False

    def shutdown(self):
        self._stop_flag.set()
        try:
            with self._lock:
                self.robot.stop()
                self.robot.shutdown()
        except Exception:
            pass


# ---------- Flask app factory ----------
def create_app() -> Flask:
    MAX_LINEAR = float(os.getenv("MAX_LINEAR_SPEED", "0.35"))
    MAX_ANGULAR = float(os.getenv("MAX_ANGULAR_SPEED", "1.0"))
    MIXER_CONTROL = os.getenv("MIXER_CONTROL", "Master")

    app = Flask(__name__)
    CORS(app)
    rc = RobotControl(MAX_LINEAR, MAX_ANGULAR, MIXER_CONTROL)

    def _bad(msg, code=400):
        return jsonify({"ok": False, "error": msg}), code

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({"ok": True})

    @app.route("/status", methods=["GET"])
    def status():
        return jsonify(
            {"ok": True, "limits": {"linear": MAX_LINEAR, "angular": MAX_ANGULAR}, "mixer": MIXER_CONTROL}
        )

    @app.route("/motion/speed", methods=["POST"])
    def motion_speed():
        data = request.get_json(silent=True) or {}
        try:
            linear = float(data.get("linear", 0.0))
            angular = float(data.get("angular", 0.0))
        except Exception:
            return _bad("Invalid 'linear' or 'angular'")
        duration = data.get("duration")
        if duration is not None:
            try:
                duration = float(duration)
            except Exception:
                return _bad("Invalid 'duration'")
        rc.set_speed(linear, angular, duration)
        return jsonify({"ok": True})

    @app.route("/motion/stop", methods=["POST"])
    def motion_stop():
        rc.stop()
        return jsonify({"ok": True})

    @app.route("/audio/volume", methods=["POST"])
    def audio_volume():
        data = request.get_json(silent=True) or {}
        try:
            level = int(data["level"])
        except Exception:
            return _bad("Missing or invalid 'level'")
        if not (0 <= level <= 100):
            return _bad("'level' must be 0..100")
        ok = rc.set_volume(level)
        return jsonify({"ok": bool(ok)})

    @app.teardown_appcontext
    def _on_teardown(exc):
        rc.shutdown()

    return app