#!/usr/bin/env python3
import time
import json
import threading
import inspect
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RealtimeSTT import AudioToTextRecorder


def filter_kwargs_for_callable(fn, kwargs: Dict[str, Any]) -> Dict[str, Any]:
    try:
        sig = inspect.signature(fn)
        accepted = set(sig.parameters.keys())
        return {k: v for k, v in kwargs.items() if k in accepted}
    except Exception:
        return kwargs


class StretchVoiceAssistant(Node):
    """
    Echo-safe STT node that:
      - listens via RealtimeSTT (pull mode: recorder.text())
      - publishes user transcripts to /stt_text (optional)
      - converts user speech to events (here: minimal stub; you can plug your LLM later)
      - subscribes to /speech_status and /speech_request to avoid self-echo
    """

    def __init__(self):
        super().__init__("stretch_voice_assistant")

        # Topics
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.stt_out", "/stt_text")
        self.declare_parameter("topics.speech_status_in", "/speech_status")
        self.declare_parameter("topics.speech_request_in", "/speech_request")

        # STT settings
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.model", "base")
        self.declare_parameter("stt.compute_type", "float32")
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.post_speech_silence_duration", 0.6)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)
        self.declare_parameter("stt.input_device", -1)

        # Turn-taking
        self.declare_parameter("assistant.listen_cooldown_s", 1.0)
        self.declare_parameter("assistant.debug_log", True)

        # Echo filtering strictness
        self.declare_parameter("assistant.echo_window_s", 3.0)  # how long after TTS to filter similar text

        self.event_topic = str(self.get_parameter("topics.event_out").value)
        self.stt_out_topic = str(self.get_parameter("topics.stt_out").value)
        self.speech_status_topic = str(self.get_parameter("topics.speech_status_in").value)
        self.speech_request_topic = str(self.get_parameter("topics.speech_request_in").value)

        self.listen_cooldown_s = float(self.get_parameter("assistant.listen_cooldown_s").value)
        self.debug_log = bool(self.get_parameter("assistant.debug_log").value)
        self.echo_window_s = float(self.get_parameter("assistant.echo_window_s").value)

        # State
        self._speech_active = False
        self._mute_until = 0.0
        self._busy_lock = threading.Lock()
        self._stop_flag = threading.Event()

        # Track what robot just said (from StudyEngine /speech_request)
        self._last_tts_text: str = ""
        self._last_tts_time: float = 0.0

        # Publishers
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.pub_stt = self.create_publisher(String, self.stt_out_topic, 10)

        # Subscribers
        self.sub_status = self.create_subscription(String, self.speech_status_topic, self._on_speech_status, 10)
        self.sub_speech_req = self.create_subscription(String, self.speech_request_topic, self._on_speech_request, 10)

        # Build RealtimeSTT kwargs
        stt_kwargs: Dict[str, Any] = dict(
            language=str(self.get_parameter("stt.language").value),
            model=str(self.get_parameter("stt.model").value),
            compute_type=str(self.get_parameter("stt.compute_type").value),
            webrtc_sensitivity=int(self.get_parameter("stt.webrtc_sensitivity").value),
            silero_sensitivity=float(self.get_parameter("stt.silero_sensitivity").value),
            silero_deactivity_detection=True,
            post_speech_silence_duration=float(self.get_parameter("stt.post_speech_silence_duration").value),
            min_length_of_recording=float(self.get_parameter("stt.min_length_of_recording").value),
            min_gap_between_recordings=float(self.get_parameter("stt.min_gap_between_recordings").value),
            enable_realtime_transcription=False,
            spinner=False,
        )

        input_device = int(self.get_parameter("stt.input_device").value)
        if input_device >= 0:
            # some versions accept one of these; we filter after
            stt_kwargs["input_device_index"] = input_device
            stt_kwargs["input_device"] = input_device
            stt_kwargs["device_index"] = input_device

        filtered_kwargs = filter_kwargs_for_callable(AudioToTextRecorder, stt_kwargs)
        self.get_logger().info(f"RealtimeSTT ctor kwargs: {sorted(filtered_kwargs.keys())}")
        self.recorder = AudioToTextRecorder(**filtered_kwargs)

        self.get_logger().info(
            f"VoiceAssistant ready | event_out={self.event_topic} stt_out={self.stt_out_topic} "
            f"speech_status_in={self.speech_status_topic} speech_request_in={self.speech_request_topic} "
            f"cooldown={self.listen_cooldown_s}s"
        )

        threading.Thread(target=self._listen_loop, daemon=True).start()

    def destroy_node(self):
        self._stop_flag.set()
        try:
            if hasattr(self.recorder, "stop"):
                self.recorder.stop()
        except Exception:
            pass
        super().destroy_node()

    # -----------------------------
    # Echo inputs: what robot says
    # -----------------------------
    def _on_speech_request(self, msg: String):
        """
        StudyEngine publishes /speech_request JSON with {"text": "..."}.
        We record it so we can ignore it if the mic picks it up.
        """
        try:
            payload = json.loads(msg.data)
            t = str(payload.get("text", "")).strip()
        except Exception:
            t = ""

        if t:
            self._last_tts_text = t.lower().strip()
            self._last_tts_time = time.time()
            if self.debug_log:
                self.get_logger().info(f"[TTS] captured outgoing text ({len(t)} chars)")

    # -----------------------------
    # Turn-taking: speech status
    # -----------------------------
    def _on_speech_status(self, msg: String):
        state = (msg.data or "").strip().lower()

        if state == "start":
            self._speech_active = True
            # Gate immediately
            self._mute_until = time.time() + 0.2

            # IMPORTANT: interrupt any blocking recorder.text() call
            # so we don't “finish” transcribing the robot's own voice.
            try:
                if hasattr(self.recorder, "abort"):
                    self.recorder.abort()
            except Exception:
                pass

            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech start -> STT gated + abort()")

        elif state == "done":
            self._speech_active = False
            # Gate after speaking (tail echo)
            self._mute_until = time.time() + self.listen_cooldown_s
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech done -> cooldown gating")

    # -----------------------------
    # Publish helpers
    # -----------------------------
    def _publish_stt(self, text: str):
        m = String()
        m.data = text
        self.pub_stt.publish(m)

    def _emit_event(self, ev: Dict[str, Any]):
        m = String()
        m.data = json.dumps(ev, ensure_ascii=False)
        self.pub_event.publish(m)
        if self.debug_log:
            self.get_logger().info(f"[EVENT_OUT] {m.data}")

    # -----------------------------
    # Echo filter
    # -----------------------------
    def _looks_like_echo(self, user_text: str) -> bool:
        now = time.time()
        if not self._last_tts_text:
            return False
        if now - self._last_tts_time > self.echo_window_s:
            return False

        u = user_text.lower().strip().rstrip(".,!?")
        t = self._last_tts_text.lower().strip().rstrip(".,!?")

        # Simple containment (works well in practice)
        if u and (u in t or t in u):
            return True

        # Also ignore if they share a long prefix (common with partial echoes)
        pref = 0
        for a, b in zip(u, t):
            if a == b:
                pref += 1
            else:
                break
        return pref >= 12  # tweakable

    # -----------------------------
    # STT loop
    # -----------------------------
    def _listen_loop(self):
        self.get_logger().info("[STT] listen loop started")

        # Start once
        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] recorder.start failed: {e}")

        while rclpy.ok() and not self._stop_flag.is_set():
            # If robot is currently speaking, don't even block in text()
            if self._speech_active:
                time.sleep(0.05)
                continue

            try:
                text = self.recorder.text()  # blocks until utterance completes
            except Exception:
                # abort() during speech start can throw / break blocking call; that's fine.
                time.sleep(0.05)
                continue

            if not text:
                continue

            cleaned = str(text).strip()
            if not cleaned:
                continue

            # Gate during mute window
            if time.time() < self._mute_until:
                if self.debug_log:
                    self.get_logger().info(f"[STT] gated transcript: {cleaned[:60]}")
                continue

            # Echo filter: ignore robot's own recent TTS text
            if self._looks_like_echo(cleaned):
                if self.debug_log:
                    self.get_logger().info(f"[ECHO] ignored: {cleaned[:60]}")
                continue

            if not self._busy_lock.acquire(blocking=False):
                continue

            try:
                self.get_logger().info(f"[USER] {cleaned}")
                self._publish_stt(cleaned)

                # ---- Minimal event wiring example ----
                # You can replace this with your LLM intent->events later.
                # For now, just emit a "heard" event.
                self._emit_event({"type": "heard", "text": cleaned})

            finally:
                self._busy_lock.release()

            time.sleep(0.01)


def main():
    rclpy.init()
    node = StretchVoiceAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()