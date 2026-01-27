#!/usr/bin/env python3
import time
import json
import threading
import inspect
from typing import Any, Dict

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
    STT + (optional) speak-back wiring to SpeechNode.

    Topics:
      Publishes: /stt_text, /speech_request
      Subscribes: /speech_status
    """

    def __init__(self):
        super().__init__("stretch_voice_assistant_min")

        # Topics
        self.declare_parameter("topics.stt_out", "/stt_text")
        self.declare_parameter("topics.speech_out", "/speech_request")
        self.declare_parameter("topics.speech_status_in", "/speech_status")

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

        # Turn-taking / echo safety
        self.declare_parameter("assistant.listen_cooldown_s", 0.8)
        self.declare_parameter("assistant.debug_log", True)

        # Speak-back test mode (set True for quick end-to-end verification)
        self.declare_parameter("assistant.parrot_mode", True)

        # TTS defaults (used when publishing /speech_request)
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.voice", "af_heart")
        self.declare_parameter("speech.interrupt", True)

        self.stt_out_topic = str(self.get_parameter("topics.stt_out").value)
        self.speech_topic = str(self.get_parameter("topics.speech_out").value)
        self.speech_status_topic = str(self.get_parameter("topics.speech_status_in").value)

        self.listen_cooldown_s = float(self.get_parameter("assistant.listen_cooldown_s").value)
        self.debug_log = bool(self.get_parameter("assistant.debug_log").value)
        self.parrot_mode = bool(self.get_parameter("assistant.parrot_mode").value)

        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)
        self.speech_interrupt = bool(self.get_parameter("speech.interrupt").value)

        self._speech_active = False
        self._mute_until = 0.0
        self._busy_lock = threading.Lock()

        # Pub/Sub
        self.pub_stt = self.create_publisher(String, self.stt_out_topic, 10)
        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)
        self.sub_status = self.create_subscription(String, self.speech_status_topic, self._on_speech_status, 10)

        # RealtimeSTT kwargs (filter to your installed version)
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
            # only some versions support these; we’ll filter after
            stt_kwargs["input_device_index"] = input_device
            stt_kwargs["input_device"] = input_device
            stt_kwargs["device_index"] = input_device

        filtered_kwargs = filter_kwargs_for_callable(AudioToTextRecorder, stt_kwargs)
        self.get_logger().info(f"RealtimeSTT ctor kwargs: {sorted(filtered_kwargs.keys())}")
        self.recorder = AudioToTextRecorder(**filtered_kwargs)

        self.get_logger().info(
            f"VoiceAssistant ready | stt_out={self.stt_out_topic} speech_out={self.speech_topic} "
            f"speech_status_in={self.speech_status_topic} parrot_mode={self.parrot_mode}"
        )

        # Start STT thread
        self._stop_flag = threading.Event()
        threading.Thread(target=self._listen_loop, daemon=True).start()

    def destroy_node(self):
        self._stop_flag.set()
        try:
            if hasattr(self.recorder, "stop"):
                self.recorder.stop()
        except Exception:
            pass
        super().destroy_node()

    def _on_speech_status(self, msg: String):
        state = (msg.data or "").strip().lower()
        if state == "start":
            self._speech_active = True
            # gate everything while speaking + a small head start
            self._mute_until = time.time() + 0.2
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech start")
        elif state == "done":
            self._speech_active = False
            # gate a bit after speaking to avoid tail echo
            self._mute_until = time.time() + self.listen_cooldown_s
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech done (cooldown)")

    def _publish_stt(self, text: str):
        m = String()
        m.data = text
        self.pub_stt.publish(m)

    def _speak(self, text: str):
        text = (text or "").strip()
        if not text:
            return
        payload = {
            "text": text,
            "volume": self.speech_volume,
            "voice": self.speech_voice,
            "interrupt": self.speech_interrupt,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_speech.publish(msg)
        if self.debug_log:
            self.get_logger().info(f"[SPEECH_OUT] {text[:120]}")

    def _listen_loop(self):
        self.get_logger().info("[STT] listen loop started")

        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] recorder.start failed: {e}")

        while rclpy.ok() and not self._stop_flag.is_set():
            # If TTS is active, don’t even block in text()
            if self._speech_active:
                time.sleep(0.05)
                continue

            try:
                text = self.recorder.text()  # blocks until utterance completes
            except Exception as e:
                self.get_logger().error(f"[STT] recorder.text error: {e}")
                time.sleep(0.2)
                continue

            if text is None:
                continue

            cleaned = str(text).strip()
            if not cleaned:
                continue

            # Gate during cooldown window
            if time.time() < self._mute_until:
                if self.debug_log:
                    self.get_logger().info(f"[STT] gated during cooldown: {cleaned[:60]}")
                continue

            # One at a time
            if not self._busy_lock.acquire(blocking=False):
                continue

            try:
                self.get_logger().info(f"[USER] {cleaned}")
                self._publish_stt(cleaned)

                # Parrot mode: speak back what we heard (for end-to-end test)
                if self.parrot_mode:
                    self._speak(f"I heard: {cleaned}")

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