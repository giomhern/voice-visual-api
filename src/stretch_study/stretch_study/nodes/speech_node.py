#!/usr/bin/env python3
import json
import threading
from queue import Queue, Empty
from typing import Any, Dict, Optional
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Kokoro + playback
import numpy as np
import sounddevice as sd
from kokoro import KPipeline


def clamp_int(x: Any, lo: int, hi: int, default: int) -> int:
    try:
        v = int(x)
    except Exception:
        return default
    return max(lo, min(hi, v))


class SpeechNode(Node):
    """
    Subscribes to /speech_request (std_msgs/String containing JSON):
      {
        "text": "hello",
        "volume": 0..100,
        "rate": 170,              # accepted but not strongly supported by kokoro (see note)
        "voice": "af_heart",
        "interrupt": true|false
      }

    Uses Kokoro TTS and plays audio through sounddevice.
    """

    def __init__(self):
        super().__init__("stretch_study_speech")

        # Topics
        self.declare_parameter("topics.speech_in", "/speech_request")

        # Kokoro defaults
        self.declare_parameter("kokoro.lang_code", "a")  # Kokoro uses lang codes like "a"
        self.declare_parameter("kokoro.default_voice", "af_heart")
        self.declare_parameter("kokoro.sample_rate", 24000)

        # Audio output device
        # If you want a specific device, set audio.device to an int device index from sd.query_devices()
        self.declare_parameter("audio.device", -1)

        self.topic_in = str(self.get_parameter("topics.speech_in").value)
        self.lang_code = str(self.get_parameter("kokoro.lang_code").value)
        self.default_voice = str(self.get_parameter("kokoro.default_voice").value)
        self.sample_rate = int(self.get_parameter("kokoro.sample_rate").value)
        self.audio_device = int(self.get_parameter("audio.device").value)

        # Load pipeline once
        self.get_logger().info("[SPEECH] Loading Kokoro pipeline...")
        self.pipeline = KPipeline(lang_code=self.lang_code)
        self.get_logger().info("[SPEECH] Kokoro pipeline loaded.")

        # Playback worker queue
        self.q: Queue[Dict[str, Any]] = Queue()
        self._stop_flag = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.pub_status = self.create_publisher(String, "/speech_status", 10)

        self.sub = self.create_subscription(String, self.topic_in, self._on_msg, 10)
        self.get_logger().info(f"[SPEECH] Ready. Subscribed to {self.topic_in}")

    def destroy_node(self):
        self._stop_flag.set()
        try:
            sd.stop()
        except Exception:
            pass
        super().destroy_node()

    def _publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.pub_status.publish(msg)

    def _on_msg(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[SPEECH] Bad JSON on {self.topic_in}: {e}")
            return

        text = str(payload.get("text", "")).strip()
        if not text:
            return

        volume = clamp_int(payload.get("volume", 60), 0, 100, 60)

        voice = str(payload.get("voice", self.default_voice)).strip() or self.default_voice
        if voice.lower() == "auto":
            voice = self.default_voice

        interrupt = bool(payload.get("interrupt", False))

        if interrupt:
            try:
                sd.stop()
            except Exception:
                pass
            self._drain_queue()

        self.q.put({"text": text, "volume": volume, "voice": voice})
        self.get_logger().info(f"[SPEECH] queued voice={voice} vol={volume} text='{text[:60]}'")

    def _drain_queue(self):
        try:
            while True:
                self.q.get_nowait()
        except Empty:
            pass

    def _worker_loop(self):
        while not self._stop_flag.is_set() and rclpy.ok():
            try:
                job = self.q.get(timeout=0.1)
            except Empty:
                continue

            try:
                self._publish_status("start")
                self._speak_kokoro(job["text"], voice=job["voice"], volume=job["volume"])
            finally:
                # Always publish done (even if playback errors) so listener can resume
                self._publish_status("done")
    def _speak_kokoro(self, text: str, voice: str, volume: int):
        # Generate audio chunks
        gen = self.pipeline(text, voice=voice)

        vol = float(volume) / 100.0
        vol = max(0.0, min(1.0, vol))

        if self.audio_device >= 0:
            # Keep input device as-is, set only output device
            current_in, current_out = sd.default.device
            sd.default.device = (current_in, self.audio_device)

        chunk_count = 0
        for _, _, audio in gen:
            chunk_count += 1
            # audio is typically a numpy array float32
            a = np.asarray(audio, dtype=np.float32) * vol

            sd.play(a, samplerate=self.sample_rate)
            sd.wait()

        self.get_logger().info(f"[SPEECH] done (chunks={chunk_count}) voice={voice}")

def main():
    rclpy.init()
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()