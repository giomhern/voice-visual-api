#!/usr/bin/env python3
import json
import threading
from queue import Queue, Empty
from typing import Any, Dict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
      {"text":"hello","volume":0..100,"voice":"af_heart","interrupt":true|false}

    Publishes /speech_status: "start" and "done"
    """

    def __init__(self):
        super().__init__("stretch_study_speech")

        self.declare_parameter("topics.speech_in", "/speech_request")
        self.declare_parameter("topics.status_out", "/speech_status")

        self.declare_parameter("kokoro.lang_code", "a")
        self.declare_parameter("kokoro.default_voice", "af_heart")
        self.declare_parameter("kokoro.sample_rate", 24000)

        # output device index; -1 = default output
        self.declare_parameter("audio.device", -1)

        self.topic_in = str(self.get_parameter("topics.speech_in").value)
        self.status_out = str(self.get_parameter("topics.status_out").value)

        self.lang_code = str(self.get_parameter("kokoro.lang_code").value)
        self.default_voice = str(self.get_parameter("kokoro.default_voice").value)
        self.sample_rate = int(self.get_parameter("kokoro.sample_rate").value)
        self.audio_device = int(self.get_parameter("audio.device").value)

        self.get_logger().info("[SPEECH] Loading Kokoro pipeline...")
        self.pipeline = KPipeline(lang_code=self.lang_code)
        self.get_logger().info("[SPEECH] Kokoro pipeline loaded.")

        self.pub_status = self.create_publisher(String, self.status_out, 10)
        self.sub = self.create_subscription(String, self.topic_in, self._on_msg, 10)

        self.q: Queue[Dict[str, Any]] = Queue()
        self._stop_flag = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(f"[SPEECH] Ready. Subscribed to {self.topic_in} | publishing {self.status_out}")

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

    def _drain_queue(self):
        try:
            while True:
                self.q.get_nowait()
        except Empty:
            pass

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
        self.get_logger().info(f"[SPEECH] queued voice={voice} vol={volume} text='{text[:80]}'")

    def _worker_loop(self):
        while not self._stop_flag.is_set() and rclpy.ok():
            try:
                job = self.q.get(timeout=0.1)
            except Empty:
                continue

            try:
                self._publish_status("start")
                self._speak_kokoro(job["text"], voice=job["voice"], volume=job["volume"])
            except Exception as e:
                self.get_logger().error(f"[SPEECH] speak error: {e}")
            finally:
                self._publish_status("done")

    def _speak_kokoro(self, text: str, voice: str, volume: int):
        # Output-only device set (do NOT break input device)
        if self.audio_device >= 0:
            try:
                current_in, current_out = sd.default.device
                sd.default.device = (current_in, self.audio_device)
            except Exception:
                pass

        gen = self.pipeline(text, voice=voice)

        vol = max(0.0, min(1.0, float(volume) / 100.0))

        chunk_count = 0
        for _, _, audio in gen:
            chunk_count += 1
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