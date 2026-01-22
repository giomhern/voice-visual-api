import json
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stretch_study.capabilities.audio import say_text, set_volume


class SpeechNode(Node):
    """
    Subscribes to /speech_request (std_msgs/String JSON) and speaks via the robot speaker.

    Message format (JSON in String):
      {
        "text": "Hello there!",
        "rate": 170,              # optional
        "voice": "auto",          # optional (pyttsx3/espeak best-effort)
        "gain": 1.0,              # optional local gain (pyttsx3 volume 0..1)
        "volume": 60,             # optional system volume 0..100 (pactl/amixer)
        "interrupt": false        # optional: if true, clears queue before speaking
      }
    """

    def __init__(self):
        super().__init__("stretch_study_speech")

        self.declare_parameter("speech.topic_in", "/speech_request")
        self.declare_parameter("speech.topic_out", "/speech_status")
        self.declare_parameter("speech.default_volume", 60)
        self.declare_parameter("speech.default_rate", 170)
        self.declare_parameter("speech.default_voice", "auto")
        self.declare_parameter("speech.queue_size", 20)

        self.topic_in = self.get_parameter("speech.topic_in").value
        self.topic_out = self.get_parameter("speech.topic_out").value

        self.default_volume = int(self.get_parameter("speech.default_volume").value)
        self.default_rate = int(self.get_parameter("speech.default_rate").value)
        self.default_voice = str(self.get_parameter("speech.default_voice").value)

        qsize = int(self.get_parameter("speech.queue_size").value)
        self._q: queue.Queue[dict] = queue.Queue(maxsize=qsize)

        self.pub_status = self.create_publisher(String, self.topic_out, 10)
        self.sub_req = self.create_subscription(String, self.topic_in, self._on_req, 10)

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        self.get_logger().info(f"SpeechNode ready. Listening on {self.topic_in}")

    def _publish_status(self, status: dict):
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def _on_req(self, msg: String):
        try:
            req = json.loads(msg.data)
        except Exception:
            req = {"text": msg.data}

        text = (req.get("text") or "").strip()
        if not text:
            return

        interrupt = bool(req.get("interrupt", False))
        if interrupt:
            # Clear queue
            while not self._q.empty():
                try:
                    self._q.get_nowait()
                except Exception:
                    break

        # Fill defaults
        req["text"] = text
        req.setdefault("volume", self.default_volume)
        req.setdefault("rate", self.default_rate)
        req.setdefault("voice", self.default_voice)
        req.setdefault("gain", None)

        try:
            self._q.put_nowait(req)
            self.get_logger().info(f"[SPEECH] enqueued: {text[:80]}")
        except queue.Full:
            self.get_logger().warn("[SPEECH] queue full; dropping utterance")
            self._publish_status({"type": "dropped", "reason": "queue_full", "text": text})

    def _loop(self):
        while not self._stop.is_set():
            try:
                req = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            text = req["text"]
            vol = req.get("volume")
            rate = req.get("rate")
            voice = req.get("voice")
            gain = req.get("gain")

            self._publish_status({"type": "start", "text": text})
            self.get_logger().info(f"[SPEECH] speaking: {text[:80]}")

            # Best-effort set system volume
            if vol is not None:
                ok, msg, meta = set_volume(int(vol))
                self.get_logger().info(f"[SPEECH] volume set ok={ok} msg={msg}")
                self._publish_status({"type": "volume", "ok": ok, "msg": msg, "meta": meta, "volume": int(vol)})

            ok, msg, meta = say_text(text, rate=rate, voice=voice, local_gain=gain)
            self._publish_status({"type": "done", "ok": ok, "msg": msg, "meta": meta, "text": text})

            if ok:
                self.get_logger().info("[SPEECH] done")
            else:
                self.get_logger().error(f"[SPEECH] failed: {msg}")

            time.sleep(0.05)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


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