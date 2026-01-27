#!/usr/bin/env python3
import time
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RealtimeSTT import AudioToTextRecorder


class StretchSTTNode(Node):
    """
    Minimal, reliable STT ROS2 node for Stretch.

    - Uses RealtimeSTT in PULL mode (text = recorder.text()) which matches the working test.
    - Publishes recognized speech to /stt_text as std_msgs/String.
    - Logs "[USER] ..." when it hears something.

    This is intentionally "dumb" and stable: no TTS, no LLM, no state machine.
    """

    def __init__(self):
        super().__init__("stretch_stt_node")

        # --- Topics ---
        self.declare_parameter("topics.stt_out", "/stt_text")
        self.stt_out_topic = str(self.get_parameter("topics.stt_out").value)

        # --- STT tuning (safe defaults) ---
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.model", "base")
        self.declare_parameter("stt.compute_type", "float32")

        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.post_speech_silence_duration", 0.6)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)

        # Explicit mic device index (PortAudio). -1 = use default input device.
        self.declare_parameter("stt.input_device", -1)

        # Debug: if True, publish even empty/whitespace transcripts (usually keep False)
        self.declare_parameter("stt.publish_empty", False)

        self.language = str(self.get_parameter("stt.language").value)
        self.model = str(self.get_parameter("stt.model").value)
        self.compute_type = str(self.get_parameter("stt.compute_type").value)

        self.webrtc_sensitivity = int(self.get_parameter("stt.webrtc_sensitivity").value)
        self.silero_sensitivity = float(self.get_parameter("stt.silero_sensitivity").value)
        self.post_speech_silence_duration = float(
            self.get_parameter("stt.post_speech_silence_duration").value
        )
        self.min_length_of_recording = float(self.get_parameter("stt.min_length_of_recording").value)
        self.min_gap_between_recordings = float(self.get_parameter("stt.min_gap_between_recordings").value)

        self.input_device = int(self.get_parameter("stt.input_device").value)
        self.publish_empty = bool(self.get_parameter("stt.publish_empty").value)

        # Publisher
        self.pub_stt = self.create_publisher(String, self.stt_out_topic, 10)

        # Build kwargs for RealtimeSTT
        stt_kwargs: Dict[str, Any] = dict(
            language=self.language,
            model=self.model,
            compute_type=self.compute_type,

            # VAD / endpointing knobs
            webrtc_sensitivity=self.webrtc_sensitivity,
            silero_sensitivity=self.silero_sensitivity,
            silero_deactivity_detection=True,
            post_speech_silence_duration=self.post_speech_silence_duration,
            min_length_of_recording=self.min_length_of_recording,
            min_gap_between_recordings=self.min_gap_between_recordings,

            # reduce console noise
            enable_realtime_transcription=False,
            spinner=False,
        )

        # Best-effort across versions: these names differ
        if self.input_device >= 0:
            stt_kwargs["input_device_index"] = self.input_device
            stt_kwargs["device_index"] = self.input_device

        # Create recorder
        self.recorder = AudioToTextRecorder(**stt_kwargs)

        self.get_logger().info(
            "StretchSTTNode ready | "
            f"out={self.stt_out_topic} lang={self.language} model={self.model} "
            f"device={self.input_device} "
            f"silero={self.silero_sensitivity} webrtc={self.webrtc_sensitivity} "
            f"silence={self.post_speech_silence_duration}s"
        )

        # Start listen thread
        self._stop_flag = threading.Event()
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop_flag.set()
        try:
            if hasattr(self.recorder, "stop"):
                self.recorder.stop()
        except Exception:
            pass
        super().destroy_node()

    def _publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.pub_stt.publish(msg)

    def _listen_loop(self):
        self.get_logger().info("[STT] listen loop started")

        # Start recorder once
        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] recorder.start failed: {e}")

        while rclpy.ok() and not self._stop_flag.is_set():
            try:
                # PULL mode: blocks until it thinks an utterance is complete
                text = self.recorder.text()
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"[STT] recorder.text error: {e}")
                time.sleep(0.2)
                continue

            if text is None:
                continue

            raw = str(text)
            cleaned = raw.strip()

            if not cleaned and not self.publish_empty:
                continue

            # Log + publish
            self.get_logger().info(f"[USER] {cleaned if cleaned else repr(raw)}")
            self._publish_text(cleaned)

            time.sleep(0.01)


def main():
    rclpy.init()
    node = StretchSTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()