#!/usr/bin/env python3
import time
import threading
import inspect
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RealtimeSTT import AudioToTextRecorder


def filter_kwargs_for_callable(fn, kwargs: Dict[str, Any]) -> Dict[str, Any]:
    """
    Keep only kwargs that are accepted by fn(**kwargs).
    Works across different RealtimeSTT versions with different parameter names.
    """
    try:
        sig = inspect.signature(fn)
        accepted = set(sig.parameters.keys())
        return {k: v for k, v in kwargs.items() if k in accepted}
    except Exception:
        # If signature introspection fails, return kwargs unchanged (best-effort)
        return kwargs


class StretchSTTNode(Node):
    """
    Minimal, reliable STT ROS2 node.

    - Pull mode: text = recorder.text()
    - Logs "[USER] ..."
    - Publishes recognized text to /stt_text
    - Filters kwargs by constructor signature to avoid "unexpected keyword" errors
    """

    def __init__(self):
        super().__init__("stretch_stt_node")

        # --- Topics ---
        self.declare_parameter("topics.stt_out", "/stt_text")
        self.stt_out_topic = str(self.get_parameter("topics.stt_out").value)

        # --- STT config ---
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.model", "base")
        self.declare_parameter("stt.compute_type", "float32")

        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.post_speech_silence_duration", 0.6)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)

        # Explicit mic device index (PortAudio). -1 = default
        self.declare_parameter("stt.input_device", -1)

        # publish empty transcripts? (usually False)
        self.declare_parameter("stt.publish_empty", False)

        # Read params
        language = str(self.get_parameter("stt.language").value)
        model = str(self.get_parameter("stt.model").value)
        compute_type = str(self.get_parameter("stt.compute_type").value)

        webrtc_sensitivity = int(self.get_parameter("stt.webrtc_sensitivity").value)
        silero_sensitivity = float(self.get_parameter("stt.silero_sensitivity").value)
        post_speech_silence_duration = float(self.get_parameter("stt.post_speech_silence_duration").value)
        min_length_of_recording = float(self.get_parameter("stt.min_length_of_recording").value)
        min_gap_between_recordings = float(self.get_parameter("stt.min_gap_between_recordings").value)

        self.input_device = int(self.get_parameter("stt.input_device").value)
        self.publish_empty = bool(self.get_parameter("stt.publish_empty").value)

        # Publisher
        self.pub_stt = self.create_publisher(String, self.stt_out_topic, 10)

        # Build a "maximal" kwargs set; we'll filter to what your version supports.
        stt_kwargs: Dict[str, Any] = dict(
            language=language,
            model=model,
            compute_type=compute_type,

            webrtc_sensitivity=webrtc_sensitivity,
            silero_sensitivity=silero_sensitivity,
            silero_deactivity_detection=True,

            post_speech_silence_duration=post_speech_silence_duration,
            min_length_of_recording=min_length_of_recording,
            min_gap_between_recordings=min_gap_between_recordings,

            enable_realtime_transcription=False,
            spinner=False,
        )

        # Device selection: try common names across versions.
        # We'll add both, then filter down to what actually exists.
        if self.input_device >= 0:
            stt_kwargs["input_device_index"] = self.input_device
            stt_kwargs["device_index"] = self.input_device  # some versions use this
            stt_kwargs["input_device"] = self.input_device  # some versions might use this

        # Filter kwargs to match your installed RealtimeSTT version
        filtered_kwargs = filter_kwargs_for_callable(AudioToTextRecorder, stt_kwargs)

        # Log what we ended up using (super helpful for debugging)
        self.get_logger().info(
            f"RealtimeSTT ctor kwargs in use: {sorted(filtered_kwargs.keys())}"
        )

        # Create recorder
        self.recorder = AudioToTextRecorder(**filtered_kwargs)

        self.get_logger().info(
            "StretchSTTNode ready | "
            f"out={self.stt_out_topic} device={self.input_device}"
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

        # Start once if available
        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] recorder.start failed: {e}")

        while rclpy.ok() and not self._stop_flag.is_set():
            try:
                text = self.recorder.text()  # blocks until utterance completes
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