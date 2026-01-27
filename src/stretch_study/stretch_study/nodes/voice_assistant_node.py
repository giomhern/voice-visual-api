import json
import re
import threading
import time
from typing import Optional, Tuple, List, Dict, Any

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RealtimeSTT import AudioToTextRecorder

EVENTS_BLOCK_RE = re.compile(r"\[EVENTS\](.*?)\[/EVENTS\]", re.DOTALL | re.IGNORECASE)

YES_WORDS = {"yes", "yep", "yeah", "correct", "confirmed", "confirm", "sounds good", "sure", "ok", "okay"}
HALLUCINATION_PHRASES = {"", " ", ".", "...", "um", "uh", "hmm"}


class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__("stretch_voice_assistant")

        # --- Ollama ---
        self.declare_parameter("ollama.url", "http://127.0.0.1:11434/api/chat")
        self.declare_parameter("ollama.model", "llama3.1:8b")
        self.declare_parameter("ollama.timeout_s", 120.0)

        # --- Topics ---
        self.declare_parameter("topics.prompt_in", "/study_prompt")
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.speech_out", "/speech_request")

        # --- Speech defaults (speech_node uses these) ---
        # IMPORTANT: For Kokoro, set to a real voice (NOT "auto")
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "af_heart")

        # --- Assistant behavior ---
        self.declare_parameter("assistant.auto_listen", True)
        self.declare_parameter("assistant.speak_prompts", True)
        self.declare_parameter("assistant.debug_log", True)
        self.declare_parameter("assistant.startup_say", True)

        # NEW: listening control so it resumes cleanly after speaking
        self.declare_parameter("assistant.listen_cooldown_s", 1.0)
        self.declare_parameter("assistant.stt_restart_after_speak", True)

        # --- STT tuning ---
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.post_speech_silence_duration", 0.4)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)

        # Optional: explicitly choose an input device (PortAudio index)
        # If you're using Pulse default source (respeaker_mono), you can leave -1.
        self.declare_parameter("stt.input_device", -1)

        # Read params
        self.ollama_url = str(self.get_parameter("ollama.url").value)
        self.ollama_model = str(self.get_parameter("ollama.model").value)
        self.ollama_timeout_s = float(self.get_parameter("ollama.timeout_s").value)

        self.prompt_topic = str(self.get_parameter("topics.prompt_in").value)
        self.event_topic = str(self.get_parameter("topics.event_out").value)
        self.speech_topic = str(self.get_parameter("topics.speech_out").value)

        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_rate = int(self.get_parameter("speech.rate").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)

        self.auto_listen = bool(self.get_parameter("assistant.auto_listen").value)
        self.speak_prompts = bool(self.get_parameter("assistant.speak_prompts").value)
        self.debug_log = bool(self.get_parameter("assistant.debug_log").value)
        self.startup_say = bool(self.get_parameter("assistant.startup_say").value)

        self.listen_cooldown_s = float(self.get_parameter("assistant.listen_cooldown_s").value)
        self.stt_restart_after_speak = bool(self.get_parameter("assistant.stt_restart_after_speak").value)

        self.stt_input_device = int(self.get_parameter("stt.input_device").value)

        # Pub/Sub
        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)

        self.sub_speech_status = self.create_subscription(
            String, "/speech_status", self._on_speech_status, 10
        )
        self._speech_active = False


        # State
        self.history: List[Dict[str, str]] = []
        self.current_prompt: Optional[str] = None
        self.current_step_id: Optional[str] = None

        self._busy_lock = threading.Lock()
        self._mute_until = 0.0
        self._stt_restart_lock = threading.Lock()

        # STT recorder
        stt_kwargs: Dict[str, Any] = dict(
            language=str(self.get_parameter("stt.language").value),
            silero_sensitivity=float(self.get_parameter("stt.silero_sensitivity").value),
            silero_deactivity_detection=True,
            webrtc_sensitivity=int(self.get_parameter("stt.webrtc_sensitivity").value),
            post_speech_silence_duration=float(self.get_parameter("stt.post_speech_silence_duration").value),
            min_length_of_recording=float(self.get_parameter("stt.min_length_of_recording").value),
            min_gap_between_recordings=float(self.get_parameter("stt.min_gap_between_recordings").value),
            enable_realtime_transcription=False,
            spinner=False,
        )

        # Best-effort: different RealtimeSTT versions use different kwarg names
        if self.stt_input_device >= 0:
            stt_kwargs["input_device_index"] = self.stt_input_device
            stt_kwargs["device_index"] = self.stt_input_device

        self.recorder = AudioToTextRecorder(**stt_kwargs)

        self.get_logger().info(
            "VoiceAssistantNode ready | "
            f"model={self.ollama_model} url={self.ollama_url} "
            f"speak_prompts={self.speak_prompts} auto_listen={self.auto_listen} "
            f"cooldown={self.listen_cooldown_s}s stt_input_device={self.stt_input_device}"
        )

        if self.startup_say:
            self._speak("Voice assistant online.", interrupt=True)

        if self.auto_listen:
            threading.Thread(target=self._listen_loop, daemon=True).start()

    # ----------------- Speech + STT gating -----------------

    def _on_speech_status(self, msg: String):
        state = (msg.data or "").strip().lower()

        if state == "start":
            self._speech_active = True
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech start -> stopping STT")

            # stop/abort STT so it cannot hear itself
            try:
                if hasattr(self.recorder, "abort"):
                    self.recorder.abort()
                if hasattr(self.recorder, "stop"):
                    self.recorder.stop()
            except Exception:
                pass

        elif state == "done":
            self._speech_active = False
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech done -> restarting STT after cooldown")

            # restart after a short cooldown
            def restart():
                time.sleep(self.listen_cooldown_s)
                try:
                    if hasattr(self.recorder, "start"):
                        self.recorder.start()
                except Exception:
                    pass

            threading.Thread(target=restart, daemon=True).start()

    def _after_speak_reset_stt(self):
        """After speaking, wait a cooldown then restart STT to avoid stuck recorder / echo."""
        if not self.stt_restart_after_speak:
            return
        if not self._stt_restart_lock.acquire(blocking=False):
            return

        def worker():
            try:
                time.sleep(self.listen_cooldown_s)
                if hasattr(self.recorder, "abort"):
                    try:
                        self.recorder.abort()
                    except Exception:
                        pass
                if hasattr(self.recorder, "start"):
                    try:
                        self.recorder.start()
                    except Exception:
                        pass
                if self.debug_log:
                    self.get_logger().info("[STT] restarted after speak")
            finally:
                self._stt_restart_lock.release()

        threading.Thread(target=worker, daemon=True).start()

    def _speak(self, text: str, interrupt: bool = False):
        text = (text or "").strip()
        if not text:
            return

        # Prevent kokoro "auto" voice 404
        voice = (self.speech_voice or "").strip()
        if voice.lower() == "auto":
            voice = "af_heart"

        msg = String()
        msg.data = json.dumps(
            {
                "text": text,
                "volume": self.speech_volume,
                "rate": self.speech_rate,
                "voice": voice,
                "interrupt": interrupt,
            },
            ensure_ascii=False,
        )
        self.pub_speech.publish(msg)

        if self.debug_log:
            self.get_logger().info(f"[SPEECH_OUT] {text[:120]}")

    def _emit_event(self, ev: dict):
        msg = String()
        msg.data = json.dumps(ev, ensure_ascii=False)
        self.pub_event.publish(msg)
        if self.debug_log:
            self.get_logger().info(f"[EVENT_OUT] {msg.data}")

    # ----------------- prompt in -----------------

    def _on_prompt(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.current_prompt = str(payload.get("prompt", "")).strip()
            self.current_step_id = str(payload.get("step", "")).strip()
        except Exception:
            self.current_prompt = msg.data.strip()
            self.current_step_id = ""

        if self.debug_log:
            self.get_logger().info(f"[PROMPT_IN] step={self.current_step_id} :: {self.current_prompt}")

        if self.speak_prompts and self.current_prompt:
            self._speak(self.current_prompt, interrupt=True)

    # ----------------- STT loop -----------------

    def _listen_loop(self):
        self.get_logger().info("[STT] listen loop started")

        # Make sure STT is actually running
        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] failed to start recorder: {e}")

        while rclpy.ok():
            # hard gate while robot is speaking
            if self._speech_active or time.time() < self._mute_until:
                time.sleep(0.05)
                continue

            # don't accept new speech while we're processing a turn
            if self._busy_lock.locked():
                time.sleep(0.05)
                continue

            try:
                text = self.recorder.text()  # blocks until an utterance completes
                if text:
                    self._on_transcript(text)
            except Exception as e:
                self.get_logger().error(f"[STT] recorder.text error: {e}")
                time.sleep(0.2)

    def _on_transcript(self, text: str):
        text = (text or "").strip()
        if not text:
            return

        cleaned = text.lower().strip().rstrip(".,!?")
        if cleaned in HALLUCINATION_PHRASES:
            return

        # Ignore anything said while we're within the post-speak cooldown window
        if time.time() < self._mute_until:
            return

        if not self._busy_lock.acquire(blocking=False):
            return

        try:
            if self.debug_log:
                self.get_logger().info(f"[USER] {text}")

            # Let the LLM decide response + events
            raw = self._chat_with_ollama(text)
            spoken, events = self._split_spoken_and_events(raw)

            if spoken.strip():
                self._speak(spoken, interrupt=False)
            else:
                self._speak("Okay.", interrupt=False)

            for ev in events:
                if isinstance(ev, dict):
                    self._emit_event(ev)

            # Minimal heuristics: "yes" advances (optional)
            if cleaned in YES_WORDS or cleaned.startswith("yes "):
                self._emit_event({"type": "advance"})

        finally:
            self._busy_lock.release()

    # ----------------- Ollama -----------------

    def _chat_with_ollama(self, user_text: str) -> str:
        self.history.append({"role": "user", "content": user_text})

        system_prompt = self._build_system_prompt()
        payload = {
            "model": self.ollama_model,
            "messages": [{"role": "system", "content": system_prompt}, *self.history],
            "stream": False,
        }

        try:
            r = requests.post(self.ollama_url, json=payload, timeout=self.ollama_timeout_s)
            r.raise_for_status()
            assistant_text = r.json()["message"]["content"]
        except Exception as e:
            assistant_text = f"I ran into an error talking to Ollama: {e}"

        self.history.append({"role": "assistant", "content": assistant_text})
        return assistant_text

    def _build_system_prompt(self) -> str:
        prompt = self.current_prompt or ""
        step = self.current_step_id or ""

        # Volume-only, step-driven behavior (keeps it predictable)
        return f"""You are Stretch, a friendly home assistant robot running a scripted onboarding study.

CURRENT STEP ID: {step}
CURRENT STEP PROMPT (follow it closely and do not jump ahead):
{prompt}

You must respond with TWO parts:
1) SPOKEN text for the user (1-2 short sentences).
2) An [EVENTS] JSON block: {{"events":[ ... ]}}

IMPORTANT RULES:
- Follow the current prompt only; do not move to future steps unless the user confirms.
- For now, the ONLY setting you may change is voice volume (0-100).
- If the user provides a volume number (0-100), include:
  {{"type":"set","scope":"global","key":"voice_volume","value":<int>}}
  and ask the user to confirm (yes/no).
- If the user says yes/confirm, include:
  {{"type":"advance"}}
- If the user says no, do NOT advance; ask what volume they want instead.
- Always include the [EVENTS] block, even if empty: {{"events":[]}}

Output format MUST be:

<spoken text>
[EVENTS]
{{"events":[ ... ]}}
[/EVENTS]
"""

    def _split_spoken_and_events(self, assistant_text: str) -> Tuple[str, List[Dict[str, Any]]]:
        if not assistant_text:
            return "", []

        m = EVENTS_BLOCK_RE.search(assistant_text)
        if not m:
            return assistant_text.strip(), []

        events_raw = m.group(1).strip()
        spoken = assistant_text[: m.start()].strip()

        events: List[Dict[str, Any]] = []
        try:
            obj = json.loads(events_raw)
            if isinstance(obj, dict) and isinstance(obj.get("events"), list):
                events = [e for e in obj["events"] if isinstance(e, dict)]
        except Exception as e:
            if self.debug_log:
                self.get_logger().error(f"[EVENTS_PARSE] failed: {e} raw={events_raw[:200]}")
        return spoken, events


def main():
    rclpy.init()
    node = VoiceAssistantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()