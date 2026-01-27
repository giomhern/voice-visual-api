#!/usr/bin/env python3
import time
import json
import threading
import inspect
import re
from typing import Any, Dict, List, Optional, Tuple

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from RealtimeSTT import AudioToTextRecorder

EVENTS_BLOCK_RE = re.compile(r"\[EVENTS\](.*?)\[/EVENTS\]", re.DOTALL | re.IGNORECASE)

YES_WORDS = {"yes", "yep", "yeah", "correct", "confirmed", "confirm", "sure", "ok", "okay"}


def filter_kwargs_for_callable(fn, kwargs: Dict[str, Any]) -> Dict[str, Any]:
    try:
        sig = inspect.signature(fn)
        accepted = set(sig.parameters.keys())
        return {k: v for k, v in kwargs.items() if k in accepted}
    except Exception:
        return kwargs


class StretchVoiceAssistant(Node):
    """
    STT -> Ollama -> [EVENTS] -> /study_event
    (optional) speaks Ollama response via /speech_request

    Also includes echo-safety by watching /speech_status + /speech_request.
    """

    def __init__(self):
        super().__init__("stretch_voice_assistant_ollama")

        # ---------------- Topics ----------------
        self.declare_parameter("topics.prompt_in", "/study_prompt")
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.speech_out", "/speech_request")
        self.declare_parameter("topics.speech_status_in", "/speech_status")
        self.declare_parameter("topics.speech_request_in", "/speech_request")  # to capture last TTS text

        self.prompt_topic = str(self.get_parameter("topics.prompt_in").value)
        self.event_topic = str(self.get_parameter("topics.event_out").value)
        self.speech_topic = str(self.get_parameter("topics.speech_out").value)
        self.speech_status_topic = str(self.get_parameter("topics.speech_status_in").value)
        self.speech_request_topic = str(self.get_parameter("topics.speech_request_in").value)

        # ---------------- Ollama ----------------
        self.declare_parameter("ollama.url", "http://127.0.0.1:11434/api/chat")
        self.declare_parameter("ollama.model", "qwen2.5:7b-instruct")
        self.declare_parameter("ollama.timeout_s", 60.0)

        self.ollama_url = str(self.get_parameter("ollama.url").value)
        self.ollama_model = str(self.get_parameter("ollama.model").value)
        self.ollama_timeout_s = float(self.get_parameter("ollama.timeout_s").value)

        # ---------------- Speech output defaults (only used if speak_responses=True) ----------------
        self.declare_parameter("assistant.speak_responses", True)
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "af_heart")
        self.declare_parameter("speech.interrupt", False)

        self.speak_responses = bool(self.get_parameter("assistant.speak_responses").value)
        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_rate = int(self.get_parameter("speech.rate").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)
        self.speech_interrupt = bool(self.get_parameter("speech.interrupt").value)

        # ---------------- Turn-taking / echo ----------------
        self.declare_parameter("assistant.listen_cooldown_s", 1.0)
        self.declare_parameter("assistant.echo_window_s", 3.0)
        self.declare_parameter("assistant.debug_log", True)

        self.listen_cooldown_s = float(self.get_parameter("assistant.listen_cooldown_s").value)
        self.echo_window_s = float(self.get_parameter("assistant.echo_window_s").value)
        self.debug_log = bool(self.get_parameter("assistant.debug_log").value)

        self._speech_active = False
        self._mute_until = 0.0

        self._last_tts_text = ""
        self._last_tts_time = 0.0

        self._busy_lock = threading.Lock()
        self._stop_flag = threading.Event()

        # ---------------- Prompt state ----------------
        self.current_prompt: str = ""
        self.current_step_id: str = ""

        # ---------------- ROS pub/sub ----------------
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)

        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)
        self.sub_status = self.create_subscription(String, self.speech_status_topic, self._on_speech_status, 10)
        self.sub_speech_req = self.create_subscription(String, self.speech_request_topic, self._on_speech_request, 10)

        # ---------------- RealtimeSTT ----------------
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.model", "base")
        self.declare_parameter("stt.compute_type", "float32")
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.post_speech_silence_duration", 0.6)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)
        self.declare_parameter("stt.input_device", -1)

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
            stt_kwargs["input_device_index"] = input_device
            stt_kwargs["input_device"] = input_device
            stt_kwargs["device_index"] = input_device

        filtered_kwargs = filter_kwargs_for_callable(AudioToTextRecorder, stt_kwargs)
        self.get_logger().info(f"RealtimeSTT ctor kwargs: {sorted(filtered_kwargs.keys())}")
        self.recorder = AudioToTextRecorder(**filtered_kwargs)

        # LLM conversation memory (keep short to reduce drift)
        self.history: List[Dict[str, str]] = []

        self.get_logger().info(
            "VoiceAssistant(Ollama) ready | "
            f"prompt_in={self.prompt_topic} event_out={self.event_topic} speech_out={self.speech_topic} "
            f"ollama_model={self.ollama_model} url={self.ollama_url} speak_responses={self.speak_responses}"
        )

        threading.Thread(target=self._listen_loop, daemon=True).start()

    # ---------------- Prompt input ----------------
    def _on_prompt(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.current_prompt = str(payload.get("prompt", "")).strip()
            self.current_step_id = str(payload.get("step", "")).strip()
        except Exception:
            self.current_prompt = (msg.data or "").strip()
            self.current_step_id = ""
        if self.debug_log:
            self.get_logger().info(f"[PROMPT_IN] step={self.current_step_id} :: {self.current_prompt[:120]}")

    # ---------------- Echo tracking ----------------
    def _on_speech_request(self, msg: String):
        # capture outgoing TTS text so we can ignore it if the mic picks it up
        try:
            payload = json.loads(msg.data)
            t = str(payload.get("text", "")).strip()
        except Exception:
            t = ""
        if t:
            self._last_tts_text = t.lower().strip()
            self._last_tts_time = time.time()

    def _on_speech_status(self, msg: String):
        state = (msg.data or "").strip().lower()
        if state == "start":
            self._speech_active = True
            self._mute_until = time.time() + 0.2
            # abort any blocking .text()
            try:
                if hasattr(self.recorder, "abort"):
                    self.recorder.abort()
            except Exception:
                pass
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech start -> gated + abort()")
        elif state == "done":
            self._speech_active = False
            self._mute_until = time.time() + self.listen_cooldown_s
            if self.debug_log:
                self.get_logger().info("[TURNTAKE] speech done -> cooldown gating")

    def _looks_like_echo(self, user_text: str) -> bool:
        if not self._last_tts_text:
            return False
        if time.time() - self._last_tts_time > self.echo_window_s:
            return False
        u = user_text.lower().strip().rstrip(".,!?")
        t = self._last_tts_text.lower().strip().rstrip(".,!?")
        if u and (u in t or t in u):
            return True
        # prefix similarity
        pref = 0
        for a, b in zip(u, t):
            if a == b:
                pref += 1
            else:
                break
        return pref >= 12

    # ---------------- Output helpers ----------------
    def _emit_event(self, ev: Dict[str, Any]):
        m = String()
        m.data = json.dumps(ev, ensure_ascii=False)
        self.pub_event.publish(m)
        if self.debug_log:
            self.get_logger().info(f"[EVENT_OUT] {m.data}")

    def _speak(self, text: str, interrupt: bool = False):
        if not self.speak_responses:
            return
        text = (text or "").strip()
        if not text:
            return
        payload = {
            "text": text,
            "volume": int(self.speech_volume),
            "rate": int(self.speech_rate),
            "voice": str(self.speech_voice),
            "interrupt": bool(interrupt or self.speech_interrupt),
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_speech.publish(m)
        if self.debug_log:
            self.get_logger().info(f"[SPEECH_OUT] {text[:120]}")

    # ---------------- Ollama ----------------
    def _build_system_prompt(self) -> str:
        # This constrains Ollama to emit the exact events StudyEngine expects.
        return f"""You are Stretch, a home assistant robot participating in a scripted onboarding study.

CURRENT STEP ID: {self.current_step_id}
CURRENT STEP PROMPT (follow it closely; do not jump ahead):
{self.current_prompt}

Your job is to interpret the user's speech and output:
1) A short SPOKEN response (1-2 sentences max).
2) An [EVENTS] JSON block containing events for the study engine.

Allowed event types and formats:

- Set a GLOBAL setting:
  {{"type":"set","scope":"global","key":"movement_speed|voice_volume|voice_profile|explainability|confirmation|social_distance","value":<value>}}

  movement_speed: "slow"|"medium"|"fast"
  voice_volume: integer 0-100
  voice_profile: "neutral"|"friendly"|"playful"
  explainability: "none"|"short"|"full"
  confirmation: "confirm"|"silent"
  social_distance: "close"|"medium"|"far"

- Advance to next step when user confirms:
  {{"type":"advance"}}

- If the current prompt expects arrival, and the user says where they are:
  {{"type":"arrive","room":"desk|bed|kitchen"}}

- If asking about performing a demo:
  {{"type":"demo_confirm","room":"desk|bed|kitchen","yes":true|false}}

RULES:
- If the user provides a valid setting value, emit a "set" event.
- If the user says yes/confirm, emit an "advance" event.
- If user says no, do NOT advance.
- ALWAYS include an [EVENTS] block even if empty: {{"events":[]}}
- Output format MUST be:

<spoken text>
[EVENTS]
{{"events":[ ... ]}}
[/EVENTS]
"""

    def _chat_with_ollama(self, user_text: str) -> str:
        # keep a short history to reduce drift
        self.history.append({"role": "user", "content": user_text})
        if len(self.history) > 10:
            self.history = self.history[-10:]

        payload = {
            "model": self.ollama_model,
            "messages": [{"role": "system", "content": self._build_system_prompt()}, *self.history],
            "stream": False,
        }

        r = requests.post(self.ollama_url, json=payload, timeout=self.ollama_timeout_s)
        r.raise_for_status()
        assistant_text = r.json()["message"]["content"]
        self.history.append({"role": "assistant", "content": assistant_text})
        return assistant_text

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

    # ---------------- STT loop ----------------
    def _listen_loop(self):
        self.get_logger().info("[STT] listen loop started")
        try:
            if hasattr(self.recorder, "start"):
                self.recorder.start()
        except Exception as e:
            self.get_logger().error(f"[STT] recorder.start failed: {e}")

        while rclpy.ok() and not self._stop_flag.is_set():
            if self._speech_active:
                time.sleep(0.05)
                continue

            try:
                text = self.recorder.text()
            except Exception:
                # abort() can interrupt a blocking call; that's okay
                time.sleep(0.05)
                continue

            if not text:
                continue

            cleaned = str(text).strip()
            if not cleaned:
                continue

            # mute window
            if time.time() < self._mute_until:
                if self.debug_log:
                    self.get_logger().info(f"[STT] gated transcript: {cleaned[:60]}")
                continue

            # echo filter
            if self._looks_like_echo(cleaned):
                if self.debug_log:
                    self.get_logger().info(f"[ECHO] ignored: {cleaned[:60]}")
                continue

            if not self._busy_lock.acquire(blocking=False):
                continue

            try:
                self.get_logger().info(f"[USER] {cleaned}")

                # quick deterministic advance on yes (helps if model misbehaves)
                low = cleaned.lower().strip().rstrip(".,!?")
                if low in YES_WORDS:
                    self._emit_event({"type": "advance"})
                    continue

                # ask Ollama for event(s)
                try:
                    raw = self._chat_with_ollama(cleaned)
                except Exception as e:
                    self.get_logger().error(f"[OLLAMA] request failed: {e}")
                    continue

                spoken, events = self._split_spoken_and_events(raw)

                # speak (optional)
                if spoken:
                    self._speak(spoken, interrupt=False)

                # publish events
                for ev in events:
                    # basic sanity: must have type
                    if isinstance(ev, dict) and ev.get("type"):
                        self._emit_event(ev)

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