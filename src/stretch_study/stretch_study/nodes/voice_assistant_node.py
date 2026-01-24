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
        self.declare_parameter("ollama.url", "http://127.0.0.1:11434/api/chat")  # <-- use 127.0.0.1
        self.declare_parameter("ollama.model", "llama3.1:8b")
        self.declare_parameter("ollama.timeout_s", 120.0)

        # --- Topics ---
        self.declare_parameter("topics.prompt_in", "/study_prompt")
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.speech_out", "/speech_request")

        # --- Speech defaults (speech_node uses these) ---
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "auto")

        # --- Assistant behavior ---
        self.declare_parameter("assistant.auto_listen", True)
        self.declare_parameter("assistant.speak_prompts", True)   # <-- default ON so you hear prompts
        self.declare_parameter("assistant.debug_log", True)
        self.declare_parameter("assistant.startup_say", True)     # <-- NEW: always speak once at startup

        # --- STT tuning ---
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.post_speech_silence_duration", 0.4)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)

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

        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)

        self.history: List[Dict[str, str]] = []
        self.current_prompt: Optional[str] = None
        self.current_step_id: Optional[str] = None

        self._busy_lock = threading.Lock()

        # STT recorder
        self.recorder = AudioToTextRecorder(
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

        self.get_logger().info(
            f"VoiceAssistantNode ready. model={self.ollama_model} url={self.ollama_url} speak_prompts={self.speak_prompts}"
        )

        # Always speak once at startup so you can verify /speech_request works
        if self.startup_say:
            self._speak("Voice assistant online.", interrupt=True)

        if self.auto_listen:
            threading.Thread(target=self._listen_loop, daemon=True).start()

    # ----------------- ROS publish helpers -----------------

    def _speak(self, text: str, interrupt: bool = False):
        text = (text or "").strip()
        if not text:
            return

        msg = String()
        msg.data = json.dumps(
            {
                "text": text,
                "volume": self.speech_volume,
                "rate": self.speech_rate,
                "voice": self.speech_voice,
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
        while rclpy.ok():
            if self._busy_lock.locked():
                time.sleep(0.05)
                continue
            try:
                self.recorder.text(self._on_transcript)
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

        if not self._busy_lock.acquire(blocking=False):
            return

        try:
            if self.debug_log:
                self.get_logger().info(f"[USER] {text}")

            raw = self._chat_with_ollama(text)
            spoken, events = self._split_spoken_and_events(raw)

            # Always speak something, even if events-only (use a small fallback)
            if spoken.strip():
                self._speak(spoken)
            else:
                self._speak("Okay.", interrupt=False)

            for ev in events:
                if isinstance(ev, dict):
                    self._emit_event(ev)

            # Optional heuristics (keep, but these can double-fire if LLM also emits them)
            if cleaned in YES_WORDS or cleaned.startswith("yes "):
                self._emit_event({"type": "advance"})
            if "desk" in cleaned:
                self._emit_event({"type": "arrive", "room": "desk"})
            elif "bed" in cleaned:
                self._emit_event({"type": "arrive", "room": "bed"})
            elif "kitchen" in cleaned:
                self._emit_event({"type": "arrive", "room": "kitchen"})

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

        return f"""You are Stretch, a friendly home assistant robot running a scripted onboarding study.

CURRENT STUDY STEP: {step}

CURRENT STUDY PROMPT (follow it closely):
{prompt}

You must respond with TWO parts:

PART A (SPOKEN): Natural language response for the user. Keep it concise (1-3 sentences).
PART B (EVENTS): A machine-readable JSON block inside [EVENTS] ... [/EVENTS].

EVENTS RULES:
- Always include the [EVENTS] block, even if it is empty: {{"events":[]}}
- Valid event objects (publish to /study_event):
  1) Advance: {{"type":"advance"}}
  2) Arrive:  {{"type":"arrive","room":"desk"}} room in ["desk","bed","kitchen"]
  3) Demo:    {{"type":"demo_confirm","room":"desk","yes":true}}
  4) Set:
     Global: {{"type":"set","scope":"global","key":"movement_speed","value":"slow"}}
     Room:   {{"type":"set","scope":"room","room":"desk","key":"voice_volume","value":40}}
     Rules:  {{"type":"set","scope":"rules","value":{{...}}}}

Allowed keys:
Global: movement_speed, voice_volume, voice_profile, explainability, confirmation, social_distance
Room: movement_speed, voice_volume, explainability, social_distance,
     cleaning_thoroughness (desk), pillow_arrangement (bed), preferred_snack (kitchen)

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