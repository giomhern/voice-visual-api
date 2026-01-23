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


# We ask the LLM to append a machine-readable JSON block like:
# [EVENTS]
# {"events":[{"type":"set","scope":"global","key":"movement_speed","value":"slow"}]}
# [/EVENTS]
EVENTS_BLOCK_RE = re.compile(r"\[EVENTS\](.*?)\[/EVENTS\]", re.DOTALL | re.IGNORECASE)

YES_WORDS = {"yes", "yep", "yeah", "correct", "confirmed", "confirm", "sounds good", "sure", "ok", "okay"}
NO_WORDS = {"no", "nope", "nah", "incorrect"}

HALLUCINATION_PHRASES = {"", " ", ".", "...", "um", "uh", "hmm", "thank you", "thanks", "bye", "goodbye"}


class VoiceAssistantNode(Node):
    """
    Voice assistant that:
      - subscribes to /study_prompt (from StudyEngine)
      - listens via STT
      - calls Ollama chat (LLM)
      - speaks LLM natural-language output via /speech_request
      - parses LLM JSON EVENTS block and publishes StudyEvents to /study_event
    """

    def __init__(self):
        super().__init__("stretch_voice_assistant")

        # -------------------------
        # Params
        # -------------------------
        self.declare_parameter("ollama.url", "http://localhost:11434/api/chat")
        self.declare_parameter("ollama.model", "llama3.1:8b")
        self.declare_parameter("ollama.timeout_s", 30.0)

        self.declare_parameter("topics.prompt_in", "/study_prompt")
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.speech_out", "/speech_request")

        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "auto")

        # STT tuning
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.post_speech_silence_duration", 0.4)
        self.declare_parameter("stt.min_length_of_recording", 0.8)
        self.declare_parameter("stt.min_gap_between_recordings", 0.3)

        # Behavior
        self.declare_parameter("assistant.auto_listen", True)     # keep listening even without prompts
        self.declare_parameter("assistant.speak_prompts", True)   # speak the incoming study prompt
        self.declare_parameter("assistant.debug_log", True)
        self.declare_parameter("assistant.fallback_heuristics", True)  # also emit simple events from user text

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
        self.fallback_heuristics = bool(self.get_parameter("assistant.fallback_heuristics").value)

        # -------------------------
        # ROS pubs/subs
        # -------------------------
        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)

        # -------------------------
        # State for LLM
        # -------------------------
        self.history: List[Dict[str, str]] = []
        self.current_prompt: Optional[str] = None
        self.current_step_id: Optional[str] = None

        # Prevent overlap
        self._busy_lock = threading.Lock()

        # -------------------------
        # STT init
        # -------------------------
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
            f"VoiceAssistantNode ready. prompt_in={self.prompt_topic} event_out={self.event_topic} speech_out={self.speech_topic}"
        )

        if self.auto_listen:
            threading.Thread(target=self._listen_loop, daemon=True).start()

    # -------------------------
    # ROS helpers
    # -------------------------
    def _speak(self, text: str, interrupt: bool = False):
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

    def _emit_event(self, ev: dict):
        msg = String()
        msg.data = json.dumps(ev, ensure_ascii=False)
        self.pub_event.publish(msg)
        if self.debug_log:
            self.get_logger().info(f"[EVENT_OUT] {msg.data}")

    # -------------------------
    # Prompt input from StudyEngine
    # -------------------------
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

        # Speak the prompt (script line)
        if self.speak_prompts and self.current_prompt:
            self._speak(self.current_prompt, interrupt=True)

    # -------------------------
    # STT loop
    # -------------------------
    def _listen_loop(self):
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

        cleaned = text.strip().lower().rstrip(".,!?")
        if cleaned in HALLUCINATION_PHRASES:
            if self.debug_log:
                self.get_logger().info(f"[STT] filtered hallucination: {text!r}")
            return

        if not self._busy_lock.acquire(blocking=False):
            return

        try:
            if self.debug_log:
                self.get_logger().info(f"[USER] {text}")

            raw_assistant = self._chat_with_ollama(user_text=text)

            spoken_text, events = self._split_spoken_and_events(raw_assistant)

            if self.debug_log:
                self.get_logger().info(f"[ASSISTANT_SPOKEN] {spoken_text[:200]}")
                self.get_logger().info(f"[ASSISTANT_EVENTS] {events}")

            # Speak only the natural language portion
            if spoken_text.strip():
                self._speak(spoken_text, interrupt=False)

            # Publish structured events (preferred)
            for ev in events:
                self._emit_event(ev)

            # Optional: heuristic fallback events from user text (helps when model forgets events block)
            if self.fallback_heuristics:
                self._fallback_emit_from_user_text(user_text=text)

        finally:
            self._busy_lock.release()

    # -------------------------
    # Ollama chat
    # -------------------------
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
        except requests.exceptions.ConnectionError:
            assistant_text = "I can't connect to Ollama right now. Please check that it is running."
        except requests.exceptions.Timeout:
            assistant_text = "My response timed out. Please try again."
        except Exception as e:
            assistant_text = f"I ran into an error talking to Ollama: {e}"

        self.history.append({"role": "assistant", "content": assistant_text})
        return assistant_text

    def _build_system_prompt(self) -> str:
        """
        Critical: instruct the model to ALWAYS output an [EVENTS] JSON block.
        We will speak only the natural language portion (everything before [EVENTS]).
        """
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
  1) Advance:
     {{"type":"advance"}}
  2) Arrive at a station:
     {{"type":"arrive","room":"desk"}} or room in ["desk","bed","kitchen"]
  3) Confirm a demo should run:
     {{"type":"demo_confirm","room":"desk","yes":true}}
  4) Apply a setting:
     Global: {{"type":"set","scope":"global","key":"movement_speed","value":"slow"}}
     Room:   {{"type":"set","scope":"room","room":"desk","key":"voice_volume","value":40}}
     Rules:  {{"type":"set","scope":"rules","value":{{...raw json object...}}}}
- Only include events you are confident about based on the user's latest message and the current study prompt.

Allowed keys you may set:
Global keys: movement_speed, voice_volume, voice_profile, explainability, confirmation, social_distance
Room keys: movement_speed, voice_volume, explainability, social_distance,
          cleaning_thoroughness (desk), pillow_arrangement (bed), preferred_snack (kitchen)

Output format MUST be:

<spoken text>
[EVENTS]
{{"events":[ ... ]}}
[/EVENTS]
"""

    # -------------------------
    # Events parsing
    # -------------------------
    def _split_spoken_and_events(self, assistant_text: str) -> Tuple[str, List[Dict[str, Any]]]:
        """
        Returns (spoken_text, events_list).
        If events block is missing or malformed, events_list will be [].
        """
        if not assistant_text:
            return "", []

        m = EVENTS_BLOCK_RE.search(assistant_text)
        if not m:
            # No events block
            return assistant_text.strip(), []

        events_raw = m.group(1).strip()

        # Spoken text is everything before [EVENTS]
        spoken_text = assistant_text[: m.start()].strip()

        events_list: List[Dict[str, Any]] = []
        try:
            obj = json.loads(events_raw)
            if isinstance(obj, dict) and isinstance(obj.get("events"), list):
                # normalize: only dicts
                events_list = [e for e in obj["events"] if isinstance(e, dict)]
        except Exception as e:
            if self.debug_log:
                self.get_logger().error(f"[EVENTS_PARSE] failed: {e} raw={events_raw[:200]}")

        return spoken_text, events_list

    # -------------------------
    # Fallback heuristics (optional)
    # -------------------------
    def _fallback_emit_from_user_text(self, user_text: str):
        lt = user_text.strip().lower()

        # Confirmations -> advance
        if lt in YES_WORDS or lt.startswith("yes "):
            self._emit_event({"type": "advance"})

        # Station mention -> arrive
        if "desk" in lt:
            self._emit_event({"type": "arrive", "room": "desk"})
        elif "bed" in lt:
            self._emit_event({"type": "arrive", "room": "bed"})
        elif "kitchen" in lt:
            self._emit_event({"type": "arrive", "room": "kitchen"})

        # Explicit no: do nothing (engine will re-prompt)
        if lt in NO_WORDS or lt.startswith("no "):
            pass


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