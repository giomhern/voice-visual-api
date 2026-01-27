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


# ----------------------------
# Regex / constants
# ----------------------------
EVENTS_BLOCK_RE = re.compile(r"\[EVENTS\](.*?)\[/EVENTS\]", re.DOTALL | re.IGNORECASE)

YES_WORDS = {"yes", "yep", "yeah", "correct", "confirmed", "confirm", "sure", "ok", "okay"}
NO_WORDS = {"no", "nope", "nah", "negative"}

SPEED_WORDS = {"slow", "medium", "fast"}
PROFILE_WORDS = {"neutral", "friendly", "playful"}
DIST_WORDS = {"close", "medium", "far"}

EXPLAIN_MAP = {
    "step by step": "full",
    "step-by-step": "full",
    "full": "full",
    "short": "short",
    "brief": "short",
    "none": "none",
    "no explanations": "none",
    "dont explain": "none",
    "don't explain": "none",
    "do not explain": "none",
}

CONFIRM_MAP = {
    "confirm every change": "confirm",
    "confirm every": "confirm",
    "confirm": "confirm",
    "ask": "confirm",
    "silent": "silent",
    "silently": "silent",
    "no confirmation": "silent",
    "dont confirm": "silent",
    "don't confirm": "silent",
    "do not confirm": "silent",
}

NUM_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
    "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15,
    "sixteen": 16, "seventeen": 17, "eighteen": 18, "nineteen": 19,
    "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
    "seventy": 70, "eighty": 80, "ninety": 90, "hundred": 100,
}


def clamp(n: int, lo: int = 0, hi: int = 100) -> int:
    return max(lo, min(hi, n))


# ----------------------------
# Signature-safe kwargs helper
# ----------------------------
def filter_kwargs_for_callable(fn, kwargs: Dict[str, Any]) -> Dict[str, Any]:
    try:
        sig = inspect.signature(fn)
        accepted = set(sig.parameters.keys())
        return {k: v for k, v in kwargs.items() if k in accepted}
    except Exception:
        return kwargs


# ----------------------------
# Local (fast) parsing
# ----------------------------
def parse_int_0_100(text: str) -> Optional[int]:
    t = text.lower()

    # digits
    m = re.search(r"\b(\d{1,3})\b", t)
    if m:
        n = int(m.group(1))
        if 0 <= n <= 100:
            return n

    # words
    t = t.replace("-", " ")
    tokens = re.findall(r"[a-z]+", t)
    if not tokens:
        return None

    used = False
    current = 0
    for w in tokens:
        if w not in NUM_WORDS:
            continue
        used = True
        val = NUM_WORDS[w]
        if val == 100:
            current = 100 if current == 0 else current * 100
        elif val >= 20:
            current += val
        else:
            current += val

    if used and 0 <= current <= 100:
        return int(current)
    return None


def local_events_from_text(user_text: str, current_expect: str = "") -> List[Dict[str, Any]]:
    """
    Fast path: returns StudyEngine-compatible events when confidently parseable.
    If nothing recognized, returns [] so we can fall back to Ollama.
    """
    t = user_text.lower().strip()
    t_clean = t.rstrip(".,!?")

    # advance
    if t_clean in YES_WORDS:
        return [{"type": "advance"}]

    # arrive: “we are at the desk”, “in the kitchen”, etc.
    m = re.search(r"\b(desk|bed|kitchen)\b", t_clean)
    if m and (
        "we are" in t_clean
        or "we're" in t_clean
        or "at the" in t_clean
        or "in the" in t_clean
        or current_expect == "arrive"
    ):
        return [{"type": "arrive", "room": m.group(1)}]

    # movement speed
    if "speed" in t_clean or "move" in t_clean:
        for w in SPEED_WORDS:
            if re.search(rf"\b{w}\b", t_clean):
                return [{"type": "set", "scope": "global", "key": "movement_speed", "value": w}]

    # voice volume
    if "volume" in t_clean or "percent" in t_clean or "%" in t_clean or "louder" in t_clean or "quieter" in t_clean:
        n = parse_int_0_100(t_clean)
        if n is not None:
            return [{"type": "set", "scope": "global", "key": "voice_volume", "value": clamp(int(n))}]

    # voice profile
    if "voice" in t_clean or "profile" in t_clean or "accent" in t_clean:
        for w in PROFILE_WORDS:
            if re.search(rf"\b{w}\b", t_clean):
                return [{"type": "set", "scope": "global", "key": "voice_profile", "value": w}]

    # explainability
    if "explain" in t_clean or "explainability" in t_clean or "explanation" in t_clean:
        for k in sorted(EXPLAIN_MAP.keys(), key=len, reverse=True):
            if k in t_clean:
                return [{"type": "set", "scope": "global", "key": "explainability", "value": EXPLAIN_MAP[k]}]

    # confirmation
    if "confirm" in t_clean or "confirmation" in t_clean or "silent" in t_clean:
        for k in sorted(CONFIRM_MAP.keys(), key=len, reverse=True):
            if k in t_clean:
                return [{"type": "set", "scope": "global", "key": "confirmation", "value": CONFIRM_MAP[k]}]

    # social distance
    if "distance" in t_clean or "stand" in t_clean or "close" in t_clean or "far" in t_clean:
        for w in DIST_WORDS:
            if re.search(rf"\b{w}\b", t_clean):
                return [{"type": "set", "scope": "global", "key": "social_distance", "value": w}]

    # “no” = no event
    if t_clean in NO_WORDS:
        return []

    return []


# ----------------------------
# Node
# ----------------------------
class StretchVoiceAssistant(Node):
    def __init__(self):
        super().__init__("stretch_voice_assistant_hybrid")

        # Topics
        self.declare_parameter("topics.prompt_in", "/study_prompt")
        self.declare_parameter("topics.event_out", "/study_event")
        self.declare_parameter("topics.speech_out", "/speech_request")
        self.declare_parameter("topics.speech_status_in", "/speech_status")
        self.declare_parameter("topics.speech_request_in", "/speech_request")  # to capture outgoing TTS for echo filter

        self.prompt_topic = str(self.get_parameter("topics.prompt_in").value)
        self.event_topic = str(self.get_parameter("topics.event_out").value)
        self.speech_topic = str(self.get_parameter("topics.speech_out").value)
        self.speech_status_topic = str(self.get_parameter("topics.speech_status_in").value)
        self.speech_request_topic = str(self.get_parameter("topics.speech_request_in").value)

        # Ollama
        self.declare_parameter("ollama.url", "http://127.0.0.1:11434/api/chat")
        self.declare_parameter("ollama.model", "llama3.1:8b")
        self.declare_parameter("ollama.timeout_s", 60.0)

        self.ollama_url = str(self.get_parameter("ollama.url").value)
        self.ollama_model = str(self.get_parameter("ollama.model").value)
        self.ollama_timeout_s = float(self.get_parameter("ollama.timeout_s").value)

        # Behavior toggles
        self.declare_parameter("assistant.speak_responses", True)      # speak LLM response
        self.declare_parameter("assistant.fast_ack_local", False)      # say "Got it" on local parse (can cause echo)
        self.declare_parameter("assistant.repair_on_bad_events", True) # second pass if EVENTS parse fails
        self.declare_parameter("assistant.debug_log", True)

        self.speak_responses = bool(self.get_parameter("assistant.speak_responses").value)
        self.fast_ack_local = bool(self.get_parameter("assistant.fast_ack_local").value)
        self.repair_on_bad_events = bool(self.get_parameter("assistant.repair_on_bad_events").value)
        self.debug_log = bool(self.get_parameter("assistant.debug_log").value)

        # Speech defaults (for /speech_request)
        self.declare_parameter("speech.volume", 60)
        self.declare_parameter("speech.rate", 170)
        self.declare_parameter("speech.voice", "af_heart")
        self.declare_parameter("speech.interrupt", False)

        self.speech_volume = int(self.get_parameter("speech.volume").value)
        self.speech_rate = int(self.get_parameter("speech.rate").value)
        self.speech_voice = str(self.get_parameter("speech.voice").value)
        self.speech_interrupt = bool(self.get_parameter("speech.interrupt").value)

        # Turn-taking / echo
        self.declare_parameter("assistant.listen_cooldown_s", 1.0)
        self.declare_parameter("assistant.echo_window_s", 3.0)

        self.listen_cooldown_s = float(self.get_parameter("assistant.listen_cooldown_s").value)
        self.echo_window_s = float(self.get_parameter("assistant.echo_window_s").value)

        self._speech_active = False
        self._mute_until = 0.0
        self._last_tts_text = ""
        self._last_tts_time = 0.0

        self._busy_lock = threading.Lock()
        self._stop_flag = threading.Event()

        # Current prompt context
        self.current_prompt = ""
        self.current_step_id = ""
        self.current_expect = ""  # optional if study engine ever includes it

        # Pub/Sub
        self.pub_event = self.create_publisher(String, self.event_topic, 10)
        self.pub_speech = self.create_publisher(String, self.speech_topic, 10)

        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self._on_prompt, 10)
        self.sub_status = self.create_subscription(String, self.speech_status_topic, self._on_speech_status, 10)
        self.sub_speech_req = self.create_subscription(String, self.speech_request_topic, self._on_speech_request, 10)

        # STT params
        self.declare_parameter("stt.language", "en")
        self.declare_parameter("stt.model", "base")
        self.declare_parameter("stt.compute_type", "float32")
        self.declare_parameter("stt.webrtc_sensitivity", 2)
        self.declare_parameter("stt.silero_sensitivity", 0.15)
        self.declare_parameter("stt.post_speech_silence_duration", 0.4)
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
            # these vary by version; we will filter
            stt_kwargs["input_device_index"] = input_device
            stt_kwargs["input_device"] = input_device
            stt_kwargs["device_index"] = input_device

        filtered_kwargs = filter_kwargs_for_callable(AudioToTextRecorder, stt_kwargs)
        self.get_logger().info(f"RealtimeSTT ctor kwargs: {sorted(filtered_kwargs.keys())}")
        self.recorder = AudioToTextRecorder(**filtered_kwargs)

        # Keep tiny history (optional but helps a bit)
        self.history: List[Dict[str, str]] = []

        self.get_logger().info(
            "VoiceAssistant(HYBRID+REPAIR) ready | "
            f"ollama_model={self.ollama_model} url={self.ollama_url} "
            f"prompt_in={self.prompt_topic} event_out={self.event_topic} speak={self.speak_responses}"
        )

        threading.Thread(target=self._listen_loop, daemon=True).start()

    # ---------------- Prompt ----------------
    def _on_prompt(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.current_prompt = str(payload.get("prompt", "")).strip()
            self.current_step_id = str(payload.get("step", "")).strip()
            self.current_expect = str(payload.get("expect", "")).strip()
        except Exception:
            self.current_prompt = (msg.data or "").strip()
            self.current_step_id = ""
            self.current_expect = ""
        if self.debug_log:
            self.get_logger().info(f"[PROMPT_IN] step={self.current_step_id} :: {self.current_prompt[:120]}")

    # ---------------- Echo tracking ----------------
    def _on_speech_request(self, msg: String):
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
            # break any blocking .text()
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
                self.get_logger().info("[TURNTAKE] speech done -> cooldown")

    def _looks_like_echo(self, user_text: str) -> bool:
        if not self._last_tts_text:
            return False
        if time.time() - self._last_tts_time > self.echo_window_s:
            return False
        u = user_text.lower().strip().rstrip(".,!?")
        t = self._last_tts_text.lower().strip().rstrip(".,!?")
        if u and (u in t or t in u):
            return True
        pref = 0
        for a, b in zip(u, t):
            if a == b:
                pref += 1
            else:
                break
        return pref >= 12

    # ---------------- Output ----------------
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

    # ---------------- Ollama: strict prompt + repair ----------------
    def _build_system_prompt(self) -> str:
        # This is intentionally short + strict (helps small models)
        return f"""You are Stretch in a scripted onboarding study.

CURRENT STEP ID: {self.current_step_id}
CURRENT STEP PROMPT:
{self.current_prompt}

ABSOLUTE RULES:
- DO NOT output XML/HTML tags (e.g., <spoken text>).
- DO NOT repeat any template instructions.
- DO NOT use markdown code fences.
- Output EXACTLY TWO parts:
  (A) Spoken response: plain text, 1-2 short sentences.
  (B) One [EVENTS] block containing STRICT JSON only.

Allowed events (use ONLY these):
- {{"type":"set","scope":"global","key":"movement_speed|voice_volume|voice_profile|explainability|confirmation|social_distance","value":...}}
- {{"type":"advance"}}
- {{"type":"arrive","room":"desk|bed|kitchen"}}
- {{"type":"demo_confirm","room":"desk|bed|kitchen","yes":true|false}}

Values:
- movement_speed: "slow"|"medium"|"fast"
- voice_volume: integer 0-100
- voice_profile: "neutral"|"friendly"|"playful"
- explainability: "none"|"short"|"full"
- confirmation: "confirm"|"silent"
- social_distance: "close"|"medium"|"far"

REQUIRED OUTPUT (no extra text before/after):

<spoken text>
[EVENTS]
{{"events":[ ... ]}}
[/EVENTS]
"""

    def _chat_with_ollama(self, user_text: str) -> str:
        # keep tiny memory
        self.history.append({"role": "user", "content": user_text})
        if len(self.history) > 2:
            self.history = self.history[-2:]

        payload = {
            "model": self.ollama_model,
            "messages": [{"role": "system", "content": self._build_system_prompt()}, *self.history],
            "stream": False,
        }

        r = requests.post(self.ollama_url, json=payload, timeout=self.ollama_timeout_s)
        r.raise_for_status()
        assistant_text = r.json()["message"]["content"]
        self.history.append({"role": "assistant", "content": assistant_text})
        if len(self.history) > 2:
            self.history = self.history[-2:]
        return assistant_text

    def _repair_to_required_format(self, bad_output: str) -> str:
        # A second “formatter” pass: cheap and fixes 1B weirdness
        repair_prompt = f"""Reformat the following into EXACTLY:

<spoken text>
[EVENTS]
{{"events":[ ... ]}}
[/EVENTS]

Rules:
- No XML/HTML tags.
- No explanations.
- No code fences.
- JSON must be valid.
- Use only allowed event types.

TEXT TO REFORMAT:
{bad_output}
"""
        payload = {
            "model": self.ollama_model,
            "messages": [
                {"role": "system", "content": "You output only the required format. No extra text."},
                {"role": "user", "content": repair_prompt},
            ],
            "stream": False,
        }
        r = requests.post(self.ollama_url, json=payload, timeout=self.ollama_timeout_s)
        r.raise_for_status()
        return r.json()["message"]["content"]

    def _split_spoken_and_events(self, assistant_text: str) -> Tuple[str, List[Dict[str, Any]]]:
        if not assistant_text:
            return "", []

        m = EVENTS_BLOCK_RE.search(assistant_text)
        if not m:
            # strip any stray tag-looking stuff
            spoken = re.sub(r"<[^>]+>", "", assistant_text).strip()
            return spoken, []

        events_raw = m.group(1).strip()
        spoken = assistant_text[: m.start()].strip()

        # strip tag-looking junk from spoken
        spoken = re.sub(r"</?spoken text>", "", spoken, flags=re.IGNORECASE).strip()
        spoken = re.sub(r"<[^>]+>", "", spoken).strip()

        events: List[Dict[str, Any]] = []
        try:
            obj = json.loads(events_raw)
            if isinstance(obj, dict) and isinstance(obj.get("events"), list):
                events = [e for e in obj["events"] if isinstance(e, dict)]
        except Exception as e:
            if self.debug_log:
                self.get_logger().error(f"[EVENTS_PARSE] failed: {e} raw={events_raw[:200]}")

        return spoken, events

    # ---------------- Main loop ----------------
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
                # abort() can interrupt a blocking call; that's fine
                time.sleep(0.05)
                continue

            if not text:
                continue

            cleaned = str(text).strip()
            if not cleaned:
                continue

            if time.time() < self._mute_until:
                continue

            if self._looks_like_echo(cleaned):
                continue

            if not self._busy_lock.acquire(blocking=False):
                continue

            try:
                self.get_logger().info(f"[USER] {cleaned}")

                # 1) FAST PATH: local parsing
                local_events = local_events_from_text(cleaned, current_expect=self.current_expect)
                if local_events:
                    for ev in local_events:
                        self._emit_event(ev)

                    if self.fast_ack_local and self.speak_responses:
                        self._speak("Got it.", interrupt=False)

                    continue

                # 2) SLOW PATH: Ollama
                try:
                    raw = self._chat_with_ollama(cleaned)
                except Exception as e:
                    self.get_logger().error(f"[OLLAMA] request failed: {e}")
                    continue

                spoken, events = self._split_spoken_and_events(raw)

                # If parsing failed and repair is enabled, do one repair pass
                if self.repair_on_bad_events and not events:
                    try:
                        repaired = self._repair_to_required_format(raw)
                        spoken2, events2 = self._split_spoken_and_events(repaired)
                        if events2:
                            spoken, events = spoken2, events2
                    except Exception as e:
                        if self.debug_log:
                            self.get_logger().error(f"[OLLAMA_REPAIR] failed: {e}")

                # speak (optional)
                if spoken:
                    self._speak(spoken, interrupt=False)

                # publish events
                for ev in events:
                    if isinstance(ev, dict) and ev.get("type"):
                        self._emit_event(ev)

            finally:
                self._busy_lock.release()

            time.sleep(0.01)

    def destroy_node(self):
        self._stop_flag.set()
        try:
            if hasattr(self.recorder, "stop"):
                self.recorder.stop()
        except Exception:
            pass
        super().destroy_node()


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