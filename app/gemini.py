# gemini.py
import os, json, re
from typing import Tuple, Dict, Any
import google.generativeai as genai

MODEL_ID = "gemini-2.5-flash"   # available on your key and supports generateContent

def _client_ready() -> bool:
    return bool(os.getenv("GEMINI_API_KEY"))

def _ensure_config():
    if _client_ready() and not getattr(genai, "_configured", False):
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        genai._configured = True

def transcribe_audio(audio_bytes: bytes, mime_type: str) -> Tuple[bool, str, Dict[str, Any]]:
    if not _client_ready():
        return False, "GEMINI_API_KEY not set", {}
    _ensure_config()
    try:
        model = genai.GenerativeModel(MODEL_ID)
        resp = model.generate_content([
            {"text": "Transcribe the following audio to plain text."},
            {"inline_data": {"mime_type": mime_type, "data": audio_bytes}}
        ])
        text = (resp.text or "").strip()
        if not text:
            return False, "Empty transcription", {"raw": str(resp)}
        return True, text, {}
    except Exception as e:
        return False, f"Transcription error: {e}", {}

INTENT_SYSTEM_PROMPT = """You are an intent parser for a home robot.
Return STRICT JSON with keys: action, params.
Allowed actions:
- "set_volume"  {"level": 0-100}
- "speak"       {"text": "<string>"}
- "robot_start" {}
- "robot_stop"  {}
Output ONLY JSON.
"""

def _coerce_json(text: str) -> Dict[str, Any]:
    """
    Try strict JSON first; if that fails, extract the first {...} block and parse that.
    Raises ValueError on failure.
    """
    text = (text or "").strip()
    # direct attempt
    try:
        return json.loads(text)
    except Exception:
        pass
    # strip code fences if present
    if text.startswith("```"):
        text = text.strip("`").strip()
        # sometimes first line is "json"
        if text.lower().startswith("json"):
            text = text[4:].strip()
    # find first JSON object
    m = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if m:
        return json.loads(m.group(0))
    # last resort: empty object triggers error upstream
    raise ValueError("no JSON object found")


def parse_intent(utterance: str) -> Tuple[bool, Dict[str, Any], Dict[str, Any]]:
    if not _client_ready():
        return False, {"error": "GEMINI_API_KEY not set"}, {}

    _ensure_config()
    try:
        model = genai.GenerativeModel(MODEL_ID)
        resp = model.generate_content(
            [
                {"text": INTENT_SYSTEM_PROMPT},
                {"text": f'Utterance: "{utterance}"'}
            ],
            generation_config={
                "temperature": 0.1,
                "top_p": 0.9,
                "max_output_tokens": 200,
                # ✅ Force JSON back
                "response_mime_type": "application/json",
            },
        )

        # Prefer .text, but be resilient to SDK variants
        raw = (getattr(resp, "text", None) or "").strip()
        if not raw:
            # Try to extract from candidates/parts if .text is empty
            try:
                cand = (resp.candidates or [])[0]
                parts = getattr(cand, "content", {}).parts or []
                raw = "".join(getattr(p, "text", "") for p in parts).strip()
            except Exception:
                raw = ""

        data = _coerce_json(raw)  # <- robust parser

        if "action" not in data or "params" not in data:
            return False, {"error": "Invalid intent JSON structure", "raw": raw}, {}
        return True, data, {"raw": raw}

    except Exception as e:
        return False, {"error": f"Intent parse error: {e}"}, {}