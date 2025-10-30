# audio.py
import subprocess
import pyttsx3
import re

# --- Volume control helpers ---

def _run_cmd(cmd):
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except Exception as e:
        return 1, "", str(e)

def set_volume(level: int):
    """
    Set system output volume to 0-100%. Tries amixer (ALSA). If you use PulseAudio/PipeWire,
    uncomment the pactl block and comment out amixer.
    """
    # --- Option A: ALSA / amixer ---
    rc, out, err = _run_cmd(["amixer", "sset", "Master", f"{level}%"])
    if rc == 0:
        return True, f"System volume set to {level}% via amixer."

    # --- Option B: PulseAudio / PipeWire via pactl ---
    # sink = "@DEFAULT_SINK@"
    # rc, out, err = _run_cmd(["pactl", "set-sink-volume", sink, f"{level}%"])
    # if rc == 0:
    #     return True, f"System volume set to {level}% via pactl."

    return False, f"Failed to set volume. amixer error: {err or out}"

# --- TTS helpers (pyttsx3 offline) ---

_engine = None

def _get_engine():
    global _engine
    if _engine is None:
        _engine = pyttsx3.init()  # uses e.g. eSpeak on Ubuntu
    return _engine

def _pick_voice(engine, pattern: str):
    """
    Try to choose a voice whose name or id contains `pattern` (case-insensitive).
    If not found, return None to keep default.
    """
    if not pattern or pattern == "auto":
        return None
    voices = engine.getProperty("voices") or []
    rp = re.compile(re.escape(pattern), re.IGNORECASE)
    for v in voices:
        name = (getattr(v, "name", "") or "")
        vid = (getattr(v, "id", "") or "")
        if rp.search(name) or rp.search(vid):
            return v
    return None

def say_text(text: str, rate=None, voice=None, local_gain=None):
    """
    Speak `text` using pyttsx3. Optional: rate (wpm), voice (substring), local_gain (0.0-1.0).
    Returns (ok, message).
    """
    try:
        eng = _get_engine()

        # optional rate
        if rate is not None:
            try:
                rate = int(rate)
                eng.setProperty("rate", rate)
            except Exception:
                pass  # ignore bad rate

        # optional local volume (engine-level gain, not system volume)
        if local_gain is not None:
            try:
                local_gain = float(local_gain)
                local_gain = max(0.0, min(1.0, local_gain))
                eng.setProperty("volume", local_gain)
            except Exception:
                pass

        # optional voice selection
        v = _pick_voice(eng, voice)
        if v:
            eng.setProperty("voice", v.id)

        eng.say(text)
        eng.runAndWait()
        return True, "Spoken."
    except Exception as e:
        return False, f"TTS error: {e}"