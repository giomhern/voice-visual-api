# audio.py
import os
import re
import shutil
import subprocess

# ---------- Helpers ----------

def _run_cmd(cmd):
    try:
        res = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False
        )
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except Exception as e:
        return 1, "", str(e)

def _which(name):
    return shutil.which(name)

def _to_int(x, default=None):
    try:
        return int(x)
    except Exception:
        return default

# ---------- Volume control ----------

def _pactl_set_volume(percent: int):
    """Try PipeWire/PulseAudio via pactl."""
    if not _which("pactl"):
        return False, "pactl not found", {"tool": "pactl"}
    # Prefer default sink handle; pactl >= 15 supports @DEFAULT_SINK@
    sink = "@DEFAULT_SINK@"
    rc, out, err = _run_cmd(["pactl", "set-sink-volume", sink, f"{percent}%"])
    if rc == 0:
        return True, f"Volume set to {percent}% via pactl.", {"tool": "pactl", "sink": sink}
    return False, f"pactl failed: {err or out}", {"tool": "pactl", "sink": sink}

def _amixer_set_volume(percent: int):
    """Try ALSA via amixer on common controls."""
    if not _which("amixer"):
        return False, "amixer not found", {"tool": "amixer"}
    for control in ("Master", "PCM"):
        rc, out, err = _run_cmd(["amixer", "sset", control, f"{percent}%"])
        if rc == 0:
            return True, f"Volume set to {percent}% via amixer:{control}.", {"tool": "amixer", "control": control}
        last_err = err or out
    return False, f"amixer failed: {last_err}", {"tool": "amixer"}

def set_volume(level: int):
    """Universal volume setter: pactl -> amixer."""
    level = max(0, min(100, int(level)))
    ok, msg, meta = _pactl_set_volume(level)
    if ok:
        return ok, msg, meta
    # Fall back to amixer
    ok2, msg2, meta2 = _amixer_set_volume(level)
    if ok2:
        return ok2, msg2, meta2
    return False, f"Failed to set volume. Tried pactl then amixer. Details: [{msg}] [{msg2}]", {"pactl": msg, "amixer": msg2}

# ---------- TTS ----------

# Strategy A: pyttsx3
_py_tts_ready = False
_engine = None
_py_tts_error = None

try:
    import pyttsx3
    _engine = pyttsx3.init()
    _py_tts_ready = True
except Exception as e:
    _py_tts_ready = False
    _py_tts_error = str(e)

def _pyttsx3_list_voices():
    if not _py_tts_ready:
        return []
    try:
        voices = _engine.getProperty("voices") or []
        out = []
        for v in voices:
            out.append({
                "id": getattr(v, "id", ""),
                "name": getattr(v, "name", ""),
                "languages": getattr(v, "languages", []),
            })
        return out
    except Exception:
        return []

def _pyttsx3_pick_voice(pattern: str):
    if not _py_tts_ready or not pattern or pattern == "auto":
        return None
    rp = re.compile(re.escape(pattern), re.IGNORECASE)
    for v in _engine.getProperty("voices") or []:
        if rp.search(getattr(v, "name", "")) or rp.search(getattr(v, "id", "")):
            return v
    return None

def _pyttsx3_say(text, rate=None, voice=None, local_gain=None):
    if not _py_tts_ready:
        return False, f"pyttsx3 not available: {_py_tts_error}", {"engine": "pyttsx3"}
    try:
        if rate is not None:
            r = _to_int(rate)
            if r is not None:
                _engine.setProperty("rate", r)
        if local_gain is not None:
            try:
                g = float(local_gain)
                _engine.setProperty("volume", max(0.0, min(1.0, g)))
            except Exception:
                pass
        v = _pyttsx3_pick_voice(voice)
        if v:
            _engine.setProperty("voice", v.id)
        _engine.say(text)
        _engine.runAndWait()
        return True, "Spoken via pyttsx3.", {"engine": "pyttsx3"}
    except Exception as e:
        return False, f"pyttsx3 error: {e}", {"engine": "pyttsx3"}

# Strategy B: espeak-ng CLI
def _espeak_available():
    return _which("espeak-ng") or _which("espeak")  # prefer espeak-ng, accept espeak

def _espeak_cmd():
    return _which("espeak-ng") or _which("espeak")

def _espeak_say(text, rate=None, voice=None):
    binpath = _espeak_cmd()
    if not binpath:
        return False, "espeak-ng/espeak not found", {"engine": "espeak-ng"}
    args = [binpath, text]
    # rate maps to -s (words per minute)
    r = _to_int(rate)
    if r:
        args = [binpath, "-s", str(r), text]
    # voice maps to -v; allow substring (best-effort)
    if voice and voice != "auto":
        args = [binpath, "-v", str(voice)] + args[len([binpath]):]
    rc, out, err = _run_cmd(args)
    if rc == 0:
        return True, "Spoken via espeak-ng.", {"engine": os.path.basename(binpath)}
    return False, f"espeak-ng error: {err or out}", {"engine": os.path.basename(binpath)}

def say_text(text: str, rate=None, voice=None, local_gain=None):
    """
    Try pyttsx3 first (offline), then fall back to espeak-ng CLI.
    Returns (ok, message, meta).
    """
    # A) pyttsx3
    ok, msg, meta = _pyttsx3_say(text, rate=rate, voice=voice, local_gain=local_gain)
    if ok:
        return ok, msg, meta
    py_err = msg

    # B) espeak-ng
    ok2, msg2, meta2 = _espeak_say(text, rate=rate, voice=voice)
    if ok2:
        return ok2, msg2, meta2

    return False, f"TTS failed. pyttsx3: [{py_err}] | espeak: [{msg2}]", {"pyttsx3": py_err, "espeak": msg2}

def list_voices():
    """Expose available voices (pyttsx3 if present, else espeak-ng presence)."""
    data = {"pyttsx3_available": _py_tts_ready, "pyttsx3_error": _py_tts_error, "pyttsx3_voices": _pyttsx3_list_voices()}
    data["espeak_available"] = bool(_espeak_available())
    return data