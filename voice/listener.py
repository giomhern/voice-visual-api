from http.client import TOO_EARLY
import io 
import threading
import wave 
import numpy as np
import sounddevice as sd 
import requests 
from pynput import keyboard

API_URL = "http://localhost:5000/voice/command"
SAMPLE_RATE = 16000
CHANNELS = 1

TOGGLE_KEY = keyboard.Key.enter

is_recording = False 
audio_chunks = []
stream = None 
lock = threading.lock()


def audio_callback(input, frames, time, status):
    if status:
        print("[listener] Audio status", status)
    with lock:
        if is_recording:
            audio_chunks.append(input.copy())


def make_wav_bytes(chunks, fs=SAMPLE_RATE, channels=CHANNELS):
    if not chunks:
        return b""
    
    audio = np.concatenate(chunks, axis=0)
    buff = io.BytesIO()
    with wave.open(buff, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(audio.tobytes())
    buff.seek(0)
    return buff.read()


def start_recording():
    global is_recording, audio_chunks, stream

    with lock:
        if is_recording:
            return 
        print("[listener] Recording started... (press ENTER to stop)")
        audio_chunks = []
        stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype='int16',
            callback=audio_callback,
        )
        stream.start()
        is_recording = True

def stop_recording():
    global is_recording, stream, audio_chunks
    with lock:
        if not is_recording:
            return

        print("[listener] Recording stopped. Processing...")
        is_recording = False
        if stream is not None:
            stream.stop()
            stream.close()
            stream = None

        chunks = audio_chunks
        audio_chunks = []

    audio_bytes = make_wav_bytes(chunks)
    send_voice_command(audio_bytes)

    print("[listener] Ready for next command! Press ENTER again.")

def send_voice_command(audio_bytes):
    if not audio_bytes:
        print("[listener] No audio captured; not sending.")
        return

    print("[listener] Sending audio to API...")

    files = {
        "audio": ("input.wav", audio_bytes, "audio/wav"),
    }
    data = {
        "mime_type": "audio/wav",
    }

    try: 
        res = requests.post(API_URL, files=files, data=data)
        j = res.json()
    except Exception as e:
        print("[listener] Error sending voice command to server: ", e)
        return 

    print("[listener] Status: ", res.status_code)
    print("[listener] Response: ", j)


def on_press(key):
    global is_recording 

    if key == TOGGLE_KEY:
        if not is_recording:
            start_recording()
        else:
            stop_recording()


if __name__ == "__main__":
    print("--- Stretch Push-to-Talk Interface ---")
    print("Press ENTER to start recording, and ENTER again to stop.")
    print("Press Ctrl+C to exit.\n")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


        


