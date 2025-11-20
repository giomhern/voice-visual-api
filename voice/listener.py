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


if __name__ == "__main__":
    print("--- Stretch Push-to-Talk Interface ---")
    print("Press ENTER to start recording, and ENTER again to stop.")
    print("Press Ctrl+C to exit.\n")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


        


