import time
from RealtimeSTT import AudioToTextRecorder


def on_text(text: str):
    """
    Callback fired every time speech is detected and transcribed
    """
    text = text.strip()
    if not text:
        return

    print(f"[VOICE INPUT] {text}")


def main():
    print("🎤 Voice input test started")
    print("Speak naturally. Press Ctrl+C to stop.\n")

    recorder = AudioToTextRecorder(
        language="en",
        model="base",               # good balance of speed/accuracy
        compute_type="float32",     # safest on Stretch
        callback=on_text,
        silence_duration=0.8,       # how long silence = end of utterance
        vad=True
    )

    try:
        recorder.start()
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 Stopping recorder")
    finally:
        recorder.stop()


if __name__ == "__main__":
    main()