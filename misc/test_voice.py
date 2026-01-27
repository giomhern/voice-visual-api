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
        # callback=on_text,
        # silence_duration=0.8,       # how long silence = end of utterance
        # vad=True
    )

    try:
        recorder.start()
        while True:
            text = recorder.text()
            if text:
                text = text.strip()
                if text:
                    print(f"[VOICE INPUT] {text}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 Stopping recorder")
    finally:
        recorder.stop()


if __name__ == "__main__":
    main()


# 2026-01-27 14:18:35.050 - RealTimeSTT: realtimestt - INFO - Starting RealTimeSTT
# 2026-01-27 14:18:35.059 - RealTimeSTT: realtimestt - INFO - Initializing audio recording (creating pyAudio input stream, sample rate: 16000 buffer size: 512
# 2026-01-27 14:18:35.060 - RealTimeSTT: realtimestt - DEBUG - Starting audio data worker with target_sample_rate=16000, buffer_size=512, input_device_index=None
# 2026-01-27 14:18:35.060 - RealTimeSTT: realtimestt - INFO - Initializing WebRTC voice with Sensitivity 3
# 2026-01-27 14:18:35.060 - RealTimeSTT: realtimestt - DEBUG - Creating PyAudio interface...
# 2026-01-27 14:18:35.060 - RealTimeSTT: realtimestt - DEBUG - WebRTC VAD voice activity detection engine initialized successfully
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - No device index supplied; using default device 18
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Retrieving highest sample rate for device index 18: {'index': 18, 'structVersion': 2, 'name': 'default', 'hostApi': 0, 'maxInputChannels': 32, 'maxOutputChannels': 32, 'defaultLowInputLatency': 0.008684807256235827, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultHighInputLatency': 0.034807256235827665, 'defaultHighOutputLatency': 0.034807256235827665, 'defaultSampleRate': 44100.0}
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Highest supported sample rate for device index 18 is 44100
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Sample rates to try for device 18: [16000, 44100]
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Attempting to initialize audio stream at 16000 Hz.
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Found 19 total audio devices on the system.
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Available input devices with input channels: [0, 5, 6, 12, 13, 14, 15, 16, 18]
# 2026-01-27 14:18:35.310 - RealTimeSTT: realtimestt - DEBUG - Validating device index 18 with info: {'index': 18, 'structVersion': 2, 'name': 'default', 'hostApi': 0, 'maxInputChannels': 32, 'maxOutputChannels': 32, 'defaultLowInputLatency': 0.008684807256235827, 'defaultLowOutputLatency': 0.008684807256235827, 'defaultHighInputLatency': 0.034807256235827665, 'defaultHighOutputLatency': 0.034807256235827665, 'defaultSampleRate': 44100.0}
# 2026-01-27 14:18:35.388 - RealTimeSTT: realtimestt - DEBUG - Device index 18 successfully validated.
# 2026-01-27 14:18:35.388 - RealTimeSTT: realtimestt - DEBUG - Opening stream with device index 18, sample_rate=16000, chunk_size=1024
# 2026-01-27 14:18:35.390 - RealTimeSTT: realtimestt - INFO - Microphone connected and validated (device index: 18, sample rate: 16000, chunk size: 1024)
# 2026-01-27 14:18:35.391 - RealTimeSTT: realtimestt - DEBUG - Audio recording initialized successfully at 16000 Hz, reading 1024 frames at a time
# 2026-01-27 14:18:35.718 - RealTimeSTT: realtimestt - DEBUG - Silero VAD voice activity detection engine initialized successfully
# 2026-01-27 14:18:35.719 - RealTimeSTT: realtimestt - DEBUG - Starting realtime worker
# 2026-01-27 14:18:35.719 - RealTimeSTT: realtimestt - DEBUG - Waiting for main transcription model to start
# 2026-01-27 14:18:36.526 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:37.530 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:38.535 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:39.536 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:40.181 - RealTimeSTT: realtimestt - DEBUG - Main transcription model ready
# 2026-01-27 14:18:40.182 - RealTimeSTT: realtimestt - DEBUG - RealtimeSTT initialization completed successfully
# 2026-01-27 14:18:40.182 - RealTimeSTT: realtimestt - INFO - recording started
# 2026-01-27 14:18:40.182 - RealTimeSTT: realtimestt - INFO - State changed from 'inactive' to 'recording'
# 2026-01-27 14:18:40.267 - RealTimeSTT: realtimestt - INFO - State changed from 'recording' to 'inactive'
# 2026-01-27 14:18:40.312 - RealTimeSTT: realtimestt - INFO - Setting listen time
# 2026-01-27 14:18:40.312 - RealTimeSTT: realtimestt - DEBUG - Waiting for recording stop
# 2026-01-27 14:18:40.541 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:41.450 - RealTimeSTT: realtimestt - INFO - recording stopped
# 2026-01-27 14:18:41.452 - RealTimeSTT: realtimestt - DEBUG - No samples removed, final audio length: 17920
# 2026-01-27 14:18:41.452 - RealTimeSTT: realtimestt - INFO - State changed from 'inactive' to 'transcribing'
# 2026-01-27 14:18:41.452 - RealTimeSTT: realtimestt - DEBUG - Adding transcription request, no early transcription started
# 2026-01-27 14:18:41.454 - RealTimeSTT: realtimestt - DEBUG - Receive from parent_transcription_pipe after sendiung transcription request, transcribe_count: 1
# 2026-01-27 14:18:41.467 - RealTimeSTT: realtimestt - INFO - State changed from 'transcribing' to 'inactive'
# 2026-01-27 14:18:41.496 - RealTimeSTT: realtimestt - DEBUG - Model base completed transcription in 0.04 seconds
# 2026-01-27 14:18:41.596 - RealTimeSTT: realtimestt - INFO - Setting listen time
# 2026-01-27 14:18:41.597 - RealTimeSTT: realtimestt - INFO - State changed from 'inactive' to 'listening'
# 2026-01-27 14:18:41.597 - RealTimeSTT: realtimestt - DEBUG - Waiting for recording start
# 2026-01-27 14:18:41.631 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:42.720 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:43.725 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:44.814 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:45.905 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:46.911 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:47.999 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:49.090 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:50.179 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:51.184 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:52.272 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:53.363 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:54.369 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:55.457 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:56.546 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:57.636 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:58.643 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:18:59.729 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:00.820 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:01.825 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:02.915 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:04.005 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:05.094 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:06.099 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:07.190 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:08.277 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:09.280 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:10.369 - RealTimeSTT: realtimestt - DEBUG - _audio_data_worker writing audio data into queue.
# 2026-01-27 14:19:10.464 - RealTimeSTT: realtimestt - INFO - KeyboardInterrupt in wait_audio, shutting down
# 2026-01-27 14:19:10.464 - RealTimeSTT: realtimestt - DEBUG - Finishing recording thread
# 2026-01-27 14:19:10.472 - RealTimeSTT: realtimestt - DEBUG - Terminating reader process
# 2026-01-27 14:19:10.557 - RealTimeSTT: realtimestt - DEBUG - Terminating transcription process
# 2026-01-27 14:19:10.557 - RealTimeSTT: realtimestt - DEBUG - Finishing realtime thread
# 2026-01-27 14:19:10.649 - RealTimeSTT: realtimestt - INFO - KeyboardInterrupt in text() method
# 2026-01-27 14:19:10.649 - RealTimeSTT: realtimestt - INFO - recording stopped
