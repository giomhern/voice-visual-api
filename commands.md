ros2 topic pub --once /speech_request std_msgs/msg/String \
'{"data":"{\"text\":\"Hello. This is Stretch speaking.\",\"volume\":60,\"rate\":170,\"voice\":\"auto\"}"}'

ros2 run stretch_study study_engine --ros-args --log-level INFO \
  -p speech.enable:=true \
  -p nav.enable:=false \
  -p motion.enable_transit:=false

[0.134s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/hello-robot/voice-visual-api/stretch_loop/install/stretch_study' in the environment variable AMENT_PREFIX_PATH doesn't exist
Starting >>> stretch_study
--- stderr: stretch_study                   
usage: setup.py [global_opts] cmd1 [cmd1_opts] [cmd2 [cmd2_opts] ...]
   or: setup.py --help [cmd1 cmd2 ...]
   or: setup.py --help-commands
   or: setup.py cmd --help

error: option --editable not recognized
---
Failed   <<< stretch_study [0.91s, exited with code 1]

Summary: 0 packages finished [1.01s]
  1 package failed: stretch_study
  1 package had stderr output: stretch_study


def build_script():
    """
    Legacy compatibility helper used by nodes/study_engine.py.

    Returns a list of dict steps:
      {"id": str, "prompt": str, "expect": Optional[str]}
    Only "expect":"arrive" is used by StudyEngine for gating arrive events.
    """
    return [
        {
            "id": "intro_greeting",
            "prompt": (
                "Hello! I'm Stretch. Thanks for unboxing me. Before we walk around your home, "
                "we need to set my general behavior settings. These settings will become my default "
                "behavior everywhere, unless you change them later for a specific room."
            ),
        },
        {
            "id": "global_movement_speed",
            "prompt": (
                "First: Movement Speed. I can move slowly, at a medium pace, or fast. "
                "Please set my default movement speed."
            ),
        },
        {
            "id": "global_voice_volume",
            "prompt": (
                "Next: Voice Volume. You can set my speaking volume anywhere between 0 and 100 percent. "
                "Please set my default speaking volume."
            ),
        },
        {
            "id": "global_voice_profile",
            "prompt": (
                "Next: Voice Profile or Accent. You can choose a neutral voice, a friendly voice, or a playful voice. "
                "Please set my default voice profile."
            ),
        },
        {
            "id": "global_explainability",
            "prompt": (
                "Next: Explainability Level. This determines how much I explain myself when I act. "
                "Options include none, short explanations, or full step-by-step explanations. "
                "Please set my default explainability level."
            ),
        },
        {
            "id": "global_social_distance",
            "prompt": (
                "Next: Social Distance. This is how far I should stand from you when idle or interacting. "
                "Options are close, medium, or far. Please set my default social distance."
            ),
        },
        {
            "id": "global_confirmation_style",
            "prompt": (
                "Finally: Confirmation Style. Should I continue providing verbal confirmations for future changes, "
                "or apply them silently? Please choose yes or no."
            ),
        },
        {
            "id": "global_complete",
            "prompt": (
                "Great. My default behavior has been set. You'll now guide me through each room, "
                "and we'll make room-specific customizations that override these defaults. "
                "Please walk with me and guide me. When we reach a station, tell me where we are."
            ),
        },

        # ---- TOUR / ARRIVAL STEPS ----
        {"id": "arrive_desk", "prompt": "Please tell me when we are now at the Desk area.", "expect": "arrive"},
        {"id": "arrive_bed", "prompt": "Please tell me when we are now at the Bed area.", "expect": "arrive"},
        {"id": "arrive_kitchen", "prompt": "Please tell me when we are now at the Kitchen area.", "expect": "arrive"},

        # End marker (engine will just keep prompting unless you advance/stop)
        {"id": "end", "prompt": "Thank you. Setup is complete."},
    ]

    bottom of script_s


    ros2 topic pub --once /speech_request std_msgs/msg/String \
'{"data":"{\"text\":\"Hello, I am using af_heart.\",\"volume\":70,\"voice\":\"af_heart\",\"interrupt\":true}"}'

ros2 pkg executables stretch_study | grep speech
grep -R "kokoro-82m" -n ~/voice-visual-api/stretch_loop/src/stretch_study | head
grep -R "Kokoro-82M" -n ~/voice-visual-api/stretch_loop/src/stretch_study | head

eightNorm.apply(module, name, dim)
[INFO] [1769227095.840019420] [stretch_study_speech]: [SPEECH] Kokoro pipeline loaded.
[INFO] [1769227095.842085371] [stretch_study_speech]: [SPEECH] Ready. Subscribed to /speech_request
[INFO] [1769227102.427175278] [stretch_study_speech]: [SPEECH] queued voice=auto vol=60 text='Voice assistant online.'
[ERROR] [1769227102.531748852] [stretch_study_speech]: [SPEECH] Kokoro failed: 404 Client Error. (Request ID: Root=1-6974435e-62deb65432c638215de22a50;b2fa5115-3498-405c-9883-b8502b38f38f)

Entry Not Found for url: https://huggingface.co/hexgrad/Kokoro-82M/resolve/main/voices/auto.pt.


python3 - <<'PY'
import sounddevice as sd
print("DEFAULT IN:", sd.default.device)
for i,d in enumerate(sd.query_devices()):
    if d.get('max_input_channels',0)>0:
        print(i, d['name'])
python3 - <<'PY'
import sounddevice as sd
print("Default device:", sd.default.device)
print("\n=== INPUT DEVICES ===")
for i,d in enumerate(sd.query_devices()):
    if d['max_input_channels'] > 0:
        print(i, d['name'], "| SR:", d['default_samplerate'])

print("\n=== OUTPUT DEVICES ===")
for i,d in enumerate(sd.query_devices()):
    if d['max_output_channels'] > 0:
        print(i, d['name'], "| SR:", d['default_samplerate'])
PY



=== INPUT DEVICES ===
0 HDA Intel PCH: ALC269VB Analog (hw:0,0) | SR: 44100.0
6 sysdefault | SR: 48000.0
12 samplerate | SR: 44100.0
13 speexrate | SR: 44100.0
14 pulse | SR: 44100.0
15 upmix | SR: 44100.0
16 vdownmix | SR: 44100.0
18 default | SR: 44100.0

=== OUTPUT DEVICES ===
0 HDA Intel PCH: ALC269VB Analog (hw:0,0) | SR: 44100.0
1 HDA Intel PCH: HDMI 0 (hw:0,3) | SR: 44100.0
2 HDA Intel PCH: HDMI 1 (hw:0,7) | SR: 44100.0
3 HDA Intel PCH: HDMI 2 (hw:0,8) | SR: 44100.0
4 HDA Intel PCH: HDMI 3 (hw:0,9) | SR: 44100.0
5 ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:1,0) | SR: 16000.0
6 sysdefault | SR: 48000.0
7 front | SR: 44100.0
8 surround40 | SR: 44100.0
9 surround51 | SR: 44100.0
10 surround71 | SR: 44100.0
11 hdmi | SR: 44100.0
12 samplerate | SR: 44100.0
13 speexrate | SR: 44100.0
14 pulse | SR: 44100.0
15 upmix | SR: 44100.0
16 vdownmix | SR: 44100.0
17 dmix | SR: 48000.0
18 default | SR: 44100.0
hello-robot@stretch-se3-3103:~/voice-visual-api$


hello-robot@stretch-se3-3103:~$ pactl list short sources
0	alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo.monitor	module-alsa-card.c	s24le 2ch 16000Hz	SUSPENDED
1	alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input	module-alsa-card.c	s16le 6ch 16000Hz	RUNNING
2	alsa_output.pci-0000_00_1f.3.analog-stereo.monitor	module-alsa-card.c	s16le 2ch 44100Hz	IDLE
[5]+  Terminated              sleep 5



pactl set-default-source alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input

pactl load-module module-remap-source \
  source_name=respecter_mono \
  master=alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input \
  channels=1 \
  channel_map=mono


pactl set-default-source respecter_mono

parecord --rate=16000 --channels=1 --format=s16le /tmp/test.wav &
sleep 5
kill $!
aplay /tmp/test.wav