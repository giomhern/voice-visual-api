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


 All log files can be found below /home/hello-robot/.ros/log/2026-01-27-14-47-31-727095-stretch-se3-3103-12032
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [stretch_driver-3]: process started with pid [12056]
[INFO] [joint_state_publisher-1]: process started with pid [12052]
[INFO] [robot_state_publisher-2]: process started with pid [12054]
[INFO] [sllidar_node-4]: process started with pid [12058]
[INFO] [scan_to_scan_filter_chain-5]: process started with pid [12060]
[INFO] [rviz2-6]: process started with pid [12062]
[INFO] [sync_slam_toolbox_node-7]: process started with pid [12064]
[sllidar_node-4] [INFO] [1769546852.113209761] [sllidar_node]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1, SLLIDAR SDK Version:2.1.0
[scan_to_scan_filter_chain-5] [INFO] [1769546852.120213276] [laser_filter]: In shadow configure done
[scan_to_scan_filter_chain-5] [WARN] [1769546852.120543675] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[sllidar_node-4] [INFO] [1769546852.121061685] [sllidar_node]: SLLidar S/N: E0FDECF0C3E09ED2A0EA98F373254110
[sllidar_node-4] [INFO] [1769546852.121140081] [sllidar_node]: Firmware Ver: 1.29
[sllidar_node-4] [INFO] [1769546852.121150459] [sllidar_node]: Hardware Rev: 7
[sllidar_node-4] [INFO] [1769546852.122720874] [sllidar_node]: SLLidar health status : 0
[sllidar_node-4] [INFO] [1769546852.122760437] [sllidar_node]: SLLidar health status : OK.
[scan_to_scan_filter_chain-5] [WARN] [1769546852.126881697] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[sync_slam_toolbox_node-7] [INFO] [1769546852.131335795] [slam_toolbox]: Node using stack size 40000000
[sync_slam_toolbox_node-7] [INFO] [1769546852.194396346] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[sync_slam_toolbox_node-7] [INFO] [1769546852.195994928] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[sllidar_node-4] [INFO] [1769546852.316191149] [sllidar_node]: current scan mode: Boost, sample rate: 8 Khz, max_distance: 12.0 m, scan frequency:10.0 Hz, 
[rviz2-6] [INFO] [1769546852.595648474] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-6] [INFO] [1769546852.595739098] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-6] [INFO] [1769546852.622464502] [rviz2]: Stereo is NOT SUPPORTED
[stretch_driver-3] [INFO] [1769546853.605603697] [stretch_driver]: For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.
[stretch_driver-3] [INFO] [1769546853.605900076] [stretch_driver]: stretch_driver started
[sync_slam_toolbox_node-7] [INFO] [1769546853.788613646] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546852.316 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546853.913599564] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546853.652 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.048542124] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546853.787 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.183824663] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546853.909 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.303750180] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.044 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.439038381] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.180 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.573655795] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.302 for reason 'discarding message because the queue is full'
[stretch_driver-3] Another process is already using Stretch. Try running "stretch_free_robot_process.py"
[stretch_driver-3] [FATAL] [1769546854.624987556] [stretch_driver]: Robot startup failed.
[sync_slam_toolbox_node-7] [INFO] [1769546854.698725162] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.438 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.833873408] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.572 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546854.953572389] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.695 for reason 'discarding message because the queue is full'
[INFO] [stretch_driver-3]: process has finished cleanly [pid 12056]
[rviz2-6] [INFO] [1769546854.962659115] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546852.316 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.088779701] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.830 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.090707423] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546853.652 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.228856366] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546854.953 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.250572108] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546853.787 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.346851263] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546853.909 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.349084871] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.088 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.483859415] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.224 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.506943247] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.044 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.619364574] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.346 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.634888062] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.180 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.744315700] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.481 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.763000811] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.302 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546855.878699153] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.617 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546855.890816188] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.438 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.014201360] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.739 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.019562354] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.572 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.133878048] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546855.874 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.146747556] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.695 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.268640458] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.010 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.274957035] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.830 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.393697176] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.132 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.403194312] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546854.953 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.528590081] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.267 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.530538299] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.088 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.663554535] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.390 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.691010365] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.224 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.783832518] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.525 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.786905294] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.346 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546856.919000813] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.660 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546856.947148040] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.481 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.059093186] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.783 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.074846053] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.617 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.179085478] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546856.918 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.203498014] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.739 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.314053968] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.054 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.330807087] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546855.874 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.448597157] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.176 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.459361192] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.010 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.574156406] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.312 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.587683682] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.132 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.708764852] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.447 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.715476827] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.267 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.843261093] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.390 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.843818652] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.570 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546857.964125416] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.705 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546857.971000677] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.525 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.098608304] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.660 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.099090969] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.840 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.224420095] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546857.963 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.227607593] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.783 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.358961781] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.098 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.387130406] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546856.918 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.494062261] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.221 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.515165124] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.054 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.613836999] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.356 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.643014406] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.176 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.748669484] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.491 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.771425655] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.312 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546858.889164932] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.613 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546858.898958488] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.447 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.008726631] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.748 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.027215425] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.570 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.143892246] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546858.885 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.155270988] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.705 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.278594129] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.006 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.283278146] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.840 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.403655421] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.141 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.410571350] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546857.963 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.538744564] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.276 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.539415487] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.098 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.673970531] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.400 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.699512272] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.221 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.793636199] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.534 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.794859976] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.356 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546859.928692072] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.670 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546859.954678771] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.491 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.051318331] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.613 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.053883078] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.792 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.188820072] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546859.928 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.211342317] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.748 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.324019801] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.050 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.339501105] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546858.885 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.449009595] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.185 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.467069233] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.006 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.583822569] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.321 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.594762057] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.141 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.718635952] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.444 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.722875006] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.276 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.838917207] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.579 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.850716575] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.400 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546860.973758095] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.714 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546860.978892491] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.534 for reason 'discarding message because the queue is full'
[sync_slam_toolbox_node-7] [INFO] [1769546861.109624638] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1769546860.837 for reason 'discarding message because the queue is full'
[rviz2-6] [INFO] [1769546861.139295982] [rviz]: Message Filter dropping message: frame 'laser' at time 1769546859.670 for reason 'discarding message because the queue is full'
^C[rviz2-6] [INFO] [1769546861.142592302] [rclcpp]: signal_handler(signum=2)
[scan_to_scan_filter_chain-5] [INFO] [1769546861.142602529] [rclcpp]: signal_handler(signum=2)
[sync_slam_toolbox_node-7] [INFO] [1769546861.142662780] [rclcpp]: signal_handler(signum=2)
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 12054]
[INFO] [sync_slam_toolbox_node-7]: process has finished cleanly [pid 12064]
[INFO] [scan_to_scan_filter_chain-5]: process has finished cleanly [pid 12060]
[INFO] [joint_state_publisher-1]: process has finished cleanly [pid 12052]
[sllidar_node-4] [INFO] [1769546861.329507892] [sllidar_node]: Stop motor
[INFO] [sllidar_node-4]: process has finished cleanly [pid 12058]

hello-robot@stretch-se3-3103:~$ lsusb
Bus 004 Device 003: ID 05e3:0626 Genesys Logic, Inc. USB3.1 Hub
Bus 004 Device 002: ID 0bda:0411 Realtek Semiconductor Corp. Hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 005: ID 2341:804d Arduino SA Hello Stepper
Bus 003 Device 022: ID 045e:028e Microsoft Corp. Xbox360 Controller
Bus 003 Device 021: ID 046d:c52b Logitech, Inc. Unifying Receiver
Bus 003 Device 020: ID 2109:2817 VIA Labs, Inc. USB2.0 Hub             
Bus 003 Device 015: ID 2341:804d Arduino SA Hello Stepper
Bus 003 Device 012: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 003 Device 019: ID 2341:804d Arduino SA Hello Wacc
Bus 003 Device 018: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
Bus 003 Device 017: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405 
Bus 003 Device 016: ID 1a40:0101 Terminus Technology Inc. Hub
Bus 003 Device 013: ID 2341:804d Arduino SA Hello Stepper
Bus 003 Device 010: ID 1a40:0101 Terminus Technology Inc. Hub
Bus 003 Device 006: ID 2341:804d Arduino SA Hello Stepper
Bus 003 Device 003: ID 05e3:0610 Genesys Logic, Inc. Hub
Bus 003 Device 009: ID 8087:0033 Intel Corp. 
Bus 003 Device 007: ID 2341:804d Arduino SA Hello Pimu
Bus 003 Device 014: ID 2886:0018 Seeed Technology Co., Ltd. ReSpeaker 4 Mic Array (UAC1.0)
Bus 003 Device 011: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC
Bus 003 Device 008: ID 0c45:6366 Microdia Webcam Vitade AF
Bus 003 Device 004: ID 1a40:0101 Terminus Technology Inc. Hub
Bus 003 Device 002: ID 0bda:5411 Realtek Semiconductor Corp. RTS5411 Hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 003: ID 2109:0817 VIA Labs, Inc. USB3.0 Hub             
Bus 002 Device 002: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435if
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
hello-robot@stretch-se3-3103:~$ lsusb -t
/:  Bus 04.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 10000M
    |__ Port 1: Dev 2, If 0, Class=Hub, Driver=hub/4p, 5000M
    |__ Port 2: Dev 3, If 0, Class=Hub, Driver=hub/4p, 5000M
/:  Bus 03.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/12p, 480M
    |__ Port 1: Dev 2, If 0, Class=Hub, Driver=hub/5p, 480M
        |__ Port 2: Dev 7, If 1, Class=CDC Data, Driver=cdc_acm, 12M
        |__ Port 2: Dev 7, If 0, Class=Communications, Driver=cdc_acm, 12M
        |__ Port 1: Dev 4, If 0, Class=Hub, Driver=hub/4p, 480M
            |__ Port 3: Dev 14, If 4, Class=Application Specific Interface, Driver=, 12M
            |__ Port 3: Dev 14, If 2, Class=Audio, Driver=snd-usb-audio, 12M
            |__ Port 3: Dev 14, If 0, Class=Audio, Driver=snd-usb-audio, 12M
            |__ Port 3: Dev 14, If 3, Class=Vendor Specific Class, Driver=, 12M
            |__ Port 3: Dev 14, If 1, Class=Audio, Driver=snd-usb-audio, 12M
            |__ Port 1: Dev 8, If 1, Class=Video, Driver=uvcvideo, 480M
            |__ Port 1: Dev 8, If 0, Class=Video, Driver=uvcvideo, 480M
            |__ Port 2: Dev 11, If 0, Class=Vendor Specific Class, Driver=ftdi_sio, 12M
    |__ Port 2: Dev 3, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 1: Dev 6, If 1, Class=CDC Data, Driver=cdc_acm, 12M
        |__ Port 1: Dev 6, If 0, Class=Communications, Driver=cdc_acm, 12M
        |__ Port 2: Dev 10, If 0, Class=Hub, Driver=hub/4p, 480M
            |__ Port 3: Dev 16, If 0, Class=Hub, Driver=hub/4p, 480M
                |__ Port 3: Dev 19, If 1, Class=CDC Data, Driver=cdc_acm, 12M
                |__ Port 3: Dev 19, If 0, Class=Communications, Driver=cdc_acm, 12M
                |__ Port 1: Dev 17, If 2, Class=Video, Driver=uvcvideo, 480M
                |__ Port 1: Dev 17, If 0, Class=Video, Driver=uvcvideo, 480M
                |__ Port 1: Dev 17, If 3, Class=Video, Driver=uvcvideo, 480M
                |__ Port 1: Dev 17, If 1, Class=Video, Driver=uvcvideo, 480M
                |__ Port 2: Dev 18, If 0, Class=Vendor Specific Class, Driver=ftdi_sio, 12M
            |__ Port 2: Dev 13, If 1, Class=CDC Data, Driver=cdc_acm, 12M
            |__ Port 2: Dev 13, If 0, Class=Communications, Driver=cdc_acm, 12M
        |__ Port 3: Dev 12, If 0, Class=Vendor Specific Class, Driver=cp210x, 12M
        |__ Port 4: Dev 15, If 1, Class=CDC Data, Driver=cdc_acm, 12M
        |__ Port 4: Dev 15, If 0, Class=Communications, Driver=cdc_acm, 12M
    |__ Port 5: Dev 20, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 3: Dev 22, If 0, Class=Vendor Specific Class, Driver=xpad, 12M
        |__ Port 1: Dev 21, If 1, Class=Human Interface Device, Driver=usbhid, 12M
        |__ Port 1: Dev 21, If 2, Class=Human Interface Device, Driver=usbhid, 12M
        |__ Port 1: Dev 21, If 0, Class=Human Interface Device, Driver=usbhid, 12M
    |__ Port 7: Dev 5, If 0, Class=Communications, Driver=cdc_acm, 12M
    |__ Port 7: Dev 5, If 1, Class=CDC Data, Driver=cdc_acm, 12M
    |__ Port 10: Dev 9, If 0, Class=Wireless, Driver=btusb, 12M
    |__ Port 10: Dev 9, If 1, Class=Wireless, Driver=btusb, 12M
/:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/3p, 20000M/x2
    |__ Port 1: Dev 2, If 4, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 1: Dev 2, If 2, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 1: Dev 2, If 0, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 1: Dev 2, If 5, Class=Human Interface Device, Driver=usbhid, 5000M
    |__ Port 1: Dev 2, If 3, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 1: Dev 2, If 1, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 3: Dev 3, If 0, Class=Hub, Driver=hub/4p, 5000M
/:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 480M
hello-robot@stretch-se3-3103:~$  rs-enumerate-devices
Device info: 
    Name                          : 	Intel RealSense D435IF
    Serial Number                 : 	246422071246
    Firmware Version              : 	5.16.0.1
    Recommended Firmware Version  : 	5.16.0.1
    Physical Port                 : 	/sys/devices/pci0000:00/0000:00:0d.0/usb2/2-1/2-1:1.0/video4linux/video0
    Debug Op Code                 : 	15
    Advanced Mode                 : 	YES
    Product Id                    : 	0B3A
    Camera Locked                 : 	YES
    Usb Type Descriptor           : 	3.2
    Product Line                  : 	D400
    Asic Serial Number            : 	337543061857
    Firmware Update Id            : 	337543061857
    Dfu Device Path               : 	

Stream Profiles supported by Stereo Module
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Infrared 1  1280x800       Y16         @ 25/15 Hz
    Infrared 1      |          Y8          @ 30/15 Hz
    Infrared 1  1280x720       Y8          @ 30/15/6 Hz
    Infrared 1   848x480       Y8          @ 90/60/30/15/6 Hz
    Infrared 1   848x100       Y8          @ 300/100 Hz
    Infrared 1   640x480       Y8          @ 90/60/30/15/6 Hz
    Infrared 1   640x400       Y16         @ 25/15 Hz
    Infrared 1   640x360       Y8          @ 90/60/30/15/6 Hz
    Infrared 1   480x270       Y8          @ 90/60/30/15/6 Hz
    Infrared 1   424x240       Y8          @ 90/60/30/15/6 Hz
    Infrared 2  1280x800       Y16         @ 25/15 Hz
    Infrared 2      |          Y8          @ 30/15 Hz
    Infrared 2  1280x720       Y8          @ 30/15/6 Hz
    Infrared 2   848x480       Y8          @ 90/60/30/15/6 Hz
    Infrared 2   848x100       Y8          @ 300/100 Hz
    Infrared 2   640x480       Y8          @ 90/60/30/15/6 Hz
    Infrared 2   640x400       Y16         @ 25/15 Hz
    Infrared 2   640x360       Y8          @ 90/60/30/15/6 Hz
    Infrared 2   480x270       Y8          @ 90/60/30/15/6 Hz
    Infrared 2   424x240       Y8          @ 90/60/30/15/6 Hz
    Depth       1280x720       Z16         @ 30/15/6 Hz
    Depth        848x480       Z16         @ 90/60/30/15/6 Hz
    Depth        848x100       Z16         @ 300/100 Hz
    Depth        640x480       Z16         @ 90/60/30/15/6 Hz
    Depth        640x360       Z16         @ 90/60/30/15/6 Hz
    Depth        480x270       Z16         @ 90/60/30/15/6 Hz
    Depth        424x240       Z16         @ 90/60/30/15/6 Hz
    Depth        256x144       Z16         @ 300/90 Hz

Stream Profiles supported by RGB Camera
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Color       1920x1080      RGB8        @ 30/15/6 Hz
    Color           |          Y8          @ 30/15/6 Hz
    Color           |          BGRA8       @ 30/15/6 Hz
    Color           |          RGBA8       @ 30/15/6 Hz
    Color           |          BGR8        @ 30/15/6 Hz
    Color           |          YUYV        @ 30/15/6 Hz
    Color       1280x720       RGB8        @ 30/15/6 Hz
    Color           |          Y8          @ 30/15/6 Hz
    Color           |          BGRA8       @ 30/15/6 Hz
    Color           |          RGBA8       @ 30/15/6 Hz
    Color           |          BGR8        @ 30/15/6 Hz
    Color           |          YUYV        @ 30/15/6 Hz
    Color        960x540       RGB8        @ 60/30/15/6 Hz
    Color           |          Y8          @ 60/30/15/6 Hz
    Color           |          BGRA8       @ 60/30/15/6 Hz
    Color           |          RGBA8       @ 60/30/15/6 Hz
    Color           |          BGR8        @ 60/30/15/6 Hz
    Color           |          YUYV        @ 60/30/15/6 Hz
    Color        848x480       RGB8        @ 60/30/15/6 Hz
    Color           |          Y8          @ 60/30/15/6 Hz
    Color           |          BGRA8       @ 60/30/15/6 Hz
    Color           |          RGBA8       @ 60/30/15/6 Hz
    Color           |          BGR8        @ 60/30/15/6 Hz
    Color           |          YUYV        @ 60/30/15/6 Hz
    Color        640x480       RGB8        @ 60/30/15/6 Hz
    Color           |          Y8          @ 60/30/15/6 Hz
    Color           |          BGRA8       @ 60/30/15/6 Hz
    Color           |          RGBA8       @ 60/30/15/6 Hz
    Color           |          BGR8        @ 60/30/15/6 Hz
    Color           |          YUYV        @ 60/30/15/6 Hz
    Color        640x360       RGB8        @ 60/30/15/6 Hz
    Color           |          Y8          @ 60/30/15/6 Hz
    Color           |          BGRA8       @ 60/30/15/6 Hz
    Color           |          RGBA8       @ 60/30/15/6 Hz
    Color           |          BGR8        @ 60/30/15/6 Hz
    Color           |          YUYV        @ 60/30/15/6 Hz
    Color        424x240       RGB8        @ 60/30/15/6 Hz
    Color           |          Y8          @ 60/30/15/6 Hz
    Color           |          BGRA8       @ 60/30/15/6 Hz
    Color           |          RGBA8       @ 60/30/15/6 Hz
    Color           |          BGR8        @ 60/30/15/6 Hz
    Color           |          YUYV        @ 60/30/15/6 Hz
    Color        320x240       RGB8        @ 60/30/6 Hz
    Color           |          Y8          @ 60/30/6 Hz
    Color           |          BGRA8       @ 60/30/6 Hz
    Color           |          RGBA8       @ 60/30/6 Hz
    Color           |          BGR8        @ 60/30/6 Hz
    Color           |          YUYV        @ 60/30/6 Hz
    Color        320x180       RGB8        @ 60/30/6 Hz
    Color           |          Y8          @ 60/30/6 Hz
    Color           |          BGRA8       @ 60/30/6 Hz
    Color           |          RGBA8       @ 60/30/6 Hz
    Color           |          BGR8        @ 60/30/6 Hz
    Color           |          YUYV        @ 60/30/6 Hz

Stream Profiles supported by Motion Module
 Supported modes:
    STREAM      FORMAT         FPS
    Accel       MOTION_XYZ32F  @ 200/100 Hz
    Gyro        MOTION_XYZ32F  @ 400/200 Hz

Device info: 
    Name                          : 	Intel RealSense D405
    Serial Number                 : 	218622272842
    Firmware Version              : 	5.12.14.100
    Recommended Firmware Version  : 	5.16.0.1
    Physical Port                 : 	/sys/devices/pci0000:00/0000:00:14.0/usb3/3-2/3-2.2/3-2.2.3/3-2.2.3.1/3-2.2.3.1:1.0/video4linux/video8
    Debug Op Code                 : 	15
    Advanced Mode                 : 	YES
    Product Id                    : 	0B5B
    Camera Locked                 : 	YES
    Usb Type Descriptor           : 	2.1
    Product Line                  : 	D400
    Asic Serial Number            : 	219323071944
    Firmware Update Id            : 	219323071944
    Dfu Device Path               : 	

Stream Profiles supported by Stereo Module
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Infrared    1280x720       UYVY        @ 5 Hz
    Infrared        |          BGRA8       @ 5 Hz
    Infrared        |          RGBA8       @ 5 Hz
    Infrared        |          BGR8        @ 5 Hz
    Infrared        |          RGB8        @ 5 Hz
    Infrared     848x480       UYVY        @ 10/5 Hz
    Infrared        |          BGRA8       @ 10/5 Hz
    Infrared        |          RGBA8       @ 10/5 Hz
    Infrared        |          BGR8        @ 10/5 Hz
    Infrared        |          RGB8        @ 10/5 Hz
    Infrared     640x480       UYVY        @ 30/15/5 Hz
    Infrared        |          BGRA8       @ 30/15/5 Hz
    Infrared        |          RGBA8       @ 30/15/5 Hz
    Infrared        |          BGR8        @ 30/15/5 Hz
    Infrared        |          RGB8        @ 30/15/5 Hz
    Infrared     640x360       UYVY        @ 30 Hz
    Infrared        |          BGRA8       @ 30 Hz
    Infrared        |          RGBA8       @ 30 Hz
    Infrared        |          BGR8        @ 30 Hz
    Infrared        |          RGB8        @ 30 Hz
    Infrared     480x270       UYVY        @ 60/30/15/5 Hz
    Infrared        |          BGRA8       @ 60/30/15/5 Hz
    Infrared        |          RGBA8       @ 60/30/15/5 Hz
    Infrared        |          BGR8        @ 60/30/15/5 Hz
    Infrared        |          RGB8        @ 60/30/15/5 Hz
    Infrared     256x144       UYVY        @ 90 Hz
    Infrared        |          BGRA8       @ 90 Hz
    Infrared        |          RGBA8       @ 90 Hz
    Infrared        |          BGR8        @ 90 Hz
    Infrared        |          RGB8        @ 90 Hz
    Infrared 1  1280x720       Y8          @ 5 Hz
    Infrared 1   848x480       Y8          @ 10/5 Hz
    Infrared 1   640x480       Y8          @ 30/15/5 Hz
    Infrared 1   640x360       Y8          @ 30 Hz
    Infrared 1   480x270       Y8          @ 60/30/15/5 Hz
    Infrared 1   256x144       Y8          @ 90 Hz
    Color       1280x720       RGB8        @ 15/10/5 Hz
    Color           |          Y8          @ 15/10/5 Hz
    Color           |          BGRA8       @ 15/10/5 Hz
    Color           |          RGBA8       @ 15/10/5 Hz
    Color           |          BGR8        @ 15/10/5 Hz
    Color           |          YUYV        @ 15/10/5 Hz
    Color        848x480       RGB8        @ 10/5 Hz
    Color           |          Y8          @ 10/5 Hz
    Color           |          BGRA8       @ 10/5 Hz
    Color           |          RGBA8       @ 10/5 Hz
    Color           |          BGR8        @ 10/5 Hz
    Color           |          YUYV        @ 10/5 Hz
    Color        640x480       RGB8        @ 30/15/5 Hz
    Color           |          Y8          @ 30/15/5 Hz
    Color           |          BGRA8       @ 30/15/5 Hz
    Color           |          RGBA8       @ 30/15/5 Hz
    Color           |          BGR8        @ 30/15/5 Hz
    Color           |          YUYV        @ 30/15/5 Hz
    Color        480x270       RGB8        @ 30/15/5 Hz
    Color           |          Y8          @ 30/15/5 Hz
    Color           |          BGRA8       @ 30/15/5 Hz
    Color           |          RGBA8       @ 30/15/5 Hz
    Color           |          BGR8        @ 30/15/5 Hz
    Color           |          YUYV        @ 30/15/5 Hz
    Color        424x240       RGB8        @ 60/30/15/5 Hz
    Color           |          Y8          @ 60/30/15/5 Hz
    Color           |          BGRA8       @ 60/30/15/5 Hz
    Color           |          RGBA8       @ 60/30/15/5 Hz
    Color           |          BGR8        @ 60/30/15/5 Hz
    Color           |          YUYV        @ 60/30/15/5 Hz
    Depth       1280x720       Z16         @ 5 Hz
    Depth        848x480       Z16         @ 10/5 Hz
    Depth        640x480       Z16         @ 30/15/5 Hz
    Depth        640x360       Z16         @ 30 Hz
    Depth        480x270       Z16         @ 60/30/15/5 Hz
    Depth        256x144       Z16         @ 90 Hz

hello-robot@stretch-se3-3103:~$ 
