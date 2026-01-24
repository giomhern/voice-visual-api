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