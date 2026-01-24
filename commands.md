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
