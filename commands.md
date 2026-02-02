ros2 topic pub --once /speech_request std_msgs/msg/String \
'{"data":"{\"text\":\"Hello. This is Stretch speaking.\",\"volume\":60,\"rate\":170,\"voice\":\"auto\"}"}'

ros2 run stretch_study study_engine --ros-args --log-level INFO \
  -p speech.enable:=true \
  -p nav.enable:=false \
  -p motion.enable_transit:=false

curl -s http://127.0.0.1:11434/api/chat \
  -H "Content-Type: application/json" \
  -d '{"model":"llama3.1:8b","messages":[{"role":"user","content":"ping"}],"stream":false}' | head -c 300; echo

  ros2 run your_pkg voice_assistant_node.py --ros-args \
  -p ollama.url:=http://127.0.0.1:11434/api/chat \
  -p ollama.model:=llama3.1:8b


  ros2 run stretch_study study_engine \
  --ros-args \
  -p nav.enable:=true \
  -p nav.goals_yaml:=/absolute/path/to/goals.yaml \
  -p motion.scaler_node:=/cmd_vel_scaler


ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {z: 0.0, w: 1.0}
  }
}"