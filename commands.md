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



  thello-robot@stretch-se3-3103:~$ ros2 topic list 
/battery
/camera/accel/imu_info
/camera/accel/metadata
/camera/accel/sample
/camera/accel/sample_corrected
/camera/aligned_depth_to_color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/image_raw/compressed
/camera/aligned_depth_to_color/image_raw/compressedDepth
/camera/aligned_depth_to_color/image_raw/theora
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/theora
/camera/color/metadata
/camera/depth/camera_info
/camera/depth/color/points
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/theora
/camera/depth/metadata
/camera/extrinsics/depth_to_accel
/camera/extrinsics/depth_to_color
/camera/extrinsics/depth_to_depth
/camera/extrinsics/depth_to_gyro
/camera/gyro/imu_info
/camera/gyro/metadata
/camera/gyro/sample
/clicked_point
/diagnostics
/funmap/marker_array
/funmap/navigation_plan_markers
/funmap/obstacle_point_cloud2
/funmap/point_cloud2
/funmap/voi_marker
/funmap/voi_marker_array
/gamepad_joy
/goal_pose
/imu_mobile_base
/imu_wrist
/initialpose
/is_gamepad_dongle
/is_homed
/is_runstopped
/is_streaming_position
/joint_limits
/joint_pose_cmd
/joint_states
/magnetometer_mobile_base
/mode
/odom
/parameter_events
/robot_description
/rosout
/stretch/cmd_vel
/stretch/joint_states
/stretch_gamepad_state
/tf
/tf_static
/tool
hello-robot@stretch-se3-3103:~$ ros2 action list
/move_base
/stretch_controller/follow_joint_trajectory
hello-robot@stretch-se3-3103:~$ ros2 service listy
usage: ros2 service [-h] [--include-hidden-services]
                    Call `ros2 service <command> -h` for more detailed usage.
                    ...
ros2 service: error: argument Call `ros2 service <command> -h` for more detailed usage.: invalid choice: 'listy' (choose from 'call', 'find', 'list', 'type')
hello-robot@stretch-se3-3103:~$ ros2 service list
/activate_streaming_position
/camera/describe_parameters
/camera/device_info
/camera/get_parameter_types
/camera/get_parameters
/camera/list_parameters
/camera/set_parameters
/camera/set_parameters_atomically
/d435i_accel_correction_node/describe_parameters
/d435i_accel_correction_node/get_parameter_types
/d435i_accel_correction_node/get_parameters
/d435i_accel_correction_node/list_parameters
/d435i_accel_correction_node/set_parameters
/d435i_accel_correction_node/set_parameters_atomically
/deactivate_streaming_position
/funmap/describe_parameters
/funmap/get_parameter_types
/funmap/get_parameters
/funmap/list_parameters
/funmap/set_parameters
/funmap/set_parameters_atomically
/funmap/trigger_align_with_nearest_cliff
/funmap/trigger_drive_to_scan
/funmap/trigger_global_localization
/funmap/trigger_head_scan
/funmap/trigger_local_localization
/funmap/trigger_lower_until_contact
/funmap/trigger_reach_until_contact
/get_joint_states
/home_the_robot
/joint_state_publisher/describe_parameters
/joint_state_publisher/get_parameter_types
/joint_state_publisher/get_parameters
/joint_state_publisher/list_parameters
/joint_state_publisher/set_parameters
/joint_state_publisher/set_parameters_atomically
/keyboard_teleop/describe_parameters
/keyboard_teleop/get_parameter_types
/keyboard_teleop/get_parameters
/keyboard_teleop/list_parameters
/keyboard_teleop/set_parameters
/keyboard_teleop/set_parameters_atomically
/make_plan
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/runstop
/rviz/describe_parameters
/rviz/get_parameter_types
/rviz/get_parameters
/rviz/list_parameters
/rviz/set_parameters
/rviz/set_parameters_atomically
/self_collision_avoidance
/stop_the_robot
/stow_the_robot
/stretch_driver/describe_parameters
/stretch_driver/get_parameter_types
/stretch_driver/get_parameters
/stretch_driver/list_parameters
/stretch_driver/set_parameters
/stretch_driver/set_parameters_atomically
/switch_to_gamepad_mode
/switch_to_navigation_mode
/switch_to_position_mode
/switch_to_trajectory_mode
