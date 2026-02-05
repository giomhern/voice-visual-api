# Voice vs Visual Interfaces API

This repository contains the source code all of the movement and demos for a research project 
comparing user interactions with Hello-Robot's Stretch in the midst of different modalities. 


## Helpful Guide for Interaction Setup 

1. For navigation, run the FUNMAP command:

```bash
ros2 launch stretch_funmap mapping.launch.py map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_20260202160851
```

2. Run the command to initiate teleop and proceed to move the base to where we want it to be:

```bash
ros2 run stretch_core keyboard_teleop --ros-args -p mapping_on:=True
```

3. Localize Stretch so he can navigate to the correct goal poses:

```bash
ros2 service call /funmap/trigger_global_localization std_srvs/srv/Trigger {}
ros2 service call /funmap/trigger_local_localization std_srvs/srv/Trigger {}
```

4. Navigate to the stretch_study package repository, build package and source:

```bash
cd voice-visual-api/src 
colcon build
source install/setup.bash
```

5. In different terminals run the following commands:

```bash
ros2 run stretch_study study_engine \
  --ros-args \
  -p nav_enable:=true \
  -p nav.goals_yaml:=/home/hello-robot/voice-visual-api/src/stretch_study/stretch_study/config/goals/goals.yaml \
  -p motion.scaler_mode:=/cmd_vel_scaler

ros2 run stretch_study keyboard_teleop
```




"""
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger

ros2 action send_goal /stretch_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint_lift', 'wrist_extension', 'joint_wrist_yaw'], points: [{positions: [0.5, 0.1, 0.0], time_from_start: {sec: 2}}]}}"
"""

$ ros2 action send_goal /stretch_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint_lift', 'wrist_extension', 'joint_wrist_yaw'], points: [{positions: [0.5, 0.1, 0.0], time_from_start: {sec: 2}}]}}"
usage: ros2 action send_goal [-h] [-f] [-t N] action_name action_type goal
ros2 action send_goal: error: the following arguments are required: goal
