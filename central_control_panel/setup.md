# Quick Setup Guide 

Below is a step-by-step guide to setup Stretch for interaction study. 

## Commands to Run


1. Stretch driver (brings up base + cmd_vel interface)
```
source /opt/ros/humble/setup.bash
ros2 launch stretch_core stretch_driver.launch.py
```

2. In a second terminal, run the following:

```
source /opt/ros/humble/setup.bash

# If homing ever fails with runstopped, do this first:
ros2 service call /enable_runstop std_srvs/srv/Trigger

# Then home:
ros2 service call /home_the_robot std_srvs/srv/Trigger

# Then nav mode:
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger

```

3. In third terminal, run the study controller:

```
source /opt/ros/humble/setup.bash
python3 stretch_study_controller.py --ros-args -p cmd_vel_topic:=/stretch/cmd_vel
```

4. Initiate the camera in a fourth terminal:

```
ros2 launch realsense2_camera rs_launch.py

```

## Demos 

To test out the vision follow demo, run the following command:

```
python3 vision_follow_demo.py --ros-args   -p cmd_vel_topic:=/stretch/cmd_vel   -p rgb_topic:=/camera/camera/color/image_raw   -p depth_topic:=/camera/camera/depth/image_rect_raw   -p distance:=medium
```

```
ros2 service call /activate_streaming_position std_srvs/srv/Trigger "{}"

```

