# stretch_study (ROS 2)

Tier 1 study controller for Stretch 3 (Hello Robot) on ROS 2:
- **Manual station registration** (participant/RA indicates desk/bed/kitchen)
- **Deterministic scripted demos** (gesture demos by default)
- Modality-agnostic: voice / visual / hybrid adapters all publish the same `StudyEvent`

## What you get
- `study_engine` node: runs the canonical interaction script as a step-by-step state machine
- `keyboard_teleop` adapter: fast manual testing / RA override via terminal
- `event_cli` adapter: send a single event from CLI and exit

## Topics
- **Input:** `/study_event` (`std_msgs/String`) — JSON StudyEvents
- **Output:** `/study_prompt` (`std_msgs/String`) — JSON prompts for UI / speech

## Build
In a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -R stretch_study_pkg ~/ros2_ws/src/stretch_study
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

> Note: the folder name in `src` should be `stretch_study` (package name).

## Run
Terminal 1 (study engine):

```bash
ros2 run stretch_study study_engine --ros-args \
  -p study.session_id:=session_001 \
  -p study.participant_id:=p001
```

Terminal 2 (keyboard teleop):

```bash
ros2 run stretch_study keyboard_teleop
```

## Optional deterministic base transit (odom-based)

For controlled demo testing, the package can optionally command the Stretch base to
move along an assumed rectangle route: **door -> desk -> bed -> kitchen**.

This uses odometry (`/odom`) and publishes velocity commands (`/stretch/cmd_vel` by default).

Enable it by setting ROS params (either in `defaults.yaml` or via `--ros-args -p`):

```bash
ros2 run stretch_study study_engine --ros-args \
  -p motion.enable_transit:=true \
  -p motion.distances_m.door_to_desk:=3.0 \
  -p motion.distances_m.desk_to_bed:=2.5 \
  -p motion.distances_m.bed_to_kitchen:=3.0
```

Then, when you type `1` (arrive desk), `2` (arrive bed), `3` (arrive kitchen) in `keyboard_teleop`,
the engine will attempt to execute the corresponding transit leg.

## Quick test flow
1. In keyboard teleop: `a` to advance greeting
2. Set globals with `g ...` commands
3. `a` to finish globals
4. `1` arrive desk, set room overrides with `r desk ...`, then `a` and `y desk`
5. Repeat for bed (`2`) and kitchen (`3`)

## Integrating real Stretch motion
Replace the gesture demo bodies in:
- `stretch_study/capabilities/deterministic_demos.py`

with calls into your Stretch ROS 2 stack (arm trajectories, base motions, etc.).

```
ros2 run stretch_study study_engine --ros-args \
  -p motion.enable_transit:=true \
  -p motion.cmd_vel_topic:=/stretch/cmd_vel \
  -p motion.odom_topic:=/odom \
  -p motion.distances_m.door_to_desk:=0.20 \
  -p motion.distances_m.desk_to_bed:=0.20 \
  -p motion.distances_m.bed_to_kitchen:=0.20
```


Error output
```

ros2 launch stretch_funmap mapping.launch.py
[INFO] [launch]: All log files can be found below /home/hello-robot/.ros/log/2026-01-20-15-58-59-136225-stretch-se3-3103-6357
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: D435i launched in high resolution
[INFO] [realsense2_camera_node-1]: process started with pid [6359]
[INFO] [stretch_driver-5]: process started with pid [6367]
[INFO] [d435i_accel_correction-2]: process started with pid [6361]
[INFO] [joint_state_publisher-3]: process started with pid [6363]
[INFO] [robot_state_publisher-4]: process started with pid [6365]
[INFO] [funmap-6]: process started with pid [6369]
[INFO] [rviz2-7]: process started with pid [6371]
[d435i_accel_correction-2] [INFO] [1768946339.731471413] [d435i_accel_correction_node]: d435i_accel_correction_node started
[rviz2-7] [INFO] [1768946339.760938725] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1768946339.761027088] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-7] [INFO] [1768946339.780920226] [rviz2]: Stereo is NOT SUPPORTED
[realsense2_camera_node-1] [INFO] [1768946339.927739022] [camera]: RealSense ROS v4.55.1
[realsense2_camera_node-1] [INFO] [1768946339.927844513] [camera]: Built with LibRealSense v2.55.1
[realsense2_camera_node-1] [INFO] [1768946339.927859188] [camera]: Running with LibRealSense v2.55.1
[realsense2_camera_node-1] [INFO] [1768946339.971228965] [camera]: Device with serial number 246422071246 was found.
[realsense2_camera_node-1] 
[realsense2_camera_node-1] [INFO] [1768946339.971337490] [camera]: Device with physical ID /sys/devices/pci0000:00/0000:00:0d.0/usb2/2-1/2-1:1.0/video4linux/video0 was found.
[realsense2_camera_node-1] [INFO] [1768946339.971345097] [camera]: Device with name Intel RealSense D435IF was found.
[realsense2_camera_node-1] [INFO] [1768946339.971489611] [camera]: Device with port number 2-1 was found.
[realsense2_camera_node-1] [INFO] [1768946339.971501211] [camera]: Device USB type: 3.2
[realsense2_camera_node-1] [INFO] [1768946339.971507202] [camera]: Resetting device...
[funmap-6] Traceback (most recent call last):
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/stretch_funmap/funmap", line 33, in <module>
[funmap-6]     sys.exit(load_entry_point('stretch-funmap==0.0.0', 'console_scripts', 'funmap')())
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/stretch_funmap/funmap", line 25, in importlib_load_entry_point
[funmap-6]     return next(matches).load()
[funmap-6]   File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
[funmap-6]     module = import_module(match.group('module'))
[funmap-6]   File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
[funmap-6]     return _bootstrap._gcd_import(name[level:], package, level)
[funmap-6]   File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
[funmap-6]   File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
[funmap-6]   File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
[funmap-6]   File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
[funmap-6]   File "<frozen importlib._bootstrap_external>", line 883, in exec_module
[funmap-6]   File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/funmap.py", line 35, in <module>
[funmap-6]     from . import mapping as ma
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/mapping.py", line 18, in <module>
[funmap-6]     from . import ros_max_height_image as rm
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/ros_max_height_image.py", line 17, in <module>
[funmap-6]     from . import navigation_planning as na
[funmap-6]   File "/home/hello-robot/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/navigation_planning.py", line 7, in <module>
[funmap-6]     from . import cython_min_cost_path as cm
[funmap-6] ImportError: cannot import name 'cython_min_cost_path' from 'stretch_funmap' (/home/hello-robot/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/__init__.py)
[ERROR] [funmap-6]: process has died [pid 6369, exit code 1, cmd '/home/hello-robot/ament_ws/install/stretch_funmap/lib/stretch_funmap/funmap --ros-args --params-file /tmp/launch_params_fn89sfbc -r /move_base_simple/goal:=/goal_pose'].
[stretch_driver-5] [INFO] [1768946341.223301715] [stretch_driver]: For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.
[stretch_driver-5] [INFO] [1768946341.223541733] [stretch_driver]: stretch_driver started
[stretch_driver-5] Key mapped to End-Of-Arm Tool: eoa_wrist_dw3_tool_sg3
[stretch_driver-5] [INFO] [1768946342.952752743] [stretch_driver]: Changed to mode = position
[stretch_driver-5] [INFO] [1768946342.953791609] [stretch_driver]: broadcast_odom_tf = True
[stretch_driver-5] [INFO] [1768946342.984789306] [stretch_driver]: rate = 30.0 Hz
[stretch_driver-5] [INFO] [1768946342.985204989] [stretch_driver]: twist timeout = 0.5 s
[stretch_driver-5] [INFO] [1768946342.985921244] [stretch_driver]: base_frame_id = base_link
[stretch_driver-5] [INFO] [1768946342.988392193] [stretch_driver]: odom_frame_id = odom
[realsense2_camera_node-1] [INFO] [1768946346.015700461] [camera]: Device with serial number 246422071246 was found.
[realsense2_camera_node-1] 
[realsense2_camera_node-1] [INFO] [1768946346.015752489] [camera]: Device with physical ID /sys/devices/pci0000:00/0000:00:0d.0/usb2/2-1/2-1:1.0/video4linux/video0 was found.
[realsense2_camera_node-1] [INFO] [1768946346.015758831] [camera]: Device with name Intel RealSense D435IF was found.
[realsense2_camera_node-1] [INFO] [1768946346.015885523] [camera]: Device with port number 2-1 was found.
[realsense2_camera_node-1] [INFO] [1768946346.015895910] [camera]: Device USB type: 3.2
[realsense2_camera_node-1] [INFO] [1768946346.015932069] [camera]: getParameters...
[realsense2_camera_node-1] [INFO] [1768946346.392878037] [camera]: JSON file is loaded! (/home/hello-robot/ament_ws/install/stretch_core/share/stretch_core/config/HighAccuracyPreset.json)
[realsense2_camera_node-1] [INFO] [1768946346.393070777] [camera]: Device Name: Intel RealSense D435IF
[realsense2_camera_node-1] [INFO] [1768946346.393111804] [camera]: Device Serial No: 246422071246
[realsense2_camera_node-1] [INFO] [1768946346.393312760] [camera]: Device physical port: /sys/devices/pci0000:00/0000:00:0d.0/usb2/2-1/2-1:1.0/video4linux/video0
[realsense2_camera_node-1] [INFO] [1768946346.393362383] [camera]: Device FW version: 5.16.0.1
[realsense2_camera_node-1] [INFO] [1768946346.393384293] [camera]: Device Product ID: 0x0B3A
[realsense2_camera_node-1] [INFO] [1768946346.393402773] [camera]: Sync Mode: On
[realsense2_camera_node-1] [WARN] [1768946346.524208680] [camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [WARN] [1768946346.524863563] [camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [WARN] [1768946346.531390507] [camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [ERROR] [1768946346.531943082] [camera]: Given value, 63 is invalid. Set ROS param back to: 100
[realsense2_camera_node-1] [INFO] [1768946346.536379607] [camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1768946346.536447722] [camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1768946346.536463361] [camera]: Stopping Sensor: Motion Module
[realsense2_camera_node-1] [INFO] [1768946346.552326743] [camera]: Starting Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1768946346.555650600] [camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 1280, Height: 720, FPS: 15
[realsense2_camera_node-1] [INFO] [1768946346.558580037] [camera]: Starting Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1768946346.560724971] [camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 1280, Height: 720, FPS: 15
[realsense2_camera_node-1] [INFO] [1768946346.590221686] [camera]: Starting Sensor: Motion Module
[realsense2_camera_node-1] [INFO] [1768946346.592336360] [camera]: Open profile: stream_type: Accel(0)Format: MOTION_XYZ32F, FPS: 100
[realsense2_camera_node-1] [INFO] [1768946346.592390213] [camera]: Open profile: stream_type: Gyro(0)Format: MOTION_XYZ32F, FPS: 200
[realsense2_camera_node-1] [INFO] [1768946346.595411448] [camera]: RealSense Node Is Up!
[realsense2_camera_node-1] [WARN] [1768946347.552837406] [camera]: XXX Hardware Notification:Motion Module failure,1.76895e+12,Error,Hardware Error
[realsense2_camera_node-1] [WARN] [1768946347.552893655] [camera]: Hardware Notification:Motion Module failure,1.76895e+12,Error,Hardware Error


```