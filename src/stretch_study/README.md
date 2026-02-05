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


ros2 topic pub --once /core_body std_msgs/msg/String \
'{"data":"{\"text\":\"Hello, I am using af_heart.\",\"volume\":70,\"voice\":\"af_heart\",\"interrupt\":true}"}'