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

Ensuring correct version of ROS is sourced...
You are about to delete and replace the existing ament workspace. If you have any personal data in the workspace, please create a back up before proceeding.
Do you want to continue? Press (y/n for yes/no): yes
Press 'y' for yes or 'n' for no.
Do you want to continue? Press (y/n for yes/no): y
Continuing to create a new ament workspace.
Downgrade to numpy 1.26.4...
Updating rosdep indices...
Deleting /home/hello-robot/ament_ws if it already exists...
Creating the workspace directory...
Cloning the workspace's packages...
Fetch ROS packages' dependencies (this might take a while)...

#############################################
FAILURE. UPDATING ROS WORKSPACE DID NOT COMPLETE.
Look at the troubleshooting guide for solutions to common issues: https://docs.hello-robot.com/0.3/installation/ros_workspace/
or contact Hello Robot support
#############################################

```


 cat ~/stretch_user/log/stretch_create_ament_workspace.202601221434_redirected.txt | head -n 60
Defaulting to user installation because normal site-packages is not writeable
Requirement already satisfied: numpy==1.26.4 in /home/hello-robot/.local/lib/python3.10/site-packages (1.26.4)
WARNING: Error parsing dependencies of flatbuffers: Invalid version: '1.12.1-git20200711.33e2d80-dfsg1-0.6'
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Add distro "ardent"
Add distro "bouncy"
Add distro "crystal"
Add distro "dashing"
Add distro "eloquent"
Add distro "foxy"
Add distro "galactic"
Add distro "groovy"
Add distro "humble"
Add distro "hydro"
Add distro "indigo"
Add distro "iron"
Add distro "jade"
Add distro "jazzy"
Add distro "kilted"
Add distro "kinetic"
Add distro "lunar"
Add distro "melodic"
Add distro "noetic"
Add distro "rolling"
updated cache in /home/hello-robot/.ros/rosdep/sources.cache
..........
=== ./audio_common (git) ===
Cloning into '.'...
=== ./realsense-ros (git) ===
Cloning into '.'...
Note: switching to '4.55.1'.

You are in 'detached HEAD' state. You can look around, make experimental
changes and commit them, and you can discard any commits you make in this
state without impacting any branches by switching back to a branch.

If you want to create a new branch to retain commits you create, you may
do so (now or later) by using -c with the switch command. Example:

  git switch -c <new-branch-name>

Or undo this operation with:

  git switch -

Turn off this advice by setting config variable advice.detachedHead to false

HEAD is now at 8a86cb88 4.55.1
=== ./respeaker_ros2 (git) ===
Cloning into '.'...
=== ./ros2_numpy (git) ===
Cloning into '.'...
=== ./rosbridge_suite (git) ===
Cloning into '.'...
hello-robot@stretch-se3-3103:~$ 

