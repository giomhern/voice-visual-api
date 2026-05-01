# Stretch Quick Start

## Free The Robot

1. Make sure the robot is free and safe to move.
2. Free the robot:

```bash
stretch_free_robot_process.py
```

3. Send the robot home:

```bash
stretch_robot_home.py
```

## Open Two Terminal Tabs

### Terminal 1

Launch the Stretch core driver:

```bash
ros2 launch stretch_core stretch_driver.launch.py
```

### Terminal 2

Start teleop:

```bash
ros2 run stretch_core keyboard_teleop
```
