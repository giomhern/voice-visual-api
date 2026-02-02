```
 ros2 launch stretch_funmap mapping.launch.py map_yaml:=/home/hello-robot/stretch_user/debug/merged_maps/merged_map_20260202160851


ros2 run stretch_study study_engine \
  --ros-args \
  -p nav.enable:=true \
  -p nav.goals_yaml:=/home/hello-robot/voice-visual-api/src/stretch_study/stretch_study/config/goals.yml \
  -p motion.scaler_node:=/cmd_vel_scaler
```

At time 1770073295.42146707
- Translation: [-0.049, -1.935, 0.000]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.830, 0.557]
- Rotation: in RPY (radian) [0.000, 0.000, -1.960]
- Rotation: in RPY (degree) [0.000, 0.000, -112.299]
- Matrix:
 -0.379  0.925  0.000 -0.049
 -0.925 -0.379 -0.000 -1.935
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
