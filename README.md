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

ros2 action send_goal /stretch_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [joint_lift, wrist_extension, joint_wrist_yaw], points: [{positions: [0.5, 0.1, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}'


[stretch_driver-7] Finished goal
[stretch_driver-7] [INFO] [1770328986.152661385] [stretch_driver]: stretch_driver joint_traj action: Achieved all target points.
[stretch_driver-7] [INFO] [1770328986.352987846] [stretch_driver]: Received goal request
[stretch_driver-7] [INFO] [1770328986.355691404] [stretch_driver]: stretch_driver joint_traj action: New trajectory received with joint_names = ['joint_head_pan', 'joint_head_tilt']
[stretch_driver-7] -------------------------------------------
[stretch_driver-7] Finished goal
[stretch_driver-7] [INFO] [1770328987.150940468] [stretch_driver]: stretch_driver joint_traj action: Achieved all target points.
[stretch_driver-7] [INFO] [1770328987.420728033] [stretch_driver]: Received goal request
[stretch_driver-7] [INFO] [1770328987.430365246] [stretch_driver]: stretch_driver joint_traj action: New trajectory received with joint_names = ['joint_head_pan', 'joint_head_tilt']
[stretch_driver-7] -------------------------------------------
[stretch_driver-7] Finished goal
[stretch_driver-7] [INFO] [1770328987.453038006] [stretch_driver]: stretch_driver joint_traj action: Achieved all target points.
[funmap-8] [INFO] [1770328987.929151412] [funmap]: perform_head_scan: Performing local localization.
[funmap-8] termination on tolfun=0.001 (Thu Feb  5 16:02:14 2026)
[funmap-8] final/bestever f-value = -2.325945e-01 -2.328959e-01 after 225/204 evaluations
[funmap-8] incumbent solution: [0.49909600641113405, 0.4955049359850482, 0.49885471262223624]
[funmap-8] std deviation: [0.00041057541913310376, 0.0006579469243379255, 0.00033635144658048546]
[funmap-8] Optimization complete.
[funmap-8] cma_result =
[funmap-8] {'initial_solution': [0.5, 0.5, 0.5], 'initial_standard_deviation': 0.02, 'options': {'tolfun': 0.001}, 'best_parameters': [0.4995155698518597, 0.49494421740634437, 0.4989035289017918], 'best_parameters_error': -0.23289585414396438, 'num_evals_to_find_best': 204, 'num_evals_total': 225, 'cma_iterations': 32, 'cma_parameter_means': [0.49909600641113405, 0.4955049359850482, 0.49885471262223624], 'cma_parameter_stddevs': [0.00041057541913310376, 0.0006579469243379255, 0.00033635144658048546]}
[funmap-8] best_parameters = [1329.6659345177327, 1327.2504749385453, -0.09868239883873997]
[funmap-8] 
[funmap-8] target_x = 1329.6659345177327 , target_y = 1327.2504749385453 , angle_deg = -0.09868239883873997
[funmap-8] combined_robot_pose = {'x_pix': 1329.665934585252, 'y_pix': 1327.2504748789522, 'theta_rad': 0.005256363865058727}
[funmap-8] map_xy_1 = [-0.01000439  0.02449715]
[funmap-8] map_ang_rad = 0.00525636386508663
[funmap-8] map_quat = (0.0, 0.0, 0.0026281789069198765, 0.9999965463318528)
[funmap-8] original_robot_map_pose = [-0.008841760538067511, 0.012363272143954163, 0.006978696638585093]
[funmap-8] corrected_robot_map_pose = [-0.010004392488488278, 0.024497150726285888, 0.00525636386508663]
[funmap-8] original_robot_map_pose_xya = [-0.008841760538067511, 0.012363272143954163, 0.006978696638585093]
[funmap-8] corrected_robot_map_pose_xya = [-0.010004392488488278, 0.024497150726285888, 0.00525636386508663]
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/merged_maps/merged_map_20260205160214_mhi
[funmap-8] Finished saving.
[funmap-8] MaxHeightImage information:
[funmap-8]      image.shape = (2667, 2667)
[funmap-8]      image.dtype = uint8
[funmap-8]      m_per_pix = 0.006
[funmap-8]      m_per_height_unit = 0.0046456692913385824
[funmap-8]      voi.x_in_m = 16.0
[funmap-8]      voi.y_in_m = 16.0
[funmap-8]      voi.z_in_m = 1.18
[funmap-8] MaxHeightImage information:
[funmap-8]      image.shape = (2667, 2667)
[funmap-8]      image.dtype = uint8
[funmap-8]      m_per_pix = 0.006
[funmap-8]      m_per_height_unit = 0.0046456692913385824
[funmap-8]      voi.x_in_m = 16.0
[funmap-8]      voi.y_in_m = 16.0
[funmap-8]      voi.z_in_m = 1.18
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/head_scans/head_scan_20260205160307_mhi
[funmap-8] Finished saving.
[funmap-8] scan_1.robot_xy_pix = [664.83314295 663.62523637]
[funmap-8] init_target = None
[funmap-8] init_target = [664.833142951034, 663.6252363667669, 0.0]
[funmap-8] target_x_constraint = [564.833142951034, 764.833142951034]
[funmap-8] target_y_constraint = [563.6252363667669, 763.6252363667669]
[funmap-8] angle_deg_constraint = [-30.0, 30.0]
[funmap-8] (3_w,7)-aCMA-ES (mu_w=2.3,w_1=58%) in dimension 3 (seed=492797, Thu Feb  5 16:03:07 2026)
[funmap-8] Iterat #Fevals   function value  axis ratio  sigma  min&max std  t[m:s]
[funmap-8]     1      7 -1.439687131146986e-01 1.0e+00 2.11e-02  2e-02  2e-02 0:00.0
[funmap-8]     2     14 -1.413046486781723e-01 1.4e+00 2.19e-02  2e-02  3e-02 0:00.0
[funmap-8]     3     21 -1.453079628580584e-01 1.7e+00 1.77e-02  1e-02  2e-02 0:00.1
[funmap-8] [INFO] [1770328989.421526289] [funmap]: perform_head_scan: Performing local map merge.
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/head_scans/head_scan_20260205160309_mhi
[funmap-8] Finished saving.
[funmap-8] scan_1.robot_xy_pix = [1329.6659281  1327.25047469]
[funmap-8] init_target = None
[funmap-8] init_target = [1329.665928101297, 1327.2504746885413, 0.0]
[funmap-8] target_x_constraint = [1129.665928101297, 1529.665928101297]
[funmap-8] target_y_constraint = [1127.2504746885413, 1527.2504746885413]
[funmap-8] angle_deg_constraint = [-45.0, 45.0]
[funmap-8] (3_w,7)-aCMA-ES (mu_w=2.3,w_1=58%) in dimension 3 (seed=447339, Thu Feb  5 16:03:09 2026)
[funmap-8] Iterat #Fevals   function value  axis ratio  sigma  min&max std  t[m:s]
[funmap-8]     1      7 -1.270808001109596e-01 1.0e+00 1.69e-02  2e-02  2e-02 0:00.1
[funmap-8]    56    392 -1.896744836596001e-01 8.3e+00 1.93e-03  3e-04  7e-04 0:01.3
[funmap-8] termination on tolfun=0.001 (Thu Feb  5 16:03:09 2026)
[funmap-8] final/bestever f-value = -1.895290e-01 -1.898757e-01 after 393/318 evaluations
[funmap-8] incumbent solution: [0.48743773879925323, 0.717005471708388, 0.3800075283081508]
[funmap-8] std deviation: [0.0002931960467886566, 0.0003093081712078891, 0.0006626101940777503]
[funmap-8] Optimization complete.
[funmap-8] cma_result =
[funmap-8] {'initial_solution': [0.5, 0.5, 0.5], 'initial_standard_deviation': 0.02, 'options': {'tolfun': 0.001}, 'best_parameters': [0.4878234660665106, 0.7167329101868826, 0.37958705511007285], 'best_parameters_error': -0.18987574785477088, 'num_evals_to_find_best': 318, 'num_evals_total': 393, 'cma_iterations': 56, 'cma_parameter_means': [0.48743773879925323, 0.717005471708388, 0.3800075283081508], 'cma_parameter_stddevs': [0.0002931960467886566, 0.0003093081712078891, 0.0006626101940777503]}
[funmap-8] best_parameters = [662.3978361643361, 706.9718184041434, -7.224776693395629]
[funmap-8] 
[funmap-8] target_x = 662.3978361643361 , target_y = 706.9718184041434 , angle_deg = -7.224776693395629
[funmap-8] combined_robot_pose = {'x_pix': 662.3978370382378, 'y_pix': 706.9718201093535, 'theta_rad': -0.12088181352339922}
[funmap-8] map_xy_1 = [-0.03922596 -0.49566184]
[funmap-8] map_ang_rad = -0.12088181352327111
[funmap-8] map_quat = (0.0, 0.0, -0.06040411400424618, 0.9981740043756709)
[funmap-8] original_robot_map_pose = [-0.010002284587591781, 0.024497163598796945, 0.005214327497634583]
[funmap-8] corrected_robot_map_pose = [-0.039225955541147606, -0.4956618413122431, -0.12088181352327111]
[funmap-8] map_xy_1 = [-0.03922596 -0.49566184]
[funmap-8] map_ang_rad = -0.12088181352354715
[funmap-8] Total time to match to the loaded map = 1.7953412532806396
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/scaled_localization_scans/localization_scaled_head_scan_20260205160309_mhi
[funmap-8]     2     14 -1.268021055009649e-01 1.3e+00 1.58e-02  1e-02  2e-02 0:00.2
[funmap-8] Finished saving.
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/scaled_localization_scans/localization_scaled_merged_map_20260205160309_mhi
[funmap-8]     3     21 -1.296262644523241e-01 1.7e+00 1.93e-02  2e-02  2e-02 0:00.3
[stretch_driver-7] [INFO] [1770328990.666285174] [stretch_driver]: Changed to mode = trajectory
[funmap-8] Finished saving.
[funmap-8] original_robot_map_pose_xya = [-0.010002284587591781, 0.024497163598796945, 0.0052143274975597976]
[funmap-8] corrected_robot_map_pose_xya = [-0.039225955541147606, -0.4956618413122431, -0.12088181352354715]
[funmap-8]    42    294 -1.801234379205739e-01 1.8e+00 6.03e-03  1e-03  1e-03 0:03.3
[funmap-8]    49    343 -1.802622148246722e-01 1.4e+00 1.05e-02  2e-03  2e-03 0:03.9
[funmap-8] [INFO] [1770328994.546480952] [funmap]: robot turn angle in degrees =70.0
[funmap-8] Exception in thread Thread-1 (spin_thread):
[funmap-8] Traceback (most recent call last):
[funmap-8]   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
[funmap-8]     self.run()
[funmap-8]   File "/usr/lib/python3.10/threading.py", line 953, in run
[funmap-8]     self._target(*self._args, **self._kwargs)
[funmap-8]   File "/home/hello-robot/ament_ws/build/hello_helpers/src/hello_helpers/hello_misc.py", line 125, in spin_thread
[funmap-8]     executor.spin()
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 298, in spin
[funmap-8]     self.spin_once()
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 828, in spin_once
[funmap-8]     self._spin_once_impl(timeout_sec)
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 825, in _spin_once_impl
[funmap-8]     future.result()
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 105, in result
[funmap-8]     raise self.exception()
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 254, in __call__
[funmap-8]     self._handler.send(None)
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 463, in handler
[funmap-8]     await call_coroutine(entity, arg)
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 411, in _execute_service
[funmap-8]     response = await await_or_execute(srv.callback, request, srv.srv_type.Response())
[funmap-8]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
[funmap-8]     return callback(*args)
[funmap-8]   File "/home/hello-robot/ament_ws/build/stretch_funmap/stretch_funmap/funmap.py", line 491, in trigger_head_scan_service_callback
[funmap-8]     self.perform_head_scan()
[funmap-8]   File "/home/hello-robot/ament_ws/build/stretch_funmap/stretch_funmap/funmap.py", line 1078, in perform_head_scan
[funmap-8]     at_goal = self.move_base.turn(
[funmap-8]   File "/home/hello-robot/ament_ws/build/stretch_funmap/stretch_funmap/navigate.py", line 384, in turn
[funmap-8]     self._future_goal = self.node.move_to_pose(pose, blocking=False)
[funmap-8]   File "/home/hello-robot/ament_ws/build/hello_helpers/src/hello_helpers/hello_misc.py", line 192, in move_to_pose
[funmap-8]     point0.positions.append(self.joint_state.position[self.joint_state.name.index(joint)])
[funmap-8] ValueError: 'rotate_mobile_base' is not in list
[funmap-8] termination on tolfun=0.001 (Thu Feb  5 16:03:14 2026)
[funmap-8] final/bestever f-value = -1.800675e-01 -1.803639e-01 after 344/335 evaluations
[funmap-8] incumbent solution: [0.5880607602937993, 0.7909795726718846, 0.47122978912374225]
[funmap-8] std deviation: [0.0021210119352606405, 0.0018277134497011406, 0.0016539383138672098]
[funmap-8] Optimization complete.
[funmap-8] cma_result =
[funmap-8] {'initial_solution': [0.5, 0.5, 0.5], 'initial_standard_deviation': 0.02, 'options': {'tolfun': 0.001}, 'best_parameters': [0.5892931343938725, 0.7916821124559663, 0.46980085920128156], 'best_parameters_error': -0.18036389760333751, 'num_evals_to_find_best': 335, 'num_evals_total': 344, 'cma_iterations': 49, 'cma_parameter_means': [0.5880607602937993, 0.7909795726718846, 0.47122978912374225], 'cma_parameter_stddevs': [0.0021210119352606405, 0.0018277134497011406, 0.0016539383138672098]}
[funmap-8] best_parameters = [1365.383181858846, 1443.9233196709279, -2.717922671884658]
[funmap-8] 
[funmap-8] target_x = 1365.383181858846 , target_y = 1443.9233196709279 , angle_deg = -2.717922671884658
[funmap-8] combined_robot_pose = {'x_pix': 1365.3831824645356, 'y_pix': 1443.9233213243808, 'theta_rad': -0.04218058482920755}
[funmap-8] map_xy_1 = [ 0.20429909 -0.67553993]
[funmap-8] map_ang_rad = -0.04218058482919264
[funmap-8] map_quat = (0.0, 0.0, -0.02108872895416445, 0.999777608026454)
[funmap-8] original_robot_map_pose = [-0.010004431392219182, 0.024497151868751565, 0.005256114609764645]
[funmap-8] corrected_robot_map_pose = [0.20429909478721253, -0.6755399279462857, -0.04218058482919264]
[funmap-8] original_robot_map_pose_xya = [-0.010004431392219182, 0.024497151868751565, 0.005256114609764645]
[funmap-8] corrected_robot_map_pose_xya = [0.20429909478721253, -0.6755399279462857, -0.04218058482919264]
[funmap-8] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/merged_maps/merged_map_20260205160314_mhi
[funmap-8] Finished saving.
[funmap-8] robot_to_odom_mat = [[ 9.99999998e-01  6.46418694e-05  0.00000000e+00 -1.51649143e-05]
[funmap-8]  [-6.46418694e-05  9.99999998e-01  0.00000000e+00 -5.06265432e-10]
[funmap-8]  [ 0.00000000e+00  0.00000000e+00  1.00000000e+00  0.00000000e+00]
[funmap-8]  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
[funmap-8] timestamp = builtin_interfaces.msg.Time(sec=1770328994, nanosec=523802003)
[funmap-8] r0 = [0. 0. 0. 1.]
[INFO] [funmap-8]: process has finished cleanly [pid 14972]
[clean_surface-9] [INFO] [1770328995.350971905] [clean_surface]: Cleaning initiating!
[clean_surface-9] [INFO] [1770328995.351468633] [stretch_funmap]: ManipulationView __init__: self.debug_directory = /home/hello-robot/stretch_user/debug/
[clean_surface-9] Exception in thread Thread-1 (spin_thread):
[clean_surface-9] Traceback (most recent call last):
[clean_surface-9]   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
[clean_surface-9]     self.run()
[clean_surface-9]   File "/usr/lib/python3.10/threading.py", line 953, in run
[clean_surface-9]     self._target(*self._args, **self._kwargs)
[clean_surface-9]   File "/home/hello-robot/ament_ws/build/hello_helpers/src/hello_helpers/hello_misc.py", line 125, in spin_thread
[clean_surface-9]     executor.spin()
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 298, in spin
[clean_surface-9]     self.spin_once()
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 828, in spin_once
[clean_surface-9]     self._spin_once_impl(timeout_sec)
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 825, in _spin_once_impl
[clean_surface-9]     future.result()
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 105, in result
[clean_surface-9]     raise self.exception()
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 254, in __call__
[clean_surface-9]     self._handler.send(None)
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 463, in handler
[clean_surface-9]     await call_coroutine(entity, arg)
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 411, in _execute_service
[clean_surface-9]     response = await await_or_execute(srv.callback, request, srv.srv_type.Response())
[clean_surface-9]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
[clean_surface-9]     return callback(*args)
[clean_surface-9]   File "/home/hello-robot/ament_ws/build/stretch_demos/stretch_demos/clean_surface.py", line 83, in trigger_clean_surface_callback
[clean_surface-9]     self.look_at_surface()
[clean_surface-9]   File "/home/hello-robot/ament_ws/build/stretch_demos/stretch_demos/clean_surface.py", line 62, in look_at_surface
[clean_surface-9]     manip.move_head(self.move_to_pose)
[clean_surface-9]   File "/home/hello-robot/ament_ws/build/stretch_funmap/stretch_funmap/manipulation_planning.py", line 275, in move_head
[clean_surface-9]     move_to_pose(pose)
[clean_surface-9]   File "/home/hello-robot/ament_ws/build/hello_helpers/src/hello_helpers/hello_misc.py", line 192, in move_to_pose
[clean_surface-9]     point0.positions.append(self.joint_state.position[self.joint_state.name.index(joint)])
[clean_surface-9] ValueError: 'joint_head_pan' is not in list
[clean_surface-9] MaxHeightImage information:
[clean_surface-9]      image.shape = (334, 434)
[clean_surface-9]      image.dtype = uint8
[clean_surface-9]      m_per_pix = 0.006
[clean_surface-9]      m_per_height_unit = 0.004566929133858267
[clean_surface-9]      voi.x_in_m = 2.6
[clean_surface-9]      voi.y_in_m = 2.0
[clean_surface-9]      voi.z_in_m = 1.16
[clean_surface-9] MaxHeightImage saving to base_filename = /home/hello-robot/stretch_user/debug/clean_surface/look_at_surface_20260205160206
[clean_surface-9] Finished saving.
[clean_surface-9] segment_max_height_image.py : find_floor
[clean_surface-9] MaxHeightImage information:
[clean_surface-9]      image.shape = (334, 434)
[clean_surface-9]      image.dtype = uint8
[clean_surface-9]      m_per_pix = 0.006
[clean_surface-9]      m_per_height_unit = 0.004566929133858267
[clean_surface-9]      voi.x_in_m = 2.6
[clean_surface-9]      voi.y_in_m = 2.0
[clean_surface-9]      voi.z_in_m = 1.16
