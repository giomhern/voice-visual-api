ros2 topic pub --once /speech_request std_msgs/msg/String \
'{"data":"{\"text\":\"Hello. This is Stretch speaking.\",\"volume\":60,\"rate\":170,\"voice\":\"auto\"}"}'

ros2 run stretch_study study_engine --ros-args --log-level INFO \
  -p speech.enable:=true \
  -p nav.enable:=false \
  -p motion.enable_transit:=false


  colcon build --symlink-install
Starting >>> stretch_study
/home/hello-robot/.local/lib/python3.10/site-packages/setuptools_scm/_integration/setuptools.py:24: RuntimeWarning: 
ERROR: setuptools==59.6.0 is used in combination with setuptools-scm>=8.x

Your build configuration is incomplete and previously worked by accident!
setuptools-scm requires setuptools>=61 (recommended: >=80)

Suggested workaround if applicable:
 - migrating from the deprecated setup_requires mechanism to pep517/518
   and using a pyproject.toml to declare build dependencies
   which are reliably pre-installed before running the build tools

  warnings.warn(
--- stderr: stretch_study                   
/home/hello-robot/.local/lib/python3.10/site-packages/setuptools_scm/_integration/setuptools.py:24: RuntimeWarning: 
ERROR: setuptools==59.6.0 is used in combination with setuptools-scm>=8.x

Your build configuration is incomplete and previously worked by accident!
setuptools-scm requires setuptools>=61 (recommended: >=80)

Suggested workaround if applicable:
 - migrating from the deprecated setup_requires mechanism to pep517/518
   and using a pyproject.toml to declare build dependencies
   which are reliably pre-installed before running the build tools

  warnings.warn(
/home/hello-robot/.local/lib/python3.10/site-packages/pkg_resources/__init__.py:116: PkgResourcesDeprecationWarning: 1.12.1-git20200711.33e2d80-dfsg1-0.6 is an invalid version and will not be supported in a future release
  warnings.warn(
---
Finished <<< stretch_study [1.29s]

Summary: 1 package finished [1.50s]
  1 package had stderr output: stretch_study
hello-robot@stretch-se3-3103:~/voice-visual-api/stretch_loop$ 

