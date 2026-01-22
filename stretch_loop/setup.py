from setuptools import find_packages, setup

package_name = 'stretch_study'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['stretch_study/launch/study.launch.py']),
        ('share/' + package_name + '/config', ['stretch_study/config/defaults.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Lab',
    maintainer_email='example@example.com',
    description='Deterministic, Tier-1 (manual station) study controller for Stretch 3 on ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'study_engine = stretch_study.nodes.study_engine:main',
            'keyboard_teleop = stretch_study.adapters.keyboard_teleop:main',
            'event_cli = stretch_study.adapters.event_cli:main',
            'speech_node = stretch_study.nodes.speech_node:main',
        ],
    },
)
