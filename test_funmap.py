#!/usr/bin/env python3
"""
test_funmap.py — Navigate to desk via goal_pose, then run desk demo.

Prerequisites:
    1. ros2 launch stretch_core stretch_driver.launch.py mode:=navigation
    2. ros2 launch stretch_funmap mapping.launch.py

Usage:
    python3 test_funmap.py
    python3 test_funmap.py --wipes 2
"""

import argparse
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Resolve paths relative to this script's location.
HERE = os.path.dirname(os.path.abspath(__file__))

_CANDIDATES = [
    os.path.join(HERE, "voice-visual-api", "src", "stretch_study", "stretch_study"),
    os.path.join(HERE, "src", "stretch_study", "stretch_study"),
]
STUDY_PKG = next((p for p in _CANDIDATES if os.path.isdir(p)), _CANDIDATES[0])
sys.path.insert(0, STUDY_PKG)

from capabilities.funmap_navigator import FunmapNavigator

GOALS_YAML = os.path.join(STUDY_PKG, "config", "goals.yml")

_CAP_CANDIDATES = [
    os.path.join(HERE, "voice-visual-api", "capabilities"),
    os.path.join(HERE, "capabilities"),
]
CAPABILITIES_DIR = next((p for p in _CAP_CANDIDATES if os.path.isdir(p)), _CAP_CANDIDATES[0])


def switch_to_position_mode(node):
    """Explicitly switch stretch_driver to position mode."""
    cli = node.create_client(Trigger, "/switch_to_position_mode")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn("switch_to_position_mode service not available")
        return False
    future = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is not None:
        node.get_logger().info(f"switch_to_position_mode: {future.result().message}")
        return future.result().success
    return False


def main():
    parser = argparse.ArgumentParser(description="FUNMAP to desk + desk demo")
    parser.add_argument("--wipes", type=int, default=1, choices=[1, 2, 3],
                        help="Number of wipes for desk demo (default: 1)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("funmap_test")
    nav = FunmapNavigator(node, goals_yaml=GOALS_YAML)

    print("Waiting for TF...")
    time.sleep(2.0)
    rclpy.spin_once(node, timeout_sec=1.0)

    try:
        # 1. Switch to navigation mode
        print("\n=== SWITCH TO POSITION MODE ===")
        switch_to_position_mode(node)

        # 2. Navigate to desk
        print("\n=== NAVIGATE TO DESK ===")
        arrived = nav.goto(name="desk", timeout_s=120.0, arrive_dist_m=0.35)
        if not arrived:
            print("Failed to reach desk. Aborting.")
            return

        # 3. Desk demo (handles its own mode switching internally)
        print(f"\n=== DESK DEMO ({args.wipes} wipe(s)) ===")
        subprocess.run(
            ["python3", os.path.join(CAPABILITIES_DIR, "desk_demo.py"),
             "--wipes", str(args.wipes)],
        )

        print("\n=== DONE ===")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
