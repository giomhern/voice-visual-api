#!/usr/bin/env python3
import argparse

import rclpy

from prewipe_wipe_test import Config as PrewipeConfig
from prewipe_wipe_test import PrewipeWipeTest
from right_angle_forward_path import RightAngleForwardPath


def main():
    parser = argparse.ArgumentParser(
        description="Stow, run the right-angle forward path, then perform the pre-wipe and wipe routine."
    )
    parser.add_argument(
        "--speed",
        choices=["slow", "medium", "fast"],
        default="medium",
        help="Movement speed preset for the path stage",
    )
    parser.add_argument(
        "--turn-deg",
        type=float,
        default=-83.0,
        help="Relative turn in degrees for the path stage. Negative is right turn.",
    )
    parser.add_argument(
        "--level",
        choices=["low", "medium", "high"],
        default="low",
        help="Wiping intensity: low=1 pass, medium=2 passes, high=3 passes",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    path_node = RightAngleForwardPath(speed=args.speed, turn_degrees=args.turn_deg)
    wipe_node = None
    path_node_destroyed = False

    try:
        rc = path_node.run()
        path_node.destroy_node()
        path_node_destroyed = True

        if rc != 0:
            raise SystemExit(rc)

        wipe_node = PrewipeWipeTest(PrewipeConfig())
        rc = wipe_node.run(level=args.level)
    finally:
        if wipe_node is not None:
            wipe_node._stop_base()
            wipe_node.destroy_node()
        elif not path_node_destroyed:
            path_node.stop()
            path_node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == "__main__":
    main()
