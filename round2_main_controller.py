#!/usr/bin/env python3
"""
round2_main_controller.py
Operator console for Round 2 of the voice study.

Round 2 has no automated demos — the operator teleops the robot from one
room to the next while the voice assistant runs each station's
Jackie-configuration dialog. After the voice flow for a station finishes
and the robot is positioned at the next room, the operator presses ENTER
here to release the voice assistant into the next section.

The voice assistant is `openai_voice_assistant.round2_main`, which exposes
`/api/section_advance` on its Flask port (default 5050). This controller:

  1. Switches the Stretch driver to position mode so
     `ros2 run stretch_core keyboard_teleop` works without an extra
     mode-switch step (skip with --no-ros).
  2. Reminds the operator how to launch keyboard_teleop.
  3. Walks through the round-2 transitions (section 2 -> 3, 3 -> 4, 4 -> 5),
     blocking on ENTER for each, then POSTing /api/section_advance.

Usage:
    # Terminal 1 — Stretch driver:
    ros2 launch stretch_core stretch_driver.launch.py mode:=navigation

    # Terminal 2 — keyboard teleop (start AFTER this controller has
    # switched the driver to position mode):
    ros2 run stretch_core keyboard_teleop

    # Terminal 3 — round 2 voice assistant:
    python3 -m openai_voice_assistant.round2_main

    # Terminal 4 — this controller:
    python3 round2_main_controller.py --voice-url http://localhost:5050

    # Skip ROS (e.g. when running on a non-robot machine):
    python3 round2_main_controller.py --voice-url http://localhost:5050 --no-ros
"""

import argparse
import json
import sys
import urllib.error
import urllib.request


# Section transitions the operator advances. Round 2 starts on section 2
# automatically, so the operator only acts at the boundaries 2->3, 3->4,
# and 4->5.
SECTION_TRANSITIONS = [
    (2, "Office (Desk)", 3, "Bedroom (Bed)"),
    (3, "Bedroom (Bed)", 4, "Kitchen"),
    (4, "Kitchen", 5, "Situational Rules"),
]


def post_advance(voice_url: str, current: int, nxt: int) -> bool:
    url = f"{voice_url.rstrip('/')}/api/section_advance"
    payload = json.dumps({"from_section": current, "to_section": nxt}).encode()
    req = urllib.request.Request(
        url, data=payload, headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            print(f"[Controller] Advance posted: HTTP {resp.status}")
            return True
    except urllib.error.URLError as e:
        print(f"[Controller] Failed to post advance to {url}: {e}")
        return False


def switch_to_position_mode() -> None:
    """Best-effort ROS service call so keyboard_teleop accepts joint
    commands. Silently no-ops if rclpy or the service is unavailable."""
    try:
        import rclpy
        from std_srvs.srv import Trigger
    except ImportError:
        print("[Controller] rclpy not available — skipping position-mode switch")
        return

    rclpy.init()
    node = rclpy.create_node("round2_controller_modeswitch")
    try:
        client = node.create_client(Trigger, "/switch_to_position_mode")
        if not client.wait_for_service(timeout_sec=5.0):
            print(
                "[Controller] /switch_to_position_mode unavailable — "
                "is stretch_driver running? Skipping."
            )
            return
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        if fut.done() and fut.result() and fut.result().success:
            print("[Controller] Switched stretch driver to position mode")
        else:
            print("[Controller] Position-mode switch did not confirm success")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Round 2 operator console (teleop + section advance)"
    )
    parser.add_argument(
        "--voice-url", type=str, default="http://localhost:5050",
        help="Round 2 voice assistant URL (default: http://localhost:5050)",
    )
    parser.add_argument(
        "--no-ros", action="store_true",
        help="Skip the position-mode switch (use on machines without rclpy)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  ROUND 2 OPERATOR CONSOLE")
    print("=" * 60)
    print(f"  Voice URL: {args.voice_url}")
    print()
    print("  In a separate terminal, run keyboard teleop:")
    print("    ros2 run stretch_core keyboard_teleop")
    print()
    print("  Transitions you'll advance (ENTER after each voice flow ends")
    print("  AND the robot is teleoped to the NEXT room):")
    for curr, curr_name, nxt, nxt_name in SECTION_TRANSITIONS:
        print(f"    Section {curr} ({curr_name}) -> Section {nxt} ({nxt_name})")
    print("=" * 60)

    if not args.no_ros:
        switch_to_position_mode()

    for curr, curr_name, nxt, nxt_name in SECTION_TRANSITIONS:
        print()
        print(f">>> Section {curr} ({curr_name}) is in progress.")
        print(
            f">>> When the voice flow for Section {curr} finishes and the "
            f"robot is at the {nxt_name} station,"
        )
        print(f">>> press ENTER to start Section {nxt} ({nxt_name}).")
        try:
            input(">>> [ENTER] ")
        except (EOFError, KeyboardInterrupt):
            print("\n[Controller] Aborted by operator")
            return 1
        post_advance(args.voice_url, curr, nxt)

    print()
    print("[Controller] All advances posted — round 2 will end after Section 5.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
