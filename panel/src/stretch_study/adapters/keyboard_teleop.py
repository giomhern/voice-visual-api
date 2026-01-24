from __future__ import annotations

import json
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """
Keyboard teleop -> publishes StudyEvents to /study_event (JSON in std_msgs/String)

Commands:
  a            advance
  1            arrive desk
  2            arrive bed
  3            arrive kitchen
  y <room>     demo_confirm yes for room (desk|bed|kitchen)
  n <room>     demo_confirm no for room
  p <profile>  profile switch (user|jackie)
  g <key> <v>  set global key=value (movement_speed, voice_volume, voice_profile, explainability, confirmation, social_distance)
  r <room> <key> <v>  set room key=value
  rules {json} set rules (provide raw JSON object)
  skip         skip current step
  h            help
  q            quit

Examples:
  g movement_speed slow
  r desk cleaning_thoroughness twice
  y desk
  rules {"no_go_zones":[{"room":"bed","after":"22:00"}],"time_rules":[{"key":"voice_volume","value":0,"after":"22:00"}]}
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('stretch_study_keyboard_teleop')
        self.declare_parameter('event_topic', '/study_event')
        self.event_topic = self.get_parameter('event_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(String, self.event_topic, 10)

        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.get_logger().info(HELP)

    def _publish(self, d: Dict):
        msg = String()
        msg.data = json.dumps(d, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f"sent: {msg.data}")

    def _loop(self):
        while not self._stop.is_set():
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if not line:
                continue
            if line == 'q':
                self._stop.set()
                break
            if line == 'h':
                print(HELP)
                continue

            parts = line.split(' ', 2)
            cmd = parts[0]

            try:
                if cmd == 'a':
                    self._publish({'type': 'advance'})
                elif cmd in ('1','2','3'):
                    room = {'1':'desk','2':'bed','3':'kitchen'}[cmd]
                    self._publish({'type': 'arrive', 'room': room})
                elif cmd in ('y','n'):
                    room = parts[1] if len(parts) > 1 else ''
                    self._publish({'type': 'demo_confirm', 'room': room, 'yes': cmd == 'y'})
                elif cmd == 'p':
                    prof = parts[1] if len(parts) > 1 else ''
                    self._publish({'type': 'profile', 'value': prof})
                elif cmd == 'g':
                    # g key value
                    key = parts[1]
                    value = parts[2] if len(parts) > 2 else ''
                    self._publish({'type': 'set', 'scope': 'global', 'key': key, 'value': _coerce(value)})
                elif cmd == 'r':
                    # r room key value
                    rest = line.split(' ', 3)
                    if len(rest) < 4:
                        raise ValueError('usage: r <room> <key> <value>')
                    room = rest[1]
                    key = rest[2]
                    value = rest[3]
                    self._publish({'type': 'set', 'scope': 'room', 'room': room, 'key': key, 'value': _coerce(value)})
                elif cmd == 'rules':
                    raw_json = line[len('rules '):].strip()
                    obj = json.loads(raw_json)
                    self._publish({'type': 'set', 'scope': 'rules', 'value': obj})
                elif cmd == 'skip':
                    self._publish({'type': 'skip'})
                else:
                    print('Unknown command. Press h for help.')
            except Exception as e:
                print(f'Error: {e}')


def _coerce(v: str):
    v = v.strip()
    if v.lower() in ('true','false'):
        return v.lower() == 'true'
    try:
        if v.isdigit() or (v.startswith('-') and v[1:].isdigit()):
            return int(v)
    except Exception:
        pass
    return v


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
