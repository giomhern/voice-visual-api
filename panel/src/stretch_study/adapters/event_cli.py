from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventCli(Node):
    def __init__(self, event_topic: str):
        super().__init__('stretch_study_event_cli')
        self.pub = self.create_publisher(String, event_topic, 10)

    def send(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f"sent: {msg.data}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Send a single StudyEvent (JSON)')
    parser.add_argument('--topic', default='/study_event')
    parser.add_argument('--json', required=True, help='Event JSON, e.g. {"type":"advance"}')
    ns, unknown = parser.parse_known_args()

    payload = json.loads(ns.json)

    rclpy.init(args=args)
    node = EventCli(ns.topic)
    node.send(payload)

    # spin a short moment to ensure publish
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
