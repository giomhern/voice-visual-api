from __future__ import annotations

import json
import os
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stretch_study.core.types import StudyEvent, Prompt
from stretch_study.core.settings import StudySettings
from stretch_study.core.logger import StudyLogger
from stretch_study.core.script_steps import ScriptContext, ScriptBuilder
from stretch_study.capabilities.deterministic_demos import DeterministicDemos


class StudyEngineNode(Node):
    def __init__(self):
        super().__init__('stretch_study_engine')

        # ---- Params ----
        self.declare_parameter('study.session_id', 'session_001')
        self.declare_parameter('study.participant_id', 'p001')
        self.declare_parameter('study.output_dir', os.path.expanduser('~/.ros/stretch_study'))
        self.declare_parameter('io.event_topic', '/study_event')
        self.declare_parameter('io.prompt_topic', '/study_prompt')

        # Motion params (optional; enable only for controlled demo testing)
        self.declare_parameter('motion.enable_transit', False)
        self.declare_parameter('motion.cmd_vel_topic', '/stretch/cmd_vel')
        self.declare_parameter('motion.odom_topic', '/odom')
        # Distances for the assumed rectangle route (meters)
        self.declare_parameter('motion.distances_m.door_to_desk', 0.0)
        self.declare_parameter('motion.distances_m.desk_to_bed', 0.0)
        self.declare_parameter('motion.distances_m.bed_to_kitchen', 0.0)

        session_id = self.get_parameter('study.session_id').get_parameter_value().string_value
        participant_id = self.get_parameter('study.participant_id').get_parameter_value().string_value
        output_dir = self.get_parameter('study.output_dir').get_parameter_value().string_value

        self.event_topic = self.get_parameter('io.event_topic').get_parameter_value().string_value
        self.prompt_topic = self.get_parameter('io.prompt_topic').get_parameter_value().string_value

        motion_cfg = {
            'enable_transit': self.get_parameter('motion.enable_transit').get_parameter_value().bool_value,
            'cmd_vel_topic': self.get_parameter('motion.cmd_vel_topic').get_parameter_value().string_value,
            'odom_topic': self.get_parameter('motion.odom_topic').get_parameter_value().string_value,
            'distances_m': {
                'door_to_desk': self.get_parameter('motion.distances_m.door_to_desk').get_parameter_value().double_value,
                'desk_to_bed': self.get_parameter('motion.distances_m.desk_to_bed').get_parameter_value().double_value,
                'bed_to_kitchen': self.get_parameter('motion.distances_m.bed_to_kitchen').get_parameter_value().double_value,
            },
        }

        # ---- Logging ----
        self.logger2 = StudyLogger(output_dir=output_dir, session_id=session_id, participant_id=participant_id)

        # ---- Settings ----
        self.settings = StudySettings()

        # ---- Prompt pub + Event sub ----
        self.prompt_pub = self.create_publisher(String, self.prompt_topic, 10)
        self.event_sub = self.create_subscription(String, self.event_topic, self._on_event, 10)

        # ---- Speech hook ----
        def say(text: str) -> None:
            # For now: publish as part of prompt payload and log
            self.get_logger().info(f"SAY: {text}")
            self.logger2.log_event({'type': 'say', 'text': text, 'profile': self.settings.active_profile})

        # ---- Capabilities (deterministic demos) ----
        self.cap = DeterministicDemos(node=self, logger=self.logger2, say_fn=say, motion_cfg=motion_cfg)

        # ---- Script context + steps ----
        self.ctx = ScriptContext(settings=self.settings, capabilities=self.cap, logger=self.logger2)
        self.steps = ScriptBuilder(self.ctx).build()
        self.step_idx = 0

        # Publish first prompt
        self._enter_step()

    # ---------------------------
    # State machine
    # ---------------------------
    def _enter_step(self) -> None:
        if self.step_idx >= len(self.steps):
            self.get_logger().info('Study complete (no more steps).')
            return

        step = self.steps[self.step_idx]
        prompt = step.on_enter()
        self.logger2.log_event({'type': 'step_enter', 'step_id': step.step_id, 'profile': self.settings.active_profile})
        self._publish_prompt(prompt)

    def _advance(self) -> None:
        # snapshot at useful boundaries
        step_id = self.steps[self.step_idx].step_id if self.step_idx < len(self.steps) else 'done'
        if step_id in ('global_complete', 'round1_rules_complete', 'round2_rules_complete'):
            self.logger2.snapshot(step_id, self.settings.active().snapshot())

        self.step_idx += 1
        self._enter_step()

    # ---------------------------
    # Event handling
    # ---------------------------
    def _on_event(self, msg: String) -> None:
        raw = msg.data
        self.logger2.log_event({'type': 'event_raw', 'raw': raw})

        try:
            d = json.loads(raw)
        except Exception as e:
            self.get_logger().warn(f"Invalid JSON on {self.event_topic}: {e}")
            self.logger2.log_event({'type': 'event_parse_error', 'raw': raw, 'error': str(e)})
            return

        ev = StudyEvent.from_dict(d)
        if not ev.type:
            self.get_logger().warn('Event missing type.')
            return

        if self.step_idx >= len(self.steps):
            return

        step = self.steps[self.step_idx]

        # Global overrides
        if ev.type == 'estop':
            self.logger2.log_event({'type': 'estop', 'value': ev.value})
            return
        if ev.type == 'skip':
            self.logger2.log_event({'type': 'step_skip', 'step_id': step.step_id})
            self._advance()
            return

        completed = False
        try:
            completed = bool(step.handle(ev))
        except Exception as e:
            self.get_logger().error(f"Error handling event in step {step.step_id}: {e}")
            self.logger2.log_event({'type': 'step_handle_error', 'step_id': step.step_id, 'error': str(e), 'event': d})
            return

        self.logger2.log_event({'type': 'event', 'step_id': step.step_id, 'event': d, 'completed_step': completed})

        if completed:
            self._advance()

    # ---------------------------
    # Prompt publishing
    # ---------------------------
    def _publish_prompt(self, prompt: Prompt) -> None:
        payload: Dict[str, Any] = {
            'step_id': prompt.step_id,
            'text': prompt.text,
        }
        if prompt.options is not None:
            payload['options'] = prompt.options
        if prompt.hint is not None:
            payload['hint'] = prompt.hint
        if prompt.payload is not None:
            payload['payload'] = prompt.payload

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.prompt_pub.publish(out)
        self.get_logger().info(f"PROMPT[{prompt.step_id}]: {prompt.text}")


def main(args=None):
    rclpy.init(args=args)
    node = StudyEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
