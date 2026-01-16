from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from .types import Prompt, StudyEvent


@dataclass
class Step:
    step_id: str
    on_enter: Callable[[], Prompt]
    handle: Callable[[StudyEvent], bool]
    # handle returns True if step is complete


class ScriptBuilder:
    """Builds the canonical step list mirroring the interaction script.

    The engine consumes the resulting steps sequentially.
    """

    def __init__(self, ctx: 'ScriptContext'):
        self.ctx = ctx

    def build(self) -> List[Step]:
        c = self.ctx
        steps: List[Step] = []

        # ---- INTRO ----
        steps.append(Step(
            step_id='intro_greeting',
            on_enter=lambda: Prompt(
                step_id='intro_greeting',
                text=(
                    "Hello. I’m Stretch. Thanks for unboxing me. "
                    "Before we walk around your home, we need to set my general behavior settings. "
                    "These settings will become my default behavior everywhere, unless you change them later for a specific room."
                ),
                hint="Send {type:'advance'} to continue."
            ),
            handle=lambda ev: ev.type == 'advance'
        ))

        # ---- GLOBAL SETTINGS WIZARD ----
        steps += self._global_settings_steps()

        # ---- ROUND 1 TOUR ----
        steps += self._tour_steps(round_name='round1', profile='user')

        # ---- ROUND 1 RULES ----
        steps += self._rules_steps(round_name='round1')

        # ---- ROUND 2 SETUP ----
        steps.append(Step(
            step_id='round2_intro',
            on_enter=lambda: Prompt(
                step_id='round2_intro',
                text=(
                    "Welcome back. I understand Jackie will be staying with you for a week. "
                    "We’ll now walk through the apartment again. This time, please configure my behavior to best support Jackie’s needs. "
                    "Just like before, please guide me to each room and tell me where we are."
                ),
                hint="Send {type:'profile', value:'jackie'} then {type:'advance'}"
            ),
            handle=self._handle_round2_intro,
        ))

        # ---- ROUND 2 TOUR ----
        steps += self._tour_steps(round_name='round2', profile='jackie')

        # ---- ROUND 2 RULES + END ----
        steps += self._rules_steps(round_name='round2')

        steps.append(Step(
            step_id='end',
            on_enter=lambda: Prompt(
                step_id='end',
                text="Thank you. This completes Round 2.",
                hint="Session complete."
            ),
            handle=lambda ev: False,
        ))

        return steps

    def _global_settings_steps(self) -> List[Step]:
        c = self.ctx
        steps: List[Step] = []

        def mk_set_step(step_id: str, text: str, key: str, options: Optional[List[Any]] = None):
            return Step(
                step_id=step_id,
                on_enter=lambda: Prompt(step_id=step_id, text=text, options=options),
                handle=lambda ev: c.handle_set(scope='global', key=key, ev=ev)
            )

        steps.append(mk_set_step(
            'global_movement_speed',
            "First: Movement Speed. I can move slowly, at a medium pace, or fast. Please set my default movement speed.",
            key='movement_speed',
            options=['slow','medium','fast'],
        ))

        steps.append(mk_set_step(
            'global_voice_volume',
            "Next: Voice Volume. You can set my speaking volume anywhere between 0 and 100 percent. Please set my default speaking volume.",
            key='voice_volume',
            options=list(range(0,101,10)),
        ))

        steps.append(mk_set_step(
            'global_voice_profile',
            "Next: Voice Profile. You can choose a neutral voice, a friendly voice, or a playful voice. Please set my default voice profile.",
            key='voice_profile',
            options=['neutral','friendly','playful'],
        ))

        steps.append(mk_set_step(
            'global_explainability',
            "Next: Explainability Level. Options include none, short explanations, or full step-by-step explanations. Please set my default explainability.",
            key='explainability',
            options=['none','short','full'],
        ))

        steps.append(mk_set_step(
            'global_confirmation',
            "Next: Confirmation Style. Should I confirm every change you make, or apply changes silently? Please choose my default confirmation style.",
            key='confirmation',
            options=['confirm','silent'],
        ))

        steps.append(mk_set_step(
            'global_social_distance',
            "Finally: Default Social Distance — how far I should stand from you when idle. Options are close, medium, or far. Please set my default social distance.",
            key='social_distance',
            options=['close','medium','far'],
        ))

        steps.append(Step(
            step_id='global_complete',
            on_enter=lambda: Prompt(
                step_id='global_complete',
                text=(
                    "Great. My default behavior has been set. "
                    "You’ll now guide me through each room, and we’ll make room-specific customizations that override these defaults. "
                    "Please walk with me and guide me. When we reach a station, tell me where we are."
                ),
                hint="Send {type:'advance'} to continue."
            ),
            handle=lambda ev: ev.type == 'advance'
        ))

        return steps

    def _tour_steps(self, round_name: str, profile: str) -> List[Step]:
        c = self.ctx
        steps: List[Step] = []

        # Ensure profile is set when entering the round.
        steps.append(Step(
            step_id=f'{round_name}_set_profile_{profile}',
            on_enter=lambda: Prompt(
                step_id=f'{round_name}_set_profile_{profile}',
                text=f"(Internal) Setting active profile to {profile}.",
                hint="Send {type:'advance'}"
            ),
            handle=lambda ev: c.handle_profile_and_advance(profile, ev)
        ))

        # Per-room loop: arrive -> configure -> demo prompt -> demo
        for room in ['desk','bed','kitchen']:
            steps.append(Step(
                step_id=f'{round_name}_{room}_wait_arrive',
                on_enter=lambda room=room: Prompt(
                    step_id=f'{round_name}_{room}_wait_arrive',
                    text=f"Please tell me when we are now at the {room.title()} area.",
                    hint=f"Send {{type:'arrive', room:'{room}'}}"
                ),
                handle=lambda ev, room=room: c.handle_arrive(room, ev)
            ))

            steps += self._room_config_steps(round_name, room)

            steps.append(Step(
                step_id=f'{round_name}_{room}_demo_confirm',
                on_enter=lambda room=room: Prompt(
                    step_id=f'{round_name}_{room}_demo_confirm',
                    text=f"I have received your customizations. Would you like me to perform the {room.title()} Demo?",
                    options=[True, False],
                    hint=f"Send {{type:'demo_confirm', room:'{room}', yes:true|false}}"
                ),
                handle=lambda ev, room=room: c.handle_demo_confirm(room, ev)
            ))

            steps.append(Step(
                step_id=f'{round_name}_{room}_demo_run',
                on_enter=lambda room=room: c.prompt_and_run_demo(room, step_id=f'{round_name}_{room}_demo_run'),
                handle=lambda ev: True,  # demo executes on enter; auto-advance
            ))

        return steps

    def _room_config_steps(self, round_name: str, room: str) -> List[Step]:
        c = self.ctx
        steps: List[Step] = []

        # Room-level overrides
        steps.append(Step(
            step_id=f'{round_name}_{room}_config_intro',
            on_enter=lambda room=room: Prompt(
                step_id=f'{round_name}_{room}_config_intro',
                text=(
                    f"In this area, you can set specific preferences that will override my general defaults. "
                    f"Please customize my behavior for the {room.title()} area."
                ),
                hint=(
                    f"Send one or more set events like: {{type:'set', scope:'room', room:'{room}', key:'movement_speed', value:'slow'}}"
                )
            ),
            handle=lambda ev: c.handle_room_config_any(room, ev)
        ))

        # We keep a single config step that can accept multiple 'set' events.
        # Participant (or UI) can decide when to advance.
        steps.append(Step(
            step_id=f'{round_name}_{room}_config_done',
            on_enter=lambda room=room: Prompt(
                step_id=f'{round_name}_{room}_config_done',
                text=f"When you are done configuring the {room.title()} area, please tell me to continue.",
                hint="Send {type:'advance'}"
            ),
            handle=lambda ev: c.handle_advance_only(ev)
        ))

        return steps

    def _rules_steps(self, round_name: str) -> List[Step]:
        c = self.ctx
        steps: List[Step] = []

        steps.append(Step(
            step_id=f'{round_name}_rules_intro',
            on_enter=lambda: Prompt(
                step_id=f'{round_name}_rules_intro',
                text=(
                    "Before we end this round, you may set home-wide situational settings, which apply regardless of room. "
                    "You may specify no-go zones and time-based rules."
                ),
                hint="Send {type:'advance'} to begin rules entry."
            ),
            handle=lambda ev: ev.type == 'advance'
        ))

        steps.append(Step(
            step_id=f'{round_name}_rules_set',
            on_enter=lambda: Prompt(
                step_id=f'{round_name}_rules_set',
                text=(
                    "Please set any situational constraints you want now. "
                    "(No-go zones and time-based settings.)"
                ),
                hint=(
                    "Send {type:'set', scope:'rules', value:{no_go_zones:[...], time_rules:[...]}} then {type:'advance'}"
                )
            ),
            handle=lambda ev: c.handle_rules_then_advance(ev)
        ))

        steps.append(Step(
            step_id=f'{round_name}_rules_complete',
            on_enter=lambda: Prompt(
                step_id=f'{round_name}_rules_complete',
                text=(
                    "Thank you. I have saved those situational rules."
                ),
                hint="Send {type:'advance'}"
            ),
            handle=lambda ev: ev.type == 'advance'
        ))

        return steps

    def _handle_round2_intro(self, ev: StudyEvent) -> bool:
        # allow either profile switch or advance; require profile switched before advancing
        if ev.type == 'profile':
            self.ctx.settings.switch_profile(str(ev.value))
            self.ctx.log_kv('profile_switch', {'active_profile': self.ctx.settings.active_profile})
            return False
        if ev.type == 'advance':
            return self.ctx.settings.active_profile == 'jackie'
        return False


class ScriptContext:
    """Hooks used by steps to mutate settings, request demos, and log."""

    def __init__(self, settings: Any, capabilities: Any, logger: Any):
        self.settings = settings
        self.cap = capabilities
        self.log = logger
        self.last_arrived_room: Optional[str] = None
        # For optional deterministic transit testing. Starts at the door (bottom-right corner).
        self.last_location: str = 'door'
        self._pending_demo_room: Optional[str] = None
        self._should_run_demo: Dict[str, bool] = {'desk': False, 'bed': False, 'kitchen': False}

    def log_kv(self, event_type: str, payload: Dict[str, Any]) -> None:
        self.log.log_event({'type': event_type, **payload})

    def handle_advance_only(self, ev: StudyEvent) -> bool:
        return ev.type == 'advance'

    def handle_profile_and_advance(self, profile: str, ev: StudyEvent) -> bool:
        if ev.type == 'profile':
            self.settings.switch_profile(str(ev.value))
            self.log_kv('profile_switch', {'active_profile': self.settings.active_profile})
            return False
        if ev.type == 'advance':
            self.settings.switch_profile(profile)
            self.log_kv('profile_switch', {'active_profile': self.settings.active_profile})
            return True
        return False

    def handle_arrive(self, room: str, ev: StudyEvent) -> bool:
        if ev.type == 'arrive' and ev.room == room:
            # Optional deterministic movement between stations (for controlled demo testing)
            try:
                eff = self.settings.effective_for_room(room)
                move_speed = str(eff.get('movement_speed', 'medium'))
                self.cap.transit(self.last_location, room, movement_speed=move_speed)
            except Exception as e:
                # Never fail the study on movement issues; log and proceed.
                self.log_kv('transit_error', {'from': self.last_location, 'to': room, 'error': str(e)})
            self.last_arrived_room = room
            self.last_location = room
            self.log_kv('arrive', {'room': room})
            return True
        return False

    def handle_set(self, scope: str, key: str, ev: StudyEvent) -> bool:
        if ev.type != 'set':
            return False
        if ev.scope != scope:
            return False
        if ev.key != key:
            return False
        try:
            self.settings.apply_event_set(scope=scope, key=key, value=ev.value, room=ev.room)
        except Exception as e:
            self.log_kv('set_error', {'scope': scope, 'key': key, 'value': ev.value, 'error': str(e)})
            return False

        self.log_kv('set', {'scope': scope, 'room': ev.room, 'key': key, 'value': ev.value, 'profile': self.settings.active_profile})
        # confirmation handled by UI/voice layer; engine just advances
        return True

    def handle_room_config_any(self, room: str, ev: StudyEvent) -> bool:
        # accept set events for this room but do not complete the step (participant sends advance on next step)
        if ev.type == 'set' and ev.scope == 'room' and ev.room == room:
            if ev.key is None:
                return False
            try:
                self.settings.apply_event_set(scope='room', key=str(ev.key), value=ev.value, room=room)
                self.log_kv('set', {'scope': 'room', 'room': room, 'key': ev.key, 'value': ev.value, 'profile': self.settings.active_profile})
            except Exception as e:
                self.log_kv('set_error', {'scope': 'room', 'room': room, 'key': ev.key, 'value': ev.value, 'error': str(e)})
            return False
        return False

    def handle_demo_confirm(self, room: str, ev: StudyEvent) -> bool:
        if ev.type == 'demo_confirm' and ev.room == room and isinstance(ev.yes, bool):
            self._should_run_demo[room] = bool(ev.yes)
            self.log_kv('demo_confirm', {'room': room, 'yes': bool(ev.yes)})
            return True
        return False

    def prompt_and_run_demo(self, room: str, step_id: str) -> Prompt:
        # Called on step enter: run demo if confirmed.
        should = self._should_run_demo.get(room, False)
        eff = self.settings.effective_for_room(room)
        self.log_kv('effective_settings', {'room': room, 'effective': eff, 'profile': self.settings.active_profile})

        if not should:
            return Prompt(step_id=step_id, text=f"Skipping {room.title()} demo.")

        # Run deterministic demo based on room + room-specific parameter
        if room == 'desk':
            self.cap.desk_demo(eff.get('cleaning_thoroughness', 'once'), eff)
        elif room == 'bed':
            self.cap.bed_demo(eff.get('pillow_arrangement', 'center'), eff)
        elif room == 'kitchen':
            self.cap.kitchen_demo(eff.get('snack_preference', 'doritos'), eff)

        return Prompt(step_id=step_id, text=f"Completed {room.title()} demo.")

    def handle_rules_then_advance(self, ev: StudyEvent) -> bool:
        if ev.type == 'set' and ev.scope == 'rules':
            try:
                self.settings.apply_event_set(scope='rules', key='rules', value=ev.value, room=None)
                self.log_kv('set', {'scope': 'rules', 'value': ev.value, 'profile': self.settings.active_profile})
            except Exception as e:
                self.log_kv('set_error', {'scope': 'rules', 'value': ev.value, 'error': str(e)})
            return False
        if ev.type == 'advance':
            return True
        return False
