from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional

from .types import (
    MovementSpeed, Explainability, Confirmation, SocialDistance, VoiceProfile,
    DeskCleaning, PillowArrangement, SnackPreference,
)


def _clamp_int(x: Any, lo: int, hi: int) -> int:
    try:
        xi = int(x)
    except Exception as e:
        raise ValueError(f"expected int, got {x!r}") from e
    return max(lo, min(hi, xi))


@dataclass
class GlobalSettings:
    movement_speed: MovementSpeed = 'medium'
    voice_volume: int = 60
    voice_profile: VoiceProfile = 'neutral'
    explainability: Explainability = 'short'
    confirmation: Confirmation = 'confirm'
    social_distance: SocialDistance = 'medium'

    def apply(self, key: str, value: Any) -> None:
        if key == 'movement_speed':
            if value not in ('slow', 'medium', 'fast'):
                raise ValueError('movement_speed must be slow|medium|fast')
            self.movement_speed = value
        elif key == 'voice_volume':
            self.voice_volume = _clamp_int(value, 0, 100)
        elif key == 'voice_profile':
            if value not in ('neutral', 'friendly', 'playful'):
                raise ValueError('voice_profile must be neutral|friendly|playful')
            self.voice_profile = value
        elif key == 'explainability':
            if value not in ('none', 'short', 'full'):
                raise ValueError('explainability must be none|short|full')
            self.explainability = value
        elif key == 'confirmation':
            if value not in ('confirm', 'silent'):
                raise ValueError('confirmation must be confirm|silent')
            self.confirmation = value
        elif key == 'social_distance':
            if value not in ('close', 'medium', 'far'):
                raise ValueError('social_distance must be close|medium|far')
            self.social_distance = value
        else:
            raise ValueError(f"unknown global setting: {key}")


@dataclass
class DeskSettings:
    movement_speed: Optional[MovementSpeed] = None
    voice_volume: Optional[int] = None
    explainability: Optional[Explainability] = None
    social_distance: Optional[SocialDistance] = None
    cleaning_thoroughness: DeskCleaning = 'once'

    def apply(self, key: str, value: Any) -> None:
        if key in ('movement_speed', 'voice_volume', 'explainability', 'social_distance'):
            # room overrides allow same keys as global
            g = GlobalSettings()
            g.apply(key, value)
            setattr(self, key, getattr(g, key))
            return
        if key == 'cleaning_thoroughness':
            if value not in ('once', 'twice', 'thorough', 'none'):
                raise ValueError('cleaning_thoroughness must be once|twice|thorough|none')
            self.cleaning_thoroughness = value
        else:
            raise ValueError(f"unknown desk setting: {key}")


@dataclass
class BedSettings:
    movement_speed: Optional[MovementSpeed] = None
    voice_volume: Optional[int] = None
    explainability: Optional[Explainability] = None
    social_distance: Optional[SocialDistance] = None
    pillow_arrangement: PillowArrangement = 'center'

    def apply(self, key: str, value: Any) -> None:
        if key in ('movement_speed', 'voice_volume', 'explainability', 'social_distance'):
            g = GlobalSettings()
            g.apply(key, value)
            setattr(self, key, getattr(g, key))
            return
        if key == 'pillow_arrangement':
            if value not in ('center', 'top'):
                raise ValueError('pillow_arrangement must be center|top')
            self.pillow_arrangement = value
        else:
            raise ValueError(f"unknown bed setting: {key}")


@dataclass
class KitchenSettings:
    movement_speed: Optional[MovementSpeed] = None
    voice_volume: Optional[int] = None
    explainability: Optional[Explainability] = None
    social_distance: Optional[SocialDistance] = None
    snack_preference: SnackPreference = 'doritos'

    def apply(self, key: str, value: Any) -> None:
        if key in ('movement_speed', 'voice_volume', 'explainability', 'social_distance'):
            g = GlobalSettings()
            g.apply(key, value)
            setattr(self, key, getattr(g, key))
            return
        if key == 'snack_preference':
            if value not in ('doritos', 'cheetos'):
                raise ValueError('snack_preference must be doritos|cheetos')
            self.snack_preference = value
        else:
            raise ValueError(f"unknown kitchen setting: {key}")


@dataclass
class SituationalRules:
    # Keep flexible: stored as structured dicts so you can evolve UI later
    no_go_zones: list = field(default_factory=list)
    time_rules: list = field(default_factory=list)

    def apply(self, value: Any) -> None:
        if not isinstance(value, dict):
            raise ValueError('rules value must be an object/dict')
        if 'no_go_zones' in value:
            self.no_go_zones = value['no_go_zones'] or []
        if 'time_rules' in value:
            self.time_rules = value['time_rules'] or []


@dataclass
class Profile:
    name: str
    global_defaults: GlobalSettings = field(default_factory=GlobalSettings)
    desk: DeskSettings = field(default_factory=DeskSettings)
    bed: BedSettings = field(default_factory=BedSettings)
    kitchen: KitchenSettings = field(default_factory=KitchenSettings)
    rules: SituationalRules = field(default_factory=SituationalRules)

    def snapshot(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class StudySettings:
    """Holds both profiles and resolves effective settings."""

    user: Profile = field(default_factory=lambda: Profile(name='user'))
    jackie: Profile = field(default_factory=lambda: Profile(name='jackie'))
    active_profile: str = 'user'

    def active(self) -> Profile:
        return self.user if self.active_profile == 'user' else self.jackie

    def switch_profile(self, name: str) -> None:
        if name not in ('user', 'jackie'):
            raise ValueError('profile must be user|jackie')
        self.active_profile = name

    def apply_event_set(self, scope: str, key: str, value: Any, room: Optional[str] = None) -> None:
        prof = self.active()
        if scope == 'global':
            # voice_profile is global-only by design
            prof.global_defaults.apply(key, value)
        elif scope == 'room':
            if room == 'desk':
                prof.desk.apply(key, value)
            elif room == 'bed':
                prof.bed.apply(key, value)
            elif room == 'kitchen':
                prof.kitchen.apply(key, value)
            else:
                raise ValueError('room must be desk|bed|kitchen')
        elif scope == 'rules':
            # rules are applied as a whole object in value
            prof.rules.apply(value)
        else:
            raise ValueError('scope must be global|room|rules')

    def effective_for_room(self, room: str) -> Dict[str, Any]:
        """Resolve effective settings for a room (global defaults + room overrides)."""
        prof = self.active()
        g = prof.global_defaults
        eff = {
            'movement_speed': g.movement_speed,
            'voice_volume': g.voice_volume,
            'voice_profile': g.voice_profile,
            'explainability': g.explainability,
            'confirmation': g.confirmation,
            'social_distance': g.social_distance,
        }
        if room == 'desk':
            r = prof.desk
            eff.update({k: v for k, v in {
                'movement_speed': r.movement_speed,
                'voice_volume': r.voice_volume,
                'explainability': r.explainability,
                'social_distance': r.social_distance,
            }.items() if v is not None})
            eff['cleaning_thoroughness'] = r.cleaning_thoroughness
        elif room == 'bed':
            r = prof.bed
            eff.update({k: v for k, v in {
                'movement_speed': r.movement_speed,
                'voice_volume': r.voice_volume,
                'explainability': r.explainability,
                'social_distance': r.social_distance,
            }.items() if v is not None})
            eff['pillow_arrangement'] = r.pillow_arrangement
        elif room == 'kitchen':
            r = prof.kitchen
            eff.update({k: v for k, v in {
                'movement_speed': r.movement_speed,
                'voice_volume': r.voice_volume,
                'explainability': r.explainability,
                'social_distance': r.social_distance,
            }.items() if v is not None})
            eff['snack_preference'] = r.snack_preference
        else:
            raise ValueError('room must be desk|bed|kitchen')
        return eff
