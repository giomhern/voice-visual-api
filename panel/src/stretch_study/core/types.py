from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Literal

MovementSpeed = Literal['slow', 'medium', 'fast']
Explainability = Literal['none', 'short', 'full']
Confirmation = Literal['confirm', 'silent']
SocialDistance = Literal['close', 'medium', 'far']
VoiceProfile = Literal['neutral', 'friendly', 'playful']

RoomName = Literal['desk', 'bed', 'kitchen']

DeskCleaning = Literal['once', 'twice', 'thorough', 'none']
PillowArrangement = Literal['center', 'top']
SnackPreference = Literal['doritos', 'cheetos']


@dataclass
class StudyEvent:
    """Normalized event emitted by any modality.

    All adapters should publish JSON matching this structure.
    """

    type: str
    room: Optional[str] = None
    scope: Optional[str] = None
    key: Optional[str] = None
    value: Any = None
    yes: Optional[bool] = None

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'StudyEvent':
        return StudyEvent(
            type=str(d.get('type', '')).strip(),
            room=d.get('room'),
            scope=d.get('scope'),
            key=d.get('key'),
            value=d.get('value'),
            yes=d.get('yes'),
        )


@dataclass
class Prompt:
    step_id: str
    text: str
    options: Optional[List[Any]] = None
    hint: Optional[str] = None
    payload: Optional[Dict[str, Any]] = None
