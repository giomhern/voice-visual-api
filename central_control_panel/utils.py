# utils.py
# Python 3.8+ compatible
#
# Logging + persistence utilities for Stretch study:
# - timeline.jsonl (append-only)
# - state_snapshot.json (exhaustive snapshot)
# - synced via monotonically increasing event_id stored in snapshot.last_event_id
#
# Usage from main:
#   from utils import StudyLogger, ensure_exhaustive_profiles
#
#   self.logger = StudyLogger(data_dir=self.data_dir, log_fn=self.get_logger().info)
#   eid = self.logger.log_event(..., phase=self.phase.name, profile=self.study.active_profile, room=self.study.current_room)
#   snap = self.logger.build_snapshot(controller=self, last_event_id=eid)
#   self.logger.write_snapshot(snap)

import json
import os
from datetime import datetime
from typing import Any, Callable, Dict, Optional, List


def atomic_write_json(path: str, obj: Dict[str, Any]) -> None:
    """Atomically write JSON to path (write temp + fsync + rename)."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def append_jsonl(path: str, obj: Dict[str, Any]) -> None:
    """Append a JSON object as one line to a JSONL file (with fsync)."""
    line = json.dumps(obj) + "\n"
    with open(path, "a") as f:
        f.write(line)
        f.flush()
        os.fsync(f.fileno())


def safe_read_json(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return None


def ensure_exhaustive_profiles(
    profiles_dict: Dict[str, Any],
    demo_defaults: Optional[Dict[str, Dict[str, Any]]] = None,
    room_names: Optional[List[str]] = None,
) -> None:
    """
    Mutates profiles_dict in-place to ensure:
      - all rooms exist
      - each room contains ALL global keys + demo keys (even if never changed)
      - rules include 'no_go' and 'time_rules'
    profiles_dict is expected to be like:
      {
        "user": ProfileData-like,
        "jackie": ProfileData-like
      }
    where each profile object has:
      .global_defaults (dict)
      .rooms (dict of room->dict)
      .rules (dict)
    """
    if room_names is None:
        room_names = ["desk", "bed", "kitchen"]

    if demo_defaults is None:
        demo_defaults = {
            "desk": {"cleaning_thoroughness": "once"},   # or "none"
            "bed": {"pillow_arrangement": "center"},
            "kitchen": {"snack_preference": "cheetos"},
        }

    for _, pd in profiles_dict.items():
        # Ensure container dicts exist
        if getattr(pd, "rooms", None) is None:
            pd.rooms = {}
        if getattr(pd, "rules", None) is None:
            pd.rules = {}

        # Ensure rules keys exist
        if "no_go" not in pd.rules:
            pd.rules["no_go"] = []
        if "time_rules" not in pd.rules:
            pd.rules["time_rules"] = []

        # Fill each room with exhaustive keys
        for room in room_names:
            if room not in pd.rooms or pd.rooms[room] is None:
                pd.rooms[room] = {}

            full = dict(pd.global_defaults)  # all global keys
            full.update(demo_defaults.get(room, {}))      # demo keys for this room
            full.update(pd.rooms.get(room, {}))           # overrides
            pd.rooms[room] = dict(full)


class StudyLogger:
    """
    Handles:
      - timeline.jsonl: append-only event log
      - state_snapshot.json: exhaustive snapshot with last_event_id
    """
    def __init__(
        self,
        data_dir: str,
        timeline_filename: str = "timeline.jsonl",
        snapshot_filename: str = "state_snapshot.json",
        log_fn: Optional[Callable[[str], None]] = None,
    ):
        self.data_dir = data_dir
        os.makedirs(self.data_dir, exist_ok=True)

        self.timeline_path = os.path.join(self.data_dir, timeline_filename)
        self.snapshot_path = os.path.join(self.data_dir, snapshot_filename)
        self.log_fn = log_fn

        self.next_event_id = 1
        self._load_event_counter_from_snapshot()

    def _log(self, msg: str) -> None:
        if self.log_fn:
            try:
                self.log_fn(msg)
            except Exception:
                pass

    def _load_event_counter_from_snapshot(self) -> None:
        snap = safe_read_json(self.snapshot_path)
        if not snap:
            self.next_event_id = 1
            return
        try:
            last_id = int(snap.get("last_event_id", 0))
            self.next_event_id = last_id + 1
        except Exception:
            self.next_event_id = 1

    def log_event(
        self,
        etype: str,
        payload: Dict[str, Any],
        phase: str,
        profile: str,
        room: Optional[str] = None,
        step_id: Optional[str] = None,
    ) -> int:
        """
        Append an event to timeline.jsonl and return event_id.
        """
        evt = {
            "event_id": self.next_event_id,
            "ts": datetime.now().isoformat(),
            "type": etype,
            "phase": phase,
            "step_id": step_id,
            "profile": profile,
            "room": room,
            "payload": payload,
        }
        append_jsonl(self.timeline_path, evt)
        self._log("Logged event %s (%d)" % (etype, evt["event_id"]))
        self.next_event_id += 1
        return int(evt["event_id"])

    def build_snapshot(
        self,
        controller: Any,
        last_event_id: int,
        room_names: Optional[List[str]] = None,
        demo_defaults: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> Dict[str, Any]:
        """
        Build an exhaustive snapshot from the live controller object.

        Expects controller to have (best-effort):
          - controller.phase.name
          - controller.step_index
          - controller.pending_steps (list with .step_id)
          - controller.study.active_profile
          - controller.study.current_room
          - controller.study.demo_confirm
          - controller.study.waiting_for
          - controller.mode_follow
          - controller.estop
          - controller.profiles dict of ProfileData-like
        """
        if room_names is None:
            room_names = ["desk", "bed", "kitchen"]

        # Ensure exhaustive per-room keys exist inside profiles
        try:
            ensure_exhaustive_profiles(controller.profiles, demo_defaults=demo_defaults, room_names=room_names)
        except Exception:
            # We still proceed; snapshot will be partially exhaustive if structure differs
            pass

        pending_step_id = None
        try:
            if 0 <= controller.step_index < len(controller.pending_steps):
                pending_step_id = controller.pending_steps[controller.step_index].step_id
        except Exception:
            pending_step_id = None

        # Build profile dict safely
        profiles_out: Dict[str, Any] = {}
        try:
            for name, pd in controller.profiles.items():
                profiles_out[name] = {
                    "global_defaults": getattr(pd, "global_defaults", {}),
                    "rooms": getattr(pd, "rooms", {}),
                    "rules": getattr(pd, "rules", {}),
                }
        except Exception:
            profiles_out = {}

        # Study fields safely
        active_profile = None
        current_room = None
        demo_confirm = None
        waiting_for = None
        try:
            active_profile = controller.study.active_profile
            current_room = controller.study.current_room
            demo_confirm = controller.study.demo_confirm
            waiting_for = controller.study.waiting_for
        except Exception:
            pass

        phase_name = None
        try:
            phase_name = controller.phase.name
        except Exception:
            phase_name = str(getattr(controller, "phase", ""))

        return {
            "ts": datetime.now().isoformat(),
            "last_event_id": int(last_event_id),
            "phase": phase_name,
            "step_index": int(getattr(controller, "step_index", 0)),
            "pending_step_id": pending_step_id,
            "active_profile": active_profile,
            "current_room": current_room,
            "mode_follow": bool(getattr(controller, "mode_follow", False)),
            "estop": bool(getattr(controller, "estop", False)),
            "study": {
                "demo_confirm": demo_confirm,
                "waiting_for": waiting_for,
            },
            "profiles": profiles_out,
        }

    def write_snapshot(self, snapshot: Dict[str, Any]) -> None:
        atomic_write_json(self.snapshot_path, snapshot)
        self._log("Wrote snapshot (last_event_id=%s)" % snapshot.get("last_event_id"))

    def checkpoint(
        self,
        controller: Any,
        label: str,
        phase: str,
        profile: str,
        room: Optional[str] = None,
        step_id: Optional[str] = None,
        extra_payload: Optional[Dict[str, Any]] = None,
    ) -> int:
        """
        Convenience: log a checkpoint event and immediately write snapshot.
        """
        payload = {"label": label}
        if extra_payload:
            payload.update(extra_payload)

        eid = self.log_event(
            etype="checkpoint",
            payload=payload,
            phase=phase,
            profile=profile,
            room=room,
            step_id=step_id,
        )
        snap = self.build_snapshot(controller=controller, last_event_id=eid)
        self.write_snapshot(snap)
        return eid
