from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


class StudyLogger:
    def __init__(self, output_dir: str, session_id: str, participant_id: str):
        self.session_id = session_id
        self.participant_id = participant_id

        out = os.path.expanduser(output_dir)
        self.base = Path(out) / session_id
        self.base.mkdir(parents=True, exist_ok=True)

        self.events_path = self.base / "events.jsonl"
        self.meta_path = self.base / "meta.json"

        # Write meta if not exists
        if not self.meta_path.exists():
            meta = {
                "session_id": session_id,
                "participant_id": participant_id,
                "created_at": datetime.utcnow().isoformat() + "Z",
            }
            self.meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")

    def _ts(self) -> str:
        return datetime.utcnow().isoformat() + "Z"

    def log_event(self, payload: Dict[str, Any]) -> None:
        rec = {
            "ts": self._ts(),
            "session_id": self.session_id,
            "participant_id": self.participant_id,
            **payload,
        }
        with self.events_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    # Backwards-compatible alias (so calling code can do logger.log(...))
    def log(self, payload: Dict[str, Any]) -> None:
        self.log_event(payload)

    def snapshot(self, name: str, data: Dict[str, Any]) -> None:
        snap_dir = self.base / "snapshots"
        snap_dir.mkdir(exist_ok=True)

        path = snap_dir / f"{name}.json"
        path.write_text(
            json.dumps(
                {
                    "ts": self._ts(),
                    "session_id": self.session_id,
                    "participant_id": self.participant_id,
                    "data": data,
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )