from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class SeqStep:
    kind: str           # 'move' or 'dwell'
    dwell_ms: int = 0
    speed: int = 50
    pos_name: Optional[str] = None
    angles: Optional[List[int]] = None
    label: str = ""
