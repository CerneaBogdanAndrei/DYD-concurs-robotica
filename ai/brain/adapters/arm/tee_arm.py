"""TeeArm: fiecare comandă merge la serviciul colegilor (pornit cu `--sim`) ȘI la `MockArm` (lumea simulată).

Serviciul verifică contractul real (nume de locuri, accesibilitate după cinematica lor, coduri de eroare), iar
`MockArm` ține scena, ca percepția simulată să vadă efectul mutărilor. Dacă serviciul refuză (`False` sau
excepție), lumea simulată nu se schimbă. Doar pentru teste de integrare, niciodată cu brațul real.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from brain.adapters.arm.mock_arm import MockArm
from brain.adapters.arm.team_arm import TeamArm
from brain.domain.errors import SafetyRejected
from brain.ports.arm import MODE_SIMULATION


class TeeArm:
    def __init__(self, service: TeamArm, shadow: MockArm) -> None:
        self.service = service
        self.shadow = shadow
        self.mode = MODE_SIMULATION

    @property
    def last_reason(self) -> Optional[str]:
        return self.service.last_reason or self.shadow.last_reason

    @property
    def position_uncertain(self) -> bool:
        return self.service.position_uncertain or self.shadow.position_uncertain

    def connect(self) -> Dict[str, Any]:
        status = self.service.connect()
        if not status.get("sim"):
            raise SafetyRejected("TeeArm cere serviciul pornit în simulare (status.sim = true); refuz brațul real")
        return status

    def home(self) -> None:
        self.service.home()
        self.shadow.home()

    def go_pose(self, name: str, speed: str = "fast") -> None:
        self.service.go_pose(name, speed)
        self.shadow.go_pose(name, speed)

    def move_joints(self, angles: List[float], time_ms: int) -> None:
        self.service.move_joints(angles, time_ms)
        self.shadow.move_joints(angles, time_ms)

    def open_gripper(self, full: bool = True) -> None:
        self.service.open_gripper(full)
        self.shadow.open_gripper(full)

    def close_gripper(self) -> None:
        self.service.close_gripper()
        self.shadow.close_gripper()

    def pick(self, location: str, cell: Optional[Tuple[int, int]] = None, yaw_deg: float = 0.0, level: int = 0,
             uv: Optional[Tuple[float, float]] = None, open_mm: Optional[float] = None) -> bool:
        if not self.service.pick(location, cell, yaw_deg=yaw_deg, level=level, uv=uv, open_mm=open_mm):
            return False
        return self.shadow.pick(location, cell, yaw_deg=yaw_deg, level=level, uv=uv, open_mm=open_mm)

    def place(self, location: str, cell: Optional[Tuple[int, int]] = None, level: int = 0, yaw_deg: float = 0.0,
              release_mm: Optional[float] = None, uv: Optional[Tuple[float, float]] = None) -> bool:
        if not self.service.place(location, cell, level=level, yaw_deg=yaw_deg, release_mm=release_mm, uv=uv):
            return False
        return self.shadow.place(location, cell, level=level, yaw_deg=yaw_deg, release_mm=release_mm, uv=uv)

    def read_joints(self) -> List[float]:
        return self.service.read_joints()

    def beep(self, count: int = 1, duration_ms: int = 100) -> None:
        self.service.beep(count, duration_ms)
