"""TeamArm: `ArmInterface` peste serviciul de mișcare al colegilor (TypeScript pe Raspberry Pi, HTTP local).

Implementat direct după contractul HTTP din `docs/CONTRACT_BRAT.md` (repo-ul de mișcare), doar cu
biblioteca standard. Diferențe de convenție tratate aici:
- yaw: serviciul măsoară unghiul în imaginea redresată de la axa coloanelor spre `r` crescător
  (sens orar pe ecran); sistemul folosește sens antiorar pozitiv → semnul se inversează;
- `speed`: serviciul are doar "fast" și "slow"; "medium" e trimis ca "slow" (variantă prudentă);
- funcțiile pe care serviciul nu le oferă (deschidere parțială, bip de altă durată) ridică
  `CapabilityUnavailable` în loc să raporteze un succes fals.
"""

from __future__ import annotations

import json
import socket
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

from brain.adapters.arm import validation as v
from brain.domain.cube import normalize_yaw_deg
from brain.domain.errors import (
    ArmCommunicationError,
    ArmTimeout,
    CalibrationMissing,
    CapabilityUnavailable,
    InvalidCommand,
)
from brain.ports.arm import MODE_HARDWARE, MODE_SIMULATION

SERVICE_BEEP_MS = 100


def to_service_yaw(yaw_deg: float) -> float:
    """Convenția sistemului (antiorar pozitiv) → convenția serviciului (spre r crescător pozitiv)."""
    return normalize_yaw_deg(-yaw_deg)


class TeamArm:
    def __init__(self, url: str = "http://127.0.0.1:8765", timeout_s: float = 60.0) -> None:
        self.url = url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.last_reason: Optional[str] = None
        self.mode = MODE_HARDWARE

    # ------------------------------------------------------------------ transport
    def _call(self, name: str, body: Optional[Dict[str, Any]] = None) -> Any:
        data = json.dumps(body or {}).encode("utf-8")
        request = urllib.request.Request(
            "{}/{}".format(self.url, name), data=data, method="POST", headers={"content-type": "application/json"}
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raise self._service_error(exc) from None
        except socket.timeout:
            raise ArmTimeout("{}: serviciul nu a răspuns în {} s (starea brațului e necunoscută)".format(name, self.timeout_s)) from None
        except (urllib.error.URLError, ConnectionError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            if isinstance(reason, socket.timeout):
                raise ArmTimeout("{}: timeout la {}".format(name, self.url)) from None
            raise ArmCommunicationError("serviciul de mișcare nu răspunde la {}: {}".format(self.url, reason)) from None
        except ValueError as exc:
            raise ArmCommunicationError("{}: răspuns JSON invalid: {}".format(name, exc)) from None
        if not isinstance(payload, dict) or "result" not in payload:
            raise ArmCommunicationError("{}: răspuns fără câmpul 'result': {!r}".format(name, payload))
        return payload

    @staticmethod
    def _service_error(exc: "urllib.error.HTTPError") -> Exception:
        try:
            error = json.loads(exc.read().decode("utf-8"))["error"]
            code, message = str(error.get("code", "hardware")), str(error.get("message", ""))
        except Exception:
            return ArmCommunicationError("HTTP {} fără detalii de la serviciu".format(exc.code))
        text = "[{}] {}".format(code, message)
        if code in ("argument", "limite"):
            return InvalidCommand(text)
        if code == "calibrare":
            return CalibrationMissing(text)
        if code == "ruta":
            return CapabilityUnavailable(text)
        return ArmCommunicationError(text)

    def _outcome(self, payload: Dict[str, Any]) -> bool:
        result = payload["result"]
        if not isinstance(result, bool):
            raise ArmCommunicationError("rezultat pick/place invalid: {!r}".format(result))
        self.last_reason = None if result else str(payload.get("reason") or "motiv nespecificat")
        return result

    # ------------------------------------------------------------------ extra
    def status(self) -> Dict[str, Any]:
        result = self._call("status")["result"]
        if not isinstance(result, dict):
            raise ArmCommunicationError("status invalid: {!r}".format(result))
        return result

    def connect(self) -> Dict[str, Any]:
        """Verifică serviciul și setează `mode` după câmpul `sim` din status."""
        status = self.status()
        self.mode = MODE_SIMULATION if status.get("sim") else MODE_HARDWARE
        return status

    # ------------------------------------------------------------------ ArmInterface
    def home(self) -> None:
        self._call("home")

    def go_pose(self, name: str, speed: str = "fast") -> None:
        v.check_speed(speed)
        if not isinstance(name, str) or not name:
            raise InvalidCommand("numele pozei trebuie să fie un șir nevid")
        self._call("go_pose", {"name": name, "speed": "fast" if speed == "fast" else "slow"})

    def move_joints(self, angles: List[float], time_ms: int) -> None:
        values = v.check_angles(angles)
        self._call("move_joints", {"angles": values, "time_ms": v.check_time_ms(time_ms)})

    def open_gripper(self, full: bool = True) -> None:
        if not full:
            raise CapabilityUnavailable("serviciul de mișcare nu oferă deschiderea parțială a ghearei")
        self._call("open_gripper")

    def close_gripper(self) -> None:
        self._call("close_gripper")

    def pick(
        self,
        location: str,
        cell: Optional[Tuple[int, int]] = None,
        yaw_deg: float = 0.0,
        level: int = 0,
        uv: Optional[Tuple[float, float]] = None,
    ) -> bool:
        cell = v.check_cell(cell)
        body: Dict[str, Any] = {
            "location": v.check_location(location),
            "cell": None if cell is None else [cell[0], cell[1]],
            "yaw_deg": to_service_yaw(v.check_yaw(yaw_deg)),
            "level": v.check_level(level),
        }
        checked_uv = v.check_uv(uv, cell)
        if checked_uv is not None:
            body["uv"] = [checked_uv[0], checked_uv[1]]
        return self._outcome(self._call("pick", body))

    def place(self, location: str, cell: Optional[Tuple[int, int]] = None, level: int = 0, yaw_deg: float = 0.0) -> bool:
        cell = v.check_cell(cell)
        body = {
            "location": v.check_location(location),
            "cell": None if cell is None else [cell[0], cell[1]],
            "level": v.check_level(level),
            "yaw_deg": to_service_yaw(v.check_yaw(yaw_deg)),
        }
        return self._outcome(self._call("place", body))

    def read_joints(self) -> List[float]:
        values = self._call("read_joints")["result"]
        if not isinstance(values, list) or len(values) != 6:
            raise ArmCommunicationError("read_joints a întors {!r}".format(values))
        failed = [i + 1 for i, value in enumerate(values) if not isinstance(value, (int, float)) or isinstance(value, bool)]
        if failed:
            raise ArmCommunicationError("citire eșuată pentru servo {}".format(failed))
        return [float(value) for value in values]

    def beep(self, count: int = 1, duration_ms: int = 100) -> None:
        v.check_beep(count, duration_ms)
        if duration_ms != SERVICE_BEEP_MS:
            raise CapabilityUnavailable("serviciul emite doar bipuri de {} ms".format(SERVICE_BEEP_MS))
        self._call("beep", {"n": count})
