"""Contractul formal cu subsistemul de mișcare.

Semnăturile de bază sunt cele din specificație. `level` și `uv` (la `pick`) și `level` (la `place`)
sunt extensiile convenite cu echipa de mișcare în `docs/CONTRACT_BRAT.md` (repo-ul lor).

Semantică comună tuturor metodelor:
- toate sunt BLOCANTE: se întorc după ce mișcarea s-a terminat;
- unghiurile servo sunt în grade, convenția `Arm_Lib`, ordinea ID 1..6;
- `yaw_deg` e în convenția sistemului: sens antiorar pozitiv văzut de sus, față de axele caroiajului
  (adaptorul convertește spre convenția serviciului);
- `InvalidCommand`: comanda e respinsă înainte de trimitere, brațul nu s-a mișcat;
- `CalibrationMissing`: poza/locul nu e calibrat;
- `MotionAborted`: serviciul a oprit un pick/place pe traseu (de ex. limită articulară); serviciul răspunde,
  dar poziția e incertă → `go_pose(observe_*)` și reobservare;
- `ArmCommunicationError` / `ArmTimeout`: starea fizică e NECUNOSCUTĂ; nicio repetare automată,
  niciun `home()` automat; urmează reobservare sau intervenție manuală;
- `CapabilityUnavailable`: adaptorul nu are funcția (niciodată un succes fals).

`pick`/`place` întorc `True` dacă secvența s-a executat fără eroare detectată și `False` dacă ținta e
inaccesibilă (motivul în `last_reason`). Cu `False` brațul NU a coborât, dar poate să se fi deplasat pe traseu
(constatarea 1 din `docs/questions-for-kinematics-team.md`): `position_uncertain` devine True până la
următorul `home`/`go_pose`. `True` NU dovedește că
cubul a fost prins sau așezat: confirmarea vine din reobservarea cu camera.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

try:  # Python 3.8+
    from typing import Literal, Protocol
except ImportError:  # pragma: no cover
    from typing_extensions import Literal, Protocol  # type: ignore

Speed = Literal["slow", "medium", "fast"]
Cell = Tuple[int, int]
MODE_SIMULATION = "simulation"
MODE_HARDWARE = "hardware"


class ArmInterface(Protocol):
    """Contract formal de comunicație cu subsistemul cinematic."""

    #: "simulation" sau "hardware" — verificat la pornire, ca MockArm să nu fie confundat cu brațul real
    mode: str
    #: motivul ultimului pick/place care a întors False
    last_reason: Optional[str]
    #: True după un pick/place = False sau o eroare cu stare necunoscută; resetat de home/go_pose
    position_uncertain: bool

    def home(self) -> None:
        """Aduce brațul în configurația sigură de repaus."""
        ...

    def go_pose(self, name: str, speed: Speed = "fast") -> None:
        """Deplasează brațul într-o configurație învățată (poza de observare etc.)."""
        ...

    def move_joints(self, angles: List[float], time_ms: int) -> None:
        """Comandă servourile ID 1..6 (sau 1..5, cu gheara neschimbată), în grade [0, 180]."""
        ...

    def open_gripper(self, full: bool = True) -> None:
        """Deschide cleștele."""
        ...

    def close_gripper(self) -> None:
        """Închide cleștele conform configurației de prindere."""
        ...

    def pick(
        self,
        location: str,
        cell: Optional[Cell] = None,
        yaw_deg: float = 0.0,
        level: int = 0,
        uv: Optional[Tuple[float, float]] = None,
        open_mm: Optional[float] = None,
    ) -> bool:
        """Preia cubul: apropiere de sus, coborâre, prindere, ridicare deasupra locului.

        `open_mm` = deschiderea de abordare (mm, interioară): gheara se pre-închide la ea înainte de deplasare
        (regula A); None = alege serviciul. Un adaptor care nu poate aplica deschiderea o ignoră."""
        ...

    def place(self, location: str, cell: Optional[Cell] = None, level: int = 0, yaw_deg: float = 0.0,
              release_mm: Optional[float] = None, uv: Optional[Tuple[float, float]] = None) -> bool:
        """Depune cubul ținut: apropiere, coborâre, eliberare, ridicare deasupra locului.

        `release_mm` = deschiderea la eliberare (mm, interioară, mică: regula C); None = alege serviciul.
        `uv` = unde se pune cubul în caroiaj (la stivuire: peste cubul de dedesubt); None = centrul celulei."""
        ...

    def read_joints(self) -> List[float]:
        """Unghiurile curente în ordinea ID1..ID6; `ArmCommunicationError` dacă o citire eșuează."""
        ...

    def beep(self, count: int = 1, duration_ms: int = 100) -> None:
        """Semnalizare acustică."""
        ...
