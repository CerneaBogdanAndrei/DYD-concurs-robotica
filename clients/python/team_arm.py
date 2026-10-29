"""
TeamArm: implementarea ArmInterface pentru creier, peste serviciul de mișcare
(TypeScript, pe Raspberry Pi). Doar biblioteca standard, fără dependențe.

Contractul complet: docs/CONTRACT_BRAT.md din repo-ul serviciului de mișcare.

    arm = TeamArm()                       # http://127.0.0.1:8765
    arm.go_pose("observe_source")
    if not arm.pick("source", (1, 2), yaw_deg=-12.0):
        print("inaccesibil:", arm.last_reason)
    arm.place("dest", (1, 2))

Celulele sunt (r, c) cu valori 0..2. Fiecare apel se întoarce DUPĂ ce mișcarea
s-a terminat. Erorile (argument greșit, calibrare lipsă, I2C) ridică ArmError.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request


class ArmError(RuntimeError):
    """Eroare raportată de serviciu. code: argument | limite | calibrare | hardware | ruta | conexiune."""

    def __init__(self, code: str, message: str, status: int | None = None):
        super().__init__(f"[{code}] {message}")
        self.code = code
        self.message = message
        self.status = status


def _cell(cell):
    return None if cell is None else [int(cell[0]), int(cell[1])]


class TeamArm:
    def __init__(self, url: str = "http://127.0.0.1:8765", timeout: float = 60.0):
        self.url = url.rstrip("/")
        self.timeout = timeout
        #: motivul ultimului pick/place care a întors False
        self.last_reason: str | None = None

    # ------------------------------------------------------------------ transport
    def _call(self, name: str, body: dict | None = None):
        data = json.dumps(body or {}).encode("utf-8")
        req = urllib.request.Request(
            f"{self.url}/{name}", data=data, method="POST",
            headers={"content-type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as res:
                return json.loads(res.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                err = json.loads(e.read().decode("utf-8"))["error"]
            except Exception:
                raise ArmError("hardware", f"HTTP {e.code}", e.code) from None
            raise ArmError(err.get("code", "hardware"), err.get("message", ""), e.code) from None
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            raise ArmError("conexiune", f"serviciul de mișcare nu răspunde la {self.url}: {e}") from None

    # ------------------------------------------------------------------ ArmInterface
    def home(self) -> None:
        """Brațul drept în sus (toate servo-urile la 90°), gheara rămâne cum e."""
        self._call("home")

    def go_pose(self, name: str, speed: str = "fast") -> None:
        """Poză învățată cu nume ("home", "observe_source", ...). speed: "fast" | "slow"."""
        self._call("go_pose", {"name": name, "speed": speed})

    def move_joints(self, angles: list[float], time_ms: int) -> None:
        """5 unghiuri (servo 1..5, gheara neschimbată) sau 6 (inclusiv gheara), în grade."""
        self._call("move_joints", {"angles": [float(a) for a in angles], "time_ms": int(time_ms)})

    def open_gripper(self) -> None:
        self._call("open_gripper")

    def close_gripper(self) -> None:
        self._call("close_gripper")

    def pick(self, location: str, cell: tuple[int, int] | None = None, yaw_deg: float = 0,
             *, level: int = 0, uv: tuple[float, float] | None = None) -> bool:
        """
        Ia cubul din `location` (caroiaj: cu `cell`; zonă: fără). Deschide gheara,
        tranzit sus, coborâre în pași, strânge, ridică deasupra locului.
        yaw_deg: rotația cubului față de axa coloanelor (imagine top-down).
        level: de pe ce nivel al unei stive (0 = masă). uv: poziția exactă (0..3).
        False = punct inaccesibil (motivul în last_reason); brațul nu a coborât.
        """
        body = {"location": location, "cell": _cell(cell), "yaw_deg": float(yaw_deg), "level": int(level)}
        if uv is not None:
            body["uv"] = [float(uv[0]), float(uv[1])]
        return self._outcome(self._call("pick", body))

    def place(self, location: str, cell: tuple[int, int] | None = None, level: int = 0,
              yaw_deg: float = 0) -> bool:
        """Pune cubul ținut în gheară în `location` / `cell`, la nivelul `level` al stivei."""
        body = {"location": location, "cell": _cell(cell), "level": int(level), "yaw_deg": float(yaw_deg)}
        return self._outcome(self._call("place", body))

    def read_joints(self) -> list[float | None]:
        """Unghiurile celor 6 servo-uri; None pentru o citire eșuată."""
        return self._call("read_joints")["result"]

    def beep(self, n: int = 1) -> None:
        self._call("beep", {"n": int(n)})

    # ------------------------------------------------------------------ extra
    def status(self) -> dict:
        """sim, poze și locuri calibrate, ultima poziție comandată."""
        return self._call("status")["result"]

    def _outcome(self, res: dict) -> bool:
        self.last_reason = res.get("reason")
        return bool(res["result"])


if __name__ == "__main__":
    # Probă rapidă: npm run sim (în repo-ul de mișcare), apoi python team_arm.py
    import sys

    arm = TeamArm(sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765")
    st = arm.status()
    print("status:", st)
    arm.go_pose("observe_source") if "observe_source" in st["poses"] else arm.home()
    print("pick source (0,0):", arm.pick("source", (0, 0), yaw_deg=10))
    print("place dest (0,0):", arm.place("dest", (0, 0)))
    print("joints:", arm.read_joints())
    try:
        arm.pick("source", (3, 3))
    except ArmError as e:
        print("eroare asteptata:", e)  # fara diacritice: consola Windows (cp1252)
    arm.home()
