"""
Cinematica directa/inversa pentru Dofbot + wrapper peste Arm_Lib.

Sistemul de coordonate al robotului: originea pe axa bazei, la nivelul mesei,
x spre fata (servo 1 = 90), z in sus. Toate valorile in mm / grade.
"""
import math
import time

import config as C


class Unreachable(Exception):
    pass


# ------------------------------------------------------------------ cinematica
def fk(s):
    """s = [s1, s2, s3, s4] (grade servo) -> (x, y, z, pitch) al punctului de prindere."""
    s1, s2, s3, s4 = s[:4]
    th = math.radians(C.BASE_SIGN * (s1 - 90))
    a2 = math.radians(s2)
    a3 = math.radians(s2 + s3 - 90)
    a4 = math.radians(s2 + s3 + s4 - 180)
    r = C.L2 * math.cos(a2) + C.L3 * math.cos(a3) + C.L4 * math.cos(a4)
    z = C.L1 + C.L2 * math.sin(a2) + C.L3 * math.sin(a3) + C.L4 * math.sin(a4)
    return r * math.cos(th), r * math.sin(th), z, math.degrees(a4)


def ik_planar(r, z, pitch_deg):
    """IK in planul bratului. pitch = unghiul ghearei fata de orizontala (-90 = in jos)."""
    a4 = math.radians(pitch_deg)
    wr = r - C.L4 * math.cos(a4)
    wz = z - C.L4 * math.sin(a4) - C.L1
    c = (wr * wr + wz * wz - C.L2 ** 2 - C.L3 ** 2) / (2 * C.L2 * C.L3)
    if c < -1.0 or c > 1.0:
        return None
    e = -math.acos(c)  # cotul "deasupra" liniei umar-incheietura
    a2 = math.atan2(wz, wr) - math.atan2(C.L3 * math.sin(e), C.L2 + C.L3 * math.cos(e))
    a3 = a2 + e
    s2 = math.degrees(a2)
    s3 = math.degrees(e) + 90
    s4 = pitch_deg - math.degrees(a3) + 90
    for v in (s2, s3, s4):
        if not (0.0 <= v <= 180.0):
            return None
    return [s2, s3, s4]


def ik(x, y, z, pitches=None):
    """-> ([s1, s2, s3, s4], pitch) sau None."""
    th = math.degrees(math.atan2(y, x))
    s1 = 90 + C.BASE_SIGN * th
    if not (0.0 <= s1 <= 180.0):
        return None
    r = math.hypot(x, y)
    for p in (pitches or C.PITCH_CANDIDATES):
        sol = ik_planar(r, z, p)
        if sol:
            return [s1] + sol, p
    return None


def wrist_for_yaw(s1, yaw_deg):
    """Unghiul servo 5 astfel incat degetele sa fie aliniate cu fetele unui cub
    rotit cu yaw_deg (in sistemul robotului). Cubul e simetric la 90 de grade."""
    th = C.BASE_SIGN * (s1 - 90)
    s5 = C.J5_CENTER + C.WRIST_SIGN * (yaw_deg - th)
    while s5 - C.J5_CENTER > 45:
        s5 -= 90
    while s5 - C.J5_CENTER < -45:
        s5 += 90
    return max(C.J5_MIN, min(C.J5_MAX, s5))


# ------------------------------------------------------------------ brat
class Arm:
    def __init__(self, sim=False):
        self.sim = sim
        self.dev = None
        if not sim:
            from Arm_Lib import Arm_Device  # biblioteca Yahboom de pe imaginea Dofbot
            self.dev = Arm_Device()
            time.sleep(0.1)
        self.grip = C.GRIP_OPEN
        self.cur = list(C.HOME) + [self.grip]
        self.cur_xyz = None  # pozitia carteziana curenta (daca e cunoscuta)

    # --- nivel jos
    def write6(self, s, t_ms):
        s = [int(round(v)) for v in s]
        if self.sim:
            print(f"   [sim] servo {s} in {t_ms} ms")
        else:
            self.dev.Arm_serial_servo_write6(s[0], s[1], s[2], s[3], s[4], s[5], int(t_ms))
            time.sleep(t_ms / 1000.0 + 0.12)
        self.cur = s

    def gripper(self, val, t_ms=C.T_GRIP):
        self.grip = val
        if self.sim:
            print(f"   [sim] gheara -> {val}")
        else:
            self.dev.Arm_serial_servo_write(6, int(val), int(t_ms))
            time.sleep(t_ms / 1000.0 + 0.1)
        self.cur[5] = int(val)

    def read(self):
        """Citeste cele 6 servo-uri (functioneaza si cu cuplul oprit)."""
        if self.sim:
            return list(self.cur)
        out = []
        for i in range(1, 7):
            v = None
            for _ in range(6):
                v = self.dev.Arm_serial_servo_read(i)
                if v is not None:
                    break
                time.sleep(0.03)
            out.append(v)
        return out

    def torque(self, on):
        if not self.sim:
            self.dev.Arm_serial_set_torque(1 if on else 0)

    def beep(self, units=1):
        if self.sim:
            return
        try:
            self.dev.Arm_Buzzer_On(units)
        except Exception:
            pass

    def pose(self, servos5, t_ms=C.T_MOVE):
        """Trimite o poza articulara (5 servo-uri) pastrand starea ghearei."""
        self.write6(list(servos5) + [self.grip], t_ms)
        self.cur_xyz = None

    def home(self):
        self.pose(C.HOME, 1200)

    # --- nivel cartezian
    def goto(self, x, y, z, yaw=0.0, pitches=None, t_ms=C.T_MOVE):
        sol = ik(x, y, z, pitches)
        if sol is None:
            raise Unreachable(f"punct inaccesibil x={x:.0f} y={y:.0f} z={z:.0f}")
        s, p = sol
        s5 = wrist_for_yaw(s[0], yaw)
        self.write6(s + [s5, self.grip], t_ms)
        self.cur_xyz = (x, y, z)
        return p

    @staticmethod
    def _near(pref):
        """Inclinarile candidate, cele mai apropiate de cea preferata primele."""
        return sorted(C.PITCH_CANDIDATES, key=lambda p: (abs(p - pref), p))

    def _line(self, x, y, z0, z1, yaw, pitch, t_ms):
        """Coborare/ridicare aproape verticala (in FINE_STEPS pasi). Inclinarea ramane
        cea ceruta daca se poate, altfel cea mai apropiata posibila."""
        n = max(1, C.FINE_STEPS)
        for i in range(1, n + 1):
            z = z0 + (z1 - z0) * i / n
            self.goto(x, y, z, yaw, self._near(pitch), t_ms / n)

    def _pitch_for(self, x, y, z_low, z_high):
        """Inclinarea pentru punctul de jos (prindere/plasare); intai una comuna cu apropierea."""
        for p in C.PITCH_CANDIDATES:
            if ik(x, y, z_low, [p]) and ik(x, y, z_high, [p]):
                return p
        for p in C.PITCH_CANDIDATES:
            if ik(x, y, z_low, [p]):
                return p
        raise Unreachable(f"punct inaccesibil x={x:.0f} y={y:.0f} z={z_low:.0f}")

    def _approach(self, x, y, z_hover, yaw, pitch):
        """Ajunge deasupra (x, y) fara sa treaca jos peste alte obiecte."""
        z_t = max(z_hover, C.TRANSIT_MIN_Z)
        if self.cur_xyz is not None:
            cx, cy, cz = self.cur_xyz
            z_t = max(z_t, cz)
            if cz < z_t:
                self._line(cx, cy, cz, z_t, yaw, pitch, C.T_FINE)
        self.goto(x, y, z_t, yaw, self._near(pitch), C.T_MOVE)
        if z_t > z_hover:
            self._line(x, y, z_t, z_hover, yaw, pitch, C.T_FINE)

    def pick(self, x, y, z, yaw=0.0):
        zh = z + C.HOVER_DZ
        p = self._pitch_for(x, y, z, zh)
        if self.grip != C.GRIP_OPEN:
            self.gripper(C.GRIP_OPEN)
        self._approach(x, y, zh, yaw, p)
        self._line(x, y, zh, z, yaw, p, C.T_FINE)
        self.gripper(C.GRIP_CLOSED)
        self._line(x, y, z, zh, yaw, p, C.T_FINE)

    def place(self, x, y, z, yaw=0.0):
        z = z + C.PLACE_GAP
        zh = z + C.HOVER_DZ
        p = self._pitch_for(x, y, z, zh)
        self._approach(x, y, zh, yaw, p)
        self._line(x, y, zh, z, yaw, p, C.T_FINE)
        self.gripper(C.GRIP_OPEN)
        self._line(x, y, z, zh, yaw, p, C.T_FINE)
