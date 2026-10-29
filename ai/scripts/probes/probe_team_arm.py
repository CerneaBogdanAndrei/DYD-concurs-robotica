"""Sondă: TeamArm (brain) împotriva serviciului colegilor pornit cu `npm run sim`."""
import json
import time
import urllib.request

from brain.adapters.arm.team_arm import TeamArm
from brain.domain.errors import ArmError

URL = "http://127.0.0.1:8765"
results = []


def check(name, fn):
    t0 = time.perf_counter()
    try:
        value = fn()
        status = "OK"
    except ArmError as exc:
        value = "{}: {}".format(type(exc).__name__, exc)
        status = "EXC"
    except Exception as exc:  # noqa
        value = "NEAȘTEPTAT {}: {}".format(type(exc).__name__, exc)
        status = "BUG"
    dt = (time.perf_counter() - t0) * 1000
    results.append((name, status, value))
    print("[{:<3}] {:<55} {:>6.0f} ms  {}".format(status, name, dt, value))
    return value


def raw(name, body):
    req = urllib.request.Request("{}/{}".format(URL, name), data=json.dumps(body).encode(), method="POST",
                                 headers={"content-type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {"http": e.code, **json.loads(e.read())}


arm = TeamArm(URL, timeout_s=10)
st = check("connect/status", arm.connect)
check("mode după connect", lambda: arm.mode)
check("home", arm.home)
check("go_pose observe_source fast", lambda: arm.go_pose("observe_source"))
check("go_pose observe_dest medium(->slow)", lambda: arm.go_pose("observe_dest", "medium"))
check("go_pose home", lambda: arm.go_pose("home"))
check("go_pose observe_medium (necalibrat)", lambda: arm.go_pose("observe_medium"))
check("go_pose speed invalid", lambda: arm.go_pose("home", "turbo"))
check("open_gripper", arm.open_gripper)
check("open_gripper(full=False)", lambda: arm.open_gripper(full=False))
check("close_gripper", arm.close_gripper)
check("read_joints", arm.read_joints)
check("move_joints 5", lambda: arm.move_joints([90, 90, 90, 90, 90], 500))
check("move_joints 6", lambda: arm.move_joints([90, 90, 90, 90, 90, 60], 500))
check("move_joints unghi 200 (refuz local)", lambda: arm.move_joints([90, 90, 90, 90, 200], 500))
check("beep 2", lambda: arm.beep(2))
check("beep 300 ms (refuz local)", lambda: arm.beep(1, 300))
for cell in [(0, 0), (0, 2), (1, 1), (2, 0), (2, 2)]:
    check("pick source{} yaw 10".format(cell), lambda c=cell: arm.pick("source", c, yaw_deg=10.0))
    check("place dest{}".format(cell), lambda c=cell: arm.place("dest", c))
check("pick source (1,1) uv=(1.62,1.41)", lambda: arm.pick("source", (1, 1), uv=(1.62, 1.41)))
check("pick zone_red level 0", lambda: arm.pick("zone_red"))
check("place zone_blue level 2", lambda: arm.place("zone_blue", None, level=2))
check("place zone_red cu cell (servicu: argument)", lambda: arm.place("zone_red", (0, 0)))
check("pick source fără cell (serviciu: argument)", lambda: arm.pick("source", None))
check("pick small (necalibrat)", lambda: arm.pick("small", (0, 0)))
check("pick level 9 (refuz local)", lambda: arm.pick("source", (0, 0), level=9))
check("pick uv=(3.5,1) (refuz local)", lambda: arm.pick("source", (0, 0), uv=(3.5, 1.0)))

print("\n--- semnul yaw: servo 5 după pick prin TeamArm vs. HTTP brut")
for yaw in (20.0, -20.0):
    arm.pick("source", (1, 1), yaw_deg=yaw)
    s5_team = arm.status()["joints"][4]
    raw("pick", {"location": "source", "cell": [1, 1], "yaw_deg": yaw})
    s5_raw = arm.status()["joints"][4]
    raw("pick", {"location": "source", "cell": [1, 1], "yaw_deg": -yaw})
    s5_raw_neg = arm.status()["joints"][4]
    print("  yaw brain {:+.0f} -> s5 {} | brut {:+.0f} -> s5 {} | brut {:+.0f} -> s5 {}".format(
        yaw, s5_team, yaw, s5_raw, -yaw, s5_raw_neg))

print("\n--- accesibilitate pe niveluri (place, serviciul simulat, geometrie NECALIBRATĂ din config.ts)")
sites = st["sites"]
for name, kind in sites.items():
    cells = [(r, c) for r in range(3) for c in range(3)] if kind == "grid" else [None]
    row = []
    for level in range(9):
        ok_cells = 0
        for cell in cells:
            if arm.place(name, cell, level=level):
                ok_cells += 1
        row.append("{}/{}".format(ok_cells, len(cells)))
    print("  {:<12} niv 0..8: {}".format(name, "  ".join(row)))

print("\n--- timeout scurt (0.001 s) pe go_pose")
fast = TeamArm(URL, timeout_s=0.001)
check("go_pose cu timeout 1 ms", lambda: fast.go_pose("observe_source"))
print("\n--- serviciu oprit (port greșit)")
dead = TeamArm("http://127.0.0.1:8799", timeout_s=2)
check("status pe port închis", dead.status)
print("\n--- GET /status și rută necunoscută")
with urllib.request.urlopen(URL + "/status", timeout=5) as r:
    print("  GET /status:", r.status)
print("  POST /fly:", raw("fly", {}))
print("  GET /home:", end=" ")
try:
    urllib.request.urlopen(URL + "/home", timeout=5)
except urllib.error.HTTPError as e:
    print(e.code, e.read().decode())
arm.home()
