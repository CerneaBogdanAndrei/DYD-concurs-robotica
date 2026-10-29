"""Accesibilitate pe niveluri, cu și fără resetarea poziției (home) înainte de fiecare apel."""
from brain.adapters.arm.team_arm import TeamArm

arm = TeamArm("http://127.0.0.1:8765", timeout_s=10)
sites = arm.connect()["sites"]

print("--- place cu home() înainte (accesibilitate pură, geometrie NECALIBRATĂ)")
for name, kind in sites.items():
    cells = [(r, c) for r in range(3) for c in range(3)] if kind == "grid" else [None]
    row = []
    for level in range(9):
        ok = 0
        for cell in cells:
            arm.home()
            ok += arm.place(name, cell, level=level)
        row.append("{}/{}".format(ok, len(cells)))
    print("  {:<12} niv 0..8: {}".format(name, "  ".join(row)))

print("\n--- pick cu home() înainte, per nivel (nivelul de pe care se ia)")
for name in ("source", "dest"):
    row = []
    for level in range(9):
        ok = 0
        for cell in [(r, c) for r in range(3) for c in range(3)]:
            arm.home()
            ok += arm.pick(name, cell, level=level)
        row.append("{}/9".format(ok))
    print("  {:<12} niv 0..8: {}".format(name, "  ".join(row)))

print("\n--- secvența turnului: place nivel k pe source(1,1), apoi imediat pick pe fiecare celulă a lui dest, nivel 0")
for level in range(7):
    arm.home()
    arm.place("source", (1, 1), level=level)
    xyz = arm.status()["xyz"]
    ok = 0
    reasons = []
    for cell in [(r, c) for r in range(3) for c in range(3)]:
        arm.place("source", (1, 1), level=level)  # readuce brațul deasupra turnului
        good = arm.pick("dest", cell)
        ok += good
        if not good:
            reasons.append("{}:{}".format(cell, arm.last_reason))
    print("  după place niv {} (gheara la {}): pick dest 9 celule -> {}/9 {}".format(level, xyz, ok, reasons[:2]))

print("\n--- aceeași secvență, dar cu go_pose('observe_dest') între place și pick")
for level in range(7):
    ok = 0
    for cell in [(r, c) for r in range(3) for c in range(3)]:
        arm.home()
        arm.place("source", (1, 1), level=level)
        arm.go_pose("observe_dest")
        ok += arm.pick("dest", cell)
    print("  niv {}: {}/9".format(level, ok))
arm.home()
