"""Cea mai bună punere de la învățare, pe fiecare celulă: comanda (uv trimis și încheietura) cu care cubul a ajuns cel
mai aproape de centru și cel mai drept. Sortarea o folosește o singură dată pe cub, fără corecții (echipa, 2026-10-30:
„nu mai pune un cub de mai multe ori, ia cea mai bună așezare din procesele de învățare”).

Se caută în jurnalele punerilor (`traseu_invatare.json`, `traseu_cub.json`, `sortare_cuburi.json`), în toate versiunile
lor din git și în cele de pe disc; scorul unei puneri e distanța de centru (mm) + 0,3 x rotirea (grd), iar o punere BINE
are întâietate. Rezultatul: `puneri_bune.json`.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/puneri_bune.py
"""
import json
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "puneri_bune.json"
LOGS = ["traseu_invatare.json", "traseu_cub.json", "sortare_cuburi.json"]
YAW_WEIGHT = 0.3  # mm de scor pentru fiecare grad de rotire


def versions(name):
    """Conținutul jurnalului `name` în fiecare commit care l-a schimbat, plus cel de pe disc."""
    rel = (HERE / name).relative_to(HERE.parents[3])
    root = str(HERE.parents[3])
    revs = subprocess.run(["git", "-C", root, "log", "--all", "--format=%h", "--", str(rel)],
                          capture_output=True, text=True).stdout.split()
    for rev in revs:
        shown = subprocess.run(["git", "-C", root, "show", f"{rev}:{rel}"], capture_output=True, text=True)
        if shown.returncode == 0:
            yield f"{name}@{rev}", shown.stdout
    if (HERE / name).exists():
        yield name, (HERE / name).read_text()


def score(m):
    return (0 if m.get("good") else 1, m["off_mm"] + YAW_WEIGHT * abs(m.get("yaw") or 0.0))


def main():
    seen, best = set(), {}
    for name in LOGS:
        for source, text in versions(name):
            try:
                moves = json.loads(text).get("moves", [])
            except ValueError:
                continue
            for m in moves:
                if "sent_uv" not in m or m.get("off_mm") is None:
                    continue
                key = (m["cell"], tuple(round(v, 4) for v in m["sent_uv"]), round(m["yaw_cmd"], 2))
                if key in seen:
                    continue
                seen.add(key)
                if m["cell"] not in best or score(m) < score(best[m["cell"]]):
                    best[m["cell"]] = dict(m, source=source)
    cells = {c: {"sent_uv": m["sent_uv"], "yaw_cmd": m["yaw_cmd"], "off_mm": round(m["off_mm"], 2),
                 "yaw": m.get("yaw"), "margins_mm": m.get("margins_mm"), "good": m.get("good"), "from": m.get("from"),
                 "source": m["source"]} for c, m in sorted(best.items())}
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "placements_seen": len(seen), "cells": cells},
              open(OUT, "w"), indent=1, ensure_ascii=False)
    for c, m in cells.items():
        print(f"{c}: {m['off_mm']:.1f} mm, rotit {m['yaw'] or 0:+.1f} grd{' (BINE)' if m['good'] else ''}; "
              f"uv ({m['sent_uv'][0]:.3f}; {m['sent_uv'][1]:.3f}), încheietura {m['yaw_cmd']:+.1f} grd  [{m['source']}]")
    missing = [f"r{r}c{c}" for r in range(3) for c in range(3) if f"r{r}c{c}" not in cells]
    print(f"{len(seen)} puneri; " + (f"fără punere învățată: {', '.join(missing)}" if missing else "toate celulele au una"))


if __name__ == "__main__":
    main()
