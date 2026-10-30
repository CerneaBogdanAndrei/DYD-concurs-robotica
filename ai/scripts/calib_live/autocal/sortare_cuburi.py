"""Sortarea cuburilor: harta culorilor din caroiaj (pasul 1), apoi cuburile scoase de echipă în stânga, în dreapta și
sub caroiaj sunt puse înapoi pe celulele culorii lor (pașii 2-4).

Punctul de referință e centrul lui r1c1 (mijlocul caroiajului), văzut din poza de start (observe_small).

Pasul 1: echipa pune cuburile (cel mult 9) pe celule; două analize la rând din poza de start trebuie să dea aceleași
celule ocupate, cu aceleași culori. Harta se scrie în `harta_culori.json` (cu imaginea adnotată, `harta_culori.jpg`)
și rămâne până la sfârșitul tuturor pașilor.

    ai/.venv/bin/python ai/scripts/calib_live/autocal/sortare_cuburi.py pas1
"""
import argparse
import json
import shutil
import sys
import time

from traseu_cub import HERE, ROOT, analyses, say, start_pose

MAP = HERE / "harta_culori.json"
COLORS = {"red": "roșu", "yellow": "galben", "green": "verde", "blue": "albastru"}
CENTER_UV = (1.5, 1.5)  # punctul de referință: centrul lui r1c1, mijlocul caroiajului


def cell_colors(obs):
    """{'r0c0': 'yellow', ...} pentru celulele ocupate; None dacă analiza nu e sigură (caroiajul negăsit, o celulă
    nesigură sau un cub cu o culoare necunoscută)."""
    if not obs["geometry"].get("ok"):
        return None
    out = {}
    for c in obs["cells"]:
        if c["occupancy"] == "empty":
            continue
        if c["occupancy"] != "occupied" or c.get("color") not in COLORS:
            return None
        out[f"r{c['row']}c{c['col']}"] = c["color"]
    return out


def table(cells):
    return "\n".join("  " + " | ".join(f"{COLORS.get(cells.get(f'r{r}c{c}'), '-'):8s}" for c in range(3))
                     for r in range(3))


def pas1():
    start_pose()
    for _ in range(5):
        (o1, _), (o2, _) = analyses(2)
        a, b = cell_colors(o1), cell_colors(o2)
        if a is not None and a == b:
            break
    else:
        sys.exit("Două analize la rând nu dau aceleași celule și culori (sau o celulă e nesigură): verificați cuburile.")
    shutil.copy(ROOT / "ai/out/preview/latest_annotated.jpg", MAP.with_suffix(".jpg"))
    json.dump({"at": time.strftime("%Y-%m-%d %H:%M:%S"), "center_uv": CENTER_UV, "cells": b,
               "colors_ro": {k: COLORS[v] for k, v in b.items()}, "image": MAP.with_suffix(".jpg").name},
              open(MAP, "w"), indent=1, ensure_ascii=False)
    say(f"Harta culorilor ({len(b)} cuburi), salvată în {MAP.name}:\n{table(b)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pas", choices=["pas1"])
    a = ap.parse_args()
    if a.pas == "pas1":
        pas1()


if __name__ == "__main__":
    main()
