"""Unelte: `report timings` (duratele măsurate → valori pentru `timing`) și `calib corners` (colțurile grilei)."""

from __future__ import annotations

import json
import re
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from brain.configuration.loader import load_system_config
from brain.domain.errors import BrainError, PerceptionInputError


def load_journal(paths: Sequence[Path]) -> List[Dict[str, Any]]:
    records = []
    for path in paths:
        for number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except ValueError:
                raise BrainError("{}:{}: linie JSON invalidă".format(path, number)) from None
    return records


def summarize_timings(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Statistici per comandă (doar comenzile reușite) și valorile propuse pentru secțiunea `timing`."""
    by_method: Dict[str, List[float]] = {}
    simulated = set()
    for record in records:
        if record.get("result") not in ("ok", "actionable"):
            continue
        by_method.setdefault(str(record.get("method")), []).append(float(record.get("duration_s", 0.0)))
        simulated.add(bool(record.get("simulated")))
    stats = {}
    for method, values in sorted(by_method.items()):
        ordered = sorted(values)
        p90 = ordered[min(len(ordered) - 1, int(round(0.9 * (len(ordered) - 1))))]
        stats[method] = {"n": len(values), "median_s": round(statistics.median(values), 2), "p90_s": round(p90, 2),
                         "max_s": round(max(values), 2)}
    proposal = {}
    for key, method in (("observe_s", "observe"), ("pick_s", "pick"), ("place_s", "place")):
        if method in stats:
            proposal[key] = stats[method]["p90_s"]  # p90: estimarea trebuie să fie prudentă
    return {"methods": stats, "proposal": proposal, "simulated": sorted(simulated)}


_LOCATION_RE = re.compile(r"^  (\w+):\s*(#.*)?$")


def write_expected_corners(system_file: Path, location: str, corners: Sequence[Tuple[float, float]]) -> None:
    """Scrie `expected_corners_px` al locației în system.yaml păstrând comentariile; revine dacă rezultatul e invalid."""
    original = system_file.read_text(encoding="utf-8")
    lines = original.splitlines(keepends=True)
    in_locations = False
    current: Optional[str] = None
    target_index = None
    for index, line in enumerate(lines):
        if line.startswith("locations:"):
            in_locations = True
            continue
        if in_locations and line and not line.startswith(" ") and not line.startswith("#") and line.strip():
            break
        match = _LOCATION_RE.match(line.rstrip("\n"))
        if in_locations and match:
            current = match.group(1)
        if in_locations and current == location and line.strip().startswith("expected_corners_px:"):
            target_index = index
            break
    if target_index is None:
        raise BrainError("{}: nu găsesc locations.{}.expected_corners_px".format(system_file, location))
    old = lines[target_index]
    indent = old[: len(old) - len(old.lstrip())]
    value = "[{}]".format(", ".join("[{:.1f}, {:.1f}]".format(x, y) for x, y in corners))
    comment = "  # din `brain calib corners` (TL, TR, BR, BL)"
    lines[target_index] = "{}expected_corners_px: {}{}\n".format(indent, value, comment)
    system_file.write_text("".join(lines), encoding="utf-8")
    try:
        load_system_config(system_file)
    except Exception:
        system_file.write_text(original, encoding="utf-8")
        raise


def corners_from_frames(config: Any, location: str, frames: Sequence[Any]) -> Tuple[Tuple[float, float], ...]:
    from brain.perception.pipeline import VisionPipeline

    observation = VisionPipeline(config.system, config.colors).analyze(frames, location).observation
    if not observation.geometry.ok or len(observation.geometry.corners_px) != 4:
        raise PerceptionInputError("grila {} nu a fost găsită sigur: {}".format(location, ", ".join(observation.reasons)))
    return observation.geometry.corners_px
