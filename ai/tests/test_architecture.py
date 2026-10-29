"""Reguli de arhitectură verificate static și la import (fără hardware)."""

import ast
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "brain"
PROJECT = PACKAGE.parent

HARDWARE_MODULES = {"Arm_Lib", "smbus", "smbus2", "serial", "RPi", "spidev", "gpiozero", "pigpio"}
# Singurele module care vor avea voie să importe biblioteci hardware (Etapa 3): adaptorul Yahboom.
HARDWARE_ALLOWED = {"brain/adapters/arm/yahboom_arm.py"}
PURE_LAYERS = {"domain": {"cv2", "numpy", "yaml"}, "memory": {"cv2", "numpy"}, "configuration": {"cv2", "numpy"}}


def _python_files(root: Path):
    return sorted(p for p in root.rglob("*.py") if "__pycache__" not in p.parts)


def _imported_roots(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name.split(".")[0]
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            yield node.module.split(".")[0]


@pytest.mark.parametrize("path", _python_files(PACKAGE), ids=lambda p: str(p.relative_to(PROJECT)))
def test_no_hardware_imports_outside_adapter(path):
    relative = path.relative_to(PROJECT).as_posix()
    if relative in HARDWARE_ALLOWED:
        return
    forbidden = set(_imported_roots(path)) & HARDWARE_MODULES
    assert not forbidden, "{} importă module hardware: {}".format(relative, forbidden)


@pytest.mark.parametrize("layer, banned", sorted(PURE_LAYERS.items()))
def test_pure_layers(layer, banned):
    for path in _python_files(PACKAGE / layer):
        used = set(_imported_roots(path)) & banned
        assert not used, "{} importă {}".format(path.relative_to(PROJECT), used)


def test_importing_everything_touches_no_hardware():
    """Importă toate modulele într-un proces curat și verifică că nicio bibliotecă hardware nu e încărcată."""
    code = (
        "import importlib, pkgutil, sys, brain\n"
        "for m in pkgutil.walk_packages(brain.__path__, 'brain.'):\n"
        "    if m.name != 'brain.__main__': importlib.import_module(m.name)\n"
        "bad = sorted(n for n in sys.modules if n.split('.')[0] in {})\n"
        "print(','.join(bad))\n"
    ).format(sorted(HARDWARE_MODULES))
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=str(PROJECT), timeout=120)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == ""


@pytest.mark.parametrize("path", _python_files(PACKAGE) + _python_files(PROJECT / "tests"),
                         ids=lambda p: str(p.relative_to(PROJECT)))
def test_python38_syntax(path):
    """Codul trebuie să ruleze pe Python 3.8 (versiunea de pe Pi e încă necunoscută)."""
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3, 8))
