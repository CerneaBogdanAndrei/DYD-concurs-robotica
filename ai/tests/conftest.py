from pathlib import Path

import pytest

from brain.configuration import load_app_config
from brain.configuration.loader import AppConfig

PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PROJECT_DIR / "config"


@pytest.fixture(scope="session")
def config_dir() -> Path:
    return CONFIG_DIR


@pytest.fixture(scope="session")
def app_config() -> AppConfig:
    return load_app_config(CONFIG_DIR)
