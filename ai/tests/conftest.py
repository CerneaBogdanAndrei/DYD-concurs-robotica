import re
from pathlib import Path

import pytest

from brain.configuration import load_app_config
from brain.configuration.loader import AppConfig

PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PROJECT_DIR / "config"
# Scenariile de test (sintetice) sunt construite pe șablonul medium. Configul real poate avea alt șablon activ
# (cel de pe masă la probă); testele folosesc o copie cu medium activ, ca să nu depindă de foaia aleasă.
TEST_ACTIVE_LOCATION = "medium"


@pytest.fixture(scope="session")
def config_dir(tmp_path_factory) -> Path:
    folder = tmp_path_factory.mktemp("config")
    for source in CONFIG_DIR.glob("*.yaml"):
        text = source.read_text(encoding="utf-8")
        if source.name == "system.yaml":  # înlocuire de text: comentariile rămân (unele teste le verifică)
            for key in ("active_locations", "staging_locations"):
                text, count = re.subn(r"(?m)^(  {}: )\[[^\]]*\]".format(key), r"\g<1>[{}]".format(TEST_ACTIVE_LOCATION), text)
                assert count == 1, key
        (folder / source.name).write_text(text, encoding="utf-8")
    return folder


@pytest.fixture(scope="session")
def app_config(config_dir) -> AppConfig:
    return load_app_config(config_dir)
