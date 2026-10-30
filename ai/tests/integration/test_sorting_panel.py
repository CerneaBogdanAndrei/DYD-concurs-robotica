import json
import sys
import time
import urllib.error
import urllib.request

import pytest

from brain.adapters.preview_server import ApiError, PreviewHub, PreviewServer
from brain.cli.sorting import SortingRunner, default_script

# Un script de sortare de probă: pasul 1 scrie harta și poza ei; „tot” scrie în jurnal până primește SIGINT.
FAKE = '''
import json, sys, time
from pathlib import Path
here = Path(__file__).parent
if sys.argv[1] == "pas1":
    (here / "harta_culori.jpg").write_bytes(b"\\xff\\xd8poza\\xff\\xd9")
    json.dump({"at": "2026-10-30 08:00:00", "cells": {"r0c0": "red", "r1c1": "green"}}, open(here / "harta_culori.json", "w"))
    print("Harta culorilor (2 cuburi)")
else:
    json.dump({"filled": {"r0c0": {"color": "red", "good": True}}}, open(here / "sortare_stare.json", "w"))
    print("Pasul 2, stânga", flush=True)
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("oprit")
        sys.exit(1)
'''


def _wait(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("condiția nu s-a îndeplinit la timp")


@pytest.fixture
def runner(tmp_path):
    script = tmp_path / "sortare_cuburi.py"
    script.write_text(FAKE, encoding="utf-8")
    return SortingRunner(script, tmp_path / "out" / "sortare.log", python=sys.executable)


def test_default_script_is_the_repo_sorting_script():
    assert default_script().name == "sortare_cuburi.py"
    assert default_script().parent.name == "autocal"


def test_photo_then_start_then_stop(runner):
    assert runner.status()["run"] is None and runner.status()["map"] is None
    assert runner.map_image() is None
    runner.post("poza")
    _wait(lambda: not runner.status()["running"])
    status = runner.status()
    assert status["run"]["exit_code"] == 0 and status["map"]["cells"] == {"r0c0": "red", "r1c1": "green"}
    assert status["map"]["image"] and runner.map_image().startswith(b"\xff\xd8")
    assert status["log"] == ["Harta culorilor (2 cuburi)"]

    runner.post("start")
    _wait(lambda: runner.status()["log"] == ["Pasul 2, stânga"])
    with pytest.raises(ApiError) as busy:
        runner.post("poza")  # o singură rulare odată
    assert busy.value.status == 409
    assert runner.status()["filled"] == {"r0c0": {"color": "red", "good": True}}
    assert runner.post("stop")["run"]["stopping"]
    _wait(lambda: not runner.status()["running"])
    assert runner.status()["run"]["exit_code"] == 1 and runner.status()["log"][-1] == "oprit"
    with pytest.raises(ApiError):
        runner.post("altceva")


def _request(url, data=None):
    req = urllib.request.Request(url, data=data, method="POST" if data is not None else "GET")
    with urllib.request.urlopen(req, timeout=5.0) as response:
        return response.status, response.read()


def test_server_routes_and_page(runner):
    server = PreviewServer(PreviewHub("small"), "127.0.0.1", 0, sorter=runner).start()
    base = "http://127.0.0.1:{}".format(server.port)
    try:
        status, page = _request(base + "/")
        assert b'id="s-card"' in page and b'"sorting": true' in page
        with pytest.raises(urllib.error.HTTPError) as missing:
            _request(base + "/sortare/harta.jpg")
        assert missing.value.code == 404
        status, body = _request(base + "/sortare/poza", data=b"")
        assert status == 200 and json.loads(body)["run"]["action"] == "poza"
        _wait(lambda: not runner.status()["running"])
        status, body = _request(base + "/sortare")
        assert json.loads(body)["map"]["cells"]["r1c1"] == "green"
        status, body = _request(base + "/sortare/harta.jpg")
        assert status == 200 and body.startswith(b"\xff\xd8")
    finally:
        server.stop()


def test_page_without_sorter_has_no_sorting_panel():
    server = PreviewServer(PreviewHub("small"), "127.0.0.1", 0).start()
    try:
        _, page = _request("http://127.0.0.1:{}/".format(server.port))
        assert b'id="s-card"' not in page
        with pytest.raises(urllib.error.HTTPError) as err:
            _request("http://127.0.0.1:{}/sortare".format(server.port))
        assert err.value.code == 404
    finally:
        server.stop()
