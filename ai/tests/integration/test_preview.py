import dataclasses
import json
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import pytest

from brain.adapters.camera import LoopingFileCamera
from brain.adapters.preview_server import PreviewHub, PreviewServer
from brain.cli.preview import PreviewSession
from brain.perception.image_io import write_image
from brain.perception.pipeline import VisionPipeline
from brain.simulation.scene_gen import generate_scene


def _get(url, timeout=5.0):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.status, response.headers.get("Content-Type"), response.read()


def _wait_for(predicate, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def running_preview(app_config, tmp_path):
    scene = generate_scene(905, app_config.system.grid_geometry("medium"), frames=5)
    paths = [str(write_image(tmp_path / "frame_{}.png".format(k), frame)) for k, frame in enumerate(scene.frames)]
    settings = dataclasses.replace(app_config.system.preview, analyze_interval_s=0.2, max_fps=30.0)
    hub = PreviewHub("medium")
    server = PreviewServer(hub, "127.0.0.1", 0).start()
    session = PreviewSession(
        LoopingFileCamera(paths), hub, settings, pipeline=VisionPipeline(app_config.system, app_config.colors),
        save_dir=tmp_path / "preview",
    ).start()
    yield "http://127.0.0.1:{}".format(server.port), session, tmp_path / "preview"
    session.stop()
    server.stop()


def test_preview_serves_page_snapshot_and_analysis(running_preview):
    base, session, save_dir = running_preview
    status, content_type, page = _get(base + "/")
    assert status == 200 and content_type.startswith("text/html") and b"/stream.mjpg" in page
    assert _wait_for(lambda: (save_dir / "latest_observation.json").is_file()), session.last_error
    _, content_type, body = _get(base + "/snapshot.jpg")
    assert content_type == "image/jpeg"
    assert cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR).shape == (480, 640, 3)
    annotated = cv2.imdecode(np.frombuffer(_get(base + "/annotated.jpg")[2], np.uint8), cv2.IMREAD_COLOR)
    assert annotated is not None
    status_json = json.loads(_get(base + "/status.json")[2].decode("utf-8"))
    assert status_json["location"] == "medium" and status_json["frames"] > 0
    assert status_json["analysis"]["actionable"] is True and len(status_json["analysis"]["cells"]) == 9
    assert (save_dir / "latest_raw.jpg").is_file() and (save_dir / "latest_annotated.jpg").is_file()
    assert session.last_error is None


def test_mjpeg_stream_delivers_jpeg_parts(running_preview):
    base = running_preview[0]
    with urllib.request.urlopen(base + "/stream.mjpg", timeout=5.0) as response:
        assert response.headers.get("Content-Type").startswith("multipart/x-mixed-replace")
        data = b""
        while data.count(b"--frame") < 2:
            chunk = response.read(16384)
            assert chunk, "fluxul s-a închis"
            data += chunk
    assert b"Content-Type: image/jpeg" in data and b"\xff\xd8" in data  # antet de part + început de JPEG


def test_annotated_is_404_before_first_analysis():
    server = PreviewServer(PreviewHub("medium"), "127.0.0.1", 0).start()
    try:
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            _get("http://127.0.0.1:{}/annotated.jpg".format(server.port))
        assert exc_info.value.code == 404
    finally:
        server.stop()
