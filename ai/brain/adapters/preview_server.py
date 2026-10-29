"""Previzualizarea camerei prin HTTP (doar biblioteca standard): Pi-ul nu are ecran.

`PreviewHub` ține ultimele imagini (cadrul brut și analiza adnotată), publicate de firul de captură.
Serverul le servește, doar pentru citire:
- `/`               pagina: fluxul live + ultima analiză + rezumatul celulelor
- `/stream.mjpg`    flux MJPEG (multipart/x-mixed-replace) cu cadrele brute
- `/snapshot.jpg`   ultimul cadru brut
- `/annotated.jpg`  ultima analiză adnotată (404 până la prima analiză)
- `/status.json`    starea: locația, fps, rezultatul ultimei analize
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

BOUNDARY = "frame"

_PAGE = """<!doctype html>
<html lang="ro"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Brain - camera</title>
<style>
body {{ margin: 0; padding: 12px; background: #1d1f21; color: #e8e8e8; font: 14px/1.4 system-ui, sans-serif; }}
h1 {{ font-size: 16px; margin: 0 0 8px; }}
.row {{ display: flex; flex-wrap: wrap; gap: 12px; }}
figure {{ margin: 0; flex: 1 1 420px; min-width: 0; }}
img {{ width: 100%; height: auto; background: #000; display: block; }}
figcaption {{ color: #aaa; margin-top: 4px; }}
pre {{ background: #111; padding: 8px; overflow-x: auto; white-space: pre-wrap; }}
</style></head><body>
<h1>Brain - camera ({location})</h1>
<div class="row">
<figure><img src="/stream.mjpg" alt="flux live"><figcaption>live (/stream.mjpg)</figcaption></figure>
<figure><img id="ann" src="/annotated.jpg" alt="analiza apare dupa prima rulare">
<figcaption>ultima analiza (/annotated.jpg), reimprospatata la 1 s</figcaption></figure>
</div>
<pre id="st">...</pre>
<script>
setInterval(function () {{
  document.getElementById("ann").src = "/annotated.jpg?t=" + Date.now();
  fetch("/status.json").then(function (r) {{ return r.json(); }}).then(function (s) {{
    document.getElementById("st").textContent = JSON.stringify(s, null, 2);
  }}).catch(function () {{}});
}}, 1000);
</script>
</body></html>
"""


class PreviewHub:
    """Ultimele imagini JPEG și starea, partajate între firele de captură/analiză și serverul HTTP."""

    def __init__(self, location: str) -> None:
        self.location = location
        self._condition = threading.Condition()
        self._raw: Optional[bytes] = None
        self._raw_seq = 0
        self._annotated: Optional[bytes] = None
        self._status: Dict[str, Any] = {"location": location, "frames": 0, "fps": 0.0, "analysis": None}
        self._closed = False
        self._fps_window: List[float] = []

    def publish_raw(self, jpeg: bytes) -> None:
        now = time.monotonic()
        with self._condition:
            self._raw = jpeg
            self._raw_seq += 1
            self._fps_window = [t for t in self._fps_window if now - t < 2.0] + [now]
            self._status["frames"] = self._raw_seq
            self._status["fps"] = round(len(self._fps_window) / 2.0, 1)
            self._condition.notify_all()

    def publish_analysis(self, jpeg: Optional[bytes], summary: Dict[str, Any]) -> None:
        """`jpeg=None` (analiză eșuată) păstrează imaginea anterioară și actualizează doar starea."""
        with self._condition:
            if jpeg is not None:
                self._annotated = jpeg
            self._status["analysis"] = summary

    def wait_raw(self, after_seq: int, timeout: float) -> Tuple[int, Optional[bytes]]:
        """Așteaptă un cadru mai nou decât `after_seq` (sau expiră). Întoarce (secvența, JPEG)."""
        with self._condition:
            self._condition.wait_for(lambda: self._raw_seq > after_seq or self._closed, timeout=timeout)
            return self._raw_seq, self._raw

    def latest(self) -> Tuple[Optional[bytes], Optional[bytes], Dict[str, Any]]:
        with self._condition:
            return self._raw, self._annotated, json.loads(json.dumps(self._status))

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        return self._closed


def _handler(hub: PreviewHub) -> type:
    class Handler(BaseHTTPRequestHandler):
        server_version = "BrainPreview/1"

        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - semnătura din biblioteca standard
            pass  # fără câte o linie în consolă pentru fiecare cadru

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 - numele cerut de BaseHTTPRequestHandler
            path = self.path.split("?", 1)[0]
            raw, annotated, status = hub.latest()
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", _PAGE.format(location=hub.location).encode("utf-8"))
            elif path == "/snapshot.jpg":
                if raw is None:
                    self._send(404, "text/plain; charset=utf-8", "niciun cadru încă".encode("utf-8"))
                else:
                    self._send(200, "image/jpeg", raw)
            elif path == "/annotated.jpg":
                if annotated is None:
                    self._send(404, "text/plain; charset=utf-8", "nicio analiză încă".encode("utf-8"))
                else:
                    self._send(200, "image/jpeg", annotated)
            elif path == "/status.json":
                self._send(200, "application/json; charset=utf-8", json.dumps(status, ensure_ascii=False).encode("utf-8"))
            elif path == "/stream.mjpg":
                self._stream()
            else:
                self._send(404, "text/plain; charset=utf-8", "rută necunoscută".encode("utf-8"))

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary={}".format(BOUNDARY))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            try:
                while not hub.closed:
                    new_seq, jpeg = hub.wait_raw(seq, timeout=1.0)
                    if jpeg is None or new_seq == seq:
                        continue
                    seq = new_seq
                    header = "--{}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n".format(BOUNDARY, len(jpeg))
                    self.wfile.write(header.encode("ascii") + jpeg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # browserul a închis fluxul

    return Handler


class PreviewServer:
    """Serverul HTTP pe un fir separat. `port=0` alege un port liber (teste)."""

    def __init__(self, hub: PreviewHub, host: str, port: int) -> None:
        self.hub = hub
        self._server = ThreadingHTTPServer((host, port), _handler(hub))
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, name="preview-http", daemon=True)

    @property
    def port(self) -> int:
        return int(self._server.server_address[1])

    def start(self) -> "PreviewServer":
        self._thread.start()
        return self

    def stop(self) -> None:
        self.hub.close()
        self._server.shutdown()
        self._server.server_close()


def local_addresses() -> List[str]:
    """Adresele IPv4 ale mașinii (pentru mesajul „deschideți http://<ip>:<port>/”). Nu trimite pachete."""
    addresses = set()
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("10.255.255.255", 1))  # UDP: doar alege interfața, nu trimite nimic
            addresses.add(probe.getsockname()[0])
        finally:
            probe.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addresses.add(str(info[4][0]))
    except OSError:
        pass
    return sorted(a for a in addresses if not a.startswith("127.")) or ["127.0.0.1"]
