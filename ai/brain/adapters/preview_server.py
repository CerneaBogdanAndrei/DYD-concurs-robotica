"""Previzualizarea camerei și pagina de lucru a echipei, prin HTTP (doar biblioteca standard): Pi-ul nu are ecran.

`PreviewHub` ține ultimele imagini (cadrul brut și analiza adnotată), publicate de firul de captură.
Rutele de citire:
- `/`               pagina (fără resurse externe: Pi-ul poate fi offline)
- `/stream.mjpg`    flux MJPEG (multipart/x-mixed-replace) cu cadrele brute
- `/snapshot.jpg`   ultimul cadru brut
- `/annotated.jpg`  ultima analiză adnotată (404 până la prima analiză)
- `/status.json`    starea: locația, fps, rezultatul ultimei analize, plus `workbench` (ținta, serviciul, execuția)

Cu un `api` (masa de lucru din `brain/cli/workbench.py`) există și:
- `GET /target`, `POST /target` (ținta S1 editată, validată strict), `POST /target/image` (poza țintei: corpul =
  octeții imaginii, maximum `MAX_UPLOAD_BYTES`), `GET /target/photo.jpg`, `GET /target/annotated.jpg`;
- `POST /plan` (planul de la starea curentă la țintă; nu mișcă brațul);
- `GET /execute`, `POST /execute`, `POST /execute/token`, `POST /execute/stop` (execuția, doar dacă e permisă).
Erorile au forma `{"error": "mesaj"}`.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

BOUNDARY = "frame"
MAX_UPLOAD_BYTES = 12 * 1024 * 1024  # o poză de telefon are de obicei 2-6 MB
MAX_JSON_BYTES = 64 * 1024
TOKEN_HEADER = "X-Brain-Token"


class ApiError(Exception):
    """Eroare cu status HTTP și mesaj pentru pagină (în română)."""

    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = int(status)
        self.message = message


class PreviewHub:
    """Ultimele imagini JPEG și starea, partajate între firele de captură/analiză și serverul HTTP."""

    def __init__(self, location: str) -> None:
        self.location = location
        self._condition = threading.Condition()
        self._raw: Optional[bytes] = None
        self._raw_seq = 0
        self._annotated: Optional[bytes] = None
        self._status: Dict[str, Any] = {"location": location, "frames": 0, "fps": 0.0, "analysis": None, "info": {}}
        self._observation: Any = None
        self._analysis_at: Optional[float] = None
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

    def publish_analysis(self, jpeg: Optional[bytes], summary: Dict[str, Any], observation: Any = None) -> None:
        """`jpeg=None` (analiză eșuată) păstrează imaginea anterioară și actualizează doar starea.
        `observation` (GridObservation) rămâne în memorie pentru planificare; nu apare în `/status.json`."""
        with self._condition:
            if jpeg is not None:
                self._annotated = jpeg
            self._status["analysis"] = summary
            self._status["location"] = self.location
            self._analysis_at = time.monotonic()
            if observation is not None or "error" in summary:
                self._observation = observation

    def set_info(self, **info: Any) -> None:
        """Date despre sursă (rezoluție, cameră, modul analizei), afișate în panoul „Live”."""
        with self._condition:
            self._status["info"].update(info)

    def latest_observation(self) -> Tuple[Any, Optional[float]]:
        """Ultima observație (GridObservation sau None) și vechimea ei în secunde."""
        with self._condition:
            age = None if self._analysis_at is None else time.monotonic() - self._analysis_at
            return self._observation, age

    def wait_raw(self, after_seq: int, timeout: float) -> Tuple[int, Optional[bytes]]:
        """Așteaptă un cadru mai nou decât `after_seq` (sau expiră). Întoarce (secvența, JPEG)."""
        with self._condition:
            self._condition.wait_for(lambda: self._raw_seq > after_seq or self._closed, timeout=timeout)
            return self._raw_seq, self._raw

    def latest(self) -> Tuple[Optional[bytes], Optional[bytes], Dict[str, Any]]:
        with self._condition:
            status = json.loads(json.dumps(self._status))
            status["analysis_age_s"] = None if self._analysis_at is None else round(time.monotonic() - self._analysis_at, 1)
            return self._raw, self._annotated, status

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        return self._closed


def _handler(hub: PreviewHub, api: Any, page: bytes) -> type:
    class Handler(BaseHTTPRequestHandler):
        server_version = "BrainPreview/2"

        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - semnătura din biblioteca standard
            pass  # fără câte o linie în consolă pentru fiecare cadru

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, payload: Any) -> None:
            self._send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False).encode("utf-8"))

        def _error(self, status: int, message: str) -> None:
            self._json(status, {"error": message})

        def _image(self, data: Optional[bytes], missing: str) -> None:
            if data is None:
                self._send(404, "text/plain; charset=utf-8", missing.encode("utf-8"))
            else:
                self._send(200, "image/jpeg", data)

        def do_GET(self) -> None:  # noqa: N802 - numele cerut de BaseHTTPRequestHandler
            path = self.path.split("?", 1)[0]
            try:
                self._get(path)
            except ApiError as exc:
                self._error(exc.status, exc.message)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def _get(self, path: str) -> None:
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", page)
                return
            if path == "/stream.mjpg":
                self._stream()
                return
            raw, annotated, status = hub.latest()
            if path == "/snapshot.jpg":
                self._image(raw, "niciun cadru încă")
            elif path == "/annotated.jpg":
                self._image(annotated, "nicio analiză încă")
            elif path == "/status.json":
                if api is not None:
                    status["workbench"] = api.status()
                self._json(200, status)
            elif api is not None and path == "/target":
                self._json(200, api.get_target())
            elif api is not None and path in ("/target/photo.jpg", "/target/annotated.jpg"):
                self._image(api.target_image(path.rsplit("/", 1)[1].split(".")[0]), "nicio poză încărcată")
            elif api is not None and path == "/execute":
                self._json(200, api.execution_status())
            else:
                self._send(404, "text/plain; charset=utf-8", "rută necunoscută".encode("utf-8"))

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0]
            try:
                if api is None:
                    raise ApiError(404, "rută necunoscută")
                self._post(path)
            except ApiError as exc:
                self._error(exc.status, exc.message)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            except Exception as exc:  # serverul nu cade la o cerere greșită
                self._error(500, "eroare internă: {}: {}".format(type(exc).__name__, exc))

        def _length(self, limit: int, required: bool = True) -> int:
            text = self.headers.get("Content-Length")
            if text is None:
                if not required:  # un POST fără corp (ex. curl -X POST): nimic de citit
                    return 0
                self.close_connection = True
                raise ApiError(411, "lipsește Content-Length")
            try:
                length = int(text)
            except ValueError:
                raise ApiError(400, "Content-Length invalid") from None
            if length < 0:
                raise ApiError(400, "Content-Length invalid")
            if length > limit:
                self.close_connection = True  # corpul nu se citește: conexiunea nu mai poate fi refolosită
                raise ApiError(413, "fișier prea mare: {:.1f} MB (maximum {:.0f} MB)".format(
                    length / 1048576.0, limit / 1048576.0))
            return length

        def _read(self, limit: int, required: bool = True) -> bytes:
            length = self._length(limit, required)
            return self.rfile.read(length) if length else b""

        def _read_json(self) -> Any:
            data = self._read(MAX_JSON_BYTES)
            if not data:
                return {}
            try:
                return json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                raise ApiError(400, "corpul nu e JSON valid") from None

        def _post(self, path: str) -> None:
            token = self.headers.get(TOKEN_HEADER)
            if path == "/target/image":
                content_type = (self.headers.get("Content-Type") or "").split(";", 1)[0].strip().lower()
                if not (content_type.startswith("image/") or content_type == "application/octet-stream"):
                    self.close_connection = True  # corpul nu se citește
                    raise ApiError(415, "trimiteți o imagine (Content-Type image/*), nu {}".format(content_type or "nimic"))
                self._json(200, api.upload_target_image(self._read(MAX_UPLOAD_BYTES)))
            elif path == "/target":
                self._json(200, api.set_target(self._read_json()))
            elif path == "/plan":
                self._read(MAX_JSON_BYTES, required=False)
                self._json(200, api.compute_plan())
            elif path == "/execute":
                self._read(MAX_JSON_BYTES, required=False)
                self._json(200, api.start_execution(token))
            elif path == "/execute/token":
                self._read(MAX_JSON_BYTES, required=False)
                self._json(200, api.check_token(token))
            elif path == "/execute/stop":
                self._read(MAX_JSON_BYTES, required=False)
                self._json(200, api.stop_execution())
            else:
                raise ApiError(404, "rută necunoscută: {}".format(path))

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
    """Serverul HTTP pe un fir separat. `port=0` alege un port liber (teste). `api` = masa de lucru (opțională)."""

    def __init__(self, hub: PreviewHub, host: str, port: int, api: Any = None) -> None:
        from brain.adapters.preview_page import render_page

        self.hub = hub
        page = render_page(hub.location, workbench=api is not None,
                           execute=bool(api is not None and getattr(api, "execute_enabled", False)))
        self._server = ThreadingHTTPServer((host, port), _handler(hub, api, page.encode("utf-8")))
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
