"""Serve live MJPEG streams from every connected camera over HTTP.

View from another machine on the network by opening http://<this-host-ip>:8000/ in a browser.
"""

import glob
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PORT = 8000
BOUNDARY = "frame"


def find_working_cameras() -> list[str]:
    working = []
    for path in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                working.append(path)
        cap.release()
    return working


class CameraStream:
    def __init__(self, device_path: str) -> None:
        self.device_path = device_path
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        self.lock = threading.Lock()
        self.jpeg = b""
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self) -> None:
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                ok2, buf = cv2.imencode(".jpg", frame)
                if ok2:
                    with self.lock:
                        self.jpeg = buf.tobytes()
            time.sleep(0.03)

    def get_jpeg(self) -> bytes:
        with self.lock:
            return self.jpeg

    def stop(self) -> None:
        self.running = False
        self.thread.join(timeout=1)
        self.cap.release()


streams: dict[str, CameraStream] = {}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/":
            self._serve_index()
        elif self.path.startswith("/feed/"):
            name = self.path.removeprefix("/feed/")
            if name in streams:
                self._serve_feed(streams[name])
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def _serve_index(self) -> None:
        imgs = "".join(
            f'<div><h3>{name}</h3><img src="/feed/{name}" width="480"></div>' for name in streams
        )
        html = f"""<html><head><title>YAM camera test</title></head>
<body style="font-family: sans-serif; background:#111; color:#eee;">
<h2>Live camera feeds ({len(streams)} found)</h2>
<div style="display:flex; flex-wrap:wrap; gap:20px;">{imgs}</div>
</body></html>"""
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_feed(self, stream: CameraStream) -> None:
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                jpeg = stream.get_jpeg()
                if jpeg:
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main() -> None:
    devices = find_working_cameras()
    print(f"found {len(devices)} working camera(s): {devices}")
    for path in devices:
        name = path.replace("/dev/", "")
        streams[name] = CameraStream(path)

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"serving on http://0.0.0.0:{PORT}/  (open http://<this-machine-ip>:{PORT}/ from another computer)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for s in streams.values():
            s.stop()


if __name__ == "__main__":
    main()
