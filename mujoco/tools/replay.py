"""./b replay <recording> — watch a recorded match (sim/soccer --record <file>) in the browser viewer.

Serves the viewer of module::WebViewer together with the recording; no simulator is needed.
"""

import functools
import http.server
import os
import shutil
import webbrowser

import b


def register(command):
    command.description = "Replay a recorded match in the browser viewer"
    command.add_argument("recording", help="file written by sim/soccer --record")
    command.add_argument("--port", type=int, default=8080, help="port to serve the viewer on (default 8080)")
    command.add_argument("--no-browser", action="store_true", help="do not open the browser")


class _Handler(http.server.SimpleHTTPRequestHandler):
    recording = ""

    def do_GET(self):
        if self.path.split("?")[0] != "/recording":
            return super().do_GET()
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(os.path.getsize(self.recording)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with open(self.recording, "rb") as file:
            shutil.copyfileobj(file, self.wfile)

    def log_message(self, *args):
        pass  # one line per served file is noise here


def run(recording, port, no_browser, **kwargs):
    recording = os.path.abspath(recording)
    with open(recording, "rb") as file:
        if file.read(8) != b"RCSSMJR1":
            raise SystemExit(f"{recording} is not a NUSim recording")

    _Handler.recording = recording
    web_root = os.path.join(b.mujoco_dir, "module", "WebViewer", "web")
    handler = functools.partial(_Handler, directory=web_root)
    url = f"http://127.0.0.1:{port}/?replay=/recording"
    with http.server.ThreadingHTTPServer(("127.0.0.1", port), handler) as server:
        print(f"Replay at {url}  (Ctrl+C to stop)", flush=True)
        if not no_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
