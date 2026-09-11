"""Local browser dashboard; avoids Tk and CustomTkinter on macOS."""
from __future__ import annotations
import json
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import TYPE_CHECKING
from urllib.parse import urlparse

import cv2
import numpy as np
from shared.config import AppConfig, SafetyZone
if TYPE_CHECKING:
    from main import PerceptionLoop

PAGE = """<!doctype html><meta charset=utf-8><title>Pedestrian Detection Vehicle</title><style>
body{margin:0;background:#17171c;color:#eee;font:16px -apple-system,sans-serif}header{padding:20px 28px;background:#202027}h1{font-size:22px;margin:0 0 5px}.muted{color:#aaa}main{display:grid;grid-template-columns:2fr 1fr;gap:18px;padding:20px}.panel{background:#282830;border-radius:14px;padding:18px}img{width:100%;background:#1b1b21;border-radius:8px}.zone{padding:15px;border-radius:9px;font-size:26px;font-weight:bold;margin:13px 0}.metric{margin:16px 0}.label{color:#aaa;font-size:13px}.value{font-size:22px;font-weight:650}button{background:#555565;color:#fff;border:0;border-radius:7px;padding:10px 13px}@media(max-width:800px){main{grid-template-columns:1fr}}</style>
<header><h1>Pedestrian Detection Vehicle</h1><div class=muted id=status>Starting…</div></header><main><section class=panel><img id=feed></section><aside class=panel><h2>Safety Dashboard</h2><div id=zone class=zone>—</div><div class=metric><div class=label>Distance</div><div id=distance class=value>—</div></div><div class=metric><div class=label>Confidence</div><div id=confidence class=value>—</div></div><div class=metric><div class=label>Frame rate</div><div id=fps class=value>—</div></div><div class=metric><div class=label>ESP32 policy (simulated)</div><div id=esp32 class=value>—</div></div><div class=metric><div class=label>Motor speed</div><div id=speed class=value>—</div></div><div id=buzzer class=muted>Buzzer: off</div><p><button onclick="fetch('/stop',{method:'POST'})">Stop simulator</button></p></aside></main><script>
const $=x=>document.getElementById(x);async function u(){try{let s=await(await fetch('/status',{cache:'no-store'})).json();for(let x of ['status','zone','distance','confidence','fps','esp32','speed','buzzer'])$(x).textContent=s[x];$('zone').style.background=s.zone_color;$('feed').src='/frame.jpg?t='+Date.now()}catch(e){$('status').textContent='Dashboard disconnected'}}setInterval(u,250);u()</script>"""

class DashboardApp:
    def __init__(self, loop: "PerceptionLoop", config: AppConfig) -> None:
        self.loop, self.config, self._running, self._error = loop, config, False, None
        self._server = None
        self._thread = None
        self.url = ""

    def show(self) -> None:
        app = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/": self.reply(200, "text/html", PAGE.encode())
                elif path == "/status": self.reply(200, "application/json", json.dumps(app._status()).encode())
                elif path == "/frame.jpg": self.reply(200, "image/jpeg", app._jpeg())
                else: self.send_error(404)
            def do_POST(self):
                if urlparse(self.path).path == "/stop":
                    app._running = False; self.reply(204, "text/plain", b"")
                else: self.send_error(404)
            def reply(self, code, kind, data):
                self.send_response(code); self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(data))); self.send_header("Cache-Control", "no-store"); self.end_headers()
                if data: self.wfile.write(data)
            def log_message(self, format, *args): pass
        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self._server.server_port}"
        self._running = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"Dashboard available at {self.url}")
        webbrowser.open(self.url)

    def pump(self) -> None: pass
    def show_error(self, message: str) -> None: self._error = message
    def run(self) -> None:
        while self._running: time.sleep(.1)
    def close(self) -> None:
        self._running = False
        if self._server: self._server.shutdown(); self._server.server_close()
        if self._thread: self._thread.join(timeout=1)

    def _status(self) -> dict:
        _, s, loop_error = self.loop.get_frame()
        error = self._error or loop_error
        zone = "ERROR" if error else s.get("zone", "—")
        colors = {SafetyZone.FAR.value:"#1a563a", SafetyZone.CAUTION.value:"#5c421e", SafetyZone.DANGER.value:"#611f25", "ERROR":"#611f25"}
        dist, confidence = s.get("distance_m"), s.get("confidence", 0)
        return {"status":f"Error: {error}" if error else s.get("status_text", "Starting…"), "zone":zone, "zone_color":colors.get(zone, "#3b3b45"), "distance":f"{dist:.2f} m" if dist is not None else "No target", "confidence":f"{confidence:.0%}" if confidence else "—", "fps":f"{s.get('fps', 0):.1f} FPS", "esp32":error or s.get("esp32", "Waiting for heartbeat…"), "speed":f"{s.get('speed_factor', 0):.0%}", "buzzer":"Buzzer: ON" if s.get("buzzer_on") else "Buzzer: off"}

    def _jpeg(self) -> bytes:
        frame, _, _ = self.loop.get_frame()
        if frame is None:
            frame = np.full((540, 720, 3), (32, 32, 38), dtype=np.uint8)
            cv2.putText(frame, "Waiting for video…", (42, 90), cv2.FONT_HERSHEY_SIMPLEX, .8, (180,180,180), 2)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return encoded.tobytes() if ok else b""
