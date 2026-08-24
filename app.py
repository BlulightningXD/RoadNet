"""RoadNet Web Dashboard local server.

Run:  python app.py
Open: http://localhost:8080
"""
from __future__ import annotations

import json
import math
import mimetypes
import random
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).parent
STATIC = ROOT / "static"


class TelemetryStore:
    def __init__(self) -> None:
        self.frames: deque[dict] = deque(maxlen=240)
        self.lock = threading.Lock()
        self.mode = "demo"
        self.protocol = "LoRa"
        self.port = "COM5"
        self.status = "Demo stream connected"
        self.running = True
        self._serial = None

    def add(self, frame: dict) -> None:
        with self.lock:
            self.frames.append(frame)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "frames": list(self.frames)[-120:], "mode": self.mode,
                "protocol": self.protocol, "port": self.port, "status": self.status,
            }

    def set_connection(self, data: dict) -> tuple[bool, str]:
        mode = data.get("mode", "demo")
        protocol = data.get("protocol", "LoRa")
        port = str(data.get("port", "COM5")).strip()
        if mode not in {"demo", "serial", "wifi"}:
            return False, "Unknown connection mode."
        self.close_serial()
        with self.lock:
            self.mode, self.protocol, self.port = mode, protocol, port
            if mode == "demo":
                self.status = "Demo stream connected"
            elif mode == "wifi":
                self.status = f"Connecting to ESP32 WiFi at {port}…"
            else:
                self.status = f"Opening {protocol} on {port}…"
        if mode == "serial":
            threading.Thread(target=self._serial_loop, daemon=True).start()
        elif mode == "wifi":
            threading.Thread(target=self._wifi_loop, daemon=True).start()
        return True, self.status

    def close_serial(self) -> None:
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _serial_loop(self) -> None:
        try:
            import serial  # Install with: pip install pyserial
            with self.lock:
                port, protocol = self.port, self.protocol
            self._serial = serial.Serial(port, 115200, timeout=1)
            with self.lock:
                self.status = f"{protocol} connected on {port}"
            while self.running and self.mode == "serial":
                raw = self._serial.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                try:
                    self.add(normalise(json.loads(raw)))
                except (ValueError, TypeError, json.JSONDecodeError):
                    continue
        except Exception as exc:
            with self.lock:
                self.status = f"Serial error: {exc}"
        finally:
            self.close_serial()

    def _wifi_loop(self) -> None:
        import urllib.request
        with self.lock:
            target = self.port if self.port.startswith("http") else f"http://{self.port}/data"
        while self.running and self.mode == "wifi":
            try:
                req = urllib.request.Request(target, headers={"User-Agent": "RoadNetWeb/1.0"})
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    if resp.status == 200:
                        payload = json.loads(resp.read().decode("utf-8"))
                        self.add(normalise(payload))
                        with self.lock:
                            self.status = f"ESP32 WiFi active ({target})"
            except Exception as exc:
                with self.lock:
                    self.status = f"WiFi polling error: {exc}"
            time.sleep(0.2)


STORE = TelemetryStore()


def normalise(raw: dict) -> dict:
    gps = raw.get("gps", {}) if isinstance(raw.get("gps"), dict) else {}
    imu = raw.get("imu", {}) if isinstance(raw.get("imu"), dict) else {}
    mesh = raw.get("mesh", {}) if isinstance(raw.get("mesh"), dict) else {}
    airbag = raw.get("airbag", {}) if isinstance(raw.get("airbag"), dict) else {}
    stats = raw.get("stats", {}) if isinstance(raw.get("stats"), dict) else {}

    # Extract GPS (support nested or flat keys: lat/latitude, lon/lng/longitude, alt/altitude, spd/speed/s, sat)
    lat = float(gps.get("lat", raw.get("lat", raw.get("latitude", 19.076))))
    lon = float(gps.get("lon", gps.get("lng", raw.get("lon", raw.get("lng", raw.get("longitude", 72.877))))))
    alt = float(gps.get("alt", raw.get("alt", raw.get("altitude", 0))))
    spd = float(gps.get("spd", raw.get("spd", raw.get("speed", raw.get("s", 0)))))
    sat = int(gps.get("sat", raw.get("sat", 0)))
    gps_ok = bool(gps.get("ok", raw.get("gps", raw.get("gps_ok", True))))

    # Extract IMU (support nested or flat keys: ax, ay, az, gx, gy, gz, dynamic, gyro)
    # If ax is in m/s^2 (e.g. magnitude > 5 while stationary), convert to g for chart
    raw_ax = float(imu.get("ax", raw.get("ax", 0)))
    raw_ay = float(imu.get("ay", raw.get("ay", 0)))
    raw_az = float(imu.get("az", raw.get("az", 0)))
    
    # Heuristic: if az > 4.0, assume values are in m/s^2 and divide by 9.80665
    if abs(raw_az) > 4.0 or abs(raw_ax) > 4.0:
        ax, ay, az = raw_ax / 9.80665, raw_ay / 9.80665, raw_az / 9.80665
    else:
        ax, ay, az = raw_ax, raw_ay, raw_az

    gx = float(imu.get("gx", raw.get("gx", 0)))
    gy = float(imu.get("gy", raw.get("gy", 0)))
    gz = float(imu.get("gz", raw.get("gz", 0)))
    dynamic = float(imu.get("dynamic", raw.get("dynamic", 0)))
    gyro_mag = float(imu.get("gyro", raw.get("gyro", 0)))

    # Extract Mesh & Nodes
    nodes = int(mesh.get("nodes", raw.get("nodes", raw.get("n", 1))))
    mask = int(mesh.get("mask", raw.get("mask", (1 << nodes) - 1 if nodes > 0 else 1)))
    node_id = int(raw.get("node", raw.get("id", raw.get("nodeId", 1))))

    # Airbag & Local/Remote Accident
    deployed = bool(airbag.get("deployed", raw.get("deployed", False)))
    status = str(airbag.get("status", raw.get("status", "DEPLOYED" if deployed else "ARMED")))

    local_accident = bool(raw.get("local", False))
    remote_accident = bool(raw.get("remote", False))
    remote_node = int(raw.get("remoteNode", raw.get("remote_node", 0)))
    remote_confidence = float(raw.get("remoteConfidence", raw.get("remote_confidence", 0)))

    crash = bool(raw.get("crash", raw.get("accident", raw.get("a", local_accident or remote_accident))))
    cc = float(raw.get("cc", raw.get("confidence", raw.get("c", 1.0 if crash else 0.0))))
    if cc > 1.0:
        cc = cc / 100.0

    return {
        "ts": int(raw.get("ts", raw.get("timestamp", time.time() * 1000))),
        "node": node_id,
        "gps": {"lat": lat, "lon": lon, "alt": alt, "spd": spd, "sat": sat, "ok": gps_ok},
        "imu": {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz, "dynamic": dynamic, "gyro": gyro_mag},
        "mesh": {"nodes": nodes, "mask": mask},
        "airbag": {"deployed": deployed, "status": status},
        "cc": max(0.0, min(1.0, cc)),
        "crash": crash,
        "local": local_accident,
        "remote": remote_accident,
        "remoteNode": remote_node,
        "remoteConfidence": remote_confidence,
        "stats": {
            "transport": str(raw.get("transport", stats.get("transport", "LoRa"))),
            "rx": int(stats.get("rx", raw.get("rx", 0))),
            "tx": int(stats.get("tx", raw.get("tx", 0))),
            "forwarded": int(stats.get("forwarded", raw.get("forwarded", 0))),
            "dropped": int(stats.get("dropped", raw.get("dropped", 0))),
            "rssi": int(stats.get("rssi", raw.get("rssi", 0))),
            "snr": float(stats.get("snr", raw.get("snr", 0))),
            "received": str(stats.get("received", raw.get("received", "None"))),
            "sent": str(stats.get("sent", raw.get("sent", "None"))),
        }
    }


def demo_loop() -> None:
    angle, nodes, counter, next_event = 0.0, 3, 0, time.monotonic() + 10
    while STORE.running:
        if STORE.mode != "demo":
            time.sleep(.2)
            continue
        counter += 100
        angle = (angle + .008) % (math.pi * 2)
        ax, ay, az = random.gauss(0, .04), random.gauss(0, .04), random.gauss(1, .04)
        gx, gy, gz, deployed, crash = random.gauss(0, 1.5), random.gauss(0, 1.5), random.gauss(0, 1.5), False, False
        if time.monotonic() >= next_event:
            next_event = time.monotonic() + 10
            ax, ay, gx, gz, deployed, crash = random.choice([-1, 1]) * random.uniform(4.5, 7), random.choice([-1, 1]) * random.uniform(3, 5), random.choice([-1, 1]) * random.uniform(220, 350), random.choice([-1, 1]) * random.uniform(180, 320), True, True
        if random.random() < .02:
            nodes = max(1, min(8, nodes + random.choice([-1, 1])))
        STORE.add(normalise({"ts": counter, "gps": {"lat": 19.076 + .003 * math.sin(angle), "lon": 72.877 + .003 * math.cos(angle), "alt": 15, "spd": 45 + random.gauss(0, 2), "ok": True}, "imu": {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}, "mesh": {"nodes": nodes, "mask": (1 << nodes) - 1}, "airbag": {"deployed": deployed, "status": "DEPLOYED" if deployed else "ARMED"}, "cc": 1 if crash else min(1, abs(ax) / 5), "crash": crash}))
        time.sleep(.1)


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/telemetry":
            return self.json_response(STORE.snapshot())
        if path in {"/", "/index.html"}:
            return self.send_file(STATIC / "index.html")
        return self.send_file(STATIC / path.lstrip("/"))

    def do_POST(self):
        if urlparse(self.path).path != "/api/connection":
            self.send_error(HTTPStatus.NOT_FOUND); return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            ok, message = STORE.set_connection(json.loads(self.rfile.read(length)))
            self.json_response({"ok": ok, "message": message}, HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self.json_response({"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST)

    def send_file(self, target: Path):
        try:
            target.resolve().relative_to(STATIC.resolve())
            if not target.is_file(): raise FileNotFoundError
            data = target.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
        except (FileNotFoundError, ValueError):
            self.send_error(HTTPStatus.NOT_FOUND)

    def json_response(self, body: dict, status=HTTPStatus.OK):
        data = json.dumps(body).encode()
        self.send_response(status); self.send_header("Content-Type", "application/json"); self.send_header("Cache-Control", "no-store"); self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)

    def log_message(self, *_): pass


if __name__ == "__main__":
    threading.Thread(target=demo_loop, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", 8080), Handler)
    print("RoadNet Web Dashboard: http://localhost:8080")
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: STORE.running = False; STORE.close_serial(); server.server_close()
