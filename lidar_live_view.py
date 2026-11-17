#!/usr/bin/env python3
"""Live browser view and optional timestamped log for a RPLIDAR S3."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import json
import signal
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import serial


DEFAULT_PORT = "/dev/cu.usbserial-1230"
BAUDRATE = 1_000_000
HTML = Path(__file__).with_name("lidar_live_view.html")


def response(serial_port: serial.Serial, command: int, expected_type: int, size: int) -> bytes:
    serial_port.reset_input_buffer()
    serial_port.write(bytes((0xA5, command)))
    header = serial_port.read(7)
    if len(header) != 7 or header[:2] != b"\xa5\x5a":
        raise RuntimeError(f"LiDAR did not answer command 0x{command:02x}")
    length = int.from_bytes(header[2:6], "little") & 0x3FFFFFFF
    if header[6] != expected_type or length != size:
        raise RuntimeError(f"Unexpected LiDAR response: {header.hex(' ')}")
    payload = serial_port.read(size)
    if len(payload) != size:
        raise RuntimeError("Incomplete LiDAR response")
    return payload


def scan_header(serial_port: serial.Serial) -> None:
    serial_port.reset_input_buffer()
    serial_port.write(b"\xa5\x20")  # Standard SCAN command.
    header = serial_port.read(7)
    if header != b"\xa5\x5a\x05\x00\x00\x40\x81":
        raise RuntimeError(f"Unexpected scan header: {header.hex(' ')}")


def decode_node(node: bytes) -> tuple[bool, float, float, int]:
    """Decode a five-byte standard SCAN measurement."""
    if len(node) != 5 or ((node[0] ^ (node[0] >> 1)) & 1) != 1 or not node[1] & 1:
        raise ValueError("Invalid scan node")
    start = bool(node[0] & 1)
    angle = (int.from_bytes(node[1:3], "little") >> 1) / 64.0
    distance = int.from_bytes(node[3:5], "little") / 4.0
    if angle >= 360:
        raise ValueError("Invalid scan angle")
    return start, angle, distance, node[0] >> 2


class ScanState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.model = ""
        self.firmware = ""
        self.health = "starting"
        self.sequence = 0
        self.scanned_at = 0.0
        self.points: list[list[float | int]] = []
        self.samples = 0
        self.error = ""

    def publish(self, nodes: list[tuple[float, float, int]]) -> None:
        if len(nodes) < 200:
            return
        points = [[round(a, 3), round(d, 2), q] for a, d, q in nodes if d > 0]
        with self.lock:
            self.sequence += 1
            self.scanned_at = time.time()
            self.points = points
            self.samples = len(nodes)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "sequence": self.sequence,
                "scanned_at": self.scanned_at,
                "model": self.model,
                "firmware": self.firmware,
                "health": self.health,
                "samples": self.samples,
                "valid": len(self.points),
                "points": self.points,
                "error": self.error,
            }


def make_handler(state: ScanState, html: bytes) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            pass

        def send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self.send_bytes(html, "text/html; charset=utf-8")
            elif path == "/api/scan":
                body = json.dumps(state.snapshot(), separators=(",", ":")).encode()
                self.send_bytes(body, "application/json")
            else:
                self.send_error(404)

        def do_POST(self) -> None:
            if urlsplit(self.path).path != "/api/stop":
                self.send_error(404)
                return
            state.stop.set()
            self.send_bytes(b'{"stopping":true}', "application/json")

    return Handler


def run(port: str, http_port: int, open_browser: bool,
        record_path: Path | None = None) -> None:
    state = ScanState()
    html = HTML.read_bytes()
    server: ThreadingHTTPServer | None = None
    serial_port: serial.Serial | None = None
    scan_started = False
    record_file = None

    def request_stop(_signum: int, _frame: object) -> None:
        state.stop.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        # Set modem control lines before opening the device. The S3 motor is
        # started by SCAN and stopped by STOP, not by either line here.
        serial_port = serial.Serial(port=None, baudrate=BAUDRATE, timeout=1, write_timeout=1)
        serial_port.dtr = False
        serial_port.rts = False
        serial_port.port = port
        serial_port.open()

        info = response(serial_port, 0x50, 0x04, 20)
        if info[0] != 0x81:
            raise RuntimeError(f"Expected RPLIDAR S3M1 (0x81), found model 0x{info[0]:02x}")
        health = response(serial_port, 0x52, 0x06, 3)
        if health[0] != 0:
            raise RuntimeError(f"LiDAR health error: status={health[0]}, code={int.from_bytes(health[1:], 'little')}")
        state.model = "RPLIDAR S3M1"
        state.firmware = f"{info[2]}.{info[1]}"
        state.health = "OK"

        if record_path is not None:
            record_path = record_path.expanduser().resolve()
            record_path.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation protects a previous run's raw scans.
            record_file = gzip.open(record_path, "xt", encoding="utf-8")
            print(f"Recording scans: {record_path}", flush=True)

        server = ThreadingHTTPServer(("127.0.0.1", http_port), make_handler(state, html))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"{state.model} firmware {state.firmware}; live view: {url}", flush=True)
        print("Press Ctrl-C in this terminal or Stop in the browser to stop scanning.", flush=True)
        if open_browser:
            webbrowser.open(url)

        scan_started = True
        scan_header(serial_port)
        pending = bytearray()
        current: list[tuple[float, float, int]] = []
        synchronized = False
        last_data = time.monotonic()
        while not state.stop.is_set():
            chunk = serial_port.read(4096)
            if chunk:
                last_data = time.monotonic()
                pending.extend(chunk)
                complete = len(pending) // 5 * 5
                for offset in range(0, complete, 5):
                    try:
                        start, angle, distance, quality = decode_node(pending[offset:offset + 5])
                    except ValueError as exc:
                        raise RuntimeError("Corrupt scan stream; stopping LiDAR") from exc
                    if start:
                        if synchronized:
                            state.publish(current)
                            if record_file is not None and len(current) >= 200:
                                record_file.write(json.dumps({
                                    "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                                    "monotonic_s": time.monotonic(),
                                    "samples": len(current),
                                    "points": current,
                                }, separators=(",", ":")) + "\n")
                                if state.sequence % 10 == 0:
                                    record_file.flush()
                        current = []
                        synchronized = True
                    if synchronized:
                        current.append((angle, distance, quality))
                        if len(current) > 10_000:
                            raise RuntimeError("Scan revolution exceeded expected size")
                del pending[:complete]
            elif time.monotonic() - last_data > 5:
                raise RuntimeError("No scan data for five seconds")
    finally:
        if serial_port and serial_port.is_open:
            if scan_started:
                try:
                    serial_port.write(b"\xa5\x25")  # STOP also stops S3 rotation.
                    serial_port.flush()
                    print("LiDAR STOP sent.", flush=True)
                except serial.SerialException as exc:
                    print(f"Could not send LiDAR STOP: {exc}", flush=True)
            serial_port.close()
        if server:
            server.shutdown()
            server.server_close()
        if record_file is not None:
            record_file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="View live RPLIDAR S3 scans in a local browser")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"USB serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--http-port", type=int, default=0, help="local UI port (0 chooses a free port)")
    parser.add_argument("--no-browser", action="store_true", help="print the UI URL without opening it")
    parser.add_argument("--record", type=Path, metavar="PATH.jsonl.gz",
                        help="save each scan with Pi monotonic and UTC timestamps for later odometry alignment")
    args = parser.parse_args()
    try:
        run(args.port, args.http_port, not args.no_browser, args.record)
    except (OSError, serial.SerialException, RuntimeError) as exc:
        parser.exit(1, f"LiDAR error: {exc}\n")


if __name__ == "__main__":
    main()
