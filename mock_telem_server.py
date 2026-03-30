"""
================================================================================
 COBRA VTOL — Mock MAVLink Telemetry Server  (Windows-safe)
 Serves dynamic, ArduPilot-style JSON MAVLink data at /mavlink/ for local
 testing.  Run this in a separate terminal before starting master_station.py.
================================================================================
"""

import http.server
import socketserver
import json
import random
import time
import logging
from pathlib import Path
from datetime import datetime

# ── Paths (Windows-safe, pathlib) ─────────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR  = BASE_DIR / "system_logs"


def setup_logger(module_name: str) -> logging.Logger:
    """Dual-output logger: DEBUG→file, INFO→console."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    log_file = LOG_DIR / f"vtol_{module_name}_{datetime.now().strftime('%Y-%m-%d')}.log"

    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)

    if not logger.handlers:
        fh = logging.FileHandler(str(log_file), encoding="utf-8")
        fh.setFormatter(logging.Formatter(
            "%(asctime)s - [%(levelname)s] - <Line: %(lineno)d> - %(message)s"
        ))

        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))

        logger.addHandler(fh)
        logger.addHandler(ch)

    return logger


log = setup_logger("MOCK_TELEMETRY")


class MAVLinkMockHandler(http.server.SimpleHTTPRequestHandler):
    """Respond to GET /mavlink/ with simulated ArduPilot telemetry JSON."""

    def do_GET(self):
        if self.path == "/mavlink/":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()

            # Base coordinates: central India (Gwalior area)
            base_lat = 262183000
            base_lon = 781828000

            jitter_lat = random.randint(-50, 50)
            jitter_lon = random.randint(-50, 50)

            # ~50 m hover with slight drift (millimetres)
            rel_alt_mm = 50000 + random.randint(-500, 500)

            # Heading in centidegrees (≈90°)
            heading = 9000 + random.randint(-200, 200)

            payload = {
                "VFR_HUD": {
                    "msg": {
                        "airspeed": 0.0,
                        "groundspeed": 0.0,
                        "alt": rel_alt_mm / 1000.0,
                        "climb": random.uniform(-0.1, 0.1),
                        "heading": int(heading / 100.0),
                        "throttle": 0
                    },
                    "index": 1,
                    "time_usec": 0
                },
                "STATUSTEXT": {"msg": None, "index": 0, "time_usec": 0},
                "SYS_STATUS": {
                    "msg": {"master_in": 178, "mav_loss": 0, "mavpackettype": "META_LINKQUALITY", "master_out": 34, "packet_loss": 19.0},
                    "index": 0,
                    "time_usec": 0
                },
                "ATTITUDE": {
                    "msg": {
                        "time_boot_ms": int(time.time() * 1000) % 10_000_000,
                        "roll": round(random.uniform(-0.05, 0.05), 4),
                        "pitch": round(random.uniform(-0.02, 0.02), 4),
                        "yaw": round(heading / 10000.0, 4),
                        "rollspeed": 0.0005,
                        "pitchspeed": 0.0013,
                        "yawspeed": -0.0014
                    },
                    "index": 1,
                    "time_usec": 0
                },
                "GPS_RAW_INT": {
                    "msg": {
                        "time_usec": 0,
                        "lat": base_lat + jitter_lat,
                        "lon": base_lon + jitter_lon,
                        "alt": rel_alt_mm + 150_000,
                        "eph": 9999, "epv": 9999, "vel": 0, "cog": 0,
                        "fix_type": 3, "satellites_visible": 10,
                        "alt_ellipsoid": 0, "h_acc": 4294967295,
                        "v_acc": 3751762688, "vel_acc": 999000, "hdg_acc": 0, "yaw": 0
                    },
                    "index": 1,
                    "time_usec": 0
                },
                "HEARTBEAT": {
                    "msg": {
                        "custom_mode": 4, "type": 2, "autopilot": 3,
                        "base_mode": 81, "system_status": 3, "mavlink_version": 3
                    },
                    "index": 1,
                    "time_usec": 0
                },
                "GPS_STATUS": {"msg": None, "index": 0, "time_usec": 0},
                "NAV_CONTROLLER_OUTPUT": {
                    "msg": {
                        "nav_roll": 0.0, "nav_pitch": 0.0, "alt_error": 0.0,
                        "aspd_error": 0.0, "xtrack_error": 0.0, "nav_bearing": -150,
                        "target_bearing": 0, "wp_dist": 0
                    },
                    "index": 1,
                    "time_usec": 0
                },
                "META_LINKQUALITY": {
                    "msg": {"master_in": 178, "mav_loss": 0, "mavpackettype": "META_LINKQUALITY", "master_out": 34, "packet_loss": 19.0},
                    "index": 0,
                    "time_usec": 0
                }
            }

            self.wfile.write(json.dumps(payload, indent=4).encode("utf-8"))
            log.debug(f"Served mock MAVLink data to {self.client_address[0]}")
        else:
            self.send_error(404, "Endpoint not found. Use /mavlink/")
            log.warning(
                f"404 for {self.path} from {self.client_address[0]}"
            )

    def log_message(self, fmt, *args):
        """Route built-in http.server logs through our custom logger."""
        log.debug(f"HTTP Server: {fmt % args}")


def run_mock_server(port: int = 56781) -> None:
    socketserver.TCPServer.allow_reuse_address = True

    with socketserver.TCPServer(("127.0.0.1", port), MAVLinkMockHandler) as httpd:
        log.info(f"Mock MAVLink Telemetry Server → http://127.0.0.1:{port}/mavlink/")
        log.info("Press Ctrl+C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            log.info("Server stopped by operator.")
            httpd.server_close()


if __name__ == "__main__":
    run_mock_server()
