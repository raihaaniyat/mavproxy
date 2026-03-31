"""
================================================================================
 COBRA VTOL — SITL Telemetry Bridge
 Connects to Mission Planner's TCP mirror port 5763, reads live MAVLink
 packets, and re-serves them as JSON at http://127.0.0.1:56781/mavlink/
 so master_station.py can poll it as a normal HTTP telemetry source.
================================================================================
"""

import threading
import time
import json
import logging
import http.server
import socketserver
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("FATAL: pymavlink not installed. Run: pip install pymavlink")
    exit(1)

# ── Configuration ─────────────────────────────────────────────────────────────
SITL_CONN = "tcp:127.0.0.1:5763"  # Mission Planner native mirror port (telemetry)
HTTP_PORT = 56781

# Shared cache — MAVLink packets stored as dicts for JSON serialisation
latest_telemetry: dict = {
    "GPS_RAW_INT":          {"msg": {}},
    "VFR_HUD":              {"msg": {}},
    "GLOBAL_POSITION_INT":  {"msg": {}},
    "HEARTBEAT":            {"msg": {}},
    "ATTITUDE":             {"msg": {}},
}

_lock = threading.Lock()

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("SITL_BRIDGE")


class MAVLinkBridgeHandler(http.server.SimpleHTTPRequestHandler):
    """Serve the cached SITL telemetry as JSON on GET /mavlink/"""

    def do_GET(self):
        if self.path == "/mavlink/":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with _lock:
                payload = json.dumps(latest_telemetry)
            self.wfile.write(payload.encode("utf-8"))
        else:
            self.send_error(404, "Use /mavlink/")

    def log_message(self, format, *args):
        pass  # silence noisy access logs


def sitl_listener(connection_string: str):
    """Background thread — reads live MAVLink from SITL and updates cache."""
    while True:
        try:
            log.info(f"Connecting to SITL on {connection_string}...")
            # source_system=253 avoids collision with Mission Planner (255) and master_station (254)
            vehicle = mavutil.mavlink_connection(connection_string, source_system=253)

            log.info("Waiting for heartbeat...")
            vehicle.wait_heartbeat()
            log.info(
                f"Heartbeat OK — target_system={vehicle.target_system} "
                f"target_component={vehicle.target_component}"
            )

            WANTED = set(latest_telemetry.keys())

            while True:
                msg = vehicle.recv_match(blocking=True, timeout=2.0)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type in WANTED:
                    with _lock:
                        latest_telemetry[msg_type]["msg"] = msg.to_dict()

        except Exception as exc:
            log.error(f"SITL connection lost: {exc} — reconnecting in 3s...")
            time.sleep(3)


def run_bridge():
    # Start MAVLink listener thread
    t = threading.Thread(target=sitl_listener, args=(SITL_CONN,), daemon=True)
    t.start()

    # Start HTTP server
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", HTTP_PORT), MAVLinkBridgeHandler) as httpd:
        log.info(f"SITL HTTP Bridge running → http://127.0.0.1:{HTTP_PORT}/mavlink/")
        log.info("Press Ctrl+C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            log.info("Bridge stopping...")
            httpd.server_close()


if __name__ == "__main__":
    run_bridge()
