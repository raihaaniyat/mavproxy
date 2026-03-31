"""
================================================================================
 COBRA VTOL — HYBRID GROUND CONTROL STATION (Windows Edition)
 Architecture : Split-Bridge (HTTP Telemetry IN → Local AI → MAVLink CMD OUT)
 Platform     : Windows 10/11  |  Python 3.10+
================================================================================
 CRITICAL CONSTRAINTS (DO NOT ALTER):
   1. MAVLink handle is  self.drone  — NEVER  self.master  (Tkinter collision)
   2. GSD altitude guard: alt = max(..., 1.0)  prevents /0 at ground spawn
   3. Display via ctk.CTkImage — prevents High-DPI blur on Windows
================================================================================
"""

import cv2
import math
import json
import time
import logging
import threading
import webbrowser
import requests
from pathlib import Path
from datetime import datetime

import customtkinter as ctk
from PIL import Image

# ── Optional heavy deps (graceful degradation) ───────────────────────────────
try:
    from pymavlink import mavutil
    PYMAVLINK_OK = True
except ImportError:
    PYMAVLINK_OK = False

try:
    from ultralytics import YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False

try:
    import folium
    FOLIUM_OK = True
except ImportError:
    FOLIUM_OK = False


# ==============================================================================
#  CONFIGURATION
# ==============================================================================

BASE_DIR        = Path(__file__).resolve().parent
LOG_DIR         = BASE_DIR / "system_logs"
DB_FILE         = BASE_DIR / "detected_targets.json"
MAP_OUTPUT      = BASE_DIR / "tactical_map.html"

# ── Network endpoints ────────────────────────────────────────────────────────
TELEMETRY_URL   = "http://127.0.0.1:56781/mavlink/"   # HTTP telemetry source
MAVLINK_CONN    = "tcp:127.0.0.1:5762"  # Mission Planner native mirror port (commands)
# For a real drone: MAVLINK_CONN = "COM3"

# ── Camera / FOV ──────────────────────────────────────────────────────────────
IMG_WIDTH, IMG_HEIGHT = 1920, 1080
FOV_H_DEG, FOV_V_DEG = 60.0, 45.0

# ── Timing ────────────────────────────────────────────────────────────────────
TELEM_POLL_MS   = 200
VISION_LOOP_MS  = 30
HTTP_TIMEOUT_S  = 0.2

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ==============================================================================
#  LOGGING
# ==============================================================================

def setup_logger() -> logging.Logger:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp    = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file = LOG_DIR / f"vtol_HYBRID_{stamp}.log"

    logger = logging.getLogger("GROUND_STATION")
    logger.setLevel(logging.DEBUG)

    if not logger.handlers:
        fh = logging.FileHandler(str(log_file), encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            "%(asctime)s - [%(levelname)s] - <Line: %(lineno)d> - %(message)s"
        ))
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
        logger.addHandler(fh)
        logger.addHandler(ch)

    return logger


log = setup_logger()


# ==============================================================================
#  WINDOWS CAMERA DISCOVERY  (DirectShow)
# ==============================================================================

def find_obs_camera(max_index: int = 10) -> "cv2.VideoCapture | None":
    log.info("Scanning for DirectShow cameras (OBS Virtual Camera)...")
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                log.info(f"  ✓ Camera found at index {idx}")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                return cap
            cap.release()
    log.warning("No DirectShow camera found on indices 0-%d.", max_index - 1)
    return None


# ==============================================================================
#  TACTICAL MAP GENERATION  (Folium)
# ==============================================================================

def generate_tactical_map() -> "Path | None":
    if not FOLIUM_OK:
        log.error("folium not installed — cannot generate map.")
        return None
    if not DB_FILE.exists():
        log.error(f"Database '{DB_FILE.name}' not found.")
        return None
    try:
        with open(DB_FILE, "r", encoding="utf-8") as f:
            targets = json.load(f)
    except (json.JSONDecodeError, OSError) as exc:
        log.error(f"Failed to read target DB: {exc}")
        return None
    if not targets:
        log.warning("Target database is empty.")
        return None

    avg_lat = sum(t["gps_lat"] for t in targets) / len(targets)
    avg_lon = sum(t["gps_lon"] for t in targets) / len(targets)
    tac_map = folium.Map(location=[avg_lat, avg_lon], zoom_start=19)
    folium.TileLayer(
        tiles=(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
        attr="Esri", name="Esri Satellite", overlay=False, control=True,
    ).add_to(tac_map)

    def _color(sev):
        return "green" if sev <= 3 else ("orange" if sev <= 7 else "red")

    for t in targets:
        folium.CircleMarker(
            location=[t["gps_lat"], t["gps_lon"]], radius=8,
            popup=folium.Popup(
                f"<b>{t.get('id','?')}</b><br>Severity: {t.get('severity_index',1)}<br>"
                f"Lat: {t['gps_lat']:.6f}<br>Lon: {t['gps_lon']:.6f}", max_width=200
            ),
            color="black", weight=1,
            fill=True, fill_color=_color(t.get("severity_index", 1)), fill_opacity=0.9,
        ).add_to(tac_map)

    folium.LayerControl().add_to(tac_map)
    tac_map.save(str(MAP_OUTPUT))
    log.info(f"Tactical map saved → {MAP_OUTPUT}")
    return MAP_OUTPUT


# ==============================================================================
#  MAIN APPLICATION
# ==============================================================================

class GroundStationApp(ctk.CTk):
    """
    Cobra VTOL Hybrid Ground Station.
    Telemetry IN  ← HTTP/JSON ← sitl_telem_bridge.py ← TCP 5763 ← Mission Planner
    Command   OUT → MAVLink   → TCP 5762              → Mission Planner → SITL
    """

    def __init__(self) -> None:
        super().__init__()
        self.title("Cobra VTOL — Hybrid Ground Station  [Windows]")
        self.geometry("1280x720")
        self.minsize(960, 540)

        log.info("═══════════════════════════════════════════════════")
        log.info("   HYBRID SYSTEM BOOT INITIATED  (Windows)")
        log.info("═══════════════════════════════════════════════════")

        # 1. AI model
        self.model = None
        if YOLO_OK:
            try:
                self.model = YOLO("yolov8n.pt")
                log.info("YOLOv8n Core Online.")
            except Exception as exc:
                log.error(f"YOLO load failed — {exc}")
        else:
            log.warning("ultralytics not installed — vision AI disabled.")

        # 2. Camera
        self.cap = None
        try:
            self.cap = find_obs_camera()
            if self.cap is None:
                log.warning("No camera detected. Start OBS Virtual Camera and restart.")
            else:
                log.info("OBS Video Feed Online.")
        except Exception as exc:
            log.error(f"Camera init failed — {exc}")

        # 3. MAVLink command link
        # Named self.drone (NOT self.master) to avoid Tkinter internal attribute collision.
        self.drone = None
        if PYMAVLINK_OK:
            try:
                log.info(f"Establishing MAVLink Command Link → {MAVLINK_CONN}")
                # source_system=254 avoids collision with Mission Planner's sysid 255
                self.drone = mavutil.mavlink_connection(MAVLINK_CONN, retries=0, source_system=254)

                # Wait for heartbeat in background — populates target_system/target_component.
                # Without this, commands go to system 0 and are silently ignored.
                def _wait_heartbeat():
                    log.info("Waiting for SITL heartbeat (up to 30s)...")
                    try:
                        self.drone.wait_heartbeat(timeout=30)
                        log.info(
                            f"✓ Heartbeat! target_system={self.drone.target_system} "
                            f"target_component={self.drone.target_component}"
                        )
                    except Exception as e:
                        log.error(f"Heartbeat timeout: {e} — QLOITER commands will be blocked.")

                threading.Thread(target=_wait_heartbeat, daemon=True).start()

                # Drain thread: flush specific packet types to prevent TCP buffer backpressure
                # that would freeze Mission Planner's waypoint upload handshake.
                # CRITICAL: HEARTBEAT is excluded so _wait_heartbeat() can receive it first.
                def _flush_tcp():
                    FLUSH_TYPES = [
                        'COMMAND_ACK', 'STATUSTEXT', 'SYS_STATUS',
                        'GPS_RAW_INT', 'VFR_HUD', 'ATTITUDE',
                        'NAV_CONTROLLER_OUTPUT', 'GLOBAL_POSITION_INT',
                    ]
                    while True:
                        try:
                            self.drone.recv_match(type=FLUSH_TYPES, blocking=True, timeout=1.0)
                        except Exception:
                            time.sleep(0.1)

                threading.Thread(target=_flush_tcp, daemon=True).start()
                log.info("SITL Command Pipeline Ready.")

            except Exception as exc:
                log.error(f"MAVLink Command Link failed — {exc} (Is SITL running?)")
                self.drone = None
        else:
            log.warning("pymavlink not installed — MAVLink commanding disabled.")

        # 4. State
        self.current_telemetry = {"lat": 0.0, "lon": 0.0, "alt_m": 0.0, "hdg": 0.0}
        self.active_target_gps: "tuple[float, float] | None" = None
        self._ctk_image_ref = None

        # 5. UI + loops
        self._build_ui()
        self._start_telemetry_loop()
        self._start_vision_loop()

    # ─── UI ───────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=3)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.video_frame = ctk.CTkFrame(self, corner_radius=8)
        self.video_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.video_label = ctk.CTkLabel(
            self.video_frame, text="Awaiting Video Feed...", font=("Consolas", 16),
        )
        self.video_label.pack(expand=True, fill="both")

        ctrl = ctk.CTkFrame(self, corner_radius=8)
        ctrl.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        ctk.CTkLabel(ctrl, text="HTTP TELEMETRY", font=("Consolas", 16, "bold")).pack(pady=(15, 5))
        self.telemetry_label = ctk.CTkLabel(
            ctrl, text="Polling server...", font=("Consolas", 13), justify="left",
        )
        self.telemetry_label.pack(pady=5, padx=10)

        ctk.CTkLabel(ctrl, text="TARGETING SYSTEM", font=("Consolas", 16, "bold")).pack(pady=(25, 5))
        self.lock_btn = ctk.CTkButton(
            ctrl, text="COMMAND: QLOITER",
            fg_color="#c0392b", hover_color="#922b21",
            font=("Consolas", 14, "bold"), command=self._cmd_qloiter,
        )
        self.lock_btn.pack(pady=8, fill="x", padx=20)

        ctk.CTkLabel(ctrl, text="Severity Index:", font=("Consolas", 12)).pack(pady=(15, 0))
        self.severity_slider = ctk.CTkSlider(ctrl, from_=1, to=10, number_of_steps=9)
        self.severity_slider.set(5)
        self.severity_slider.pack(pady=8, fill="x", padx=20)

        self.save_btn = ctk.CTkButton(
            ctrl, text="Save Target to DB", font=("Consolas", 13), command=self._save_target,
        )
        self.save_btn.pack(pady=8, fill="x", padx=20)

        self.map_btn = ctk.CTkButton(
            ctrl, text="Open Tactical Map", font=("Consolas", 13), command=self._open_tactical_map,
        )
        self.map_btn.pack(pady=8, fill="x", padx=20)

        self.status_label = ctk.CTkLabel(
            ctrl, text="System Nominal", text_color="#2ecc71", font=("Consolas", 12),
        )
        self.status_label.pack(side="bottom", pady=15)

    # ─── GPS Math (GSD Projection) ────────────────────────────────────────

    def _calculate_gps(self, px: float, py: float) -> "tuple[float, float] | None":
        try:
            cx, cy = IMG_WIDTH / 2.0, IMG_HEIGHT / 2.0
            alt = max(self.current_telemetry["alt_m"], 1.0)  # guard: no /0 at ground

            gsd_x = (2 * alt * math.tan(math.radians(FOV_H_DEG) / 2)) / IMG_WIDTH
            gsd_y = (2 * alt * math.tan(math.radians(FOV_V_DEG) / 2)) / IMG_HEIGHT

            dx_m = (px - cx) * gsd_x
            dy_m = (cy - py) * gsd_y

            hdg_rad = math.radians(self.current_telemetry["hdg"])
            east_m  =  dx_m * math.cos(hdg_rad) + dy_m * math.sin(hdg_rad)
            north_m = -dx_m * math.sin(hdg_rad) + dy_m * math.cos(hdg_rad)

            lat_off = north_m / 111320.0
            lon_off = east_m / (111320.0 * math.cos(math.radians(self.current_telemetry["lat"])))

            return round(self.current_telemetry["lat"] + lat_off, 8), \
                   round(self.current_telemetry["lon"] + lon_off, 8)
        except Exception as exc:
            log.error(f"GPS math error: {exc}", exc_info=True)
            return None

    # ─── QLOITER Command ──────────────────────────────────────────────────

    def _cmd_qloiter(self) -> None:
        """Send QLOITER (ArduPlane custom mode 19) via MAVLink TCP."""

        # Allow manual triggering even without YOLO target (for testing)
        if self.active_target_gps:
            self.status_label.configure(text="TARGET LOCKED — Loiter Commanded", text_color="#f39c12")
        else:
            self.status_label.configure(text="Manual QLOITER Commanded", text_color="#f39c12")

        if self.drone is None:
            self.status_label.configure(text="No MAVLink connection!", text_color="#e74c3c")
            log.error("Cannot send QLOITER: MAVLink disconnected.")
            return

        # Guard: heartbeat not yet received — target_system would be 0
        if self.drone.target_system == 0:
            self.status_label.configure(text="Waiting for heartbeat...", text_color="#e67e22")
            log.warning("QLOITER skipped — heartbeat not received yet (target_system=0).")
            return

        try:
            # Step 1: LOITER first to decelerate safely before VTOL transition
            self.drone.mav.command_long_send(
                self.drone.target_system, self.drone.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                12, 0, 0, 0, 0, 0  # mode 12 = LOITER
            )
            log.info("Intermediate: LOITER commanded — decelerating...")
            time.sleep(2)  # brief pause for speed reduction

            # Step 2: Switch to QLOITER (ArduPlane custom mode 19)
            self.drone.mav.command_long_send(
                self.drone.target_system,
                self.drone.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                19,            # QLOITER
                0, 0, 0, 0, 0
            )
            log.warning(
                f"──► MAVLink COMMAND SENT: QLOITER "
                f"(sys={self.drone.target_system} comp={self.drone.target_component} mode=19)"
            )
        except Exception as exc:
            log.error(f"QLOITER command failed: {exc}", exc_info=True)
            self.status_label.configure(text="Command Error!", text_color="#e74c3c")

    # ─── Save Target to JSON DB ───────────────────────────────────────────

    def _save_target(self) -> None:
        if not self.active_target_gps:
            self.status_label.configure(text="Cannot save: No target", text_color="#e74c3c")
            return
        try:
            severity = int(self.severity_slider.get())
            db: list = []
            if DB_FILE.exists():
                try:
                    with open(DB_FILE, "r", encoding="utf-8") as f:
                        db = json.load(f)
                except (json.JSONDecodeError, OSError):
                    db = []

            target = {
                "id":             f"Human {len(db) + 1}",
                "timestamp":      datetime.now().isoformat(),
                "gps_lat":        self.active_target_gps[0],
                "gps_lon":        self.active_target_gps[1],
                "severity_index": severity,
            }
            db.append(target)
            with open(DB_FILE, "w", encoding="utf-8") as f:
                json.dump(db, f, indent=4, ensure_ascii=False)

            self.status_label.configure(
                text=f"{target['id']} Saved (sev={severity})", text_color="#2ecc71"
            )
            log.info(f"Target saved → {target['id']}  sev={severity}")
        except Exception as exc:
            log.error(f"DB save failed: {exc}", exc_info=True)
            self.status_label.configure(text="DB Save Error!", text_color="#e74c3c")

    # ─── Tactical Map ─────────────────────────────────────────────────────

    def _open_tactical_map(self) -> None:
        self.status_label.configure(text="Generating map...", text_color="#3498db")

        def _worker():
            result = generate_tactical_map()
            if result and result.exists():
                webbrowser.open(result.as_uri())
                self.after(0, lambda: self.status_label.configure(
                    text="Map opened in browser", text_color="#2ecc71"))
            else:
                self.after(0, lambda: self.status_label.configure(
                    text="Map generation failed", text_color="#e74c3c"))

        threading.Thread(target=_worker, daemon=True).start()

    # ─── Telemetry Polling Loop (HTTP) ────────────────────────────────────

    def _start_telemetry_loop(self) -> None:
        self._poll_telemetry()

    def _poll_telemetry(self) -> None:
        try:
            resp = requests.get(TELEMETRY_URL, timeout=HTTP_TIMEOUT_S)
            if resp.status_code == 200:
                data    = resp.json()
                gps_raw = data.get("GPS_RAW_INT", {}).get("msg") or {}
                vfr_hud = data.get("VFR_HUD",     {}).get("msg") or {}

                self.current_telemetry["lat"]   = gps_raw.get("lat",     0) / 1e7
                self.current_telemetry["lon"]   = gps_raw.get("lon",     0) / 1e7
                self.current_telemetry["alt_m"] = vfr_hud.get("alt",     0.0)
                self.current_telemetry["hdg"]   = vfr_hud.get("heading", 0.0)

                t = self.current_telemetry
                self.telemetry_label.configure(
                    text=(
                        f"Lat : {t['lat']:.6f}\n"
                        f"Lon : {t['lon']:.6f}\n"
                        f"Alt : {t['alt_m']:.1f} m\n"
                        f"Hdg : {t['hdg']:.1f}°"
                    ),
                    text_color="white",
                )
        except requests.exceptions.ConnectionError:
            self.telemetry_label.configure(text="HTTP SERVER OFFLINE", text_color="#e74c3c")
        except requests.exceptions.Timeout:
            self.telemetry_label.configure(text="HTTP TIMEOUT", text_color="#e67e22")
        except Exception as exc:
            log.error(f"Telemetry poll error: {exc}", exc_info=True)
            self.telemetry_label.configure(text="TELEM ERROR", text_color="#e74c3c")

        self.after(TELEM_POLL_MS, self._poll_telemetry)

    # ─── Vision Loop (YOLOv8 + CTkImage) ──────────────────────────────────

    def _start_vision_loop(self) -> None:
        self._process_frame()

    def _process_frame(self) -> None:
        try:
            if self.cap is None or not self.cap.isOpened():
                self.after(VISION_LOOP_MS, self._process_frame)
                return

            ret, frame = self.cap.read()
            if not ret:
                self.after(VISION_LOOP_MS, self._process_frame)
                return

            self.active_target_gps = None

            if self.model is not None:
                try:
                    results = self.model.predict(source=frame, classes=[0], conf=0.5, verbose=False)
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                            gps = self._calculate_gps(cx, cy)
                            if gps:
                                self.active_target_gps = gps
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                                cv2.putText(
                                    frame, f"GPS: {gps[0]:.5f}, {gps[1]:.5f}",
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                                )
                except Exception as exc:
                    log.error(f"YOLO inference error: {exc}", exc_info=True)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img   = Image.fromarray(frame_rgb)
            disp_w = self.video_frame.winfo_width()  - 20
            disp_h = self.video_frame.winfo_height() - 20

            if disp_w > 50 and disp_h > 50:
                self._ctk_image_ref = ctk.CTkImage(
                    light_image=pil_img, dark_image=pil_img, size=(disp_w, disp_h),
                )
                self.video_label.configure(image=self._ctk_image_ref, text="")

        except Exception as exc:
            log.error(f"Vision loop crashed: {exc}", exc_info=True)

        self.after(VISION_LOOP_MS, self._process_frame)

    # ─── Shutdown ─────────────────────────────────────────────────────────

    def on_closing(self) -> None:
        log.info("System shutting down...")
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception as exc:
            log.error(f"Camera release error: {exc}")
        try:
            if self.drone:
                self.drone.close()
        except Exception as exc:
            log.error(f"MAVLink close error: {exc}")
        self.destroy()
        log.info("Shutdown complete.")


# ==============================================================================
#  ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
    log.info("Launching Cobra VTOL Ground Station (Windows)...")
    app = GroundStationApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
