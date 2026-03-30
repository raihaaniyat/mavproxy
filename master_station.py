"""
================================================================================
 COBRA VTOL — HYBRID GROUND CONTROL STATION (Windows Edition)
 Architecture : Split-Bridge (HTTP Telemetry IN → Local AI → MAVLink CMD OUT)
 Platform     : Windows 10/11  |  Python 3.10+
 Consolidates : master_station, telemetry_client, visual_module,
                operator_module, operator_module2, map_module
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
#  CONFIGURATION — edit these constants for your hardware setup
# ==============================================================================

# ── Paths (all relative to this script's location) ───────────────────────────
BASE_DIR        = Path(__file__).resolve().parent
LOG_DIR         = BASE_DIR / "system_logs"
DB_FILE         = BASE_DIR / "detected_targets.json"
MAP_OUTPUT      = BASE_DIR / "tactical_map.html"

# ── Network endpoints ────────────────────────────────────────────────────────
TELEMETRY_URL   = "http://127.0.0.1:56781/mavlink/"   # HTTP telemetry source
MAVLINK_CONN    = "COM3"                 # MAVLink command sink
# For a real drone you might switch to:
#   MAVLINK_CONN = "udp:0.0.0.0:14550"   (radio telemetry)
#   MAVLINK_CONN = "COM3"                 (serial USB)

# ── Camera / FOV ──────────────────────────────────────────────────────────────
IMG_WIDTH, IMG_HEIGHT = 1920, 1080
FOV_H_DEG, FOV_V_DEG = 60.0, 45.0       # Camera field-of-view

# ── Timing ────────────────────────────────────────────────────────────────────
TELEM_POLL_MS   = 200       # HTTP poll interval
VISION_LOOP_MS  = 30        # Frame processing interval
HTTP_TIMEOUT_S  = 0.2       # requests.get timeout

# ── Appearance ────────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ==============================================================================
#  LOGGING  (Windows-safe, pathlib-based)
# ==============================================================================

def setup_logger() -> logging.Logger:
    """Create a dual-output logger: DEBUG→file, INFO→console."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    stamp     = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file  = LOG_DIR / f"vtol_HYBRID_{stamp}.log"

    logger = logging.getLogger("GROUND_STATION")
    logger.setLevel(logging.DEBUG)

    # Prevent duplicate handlers on reload
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
    """
    Scan DirectShow device indices 0..max_index and return the first
    VideoCapture that successfully opens.  On Windows, OBS Virtual Camera
    registers as a DirectShow source whose index may shift if other
    cameras are plugged / unplugged.
    """
    log.info("Scanning for DirectShow cameras (OBS Virtual Camera)...")

    for idx in range(max_index):
        log.debug(f"  Probing index {idx} with CAP_DSHOW...")
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)

        if cap.isOpened():
            # Try to actually grab a frame — some ghost devices "open" but
            # never deliver frames.
            ret, _ = cap.read()
            if ret:
                log.info(f"  ✓ Camera found at index {idx}")
                # Set desired resolution and minimal buffer
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                return cap
            else:
                log.debug(f"  ✗ Index {idx} opened but delivered no frame.")
                cap.release()
        else:
            log.debug(f"  ✗ Index {idx} did not open.")

    log.warning("No DirectShow camera found on indices 0-%d.", max_index - 1)
    return None


# ==============================================================================
#  TACTICAL MAP GENERATION  (Folium)
# ==============================================================================

def generate_tactical_map() -> Path | None:
    """Read the target database and produce an interactive Leaflet map."""
    if not FOLIUM_OK:
        log.error("folium is not installed — cannot generate tactical map.")
        return None

    if not DB_FILE.exists():
        log.error(f"Database '{DB_FILE.name}' not found. Detect targets first.")
        return None

    try:
        with open(DB_FILE, "r", encoding="utf-8") as f:
            targets = json.load(f)
    except (json.JSONDecodeError, OSError) as exc:
        log.error(f"Failed to read target DB: {exc}")
        return None

    if not targets:
        log.warning("Target database is empty — nothing to map.")
        return None

    avg_lat = sum(t["gps_lat"] for t in targets) / len(targets)
    avg_lon = sum(t["gps_lon"] for t in targets) / len(targets)

    tac_map = folium.Map(location=[avg_lat, avg_lon], zoom_start=19)

    # Esri satellite imagery
    folium.TileLayer(
        tiles=(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
        attr="Esri", name="Esri Satellite",
        overlay=False, control=True,
    ).add_to(tac_map)

    def _severity_color(sev: int) -> str:
        if sev <= 3:
            return "green"
        if sev <= 7:
            return "orange"
        return "red"

    for t in targets:
        lat, lon = t["gps_lat"], t["gps_lon"]
        sev = t.get("severity_index", 1)
        hid = t.get("id", "Unknown")
        popup_html = (
            f"<b>{hid}</b><br>"
            f"Severity: {sev}<br>"
            f"Lat: {lat:.6f}<br>"
            f"Lon: {lon:.6f}"
        )
        folium.CircleMarker(
            location=[lat, lon], radius=8,
            popup=folium.Popup(popup_html, max_width=200),
            color="black", weight=1,
            fill=True, fill_color=_severity_color(sev), fill_opacity=0.9,
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

    Data flows:
        Telemetry IN  ←  HTTP GET  ←  127.0.0.1:56781/mavlink/
        Video     IN  ←  DirectShow (OBS Virtual Camera)
        Command   OUT →  MAVLink TCP → 127.0.0.1:5760
    """

    # ─── Initialisation ──────────────────────────────────────────────────

    def __init__(self) -> None:
        super().__init__()
        self.title("Cobra VTOL — Hybrid Ground Station  [Windows]")
        self.geometry("1280x720")
        self.minsize(960, 540)

        log.info("═══════════════════════════════════════════════════")
        log.info("   HYBRID SYSTEM BOOT INITIATED  (Windows)")
        log.info("═══════════════════════════════════════════════════")

        # -- 1. AI model -------------------------------------------------------
        self.model = None
        if YOLO_OK:
            try:
                self.model = YOLO("yolov8n.pt")
                log.info("YOLOv8n Core Online.")
            except Exception as exc:
                log.error(f"CRITICAL: YOLO load failed — {exc}")
        else:
            log.warning("ultralytics not installed — vision AI disabled.")

        # -- 2. Video capture (DirectShow) -------------------------------------
        self.cap = None
        try:
            self.cap = find_obs_camera()
            if self.cap is None:
                log.warning(
                    "No camera detected. Video feed will show placeholder. "
                    "Start OBS Virtual Camera and restart the station."
                )
            else:
                log.info("OBS Video Feed Online (DirectShow).")
        except Exception as exc:
            log.error(f"CRITICAL: Camera init failed — {exc}")

        # -- 3. MAVLink command link (WRITE-ONLY) ------------------------------
        # IMPORTANT: named  self.drone  to avoid Tkinter's internal
        #            self.master attribute collision (instant crash).
        self.drone = None
        if PYMAVLINK_OK:
            try:
                log.info(f"Establishing MAVLink Command Link → {MAVLINK_CONN}")
                # retries=0 prevents 'actively refused it sleeping' loop
                self.drone = mavutil.mavlink_connection(MAVLINK_CONN, retries=0)
                log.info("SITL Command Pipeline Ready.")
            except Exception as exc:
                log.error(f"MAVLink Command Link failed — {exc} (Is SITL running?)")
                self.drone = None
        else:
            log.warning("pymavlink not installed — MAVLink commanding disabled.")

        # -- 4. State ----------------------------------------------------------
        self.current_telemetry = {
            "lat": 0.0, "lon": 0.0, "alt_m": 0.0, "hdg": 0.0,
        }
        self.active_target_gps: tuple[float, float] | None = None
        self._ctk_image_ref = None  # prevent GC of displayed image

        # -- 5. UI + loops  ----------------------------------------------------
        self._build_ui()
        self._start_telemetry_loop()
        self._start_vision_loop()

    # ─── UI Layout ────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """Construct a lightweight two-column dashboard."""
        self.grid_columnconfigure(0, weight=3)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ── Left: Video feed ─────────────────────────────────────────────
        self.video_frame = ctk.CTkFrame(self, corner_radius=8)
        self.video_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.video_label = ctk.CTkLabel(
            self.video_frame, text="Awaiting Video Feed...",
            font=("Consolas", 16),
        )
        self.video_label.pack(expand=True, fill="both")

        # ── Right: Controls ──────────────────────────────────────────────
        ctrl = ctk.CTkFrame(self, corner_radius=8)
        ctrl.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # -- Telemetry readout -------------------------------------------------
        ctk.CTkLabel(
            ctrl, text="HTTP TELEMETRY",
            font=("Consolas", 16, "bold"),
        ).pack(pady=(15, 5))

        self.telemetry_label = ctk.CTkLabel(
            ctrl, text="Polling server...",
            font=("Consolas", 13), justify="left",
        )
        self.telemetry_label.pack(pady=5, padx=10)

        # -- Targeting ---------------------------------------------------------
        ctk.CTkLabel(
            ctrl, text="TARGETING SYSTEM",
            font=("Consolas", 16, "bold"),
        ).pack(pady=(25, 5))

        self.lock_btn = ctk.CTkButton(
            ctrl, text="COMMAND: QLOITER",
            fg_color="#c0392b", hover_color="#922b21",
            font=("Consolas", 14, "bold"),
            command=self._cmd_qloiter,
        )
        self.lock_btn.pack(pady=8, fill="x", padx=20)

        ctk.CTkLabel(ctrl, text="Severity Index:", font=("Consolas", 12)).pack(
            pady=(15, 0),
        )
        self.severity_slider = ctk.CTkSlider(
            ctrl, from_=1, to=10, number_of_steps=9,
        )
        self.severity_slider.set(5)
        self.severity_slider.pack(pady=8, fill="x", padx=20)

        self.save_btn = ctk.CTkButton(
            ctrl, text="Save Target to DB",
            font=("Consolas", 13),
            command=self._save_target,
        )
        self.save_btn.pack(pady=8, fill="x", padx=20)

        # -- Map ---------------------------------------------------------------
        self.map_btn = ctk.CTkButton(
            ctrl, text="Open Tactical Map",
            font=("Consolas", 13),
            command=self._open_tactical_map,
        )
        self.map_btn.pack(pady=8, fill="x", padx=20)

        # -- Status bar --------------------------------------------------------
        self.status_label = ctk.CTkLabel(
            ctrl, text="System Nominal",
            text_color="#2ecc71", font=("Consolas", 12),
        )
        self.status_label.pack(side="bottom", pady=15)

    # ─── GPS Math (GSD Projection) ────────────────────────────────────────

    def _calculate_gps(
        self, px: float, py: float,
    ) -> tuple[float, float] | None:
        """
        Project a pixel (px, py) in the camera frame to real-world GPS using
        Ground Sample Distance math.

        CRITICAL: altitude is clamped to ≥1.0 m to prevent division-by-zero
        when the SITL drone spawns at 0 m AGL.
        """
        try:
            cx, cy = IMG_WIDTH / 2.0, IMG_HEIGHT / 2.0

            # ── Zero-division guard ──────────────────────────────────────
            alt = max(self.current_telemetry["alt_m"], 1.0)

            gsd_x = (2 * alt * math.tan(math.radians(FOV_H_DEG) / 2)) / IMG_WIDTH
            gsd_y = (2 * alt * math.tan(math.radians(FOV_V_DEG) / 2)) / IMG_HEIGHT

            dx_m = (px - cx) * gsd_x
            dy_m = (cy - py) * gsd_y   # image-Y inverted vs. world-Y

            hdg_rad = math.radians(self.current_telemetry["hdg"])

            east_m  =  dx_m * math.cos(hdg_rad) + dy_m * math.sin(hdg_rad)
            north_m = -dx_m * math.sin(hdg_rad) + dy_m * math.cos(hdg_rad)

            lat_off = north_m / 111320.0
            lon_off = east_m / (
                111320.0 * math.cos(math.radians(self.current_telemetry["lat"]))
            )

            t_lat = round(self.current_telemetry["lat"] + lat_off, 8)
            t_lon = round(self.current_telemetry["lon"] + lon_off, 8)

            log.debug(f"GPS projection: px=({px},{py}) → ({t_lat}, {t_lon})")
            return t_lat, t_lon

        except Exception as exc:
            log.error(f"Math Engine CRASHED: {exc}", exc_info=True)
            return None

    # ─── QLOITER Command ──────────────────────────────────────────────────

    def _cmd_qloiter(self) -> None:
        """Send a QLOITER mode-change command over the MAVLink TCP link."""
        if not self.active_target_gps:
            self.status_label.configure(
                text="NO HUMAN IN FOV", text_color="#e74c3c",
            )
            log.warning("QLOITER aborted — no active target.")
            return

        self.status_label.configure(
            text="TARGET LOCKED — Loiter Commanded", text_color="#f39c12",
        )

        if self.drone is None:
            log.error("Cannot send QLOITER: MAVLink disconnected.")
            return

        try:
            self.drone.set_mode("QLOITER")
            log.warning("──► MAVLINK COMMAND SENT: QLOITER")
        except Exception as exc:
            log.error(f"Failed to send QLOITER to SITL: {exc}", exc_info=True)

    # ─── Save Target to JSON DB ───────────────────────────────────────────

    def _save_target(self) -> None:
        """Persist the currently locked GPS target into the JSON database."""
        if not self.active_target_gps:
            self.status_label.configure(
                text="Cannot save: No target", text_color="#e74c3c",
            )
            return

        try:
            severity = int(self.severity_slider.get())

            # ── Read existing DB (pathlib) ────────────────────────────────
            db: list[dict] = []
            if DB_FILE.exists():
                try:
                    with open(DB_FILE, "r", encoding="utf-8") as f:
                        db = json.load(f)
                except (json.JSONDecodeError, OSError):
                    log.warning("Corrupt / empty DB file — starting fresh.")
                    db = []

            target = {
                "id":              f"Human {len(db) + 1}",
                "timestamp":       datetime.now().isoformat(),
                "gps_lat":         self.active_target_gps[0],
                "gps_lon":         self.active_target_gps[1],
                "severity_index":  severity,
            }
            db.append(target)

            with open(DB_FILE, "w", encoding="utf-8") as f:
                json.dump(db, f, indent=4, ensure_ascii=False)

            self.status_label.configure(
                text=f"{target['id']} Saved (sev={severity})",
                text_color="#2ecc71",
            )
            log.info(f"Target saved → {target['id']}  sev={severity}")

        except Exception as exc:
            log.error(f"Database save failed: {exc}", exc_info=True)
            self.status_label.configure(
                text="DB Save Error!", text_color="#e74c3c",
            )

    # ─── Tactical Map ─────────────────────────────────────────────────────

    def _open_tactical_map(self) -> None:
        """Generate and open the Folium tactical map in the default browser."""
        self.status_label.configure(
            text="Generating map...", text_color="#3498db",
        )

        def _worker():
            result = generate_tactical_map()
            if result and result.exists():
                webbrowser.open(result.as_uri())
                log.info("Tactical map opened in browser.")
                self.after(0, lambda: self.status_label.configure(
                    text="Map opened in browser", text_color="#2ecc71",
                ))
            else:
                self.after(0, lambda: self.status_label.configure(
                    text="Map generation failed", text_color="#e74c3c",
                ))

        threading.Thread(target=_worker, daemon=True).start()

    # ─── Telemetry Polling Loop (HTTP) ────────────────────────────────────

    def _start_telemetry_loop(self) -> None:
        self._poll_telemetry()

    def _poll_telemetry(self) -> None:
        """Non-blocking HTTP poll; reschedules itself via .after()."""
        try:
            log.debug(f"HTTP GET → {TELEMETRY_URL}")
            resp = requests.get(TELEMETRY_URL, timeout=HTTP_TIMEOUT_S)

            if resp.status_code == 200:
                data = resp.json()
                
                # Unwrap MAVLink REST API nested payload
                gps_raw = data.get("GPS_RAW_INT", {}).get("msg") or {}
                vfr_hud = data.get("VFR_HUD", {}).get("msg") or {}

                # lat/lon scaled by 1e7 in GPS_RAW_INT
                self.current_telemetry["lat"]   = gps_raw.get("lat", 0) / 1e7
                self.current_telemetry["lon"]   = gps_raw.get("lon", 0) / 1e7
                
                # alt (m) and heading (deg) in VFR_HUD
                self.current_telemetry["alt_m"] = vfr_hud.get("alt", 0.0)
                self.current_telemetry["hdg"]   = vfr_hud.get("heading", 0.0)

                t = self.current_telemetry
                text = (
                    f"Lat : {t['lat']:.6f}\n"
                    f"Lon : {t['lon']:.6f}\n"
                    f"Alt : {t['alt_m']:.1f} m\n"
                    f"Hdg : {t['hdg']:.1f}°"
                )
                self.telemetry_label.configure(text=text, text_color="white")
                log.debug(
                    f"Telemetry OK — lat={t['lat']:.6f} lon={t['lon']:.6f} "
                    f"alt={t['alt_m']:.1f} hdg={t['hdg']:.1f}"
                )
            else:
                log.warning(f"HTTP {resp.status_code} from telemetry server.")
                self.telemetry_label.configure(
                    text=f"HTTP {resp.status_code}", text_color="#e67e22",
                )

        except requests.exceptions.ConnectionError:
            log.debug("Telemetry server unreachable (ConnectionError).")
            self.telemetry_label.configure(
                text="HTTP SERVER OFFLINE", text_color="#e74c3c",
            )
        except requests.exceptions.Timeout:
            log.debug("Telemetry request timed out.")
            self.telemetry_label.configure(
                text="HTTP TIMEOUT", text_color="#e67e22",
            )
        except Exception as exc:
            log.error(f"Telemetry poll error: {exc}", exc_info=True)
            self.telemetry_label.configure(
                text="TELEM ERROR", text_color="#e74c3c",
            )

        # Reschedule
        self.after(TELEM_POLL_MS, self._poll_telemetry)

    # ─── Vision Loop (YOLOv8 + CTkImage) ──────────────────────────────────

    def _start_vision_loop(self) -> None:
        self._process_frame()

    def _process_frame(self) -> None:
        """
        Grab a frame, run YOLO inference, annotate, and render to CTkImage.
        Reschedules itself via .after() so Tkinter never blocks.
        """
        try:
            if self.cap is None or not self.cap.isOpened():
                log.debug("No camera — skipping vision frame.")
                self.after(VISION_LOOP_MS, self._process_frame)
                return

            ret, frame = self.cap.read()
            if not ret:
                log.debug("cap.read() returned False — no frame.")
                self.after(VISION_LOOP_MS, self._process_frame)
                return

            # ── YOLO inference ───────────────────────────────────────────
            self.active_target_gps = None

            if self.model is not None:
                try:
                    results = self.model.predict(
                        source=frame, classes=[0], conf=0.5, verbose=False,
                    )

                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx = int((x1 + x2) / 2)
                            cy = int((y1 + y2) / 2)

                            gps = self._calculate_gps(cx, cy)
                            if gps:
                                self.active_target_gps = gps

                                # Annotate frame
                                cv2.rectangle(
                                    frame, (x1, y1), (x2, y2),
                                    (0, 0, 255), 2,
                                )
                                cv2.circle(
                                    frame, (cx, cy), 5, (0, 255, 0), -1,
                                )
                                cv2.putText(
                                    frame,
                                    f"GPS: {gps[0]:.5f}, {gps[1]:.5f}",
                                    (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2,
                                )
                                log.debug(
                                    f"Human detected px=({cx},{cy}) → "
                                    f"GPS=({gps[0]:.6f},{gps[1]:.6f})"
                                )

                except Exception as exc:
                    log.error(f"YOLO inference error: {exc}", exc_info=True)

            # ── Render to CTkImage (High-DPI safe) ───────────────────────
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img   = Image.fromarray(frame_rgb)

            disp_w = self.video_frame.winfo_width()  - 20
            disp_h = self.video_frame.winfo_height() - 20

            if disp_w > 50 and disp_h > 50:
                # CTkImage handles DPI scaling internally — no blur on Windows
                self._ctk_image_ref = ctk.CTkImage(
                    light_image=pil_img, dark_image=pil_img,
                    size=(disp_w, disp_h),
                )
                self.video_label.configure(
                    image=self._ctk_image_ref, text="",
                )

        except Exception as exc:
            log.error(f"Vision loop crashed: {exc}", exc_info=True)

        # Reschedule
        self.after(VISION_LOOP_MS, self._process_frame)

    # ─── Shutdown ─────────────────────────────────────────────────────────

    def on_closing(self) -> None:
        """Clean teardown of all hardware resources."""
        log.info("System shutting down...")

        try:
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
                log.debug("Camera released.")
        except Exception as exc:
            log.error(f"Error releasing camera: {exc}")

        try:
            if self.drone is not None:
                self.drone.close()
                log.debug("MAVLink connection closed.")
        except Exception as exc:
            log.error(f"Error closing MAVLink: {exc}")

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
