"""
================================================================================
 COBRA VTOL — DIAGNOSTIC GCS (Pose Estimation Edition)
 Architecture : Unified MAVLink TCP (Mission Planner SITL)
 Platform     : Windows 10/11  |  Python 3.10+
 Firmware     : ArduPlane / QuadPlane
================================================================================
"""

import cv2
import math
import json
import time
import logging
import threading
import webbrowser
from pathlib import Path
from datetime import datetime

import customtkinter as ctk
from PIL import Image

# ── Optional heavy deps ──────────────────────────────────────────────────────
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

MAVLINK_CONN    = "tcp:127.0.0.1:5762" 

IMG_WIDTH, IMG_HEIGHT = 1920, 1080
FOV_H_DEG, FOV_V_DEG = 60.0, 45.0 

TELEM_POLL_MS   = 100 
VISION_LOOP_MS  = 30 

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


# ==============================================================================
#  LOGGING
# ==============================================================================
def setup_logger() -> logging.Logger:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp     = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file  = LOG_DIR / f"vtol_UNIFIED_{stamp}.log"

    logger = logging.getLogger("GROUND_STATION")
    logger.setLevel(logging.DEBUG)

    if not logger.handlers:
        fh = logging.FileHandler(str(log_file), encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter("%(asctime)s - [%(levelname)s] - %(message)s"))

        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))

        logger.addHandler(fh)
        logger.addHandler(ch)

    return logger

log = setup_logger()


# ==============================================================================
#  CAMERA DISCOVERY
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
            else:
                cap.release()
    log.warning("No DirectShow camera found.")
    return None


# ==============================================================================
#  MAP GENERATION
# ==============================================================================
def generate_tactical_map() -> Path | None:
    if not FOLIUM_OK: return None
    if not DB_FILE.exists(): return None

    try:
        with open(DB_FILE, "r", encoding="utf-8") as f:
            targets = json.load(f)
    except Exception:
        return None

    valid_targets = [t for t in targets if "gps_lat" in t and "gps_lon" in t]
    if not valid_targets: return None

    avg_lat = sum(t["gps_lat"] for t in valid_targets) / len(valid_targets)
    avg_lon = sum(t["gps_lon"] for t in valid_targets) / len(valid_targets)

    tac_map = folium.Map(location=[avg_lat, avg_lon], zoom_start=19)

    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri", name="Esri Satellite",
        overlay=False, control=True,
    ).add_to(tac_map)

    def _severity_color(sev: int) -> str:
        if sev <= 3: return "green"
        if sev <= 7: return "orange"
        return "red"

    for t in valid_targets:
        lat, lon = t["gps_lat"], t["gps_lon"]
        sev = t.get("severity_index", 1)
        hid = t.get("id", "Unknown")
        pose = t.get("pose", "UNKNOWN")
        
        # Enhanced popup with Pose data
        popup_html = (
            f"<div style='min-width: 150px;'>"
            f"<b>{hid}</b><br>"
            f"Severity: <b><span style='color:{_severity_color(sev)}'>{sev}</span></b><br>"
            f"Pose: <b>{pose}</b><br>"
            f"Lat: {lat:.6f}<br>Lon: {lon:.6f}"
            f"</div>"
        )
        folium.CircleMarker(
            location=[lat, lon], radius=8,
            popup=folium.Popup(popup_html, max_width=250),
            color="black", weight=1,
            fill=True, fill_color=_severity_color(sev), fill_opacity=0.9,
        ).add_to(tac_map)

    folium.LayerControl().add_to(tac_map)
    tac_map.save(str(MAP_OUTPUT))
    return MAP_OUTPUT


# ==============================================================================
#  MAIN APPLICATION
# ==============================================================================
class GroundStationApp(ctk.CTk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Cobra VTOL — Diagnostic Ground Station  [Windows]")
        self.geometry("1280x800")
        self.minsize(960, 600)

        log.info("═══════════════════════════════════════════════════")
        log.info("   DIAGNOSTIC BOOT INITIATED (ArduPlane Edition)")
        log.info("═══════════════════════════════════════════════════")

        self.model = None
        if YOLO_OK:
            try:
                self.model = YOLO("yolov8n-pose.pt")
                log.info("YOLOv8-Pose Core Online.")
            except Exception as exc:
                log.error(f"CRITICAL: YOLO load failed — {exc}")

        self.cap = find_obs_camera()

        self.drone = None
        self.heartbeat_received = False
        if PYMAVLINK_OK:
            try:
                self.drone = mavutil.mavlink_connection(MAVLINK_CONN, source_system=254, retries=0)
                log.info("SITL Pipeline Ready (Awaiting Heartbeat).")
            except Exception as exc:
                log.error(f"MAVLink Link failed — {exc}")

        self.current_telemetry = {"lat": 0.0, "lon": 0.0, "alt_m": 0.0, "hdg": 0.0}
        self.active_target_gps: tuple[float, float] | None = None
        self.last_detected_pose = "UNKNOWN"
        self._ctk_image_ref = None 

        self._build_ui()
        self._start_telemetry_loop()
        self._start_vision_loop()

    # ─── Thread-Safe GUI Logger ──────────────────────────────────────────────
    
    def _log_ui(self, message: str) -> None:
        log.info(message)
        def _update():
            self.console.configure(state="normal")
            ts = datetime.now().strftime("%H:%M:%S")
            self.console.insert("end", f"[{ts}] {message}\n")
            self.console.see("end")
            self.console.configure(state="disabled")
        self.after(0, _update)

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=3)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ── Left: Video feed ──
        self.video_frame = ctk.CTkFrame(self, corner_radius=8)
        self.video_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.video_label = ctk.CTkLabel(self.video_frame, text="Awaiting Video Feed...", font=("Consolas", 16))
        self.video_label.pack(expand=True, fill="both")

        # ── Right: Controls ──
        ctrl = ctk.CTkFrame(self, corner_radius=8)
        ctrl.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Telemetry Text
        ctk.CTkLabel(ctrl, text="MAVLINK TELEMETRY", font=("Consolas", 16, "bold")).pack(pady=(10, 0))
        self.telemetry_label = ctk.CTkLabel(ctrl, text="Awaiting Heartbeat...", font=("Consolas", 13), justify="left")
        self.telemetry_label.pack(pady=5, padx=10)

        # Diagnostics Console
        ctk.CTkLabel(ctrl, text="COMMAND DIAGNOSTICS", font=("Consolas", 14, "bold")).pack(pady=(10, 0))
        self.console = ctk.CTkTextbox(ctrl, height=140, font=("Consolas", 11), fg_color="#1e1e1e", text_color="#00ff00")
        self.console.pack(fill="x", padx=10, pady=5)
        self.console.configure(state="disabled")

        # Flight Modes
        ctk.CTkLabel(ctrl, text="FLIGHT MODES", font=("Consolas", 14, "bold")).pack(pady=(10, 0))
        mode_frame = ctk.CTkFrame(ctrl, fg_color="transparent")
        mode_frame.pack(fill="x", padx=10)
        
        self.auto_btn = ctk.CTkButton(mode_frame, text="AUTO", width=80, command=self._cmd_auto)
        self.auto_btn.pack(side="left", expand=True, padx=2)
        
        self.guided_btn = ctk.CTkButton(mode_frame, text="GUIDED", width=80, command=self._cmd_guided)
        self.guided_btn.pack(side="left", expand=True, padx=2)

        # Targeting & QLOITER
        self.lock_btn = ctk.CTkButton(
            ctrl, text="TARGET: QLOITER", fg_color="#c0392b", hover_color="#922b21",
            font=("Consolas", 14, "bold"), command=self._cmd_qloiter
        )
        self.lock_btn.pack(pady=10, fill="x", padx=20)

        # Altitude Selection
        alt_frame = ctk.CTkFrame(ctrl, fg_color="transparent")
        alt_frame.pack(fill="x", padx=20, pady=0)
        
        self.alt_entry = ctk.CTkEntry(alt_frame, width=80, placeholder_text="Alt (m)")
        self.alt_entry.pack(side="left", padx=5)
        
        self.alt_btn = ctk.CTkButton(alt_frame, text="SET ALT", width=80, command=self._cmd_change_alt)
        self.alt_btn.pack(side="left", padx=5, expand=True)

        # Save to DB 
        ctk.CTkLabel(ctrl, text="Severity Index:", font=("Consolas", 12)).pack(pady=(10, 0))
        self.severity_slider = ctk.CTkSlider(ctrl, from_=1, to=10, number_of_steps=9)
        self.severity_slider.set(5)
        self.severity_slider.pack(pady=5, fill="x", padx=20)

        self.save_btn = ctk.CTkButton(ctrl, text="Save Target to DB", fg_color="#27ae60", hover_color="#2ecc71", font=("Consolas", 13, "bold"), command=self._save_target)
        self.save_btn.pack(pady=5, fill="x", padx=20)

        self.map_btn = ctk.CTkButton(ctrl, text="Open Tactical Map", font=("Consolas", 13), command=self._open_tactical_map)
        self.map_btn.pack(pady=5, fill="x", padx=20)

        # Status output
        self.status_label = ctk.CTkLabel(ctrl, text="System Nominal", text_color="#2ecc71", font=("Consolas", 12))
        self.status_label.pack(side="bottom", pady=10)


    def _calculate_gps(self, px: float, py: float) -> tuple[float, float] | None:
        try:
            cx, cy = IMG_WIDTH / 2.0, IMG_HEIGHT / 2.0
            alt = max(self.current_telemetry["alt_m"], 1.0)

            gsd_x = (2 * alt * math.tan(math.radians(FOV_H_DEG) / 2)) / IMG_WIDTH
            gsd_y = (2 * alt * math.tan(math.radians(FOV_V_DEG) / 2)) / IMG_HEIGHT

            dx_m = (px - cx) * gsd_x
            dy_m = (cy - py) * gsd_y

            hdg_rad = math.radians(self.current_telemetry["hdg"])
            east_m  =  dx_m * math.cos(hdg_rad) + dy_m * math.sin(hdg_rad)
            north_m = -dx_m * math.sin(hdg_rad) + dy_m * math.cos(hdg_rad)

            lat_off = north_m / 111320.0
            lon_off = east_m / (111320.0 * math.cos(math.radians(self.current_telemetry["lat"])))

            return round(self.current_telemetry["lat"] + lat_off, 8), round(self.current_telemetry["lon"] + lon_off, 8)
        except Exception:
            return None

    # ─── Flight Mode Methods (ArduPlane Mappings) ────────────────────────────

    def _cmd_auto(self) -> None:
        # ArduPlane AUTO Mode = 10
        self._set_flight_mode("AUTO", 10)

    def _cmd_guided(self) -> None:
        # ArduPlane GUIDED Mode = 15
        self._set_flight_mode("GUIDED", 15)

    def _set_flight_mode(self, mode_name: str, mode_id: int) -> None:
        if self.drone is None: return
        try:
            self._log_ui(f"➡️ SEND: MAV_CMD_DO_SET_MODE ({mode_id} - {mode_name})")
            self.drone.mav.command_long_send(
                self.drone.target_system, self.drone.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0
            )
            self.status_label.configure(text=f"MODE: {mode_name}", text_color="#3498db")
        except Exception as e:
            self._log_ui(f"❌ ERROR: {e}")

    # ─── Altitude Change ─────────────────────────────────────────────────────
    
    def _cmd_change_alt(self) -> None:
        if self.drone is None: return
        try:
            target_alt = float(self.alt_entry.get())
            sys_id = getattr(self.drone, 'target_system', 1)
            comp_id = getattr(self.drone, 'target_component', 1)

            self._log_ui(f"➡️ SEND: MAV_CMD_DO_REPOSITION (192) -> Alt={target_alt}m")
            self.drone.mav.command_int_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0, 0, -1.0, 1, 0, 0, 
                int(self.current_telemetry["lat"] * 1e7),
                int(self.current_telemetry["lon"] * 1e7),
                target_alt
            )
        except ValueError:
            self._log_ui("❌ ERROR: Invalid Altitude Input")
        except Exception as e:
            self._log_ui(f"❌ ERROR: {e}")

    # ─── Robust QLOITER (Threaded for QuadPlane Transition) ──────────────────

    def _cmd_qloiter(self) -> None:
        if not self.active_target_gps:
            self._log_ui("⚠️ ABORT: No target acquired in FOV.")
            return
        if self.drone is None: return

        def _send_sequence():
            try:
                # ArduPlane requires a brief stabilization in fixed-wing LOITER
                # before engaging the VTOL QLOITER motors
                self._log_ui("➡️ SEND: MAV_CMD_DO_SET_MODE (12 - LOITER)")
                self.drone.mav.command_long_send(
                    self.drone.target_system, self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    12, 0, 0, 0, 0, 0
                )
                
                time.sleep(1.5)

                self._log_ui("➡️ SEND: MAV_CMD_DO_SET_MODE (19 - QLOITER)")
                self.drone.mav.command_long_send(
                    self.drone.target_system, self.drone.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    19, 0, 0, 0, 0, 0
                )
            except Exception as exc:
                self._log_ui(f"❌ THREAD ERROR: {exc}")
                
        threading.Thread(target=_send_sequence, daemon=True).start()

    # ─── DB Save (Pure Save) ─────────────────────────────────────────────────

    def _save_target(self) -> None:
        """Saves target GPS+Pose to JSON. Does NOT auto-regenerate the map."""
        if not self.active_target_gps:
            self.status_label.configure(text="Cannot save: No target", text_color="#e74c3c")
            return
        try:
            severity = int(self.severity_slider.get())
            db = []
            if DB_FILE.exists():
                try:
                    with open(DB_FILE, "r", encoding="utf-8") as f:
                        db = json.load(f)
                except Exception:
                    pass

            target = {
                "id": f"Human {len(db) + 1}",
                "timestamp": datetime.now().isoformat(),
                "gps_lat": self.active_target_gps[0],
                "gps_lon": self.active_target_gps[1],
                "severity_index": severity,
                "pose": self.last_detected_pose 
            }
            db.append(target)
            
            with open(DB_FILE, "w", encoding="utf-8") as f:
                json.dump(db, f, indent=4, ensure_ascii=False)

            self.status_label.configure(text=f"{target['id']} Saved", text_color="#2ecc71")
            self._log_ui(f"💾 SAVED: {target['id']} | Pose: {self.last_detected_pose}")

        except Exception as exc:
            self._log_ui(f"❌ DB ERROR: {exc}")

    def _open_tactical_map(self) -> None:
        """Generates the map on-demand and opens it in the browser."""
        self._log_ui("🗺️ Generating Tactical Map...")
        def _worker():
            result = generate_tactical_map()
            if result and result.exists():
                webbrowser.open(result.as_uri())
            else:
                self._log_ui("❌ MAP ERROR: Could not generate map.")
        threading.Thread(target=_worker, daemon=True).start()


    # ─── Telemetry Polling & ACK Parsing ─────────────────────────────────────

    def _start_telemetry_loop(self) -> None:
        self._poll_telemetry()

    def _poll_telemetry(self) -> None:
        if self.drone is None:
            self.after(TELEM_POLL_MS, self._poll_telemetry)
            return

        try:
            if not self.heartbeat_received:
                msg = self.drone.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.get_srcSystem() != 0:
                    self.heartbeat_received = True
                    self.drone.target_system    = msg.get_srcSystem()
                    self.drone.target_component = msg.get_srcComponent()
                    self.drone.mav.request_data_stream_send(
                        self.drone.target_system, self.drone.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                    )
                    self._log_ui("✅ LINK ESTABLISHED: Heartbeat Confirmed.")
                else:
                    self.telemetry_label.configure(text="Awaiting Heartbeat...", text_color="#e67e22")
                    self.after(TELEM_POLL_MS, self._poll_telemetry)
                    return

            while True:
                msg = self.drone.recv_match(blocking=False)
                if not msg:
                    break

                msg_type = msg.get_type()

                if msg_type == 'GLOBAL_POSITION_INT':
                    self.current_telemetry["lat"] = msg.lat / 1e7
                    self.current_telemetry["lon"] = msg.lon / 1e7
                    self.current_telemetry["alt_m"] = msg.relative_alt / 1000.0 
                elif msg_type == 'VFR_HUD':
                    self.current_telemetry["hdg"] = msg.heading
                elif msg_type == 'COMMAND_ACK':
                    ack_results = {0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED"}
                    res_str = ack_results.get(msg.result, f"UNKNOWN({msg.result})")
                    self._log_ui(f"⬅️ ACK [CMD: {msg.command}]: {res_str}")

            t = self.current_telemetry
            text = (
                f"Lat : {t['lat']:.6f}\n"
                f"Lon : {t['lon']:.6f}\n"
                f"Alt : {t['alt_m']:.1f} m\n"
                f"Hdg : {t['hdg']:.1f}°"
            )
            self.telemetry_label.configure(text=text, text_color="white")

        except Exception as exc:
            pass

        self.after(TELEM_POLL_MS, self._poll_telemetry)

    # ─── Vision & Pose Loop ──────────────────────────────────────────────────

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
            self.last_detected_pose = "UNKNOWN"
            
            annotated_frame = frame.copy()

            if self.model is not None:
                try:
                    # Run YOLO Pose prediction
                    results = self.model.predict(source=frame, classes=[0], conf=0.5, verbose=False)
                    
                    if len(results) > 0 and len(results[0].boxes) > 0:
                        annotated_frame = results[0].plot()

                        for box in results[0].boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                            
                            # ── Drone-Optimized Pose Heuristic ──
                            box_w = x2 - x1
                            box_h = y2 - y1
                            ratio = box_h / float(box_w) if box_w > 0 else 0
                            
                            if ratio < 0.8:
                                self.last_detected_pose = "LYING DOWN"
                            elif ratio < 1.3:
                                self.last_detected_pose = "SITTING/CROUCHING"
                            else:
                                self.last_detected_pose = "STANDING"

                            gps = self._calculate_gps(cx, cy)
                            if gps:
                                self.active_target_gps = gps
                                text_tag = f"GPS: {gps[0]:.5f}, {gps[1]:.5f} | {self.last_detected_pose}"
                                cv2.putText(
                                    annotated_frame, text_tag,
                                    (x1, max(15, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
                                )
                except Exception:
                    pass

            # Render to UI
            frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            pil_img   = Image.fromarray(frame_rgb)
            disp_w = self.video_frame.winfo_width()  - 20
            disp_h = self.video_frame.winfo_height() - 20

            if disp_w > 50 and disp_h > 50:
                self._ctk_image_ref = ctk.CTkImage(
                    light_image=pil_img, dark_image=pil_img, size=(disp_w, disp_h)
                )
                self.video_label.configure(image=self._ctk_image_ref, text="")
        except Exception:
            pass
            
        self.after(VISION_LOOP_MS, self._process_frame)

    def on_closing(self) -> None:
        log.info("System shutting down...")
        try:
            if self.cap is not None and self.cap.isOpened(): self.cap.release()
            if self.drone is not None: self.drone.close()
        except Exception:
            pass
        self.destroy()

if __name__ == "__main__":
    app = GroundStationApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()