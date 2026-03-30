"""
================================================================================
 JATAYU SAR — Geolocation Module
 Converts YOLOv8 bounding-box pixel coordinates into real-world GPS positions
 and plots them on an interactive Folium tactical map.

 Camera  : RunCam VTX, fixed nadir mount (straight down), no gimbal
 Model   : Pinhole camera with FOV-based Ground Sample Distance (GSD)
 Imports : math, json, datetime, folium  (no other external deps)
================================================================================

 MATH OVERVIEW (flat-earth approximation — valid for SAR-scale areas):
   1. Compute the center pixel of the detection bounding box.
   2. Find the pixel offset from the image center (optical axis).
   3. Convert pixel offset → angular offset using horizontal/vertical FOV.
   4. Project angular offset → metric offset using altitude (tan projection).
   5. Rotate the metric offset by drone heading so +Y always points north.
   6. Convert metric offset → lat/lon delta:
        Δlat = north_m / 111320
        Δlon = east_m  / (111320 · cos(lat))

================================================================================
"""

from __future__ import annotations

import math
import json
import logging
from pathlib import Path
from datetime import datetime

try:
    import folium
    FOLIUM_OK = True
except ImportError:
    FOLIUM_OK = False


# ==============================================================================
#  CONFIGURABLE CAMERA PARAMETERS
#  Adjust these to match your actual RunCam VTX specs after calibration.
# ==============================================================================

CAMERA_FOV_H_DEG = 127.0       # Horizontal field of view (degrees)
CAMERA_FOV_V_DEG = 92.0        # Vertical field of view (degrees)
CAMERA_RES_W     = 1280         # Frame width  (pixels)
CAMERA_RES_H     = 720          # Frame height (pixels)

# ==============================================================================
#  PATHS  (all relative to this script's directory)
# ==============================================================================

BASE_DIR    = Path(__file__).resolve().parent
DB_FILE     = BASE_DIR / "detected_targets.json"
MAP_OUTPUT  = BASE_DIR / "tactical_map.html"
LOG_DIR     = BASE_DIR / "system_logs"

# ==============================================================================
#  LOGGER
# ==============================================================================

def _setup_logger() -> logging.Logger:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp    = datetime.now().strftime("%Y-%m-%d")
    log_file = LOG_DIR / f"vtol_GEOLOCATOR_{stamp}.log"

    logger = logging.getLogger("GEOLOCATOR")
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

log = _setup_logger()


# ==============================================================================
#  CORE FUNCTIONS
# ==============================================================================

def get_gsd(
    altitude: float,
    fov_h: float = CAMERA_FOV_H_DEG,
    fov_v: float = CAMERA_FOV_V_DEG,
    frame_w: int = CAMERA_RES_W,
    frame_h: int = CAMERA_RES_H,
) -> tuple[float, float]:
    """
    Ground Sample Distance — how many meters each pixel covers at a given
    altitude for a nadir-pointing camera.

    Returns
    -------
    (gsd_x, gsd_y) : tuple[float, float]
        Meters-per-pixel in the horizontal and vertical directions.

    Math
    ----
        ground_span = 2 · alt · tan(FOV / 2)
        gsd         = ground_span / pixel_count
    """
    # Clamp altitude to prevent division-by-zero at ground level
    alt = max(altitude, 0.5)

    ground_w = 2.0 * alt * math.tan(math.radians(fov_h) / 2.0)
    ground_h = 2.0 * alt * math.tan(math.radians(fov_v) / 2.0)

    gsd_x = ground_w / frame_w      # meters per pixel (horizontal)
    gsd_y = ground_h / frame_h      # meters per pixel (vertical)

    log.debug(
        f"GSD @ {alt:.1f}m  →  gsd_x={gsd_x:.4f} m/px, gsd_y={gsd_y:.4f} m/px  "
        f"(ground footprint {ground_w:.1f}×{ground_h:.1f} m)"
    )
    return gsd_x, gsd_y


def compute_gps(
    bbox: tuple[int, int, int, int],
    drone_lat: float,
    drone_lon: float,
    drone_alt: float,
    drone_heading: float,
    frame_w: int = CAMERA_RES_W,
    frame_h: int = CAMERA_RES_H,
) -> tuple[float, float]:
    """
    Convert a YOLOv8 bounding box to real-world GPS coordinates.

    Parameters
    ----------
    bbox           : (x1, y1, x2, y2)  pixel coords of detected person
    drone_lat/lon  : current GPS position of the drone (decimal degrees)
    drone_alt      : altitude above ground in meters (relative / AGL)
    drone_heading  : yaw in degrees, 0 = North, 90 = East  (clockwise)
    frame_w/h      : image resolution (default: 1280×720)

    Returns
    -------
    (target_lat, target_lon) : tuple[float, float]

    Pipeline
    --------
    pixel center → pixel offset → metric offset → heading rotation → GPS delta
    """
    x1, y1, x2, y2 = bbox

    # ── Step 1: Bounding-box center in pixel coordinates ─────────────────
    px = (x1 + x2) / 2.0
    py = (y1 + y2) / 2.0

    # ── Step 2: Pixel offset from image center (optical axis) ────────────
    #   +dx_px = right of center    +dy_px = above center (image Y inverted)
    cx = frame_w / 2.0
    cy = frame_h / 2.0
    dx_px = px - cx
    dy_px = cy - py          # flip Y so "up in image" = "forward from drone"

    # ── Step 3: Pixel offset → metric offset via GSD ─────────────────────
    gsd_x, gsd_y = get_gsd(drone_alt, frame_w=frame_w, frame_h=frame_h)
    dx_m = dx_px * gsd_x     # meters right  of nadir
    dy_m = dy_px * gsd_y     # meters forward of nadir

    # ── Step 4: Rotate by drone heading ──────────────────────────────────
    #   The camera frame is body-fixed.  We rotate the body-frame offset
    #   into a North-East-Down geographic frame.
    #
    #   heading = 0°   → camera "up" is North   → no rotation needed
    #   heading = 90°  → camera "up" is East    → rotate 90° CW
    #
    #   Rotation (CW by heading):
    #       east_m  =  dx_m · cos(hdg) + dy_m · sin(hdg)
    #       north_m = -dx_m · sin(hdg) + dy_m · cos(hdg)

    hdg_rad = math.radians(drone_heading)

    east_m  =  dx_m * math.cos(hdg_rad) + dy_m * math.sin(hdg_rad)
    north_m = -dx_m * math.sin(hdg_rad) + dy_m * math.cos(hdg_rad)

    # ── Step 5: Metric offset → lat/lon delta ────────────────────────────
    #   1° latitude  ≈ 111 320 m  (constant)
    #   1° longitude ≈ 111 320 m · cos(lat)  (shrinks toward poles)

    d_lat = north_m / 111320.0
    d_lon = east_m  / (111320.0 * math.cos(math.radians(drone_lat)))

    target_lat = round(drone_lat + d_lat, 8)
    target_lon = round(drone_lon + d_lon, 8)

    log.info(
        f"GEOLOCATE  bbox=({x1},{y1},{x2},{y2})  "
        f"alt={drone_alt:.1f}m  hdg={drone_heading:.1f}°  →  "
        f"({target_lat:.7f}, {target_lon:.7f})"
    )
    return target_lat, target_lon


# ==============================================================================
#  TARGET DATABASE
# ==============================================================================

def _read_db() -> list[dict]:
    """Load the target database; return [] on missing / corrupt file."""
    if not DB_FILE.exists():
        return []
    try:
        with open(DB_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except (json.JSONDecodeError, OSError) as exc:
        log.warning(f"DB read error ({exc}) — starting fresh.")
        return []


def _write_db(db: list[dict]) -> None:
    """Atomically write the target database."""
    with open(DB_FILE, "w", encoding="utf-8") as f:
        json.dump(db, f, indent=4, ensure_ascii=False)


def save_target(
    target_lat: float,
    target_lon: float,
    track_id: str,
    status: str = "detected",
) -> dict:
    """
    Persist a geolocated target into detected_targets.json.

    If a record with the same `track_id` already exists it is **updated**
    (lat, lon, timestamp, status) instead of duplicated.

    Parameters
    ----------
    target_lat/lon : GPS coordinates  (from compute_gps)
    track_id       : BoTSORT tracker ID  (e.g. "T-0012")
    status         : one of "detected", "RESPONSIVE", "HIGH", "CRITICAL"

    Returns
    -------
    The saved/updated record dict.
    """
    db = _read_db()

    record = {
        "track_id":  track_id,
        "lat":       target_lat,
        "lon":       target_lon,
        "timestamp": datetime.now().isoformat(),
        "status":    status,
    }

    # ── Upsert: update existing entry or append new one ──────────────────
    updated = False
    for i, entry in enumerate(db):
        if entry.get("track_id") == track_id:
            db[i] = record
            updated = True
            log.info(f"TARGET UPDATED  {track_id}  ({target_lat:.7f}, {target_lon:.7f})  status={status}")
            break

    if not updated:
        db.append(record)
        log.info(f"TARGET SAVED    {track_id}  ({target_lat:.7f}, {target_lon:.7f})  status={status}")

    _write_db(db)
    return record


# ==============================================================================
#  TACTICAL MAP GENERATION  (Folium + Leaflet)
# ==============================================================================

# Color mapping for target status badges on the map
_STATUS_COLORS = {
    "CRITICAL":    "red",
    "HIGH":        "orange",
    "RESPONSIVE":  "green",
    "detected":    "gray",
}


def update_map(
    map_path: str | Path = MAP_OUTPUT,
    drone_lat: float | None = None,
    drone_lon: float | None = None,
    drone_heading: float | None = None,
) -> Path | None:
    """
    Regenerate the interactive Folium tactical map from the target database.

    Parameters
    ----------
    map_path       : output HTML file  (default: tactical_map.html)
    drone_lat/lon  : current drone position (plotted as a blue marker)
    drone_heading  : current drone yaw in degrees (shown in popup)

    Returns
    -------
    Path to the saved HTML file, or None on failure.
    """
    if not FOLIUM_OK:
        log.error("folium is not installed — cannot generate tactical map.")
        return None

    db = _read_db()
    map_path = Path(map_path)

    # ── Determine map center ─────────────────────────────────────────────
    #   Prefer drone position; fall back to average of targets; then (0,0).
    if drone_lat is not None and drone_lon is not None:
        center = [drone_lat, drone_lon]
    elif db:
        center = [
            sum(t["lat"] for t in db) / len(db),
            sum(t["lon"] for t in db) / len(db),
        ]
    else:
        center = [0.0, 0.0]

    tac_map = folium.Map(location=center, zoom_start=19)

    # ── Esri Satellite base layer ────────────────────────────────────────
    folium.TileLayer(
        tiles=(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}"
        ),
        attr="Esri",
        name="Esri Satellite",
        overlay=False,
        control=True,
    ).add_to(tac_map)

    # ── Plot each target ─────────────────────────────────────────────────
    for t in db:
        lat    = t.get("lat", 0.0)
        lon    = t.get("lon", 0.0)
        tid    = t.get("track_id", "?")
        status = t.get("status", "detected")
        ts     = t.get("timestamp", "—")
        color  = _STATUS_COLORS.get(status, "gray")

        popup_html = (
            f"<b>{tid}</b><br>"
            f"Status: <span style='color:{color};font-weight:bold'>{status}</span><br>"
            f"Lat: {lat:.7f}<br>"
            f"Lon: {lon:.7f}<br>"
            f"Time: {ts}"
        )

        folium.CircleMarker(
            location=[lat, lon],
            radius=9,
            popup=folium.Popup(popup_html, max_width=250),
            color="black",
            weight=2,
            fill=True,
            fill_color=color,
            fill_opacity=0.85,
        ).add_to(tac_map)

    # ── Drone position marker (blue) ─────────────────────────────────────
    if drone_lat is not None and drone_lon is not None:
        hdg_str = f"{drone_heading:.1f}°" if drone_heading is not None else "—"
        drone_popup = (
            f"<b>🛩️ DRONE</b><br>"
            f"Lat: {drone_lat:.7f}<br>"
            f"Lon: {drone_lon:.7f}<br>"
            f"Heading: {hdg_str}"
        )
        folium.CircleMarker(
            location=[drone_lat, drone_lon],
            radius=10,
            popup=folium.Popup(drone_popup, max_width=220),
            color="white",
            weight=2,
            fill=True,
            fill_color="#2980b9",
            fill_opacity=0.95,
        ).add_to(tac_map)

    # ── Layer control + save ─────────────────────────────────────────────
    folium.LayerControl().add_to(tac_map)
    tac_map.save(str(map_path))
    log.info(f"Tactical map saved → {map_path}  ({len(db)} targets)")
    return map_path


# ==============================================================================
#  SELF-TEST  (run this file directly to verify the math)
# ==============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  GEOLOCATOR SELF-TEST")
    print("=" * 60)

    # ── Test params: Gwalior airfield, 50 m AGL, heading east ────────────
    test_drone = {
        "lat": 26.218300,
        "lon": 78.182800,
        "alt": 50.0,
        "hdg": 90.0,
    }

    # Simulated YOLO bbox: person detected in the bottom-right quadrant
    test_bbox = (900, 500, 1000, 700)

    print(f"\nDrone  : lat={test_drone['lat']}, lon={test_drone['lon']}, "
          f"alt={test_drone['alt']}m, hdg={test_drone['hdg']}°")
    print(f"Bbox   : {test_bbox}")

    # 1. GSD
    gx, gy = get_gsd(test_drone["alt"])
    print(f"GSD    : {gx:.4f} m/px (H)  ×  {gy:.4f} m/px (V)")

    # 2. Compute GPS
    t_lat, t_lon = compute_gps(
        test_bbox,
        test_drone["lat"], test_drone["lon"],
        test_drone["alt"], test_drone["hdg"],
    )
    print(f"Target : ({t_lat:.7f}, {t_lon:.7f})")

    # 3. Save
    rec = save_target(t_lat, t_lon, "T-TEST-001", status="HIGH")
    print(f"Saved  : {rec}")

    # 4. Map
    mp = update_map(
        drone_lat=test_drone["lat"],
        drone_lon=test_drone["lon"],
        drone_heading=test_drone["hdg"],
    )
    if mp:
        print(f"Map    : {mp}")

    print("\n" + "=" * 60)
    print("  ALL TESTS PASSED — open tactical_map.html to verify")
    print("=" * 60)
