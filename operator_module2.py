import math
import json
import os
import logging
import requests
from datetime import datetime

# ==============================================================================
# MODULE: Targeting Math & Operator Logic (Live Telemetry Version)
# ==============================================================================

def setup_logger(module_name):
    if not os.path.exists('system_logs'):
        os.makedirs('system_logs')
    log_filename = f"system_logs/vtol_{module_name}_{datetime.now().strftime('%Y-%m-%d')}.log"
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    file_handler = logging.FileHandler(log_filename)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - [%(levelname)s] - %(message)s'))
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    return logger

log = setup_logger("OPERATOR_LOGIC")

# --- CAMERA CONFIGURATION ---
IMG_WIDTH = 1920
IMG_HEIGHT = 1080
FOV_H_DEG = 60.0  
FOV_V_DEG = 45.0  

DB_FILE = "detected_targets.json"
TELEMETRY_URL = "http://127.0.0.1:56781/mavlink/"

def get_live_telemetry():
    """Fetches the live VTOL position from the MAVLink server."""
    try:
        response = requests.get(TELEMETRY_URL, timeout=2)
        if response.status_code == 200:
            data = response.json()
            pos = data.get("GLOBAL_POSITION_INT", {})
            
            return {
                'lat': pos.get("lat", 0) / 1e7,
                'lon': pos.get("lon", 0) / 1e7,
                'alt_m': pos.get("relative_alt", 0) / 1000.0,
                'hdg': pos.get("hdg", 0) / 100.0
            }
        else:
            log.error(f"Telemetry server returned status: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        log.error(f"Could not connect to live telemetry. Is the mock server running? Error: {e}")
        return None

def calculate_target_gps(drone_lat, drone_lon, drone_alt_m, drone_hdg_deg, pixel_x, pixel_y):
    """Calculates the exact GPS coordinates of a pixel in a nadir-facing camera."""
    cx = IMG_WIDTH / 2.0
    cy = IMG_HEIGHT / 2.0

    dx_px = pixel_x - cx
    dy_px = cy - pixel_y  

    fov_h_rad = math.radians(FOV_H_DEG)
    fov_v_rad = math.radians(FOV_V_DEG)
    
    ground_width_m = 2 * drone_alt_m * math.tan(fov_h_rad / 2)
    ground_height_m = 2 * drone_alt_m * math.tan(fov_v_rad / 2)
    
    gsd_x = ground_width_m / IMG_WIDTH
    gsd_y = ground_height_m / IMG_HEIGHT

    offset_x_m = dx_px * gsd_x  
    offset_y_m = dy_px * gsd_y  

    hdg_rad = math.radians(drone_hdg_deg)
    
    east_offset_m = offset_x_m * math.cos(hdg_rad) + offset_y_m * math.sin(hdg_rad)
    north_offset_m = -offset_x_m * math.sin(hdg_rad) + offset_y_m * math.cos(hdg_rad)

    lat_offset = north_offset_m / 111320.0
    lon_offset = east_offset_m / (111320.0 * math.cos(math.radians(drone_lat)))

    return round(drone_lat + lat_offset, 8), round(drone_lon + lon_offset, 8)

def save_target_to_db(target_data):
    """Saves the verified target to our JSON database."""
    db = []
    if os.path.exists(DB_FILE):
        with open(DB_FILE, 'r') as f:
            try:
                db = json.load(f)
            except json.JSONDecodeError:
                db = []
                
    human_id = f"Human {len(db) + 1}"
    target_data["id"] = human_id
    db.append(target_data)
    
    with open(DB_FILE, 'w') as f:
        json.dump(db, f, indent=4)
    log.info(f"Target saved to database as {human_id}.")

def process_detection(human_pixels):
    """
    Pulls live telemetry, calculates target GPS, and handles operator CLI.
    """
    log.warning(">>> HUMAN DETECTED: PULLING LIVE TELEMETRY <<<")
    
    drone_telemetry = get_live_telemetry()
    if not drone_telemetry:
        log.error("Cannot process target without active telemetry stream. Aborting.")
        return

    log.info(f"Drone Live Pos - Lat: {drone_telemetry['lat']}, Lon: {drone_telemetry['lon']}, Alt: {drone_telemetry['alt_m']}m, Hdg: {drone_telemetry['hdg']}°")

    lat, lon = calculate_target_gps(
        drone_lat=drone_telemetry['lat'],
        drone_lon=drone_telemetry['lon'],
        drone_alt_m=drone_telemetry['alt_m'],
        drone_hdg_deg=drone_telemetry['hdg'],
        pixel_x=human_pixels['x'],
        pixel_y=human_pixels['y']
    )
    
    log.info(f"Calculated Target Location: Lat {lat}, Lon {lon}")
    
    print("\n" + "="*50)
    print("VTOL entering LOITER mode. Video recording started.")
    print("="*50)
    
    while True:
        is_critical = input("Is this human critical? (y/n): ").strip().lower()
        if is_critical in ['y', 'n']:
            break
        print("Invalid input. Please enter 'y' or 'n'.")

    if is_critical == 'y':
        while True:
            try:
                severity = int(input("Enter Severity Index (1-10): ").strip())
                if 1 <= severity <= 10:
                    break
                print("Index must be between 1 and 10.")
            except ValueError:
                print("Please enter a valid number.")
        
        target_record = {
            "timestamp": datetime.now().isoformat(),
            "gps_lat": lat,
            "gps_lon": lon,
            "severity_index": severity,
            "status": "CRITICAL",
            "video_path": f"/vtol_data/videos/loiter_{datetime.now().strftime('%H%M%S')}.mp4" 
        }
        save_target_to_db(target_record)
        log.info("Critical target logged. Resuming surveillance path...")
    else:
        log.info("Target marked non-critical. VTOL exiting loiter, resuming path...")
        
    print("="*50 + "\n")

# ==============================================================================
# MAIN EXECUTION (Testing with live telemetry)
# ==============================================================================
if __name__ == "__main__":
    # Mocking ONLY the camera pixel data. 
    # The telemetry is now fetched live from the server.
    mock_pixel_data = {
        'x': 1600,
        'y': 900
    }
    
    log.info("Starting Target Processing Test...")
    # Make sure your mock_telemetry_server.py is running in another terminal!
    process_detection(mock_pixel_data)
