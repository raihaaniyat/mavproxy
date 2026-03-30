import math
import json
import os
import logging
from datetime import datetime

# ==============================================================================
# MODULE: Targeting Math & Operator Logic
# DESCRIPTION: Converts pixel coordinates to GPS coordinates, handles the CLI 
# operator prompts, and saves target data to a JSON database.
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
# You will need to adjust these based on your actual Cobra VTOL camera specs
IMG_WIDTH = 1920
IMG_HEIGHT = 1080
FOV_H_DEG = 60.0  # Horizontal Field of View in degrees
FOV_V_DEG = 45.0  # Vertical Field of View in degrees

DB_FILE = "detected_targets.json"

def calculate_target_gps(drone_lat, drone_lon, drone_alt_m, drone_hdg_deg, pixel_x, pixel_y):
    """
    Calculates the exact GPS coordinates of a pixel in a nadir-facing camera.
    """
    # 1. Find the center of the image
    cx = IMG_WIDTH / 2.0
    cy = IMG_HEIGHT / 2.0

    # 2. Calculate pixel offsets (Assuming Top of image is Forward)
    dx_px = pixel_x - cx
    dy_px = cy - pixel_y  # Inverted so 'up' on the image is positive Y (forward)

    # 3. Calculate Ground Sample Distance (Meters per pixel)
    fov_h_rad = math.radians(FOV_H_DEG)
    fov_v_rad = math.radians(FOV_V_DEG)
    
    ground_width_m = 2 * drone_alt_m * math.tan(fov_h_rad / 2)
    ground_height_m = 2 * drone_alt_m * math.tan(fov_v_rad / 2)
    
    gsd_x = ground_width_m / IMG_WIDTH
    gsd_y = ground_height_m / IMG_HEIGHT

    # 4. Convert pixel offset to meter offset relative to the drone
    offset_x_m = dx_px * gsd_x  # Right/Left
    offset_y_m = dy_px * gsd_y  # Forward/Backward

    # 5. Rotate the meter offsets by the drone's heading (yaw)
    hdg_rad = math.radians(drone_hdg_deg)
    
    # 2D Rotation Matrix to align with True North/East
    east_offset_m = offset_x_m * math.cos(hdg_rad) + offset_y_m * math.sin(hdg_rad)
    north_offset_m = -offset_x_m * math.sin(hdg_rad) + offset_y_m * math.cos(hdg_rad)

    # 6. Convert meter offsets to GPS Decimal Degrees
    # 1 degree of latitude is approx 111,320 meters
    lat_offset = north_offset_m / 111320.0
    lon_offset = east_offset_m / (111320.0 * math.cos(math.radians(drone_lat)))

    target_lat = drone_lat + lat_offset
    target_lon = drone_lon + lon_offset

    return round(target_lat, 8), round(target_lon, 8)

def save_target_to_db(target_data):
    """Saves the verified target to our JSON database."""
    db = []
    if os.path.exists(DB_FILE):
        with open(DB_FILE, 'r') as f:
            try:
                db = json.load(f)
            except json.JSONDecodeError:
                db = []
                
    # Assign a Human ID (e.g., Human 1, Human 2)
    human_id = f"Human {len(db) + 1}"
    target_data["id"] = human_id
    db.append(target_data)
    
    with open(DB_FILE, 'w') as f:
        json.dump(db, f, indent=4)
    log.info(f"Target saved to database as {human_id}.")

def process_detection(drone_telemetry, human_pixels):
    """
    Called when YOLO detects a human. Calculates GPS, prompts operator, 
    and handles the VTOL loiter logic.
    """
    log.warning(">>> HUMAN DETECTED IN CAMERA FOV <<<")
    
    lat, lon = calculate_target_gps(
        drone_lat=drone_telemetry['lat'],
        drone_lon=drone_telemetry['lon'],
        drone_alt_m=drone_telemetry['alt_m'],
        drone_hdg_deg=drone_telemetry['hdg'],
        pixel_x=human_pixels['x'],
        pixel_y=human_pixels['y']
    )
    
    log.info(f"Calculated Target Location: Lat {lat}, Lon {lon}")
    
    # --- OPERATOR CLI PROMPT ---
    # In a real integration, this is where you would send a MAVLink command
    # to put the VTOL into LOITER mode while waiting for the operator.
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
        
        # Save the data
        target_record = {
            "timestamp": datetime.now().isoformat(),
            "gps_lat": lat,
            "gps_lon": lon,
            "severity_index": severity,
            "status": "CRITICAL",
            # We will mock the video path for now
            "video_path": f"/vtol_data/videos/loiter_{datetime.now().strftime('%H%M%S')}.mp4" 
        }
        save_target_to_db(target_record)
        log.info("Critical target logged. Resuming surveillance path...")
    else:
        log.info("Target marked non-critical. VTOL exiting loiter, resuming path...")
        
    print("="*50 + "\n")

# ==============================================================================
# MAIN EXECUTION (Testing the logic)
# ==============================================================================
if __name__ == "__main__":
    # Mocking data from Phase 1 (Telemetry)
    # Using base coordinates around Gwalior for realistic testing
    mock_telemetry = {
        'lat': 26.218300,
        'lon': 78.182800,
        'alt_m': 50.0,   # 50 meters high
        'hdg': 90.0      # Facing East
    }
    
    # Mocking data from Phase 2 (Vision)
    # Let's pretend YOLO found a human in the bottom right quadrant of the screen
    mock_pixel_data = {
        'x': 1600,
        'y': 900
    }
    
    log.info("Starting Target Processing Test...")
    process_detection(mock_telemetry, mock_pixel_data)
