import requests
import time
import logging

# ==============================================================================
# MODULE: Telemetry Client
# DESCRIPTION: Fetches data from the drone (or our mock server), parses it,
# and extracts the exact GPS and attitude data needed for human targeting.
# ==============================================================================

# 1. Setup basic terminal logging (We will keep it simple for the CLI)
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
log = logging.getLogger("TELEMETRY_CLIENT")

# The URL of our mock MAVLink server (Phase 1 testing)
MOCK_SERVER_URL = "http://127.0.0.1:56781/mavlink/"

def fetch_telemetry():
    """
    Fetches the latest telemetry packet.
    LATER: We will replace this requests.get() with pymavlink.mavrecv() 
    when connecting to the real Cobra VTOL.
    """
    try:
        response = requests.get(MOCK_SERVER_URL, timeout=2)
        if response.status_code == 200:
            return response.json()
        else:
            log.warning(f"Server returned status code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        log.error(f"Connection failed. Is the mock server running? Error: {e}")
        return None

def parse_and_display(mavlink_data):
    """
    Extracts the exact variables we need for the Orthomosaic and YOLO math.
    MAVLink coordinates are scaled by 1e7, so we divide by 10,000,000.
    """
    if not mavlink_data:
        return

    try:
        # Extract Global Position data
        pos_data = mavlink_data.get("GLOBAL_POSITION_INT", {})
        
        # Convert scaled integers back to standard decimal degrees
        lat = pos_data.get("lat", 0) / 1e7
        lon = pos_data.get("lon", 0) / 1e7
        
        # Convert millimeters to meters
        alt_meters = pos_data.get("relative_alt", 0) / 1000.0
        
        # Convert centidegrees to standard degrees (0-360)
        heading = pos_data.get("hdg", 0) / 100.0

        # Extract Flight Mode / Status
        heartbeat = mavlink_data.get("HEARTBEAT", {})
        mode = heartbeat.get("custom_mode", "UNKNOWN")

        # Print cleanly to the terminal
        log.info(f"Mode: {mode} | Lat: {lat:.6f}, Lon: {lon:.6f} | Alt: {alt_meters:.2f}m | Hdg: {heading:.1f}°")

    except KeyError as e:
        log.error(f"Malformed data received, missing key: {e}")

# ==============================================================================
# MAIN EXECUTION LOOP
# ==============================================================================
if __name__ == "__main__":
    log.info("Starting Telemetry Client. Attempting to connect to VTOL data stream...")
    
    try:
        # This is our main flight loop. It runs continuously.
        while True:
            # 1. Fetch the data
            raw_data = fetch_telemetry()
            
            # 2. Parse and print the data
            if raw_data:
                parse_and_display(raw_data)
            
            # 3. Sleep to simulate a 1Hz (1 update per second) loop
            # Real MAVLink streams usually come in at 10Hz or faster, but 1Hz is good for CLI testing.
            time.sleep(1)
            
    except KeyboardInterrupt:
        log.info("Client stopped by operator (Ctrl+C).")
