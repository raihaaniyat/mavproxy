import json
import os
import folium
import logging
from datetime import datetime

# ==============================================================================
# MODULE: Interactive Tactical Map (Folium)
# DESCRIPTION: Reads the detected_targets.json database and plots them on a 
# real-world interactive satellite map.
# ==============================================================================

def setup_logger(module_name):
    if not os.path.exists('system_logs'):
        os.makedirs('system_logs')
    log_filename = f"system_logs/vtol_{module_name}_{datetime.now().strftime('%Y-%m-%d')}.log"
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.INFO)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
    logger.addHandler(console_handler)
    return logger

log = setup_logger("MAP_MODULE")
DB_FILE = "detected_targets.json"
MAP_OUTPUT = "tactical_map.html"

def get_severity_color(severity):
    """Returns a color string compatible with Folium."""
    if severity <= 3: return 'green'      
    elif severity <= 7: return 'orange'   
    else: return 'red'                    

def generate_real_map():
    if not os.path.exists(DB_FILE):
        log.error(f"Database '{DB_FILE}' not found. Run the detection logic first.")
        return

    with open(DB_FILE, 'r') as f:
        targets = json.load(f)

    if not targets:
        log.warning("Database is empty. No targets to map.")
        return

    log.info(f"Loaded {len(targets)} targets. Building interactive satellite map...")

    # Calculate the center of the map based on all targets
    avg_lat = sum(t['gps_lat'] for t in targets) / len(targets)
    avg_lon = sum(t['gps_lon'] for t in targets) / len(targets)

    # Initialize the map centered on our targets
    tactical_map = folium.Map(location=[avg_lat, avg_lon], zoom_start=19)

    # Add a high-resolution Satellite Tile Layer (Esri World Imagery)
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='Esri',
        name='Esri Satellite',
        overlay=False,
        control=True
    ).add_to(tactical_map)

    # Plot each target
    for t in targets:
        lat = t['gps_lat']
        lon = t['gps_lon']
        severity = t.get('severity_index', 1)
        color = get_severity_color(severity)
        human_id = t.get('id', 'Unknown')
        
        # Build a popup that shows info when the operator clicks the dot
        popup_html = f"<b>{human_id}</b><br>Severity: {severity}<br>Lat: {lat:.6f}<br>Lon: {lon:.6f}"
        
        # Add a colored circle marker
        folium.CircleMarker(
            location=[lat, lon],
            radius=8,
            popup=folium.Popup(popup_html, max_width=200),
            color='black',
            weight=1,
            fill=True,
            fill_color=color,
            fill_opacity=0.9
        ).add_to(tactical_map)

    # Add a layer control so the operator can toggle between map styles
    folium.LayerControl().add_to(tactical_map)

    # Save the HTML file
    tactical_map.save(MAP_OUTPUT)
    log.info(f"Map successfully generated! Open '{MAP_OUTPUT}' in your web browser.")

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
if __name__ == "__main__":
    generate_real_map()
