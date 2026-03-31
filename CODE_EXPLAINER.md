# MAVProxy (Cobra VTOL Hybrid Ground Station) - Code Explanation

This markdown explains the main code components in the repository and how they work.

## Overview

This repository implements a prototype Windows-based hybrid ground control station (GCS) for a VTOL UAV (Cobra). Design pattern:
- Telemetry input via HTTP (mock or real MAVLink feed)
- Local AI/human detection with YOLO
- Target geo-ref conversion and database write
- Tactical map generation via Folium
- MAVLink command output to a drone controller

## Key files

### `master_station.py`
- Central application and UI engine (CustomTkintergiot ).
- Contains:
  - Logger setup (`setup_logger`).
  - Camera discovery using DirectShow (`find_obs_camera`).
  - Tactical map generator (`generate_tactical_map`) with Folium.
  - GroundStationApp class combining:
    - YOLOv8 model load
    - Telemetry polling
    - Real-time detection and plotting
    - Operator controls and MAVLink actions
  - Handles file paths:
    - `detected_targets.json`, `tactical_map.html`, logs
  - Fallbacks for missing modules: `pymavlink`, `ultralytics`, `folium`.

### `telemetry_client.py`
- HTTP client for periodic telemetry fetch.
- Fetches from `http://127.0.0.1:56781/mavlink/` by default.
- `fetch_telemetry()` returns JSON or logs errors.
- `parse_and_display()` extracts key data from MAVLink packages:
  - `GLOBAL_POSITION_INT` for lat/lon/alt/heading
  - `HEARTBEAT` for flight mode
- Standalone loop in `__main__` showing periodic CLI updates.

### `map_module.py`
- Reads `detected_targets.json` and renders interactive Folium map to `tactical_map.html`.
- Calculations:
  - center coordinates through average lat/lon
  - marker severity color mapping
  - Esri satellite tiles + Leaflet controls
- GIS output for operators.

### `operator_module.py` (and `operator_module2.py` similar)
- Target geometry and human-in-loop operator handling.
- `calculate_target_gps(...)` maps pixel coordinates in camera frame to world GPS using:
  - image center offsets
  - ground sample distance (GSD) from altitude + FOV
  - heading rotation
  - lat/lon offset conversion
- `save_target_to_db(...)` writes entries to JSON store.
- `process_detection(...)` prompts operator for criticality and severity.
- sample `__main__` for manual testing.

## Data files

- `detected_targets.json`: JSON list of tracked targets.
  - Example fields: `id`, `timestamp`, `gps_lat`, `gps_lon`, `severity_index`, `status`, `video_path`
- `tactical_map.html`: generated interactive map output.

## Launch workflow

1. Ensure Python environment has dependencies: `customtkinter`, `pymavlink`, `ultralytics`, `folium`, `requests`, `opencv-python`, `Pillow`.
2. Start mock telemetry server (if needed) via `mock_telem_server.py`.
3. Run `telemetry_client.py` for inbound MAVLink data.
4. Run `master_station.py` to start UI and detection pipeline.
5. Generated outputs:
   - `system_logs/*.log` for runtime diagnostics
   - `detected_targets.json` for target record
   - `tactical_map.html` for visual map

## Branch and merge status

- `master` and `main` are aligned in git.
- Remote repository: `https://github.com/raihaaniyat/mavproxy.git`.

## Notes for contributors

- `master_station.py` contains the greatest number of responsibilities; consider splitting into separate modules (UI, telemetry, vision, mission control).
- Unit tests are currently minimal; add tests for `calculate_target_gps`, JSON DB roundtrip, and no-dependency fallback behavior.
- For hardening, add exception handlers in all I/O and network functions.
