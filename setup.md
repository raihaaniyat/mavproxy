# Cobra VTOL — Hybrid Ground Station Setup Guide

This document outlines the technical stack and setup requirements for the **Cobra VTOL Ground Station** (Windows Edition).

## 🚀 Tech Stack Overview

### Core Language
*   **Python 3.10+**: The entire backend and logic are written in Python.

### GUI & Frontend
*   **CustomTkinter**: A modern, dark-themed UI wrapper for standard Tkinter.
*   **Pillow (PIL)**: Used for high-DPI image rendering and manipulation.
*   **Leaflet.js**: The engine behind the interactive tactical map.

### Computer Vision & AI
*   **Ultralytics YOLOv8**: Real-time human detection and coordinate projection.
*   **OpenCV (cv2)**: Video stream capture (via DirectShow) and frame annotation.

### Drones & MAVLink
*   **PyMAVLink**: Python implementation of the MAVLink protocol for drone commanding.
*   **MAVLink Protocol**: Standard communication protocol for unmanned vehicles.
*   **SITL (Software In The Loop)**: Used for simulating drone flight and telemetry.

### Mapping & GIS
*   **Folium**: A Python library that generates Leaflet-based HTML maps.
*   **Esri World Imagery**: Provides high-resolution satellite tile layers.

### Networking & Telemetry
*   **Requests**: Handles HTTP GET polling for telemetry data from the drone/server.
*   **HTTP/JSON REST API**: The bridge for telemetry data exchange.

---

## 🛠️ Installation & Dependencies

To set up the environment, you will need the following Python libraries:

```powershell
pip install customtkinter opencv-python ultralytics pymavlink folium requests pillow
```

## 📂 Project Structure

| File | Description |
| :--- | :--- |
| **`master_station.py`** | The main dashboard combining Video, AI, and Telemetry. |
| **`map_module.py`** | Logic for generating the interactive `tactical_map.html`. |
| **`telemetry_client.py`** | Standalone script to test telemetry polling. |
| **`mock_telem_server.py`** | Simulates an ArduPilot telemetry stream for local testing. |
| **`detected_targets.json`** | Persistent database for all saved human target coordinates. |

---

## 🚦 How to Run (Testing Flow)

1.  **Start Simulation**: Ensure ArduPilot SITL is running and serving telemetry.
2.  **Mock Telemetry (Optional)**: If SITL is unavailable, run `python mock_telem_server.py`.
3.  **Virtual Camera**: Start **OBS Virtual Camera** (or a real webcam) to provide the video feed.
4.  **Launch Station**: Run `python master_station.py`.

## ⚠️ Critical Setup Notes
*   **Windows DirectShow**: The system uses `cv2.CAP_DSHOW` to find cameras. Ensure your camera is indexed correctly in `master_station.py`.
*   **COM Ports**: Update `MAVLINK_CONN` in `master_station.py` to match your telemetry radio or UDP port.
*   **DPI Scaling**: CustomTkinter handles Windows "High DPI" scaling automatically to prevent blur.
