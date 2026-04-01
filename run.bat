@echo off
echo =======================================================
echo  COBRA VTOL Ground Station  [Mission Planner SITL]
echo =======================================================
echo.
echo PRE-FLIGHT CHECKLIST:
echo   1. Open Mission Planner
echo   2. Go to: Simulation -^> Plane  (or QuadPlane)
echo   3. Wait until the HUD shows GPS and a heartbeat
echo   4. THEN press any key here to connect...
echo.
echo NOTE: master_station.py connects to TCP 5762 directly.
echo       The HTTP bridge (sitl_telem_bridge.py) is NOT needed anymore.
echo.
pause

REM Launch the Ground Station — connects directly to Mission Planner on TCP 5762
start "VTOL Ground Station" cmd /k ".\myenv\Scripts\activate && python master_station.py"

echo.
echo Ground Station launched!
echo.
echo ARCHITECTURE:
echo   Mission Planner SITL ^<--^> TCP 5762 ^<--^> master_station.py (telemetry + commands)
echo.
echo If connection fails on 5762, try changing MAVLINK_CONN in master_station.py to:
echo   tcp:127.0.0.1:5763
echo.
pause
