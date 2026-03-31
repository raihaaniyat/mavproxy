@echo off
echo =======================================================
echo  COBRA VTOL Ground Station  [Native Mission Planner]
echo =======================================================
echo.
echo STEP 1: Make sure Mission Planner is already running
echo         with Simulation -^> Plane started.
echo.
echo STEP 2: Launching Telemetry Bridge + Ground Station UI...
echo.

REM 1. Start the Telemetry Bridge (reads from Mission Planner TCP 5763)
start "VTOL HTTP Bridge (TCP 5763)" cmd /k ".\myenv\Scripts\activate && python sitl_telem_bridge.py"

timeout /t 2 /nobreak >nul

REM 2. Start the Main Ground Station UI (commands to TCP 5762)
start "VTOL Ground Station UI" cmd /k ".\myenv\Scripts\activate && python master_station.py"

echo.
echo All modules launched in virtual environment!
echo.
echo ARCHITECTURE:
echo   Mission Planner SITL ^<--^> TCP 5762 ^<--^> master_station.py     (COMMANDS)
echo   Mission Planner SITL ^<--^> TCP 5763 ^<--^> sitl_telem_bridge.py  (TELEMETRY)
echo                                             HTTP:56781 ^<--^> master_station.py
echo.
pause
