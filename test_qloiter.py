"""
COBRA VTOL — QLOITER Command Diagnostic
Run this in the venv while Mission Planner SITL is active to verify
the QLOITER command is being accepted by the flight controller.
"""
from pymavlink import mavutil
import time

CONN = "tcp:127.0.0.1:5762"
QLOITER_MODE = 19

print(f"\n[1] Connecting to {CONN}...")
conn = mavutil.mavlink_connection(CONN, source_system=254)

print("[2] Waiting for heartbeat (up to 15s)...")
hb = conn.wait_heartbeat(timeout=15)
if not hb:
    print("ERROR: No heartbeat. Is Mission Planner SITL running?")
    exit(1)

print(f"    ✓ Heartbeat! target_system={conn.target_system}  target_component={conn.target_component}")
print(f"    ✓ Current mode: {conn.flightmode}")

# Check Q_ENABLE
print(f"\n[3] Checking Q_ENABLE parameter...")
conn.mav.param_request_read_send(conn.target_system, conn.target_component, b'Q_ENABLE', -1)
param = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
if param:
    val = int(param.param_value)
    print(f"    Q_ENABLE = {val}")
    if val == 0:
        print("    ✗ Q_ENABLE is 0! Go to Config -> Full Parameter List -> set Q_ENABLE=1 and reboot SITL.")
        conn.close()
        exit(1)
    else:
        print("    ✓ QuadPlane is enabled.")
else:
    print("    Could not read Q_ENABLE. Proceeding anyway...")

# Step 1: Transition to LOITER first
print(f"\n[4] Switching to LOITER to decelerate before QLOITER...")
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    12, 0, 0, 0, 0, 0  # mode 12 = LOITER
)
time.sleep(3)

# Step 2: Send QLOITER
print(f"[5] Sending QLOITER (mode {QLOITER_MODE})...")
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    QLOITER_MODE, 0, 0, 0, 0, 0
)

print("[6] Waiting for COMMAND_ACK (5s)...")
ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack:
    results = {0: "✓ ACCEPTED", 1: "✗ TEMPORARILY REJECTED", 2: "✗ DENIED", 3: "✗ UNSUPPORTED", 4: "✗ FAILED"}
    result_str = results.get(ack.result, f"UNKNOWN ({ack.result})")
    print(f"    ACK result: {result_str}")
    if ack.result == 0:
        print("\n  ✓ SUCCESS! QLOITER accepted. Check Mission Planner — plane should be hovering.")
    elif ack.result == 4:
        print("\n  DIAGNOSIS: FAILED even after LOITER transition.")
        print("  → Set Q_ENABLE=1 in Full Parameter List and restart SITL.")
        print("  → Also ensure the Simulation model is QuadPlane, not standard Plane.")
else:
    print("    No ACK received (timeout).")

conn.close()
print("\nDiagnostic complete.")
