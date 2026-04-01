"""
================================================================================
 COBRA VTOL — SITL Connection Finder
 Tries every common Mission Planner / ArduPilot SITL connection string
 and tells you exactly which one is live so you can paste it into master_station.py
================================================================================
Run:  .\\myenv\\Scripts\\python.exe find_sitl.py
"""
from pymavlink import mavutil
import socket
import time

# All common SITL / Mission Planner output connection strings
CANDIDATES = [
    # ── UDP (most common for Mission Planner built-in SITL) ──────────────────
    ("udp:0.0.0.0:14550",   "UDP listen 14550  — standard MP/MAVProxy GCS output"),
    ("udp:0.0.0.0:14551",   "UDP listen 14551  — secondary GCS output"),
    ("udp:0.0.0.0:14552",   "UDP listen 14552  — tertiary GCS output"),
    # ── TCP CLIENT (connect-to) ───────────────────────────────────────────────
    ("tcp:127.0.0.1:5760",  "TCP 5760  — primary ArduPlane SITL server"),
    ("tcp:127.0.0.1:5762",  "TCP 5762  — secondary SITL client slot"),
    ("tcp:127.0.0.1:5763",  "TCP 5763  — tertiary SITL client slot"),
    # ── TCP SERVER (listen for incoming connection from MP) ───────────────────
    ("tcpin:0.0.0.0:5760",  "TCPin 5760 — listen for MP to connect in"),
    ("tcpin:0.0.0.0:14550", "TCPin 14550— listen for MP to connect in"),
]

TIMEOUT = 6   # seconds per candidate

print("=" * 65)
print("  COBRA VTOL — SITL Connection Finder")
print("=" * 65)
print(f"  Testing {len(CANDIDATES)} connection strings, {TIMEOUT}s each...")
print("  Make sure Mission Planner SITL (Simulation → Plane) is running!")
print("=" * 65)

# Also quick TCP port check first so we know what's actually open
print("\n[PRE-SCAN] TCP port reachability check (127.0.0.1):")
for port in [5760, 5761, 5762, 5763, 14550, 14551]:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1)
    try:
        result = s.connect_ex(("127.0.0.1", port))
        status = "✓ OPEN" if result == 0 else "✗ closed"
    except Exception:
        status = "✗ error"
    finally:
        s.close()
    print(f"    TCP 127.0.0.1:{port:5d}  →  {status}")

print()
working = []

for conn_str, description in CANDIDATES:
    print(f"\n[TRYING]  {conn_str}")
    print(f"          {description}")
    try:
        conn = mavutil.mavlink_connection(conn_str, source_system=254, retries=0)
        hb = conn.wait_heartbeat(timeout=TIMEOUT)
        if hb:
            mode = getattr(conn, 'flightmode', 'UNKNOWN')
            sysid = conn.target_system
            print(f"  ✓✓✓  HEARTBEAT RECEIVED!  sysid={sysid}  mode={mode}")
            working.append((conn_str, description, sysid, mode))
        else:
            print(f"  ✗    No heartbeat within {TIMEOUT}s (port may be open but idle)")
        conn.close()
    except ConnectionRefusedError:
        print(f"  ✗    Connection refused (port not listening)")
    except OSError as e:
        print(f"  ✗    OS error: {e}")
    except Exception as e:
        print(f"  ✗    {type(e).__name__}: {e}")

print("\n" + "=" * 65)
if working:
    print("  RESULTS — Live MAVLink connections found:")
    print("=" * 65)
    for conn_str, desc, sysid, mode in working:
        print(f"\n  ✓  {conn_str}")
        print(f"     {desc}")
        print(f"     sysid={sysid}  current_mode={mode}")
        print()
    best = working[0][0]
    print("  ► Paste this into master_station.py line 61:")
    print(f'\n     MAVLINK_CONN = "{best}"')
    print()
    if best.startswith("udp"):
        print("  NOTE: UDP 'listen' means your script waits for Mission Planner")
        print("  to SEND packets to you — no code changes needed beyond the")
        print("  connection string, pymavlink handles it automatically.")
else:
    print("  ✗  NO LIVE CONNECTION FOUND on any candidate port.")
    print("=" * 65)
    print("""
  TROUBLESHOOTING:
  1. Is Mission Planner open and Simulation running?
       Simulation menu → Plane → wait for HUD to show GPS fix
  2. Check Mission Planner's MAVLink output:
       Config → Planner → enable "Output1" → set to UDP → port 14550
  3. Or add a MAVProxy output:
       In Mission Planner console type:
         output add 127.0.0.1:14550
  4. Re-run this script after making changes.
""")

print("=" * 65)
