#!/usr/bin/env python3
from pymavlink import mavutil

PORT = "/dev/ttyTHS1"  # or /dev/ttyACM0
BAUD = 57600

m = mavutil.mavlink_connection(PORT, baud=BAUD)
print(f"[MAV] Waiting for heartbeat on {PORT} @ {BAUD}…")
m.wait_heartbeat()
print(f"[MAV] Heartbeat from system {m.target_system} component {m.target_component}")

while True:
    msg = m.recv_match(blocking=True, timeout=2)
    if msg:
        print("[MAV]", msg.get_type(), msg.to_dict())
