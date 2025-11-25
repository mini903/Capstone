# pixhawk_reader.py
# pymavlink 필요: pip install pymavlink

from pymavlink import mavutil
import time

class PixhawkReader:
    def __init__(self, port="/dev/ttyAMA0", baud=57600, timeout=30):
        self.port = port
        self.baud = baud
        print(f"[PixhawkReader] connecting to {port} @{baud}...")
        self.m = mavutil.mavlink_connection(port, baud=baud)
        self.m.wait_heartbeat(timeout=timeout)
        print("[PixhawkReader] heartbeat received")
        self.target_system = self.m.target_system
        self.target_component = self.m.target_component

    def get_global_position(self, blocking=True, timeout=2):
        msg = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=blocking, timeout=timeout)
        if not msg:
            return None
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        rel_alt = msg.relative_alt / 1000.0
        return {"lat": lat, "lon": lon, "alt": rel_alt}

    def get_heading(self, blocking=True, timeout=2):
        # VFR_HUD has heading? not always. Use ATTITUDE + COMPASS_HEADING if available.
        # Try to read 'VFR_HUD' for heading or 'HIL_CONTROLS' else fallback to 0.
        msg = self.m.recv_match(type='VFR_HUD', blocking=blocking, timeout=timeout)
        if msg and hasattr(msg, 'heading'):
            return float(msg.heading)
        # fallback: try GLOBAL_POSITION_INT 'hdg' not always present
        # Or read 'ATTITUDE' and return yaw in degrees (needs conversion)
        msg2 = self.m.recv_match(type='ATTITUDE', blocking=blocking, timeout=timeout)
        if msg2:
            import math
            yaw = msg2.yaw
            return math.degrees(yaw) % 360
        return None

    def send_heartbeat(self):
        # optional: send heartbeat to keep link alive
        self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

if __name__ == "__main__":
    pr = PixhawkReader("/dev/ttyAMA0", 57600)
    while True:
        pos = pr.get_global_position()
        hdg = pr.get_heading()
        print("pos:", pos, "hdg:", hdg)
        time.sleep(1)
