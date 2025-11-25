# autonomous_pi_nav.py
# Pixhawk에서 위치 얻어와서 간단한 bearing control로 differential drive를 구동
# 필요: motor_control.py, pixhawk_reader.py

import math
import time
from pixhawk_reader import PixhawkReader
import motor_control as mc

# --- 설정 (사용 환경에 맞게 변경) ---
PIXHAWK_PORT = "/dev/ttyAMA0"
PIXHAWK_BAUD = 57600

# 웨이포인트 리스트 (lat, lon, rel_alt)
WAYPOINTS = [
    (37.123456, 127.123456, 0),
    (37.123556, 127.123656, 0),
    # 필요시 추가
]

# 제어 파라미터
BASE_SPEED = 40.0       # 기본 전진 속도 (%)
MAX_SPEED = 80.0
STEER_KP = 1.2          # 회전 이득 (각오차 -> 좌우 속도 차)
DIST_TOLERANCE_M = 3.0  # 도달 허용 반경 (m)
LOOP_DELAY = 0.3

# --- 유틸리티 ---
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing_to_target(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2-lon1))*math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1))*math.sin(math.radians(lat2)) - math.sin(math.radians(lat1))*math.cos(math.radians(lat2))*math.cos(math.radians(lon2-lon1))
    brng = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return brng

def angle_diff(a, b):
    # smallest difference a-b in degrees
    d = (a - b + 180) % 360 - 180
    return d

# --- 메인 루프 ---
def run_mission():
    pr = PixhawkReader(PIXHAWK_PORT, PIXHAWK_BAUD)
    time.sleep(1)

    for idx, wp in enumerate(WAYPOINTS):
        target_lat, target_lon, _ = wp
        print(f"=== WP {idx+1}/{len(WAYPOINTS)} -> {target_lat}, {target_lon}")
        reached = False
        start_time = time.time()
        while not reached:
            pos = pr.get_global_position()
            if pos is None:
                print("[WARN] GPS data not available")
                time.sleep(1); continue
            curr_lat = pos['lat']; curr_lon = pos['lon']
            dist = haversine(curr_lat, curr_lon, target_lat, target_lon)
            hdg = pr.get_heading()
            if hdg is None:
                hdg = 0.0
            brg = bearing_to_target(curr_lat, curr_lon, target_lat, target_lon)
            err = angle_diff(brg, hdg)  # 원하는방향 - 현재방향

            print(f"[NAV] dist={dist:.1f}m, bearing={brg:.1f}, heading={hdg:.1f}, err={err:.1f}")

            if mc.kill_pressed():
                print("[EMERGENCY] Kill pressed -> STOP")
                mc.stop()
                return

            if dist <= DIST_TOLERANCE_M:
                print("[NAV] Waypoint reached")
                mc.stop()
                reached = True
                time.sleep(1)
                break

            # 간단한 P 제어로 차 좌/우 속도 결정
            steer = STEER_KP * (err / 180.0)  # normalized -1..1 approximately
            # left = base + steer, right = base - steer
            left = BASE_SPEED + (steer * BASE_SPEED)
            right = BASE_SPEED - (steer * BASE_SPEED)

            # clamp
            left = max(-MAX_SPEED, min(MAX_SPEED, left))
            right = max(-MAX_SPEED, min(MAX_SPEED, right))

            mc.set_motor_speed(left, right)
            time.sleep(LOOP_DELAY)

    print("[MISSION] All waypoints completed. STOP")
    mc.stop()

if __name__ == "__main__":
    try:
        run_mission()
    except KeyboardInterrupt:
        print("User interrupt: stopping")
        mc.stop()
    except Exception as e:
        print("Exception:", e)
        mc.stop()
