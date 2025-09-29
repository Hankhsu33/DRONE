#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
papaya_mission.py - YOLO 木瓜任務版
功能：
1. 連線 SITL (GUIDED 模式)
2. 起飛到 5 公尺
3. 以 1 m/s 往機頭方向飛，同時用 YOLO 偵測木瓜
4. 偵測到「成熟木瓜」：立即降落
5. 降落後再起飛回到 5 公尺
6. 啟動 RTL (返航)
"""

import time
import argparse
import cv2
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from ultralytics import YOLO
from dronekit import LocationGlobalRelative

# ====== YOLO 模型載入 ======
model = YOLO("best.pt")  # 你的木瓜模型
model.to("cpu")   # 強制用 CPU

def detect_papaya(frame):
    """回傳 (偵測到木瓜?, 類別名稱)"""
    results = model(frame)
    for r in results:
        for box in r.boxes:
            cls_name = r.names[int(box.cls)]
            conf = float(box.conf)
            if cls_name in ["matang", "belum_matang", "setengah_matang"] and conf > 0.6:
                print(f"[YOLO] 偵測到木瓜：{cls_name}, conf={conf:.2f}")
                return True, cls_name
    return False, None



def goto_alt(vehicle, target_alt):
    """改變高度但保持 ARM，不會觸發 LAND/DISARM"""
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    point = LocationGlobalRelative(lat, lon, target_alt)
    vehicle.simple_goto(point)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"[GotoAlt] Alt={alt:.2f} -> {target_alt}")
        if abs(alt - target_alt) < 0.2:
            break
        time.sleep(0.5)
def arm_and_takeoff(vehicle, target_alt):
    """解鎖並起飛"""
    print("[*] 解鎖馬達")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("[等待] 馬達解鎖中...")
        time.sleep(1)

    print(f"[*] 起飛到 {target_alt} m")
    vehicle.simple_takeoff(target_alt)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"[Takeoff] Alt={alt:.2f} m")
        if alt >= target_alt * 0.85:
            print("[*] 到達目標高度")
            break
        time.sleep(1)


def send_body_velocity(vehicle, vx, vy, vz):
    """機體座標系速度控制"""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # 僅啟用 vx,vy,vz
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def print_status(vehicle, prefix=""):
    """輸出狀態：高度、經緯度、速度"""
    alt = vehicle.location.global_relative_frame.alt
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    vx, vy, vz = vehicle.velocity
    print(f"{prefix} Alt={alt:.2f} m | Lat={lat:.6f}, Lon={lon:.6f} | V=({vx:.2f},{vy:.2f},{vz:.2f}) m/s")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conn", default="udp:127.0.0.1:14550", help="MAVLink 連線")
    args = parser.parse_args()

    print(f"[*] 連線：{args.conn}")
    vehicle = connect(args.conn, wait_ready=True, timeout=60)

    # Step 1: 起飛
    arm_and_takeoff(vehicle, 5)

    # Step 2: 開啟攝影機
    cap = cv2.VideoCapture(0)  # 攝影機或 Gazebo 串流（如 udp://@127.0.0.1:5600）

    found = False
    start = time.time()

    # Step 3: 前進並檢測
    while not found and time.time() - start < 30:  # 最多 30 秒
        ret, frame = cap.read()
        if not ret:
            continue

        detected, cls_name = detect_papaya(frame)
        if detected and cls_name in ["matang", "setengah_matang", "belum_matang"]: # 成熟木瓜才下降
            print("[*] 偵測到木瓜 → 緩降至 1.5 m")
            goto_alt(vehicle, 1.5)

            print("[*] 等待夾取 5 秒")
            time.sleep(5)

            print("[*] 上升回 5 m")
            goto_alt(vehicle, 5)

            print("[*] 啟動 RTL")
            vehicle.mode = VehicleMode("RTL")
            found = True
            break

        else:
            # 沒偵測到木瓜就繼續前進
            send_body_velocity(vehicle, 0.5, 0, 0)
            print_status(vehicle, "[Forward]")
            time.sleep(0.2)

    cap.release()

    # Step 4: 如果沒偵測到 → 返航
    if not found:
        print("[*] 未偵測到木瓜 → 啟動返航")
        vehicle.mode = VehicleMode("RTL")

    # Step 5: 等待返航結束
    while vehicle.armed:
        print_status(vehicle, "[RTL]")
        time.sleep(1)

    print("[*] 任務結束，馬達已上鎖")
    vehicle.close()


if __name__ == "__main__":
    main()
