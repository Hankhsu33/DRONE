#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
papaya_patrol_v3_live.py
巡邏 + YOLO 偵測 + 緩降夾取 + 返航放下 + 回到抓取點「續飛同一航道到終點」再換航道
- 使用背景執行緒持續抓取最新影格，避免 GUI 卡住或停在舊畫面
- RTL / 降落等待 / 上升 / 換航道 → 全程持續顯示最新影像
"""

import time
import argparse
import threading
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from ultralytics import YOLO

# ---------- 背景相機讀取 ----------
class CamReader:
    def __init__(self, src):
        self.cap = (cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                    if isinstance(src, str) and src.startswith("udp://")
                    else cv2.VideoCapture(src))
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        while self.running:
            self.cap.grab()
            ret, img = self.cap.retrieve()
            if ret:
                with self.lock:
                    self.frame = img
            time.sleep(0.001)

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def release(self):
        self.running = False
        try:
            self.t.join(timeout=0.5)
        except Exception:
            pass
        self.cap.release()

# ---------- YOLO ----------
model = YOLO("best.pt")
model.to("cpu")

def detect_papaya(frame, targets=("matang","setengah_matang","belum_matang"),
                  conf_thres=0.6, show=True):
    results = model(frame, verbose=False)
    if show:
        annotated = results[0].plot()
        cv2.imshow("YOLO Detection", annotated)
        cv2.waitKey(1)

    for r in results:
        for b in r.boxes:
            name = r.names[int(b.cls)]
            conf = float(b.conf)
            if name in targets and conf >= conf_thres:
                print(f"[YOLO] 偵測到木瓜：{name}, conf={conf:.2f}")
                return True, name
    return False, None

# ---------- 飛行工具 ----------
def arm_and_takeoff(v, alt, cam=None):
    print("[*] 解鎖並起飛")
    v.mode = VehicleMode("GUIDED")
    v.armed = True
    while not v.armed:
        if cam:
            ok, fr = cam.read()
            if ok: cv2.imshow("YOLO Detection", fr)
        cv2.waitKey(1)
        time.sleep(0.5)
    v.simple_takeoff(alt)
    while True:
        h = v.location.global_relative_frame.alt or 0.0
        print(f"[Takeoff] Alt={h:.2f} m")
        if cam:
            ok, fr = cam.read()
            if ok: cv2.imshow("YOLO Detection", fr)
        cv2.waitKey(1)
        if h >= alt * 0.9: break
        time.sleep(0.5)

def goto_alt(v, target_alt, tol=0.15, cam=None):
    here = v.location.global_frame
    tgt = LocationGlobalRelative(here.lat, here.lon, target_alt)
    v.simple_goto(tgt)
    while True:
        h = v.location.global_relative_frame.alt or 0.0
        print(f"[GotoAlt] Alt={h:.2f} -> {target_alt:.2f}")
        if cam:
            ok, fr = cam.read()
            if ok: cv2.imshow("YOLO Detection", fr)
        cv2.waitKey(1)
        if abs(h - target_alt) <= tol: break
        time.sleep(0.2)

def send_world_velocity(v, vx, vy, vz, dt=0.2, cam=None):
    msg = v.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz,
        0, 0, 0, 0, 0
    )
    v.send_mavlink(msg); v.flush()
    if cam:
        ok, fr = cam.read()
        if ok: cv2.imshow("YOLO Detection", fr)
    cv2.waitKey(1)
    time.sleep(dt)

def hold_position(v, seconds=1.2, cam=None):
    t0 = time.time()
    while time.time() - t0 < seconds:
        send_world_velocity(v, 0, 0, 0, dt=0.1, cam=cam)

def wait_reach(v, lat, lon, alt=None, cam=None, eps_latlon=1e-5):
    while True:
        p = v.location.global_frame
        if abs(p.lat-lat)<eps_latlon and abs(p.lon-lon)<eps_latlon:
            if alt is None: break
            h = v.location.global_relative_frame.alt or 0.0
            if abs(h-alt)<0.2: break
        if cam:
            ok, fr = cam.read()
            if ok: cv2.imshow("YOLO Detection", fr)
        cv2.waitKey(1)
        time.sleep(0.4)

# ---------- 巡邏 ----------
def lawnmower_patrol(v, cam: CamReader,
                     length=20.0, width=5.0, passes=4,
                     v_forward=1.0, v_strafe=0.6, stop_sec=1.5,
                     work_alt=5.0, grab_alt=1.5, grip_wait=5.0,
                     targets=("matang","setengah_matang","belum_matang"),
                     conf_thres=0.6, cooldown_sec=10.0):

    direction = +1
    last_trigger_ts = 0.0

    for i in range(passes):
        print(f"[*] 航道 {i+1}/{passes} 方向={'北' if direction>0 else '南'}")
        seg_speed_x = direction * v_forward
        lane_duration = length / max(v_forward, 1e-6)
        t_lane_start = time.time()

        while True:
            elapsed = time.time() - t_lane_start
            if elapsed >= lane_duration: break

            ok, frame = cam.read()
            if ok and (time.time()-last_trigger_ts > cooldown_sec):
                detected, name = detect_papaya(frame, targets, conf_thres, show=True)
                if detected:
                    last_trigger_ts = time.time()
                    print(f"[*] 偵測到 {name} → 緩降至 {grab_alt} m")
                    hold_position(v, 0.7, cam)
                    goto_alt(v, grab_alt, cam=cam)
                    print(f"[*] 等待夾取 {grip_wait} 秒")
                    hold_position(v, grip_wait, cam)
                    print(f"[*] 上升回 {work_alt} m")
                    goto_alt(v, work_alt, cam=cam)

                    grab_point = v.location.global_frame
                    flown_time = elapsed
                    remaining_time = max(lane_duration-flown_time,0.0)
                    print(f"[*] 記錄抓取點 Lat={grab_point.lat}, Lon={grab_point.lon}")

                    print("[*] RTL 回基地放下木瓜")
                    v.mode = VehicleMode("RTL")
                    while v.armed:
                        send_world_velocity(v,0,0,0,dt=0.2,cam=cam)
                    print("[*] 已降落 → 模擬放下 5 秒")
                    hold_position(v,5,cam)

                    print("[*] 起飛回抓取點，續飛同一航道")
                    arm_and_takeoff(v, work_alt, cam)
                    v.simple_goto(LocationGlobalRelative(grab_point.lat, grab_point.lon, work_alt))
                    wait_reach(v, grab_point.lat, grab_point.lon, work_alt, cam)
                    hold_position(v,0.6,cam)

                    t_resume = time.time()
                    while time.time()-t_resume < remaining_time:
                        send_world_velocity(v, seg_speed_x,0,0,dt=0.2,cam=cam)
                    break

            send_world_velocity(v, seg_speed_x, 0, 0, dt=0.2, cam=cam)

        hold_position(v, stop_sec, cam)
        if i<passes-1:
            print("[*] 橫移到下一航道")
            t_side=time.time()
            side_time=width/max(v_strafe,1e-6)
            while time.time()-t_side<side_time:
                send_world_velocity(v,0,+v_strafe,0,dt=0.2,cam=cam)
            hold_position(v,0.8,cam)
        direction*=-1

# ---------- 主程式 ----------
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--conn",default="udp:127.0.0.1:14550")
    ap.add_argument("--work_alt",type=float,default=5.0)
    ap.add_argument("--grab_alt",type=float,default=1.5)
    ap.add_argument("--grip_wait",type=float,default=5.0)
    ap.add_argument("--length",type=float,default=20.0)
    ap.add_argument("--width",type=float,default=5.0)
    ap.add_argument("--passes",type=int,default=4)
    ap.add_argument("--v_forward",type=float,default=1.0)
    ap.add_argument("--v_strafe",type=float,default=0.6)
    ap.add_argument("--video",default="0")
    ap.add_argument("--targets",default="any",choices=["any","matang_only"])
    ap.add_argument("--conf",type=float,default=0.6)
    ap.add_argument("--cooldown",type=float,default=10.0)
    args=ap.parse_args()

    print(f"[*] 連線：{args.conn}")
    v=connect(args.conn,wait_ready=True,timeout=60)

    src=int(args.video) if args.video.isdigit() else args.video
    cam=CamReader(src)

    arm_and_takeoff(v,args.work_alt,cam)

    targets=("matang","setengah_matang","belum_matang") if args.targets=="any" else ("matang",)

    try:
        lawnmower_patrol(v,cam,
            length=args.length,width=args.width,passes=args.passes,
            v_forward=args.v_forward,v_strafe=args.v_strafe,stop_sec=1.5,
            work_alt=args.work_alt,grab_alt=args.grab_alt,grip_wait=args.grip_wait,
            targets=targets,conf_thres=args.conf,cooldown_sec=args.cooldown)
    finally:
        cam.release()
        cv2.destroyAllWindows()

    print("[*] 巡邏完成 → RTL")
    v.mode=VehicleMode("RTL")
    while v.armed:
        send_world_velocity(v,0,0,0,dt=0.2,cam=cam)
    print("[*] 任務結束")
    v.close()

if __name__=="__main__":
    main()
