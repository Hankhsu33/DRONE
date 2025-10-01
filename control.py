# webcam_control.py — 用攝影機自動夾取木瓜（不使用搖桿）
import time, serial, cv2
from math import copysign

# ===== 需要：pip install ultralytics opencv-python pyserial =====
try:
    from ultralytics import YOLO
except ImportError:
    raise SystemExit("請先安裝: pip install ultralytics opencv-python pyserial")

# ===== 基本設定（依環境調整）=====
SERIAL_PORT = "/dev/ttyACM0"   # 你的 Arduino 串口
BAUD        = 115200           # 要與 Arduino 端一致（你若是 9600 就改成 9600）
CAMERA_ID   = 0                # USB webcam 的編號
model = YOLO("/home/zack/papaya_model/best.ncnn.param")       # 你的木瓜 YOLO 模型
TARGET_NAMES = {"papaya", "matang", "belum_matang"}  # 依你的模型類別名稱調整

# 舵機索引: 0=旋轉, 1=前後, 2=上下, 3=夾爪
HOME = [90, 90, 150, 100]    # 回家姿態（夾爪打開）
GRAB_ANGLE    = 30          # 夾爪關
RELEASE_ANGLE = 100         # 夾爪開

# 控制/判斷參數
CENTER_TOL    = 0.10   # 置中容差（相對畫面寬高比例）
SIZE_NEAR     = 0.32   # bbox高 / 畫面高 >= 觸發夾取
STABLE_NEEDED = 5      # 連續穩定幀數
FPS_DELAY     = 0.03   # 主迴圈 sleep

# 步進與安全角度範圍
STEP_PAN  = 2
STEP_EXT  = 2
STEP_TILT = 2
PAN_MIN, PAN_MAX   = 0, 180
EXT_MIN, EXT_MAX   = 40, 150
TILT_MIN, TILT_MAX = 40, 140

def connect_arduino():
    while True:
        try:
            print("連線 Arduino...")
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
            time.sleep(2)  # 等 Arduino reset
            print("✅ Arduino OK")
            return ser
        except serial.SerialException:
            print("❌ 找不到 Arduino，2 秒後重試")
            time.sleep(2)

def send_angle(ser, idx, angle):
    angle = int(max(0, min(180, angle)))
    ser.write(f"S{idx}:{angle}\n".encode())

def goto_pose(ser, pose, delay=0.02):
    for i, ang in enumerate(pose):
        send_angle(ser, i, ang)
        time.sleep(delay)

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def bbox_center_and_height(box):
    x1, y1, x2, y2 = box
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    h  = (y2 - y1)
    return cx, cy, h

def main():
    ser = connect_arduino()
    goto_pose(ser, HOME, 0.05)
    send_angle(ser, 3, RELEASE_ANGLE)  # 夾爪打開

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        raise SystemExit("無法開啟攝影機")

    model = YOLO(MODEL_PATH)

    state  = "SEARCH"   # SEARCH → ALIGN → APPROACH → GRAB → LIFT → RETURN → RELEASE
    stable = 0
    pose   = HOME[:]
    last_grab_time = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        H, W = frame.shape[:2]
        cx_tgt, cy_tgt = W/2, H/2

        # YOLO 偵測
        res = model(frame, verbose=False)[0]
        found = None
        best_h = 0
        for box, conf, cls in zip(res.boxes.xyxy, res.boxes.conf, res.boxes.cls):
            name = res.names[int(cls)]
            if name not in TARGET_NAMES or float(conf) < 0.6:
                continue
            x1, y1, x2, y2 = map(float, box)
            _, _, h = bbox_center_and_height((x1,y1,x2,y2))
            if h > best_h:
                best_h = h
                found  = (x1,y1,x2,y2)

        if found is None:
            # 找不到 → 回到搜尋或維持
            stable = 0
            if state not in ("SEARCH","RETURN","RELEASE"):
                state = "SEARCH"
                print("→ SEARCH")
            time.sleep(FPS_DELAY)
            continue

        bx, by, bh = bbox_center_and_height(found)
        ex = (bx - cx_tgt) / W   # 水平誤差（-0.5~0.5）
        ey = (by - cy_tgt) / H   # 垂直誤差
        size_frac = bh / H

        centered   = (abs(ex) <= CENTER_TOL) and (abs(ey) <= CENTER_TOL)
        near_enough= size_frac >= SIZE_NEAR

        if state == "SEARCH":
            state = "ALIGN"; print("→ ALIGN")

        elif state == "ALIGN":
            # 旋轉/上下 微調置中
            if abs(ex) > CENTER_TOL:
                pose[0] = clamp(pose[0] - copysign(STEP_PAN, ex), PAN_MIN, PAN_MAX)
                send_angle(ser, 0, pose[0])
            if abs(ey) > CENTER_TOL:
                pose[2] = clamp(pose[2] + copysign(STEP_TILT, ey), TILT_MIN, TILT_MAX)
                send_angle(ser, 2, pose[2])

            stable = stable + 1 if centered else 0
            if stable >= STABLE_NEEDED:
                state = "APPROACH"; print("→ APPROACH")

        elif state == "APPROACH":
            # 未置中就回 ALIGN；置中但不夠近就前伸；夠近則夾取
            if not centered:
                state = "ALIGN"; stable = 0; print("↩ ALIGN（置中修正）")
            else:
                if not near_enough:
                    pose[1] = clamp(pose[1] + STEP_EXT, EXT_MIN, EXT_MAX)  # 前伸
                    send_angle(ser, 1, pose[1])
                else:
                    state = "GRAB"; print("→ GRAB")

        elif state == "GRAB":
            send_angle(ser, 3, GRAB_ANGLE)  # 關爪
            time.sleep(0.3)
            last_grab_time = time.time()
            state = "LIFT"; print("→ LIFT")

        elif state == "LIFT":
            # 微抬高，避免拖拉
            pose[2] = clamp(pose[2] - 10, TILT_MIN, TILT_MAX)
            send_angle(ser, 2, pose[2])
            time.sleep(0.3)
            state = "RETURN"; print("→ RETURN")

        elif state == "RETURN":
            # 收回/回正/回到 HOME
            pose[1] = HOME[1]; send_angle(ser, 1, pose[1]); time.sleep(0.25)
            pose[0] = HOME[0]; send_angle(ser, 0, pose[0]); time.sleep(0.25)
            pose[2] = HOME[2]; send_angle(ser, 2, pose[2]); time.sleep(0.25)
            state = "RELEASE"; print("→ RELEASE")

        elif state == "RELEASE":
            if time.time() - last_grab_time < 0.5:  # 防抖
                time.sleep(0.2)
            send_angle(ser, 3, RELEASE_ANGLE)  # 放開
            time.sleep(0.3)
            goto_pose(ser, HOME, 0.05)
            stable = 0
            state = "SEARCH"
            print("✅ 完成一循環，回到 SEARCH")

        time.sleep(FPS_DELAY)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye")
