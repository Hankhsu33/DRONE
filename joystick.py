from evdev import InputDevice, ecodes
import serial, time, os

def connect_arduino():
    while True:
        try:
            print("嘗試連線 Arduino...")
            arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)
            print("✅ 已連線 Arduino")
            return arduino
        except serial.SerialException:
            print("❌ 找不到 Arduino，2 秒後重試...")
            time.sleep(2)

dev = InputDevice('/dev/input/event9')  # ← 請依照實際 event 編號修改
info_x = dev.absinfo(ecodes.ABS_RX)

servo_labels = ["旋轉", "前後", "上下", "夾爪"]
current_servo = 0
claw_closed = False
last_angle = [90, 90,30,100 ]    
reset_angle = [90, 90,30,100]

def normalize(raw):
    return 2.0 * (raw - info_x.min) / (info_x.max - info_x.min) - 1.0

arduino = connect_arduino()

def send_angle(index, angle):
    global arduino  # 明確告訴 Python 使用全域變數
    try:
        arduino.write(f"S{index}:{angle}\n".encode())
        print(f"🎮 控制 {servo_labels[index]} → 角度: {angle}°")
    except serial.SerialException:
        print("⚠️ 傳送失敗，重新連線...")
        arduino = connect_arduino()

print("開始控制舵機...")

for i in range(4):  # 啟動時先重設一次
    send_angle(i, last_angle[i])
    time.sleep(0.1)
SMOOTH_LIMIT = 2  # 每次最多變動 2 度
DEADZONE = 0.4    # 中心死區

for event in dev.read_loop():
    if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_RX:
        norm = normalize(event.value)
        if abs(norm) < 0.4:
            continue  # 忽略微小變化

        angle = int((norm + 1) * 90)
        angle = max(0, min(180, angle))  # 限制在範圍內

        if current_servo != 3 and last_angle[current_servo] != angle:
            delta = angle - last_angle[current_servo]
            # 限制角度變化速率
            if abs(delta) > SMOOTH_LIMIT:
                delta = SMOOTH_LIMIT if delta > 0 else -SMOOTH_LIMIT
            last_angle[current_servo] += delta
            last_angle[current_servo] = max(0, min(180, last_angle[current_servo]))  # 再次限制範圍
            send_angle(current_servo, int(last_angle[current_servo]))


    elif event.type == ecodes.EV_KEY and event.value == 1:
        if event.code == ecodes.BTN_TL:  # L1 鍵
            current_servo = (current_servo + 1) % 4
            print(f"✅ 切換控制：{servo_labels[current_servo]}")
        elif event.code == ecodes.BTN_TR:  # R1 鍵
            current_servo = (current_servo - 1) % 4
            print(f"✅ 切換控制：{servo_labels[current_servo]}")
        elif event.code == ecodes.BTN_SOUTH and current_servo == 3:  # A 鍵
            claw_closed = not claw_closed
            angle = 30 if claw_closed else 100
            if last_angle[3] != angle:
                last_angle[3] = angle
                send_angle(3, angle)
                print(f"🤖 夾爪已 {'關閉' if claw_closed else '打開'}")
        elif event.code == ecodes.BTN_EAST:  # B 鍵：重置所有角度
            print("🔄 所有舵機重置為初始角度")
            for i in range(4):
                last_angle[i] = reset_angle[i]
                send_angle(i, reset_angle[i])
                time.sleep(0.1)
