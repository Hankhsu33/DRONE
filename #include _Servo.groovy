#include <Servo.h>

Servo servos[4];
int pins[4] = {7, 8, 9, 10};  // 旋轉、前後、上下、夾爪

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    servos[i].attach(pins[i]);
    servos[i].write(90);
  }
  Serial.println("READY");  // ← 開機訊息，給 Pi 做握手
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // 握手通道：不影響控制命令
  if (input == "PING") { Serial.println("PONG"); return; }

  // 控制格式 1: "S{index}:{angle}"
  if (input.startsWith("S")) {
    int colon = input.indexOf(':');
    if (colon > 1) {
      int idx = input.substring(1, colon).toInt();
      int angle = input.substring(colon + 1).toInt();
      if (idx >= 0 && idx < 4) servos[idx].write(constrain(angle, 0, 180));
    }
    return;
  }

  // 控制格式 2: "pin,angle"
  int comma = input.indexOf(',');
  if (comma > 0) {
    int pin = input.substring(0, comma).toInt();
    int angle = input.substring(comma + 1).toInt();
    for (int i = 0; i < 4; i++) {
      if (pins[i] == pin) { servos[i].write(constrain(angle, 0, 180)); break; }
    }
  }
}
