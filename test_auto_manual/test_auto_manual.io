
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// ================== Bluetooth ==================
SoftwareSerial BT(10, 11); // HC-05: RX=10, TX=11

// ================== IR line sensors ==================
#define IR_LEFT   A0
#define IR_RIGHT  A1
// Nếu module IR  trả ngược (đen=1, trắng=0) thì đảo điều kiện trong autoStep()

// ================== HC-SR04 (cố định phía trước) ==================
#define TRIGGER_PIN  A2
#define ECHO_PIN     A3
#define MAX_DISTANCE 50      // đo tối đa 50cm
#define STOP_DISTANCE 5      // dừng khi vật ≤ 5cm

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================== Motors (Adafruit Motor Shield) ==================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ================== Speed & Mode ==================
int speedVal = 230;      // 0..255
char lastCmd = 'S';

enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1 };
Mode mode = MODE_MANUAL;

// ================== Helpers ==================
void setAllSpeeds(int spd) {
  motor1.setSpeed(spd);
  motor2.setSpeed(spd);
  motor3.setSpeed(spd);
  motor4.setSpeed(spd);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void goForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void goBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft() {
  // quay trái tại chỗ (differential)
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  // quay phải tại chỗ
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

int getDistance() {
  delay(40);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 100; // không bắt echo => coi như xa
  return cm;
}

void report(const __FlashStringHelper* msg) {
  Serial.println(msg);
  BT.println(msg);
}

// ================== AUTO MODE: line follow + stop at 5cm ==================
void autoStep() {
  // 1) Kiểm tra vật cản
  int d = getDistance();
  if (d <= STOP_DISTANCE) {
    stopAll();
    Serial.print(F("[AUTO] STOP distance=")); Serial.print(d); Serial.println(F("cm"));
    BT.print(F("[AUTO] STOP distance=")); BT.print(d); BT.println(F("cm"));
    return;
  }

  // 2) Đọc IR
  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  // Logic mặc định: ĐEN=0, TRẮNG=1
  if (L == 0 && R == 0) {
    goForward();
    //Serial.println(F("[AUTO] Forward"));
  }
  else if (L == 0 && R == 1) {
    turnLeft();
    //Serial.println(F("[AUTO] Adjust Left"));
  }
  else if (L == 1 && R == 0) {
    turnRight();
    //Serial.println(F("[AUTO] Adjust Right"));
  }
  else { // L==1 && R==1
    stopAll();
    //Serial.println(F("[AUTO] Stop (lost line)"));
  }
}

// ================== Command handling ==================
void handleCommand(char c) {
  if (c == '\n' || c == '\r') return;
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

  // Phím số đổi tốc độ (áp dụng cho cả Manual & Auto)
  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    int newSpeed = map(lvl, 0, 9, 80, 255);
    speedVal = newSpeed;
    setAllSpeeds(speedVal);
    Serial.print(F("[SPD] ")); Serial.println(speedVal);
    BT.print(F("[SPD] "));    BT.println(speedVal);
    return;
  }

  switch (c) {
    // ===== Chuyển chế độ =====
    case 'A':
      mode = MODE_AUTO;
      stopAll(); // an toàn trước khi vào AUTO
      report(F("[MODE] AUTO (Line + Stop@5cm)"));
      break;

    case 'M':
      mode = MODE_MANUAL;
      stopAll();
      report(F("[MODE] MANUAL (Bluetooth control)"));
      break;

    // ===== Lệnh chuyển động =====
    case 'F':
    case 'B':
    case 'L':
    case 'R':
      if (mode == MODE_MANUAL) {
        lastCmd = c;
        if (c == 'F') { goForward(); report(F("Forward")); }
        else if (c == 'B') { goBackward(); report(F("Backward")); }
        else if (c == 'L') { turnLeft(); report(F("Left")); }
        else if (c == 'R') { turnRight(); report(F("Right")); }
      } else {
        // Trong AUTO: bỏ qua lệnh di chuyển tay
        report(F("[WARN] In AUTO mode. Use 'M' to switch to Manual."));
      }
      break;

    case 'S':
      stopAll();
      report(F("Stop"));
      break;

    default:
      stopAll();
      report(F("Unknown -> Stop"));
      break;
  }
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  setAllSpeeds(speedVal);
  stopAll();

  Serial.println(F("Ready. Commands:"));
  Serial.println(F("  Manual: F,B,L,R,S and 0..9 for speed"));
  Serial.println(F("  Modes : A=Auto (line+stop@5cm), M=Manual"));
  BT.println(F("Ready (BT). F,B,L,R,S, 0..9 | A=Auto, M=Manual"));
}

void loop() {
  // Nhận lệnh từ Bluetooth
  if (BT.available()) {
    char c = BT.read();
    handleCommand(c);
  }

  // (Tuỳ chọn) Nhận lệnh từ USB Serial để test
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }

  // Nếu ở AUTO -> chạy tự động mỗi vòng loop
  if (mode == MODE_AUTO) {
    autoStep();
  }
}
