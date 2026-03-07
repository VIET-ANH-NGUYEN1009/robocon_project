/*
  Arduino UNO R3 + AFMotor v1 + Bluetooth HC-05 + IR line + HC-SR04 + 2 Servo
  Điều khiển:
    - W : Nâng (servo 360°)
    - u : Hạ (servo 360°)
    - N : Gắp (servo 180°)
    - q : Thả (servo 180°)
    - S : Dừng xe + dừng nâng/hạ
    - K : AUTO (dò line + stop @5cm)
    - P : MANUAL (Bluetooth)
    - F/B/L/R : Tiến/Lùi/Trái/Phải (MANUAL)
    - 0..9 : chỉnh tốc độ (80..255)

  LƯU Ý Motor Shield v1:
    - M1 dùng D11 (PWM) => sẽ xung đột nếu bạn dùng Bluetooth ở D11.
    - Code này giữ SoftwareSerial BT(10,11) như code gốc của bạn.
    - Nếu bạn dùng cổng M1, vui lòng đổi Bluetooth sang các chân khác (VD: RX=D2, TX=D9),
      và đổi SERVO_LIFT_PIN sang A4 để tránh trùng.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Servo.h>

// ================== Bluetooth (GIỮ như code gốc) ==================
// HC-05 TXD -> D10 (RX của Arduino SoftwareSerial)
// HC-05 RXD -> D11 (TX của Arduino SoftwareSerial) (NHỚ chia áp 5V->3.3V)
SoftwareSerial BT(10, 11);

// ================== IR line sensors ==================
#define IR_LEFT   A0
#define IR_RIGHT  A1

// ================== HC-SR04 ==================
#define TRIGGER_PIN  A2
#define ECHO_PIN     A3
#define MAX_DISTANCE 50
#define STOP_DISTANCE 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================== Motors (AFMotor v1) ==================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ================== Speed & Mode ==================
int speedVal = 230;
char lastCmd = 'S';

enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1 };
Mode mode = MODE_MANUAL;

// ================== LED trạng thái ==================
const int LED_PIN = LED_BUILTIN;   // chân 13 Arduino
unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 300;  // AUTO nháy 300ms

// ================== Servo (Nâng/Hạ + Gắp/Thả) ==================
// Tránh Timer/xung đột: KHÔNG dùng D3/D5/D6/D11 cho servo (PWM motor), KHÔNG dùng D9/D10 nếu bạn cần servo header của shield
const int SERVO_LIFT_PIN = 2;   // Servo 360° - nâng/hạ (continuous rotation)
const int SERVO_GRIP_PIN = A5;  // Servo 180° - gắp/thả (A5 = digital 19)

Servo servoLift; // 360° nâng/hạ
Servo servoGrip; // 180° gắp/thả

// Tham số servo 360° (microseconds)
int PWM_STOP = 1500;   // dừng
int PWM_UP   = 1700;   // nâng (chỉnh tùy servo: 1650..1750)
int PWM_DOWN = 1300;   // hạ  (chỉnh tùy servo: 1250..1350)

// Thời gian “nhịp” cho nâng/hạ, sau đó tự dừng (ms)
unsigned long LIFT_PULSE_MS = 250;
// true: nhấn 1 lần chạy ~LIFT_PULSE_MS rồi tự dừng; false: chạy liên tục tới khi bấm S/W/u
bool pulseMode = true;

// Góc servo 180° (điều chỉnh theo cơ cấu kẹp)
int ANGLE_GRAB     = 120; // gắp (kẹp chặt)
int ANGLE_RELEASE  = 60;  // thả (mở kẹp)

// Trạng thái nâng/hạ để auto-dừng (pulseMode)
bool liftingActive = false;
unsigned long motionStart = 0;

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
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

int getDistance() {
  delay(40);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 100;
  return cm;
}

void report(const __FlashStringHelper* msg) {
  Serial.println(msg);
  BT.println(msg);
}

// ================== Servo helpers ==================
void stopLift() {
  servoLift.writeMicroseconds(PWM_STOP);
}

void startLiftUp() {
  servoLift.writeMicroseconds(PWM_UP);
  liftingActive = true;
  motionStart = millis();
  if (!pulseMode) liftingActive = false; // nếu không dùng pulse, chạy liên tục tới khi bấm S/W/u
}

void startLiftDown() {
  servoLift.writeMicroseconds(PWM_DOWN);
  liftingActive = true;
  motionStart = millis();
  if (!pulseMode) liftingActive = false;
}

void grab() {
  servoGrip.write(ANGLE_GRAB);
  delay(150); // cho kẹp ổn định
}

void releaseObject() {
  servoGrip.write(ANGLE_RELEASE);
  delay(150);
}

// ================== AUTO MODE ==================
void autoStep() {
  int d = getDistance();
  if (d <= STOP_DISTANCE) {
    stopAll();
    Serial.print(F("[AUTO] STOP distance=")); Serial.print(d); Serial.println(F("cm"));
    BT.print(F("[AUTO] STOP distance=")); BT.print(d); BT.println(F("cm"));
    return;
  }

  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  if (L == 0 && R == 0) {
    goForward();
  }
  else if (L == 0 && R == 1) {
    turnLeft();
  }
  else if (L == 1 && R == 0) {
    turnRight();
  }
  else {
    stopAll();
  }
}

// ================== Handle Commands ==================
void handleCommand(char c) {
  if (c == '\n' || c == '\r') return;
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A'; // quy về HOA

  // ===== Tốc độ (0..9) =====
  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    int newSpeed = map(lvl, 0, 9, 80, 255);
    speedVal = newSpeed;
    setAllSpeeds(speedVal);
    report(F("[SPD] Speed changed"));
    return;
  }

  switch (c) {
    // ===== Chế độ lái =====
    case 'K':   // AUTO mode
      mode = MODE_AUTO;
      stopAll();
      report(F("[MODE] AUTO (Line + Stop@5cm)"));
      ledState = true;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
      break;

    case 'P':   // MANUAL mode
      mode = MODE_MANUAL;
      stopAll();
      report(F("[MODE] MANUAL (Bluetooth control)"));
      digitalWrite(LED_PIN, HIGH);  // sáng liên tục
      break;

    // ===== Điều khiển xe (MANUAL) =====
    case 'F':
    case 'B':
    case 'L':
    case 'R':
      if (mode == MODE_MANUAL) {
        if (c == 'F') { goForward(); report(F("Forward")); }
        else if (c == 'B') { goBackward(); report(F("Backward")); }
        else if (c == 'L') { turnLeft(); report(F("Left")); }
        else if (c == 'R') { turnRight(); report(F("Right")); }
      } else {
        report(F("[WARN] In AUTO mode. Press P for MANUAL."));
      }
      break;

    case 'S':   // Dừng xe + dừng nâng/hạ
      stopAll();
      stopLift();
      liftingActive = false;
      report(F("Stop (car + lift)"));
      break;

    // ===== Nâng/Hạ + Gắp/Thả =====
    case 'W':   // Nâng (servo 360°)
      startLiftUp();
      report(F("[LIFT] UP"));
      break;

    case 'U':   // Hạ (nhận cả 'u' do đã upper-case)
      startLiftDown();
      report(F("[LIFT] DOWN"));
      break;

    case 'N':   // Gắp (servo 180°)
      grab();
      report(F("[GRIP] GRAB"));
      break;

    case 'Q':   // Thả (nhận cả 'q')
      releaseObject();
      report(F("[GRIP] RELEASE"));
      break;

    default:
      // Không nhận dạng -> chỉ dừng xe
      stopAll();
      report(F("Unknown -> Stop"));
      break;
  }
}

// ================== Setup ==================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Servo attach
  servoLift.attach(SERVO_LIFT_PIN);
  servoGrip.attach(SERVO_GRIP_PIN);
  stopLift();
  servoGrip.write(ANGLE_RELEASE);

  setAllSpeeds(speedVal);
  stopAll();

  digitalWrite(LED_PIN, HIGH); // default MANUAL: sáng

  // Gợi ý lệnh qua Serial
  Serial.println(F("Ready: F/B/L/R, K(AUTO)/P(MANUAL), 0..9 speed"));
  Serial.println(F("Lift/Grip: W(UP), u(DOWN), N(GRAB), q(RELEASE), S(STOP car+lift)"));
}

// ================== Loop ==================
void loop() {
  if (BT.available()) {
    handleCommand(BT.read());
  }

  if (Serial.available()) {
    handleCommand(Serial.read());
  }

  // LED trạng thái
  if (mode == MODE_AUTO) {
    unsigned long now = millis();
    if (now - lastBlink >= BLINK_INTERVAL) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // Auto chạy
  if (mode == MODE_AUTO) {
    autoStep();
  }

  // Tự dừng servo 360° nếu đang ở chế độ pulse
  if (pulseMode && liftingActive) {
    if (millis() - motionStart >= LIFT_PULSE_MS) {
      stopLift();
      liftingActive = false;
    }
  }
}
