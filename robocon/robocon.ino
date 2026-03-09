/*
  Arduino UNO R3 + Adafruit Motor Shield v1 + Bluetooth HC-05 (UART cứng D0/D1)
  + IR line (A0/A1) + HC-SR04 (A2/A3) + 2 Servo (A4/A5)

  Điều khiển:
    - W : Nâng (servo 360° – A4)
    - U : Hạ (servo 360° – A4)  (nhận cả 'u')
    - N : Gắp (servo 180° – A5)
    - Q : Thả (servo 180° – A5) (nhận cả 'q')
    - S : Dừng xe + dừng nâng/hạ
    - K : AUTO (dò line + stop @5cm)
    - P : MANUAL (Bluetooth)
    - F/B/L/R : Tiến/Lùi/Trái/Phải (MANUAL)
    - 0..9 : chỉnh tốc độ (80..255)

  LƯU Ý:
    - Motor Shield v1 dùng rất nhiều chân:
        PWM: D3 (M2), D5 (M3), D6 (M4), D11 (M1)
        Dir: M1(D12,D13) M2(D4,D7) M3(D8,D9) M4(D2,D10)
      => KHÔNG dùng các chân trên cho cảm biến/servo/Bluetooth.
    - LED_BUILTIN (D13) bị M1 dùng => KHÔNG chớp LED trên D13.
    - Bluetooth dùng D0/D1 (UART cứng). NHỚ rút TX/RX HC-05 khi Upload code.
*/

#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// ================== Bluetooth qua UART cứng (D0/D1) ==================
// Dùng chung Serial cho cả PC (USB) và HC-05. Tránh đọc/ghi 2 lần.
#define BT Serial

// ================== IR line sensors ==================
#define IR_LEFT   A0
#define IR_RIGHT  A1

// ================== HC-SR04 ==================
#define TRIGGER_PIN   A2
#define ECHO_PIN      A3
#define MAX_DISTANCE  50
#define STOP_DISTANCE 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================== Motors (AFMotor v1) ==================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ================== Speed & Mode ==================
int speedVal = 230;
enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1 };
Mode mode = MODE_MANUAL;

// ================== LED trạng thái ==================
// KHÔNG dùng D13 (LED_BUILTIN) vì xung đột M1. Đặt -1 để vô hiệu.
const int LED_PIN = -1;
unsigned long lastBlink = 0;
bool ledState = false;
const unsigned long BLINK_INTERVAL = 300;  // AUTO nháy 300ms

// ================== Servo (Nâng/Hạ + Gắp/Thả) ==================
// Tránh mọi chân của shield: dùng A4 & A5 là an toàn.
const int SERVO_LIFT_PIN = A4;  // Servo 360° - nâng/hạ (continuous rotation)
const int SERVO_GRIP_PIN = A5;  // Servo 180° - gắp/thả

Servo servoLift; // 360° nâng/hạ
Servo servoGrip; // 180° gắp/thả

// Tham số servo 360° (microseconds)
int PWM_STOP = 1500;   // dừng
int PWM_UP   = 1700;   // nâng (chỉnh tùy servo: 1650..1750)
int PWM_DOWN = 1300;   // hạ  (chỉnh tùy servo: 1250..1350)

// Thời gian “nhịp” cho nâng/hạ, sau đó tự dừng (ms)
unsigned long LIFT_PULSE_MS = 250;
// true: nhấn 1 lần chạy ~LIFT_PULSE_MS rồi tự dừng; false: chạy liên tục tới khi bấm S/W/U
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
  if (cm == 0) cm = 100;  // khi không bắt được echo
  return cm;
}

// In ra cổng BT (Serial). Tránh in 2 lần.
void report(const __FlashStringHelper* msg) {
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
  if (!pulseMode) liftingActive = false; // nếu không dùng pulse, chạy liên tục tới khi bấm S/W/U
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
      if (LED_PIN >= 0) {
        ledState = true;
        digitalWrite(LED_PIN, ledState);
        lastBlink = millis();
      }
      break;

    case 'P':   // MANUAL mode
      mode = MODE_MANUAL;
      stopAll();
      report(F("[MODE] MANUAL (Bluetooth control)"));
      if (LED_PIN >= 0) digitalWrite(LED_PIN, HIGH);  // sáng liên tục
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

    case 'U':   // Hạ
      startLiftDown();
      report(F("[LIFT] DOWN"));
      break;

    case 'N':   // Gắp (servo 180°)
      grab();
      report(F("[GRIP] GRAB"));
      break;

    case 'Q':   // Thả
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
  // UART cứng cho HC-05 & Serial Monitor
  BT.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);

  // Servo attach trên A4 & A5 (tránh xung đột shield)
  servoLift.attach(SERVO_LIFT_PIN);
  servoGrip.attach(SERVO_GRIP_PIN);
  stopLift();
  servoGrip.write(ANGLE_RELEASE);

  setAllSpeeds(speedVal);
  stopAll();

  if (LED_PIN >= 0) digitalWrite(LED_PIN, HIGH); // default MANUAL: sáng

  // Gợi ý lệnh qua BT/Serial
  BT.println(F("Ready: F/B/L/R, K(AUTO)/P(MANUAL), 0..9 speed"));
  BT.println(F("Lift/Grip: W(UP), U(DOWN), N(GRAB), Q(RELEASE), S(STOP car+lift)"));
  BT.println(F("Note: Using HW Serial D0/D1 for HC-05. Unplug RX/TX when uploading."));
}

// ================== Loop ==================
void loop() {
  // Chỉ đọc từ BT (Serial). Tránh double-read.
  if (BT.available()) {
    handleCommand(BT.read());
  }

  // LED trạng thái (nếu có)
  if (LED_PIN >= 0) {
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
