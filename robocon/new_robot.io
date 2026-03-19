/*
  Arduino UNO R3 + Adafruit Motor Shield v1 + Bluetooth HC-05 (UART D0/D1)
  + IR line (A0/A1) + HC-SR04 (A2/A3)
  + Robot Arm SG90 (NO BASE): Shoulder + Elbow + Gripper

  Điều khiển:
    - W : Shoulder UP
    - U : Shoulder DOWN
    - J : Elbow UP
    - M : Elbow DOWN
    - N : Grip CLOSE
    - Q : Grip OPEN
    - S : Stop car
    - C : AUTO (line follow + Stop @5cm)
    - K : MANUAL (off AUTO)
    - F/B/L/R : Forward/Backward/Left/Right (MANUAL)
    - 0..9 : speed (80..255)
*/

#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// ================== Bluetooth UART D0/D1 ==================
#define BT Serial

// ================== IR line ==================
#define IR_LEFT   A0
#define IR_RIGHT  A1

// ================== HC-SR04 ==================
#define TRIGGER_PIN   A2
#define ECHO_PIN      A3
#define MAX_DISTANCE  50
#define STOP_DISTANCE 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================== Motors ==================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ================== Speed & Mode ==================
int speedVal = 230;
enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1 };
Mode mode = MODE_MANUAL;

// ================== Robot Arm (SG90 - NO BASE) ==================
// Motor Shield v1: servo headers mặc định là D9 (SERVO_1) và D10 (SERVO_2).
// Tránh dùng D12 vì shield v1 dùng D12 để giao tiếp 74HC595. -> dùng D2 cho Elbow. (an toàn)
const int SERVO_SHOULDER_PIN = 9;   // SG90: Shoulder
const int SERVO_ELBOW_PIN    = 2;   // SG90: Elbow  (pin free)
const int SERVO_GRIP_PIN     = 10;  // SG90: Gripper

Servo servoShoulder;
Servo servoElbow;
Servo servoGrip;

// SG90 pulse width hay gặp: ~500..2400us (tuỳ con, tuỳ clone)
const int SG90_MIN_US = 500;
const int SG90_MAX_US = 2400;

// ---- Góc hiện tại
int shoulderAng = 90;
int elbowAng    = 90;

// ---- Giới hạn an toàn (chỉnh theo cơ khí tay robot để không kẹt)
int SHOULDER_MIN = 10,  SHOULDER_MAX = 170;
int ELBOW_MIN    = 10,  ELBOW_MAX    = 170;

// ---- Gripper open/close (hay bị ngược tuỳ bộ kẹp, chỉnh lại nếu cần)
int GRIP_OPEN  = 70;
int GRIP_CLOSE = 120;

// ---- Bước mỗi lần bấm
int STEP_ANG = 5;

// ================== Motor helpers ==================
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
  BT.println(msg);
}

// ================== Robot Arm helpers ==================
int clampInt(int v, int vmin, int vmax) {
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

// move mượt (giảm giật SG90)
void moveServoSmooth(Servo &sv, int &cur, int target) {
  target = clampInt(target, 0, 180);
  if (target == cur) return;

  int step = (target > cur) ? 1 : -1;
  while (cur != target) {
    cur += step;
    sv.write(cur);
    delay(8);
  }
}

void shoulderUp() {
  int t = clampInt(shoulderAng + STEP_ANG, SHOULDER_MIN, SHOULDER_MAX);
  moveServoSmooth(servoShoulder, shoulderAng, t);
}
void shoulderDown() {
  int t = clampInt(shoulderAng - STEP_ANG, SHOULDER_MIN, SHOULDER_MAX);
  moveServoSmooth(servoShoulder, shoulderAng, t);
}
void elbowUp() {
  int t = clampInt(elbowAng + STEP_ANG, ELBOW_MIN, ELBOW_MAX);
  moveServoSmooth(servoElbow, elbowAng, t);
}
void elbowDown() {
  int t = clampInt(elbowAng - STEP_ANG, ELBOW_MIN, ELBOW_MAX);
  moveServoSmooth(servoElbow, elbowAng, t);
}

void gripClose() {
  servoGrip.write(GRIP_CLOSE);
  delay(150);
}
void gripOpen() {
  servoGrip.write(GRIP_OPEN);
  delay(150);
}

// ================== AUTO MODE ==================
void autoStep() {
  int d = getDistance();
  if (d <= STOP_DISTANCE) {
    stopAll();
    BT.print(F("[AUTO] STOP distance="));
    BT.print(d);
    BT.println(F("cm"));
    return;
  }

  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  // Tùy cảm biến line: 0/1 có thể đảo. Bạn đang dùng logic như code gốc.
  if (L == 0 && R == 0) goForward();
  else if (L == 0 && R == 1) turnLeft();
  else if (L == 1 && R == 0) turnRight();
  else stopAll();
}

// ================== Handle Commands ==================
void handleCommand(char c) {
  if (c == '\n' || c == '\r') return;
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

  // speed 0..9
  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    speedVal = map(lvl, 0, 9, 80, 255);
    setAllSpeeds(speedVal);
    report(F("[SPD] Speed changed"));
    return;
  }

  switch (c) {

    case 'C':
      mode = MODE_AUTO;
      stopAll();
      report(F("[MODE] AUTO"));
      break;

    case 'K':
      mode = MODE_MANUAL;
      stopAll();
      report(F("[MODE] MANUAL"));
      break;

    // Manual drive
    case 'F':
    case 'B':
    case 'L':
    case 'R':
      if (mode == MODE_MANUAL) {
        if (c == 'F') { goForward();  report(F("Forward")); }
        else if (c == 'B') { goBackward(); report(F("Backward")); }
        else if (c == 'L') { turnLeft();   report(F("Left")); }
        else if (c == 'R') { turnRight();  report(F("Right")); }
      } else {
        report(F("[WARN] Dang AUTO -> Nhan K de thoat"));
      }
      break;

    case 'S':
      stopAll();
      report(F("Stop all"));
      break;

    // ===== Arm (NO BASE) =====
    case 'W':
      shoulderUp();
      report(F("[ARM] SHOULDER UP"));
      break;

    case 'U':
      shoulderDown();
      report(F("[ARM] SHOULDER DOWN"));
      break;

    case 'J':
      elbowUp();
      report(F("[ARM] ELBOW UP"));
      break;

    case 'M':
      elbowDown();
      report(F("[ARM] ELBOW DOWN"));
      break;

    case 'N':
      gripClose();
      report(F("[GRIP] CLOSE"));
      break;

    case 'Q':
      gripOpen();
      report(F("[GRIP] OPEN"));
      break;

    default:
      stopAll();
      report(F("Unknown -> Stop"));
      break;
  }
}

// ================== Setup ==================
void setup() {
  BT.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Attach SG90 with pulse range để đủ hành trình (tùy servo)
  servoShoulder.attach(SERVO_SHOULDER_PIN, SG90_MIN_US, SG90_MAX_US);
  servoElbow.attach(SERVO_ELBOW_PIN, SG90_MIN_US, SG90_MAX_US);
  servoGrip.attach(SERVO_GRIP_PIN, SG90_MIN_US, SG90_MAX_US);

  // Pose ban đầu
  servoShoulder.write(shoulderAng);
  servoElbow.write(elbowAng);
  servoGrip.write(GRIP_OPEN);

  setAllSpeeds(speedVal);
  stopAll();

  BT.println(F("Ready: F/B/L/R | C=AUTO | K=MANUAL | 0..9 speed"));
  BT.println(F("ARM: W/U=Shoulder  J/M=Elbow  N/Q=Grip (SG90)"));
}

// ================== Loop ==================
void loop() {
  if (BT.available()) handleCommand(BT.read());
  if (mode == MODE_AUTO) autoStep();
}
