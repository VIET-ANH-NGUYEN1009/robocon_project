/*
  Arduino UNO R3 + Adafruit Motor Shield v1 + Bluetooth HC-05 (UART D0/D1)
  + IR line (A0/A1) + HC-SR04 (A2/A3) + Servo (D9/D10)

  Điều khiển:
    - W : Nâng (servo 360° – D9)
    - U : Hạ  (servo 360° – D9)
    - N : Gắp (servo 180° – D10)
    - Q : Thả (servo 180° – D10)
    - S : Dừng xe
    - C : AUTO (dò line + Stop @5cm)
    - K : MANUAL (tắt AUTO)
    - F/B/L/R : Tiến/Lùi/Trái/Phải (MANUAL)
    - 0..9 : chỉnh tốc độ (80..255)
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

// ================== Servo ==================
const int SERVO_LIFT_PIN = 9;  
const int SERVO_GRIP_PIN = 10;

Servo servoLift;
Servo servoGrip;

// Servo 360° PWM
int PWM_STOP = 1500;
int PWM_UP   = 1700;
int PWM_DOWN = 1300;

unsigned long LIFT_PULSE_MS = 250;
bool pulseMode = true;
bool liftingActive = false;
unsigned long motionStart = 0;

// Servo 180°
int ANGLE_GRAB     = 120;
int ANGLE_RELEASE  = 60;

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

// ================== Servo helpers ==================
void stopLift() {
  servoLift.writeMicroseconds(PWM_STOP);
}

void startLiftUp() {
  servoLift.writeMicroseconds(PWM_UP);
  liftingActive = true;
  motionStart = millis();
}

void startLiftDown() {
  servoLift.writeMicroseconds(PWM_DOWN);
  liftingActive = true;
  motionStart = millis();
}

void grab() {
  servoGrip.write(ANGLE_GRAB);
  delay(150);
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
    BT.print(F("[AUTO] STOP distance="));
    BT.print(d);
    BT.println(F("cm"));
    return;
  }

  int L = digitalRead(IR_LEFT);
  int R = digitalRead(IR_RIGHT);

  if (L == 0 && R == 0) goForward();
  else if (L == 0 && R == 1) turnLeft();
  else if (L == 1 && R == 0) turnRight();
  else stopAll();
}

// ================== Handle Commands ==================
void handleCommand(char c) {
  if (c == '\n' || c == '\r') return;
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    speedVal = map(lvl, 0, 9, 80, 255);
    setAllSpeeds(speedVal);
    report(F("[SPD] Speed changed"));
    return;
  }

  switch (c) {

    // ---------- ĐÃ SỬA LỆNH ----------
    case 'C':   // Bật AUTO
      mode = MODE_AUTO;
      stopAll();
      report(F("[MODE] AUTO"));
      break;

    case 'K':   // Tắt AUTO
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
        if (c == 'F') { goForward(); report(F("Forward")); }
        else if (c == 'B') { goBackward(); report(F("Backward")); }
        else if (c == 'L') { turnLeft(); report(F("Left")); }
        else if (c == 'R') { turnRight(); report(F("Right")); }
      } else {
        report(F("[WARN] Đang AUTO → Nhấn K để thoát"));
      }
      break;

    case 'S':
      stopAll();
      stopLift();
      liftingActive = false;
      report(F("Stop all"));
      break;

    case 'W':
      startLiftUp();
      report(F("[LIFT] UP"));
      break;

    case 'U':
      startLiftDown();
      report(F("[LIFT] DOWN"));
      break;

    case 'N':
      grab();
      report(F("[GRIP] GRAB"));
      break;

    case 'Q':
      releaseObject();
      report(F("[GRIP] RELEASE"));
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

  servoLift.attach(SERVO_LIFT_PIN);
  servoGrip.attach(SERVO_GRIP_PIN);

  stopLift();
  servoGrip.write(ANGLE_RELEASE);

  setAllSpeeds(speedVal);
  stopAll();

  BT.println(F("Ready: F/B/L/R | C=AUTO | K=MANUAL | 0..9 speed"));
  BT.println(F("Lift: W=UP, U=DOWN | Grip: N=GRAB, Q=RELEASE"));
}

// ================== Loop ==================
void loop() {
  if (BT.available()) handleCommand(BT.read());

  if (mode == MODE_AUTO) autoStep();

  if (pulseMode && liftingActive) {
    if (millis() - motionStart >= LIFT_PULSE_MS) {
      stopLift();
      liftingActive = false;
    }
  }
}
