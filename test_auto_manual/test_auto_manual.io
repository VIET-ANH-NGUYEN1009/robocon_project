#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// ================== Bluetooth ==================
SoftwareSerial BT(10, 11); // HC-05: RX=10, TX=11

// ================== IR line sensors ==================
#define IR_LEFT   A0
#define IR_RIGHT  A1

// ================== HC-SR04 ==================
#define TRIGGER_PIN  A2
#define ECHO_PIN     A3
#define MAX_DISTANCE 50
#define STOP_DISTANCE 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================== Motors ==================
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
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

  // ===== Tốc độ =====
  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    int newSpeed = map(lvl, 0, 9, 80, 255);
    speedVal = newSpeed;
    setAllSpeeds(speedVal);
    report(F("[SPD] Speed changed"));
    return;
  }

  switch (c) {
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

// ================== Setup ==================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(LED_PIN, OUTPUT);

  setAllSpeeds(speedVal);
  stopAll();

  digitalWrite(LED_PIN, HIGH); // default MANUAL: sáng
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
}
