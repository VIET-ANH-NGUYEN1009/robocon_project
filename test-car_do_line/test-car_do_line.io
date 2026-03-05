#include <NewPing.h>
#include <AFMotor.h>

// ===== HC-SR04 =====
#define TRIGGER_PIN A2
#define ECHO_PIN    A3
#define MAX_DISTANCE 50

#define STOP_DISTANCE 5   // dừng khi vật cách 5cm

// ===== IR SENSOR =====
#define irLeft  A0
#define irRight A1

// ===== MOTOR =====
#define BASE_SPEED 120

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  motor1.setSpeed(BASE_SPEED);
  motor2.setSpeed(BASE_SPEED);
  motor3.setSpeed(BASE_SPEED);
  motor4.setSpeed(BASE_SPEED);

  Stop();
  Serial.println("Line follow + Stop at 5cm (No avoid)");
}

// ================= MAIN LOOP =================
void loop() {

  // --- kiểm tra vật cản ---
  int distance = getDistance();
  if (distance <= STOP_DISTANCE) {
    Stop();
    Serial.println("STOP! Object < 5cm");
    return; // không chạy dò line nữa, đứng im
  }

  // --- đọc IR ---
  int L = digitalRead(irLeft);
  int R = digitalRead(irRight);

  // ========= LINE FOLLOW =========
  if (L == 0 && R == 0) {
    moveForward();
    Serial.println("Forward");
  }
  else if (L == 0 && R == 1) {
    moveLeft();
    Serial.println("Left");
  }
  else if (L == 1 && R == 0) {
    moveRight();
    Serial.println("Right");
  }
  else if (L == 1 && R == 1) {
    Stop();
    Serial.println("Stop (lost line)");
  }
}

// ================= GET DISTANCE =================
int getDistance() {
  delay(40);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 100;  // không bắt echo = xa
  return cm;
}

// ================= MOTOR CONTROL =================
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}
