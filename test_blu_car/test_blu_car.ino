#include <AFMotor.h>
#include <SoftwareSerial.h>
//https://github.com/adafruit/Adafruit-Motor-Shield-library

// SoftwareSerial: RX từ HC-05 là chân 10, TX ra HC-05 là chân 11
SoftwareSerial BT(10, 11); // RX, TX

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int speedVal = 230;
char value = 'S';

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

void setup() {
  Serial.begin(9600);   
  BT.begin(9600);      
  setAllSpeeds(speedVal);
  stopAll();
  Serial.println(F("Ready (USB) & BT. Commands: F,B,L,R,S and 0..9 for speed"));
}

void handleCommand(char c) {
  if (c == '\n' || c == '\r') return;
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

  // Phím số đổi tốc độ
  if (c >= '0' && c <= '9') {
    int lvl = c - '0';
    int newSpeed = map(lvl, 0, 9, 80, 255);
    speedVal = newSpeed;
    setAllSpeeds(speedVal);
    Serial.print(F("[SPD] ")); Serial.println(speedVal);
    BT.print(F("[SPD] "));    BT.println(speedVal);
    return;
  }

  value = c;
  switch (value) {
    case 'F': goForward(); Serial.println(F("Forward")); BT.println(F("Forward")); break;
    case 'B': goBackward(); Serial.println(F("Backward")); BT.println(F("Backward")); break;
    case 'L': turnLeft();   Serial.println(F("Left"));     BT.println(F("Left"));     break;
    case 'R': turnRight();  Serial.println(F("Right"));    BT.println(F("Right"));    break;
    case 'S': stopAll();    Serial.println(F("Stop"));     BT.println(F("Stop"));     break;
    default:  stopAll();    Serial.println(F("Unknown -> Stop")); BT.println(F("Unknown -> Stop")); break;
  }
}

void loop() {
  // Nhận từ Bluetooth
  if (BT.available()) {
    char c = BT.read();
    handleCommand(c);
  }
  // (Tuỳ chọn) Nhận từ USB Serial để test
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }
}