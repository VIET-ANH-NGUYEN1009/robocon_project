#include <SoftwareSerial.h>

const uint8_t BT_RX = 10; // Arduino nhận từ HC-05 TXD
const uint8_t BT_TX = 11; // Arduino gửi tới HC-05 RXD (qua chia áp)
SoftwareSerial BT(BT_RX, BT_TX); // RX, TX

const uint8_t LED_PIN = 13; // LED tích hợp trên Uno

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);      // Serial USB để debug
  BT.begin(9600);          // Mặc định HC-05 data mode là 9600

  Serial.println(F("=== Test HC-05: Echo & LED ==="));
  Serial.println(F("Ghep doi tren dien thoai: Ten 'HC-05' - PIN thuong la 1234 hoac 0000"));
  Serial.println(F("Mo app Serial Bluetooth, ket noi, go chuoi -> se nhan lai (echo)."));
}

void loop() {
  // Nếu có dữ liệu từ HC-05 -> gửi lên Serial và nháy LED
  if (BT.available()) {
    char c = BT.read();
    Serial.write(c);

    // Nháy LED mỗi khi nhận ký tự
    digitalWrite(LED_PIN, HIGH);
    delay(5);
    digitalWrite(LED_PIN, LOW);

    // Echo lại về Bluetooth
    BT.write(c);
  }

  // Nếu có dữ liệu từ USB Serial (PC) -> gửi sang HC-05
  if (Serial.available()) {
    char c = Serial.read();
    BT.write(c);
  }
}