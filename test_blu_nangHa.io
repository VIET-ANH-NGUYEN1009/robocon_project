#include <Servo.h>
#include <SoftwareSerial.h>

// ==== Cấu hình chân ====
const int PIN_SERVO_360 = 5;  
const int PIN_SERVO_180 = 6;   
const int BT_RX = 10;   
const int BT_TX = 11;          

// ==== Bluetooth ====
SoftwareSerial BT(BT_RX, BT_TX); // RX, TX

// ==== Servo ====
Servo servo360; 
Servo servo180;

// ==== Tham số hiệu chuẩn (có thể chỉnh) ====
// Servo 360° dùng giá trị microseconds: ~1500 = dừng, <1500 quay một chiều, >1500 quay chiều ngược
int PWM_STOP = 1500;
int PWM_UP   = 1700; 
int PWM_DOWN = 1300;

// Thời gian xoay nâng/hạ (ms) nếu muốn chạy theo nhịp rồi dừng tự động
unsigned long LIFT_PULSE_MS = 250; 

// Góc servo 180° cho gắp/thả
int ANGLE_GRAB   = 120; 
int ANGLE_RELEASE= 60;  

// Bật/tắt chế độ “nhấn là chạy theo nhịp rồi tự dừng”
bool pulseMode = true;

// ==== Trạng thái ====
char lastCmd = '\0';
unsigned long motionStart = 0;
bool liftingActive = false;

void setup() {
  Serial.begin(9600);
  BT.begin(9600); 

  servo360.attach(PIN_SERVO_360);
  servo180.attach(PIN_SERVO_180);

  // Dừng servo 360, đặt kẹp ở trạng thái thả
  stopLift();
  servo180.write(ANGLE_RELEASE);

  Serial.println(F("Ready: W=nang, u=ha, N=gap, q=tha"));
  Serial.println(F("Goi y: co the giu phim hoac dung pulseMode = true de auto-dung."));
}

void loop() {
  // Đọc từ Bluetooth
  if (BT.available()) {
    char c = (char)BT.read();
    handleCommand(c);
  }

  // Đọc từ Serial (debug / test trên PC)
  if (Serial.available()) {
    char c = (char)Serial.read();
    handleCommand(c);
  }

  // Nếu đang ở chế độ chạy theo nhịp (pulseMode) thì tự dừng sau LIFT_PULSE_MS
  if (pulseMode && liftingActive) {
    if (millis() - motionStart >= LIFT_PULSE_MS) {
      stopLift();
      liftingActive = false;
    }
  }
}

void handleCommand(char c) {

  switch (c) {
    case 'W': // NÂNG
    case 'w':
      startLiftUp();
      lastCmd = c;
      feedback("NANG");
      break;

    case 'u': // HẠ
    case 'U':
      startLiftDown();
      lastCmd = c;
      feedback("HA");
      break;

    case 'N': // GẮP
      grab();
      lastCmd = c;
      feedback("GAP");
      break;

    case 'q': // THẢ
    case 'Q':
      releaseObject();
      lastCmd = c;
      feedback("THA");
      break;

    case 'S': // DỪNG KHẨN cấp cho nâng/hạ (tùy chọn)
    case 's':
      stopLift();
      lastCmd = c;
      feedback("STOP");
      break;

    case 'P': // Bật/tắt pulseMode (tùy chọn)
    case 'p':
      pulseMode = !pulseMode;
      feedback(pulseMode ? "PULSE=ON" : "PULSE=OFF");
      break;

    default:
      // Bỏ qua ký tự khác (như newline)
      break;
  }
}

// ==== HÀM ĐIỀU KHIỂN NÂNG/HẠ (SERVO 360) ====
void startLiftUp() {
  servo360.writeMicroseconds(PWM_UP);
  liftingActive = true;
  motionStart = millis();
  if (!pulseMode) liftingActive = false; 
}

void startLiftDown() {
  servo360.writeMicroseconds(PWM_DOWN);
  liftingActive = true;
  motionStart = millis();
  if (!pulseMode) liftingActive = false;
}

void stopLift() {
  servo360.writeMicroseconds(PWM_STOP);
}

// ==== HÀM GẮP/THẢ (SERVO 180) ====
void grab() {
  servo180.write(ANGLE_GRAB);
  delay(200); 
}

void releaseObject() {
  servo180.write(ANGLE_RELEASE);
  delay(200);
}

// ==== PHẢN HỒI QUA SERIAL/BT ====
void feedback(const char* msg) {
  Serial.println(msg);
  BT.println(msg);
}
