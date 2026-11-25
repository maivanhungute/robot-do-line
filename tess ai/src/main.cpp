#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==============================================================================
// ESP32 CHÍNH: WIFI WEB-TUNING + ĐIỀU KHIỂN MOTOR L298N + THUẬT TOÁN ĐUA LINE
// Nhận dữ liệu 8 mắt siêu tốc từ ESP32 Phụ qua UART2 (921600 baud, trễ < 0.2ms)
// 3 NÚT BẤM NGOÀI (Trở kéo Mass -> Active HIGH) + CÒI CHÍP BUZZER AUDIO FEEDBACK
// TÍNH NĂNG CHẨN ĐOÁN TEST TỪNG MOTOR TRỰC TIẾP QUA WEB & BÙ BÃO HÒA ĐỘNG CƠ
// ==============================================================================

#define AP_PASSWORD "linecar123"  // mật khẩu WiFi của xe (>= 8 ký tự)

// Khai báo trước các hàm (Forward Declarations)
void saveSettings();
void loadSettings();
void resetDefaultSettings();
void stopMotors();
void setMotor(int leftSpeed, int rightSpeed);
void sendCommandToSub(uint8_t cmd);
void emergencyStop();
void startRace();
void setGear(int gear);
void cycleGear();
void triggerCali();
void updateCaliProcess();
void checkExternalButtons();
void checkButton();
void triggerBeep(int count, int msOn, int msOff = 50);
void updateBuzzer();
void onSample(unsigned long idx);
void webTask(void *);

// 1. CẤU HÌNH PHẦN CỨNG ĐỘNG CƠ L298N
// Motor Bên Trái (Điều khiển ngõ ra OUT1, OUT2 của L298N):
#define IN1 19  // D19 - Motor Trái Tiến (IN1=PWM, IN2=0) -> OUT1/OUT2
#define IN2 21  // D21 - Motor Trái Lùi  (IN1=0, IN2=PWM) -> OUT1/OUT2

// Motor Bên Phải (Điều khiển ngõ ra OUT3, OUT4 của L298N):
#define IN3 25  // D25 - Motor Phải Tiến (IN3=PWM, IN4=0) -> OUT3/OUT4
#define IN4 26  // D26 - Motor Phải Lùi  (IN3=0, IN4=PWM) -> OUT3/OUT4

#define LED_PIN 2     // Đèn LED báo trạng thái
#define BTN_PIN 0     // Nút BOOT trên mạch

// CẤU HÌNH 3 NÚT BẤM NGOÀI (ĐIỆN TRỞ KÉO XUỐNG MASS -> BẤM NÚT = HIGH)
#define PIN_BTN1 22   // Nút 1: Xuất phát / Dừng khẩn cấp (Ăn ngay, không độ trễ)
#define PIN_BTN2 23   // Nút 2: Nhấn nhả = Căn tâm | Giữ 1.2s = Cali 5s
#define PIN_BTN3 4    // Nút 3: Hộp số 3 cấp tốc độ (160 / 190 / 230)
#define BUZZER_PIN 27 // Còi chíp báo trạng thái (Buzzer)

// Cấu hình UART2 kết nối với ESP32 Phụ (xuly_tinhieu)
#define UART_RX2 16  // Nối với TX2 (GPIO 17) của ESP32 Phụ
#define UART_TX2 17  // Nối với RX2 (GPIO 16) của ESP32 Phụ
#define UART_BAUD 921600 // Tốc độ cực cao 921600 baud đồng bộ cả 2 con

// ==============================================================================
// 2. CẤU TRÚC GÓI TIN DỮ LIỆU CẢM BIẾN (ĐỒNG BỘ 100% VỚI ESP32 PHỤ)
// ==============================================================================
#pragma pack(push, 1)
struct SensorDataPacket {
  uint8_t  header;        // 0xAA: Byte nhận diện
  uint8_t  sensorBits;    // 8 bit biểu diễn 8 mắt (Bit 0 = S0 ... Bit 7 = S7)
  int16_t  error_x1000;   // Sai số vị trí (-3500 đến +3500)
  uint16_t sumWeight;     // Tổng trọng số bắt vạch (0 - 8000)
  uint8_t  activeCount;   // Số mắt đang chạm vạch (0 - 8)
  uint8_t  weights[8];    // Độ đậm từng mắt (0 - 250)
  uint8_t  isCalibrating; // 1: đang cali, 0: bình thường
  uint8_t  seq;           // Số thứ tự gói (tăng dần, tự quay vòng) -> phát hiện mất gói
  uint8_t  checksum;      // Byte kiểm tra toàn vẹn
};
#pragma pack(pop)

SensorDataPacket rxPacket;
unsigned long lastPacketTime = 0;
unsigned long totalPacketsReceived = 0;
bool subMcuConnected = false;
bool newSample = false;              // có mẫu cảm biến MỚI chưa được vòng PD dùng
unsigned long sampleUs = 0;          // thời điểm nhận mẫu mới nhất
unsigned long samplePrevUs = 0;      // thời điểm nhận mẫu trước đó (để tính dt cho đạo hàm)
unsigned long lostPackets = 0;       // số gói bị mất (phát hiện qua số thứ tự seq) trong lượt chạy
unsigned long maxLoopUs = 0;         // thời gian lớn nhất giữa 2 vòng loop() khi đang chạy (micro giây)
uint8_t lastSeq = 0;
bool haveSeq = false;
unsigned long sampleIdx = 0;         // số thứ tự mẫu (tăng theo seq, kể cả gói bị mất) -> đồng hồ mẫu ~1ms/mẫu

// Dữ liệu cảm biến trích xuất từ gói tin
const int NUM_SENSORS = 8;
int s[NUM_SENSORS];            // 0: trắng, 1: đen
int sensorWeight[NUM_SENSORS]; // 0 -> 1000
int activeCount = 0;
long sumWeight = 0;
float error = 0.0f;
float lastError = 0.0f;
float dFiltered = 0.0f;
float currentSteer = 0.0f;

// ==============================================================================
// 3. THÔNG SỐ VẬN HÀNH (ĐƯỢC LƯU FLASH & ĐIỀU CHỈNH QUA WIFI WEB)
// ==============================================================================
int   baseSpeed          = 190;   // Tốc độ cơ bản bám line (100 - 255)
int   spinSpeed          = 140;   // Tốc độ xoay compa tìm vạch (90 - 220)
int   pivotFwd           = 180;   // Bánh ngoài khi ôm cua 90 độ (120 - 255)
int   pivotRev           = -45;   // Bánh trong khi ôm cua 90 độ (-100 - 0)
int   centerTestSpd      = 90;    // Tốc độ xoay khi test căn tâm (70 - 255)
int   brakePwm           = 100;   // Lực phanh ngược dập quán tính xoay (80 - 200)
int   brakeTime          = 14;    // Thời gian xung phanh ngược (10 - 40 ms)

float Kp                 = 24.0f; // Hệ số tỉ lệ P (Độ nhạy lái mượt mà, phản hồi tức thời)
float Kd                 = 15.0f; // Hệ số vi phân D, tính theo tốc độ đổi sai số trên 10ms (Dập tắt quán tính lắc xe)
float STEER_SMOOTH       = 0.75f; // Làm mịn phản hồi bẻ lái (0.75 mượt mà và nhạy)

bool  INVERT_LEFT        = false; // Đảo chiều Motor Trái
bool  INVERT_RIGHT       = false; // Đảo chiều Motor Phải (Mặc định false để IN1 và IN3 cùng cấp điện khi chạy tiến)
bool  ENABLE_SPIN_SEARCH = true;  // Khi mất vạch quá lâu: rẽ về phía ưu tiên để tìm lại vạch (tắt = dừng xe)

// --- CẤU HÌNH LỘ TRÌNH (lưu Flash, chỉnh qua web) ---
enum Act : uint8_t { ACT_STRAIGHT = 0, ACT_LEFT = 1, ACT_RIGHT = 2 };
const int MAX_FLAGS = 8;
// Hành động tại từng cờ (cờ 1..8) cho BẢN ĐỒ GỐC. Bản lật tự đảo trái/phải.
// Mặc định: cờ1 PHẢI (U->thanh ngang), cờ2 THẲNG (chân T cụt), cờ3 PHẢI (sống E), cờ4 THẲNG (thanh giữa cụt), cờ5 TRÁI (góc đáy)
uint8_t flagAct[MAX_FLAGS] = {ACT_RIGHT, ACT_STRAIGHT, ACT_RIGHT, ACT_STRAIGHT, ACT_LEFT, ACT_STRAIGHT, ACT_STRAIGHT, ACT_STRAIGHT};
const uint32_t DEFAULT_FLAGS_PACKED = 290;  // mã hóa 2 bit/cờ của bảng mặc định ở trên
int   mapMode   = 0;      // 0 = tự nhận ở cờ đầu | 1 = bản đồ GỐC | 2 = bản đồ LẬT NGANG
int   gapHoldMs = 250;    // Mất vạch quá từng này ms thì không giữ lái nữa mà rẽ về phía ưu tiên (đoạn đứt nét chữ U)
int   lostDir   = -1;     // Phía ưu tiên khi mất vạch ở BẢN GỐC: -1 = trái (cua U hướng trái), +1 = phải. Bản lật tự đảo

uint32_t packFlags() {
  uint32_t v = 0;
  for (int i = 0; i < MAX_FLAGS; i++) v |= ((uint32_t)(flagAct[i] & 3)) << (2 * i);
  return v;
}

void unpackFlags(uint32_t v) {
  for (int i = 0; i < MAX_FLAGS; i++) {
    uint8_t a = (v >> (2 * i)) & 3;
    flagAct[i] = (a > ACT_RIGHT) ? (uint8_t)ACT_STRAIGHT : a;
  }
}

// --- THÔNG SỐ THỜI GIAN / NGƯỠNG LỘ TRÌNH (chỉnh trực tiếp trên web, lưu Flash, áp dụng ngay) ---
int wideMinActive     = 5;    // >= từng này mắt cùng đen mới coi là vạch ngang (vạch thường chỉ phủ 2-3 mắt)
int wideConfirm       = 2;    // số mẫu liên tiếp để xác nhận 1 mốc (lọc nhiễu)
int rearmMs           = 60;   // hết vạch ngang đủ từng này mẫu (~ms) mới đếm mốc kế (chống đếm đôi)
int turnAdvMs         = 60;   // chạy thẳng thêm để TRỤC BÁNH tới tâm ngã rẽ (= khoảng cách dàn mắt-trục / vận tốc)
int turnAdvSpeed      = 150;  // PWM khi chạy thẳng thêm
int preBrakeMs        = 12;   // xung phanh lùi dập quán tính trước khi quay
int preBrakePwm       = 90;
int turnMinMs         = 40;   // thời gian quay tối thiểu
int turnMaxMs         = 600;  // quay quá lâu mà không bắt được vạch thì thôi
int gapSpeedPct       = 85;   // % tốc độ khi vượt đoạn đứt nét
int lostArcInnerPct   = 30;   // mất vạch quá gapHoldMs: bánh trong chạy % tốc độ xoay -> vòng cung, KHÔNG xoay tại chỗ
int lostTurnMaxMs     = 1500; // vòng cung tìm vạch tối đa từng này ms rồi dừng xe
int finishRollMs      = 80;   // chặng cuối mất vạch -> lăn thêm rồi dừng (vào ô FINISH)
int turnApproachSpeed = 150;  // PWM khi sắp tới mốc RẼ (đặt 255 để tắt giảm ga)
int slowStartPct      = 55;   // chạy hết ga tới % thời gian đoạn trước rồi mới giảm ga
int firstSlowAfterMs  = 0;    // đoạn START->mốc 1 (cua U): sau từng này ms mới giảm ga

struct TParam { const char *key; const char *name; int *ptr; int mn; int mx; int def; };
TParam tparams[] = {
  {"wmin",  "Số mắt tối thiểu coi là vạch ngang",         &wideMinActive,     3,   8,    5},
  {"wcnf",  "Số mẫu xác nhận 1 mốc",                       &wideConfirm,       1,   10,   2},
  {"rearm", "Chống đếm đôi (mẫu ≈ ms)",                    &rearmMs,           10,  300,  60},
  {"tadv",  "Chạy thêm trước khi rẽ (ms)",                 &turnAdvMs,         0,   300,  60},
  {"tadvs", "PWM khi chạy thêm",                           &turnAdvSpeed,      60,  255,  150},
  {"pbms",  "Xung phanh trước khi quay (ms)",              &preBrakeMs,        0,   60,   12},
  {"pbpwm", "PWM phanh trước khi quay",                    &preBrakePwm,       40,  255,  90},
  {"tmin",  "Thời gian quay tối thiểu (ms)",               &turnMinMs,         0,   300,  40},
  {"tmax",  "Thời gian quay tối đa (ms)",                  &turnMaxMs,         100, 1500, 600},
  {"gspd",  "Tốc độ qua đoạn đứt nét (%)",                 &gapSpeedPct,       30,  100,  85},
  {"lain",  "Bánh trong khi tìm vạch (% tốc độ xoay)",     &lostArcInnerPct,   0,   100,  30},
  {"ltmx",  "Thời gian tìm vạch tối đa (ms)",              &lostTurnMaxMs,     200, 4000, 1500},
  {"froll", "Lăn thêm khi tới đích (ms)",                  &finishRollMs,      0,   500,  80},
  {"tapp",  "Tốc độ tiếp cận mốc rẽ (PWM)",                &turnApproachSpeed, 80,  255,  150},
  {"sslw",  "Bắt đầu giảm ga ở % thời gian đoạn trước",    &slowStartPct,      10,  100,  55},
  {"fslw",  "Giảm ga đoạn đầu (U) sau (ms)",               &firstSlowAfterMs,  0,   5000, 0},
};
const int TPARAM_N = sizeof(tparams) / sizeof(tparams[0]);

// Bộ nhớ góc cua 90 độ
int   lastLineSide       = 0;
unsigned long timeLostLine = 0;
float currentSpeed       = 190.0f;

// Trạng thái vận hành
bool isRunning           = false;
bool isStartingCountdown = false;
unsigned long startCountdownTimer = 0;
const unsigned long START_DELAY_MS = 650; // 650ms: đủ cho 3 nhịp bíp nhanh để kịp buông tay thoải mái!
bool isTestCenteringMode = false;
bool isCenteredLocked    = false; // Trạng thái khóa tâm chống lắc qua lại
int  lastBrakeDir        = 0;
int  lastMotorL          = 0;
int  lastMotorR          = 0;
int  currentGear         = 2;     // Hộp số mặc định: Cấp 2 (190)

// Quản lý đếm nhịp Cali 5 giây
bool isCalibratingLocal  = false;
unsigned long calibStartTime = 0;
bool calibAcked = false;             // ESP32 Phụ đã xác nhận vào chế độ cali (qua cờ isCalibrating trong gói tin)
unsigned long lastCaliCmdMs = 0;

// Quản lý Test Motor riêng biệt (2 giây)
bool isMotorTesting      = false;
unsigned long motorTestTimer = 0;

Preferences prefs;
WebServer server(80);
bool settingsDirty = false;          // có thay đổi cần ghi Flash; chỉ ghi khi xe đứng yên (ghi Flash làm khựng vòng điều khiển)

// ==============================================================================
// 4. ÂM THANH CÒI BÁO PHI CHẶN (NON-BLOCKING BUZZER ENGINE) & HỘP SỐ TỐC ĐỘ
// Hoàn toàn không dùng delay(), loại bỏ 100% tình trạng khựng/giật/mất gói UART
// ==============================================================================
int buzzerPatternCount = 0;
int buzzerBeepOnMs = 0;
int buzzerBeepOffMs = 0;
unsigned long buzzerTimer = 0;
bool buzzerIsOn = false;

void triggerBeep(int count, int msOn, int msOff) {
  if (count <= 0) return;
  buzzerPatternCount = count;
  buzzerBeepOnMs = msOn;
  buzzerBeepOffMs = msOff;
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerIsOn = true;
  buzzerTimer = millis();
}

void updateBuzzer() {
  if (buzzerPatternCount <= 0) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
    }
    return;
  }
  unsigned long elapsed = millis() - buzzerTimer;
  if (buzzerIsOn) {
    if (elapsed >= (unsigned long)buzzerBeepOnMs) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      buzzerTimer = millis();
      buzzerPatternCount--;
    }
  } else {
    if (elapsed >= (unsigned long)buzzerBeepOffMs) {
      if (buzzerPatternCount > 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerIsOn = true;
        buzzerTimer = millis();
      }
    }
  }
}

void setGear(int gear) {
  currentGear = constrain(gear, 1, 3);
  if (currentGear == 1) {
    baseSpeed = 160;
    triggerBeep(1, 100, 50); // Số 1: 1 bíp phi chặn
  } else if (currentGear == 2) {
    baseSpeed = 190;
    triggerBeep(2, 70, 50);  // Số 2: 2 bíp phi chặn
  } else {
    baseSpeed = 230;
    triggerBeep(3, 50, 40);  // Số 3: 3 bíp phi chặn
  }
  currentSpeed = (float)baseSpeed;
  settingsDirty = true;
  Serial.printf(">>> [HỘP SỐ] Chuyển sang CẤP %d (Tốc độ Base: %d)\n", currentGear, baseSpeed);
}

void cycleGear() {
  int nextGear = currentGear + 1;
  if (nextGear > 3) nextGear = 1;
  setGear(nextGear);
}

void startRace() {
  if (isRunning || isStartingCountdown) return;
  isTestCenteringMode = false;
  isCenteredLocked = false;
  isMotorTesting = false;

  stopMotors(); // Đảm bảo xe đứng yên tuyệt đối khi vừa bấm nút

  // Còi đếm nhịp 3 tiếng bíp nhanh để người vận hành kịp buông tay ra!
  triggerBeep(3, 80, 70);

  isStartingCountdown = true;
  startCountdownTimer = millis();

  digitalWrite(LED_PIN, LOW);
  Serial.println(">>> [CHUẨN BỊ XUẤT PHÁT] Đếm 3 bíp nhanh... Hãy buông tay ra!");
}

void emergencyStop() {
  isStartingCountdown = false; // Hủy đếm ngược nếu đang trong lúc đếm
  isRunning = false;
  isTestCenteringMode = false;
  isCenteredLocked = false;
  isCalibratingLocal = false;
  isMotorTesting = false;
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  triggerBeep(1, 200); // 1 bíp dài phi chặn báo dừng an toàn
  Serial.println(">>> [DỪNG KHẨN CẤP] Đã phanh xe lập tức!");
}

void triggerCali() {
  if (isRunning || isStartingCountdown) emergencyStop();   // không cali khi xe đang chạy (dữ liệu cảm biến lúc cali không dùng để lái)
  isCalibratingLocal = true;
  calibAcked = false;
  calibStartTime = millis();
  lastCaliCmdMs = millis();
  sendCommandToSub(0xC1); // Gửi lệnh bắt đầu cali sang ESP32 Phụ (gửi lại tới khi Phụ xác nhận)
  triggerBeep(1, 120);    // 1 bíp bắt đầu phi chặn
  Serial.println(">>> [CALI CẢM BIẾN] Bắt đầu 5 giây! Hãy rê dàn mắt qua vạch đen...");
}

void updateCaliProcess() {
  if (!isCalibratingLocal) return;

  unsigned long now = millis();

  if (!calibAcked) {
    // Chờ ESP32 Phụ xác nhận đã vào cali (cờ isCalibrating trong gói tin); chưa thấy thì gửi lại lệnh
    if (rxPacket.isCalibrating) {
      calibAcked = true;
    } else if (now - calibStartTime >= 1500) {
      isCalibratingLocal = false;
      triggerBeep(3, 40, 40);
      Serial.println(">>> [CALI CẢM BIẾN] LỖI: ESP32 Phụ không phản hồi lệnh cali!");
    } else if (now - lastCaliCmdMs >= 100) {
      lastCaliCmdMs = now;
      sendCommandToSub(0xC1);
    }
    return;
  }

  // ESP32 Phụ tự kết thúc sau 5s và báo isCalibrating = 0 -> xong, đã lưu Flash bên Phụ
  if (!rxPacket.isCalibrating) {
    isCalibratingLocal = false;
    triggerBeep(2, 100, 60); // 2 bíp phi chặn báo xong!
    Serial.println(">>> [CALI CẢM BIẾN] ESP32 Phụ báo xong & đã lưu Flash NVS!");
  } else if (now - calibStartTime >= 7000) {
    sendCommandToSub(0xC2); // quá giờ mà Phụ chưa tự kết thúc: ép kết thúc
  }
}

// ==============================================================================
// 5. ĐIỀU KHIỂN ĐỘNG CƠ L298N
// ==============================================================================
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  lastMotorL = leftSpeed;
  lastMotorR = rightSpeed;

  if (INVERT_LEFT)  leftSpeed  = -leftSpeed;
  if (INVERT_RIGHT) rightSpeed = -rightSpeed;

  // Motor Trái (IN1, IN2)
  if (leftSpeed > 0) {
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0);
  } else if (leftSpeed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, -leftSpeed);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }

  // Motor Phải (IN3, IN4)
  if (rightSpeed > 0) {
    analogWrite(IN3, rightSpeed);
    analogWrite(IN4, 0);
  } else if (rightSpeed < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, -rightSpeed);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
}

void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  lastMotorL = 0;
  lastMotorR = 0;
}

// ==============================================================================
// 6. LƯU & TẢI THÔNG SỐ TỪ BỘ NHỚ FLASH PREFERENCES
// ==============================================================================
void saveSettings() {
  prefs.begin("car_cfg", false);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("kd2", Kd);
  prefs.putInt("base", baseSpeed);
  prefs.putInt("spin", spinSpeed);
  prefs.putInt("pfwd", pivotFwd);
  prefs.putInt("prev", pivotRev);
  prefs.putInt("cspd", centerTestSpd);
  prefs.putInt("bpwm", brakePwm);
  prefs.putInt("btim", brakeTime);
  prefs.putFloat("smooth", STEER_SMOOTH);
  prefs.putBool("invL", INVERT_LEFT);
  prefs.putBool("invR", INVERT_RIGHT);
  prefs.putBool("spinSrch", ENABLE_SPIN_SEARCH);
  prefs.putInt("gear", currentGear);
  prefs.putInt("map", mapMode);
  prefs.putInt("gap", gapHoldMs);
  prefs.putInt("ldir", lostDir);
  prefs.putUInt("flg", packFlags());
  for (int i = 0; i < TPARAM_N; i++) prefs.putInt(tparams[i].key, *tparams[i].ptr);
  prefs.end();
  Serial.println(">>> [FLASH NVS] Đã lưu thông số xe vào Flash!");
}

// Chỉ ghi Flash khi xe thật sự đứng yên: tránh khựng vòng điều khiển giữa lúc đang chạy
void flushSettingsIfIdle() {
  if (!settingsDirty) return;
  if (isRunning || isStartingCountdown || isTestCenteringMode || isMotorTesting || isCalibratingLocal) return;
  settingsDirty = false;
  saveSettings();
}

void loadSettings() {
  prefs.begin("car_cfg", true);
  Kp                 = prefs.getFloat("kp", 24.0f);
  Kd                 = prefs.getFloat("kd2", 15.0f);
  baseSpeed          = prefs.getInt("base", 190);
  spinSpeed          = prefs.getInt("spin", 140);
  pivotFwd           = prefs.getInt("pfwd", 180);
  pivotRev           = prefs.getInt("prev", -45);
  centerTestSpd      = prefs.getInt("cspd", 90);
  brakePwm           = prefs.getInt("bpwm", 100);
  brakeTime          = prefs.getInt("btim", 14);
  STEER_SMOOTH       = prefs.getFloat("smooth", 0.75f);
  INVERT_LEFT        = prefs.getBool("invL", false);
  INVERT_RIGHT       = prefs.getBool("invR", false);
  ENABLE_SPIN_SEARCH = prefs.getBool("spinSrch", true);
  currentGear        = prefs.getInt("gear", 2);
  mapMode            = constrain(prefs.getInt("map", 0), 0, 2);
  gapHoldMs          = constrain(prefs.getInt("gap", 250), 0, 2000);
  lostDir            = (prefs.getInt("ldir", -1) >= 0) ? +1 : -1;
  unpackFlags(prefs.getUInt("flg", DEFAULT_FLAGS_PACKED));
  for (int i = 0; i < TPARAM_N; i++) *tparams[i].ptr = constrain(prefs.getInt(tparams[i].key, tparams[i].def), tparams[i].mn, tparams[i].mx);
  prefs.end();
  currentSpeed = (float)baseSpeed;
  Serial.println(">>> [FLASH NVS] Đã nạp thông số từ Flash thành công!");
}

void resetDefaultSettings() {
  Kp                 = 24.0f;
  Kd                 = 15.0f;
  baseSpeed          = 190;
  spinSpeed          = 140;
  pivotFwd           = 180;
  pivotRev           = -45;
  centerTestSpd      = 90;
  brakePwm           = 100;
  brakeTime          = 14;
  STEER_SMOOTH       = 0.75f;
  INVERT_LEFT        = false;
  INVERT_RIGHT       = false;
  ENABLE_SPIN_SEARCH = true;
  currentGear        = 2;
  mapMode            = 0;
  gapHoldMs          = 250;
  lostDir            = -1;
  unpackFlags(DEFAULT_FLAGS_PACKED);
  for (int i = 0; i < TPARAM_N; i++) *tparams[i].ptr = tparams[i].def;
  settingsDirty = true;
  currentSpeed = (float)baseSpeed;
  Serial.println(">>> [FLASH NVS] Đã khôi phục cài đặt gốc mặc định!");
}

// ==============================================================================
// 7. NHẬN DỮ LIỆU TỪ ESP32 PHỤ QUA UART2 (NON-BLOCKING)
// ==============================================================================
void receiveFromSubMcu() {
  // Gom byte vào bộ đệm riêng để khi checksum sai chỉ bỏ 1 byte rồi tìm header tiếp,
  // không nuốt luôn gói đúng nằm phía sau
  static uint8_t acc[128];
  static int accLen = 0;
  const int PKT = (int)sizeof(SensorDataPacket);

  for (int round = 0; round < 40; round++) {   // xả hết hàng đợi UART (tối đa 40 vòng) kể cả khi vừa bị chặn
    while (Serial2.available() > 0 && accLen < (int)sizeof(acc)) {
      acc[accLen++] = (uint8_t)Serial2.read();
    }

    int pos = 0;
    while (accLen - pos >= PKT) {
      if (acc[pos] != 0xAA) { pos++; continue; }

      uint8_t chk = 0;
      for (int i = 0; i < PKT - 1; i++) chk ^= acc[pos + i];

      if (chk != acc[pos + PKT - 1]) { pos++; continue; }   // sai checksum: bỏ 1 byte, thử lại

      memcpy(&rxPacket, acc + pos, PKT);
      pos += PKT;

      lastPacketTime = millis();
      subMcuConnected = true;
      totalPacketsReceived++;
      samplePrevUs = sampleUs;
      sampleUs = micros();
      newSample = true;

      uint8_t seqStep = 1;
      if (haveSeq) {
        uint8_t gap = (uint8_t)(rxPacket.seq - lastSeq);
        if (gap > 1 && gap < 128) lostPackets += (gap - 1);   // gap lớn bất thường = đồng bộ lại sau reset, không tính
        seqStep = (gap >= 1 && gap < 128) ? gap : 1;
      }
      sampleIdx += seqStep;
      lastSeq = rxPacket.seq;
      haveSeq = true;

      // Trích xuất dữ liệu
      error       = (float)rxPacket.error_x1000 / 1000.0f;
      sumWeight   = rxPacket.sumWeight;
      activeCount = rxPacket.activeCount;

      for (int i = 0; i < NUM_SENSORS; i++) {
        s[i]            = (rxPacket.sensorBits >> i) & 1;
        sensorWeight[i] = rxPacket.weights[i] * 4;
      }

      onSample(sampleIdx);   // nhận diện vạch ngang cho TỪNG mẫu: vòng loop chậm cũng không lỡ mốc
    }

    if (pos > 0) {
      memmove(acc, acc + pos, accLen - pos);
      accLen -= pos;
    }
    if (Serial2.available() == 0) break;
  }

  // Cảnh báo nếu mất kết nối với con phụ quá 500ms
  if (millis() - lastPacketTime > 500) {
    subMcuConnected = false;
  }
}

// Gửi lệnh điều khiển sang ESP32 Phụ
void sendCommandToSub(uint8_t cmd) {
  Serial2.write(cmd);
}

// ==============================================================================
// 8. CHẾ ĐỘ TEST CĂN TÂM TẠI CHỖ (CENTERING TEST) - TỰ ĐỘNG KHÓA TÂM CHỐNG LẮC
// ==============================================================================
void runTestCentering() {
  receiveFromSubMcu();

  if (!subMcuConnected) {
    stopMotors();
    return;
  }

  // 1. Nếu xe đã khóa tâm:
  if (isCenteredLocked) {
    // Vùng đệm chống rung lắc (Deadband Hysteresis): Chỉ mở khóa nếu xe bị kéo lệch hẳn > 0.55f hoặc mất line
    if (fabs(error) > 0.55f || (s[3] == 0 && s[4] == 0 && sumWeight < 60)) {
      isCenteredLocked = false;
    } else {
      stopMotors();
      digitalWrite(LED_PIN, HIGH); // Đèn LED sáng đứng báo chuẩn tâm!
      return;
    }
  }

  // 2. Kiểm tra điều kiện khóa tâm: vạch nằm giữa S3/S4 và sai số <= 0.35f
  if (fabs(error) <= 0.35f && (s[3] == 1 || s[4] == 1 || sumWeight > 60)) {
    stopMotors();
    isCenteredLocked = true;
    lastBrakeDir = 0;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  digitalWrite(LED_PIN, LOW);

  // 3. Tốc độ xoay căn tâm tỉ lệ động theo thanh trượt centerTestSpd (70 - 255)
  int spd;
  if (fabs(error) < 0.8f) {
    spd = centerTestSpd * 50 / 100; // Khi gần tâm: lấy 50% lực cài đặt để bò êm không lố
  } else if (fabs(error) < 1.8f) {
    spd = centerTestSpd * 85 / 100; // Cự ly vừa: lấy 85% lực cài đặt
  } else {
    spd = centerTestSpd;            // Cự ly xa: lấy 100% lực thanh trượt Web
  }
  spd = constrain(spd, 0, 255);

  if (error < 0) {
    lastBrakeDir = -1;
    setMotor(-spd, spd); // Xoay trái về tim
  } else {
    lastBrakeDir = 1;
    setMotor(spd, -spd); // Xoay phải về tim
  }
}

// ==============================================================================
// 9. LỘ TRÌNH BẢN ĐỒ "UTE" - ĐẾM CỜ (FLAG) CÁC MỐC VẠCH NGANG
// Xe chỉ đi 1 đường duy nhất: START -> U -> thanh ngang trên -> sống E -> thanh đáy E -> FINISH
// Xe cứ chạy là cứ ĐẾM mọi vạch ngang gặp được (flagCount), số cờ vẫn còn trên hệ thống
// sau khi dừng (hiện trên web, xóa khi xuất phát lượt mới). Tới cờ số n thì làm theo
// flagAct[n-1] (THẲNG / TRÁI / PHẢI) - cấu hình trên web, lưu Flash. Đếm lệch do xe lắc
// thì chỉ cần đổi số cờ, không phải sửa code. Không bao giờ quay đầu.
// ==============================================================================
int mapSign = 0;   // +1 bản đồ gốc, -1 bản đồ lật ngang, 0 chưa biết (chế độ tự nhận)

// --- THÔNG SỐ TINH CHỈNH LỘ TRÌNH (cần chạy thử để canh theo xe thật) ---
// (Các thông số thời gian/ngưỡng của lộ trình nằm ở mục 3, bảng tparams[]: chỉnh trực tiếp trên web, lưu Flash)

enum NavState : uint8_t { NAV_FOLLOW, NAV_ADVANCE, NAV_PREBRAKE, NAV_PIVOT, NAV_BRAKE, NAV_DONE };

NavState nav = NAV_FOLLOW;
int   flagCount = 0;             // CỜ: số vạch ngang đã đếm được trong lượt chạy này
int   navDir = 0;                // +1 đang rẽ phải, -1 đang rẽ trái
bool  evtLatched = true;         // đang đè lên 1 mốc -> không đếm lại
bool  pivotLeftOldLine = false;  // đã rời vạch cũ khi quay
bool  holdResume = true;         // vừa thoát khỏi đoạn giữ lái -> reset đạo hàm PD
int   wideCnt = 0;
unsigned long navTimer = 0;
unsigned long lastWideIdx = 0;   // chỉ số mẫu cuối cùng còn thấy vạch ngang (đếm theo MẪU, ~1ms/mẫu, không phụ thuộc đồng hồ vòng loop)
unsigned long segStartMs = 0;    // lúc bắt đầu đoạn đang chạy (tính từ khi xe bám line trở lại sau mốc)
unsigned long prevSegMs = 0;     // thời gian đoạn vừa chạy xong -> dự đoán điểm giảm ga của đoạn sau
unsigned long segLog[MAX_FLAGS]; // thời gian từng đoạn, in ra khi về đích để tinh chỉnh
float steerAvg = 0.0f;           // lực lái trung bình gần đây (dùng khi qua nhánh cụt / đoạn đứt)
const int EVT_Q = 4;
int8_t evtQ[EVT_Q];              // hàng đợi mốc đã đếm ở mức từng mẫu, chờ followLine xử lý (+1 phải, -1 trái)
int    evtCount = 0;

void resetRoute() {
  flagCount = 0;
  evtCount = 0;
  lostPackets = 0;
  maxLoopUs = 0;
  haveSeq = false;
  mapSign = (mapMode == 1) ? +1 : (mapMode == 2) ? -1 : 0;
  nav = NAV_FOLLOW;
  evtLatched = true;             // vạch ngang ở ô START coi như 1 mốc đang đè, đợi hết mới armed
  lastWideIdx = sampleIdx;
  segStartMs = millis();
  prevSegMs = 0;
  for (int i = 0; i < MAX_FLAGS; i++) segLog[i] = 0;
  wideCnt = 0;
  steerAvg = 0.0f;
  timeLostLine = 0;
  holdResume = true;
}

void finishRace() {
  nav = NAV_DONE;
  isRunning = false;
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  triggerBeep(3, 150, 80);
  Serial.println(">>> [FINISH] Đã tới đích, dừng xe!");
  Serial.printf("    Tổng số cờ đã đếm: %d | Gói UART mất: %lu | Vòng loop dài nhất: %lu us\n", flagCount, lostPackets, maxLoopUs);
  for (int i = 0; i < flagCount && i < MAX_FLAGS; i++) {
    Serial.printf("    Thời gian đoạn tới cờ %d: %lu ms\n", i + 1, segLog[i]);
  }
}

// Số thứ tự cờ rẽ cuối cùng trong cấu hình (sau cờ này, mất vạch = tới đích). 0 = không có cờ rẽ nào
int lastTurnFlagNo() {
  for (int i = MAX_FLAGS - 1; i >= 0; i--) {
    if (flagAct[i] != ACT_STRAIGHT) return i + 1;
  }
  return 0;
}

// Hành động tại cờ số n (1..), đã tính đảo trái/phải cho bản đồ lật
Act actAtFlag(int n) {
  if (n < 1 || n > MAX_FLAGS) return ACT_STRAIGHT;
  Act a = (Act)flagAct[n - 1];
  if (mapSign < 0 && a != ACT_STRAIGHT) a = (a == ACT_LEFT) ? ACT_RIGHT : ACT_LEFT;
  return a;
}

// +1: vạch ngang/nhánh lệch phải, -1: lệch trái, 0: không có (chỉ vạch thường)
int detectWide() {
  if (activeCount < wideMinActive) return 0;
  bool leftEdge  = s[0] || s[1];
  bool rightEdge = s[6] || s[7];
  if (rightEdge && !leftEdge) return +1;
  if (leftEdge && !rightEdge) return -1;
  return 0;
}

// Gọi cho MỖI gói cảm biến nhận được (không phụ thuộc tốc độ vòng loop):
// xác nhận vạch ngang bằng wideConfirm mẫu liên tiếp, đếm đúng 1 lần mỗi mốc, đẩy vào hàng đợi
void onSample(unsigned long idx) {
  if (!isRunning || nav != NAV_FOLLOW) { wideCnt = 0; return; }

  bool hasLine = (sumWeight > 60 || activeCount > 0);
  int  wide = hasLine ? detectWide() : 0;

  if (wide != 0) {
    lastWideIdx = idx;
    if (wideCnt < 255) wideCnt++;
  } else {
    wideCnt = 0;
  }
  // Hết vạch ngang đủ rearmMs mới cho phép đếm mốc tiếp theo (mỗi mốc chỉ tính 1 lần)
  if (evtLatched && wide == 0 && idx - lastWideIdx >= (unsigned long)rearmMs) evtLatched = false;

  if (!evtLatched && wideCnt >= wideConfirm) {
    evtLatched = true;
    if (evtCount < EVT_Q) evtQ[evtCount++] = (int8_t)wide;
  }
}

// ==============================================================================
// 10. ĐIỀU PHỐI ĐÁNH LÁI TRỢ LỰC BÙ BÃO HÒA (SATURATION COMPENSATION)
// Giữ 100% tốc độ cao và 100% lực bẻ lái kể cả khi chạy hết ga 255 PWM
// ==============================================================================
void xuLyDanhLai(float rawPid, float currentSpd, int &outLeft, int &outRight) {
  // 1. Lực đánh lái phản hồi tức thời (STEER_SMOOTH = 0.85 triệt tiêu trễ pha)
  float targetSteer = rawPid;
  currentSteer = currentSteer + STEER_SMOOTH * (targetSteer - currentSteer);

  // Xe chạy 100% tốc độ cài đặt (currentSpd), không bị ghìm ga
  float targetLeft  = currentSpd + currentSteer;
  float targetRight = currentSpd - currentSteer;

  // 2. BÙ BÃO HÒA ĐỘNG CƠ (Saturation Compensation):
  // Khi xe chạy tốc độ cao gần 255, nếu một bánh vượt quá 255 thì thay vì chỉ cắt ngọn bánh đó (làm mất lực lái),
  // ta kéo đồng thời cả 2 bánh xuống để BẢO TOÀN 100% HIỆU SỐ BẺ CUA (targetLeft - targetRight)!
  float maxVal = (targetLeft > targetRight) ? targetLeft : targetRight;
  if (maxVal > 255.0f) {
    float excess = maxVal - 255.0f;
    targetLeft  -= excess;
    targetRight -= excess;
  }
  float minVal = (targetLeft < targetRight) ? targetLeft : targetRight;
  if (minVal < -255.0f) {
    float deficit = -255.0f - minVal;
    targetLeft  += deficit;
    targetRight += deficit;
  }

  // Cho phép bánh quay ngược (âm) khi cần bẻ cua gắt
  outLeft  = (int)constrain(targetLeft,  -255, 255);
  outRight = (int)constrain(targetRight, -255, 255);
}

// ==============================================================================
// 11. VÒNG LẶP DÒ LINE TỐC ĐỘ CAO (1000Hz) + MÁY TRẠNG THÁI LỘ TRÌNH
// ==============================================================================
void followLine() {
  static unsigned long lastLoopTime = 0;
  if (millis() - lastLoopTime < 1) return; // 1000Hz siêu tốc đồng bộ gói tin 1ms từ mạch phụ
  lastLoopTime = millis();

  receiveFromSubMcu();

  if (!subMcuConnected) {
    stopMotors();
    return;
  }

  unsigned long now = millis();

  // ---- A. CÁC PHA QUẸO (không dùng delay, vẫn phục vụ web/nút dừng) ----
  if (nav == NAV_DONE) {
    stopMotors();
    return;
  }

  if (nav == NAV_ADVANCE) {
    // Chạy thẳng thêm để trục bánh xe tới đúng tâm ngã rẽ rồi mới quay
    setMotor(turnAdvSpeed, turnAdvSpeed);
    if (now - navTimer >= (unsigned long)turnAdvMs) {
      nav = NAV_PREBRAKE;
      navTimer = now;
    }
    return;
  }

  if (nav == NAV_PREBRAKE) {
    // Phanh lùi ngắn: dập quán tính tịnh tiến để xe không trượt qua tâm ngã rẽ
    setMotor(-preBrakePwm, -preBrakePwm);
    if (now - navTimer >= (unsigned long)preBrakeMs) {
      nav = NAV_PIVOT;
      navTimer = now;
      pivotLeftOldLine = false;
    }
    return;
  }

  if (nav == NAV_PIVOT) {
    if (navDir > 0) setMotor(pivotFwd, pivotRev);   // rẽ phải
    else            setMotor(pivotRev, pivotFwd);   // rẽ trái

    bool centerSeen = (s[2] || s[3] || s[4] || s[5]);
    if (!centerSeen) pivotLeftOldLine = true;       // đã rời khỏi vạch cũ

    unsigned long t = now - navTimer;
    if ((t >= (unsigned long)turnMinMs && pivotLeftOldLine && centerSeen) || t >= (unsigned long)turnMaxMs) {
      nav = NAV_BRAKE;
      navTimer = now;
    }
    return;
  }

  if (nav == NAV_BRAKE) {
    // Xung phanh ngược dập quán tính xoay, khóa đầu xe lên vạch mới
    if (navDir > 0) setMotor(-brakePwm, brakePwm);
    else            setMotor(brakePwm, -brakePwm);
    if (now - navTimer >= (unsigned long)brakeTime) {
      stopMotors();
      nav = NAV_FOLLOW;
      lastLineSide = navDir;
      lastError = 0; dFiltered = 0; currentSteer = 0;
      timeLostLine = 0;
      wideCnt = 0;
      evtLatched = true;         // vừa quẹo xong: đợi hết vạch ngang mới đếm mốc kế
      lastWideIdx = sampleIdx;
      segStartMs = now;
      holdResume = true;
    }
    return;
  }

  // Mốc kế tiếp là mốc RẼ -> giảm ga để vào mốc với tốc độ cố định, quán tính nhỏ và lặp lại được
  bool nextIsTurn = (actAtFlag(flagCount + 1) != ACT_STRAIGHT);
  float cruise = (float)baseSpeed;
  if (nextIsTurn && cruise > turnApproachSpeed) {
    // Chạy hết ga, chỉ giảm ga ở đoạn cuối trước mốc rẽ
    unsigned long slowAt = (flagCount == 0) ? (unsigned long)firstSlowAfterMs
                                           : prevSegMs * (unsigned long)slowStartPct / 100UL;
    if (now - segStartMs >= slowAt) cruise = (float)turnApproachSpeed;
  }

  // ---- B. XỬ LÝ CÁC MỐC: onSample() đã đếm ở mức TỪNG MẪU nên không lỡ mốc dù vòng lặp chậm ----
  bool hasLine = (sumWeight > 60 || activeCount > 0);
  int  wide = hasLine ? detectWide() : 0;

  while (evtCount > 0) {
    int ew = evtQ[0];                               // phía của mốc: +1 phải, -1 trái
    for (int i = 1; i < evtCount; i++) evtQ[i - 1] = evtQ[i];
    evtCount--;

    flagCount++;                                    // +1 cờ: đếm MỌI vạch ngang gặp được

    // Chế độ tự nhận: cờ 1 cấu hình là rẽ PHẢI mà nhánh nằm bên TRÁI -> bản đồ lật (và ngược lại)
    if (flagCount == 1 && mapSign == 0) {
      if (flagAct[0] == ACT_RIGHT)     mapSign = ew;
      else if (flagAct[0] == ACT_LEFT) mapSign = -ew;
    }

    Act a = actAtFlag(flagCount);
    // Chỉ rẽ về phía thật sự có nhánh: rẽ phải cần nhánh bên phải, rẽ trái cần nhánh bên trái
    if (a == ACT_RIGHT && ew != +1) a = ACT_STRAIGHT;
    if (a == ACT_LEFT  && ew != -1) a = ACT_STRAIGHT;

    prevSegMs = now - segStartMs;
    if (flagCount <= MAX_FLAGS) segLog[flagCount - 1] = prevSegMs;

    if (a == ACT_STRAIGHT) {
      segStartMs = now;                             // đi thẳng, xử lý ở phần giữ lái bên dưới (mốc rẽ thì đo lại sau khi quẹo xong)
    } else {
      nav = NAV_ADVANCE;
      navDir = (a == ACT_RIGHT) ? +1 : -1;
      navTimer = now;
      evtCount = 0;
      setMotor(turnAdvSpeed, turnAdvSpeed);
      return;
    }
  }

  // ---- C. MẤT VẠCH: vạch đứt đoạn (chữ U) / tới đích ----
  if (!hasLine) {
    if (timeLostLine == 0) timeLostLine = now;
    unsigned long lostDuration = now - timeLostLine;
    holdResume = true;

    int ltf = lastTurnFlagNo();
    if (ltf > 0 && flagCount >= ltf) {
      // Đã qua cờ rẽ cuối cùng (thanh đáy chữ E): hết vạch = tới ô FINISH
      if (lostDuration >= (unsigned long)finishRollMs) {
        finishRace();
      } else {
        int leftPWM, rightPWM;
        xuLyDanhLai(steerAvg, cruise, leftPWM, rightPWM);
        setMotor(leftPWM, rightPWM);
      }
      return;
    }

    if (lostDuration < (unsigned long)gapHoldMs) {
      // Đoạn đứt nét: giữ nguyên độ lái đang có (xe đang vào cua U thì tiếp tục cua), KHÔNG xoay tìm
      int leftPWM, rightPWM;
      xuLyDanhLai(steerAvg, cruise * gapSpeedPct / 100.0f, leftPWM, rightPWM);
      setMotor(leftPWM, rightPWM);
      return;
    }

    // Mất vạch quá thời gian đặt (gapHoldMs): rẽ VÒNG CUNG về phía ưu tiên theo bản đồ, không xoay tròn tại chỗ
    if (ENABLE_SPIN_SEARCH && lostDuration < (unsigned long)gapHoldMs + (unsigned long)lostTurnMaxMs) {
      int dir = (mapSign != 0) ? lostDir * mapSign              // bản đồ đã biết: phía ưu tiên đã đặt
                               : (lastLineSide != 0 ? lastLineSide : lostDir);
      int outer = spinSpeed;
      int inner = spinSpeed * lostArcInnerPct / 100;
      if (dir > 0) setMotor(outer, inner);                      // rẽ phải
      else         setMotor(inner, outer);                      // rẽ trái
    } else {
      stopMotors();
    }
    return;
  }

  timeLostLine = 0;

  // ---- D. ĐANG ĐÈ LÊN VẠCH NGANG / NHÁNH CỤT: giữ lái, không để nhánh bên kéo xe lệch vào ----
  if (wide != 0 || evtLatched) {
    int leftPWM, rightPWM;
    xuLyDanhLai(steerAvg, cruise, leftPWM, rightPWM);
    setMotor(leftPWM, rightPWM);
    holdResume = true;
    return;
  }

  // ---- E. PD BÁM LINE BÌNH THƯỜNG ----
  if (error < -1.0f) lastLineSide = -1;
  else if (error > 1.0f) lastLineSide = 1;

  if (holdResume) {            // vừa thoát giữ lái: tránh xung đạo hàm giả
    lastError = error;
    dFiltered = 0.0f;
    holdResume = false;
    newSample = false;
  }

  float P = Kp * error;
  if (newSample) {             // chỉ tính đạo hàm khi có mẫu cảm biến MỚI, chuẩn hóa theo thời gian thật
    newSample = false;
    float dtUs = (float)(sampleUs - samplePrevUs);
    if (dtUs < 500.0f)   dtUs = 500.0f;
    if (dtUs > 20000.0f) dtUs = 20000.0f;
    float dRaw = (error - lastError) * (10000.0f / dtUs);   // đơn vị: sai số trên 10ms
    dFiltered = 0.6f * dFiltered + 0.4f * dRaw;
    lastError = error;
  }
  float D = Kd * dFiltered;

  float rawPid = P + D;
  steerAvg += 0.02f * (rawPid - steerAvg);   // ~50ms: lực lái trung bình (giữ khi qua nhánh/đoạn đứt)
  currentSpeed = cruise;

  int leftPWM, rightPWM;
  xuLyDanhLai(rawPid, currentSpeed, leftPWM, rightPWM);
  setMotor(leftPWM, rightPWM);
}

// ==============================================================================
// 12. GIAO DIỆN WEB SERVER ĐIỀU KHIỂN & TINH CHỈNH TỪ XA TRÊN ĐIỆN THOẠI
// ==============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32 Line Robot Pro</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: #0f172a; color: #f8fafc; padding: 12px; }
    .card { background: #1e293b; border-radius: 12px; padding: 14px; margin-bottom: 12px; border: 1px solid #334155; }
    h2 { font-size: 16px; margin-bottom: 10px; color: #38bdf8; display: flex; justify-content: space-between; align-items: center; }
    .btn-group { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px; }
    button { padding: 13px; font-size: 15px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; transition: 0.1s; }
    button:active { transform: scale(0.98); }
    .btn-start { background: #10b981; color: white; }
    .btn-stop { background: #ef4444; color: white; font-size: 18px; grid-column: span 2; padding: 18px; }
    .btn-center { background: #8b5cf6; color: white; }
    .btn-center.active { background: #d946ef; box-shadow: 0 0 10px #d946ef; }
    .btn-calib { background: #f59e0b; color: white; }
    .btn-save { background: #3b82f6; color: white; }
    .btn-reset { background: #475569; color: #f87171; border: 1px solid #ef4444; }
    .sensors { display: flex; justify-content: space-between; gap: 4px; margin: 10px 0; }
    .sensor-box { flex: 1; height: 36px; border-radius: 4px; background: #334155; display: flex; align-items: center; justify-content: center; font-size: 11px; font-weight: bold; }
    .sensor-box.active { background: #ef4444; color: white; box-shadow: 0 0 8px #ef4444; }
    .status-row { display: flex; justify-content: space-between; margin-bottom: 6px; font-size: 14px; }
    .val-highlight { color: #facc15; font-weight: bold; }
    .form-group { margin-bottom: 12px; }
    .form-group label { display: flex; justify-content: space-between; font-size: 13px; margin-bottom: 4px; color: #94a3b8; }
    .form-group label span { color: #38bdf8; font-weight: bold; }
    input[type=range] { width: 100%; height: 6px; background: #334155; border-radius: 3px; outline: none; accent-color: #38bdf8; }
    .toggle-row { display: flex; justify-content: space-between; align-items: center; padding: 8px 0; border-top: 1px solid #334155; font-size: 14px; }
    input[type=checkbox] { width: 20px; height: 20px; accent-color: #38bdf8; }
    .badge { font-size: 11px; padding: 2px 8px; border-radius: 10px; }
    .badge-ok { background: #065f46; color: #34d399; }
    .badge-err { background: #7f1d1d; color: #f87171; }
    select { background: #334155; color: #f8fafc; border: 1px solid #475569; border-radius: 6px; padding: 8px; font-size: 14px; width: 100%; }
    .toggle-row select { width: auto; min-width: 110px; }
    #calib-banner { display: none; background: #d97706; padding: 12px; border-radius: 8px; text-align: center; font-weight: bold; margin-bottom: 10px; font-size: 15px; }
  </style>
</head>
<body>

  <!-- BANNER ĐẾM NGƯỢC CALI -->
  <div id="calib-banner">⏳ ĐANG CALI CẢM BIẾN: RÊ XE NGAY! (5 GIÂY)</div>

  <!-- TRẠNG THÁI XE & CẢM BIẾN -->
  <div class="card">
    <h2>
      <span>TRẠNG THÁI XE</span>
      <span id="sub-status" class="badge badge-err">Mất kết nối MCU phụ</span>
    </h2>
    <div class="sensors">
      <div id="s0" class="sensor-box">S0</div>
      <div id="s1" class="sensor-box">S1</div>
      <div id="s2" class="sensor-box">S2</div>
      <div id="s3" class="sensor-box">S3</div>
      <div id="s4" class="sensor-box">S4</div>
      <div id="s5" class="sensor-box">S5</div>
      <div id="s6" class="sensor-box">S6</div>
      <div id="s7" class="sensor-box">S7</div>
    </div>
    <div class="status-row">
      <span>Trạng thái: <strong id="car-state" style="color:#ef4444;">ĐANG DỪNG</strong></span>
      <span>Hộp số: <strong id="car-gear" style="color:#38bdf8;">CẤP 2 (190)</strong></span>
    </div>
    <div class="status-row">
      <span>Lệch tâm: <span id="car-err" class="val-highlight">0.00</span></span>
      <span>Động cơ: L=<span id="pwm-l" class="val-highlight">0</span> | R=<span id="pwm-r" class="val-highlight">0</span></span>
    </div>
    <div class="status-row">
      <span>Cờ đã đếm: <span id="flag-count" class="val-highlight">0</span></span>
      <span>Bản đồ: <span id="map-state" class="val-highlight">chưa rõ</span></span>
    </div>
    <div class="status-row">
      <span>Mất gói UART: <span id="lost-pk" class="val-highlight">0</span></span>
      <span>Loop dài nhất: <span id="max-loop" class="val-highlight">0</span> ms</span>
    </div>
  </div>

  <!-- NÚT BẤM ĐIỀU KHIỂN & HỘP SỐ -->
  <div class="card">
    <h2><span>ĐIỀU KHIỂN & HỘP SỐ</span></h2>
    <div class="btn-group">
      <button class="btn-stop" onclick="sendCmd('stop')">🛑 DỪNG KHẨN CẤP (NÚT 1)</button>
      <button class="btn-start" onclick="sendCmd('start')">▶ XUẤT PHÁT (NÚT 1)</button>
      <button id="btn-center" class="btn-center" onclick="sendCmd('center_toggle')">🎯 TEST CĂN TÂM (NÚT 2)</button>
      <button id="btn-calib" class="btn-calib" style="grid-column: span 2;" onclick="sendCmd('calib_start')">✨ BẮT ĐẦU CALI 5 GIÂY (NÚT 2 GIỮ)</button>
    </div>
    <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-top: 6px;">
      <button id="gear-btn-1" style="background:#334155; color:#94a3b8; padding: 10px; font-size:13px; font-weight:bold; border-radius:6px; border:1px solid #475569;" onclick="sendCmd('gear1')">Số 1: 160</button>
      <button id="gear-btn-2" style="background:#334155; color:#94a3b8; padding: 10px; font-size:13px; font-weight:bold; border-radius:6px; border:1px solid #475569;" onclick="sendCmd('gear2')">Số 2: 190</button>
      <button id="gear-btn-3" style="background:#334155; color:#94a3b8; padding: 10px; font-size:13px; font-weight:bold; border-radius:6px; border:1px solid #475569;" onclick="sendCmd('gear3')">Số 3: 230</button>
    </div>
  </div>

  <!-- BẢNG CHẨN ĐOÁN TEST ĐIỆN ÁP TỪNG CHÂN (3 GIÂY) -->
  <div class="card">
    <h2><span>⚡ TEST ĐIỆN ÁP TRỰC TIẾP TỪNG CHÂN (3 GIÂY)</span></h2>
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 8px;">
      <button style="background:#0284c7; color:white; padding:12px 4px; font-size:12px; font-weight:bold;" onclick="sendCmd('test_in1')">Bật IN1 (D19 Trái Tiến)</button>
      <button style="background:#475569; color:white; padding:12px 4px; font-size:12px; font-weight:bold;" onclick="sendCmd('test_in2')">Bật IN2 (D21 Trái Lùi)</button>
      <button style="background:#0284c7; color:white; padding:12px 4px; font-size:12px; font-weight:bold;" onclick="sendCmd('test_in3')">Bật IN3 (D25 Phải Tiến)</button>
      <button style="background:#475569; color:white; padding:12px 4px; font-size:12px; font-weight:bold;" onclick="sendCmd('test_in4')">Bật IN4 (D26 Phải Lùi)</button>
    </div>
    <button style="background:#10b981; color:white; padding:14px; font-size:14px; font-weight:bold; width:100%;" onclick="sendCmd('test_both_fwd')">⚡ BẬT CẢ IN1 & IN3 (CẢ 2 BÁNH CÙNG TIẾN)</button>
  </div>

  <!-- CẤU HÌNH TỐC ĐỘ VẬN HÀNH -->
  <div class="card">
    <h2><span>CẤU HÌNH TỐC ĐỘ (SPEED CONFIG)</span></h2>

    <div class="form-group">
      <label>Tốc độ bám thẳng (Base Speed): <span id="v-base">190</span></label>
      <input type="range" id="in-base" min="100" max="255" step="5" value="190" oninput="updateVal('base', this.value)">
    </div>

    <div class="form-group">
      <label>Tốc độ xoay tròn khi không có Line (Spin Speed): <span id="v-spin">140</span></label>
      <input type="range" id="in-spin" min="90" max="220" step="5" value="140" oninput="updateVal('spin', this.value)">
    </div>

    <div class="form-group">
      <label>Bánh ngoài ôm cua 90° (Pivot Fwd): <span id="v-pfwd">180</span></label>
      <input type="range" id="in-pfwd" min="120" max="255" step="5" value="180" oninput="updateVal('pfwd', this.value)">
    </div>

    <div class="form-group">
      <label>Bánh trong ôm cua 90° (Pivot Rev): <span id="v-prev">-45</span></label>
      <input type="range" id="in-prev" min="-100" max="0" step="5" value="-45" oninput="updateVal('prev', this.value)">
    </div>

    <div class="form-group">
      <label>Tốc độ xoay căn tâm (Centering Speed): <span id="v-cspd">90</span></label>
      <input type="range" id="in-cspd" min="70" max="255" step="5" value="90" oninput="updateVal('cspd', this.value)">
    </div>

    <div class="form-group">
      <label>Lực xung phanh dập quán tính (Brake PWM): <span id="v-bpwm">100</span></label>
      <input type="range" id="in-bpwm" min="80" max="200" step="5" value="100" oninput="updateVal('bpwm', this.value)">
    </div>

    <div class="form-group">
      <label>Thời gian xung phanh (Brake Time ms): <span id="v-btim">14</span></label>
      <input type="range" id="in-btim" min="10" max="40" step="2" value="14" oninput="updateVal('btim', this.value)">
    </div>
  </div>

  <!-- LỘ TRÌNH & ĐẾM CỜ -->
  <div class="card">
    <h2><span>LỘ TRÌNH & ĐẾM CỜ</span></h2>

    <div class="form-group">
      <label>Bản đồ</label>
      <select id="in-map" onchange="saveConfig(false)">
        <option value="0">Tự nhận ở cờ đầu tiên</option>
        <option value="1">Bản đồ gốc</option>
        <option value="2">Bản đồ lật ngang</option>
      </select>
    </div>

    <div class="form-group">
      <label>Mất vạch quá (ms) thì rẽ về phía ưu tiên: <span id="v-gap">250</span></label>
      <input type="range" id="in-gap" min="0" max="1000" step="10" value="250" oninput="updateVal('gap', this.value)">
    </div>

    <div class="form-group">
      <label>Phía ưu tiên khi mất vạch (bản gốc, bản lật tự đảo)</label>
      <select id="in-ldir" onchange="saveConfig(false)">
        <option value="-1">Trái</option>
        <option value="1">Phải</option>
      </select>
    </div>

    <div class="form-group">
      <label>Hành động tại từng cờ (theo bản đồ gốc)</label>
      <div id="flag-list"></div>
    </div>
  </div>

  <!-- THỜI GIAN & NGƯỠNG LỘ TRÌNH (TỰ CANH) -->
  <div class="card">
    <h2><span>THỜI GIAN & NGƯỠNG LỘ TRÌNH (TỰ CANH)</span></h2>
    <div style="font-size:12px; color:#94a3b8; margin-bottom:10px;">Kéo thanh trượt rồi thả tay: áp dụng ngay, tự lưu Flash khi xe dừng.</div>
    <div id="tparam-list"></div>
  </div>

  <!-- CẤU HÌNH ĐÁNH LÁI & HỆ SỐ PD -->
  <div class="card">
    <h2><span>CẤU HÌNH ĐÁNH LÁI & PD</span></h2>
    
    <div class="form-group">
      <label>Kp (Độ nhạy lái): <span id="v-kp">24.0</span></label>
      <input type="range" id="in-kp" min="5" max="60" step="0.5" value="24" oninput="updateVal('kp', this.value)">
    </div>

    <div class="form-group">
      <label>Kd (Hãm quán tính lắc): <span id="v-kd">15.0</span></label>
      <input type="range" id="in-kd" min="0" max="60" step="0.5" value="15" oninput="updateVal('kd', this.value)">
    </div>

    <div class="form-group">
      <label>Độ làm mịn tay lái (Steer Smooth): <span id="v-smooth">0.75</span></label>
      <input type="range" id="in-smooth" min="0.10" max="0.80" step="0.05" value="0.75" oninput="updateVal('smooth', this.value)">
    </div>

    <div class="toggle-row">
      <span>Tự động xoay compa tìm line khi mất</span>
      <input type="checkbox" id="in-spinsrch" onchange="saveConfig(false)">
    </div>
    <div class="toggle-row">
      <span>Đảo chiều Motor Trái</span>
      <input type="checkbox" id="in-invl" onchange="saveConfig(false)">
    </div>
    <div class="toggle-row">
      <span>Đảo chiều Motor Phải</span>
      <input type="checkbox" id="in-invr" onchange="saveConfig(false)">
    </div>

    <div class="btn-group" style="margin-top: 14px;">
      <button class="btn-save" onclick="saveConfig(true)">💾 LƯU VÀO FLASH</button>
      <button class="btn-reset" onclick="resetDefaults()">🔄 KHÔI PHỤC MẶC ĐỊNH</button>
    </div>
  </div>

  <script>
    function updateVal(key, val) {
      document.getElementById('v-' + key).innerText = val;
    }

    function sendCmd(cmd) {
      fetch('/action?cmd=' + cmd);
    }

    function updateGearButtons(g) {
      for (let i = 1; i <= 3; i++) {
        let btn = document.getElementById('gear-btn-' + i);
        if (btn) {
          if (i === g) {
            btn.style.background = '#0284c7';
            btn.style.color = '#ffffff';
            btn.style.borderColor = '#38bdf8';
          } else {
            btn.style.background = '#334155';
            btn.style.color = '#94a3b8';
            btn.style.borderColor = '#475569';
          }
        }
      }
    }

    function saveConfig(showAlert) {
      let kp = document.getElementById('in-kp').value;
      let kd = document.getElementById('in-kd').value;
      let base = document.getElementById('in-base').value;
      let spin = document.getElementById('in-spin').value;
      let pfwd = document.getElementById('in-pfwd').value;
      let prev = document.getElementById('in-prev').value;
      let cspd = document.getElementById('in-cspd').value;
      let bpwm = document.getElementById('in-bpwm').value;
      let btim = document.getElementById('in-btim').value;
      let smooth = document.getElementById('in-smooth').value;
      let spinsrch = document.getElementById('in-spinsrch').checked ? 1 : 0;
      let invl = document.getElementById('in-invl').checked ? 1 : 0;
      let invr = document.getElementById('in-invr').checked ? 1 : 0;

      let map = document.getElementById('in-map').value;
      let gap = document.getElementById('in-gap').value;
      let ldir = document.getElementById('in-ldir').value;
      let fl = '';
      for (let i = 0; i < 8; i++) fl += document.getElementById('in-fl' + i).value;

      let url = `/save?kp=${kp}&kd=${kd}&base=${base}&spin=${spin}&pfwd=${pfwd}&prev=${prev}&cspd=${cspd}&bpwm=${bpwm}&btim=${btim}&smooth=${smooth}&spinsrch=${spinsrch}&invl=${invl}&invr=${invr}&map=${map}&gap=${gap}&ldir=${ldir}&fl=${fl}`;
      fetch(url).then(() => {
        if (showAlert) alert('Đã lưu toàn bộ thông số vĩnh viễn vào Flash NVS!');
      });
    }

    function loadConfig() {
      fetch('/get_config').then(r => r.json()).then(cfg => {
        document.getElementById('in-kp').value = cfg.kp; updateVal('kp', cfg.kp);
        document.getElementById('in-kd').value = cfg.kd; updateVal('kd', cfg.kd);
        document.getElementById('in-base').value = cfg.base; updateVal('base', cfg.base);
        document.getElementById('in-spin').value = cfg.spin; updateVal('spin', cfg.spin);
        document.getElementById('in-pfwd').value = cfg.pfwd; updateVal('pfwd', cfg.pfwd);
        document.getElementById('in-prev').value = cfg.prev; updateVal('prev', cfg.prev);
        document.getElementById('in-cspd').value = cfg.cspd; updateVal('cspd', cfg.cspd);
        document.getElementById('in-bpwm').value = cfg.bpwm; updateVal('bpwm', cfg.bpwm);
        document.getElementById('in-btim').value = cfg.btim; updateVal('btim', cfg.btim);
        document.getElementById('in-smooth').value = cfg.smooth; updateVal('smooth', cfg.smooth);
        document.getElementById('in-spinsrch').checked = cfg.spinsrch;
        document.getElementById('in-invl').checked = cfg.invl;
        document.getElementById('in-invr').checked = cfg.invr;
        document.getElementById('in-map').value = cfg.map;
        document.getElementById('in-gap').value = cfg.gap; updateVal('gap', cfg.gap);
        document.getElementById('in-ldir').value = cfg.ldir;
        for (let i = 0; i < 8; i++) document.getElementById('in-fl' + i).value = cfg.fl[i];
      });
    }

    function resetDefaults() {
      if (confirm('Khôi phục toàn bộ thông số về cấu hình chuẩn ban đầu?')) {
        fetch('/action?cmd=reset_default').then(() => {
          alert('Đã khôi phục cài đặt gốc thành công!');
          loadConfig();
          loadTParams();
        });
      }
    }

    function loadTParams() {
      fetch('/tparams').then(r => r.json()).then(list => {
        let html = '';
        list.forEach(p => {
          let range = p.mx - p.mn;
          let step = range >= 1000 ? 50 : (range >= 300 ? 10 : (range >= 100 ? 5 : 1));
          html += '<div class="form-group"><label>' + p.n + ': <span id="tv-' + p.k + '">' + p.v + '</span></label>' +
                  '<input type="range" id="tp-' + p.k + '" min="' + p.mn + '" max="' + p.mx + '" step="' + step + '" value="' + p.v + '" ' +
                  'oninput="document.getElementById(\'tv-' + p.k + '\').innerText=this.value" ' +
                  'onchange="fetch(\'/tsave?' + p.k + '=\' + this.value)"></div>';
        });
        document.getElementById('tparam-list').innerHTML = html;
      });
    }

    (function buildFlagList() {
      let html = '';
      for (let i = 0; i < 8; i++) {
        html += '<div class="toggle-row"><span>Cờ ' + (i + 1) + '</span>' +
                '<select id="in-fl' + i + '" onchange="saveConfig(false)">' +
                '<option value="0">Đi thẳng</option><option value="1">Rẽ trái</option><option value="2">Rẽ phải</option>' +
                '</select></div>';
      }
      document.getElementById('flag-list').innerHTML = html;
    })();

    loadConfig();
    loadTParams();

    // Cập nhật trạng thái thời gian thực mỗi 150ms
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(st => {
        let stateEl = document.getElementById('car-state');
        if (st.centering) {
          stateEl.innerText = 'CĂN TÂM TẠI CHỖ';
          stateEl.style.color = '#d946ef';
          document.getElementById('btn-center').classList.add('active');
          document.getElementById('btn-center').innerText = '🎯 DỪNG CĂN TÂM';
        } else if (st.running) {
          stateEl.innerText = 'ĐANG CHẠY';
          stateEl.style.color = '#10b981';
          document.getElementById('btn-center').classList.remove('active');
          document.getElementById('btn-center').innerText = '🎯 TEST CĂN TÂM (NÚT 2)';
        } else {
          stateEl.innerText = 'ĐANG DỪNG';
          stateEl.style.color = '#ef4444';
          document.getElementById('btn-center').classList.remove('active');
          document.getElementById('btn-center').innerText = '🎯 TEST CĂN TÂM (NÚT 2)';
        }

        let gearLabels = ["", "CẤP 1 (160)", "CẤP 2 (190)", "CẤP 3 (230)"];
        let g = st.gear || 2;
        let gearEl = document.getElementById('car-gear');
        if (gearEl) gearEl.innerText = gearLabels[g] || ("CẤP " + g);
        updateGearButtons(g);

        // Banner Cali đếm ngược tự động cập nhật từ ESP32
        let calibBanner = document.getElementById('calib-banner');
        if (st.cali) {
          calibBanner.style.display = 'block';
          calibBanner.style.background = '#d97706';
          calibBanner.innerText = '⏳ ĐANG CALI CẢM BIẾN: RÊ XE NGAY! (5 GIÂY)';
        } else if (calibBanner.style.display === 'block' && !st.cali) {
          calibBanner.style.background = '#10b981';
          calibBanner.innerText = '✅ ĐÃ HIỆU CHUẨN XONG & LƯU FLASH!';
          setTimeout(() => { calibBanner.style.display = 'none'; }, 2000);
        }

        document.getElementById('car-err').innerText = st.err.toFixed(2);
        document.getElementById('pwm-l').innerText = st.pwmL;
        document.getElementById('pwm-r').innerText = st.pwmR;
        document.getElementById('flag-count').innerText = st.flag;
        document.getElementById('lost-pk').innerText = st.lost;
        document.getElementById('max-loop').innerText = (st.maxloop / 1000).toFixed(1);
        document.getElementById('map-state').innerText = st.map > 0 ? 'GỐC' : (st.map < 0 ? 'LẬT' : 'chưa rõ');

        let badge = document.getElementById('sub-status');
        if (st.subOk) {
          badge.className = 'badge badge-ok';
          badge.innerText = 'Đã kết nối MCU Phụ';
        } else {
          badge.className = 'badge badge-err';
          badge.innerText = 'Mất kết nối MCU Phụ';
        }

        // Cập nhật 8 ô cảm biến
        for (let i = 0; i < 8; i++) {
          let box = document.getElementById('s' + i);
          let isActive = (st.bits & (1 << i)) !== 0;
          if (isActive) box.classList.add('active');
          else box.classList.remove('active');
        }
      }).catch(()=>{});
    }, 150);
  </script>
</body>
</html>
)rawliteral";

// ==============================================================================
// 13. KHỞI TẠO CÁC ENDPOINT WEB SERVER
// ==============================================================================
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, []() {
    char json[360];
    snprintf(json, sizeof(json), "{\"running\":%s,\"centering\":%s,\"cali\":%s,\"gear\":%d,\"subOk\":%s,\"err\":%.2f,\"bits\":%d,\"pwmL\":%d,\"pwmR\":%d,\"flag\":%d,\"map\":%d,\"lost\":%lu,\"maxloop\":%lu}",
            isRunning ? "true" : "false",
            isTestCenteringMode ? "true" : "false",
            isCalibratingLocal ? "true" : "false",
            currentGear,
            subMcuConnected ? "true" : "false",
            error,
            rxPacket.sensorBits,
            lastMotorL,
            lastMotorR,
            flagCount,
            mapSign,
            lostPackets,
            maxLoopUs);
    server.send(200, "application/json", json);
  });

  server.on("/get_config", HTTP_GET, []() {
    char flagStr[MAX_FLAGS + 1];
    for (int i = 0; i < MAX_FLAGS; i++) flagStr[i] = (char)('0' + flagAct[i]);
    flagStr[MAX_FLAGS] = 0;
    char json[560];
    snprintf(json, sizeof(json), "{\"kp\":%.1f,\"kd\":%.1f,\"base\":%d,\"spin\":%d,\"pfwd\":%d,\"prev\":%d,\"cspd\":%d,\"bpwm\":%d,\"btim\":%d,\"smooth\":%.2f,\"spinsrch\":%s,\"invl\":%s,\"invr\":%s,\"map\":%d,\"gap\":%d,\"ldir\":%d,\"fl\":\"%s\"}",
            Kp, Kd, baseSpeed, spinSpeed, pivotFwd, pivotRev, centerTestSpd, brakePwm, brakeTime, STEER_SMOOTH,
            ENABLE_SPIN_SEARCH ? "true" : "false",
            INVERT_LEFT ? "true" : "false",
            INVERT_RIGHT ? "true" : "false",
            mapMode, gapHoldMs, lostDir, flagStr);
    server.send(200, "application/json", json);
  });

  server.on("/action", HTTP_GET, []() {
    if (server.hasArg("cmd")) {
      String cmd = server.arg("cmd");
      if (cmd == "start") {
        startRace();
      } else if (cmd == "stop") {
        emergencyStop();
      } else if (cmd == "center_toggle") {
        if (isRunning) emergencyStop();
        isTestCenteringMode = !isTestCenteringMode;
        isCenteredLocked = false;
        stopMotors();
        lastBrakeDir = 0;
        if (isTestCenteringMode) triggerBeep(2, 60, 50);
        else                     triggerBeep(1, 80);
      } else if (cmd == "calib_start") {
        triggerCali();
      } else if (cmd == "calib_stop") {
        isCalibratingLocal = false;
        sendCommandToSub(0xC2);
        triggerBeep(2, 100, 60);
      } else if (cmd == "gear1") {
        setGear(1);
      } else if (cmd == "gear2") {
        setGear(2);
      } else if (cmd == "gear3") {
        setGear(3);
      } else if (cmd == "cycle_gear") {
        cycleGear();
      } else if (cmd == "test_in1") {
        isRunning = false; isTestCenteringMode = false; isMotorTesting = true; motorTestTimer = millis();
        analogWrite(IN1, 220); analogWrite(IN2, 0); analogWrite(IN3, 0); analogWrite(IN4, 0);
        lastMotorL = 220; lastMotorR = 0;
        Serial.println(">>> [CHẨN ĐOÁN] Bật trực tiếp IN1 (D19) = 220 PWM trong 3s...");
      } else if (cmd == "test_in2") {
        isRunning = false; isTestCenteringMode = false; isMotorTesting = true; motorTestTimer = millis();
        analogWrite(IN1, 0); analogWrite(IN2, 220); analogWrite(IN3, 0); analogWrite(IN4, 0);
        lastMotorL = -220; lastMotorR = 0;
        Serial.println(">>> [CHẨN ĐOÁN] Bật trực tiếp IN2 (D21) = 220 PWM trong 3s...");
      } else if (cmd == "test_in3") {
        isRunning = false; isTestCenteringMode = false; isMotorTesting = true; motorTestTimer = millis();
        analogWrite(IN1, 0); analogWrite(IN2, 0); analogWrite(IN3, 220); analogWrite(IN4, 0);
        lastMotorL = 0; lastMotorR = 220;
        Serial.println(">>> [CHẨN ĐOÁN] BẬT TRỰC TIẾP IN3 (D25) = 220 PWM trong 3s...");
      } else if (cmd == "test_in4") {
        isRunning = false; isTestCenteringMode = false; isMotorTesting = true; motorTestTimer = millis();
        analogWrite(IN1, 0); analogWrite(IN2, 0); analogWrite(IN3, 0); analogWrite(IN4, 220);
        lastMotorL = 0; lastMotorR = -220;
        Serial.println(">>> [CHẨN ĐOÁN] BẬT TRỰC TIẾP IN4 (D26) = 220 PWM trong 3s...");
      } else if (cmd == "test_both_fwd") {
        isRunning = false; isTestCenteringMode = false; isMotorTesting = true; motorTestTimer = millis();
        analogWrite(IN1, 220); analogWrite(IN2, 0); analogWrite(IN3, 220); analogWrite(IN4, 0);
        lastMotorL = 220; lastMotorR = 220;
        Serial.println(">>> [CHẨN ĐOÁN] BẬT CẢ IN1 (D19) & IN3 (D25) = 220 PWM trong 3s...");
      } else if (cmd == "reset_default") {
        resetDefaultSettings();
      }
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/tparams", HTTP_GET, []() {
    String j = "[";
    for (int i = 0; i < TPARAM_N; i++) {
      if (i) j += ",";
      j += "{\"k\":\""; j += tparams[i].key;
      j += "\",\"n\":\""; j += tparams[i].name;
      j += "\",\"v\":"; j += *tparams[i].ptr;
      j += ",\"mn\":"; j += tparams[i].mn;
      j += ",\"mx\":"; j += tparams[i].mx;
      j += "}";
    }
    j += "]";
    server.send(200, "application/json", j);
  });

  server.on("/tsave", HTTP_GET, []() {
    for (int i = 0; i < TPARAM_N; i++) {
      if (server.hasArg(tparams[i].key)) {
        *tparams[i].ptr = constrain(server.arg(tparams[i].key).toInt(), tparams[i].mn, tparams[i].mx);
      }
    }
    settingsDirty = true;
    server.send(200, "text/plain", "OK");
  });

  server.on("/save", HTTP_GET, []() {
    if (server.hasArg("kp"))       Kp = constrain(server.arg("kp").toFloat(), 0.0f, 60.0f);
    if (server.hasArg("kd"))       Kd = constrain(server.arg("kd").toFloat(), 0.0f, 60.0f);
    if (server.hasArg("base"))     baseSpeed = constrain(server.arg("base").toInt(), 100, 255);
    if (server.hasArg("spin"))     spinSpeed = constrain(server.arg("spin").toInt(), 90, 220);
    if (server.hasArg("pfwd"))     pivotFwd = constrain(server.arg("pfwd").toInt(), 120, 255);
    if (server.hasArg("prev"))     pivotRev = constrain(server.arg("prev").toInt(), -100, 0);
    if (server.hasArg("cspd"))     centerTestSpd = constrain(server.arg("cspd").toInt(), 70, 255);
    if (server.hasArg("bpwm"))     brakePwm = constrain(server.arg("bpwm").toInt(), 80, 200);
    if (server.hasArg("btim"))     brakeTime = constrain(server.arg("btim").toInt(), 10, 40);
    if (server.hasArg("smooth"))   STEER_SMOOTH = constrain(server.arg("smooth").toFloat(), 0.10f, 0.80f);
    if (server.hasArg("spinsrch")) ENABLE_SPIN_SEARCH = (server.arg("spinsrch") == "1");
    if (server.hasArg("invl"))     INVERT_LEFT = (server.arg("invl") == "1");
    if (server.hasArg("invr"))     INVERT_RIGHT = (server.arg("invr") == "1");
    if (server.hasArg("map"))      mapMode = constrain(server.arg("map").toInt(), 0, 2);
    if (server.hasArg("gap"))      gapHoldMs = constrain(server.arg("gap").toInt(), 0, 2000);
    if (server.hasArg("ldir"))     lostDir = (server.arg("ldir").toInt() >= 0) ? +1 : -1;
    if (server.hasArg("fl")) {
      String fl = server.arg("fl");
      for (int i = 0; i < MAX_FLAGS && i < (int)fl.length(); i++) {
        char c = fl[i];
        flagAct[i] = (c >= '0' && c <= '2') ? (uint8_t)(c - '0') : (uint8_t)ACT_STRAIGHT;
      }
    }

    settingsDirty = true;
    server.send(200, "text/plain", "SAVED");
  });

  server.begin();
  Serial.println(">>> [WIFI] Web Server đã sẵn sàng tại http://192.168.4.1");
}

// ==============================================================================
// 14. XỬ LÝ 3 NÚT NHẤN NGOÀI (ACTIVE HIGH VỚI TRỞ KÉO MASS)
// ==============================================================================
void checkExternalButtons() {
  unsigned long now = millis();

  // --- NÚT 1 (GPIO 22): XUẤT PHÁT / DỪNG KHẨN CẤP (ĂN NGAY, KHÔNG TRỄ) ---
  static unsigned long btn1LastChange = 0;
  static bool btn1LastState = LOW;
  bool b1 = digitalRead(PIN_BTN1);

  if (b1 != btn1LastState) {
    if (now - btn1LastChange > 60) { // Chống rung phím 60ms
      btn1LastChange = now;
      btn1LastState = b1;
      if (b1 == HIGH) { // Bấm nút xuống
        if (isRunning || isStartingCountdown || isTestCenteringMode || isMotorTesting) {
          emergencyStop();
        } else {
          startRace();
        }
      }
    }
  }

  // --- NÚT 2 (GPIO 23): NHẤN NHẸ = TEST CĂN TÂM | GIỮ 1.2s = CALI 5s ---
  static unsigned long btn2PressTime = 0;
  static bool btn2LastState = LOW;
  static bool btn2LongActionDone = false;
  bool b2 = digitalRead(PIN_BTN2);

  if (b2 == HIGH && btn2LastState == LOW) {
    // Vừa nhấn xuống
    btn2PressTime = now;
    btn2LongActionDone = false;
    btn2LastState = HIGH;
  } else if (b2 == HIGH && btn2LastState == HIGH) {
    // Đang giữ nút
    if (!btn2LongActionDone && (now - btn2PressTime >= 1200)) {
      btn2LongActionDone = true;
      if (isRunning || isMotorTesting) emergencyStop();
      triggerCali();
    }
  } else if (b2 == LOW && btn2LastState == HIGH) {
    // Vừa nhả nút ra
    unsigned long dur = now - btn2PressTime;
    btn2LastState = LOW;
    if (!btn2LongActionDone && dur >= 60 && dur < 1200) {
      if (isRunning || isMotorTesting) emergencyStop();
      isTestCenteringMode = !isTestCenteringMode;
      isCenteredLocked = false;
      stopMotors();
      lastBrakeDir = 0;
      if (isTestCenteringMode) {
        triggerBeep(2, 60, 50);
        Serial.println(">>> [NÚT 2] Bật chế độ Test căn tâm!");
      } else {
        triggerBeep(1, 80);
        Serial.println(">>> [NÚT 2] Tắt chế độ Test căn tâm!");
      }
    }
  }

  // --- NÚT 3 (GPIO 4): ĐỔI CẤP SỐ TỐC ĐỘ (1 -> 2 -> 3 -> 1) ---
  static unsigned long btn3LastChange = 0;
  static bool btn3LastState = LOW;
  bool b3 = digitalRead(PIN_BTN3);

  if (b3 != btn3LastState) {
    if (now - btn3LastChange > 60) { // Chống rung 60ms
      btn3LastChange = now;
      btn3LastState = b3;
      if (b3 == HIGH) { // Bấm nút xuống
        cycleGear();
      }
    }
  }
}

// ==============================================================================
// 15. XỬ LÝ NÚT NHẤN BOOT TRÊN ESP32 CHÍNH (NÚT DỰ PHÒNG)
// ==============================================================================
void checkButton() {
  static unsigned long lastPress = 0;
  static bool lastBtn = HIGH;
  bool btn = digitalRead(BTN_PIN);

  if (lastBtn == HIGH && btn == LOW) {
    if (millis() - lastPress > 200) {
      lastPress = millis();
      if (isRunning || isStartingCountdown || isTestCenteringMode || isMotorTesting) {
        emergencyStop();
      } else {
        startRace();
      }
    }
  }
  lastBtn = btn;
}

// ==============================================================================
// 16. SETUP & LOOP CHÍNH
// ==============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Vô hiệu hóa reset sụt áp

  Serial.begin(115200);

  // Khởi tạo UART2 nhận dữ liệu từ ESP32 Phụ
  Serial2.setRxBufferSize(2048);   // mặc định 256 byte chỉ chứa ~15ms dữ liệu -> tràn khi loop bị chặn; 2048 byte chứa ~120ms
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX2, UART_TX2);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP); // Nút BOOT trên kit ESP32

  // 3 Nút bấm ngoài (Trở kéo mass ngoài -> Active HIGH)
  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_BTN2, INPUT);
  pinMode(PIN_BTN3, INPUT);

  // Còi chíp
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM motor 8kHz thay vì ~1kHz mặc định: hết rít L298N, điều khiển tốc độ thấp mượt hơn
  const int motorPins[4] = {IN1, IN2, IN3, IN4};
  for (int i = 0; i < 4; i++) {
    analogWrite(motorPins[i], 0);
    analogWriteFrequency(motorPins[i], 8000);
  }

  stopMotors();

  // Đọc cấu hình đã lưu
  loadSettings();

  // Phát WiFi Access Point (Mở, không cần mật khẩu)
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-LineCar", AP_PASSWORD);

  // Âm thanh chào mừng khởi động (2 bíp ngắn phi chặn)
  triggerBeep(2, 60, 60);

  Serial.println("\n==============================================================");
  Serial.println("  ESP32 CHÍNH: WIFI WEB-TUNING + ĐIỀU KHIỂN MOTOR L298N");
  Serial.println("  - Motor Trái: IN1 (D19), IN2 (D21)");
  Serial.println("  - Motor Phải: IN3 (D25), IN4 (D26) [Mới: Sạch sẽ, không vướng strapping]");
  Serial.println("  - 3 Nút bấm ngoài: BTN1 (D22), BTN2 (D23), BTN3 (D4)");
  Serial.println("  - Còi chíp: BUZZER (D27)");
  Serial.println("  - Kết nối WiFi: 'ESP32-LineCar' (mật khẩu trong AP_PASSWORD) -> Mở: http://192.168.4.1");
  Serial.println("  - Tốc độ UART2 nhận từ ESP32 Phụ: 921600 baud");
  Serial.println("==============================================================");

  setupWebServer();
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);
}

// Web server chạy ở CORE 0 (cùng WiFi) để việc phục vụ trang web (có thể chặn hàng chục ms) không bao giờ làm khựng vòng điều khiển ở CORE 1
void webTask(void *) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void loop() {
  // Đo khoảng cách giữa 2 vòng loop khi xe đang chạy (để biết có bị chặn/chậm không)
  static unsigned long lastLoopUs = 0;
  unsigned long nowLoopUs = micros();
  if (isRunning && lastLoopUs != 0 && (nowLoopUs - lastLoopUs) > maxLoopUs) maxLoopUs = nowLoopUs - lastLoopUs;
  lastLoopUs = nowLoopUs;

  // Cập nhật trạng thái còi phi chặn ngay đầu vòng lặp (hoàn toàn 0ms trễ)
  updateBuzzer();

  // Luôn nhận dữ liệu cảm biến mới nhất từ ESP32 Phụ ở đầu vòng lặp
  receiveFromSubMcu();

  // 2. Kiểm tra các nút nhấn cứng (3 nút ngoài + nút BOOT dự phòng)
  checkExternalButtons();
  checkButton();

  // 3. Tiến trình đếm nhịp Cali nếu đang kích hoạt
  updateCaliProcess();

  // 4. Tiến trình Test Động Cơ riêng biệt (nếu bấm test từ Web)
  if (isMotorTesting) {
    if (millis() - motorTestTimer >= 3000) {
      isMotorTesting = false;
      stopMotors();
      Serial.println(">>> [TEST ĐIỆN ÁP] Đã ngắt điện sau 3 giây.");
    }
  }
  // 5. Tiến trình đếm ngược 3 tiếng bíp xuất phát (buông tay an toàn)
  else if (isStartingCountdown) {
    stopMotors(); // Giữ xe đứng yên hoàn toàn khi đang đếm 3 bíp
    if (millis() - startCountdownTimer >= START_DELAY_MS) {
      isStartingCountdown = false;
      isRunning = true;
      resetRoute();
      currentSpeed = (float)baseSpeed;
      digitalWrite(LED_PIN, HIGH);
      Serial.println(">>> [XUẤT PHÁT] Xe chính thức phóng đi!");
    }
  }
  // 6. Xử lý chế độ Test căn tâm tại chỗ
  else if (isTestCenteringMode) {
    runTestCentering();
  }
  // 7. Xử lý chế độ Chạy đua line
  else if (isRunning) {
    followLine();
  } 
  // 8. Chế độ Chờ (Standby)
  else {
    stopMotors();
  }

  flushSettingsIfIdle();

  // In Serial Monitor debug kiểm tra mỗi 400ms (không in khi đang chạy, và không bao giờ chờ buffer đầy)
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= 400) {
    lastDbg = millis();
    if (!isRunning && Serial.availableForWrite() >= 200) Serial.printf("[Monitor] Sub: %s | PWM_L: %4d | PWM_R: %4d | Err: %5.2f | Sum: %4ld | Chạy: %d | Cờ: %d/%d | Nav: %d | Map: %d | Mất gói: %lu | LoopMax: %lu us\n",
                  subMcuConnected ? "OK" : "MẤT KẾT NỐI",
                  lastMotorL,
                  lastMotorR,
                  error,
                  sumWeight,
                  isRunning ? 1 : 0,
                  flagCount, lastTurnFlagNo(), (int)nav, mapSign, lostPackets, maxLoopUs);
  }
}
