#include <Arduino.h>
#include <Preferences.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==============================================================================
// ESP32 PHỤ: CHUYÊN XỬ LÝ TÍN HIỆU 8 MẮT CẢM BIẾN ANALOG + HIỆU CHUẨN (CALIBRATION)
// Giao tiếp với ESP32 Chính qua UART2 (Tốc độ cực cao 921600 baud, độ trễ < 0.1ms)
// Hoạt động độc lập siêu tốc, nhận lệnh Cali từ ESP32 Chính qua UART (0xC1 / 0xC2)
// Đã loại bỏ hoàn toàn nút BOOT và LED GPIO 2 để giải phóng 100% tài nguyên CPU
// ==============================================================================

// 1. CẤU HÌNH PHẦN CỨNG 8 MẮT CẢM BIẾN ANALOG
#define PIN_S0 14  // Mắt 0 - Trái ngoài cùng
#define PIN_S1 27  // Mắt 1 - Trái ngoài vừa
#define PIN_S2 26  // Mắt 2 - Trái trong vừa
#define PIN_S3 25  // Mắt 3 - Trái sát tâm (Tim xe)
#define PIN_S4 33  // Mắt 4 - Phải sát tâm (Tim xe)
#define PIN_S5 32  // Mắt 5 - Phải trong vừa
#define PIN_S6 35  // Mắt 6 - Phải ngoài vừa
#define PIN_S7 34  // Mắt 7 - Phải ngoài cùng

// Cấu hình chân UART2 giao tiếp với ESP32 Chính
#define UART_RX2 16  // Nối với TX2 (GPIO 17) của ESP32 Chính
#define UART_TX2 17  // Nối với RX2 (GPIO 16) của ESP32 Chính
#define UART_BAUD 921600 // Tốc độ cực cao 921600 baud đồng bộ cả 2 con

const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S5, PIN_S6, PIN_S7};

// Cực tính vạch đen: false = vạch đen áp THẤP, nền trắng áp CAO (chuẩn ngõ AO)
bool LINE_BLACK_IS_HIGH = false;
bool INVERT_SENSOR_ORDER = false;

// Mảng lưu giá trị cảm biến
int rawVal[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorWeight[NUM_SENSORS]; // 0 -> 1000
int s[NUM_SENSORS];            // 0: Trắng, 1: Đen
int activeCount = 0;
long sumWeight = 0;
float error = 0.0f;
float lastKnownError = 0.0f;

// Trạng thái hiệu chuẩn
bool isCalibrating = false;
bool isCalibrated  = false;
unsigned long calibStartTime = 0;
const unsigned long CALIB_DURATION_MS = 5000;
int calibMin[NUM_SENSORS];
int calibMax[NUM_SENSORS];

Preferences prefs;

// ==============================================================================
// 2. CẤU TRÚC GÓI TIN TRUYỀN DỮ LIỆU SIÊU TỐC (BINARY DATA PACKET)
// ==============================================================================
#pragma pack(push, 1)
struct SensorDataPacket {
  uint8_t  header;        // 0xAA: Byte nhận diện đầu gói tin
  uint8_t  sensorBits;    // 8 bit biểu diễn 8 mắt (Bit 0 = S0 ... Bit 7 = S7)
  int16_t  error_x1000;   // Sai số nhân 1000 (từ -3500 đến +3500)
  uint16_t sumWeight;     // Tổng trọng số bắt vạch (0 - 8000)
  uint8_t  activeCount;   // Số mắt đang chạm vạch (0 - 8)
  uint8_t  weights[8];    // Trọng số từng mắt (0 - 250, thu nhỏ từ 1000 chia 4)
  uint8_t  isCalibrating; // 1 nếu đang hiệu chuẩn, 0 nếu đang hoạt động bình thường
  uint8_t  seq;           // Số thứ tự gói (tăng dần, tự quay vòng) để ESP32 Chính phát hiện mất gói
  uint8_t  checksum;      // Byte kiểm tra toàn vẹn dữ liệu chống nhiễu
};
#pragma pack(pop)

SensorDataPacket txPacket;

void finalizePacket() {
  uint8_t chk = 0;
  uint8_t *ptr = (uint8_t*)&txPacket;
  for (size_t i = 0; i < sizeof(SensorDataPacket) - 1; i++) {
    chk ^= ptr[i];
  }
  txPacket.checksum = chk;
}

// Đang cali: vẫn gửi gói tin đều đặn với cờ isCalibrating để ESP32 Chính biết (xác nhận lệnh + báo kết thúc)
void packCalibrationPacket() {
  txPacket.header        = 0xAA;
  txPacket.sensorBits    = 0;
  txPacket.error_x1000   = 0;
  txPacket.sumWeight     = 0;
  txPacket.activeCount   = 0;
  txPacket.isCalibrating = isCalibrating ? 1 : 0;
  for (int i = 0; i < 8; i++) txPacket.weights[i] = 0;
  finalizePacket();
}

// ==============================================================================
// 3. HIỆU CHUẨN CẢM BIẾN (CALIBRATION) & LƯU VÀO FLASH NVS
// ==============================================================================
void saveCalibration() {
  prefs.begin("sensor_calib", false);
  for (int i = 0; i < NUM_SENSORS; i++) {
    char k_min[12], k_max[12];
    snprintf(k_min, sizeof(k_min), "mi%d", i);
    snprintf(k_max, sizeof(k_max), "ma%d", i);
    prefs.putInt(k_min, sensorMin[i]);
    prefs.putInt(k_max, sensorMax[i]);
  }
  prefs.putBool("valid", true);
  prefs.end();
  Serial.println(">>> [FLASH NVS] Đã lưu thông số Cali vào Flash của ESP32 Phụ!");
}

void loadCalibration() {
  prefs.begin("sensor_calib", true);
  bool valid = prefs.getBool("valid", false);
  if (valid) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      char k_min[12], k_max[12];
      snprintf(k_min, sizeof(k_min), "mi%d", i);
      snprintf(k_max, sizeof(k_max), "ma%d", i);
      sensorMin[i] = prefs.getInt(k_min, 400);
      sensorMax[i] = prefs.getInt(k_max, 3200);
    }
    isCalibrated = true;
    Serial.println(">>> [FLASH NVS] Đã đọc dữ liệu Cali từ Flash!");
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 400;
      sensorMax[i] = 3200;
    }
    isCalibrated = false;
    Serial.println(">>> [FLASH NVS] Dùng dải mặc định (400 - 3200).");
  }
  prefs.end();
}

void startCalibration() {
  isCalibrating = true;
  calibStartTime = millis();

  for (int i = 0; i < NUM_SENSORS; i++) {
    calibMin[i] = 4095;
    calibMax[i] = 0;
  }

  Serial.println("\n=======================================================");
  Serial.println(">>> [BẮT ĐẦU HIỆU CHUẨN CẢM BIẾN (CALI)]");
  Serial.println("-> Cầm tay rê dàn mắt qua lại ngang qua vạch đen & nền trắng.");
  Serial.println("-> Tự động hoàn tất sau 5s hoặc khi nhận lệnh kết thúc từ MCU Chính.");
  Serial.println("=======================================================");
}

void finishCalibration() {
  if (!isCalibrating) return;
  isCalibrating = false;

  Serial.println("\n=======================================================");
  Serial.println(">>> [KẾT QUẢ HIỆU CHUẨN 8 MẮT]");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int diff = calibMax[i] - calibMin[i];
    if (diff >= 200) {
      sensorMin[i] = calibMin[i];
      sensorMax[i] = calibMax[i];
    }
    Serial.printf("S%d | Min: %4d | Max: %4d | Diff: %4d\n", i, sensorMin[i], sensorMax[i], (sensorMax[i] - sensorMin[i]));
  }
  Serial.println("=======================================================");

  isCalibrated = true;
  saveCalibration();
}

void runCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int pinIdx = INVERT_SENSOR_ORDER ? (NUM_SENSORS - 1 - i) : i;
    int val = analogRead(sensorPins[pinIdx]);
    if (val < calibMin[i]) calibMin[i] = val;
    if (val > calibMax[i]) calibMax[i] = val;
  }

  if (millis() - calibStartTime >= CALIB_DURATION_MS) {
    finishCalibration();
  }
}

// ==============================================================================
// 4. THUẬT TOÁN ĐỌC ANALOG & TÍNH TOÁN TRỌNG TÂM (CENTER OF MASS)
// Trọng số tuyến tính siêu mượt chuẩn -3.5 đến +3.5 (Không phóng đại quá đà, không bỏ line)
// ==============================================================================
void readSensors() {
  sumWeight = 0;
  long weightedSum = 0;
  activeCount = 0;
  uint8_t bits = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int pinIdx = INVERT_SENSOR_ORDER ? (NUM_SENSORS - 1 - i) : i;
    rawVal[i] = analogRead(sensorPins[pinIdx]);

    // Kiểm tra an toàn dải đo min/max
    int sMin = sensorMin[i];
    int sMax = sensorMax[i];
    if (sMax <= sMin + 150) {
      sMin = 400;
      sMax = 3200;
    }

    // Chuẩn hóa dải đo về 0 (nền trắng) đến 1000 (vạch đen):
    int w = 0;
    if (!LINE_BLACK_IS_HIGH) {
      w = map(rawVal[i], sMax, sMin, 0, 1000);
    } else {
      w = map(rawVal[i], sMin, sMax, 0, 1000);
    }
    w = constrain(w, 0, 1000);

    // Lọc nhiễu sàn thông minh: cắt bỏ dưới 80 và kéo dãn tuyến tính (KHÔNG bình phương)
    if (w < 80) {
      w = 0;
    } else {
      w = (int)(((long)(w - 80) * 1000) / 920);
    }

    sensorWeight[i] = w;
    s[i] = (w >= 180) ? 1 : 0;
    if (s[i]) {
      activeCount++;
      bits |= (1 << i); // Ghi nhận bit tương ứng
    }

    // Trọng số tuyến tính chuẩn xác: Giúp sai số chuyển dịch mượt mà giữa các mắt
    weightedSum += (long)w * (i * 1000);
    sumWeight   += (long)w;
  }

  // Nội suy trọng tâm chuẩn xác (-3.5 đến +3.5)
  if (sumWeight > 60) {
    float linePosition = (float)weightedSum / (float)sumWeight;
    error = (linePosition - 3500.0f) / 1000.0f; // Chuẩn hóa dải từ -3.5f đến +3.5f
    error = constrain(error, -3.5f, 3.5f);
    lastKnownError = error;
  } else {
    // Khi chớm mất vạch: Giữ nguyên hướng lệch mép chuẩn (-3.5 hoặc +3.5)
    if (lastKnownError < 0) error = -3.5f;
    else                    error = 3.5f;
  }

  // Đóng gói dữ liệu gửi sang ESP32 Chính
  txPacket.header        = 0xAA;
  txPacket.sensorBits    = bits;
  txPacket.error_x1000   = (int16_t)round(constrain(error, -3.5f, 3.5f) * 1000.0f);
  txPacket.sumWeight     = (uint16_t)constrain(sumWeight, 0, 65535);
  txPacket.activeCount   = (uint8_t)activeCount;
  txPacket.isCalibrating = isCalibrating ? 1 : 0;

  for (int i = 0; i < 8; i++) {
    txPacket.weights[i] = (uint8_t)(sensorWeight[i] / 4); // Thu nhỏ 0..1000 về 0..250
  }

  finalizePacket();   // checksum XOR
}

// ==============================================================================
// 5. TRUYỀN DỮ LIỆU SANG ESP32 CHÍNH & NHẬN LỆNH ĐIỀU KHIỂN
// ==============================================================================
void sendPacketToMain() {
  static uint8_t txSeq = 0;
  txPacket.seq = txSeq++;
  finalizePacket();   // seq đổi -> tính lại checksum
  Serial2.write((uint8_t*)&txPacket, sizeof(SensorDataPacket));
}

void checkCommandsFromMain() {
  while (Serial2.available() > 0) {
    uint8_t cmd = Serial2.read();
    if (cmd == 0xC1) {
      // Lệnh từ ESP32 Chính: Bắt đầu Calibration (Chính gửi lặp tới khi nhận xác nhận -> không khởi động lại)
      if (!isCalibrating) startCalibration();
    } else if (cmd == 0xC2) {
      // Lệnh từ ESP32 Chính: Kết thúc Calibration
      finishCalibration();
    }
  }
}

// ==============================================================================
// 6. SETUP & LOOP CHÍNH
// ==============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Vô hiệu hóa reset sụt áp

  Serial.begin(115200);

  // Khởi tạo UART2 tốc độ cao 921600 baud nối sang ESP32 Chính
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX2, UART_TX2);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  loadCalibration();

  Serial.println("\n=======================================================");
  Serial.println("  ESP32 PHỤ: XỬ LÝ TÍN HIỆU ANALOG 8 MẮT DÒ LINE");
  Serial.println("  - Tốc độ UART2 sang ESP32 Chính: 921600 baud");
  Serial.println("  - Nhận lệnh Cali từ ESP32 Chính qua UART2 (0xC1 / 0xC2)");
  Serial.println("=======================================================");
}

void loop() {
  // Đo thời gian lớn nhất của 1 vòng lặp (phải << 1000us để gói tin đi đều 1ms/gói)
  static unsigned long lastLoopUs = 0, maxLoopUs = 0;
  unsigned long nowLoopUs = micros();
  if (lastLoopUs != 0 && (nowLoopUs - lastLoopUs) > maxLoopUs) maxLoopUs = nowLoopUs - lastLoopUs;
  lastLoopUs = nowLoopUs;

  // 1. Nhận lệnh điều khiển từ ESP32 Chính (0xC1: Bắt đầu Cali, 0xC2: Kết thúc Cali)
  checkCommandsFromMain();

  // 2. Chế độ Cali hoặc chế độ Đọc cảm biến
  if (isCalibrating) {
    runCalibration();
    packCalibrationPacket();
  } else {
    readSensors();
  }

  // 3. Bắn gói tin sang ESP32 Chính mỗi 1ms (1000Hz siêu tốc ở 921600 baud)
  static unsigned long lastSendUs = 0;
  static unsigned long totalPacketsSent = 0;
  unsigned long nowUs = micros();
  if (nowUs - lastSendUs >= 1000) {
    lastSendUs = nowUs;
    sendPacketToMain();
    totalPacketsSent++;
  }

  // 4. In Serial Monitor debug mỗi 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500 && Serial.availableForWrite() >= 160) {
    lastPrint = millis();
    Serial.printf("[TX -> Main] Sent: %lu | Bits: [ ", totalPacketsSent);
    for (int i = 0; i < 8; i++) Serial.printf("%d ", (txPacket.sensorBits >> i) & 1);
    Serial.printf("] | Err: %5.2f | Sum: %4ld | LoopMax: %lu us\n", error, sumWeight, maxLoopUs);
    maxLoopUs = 0;
  }
}
