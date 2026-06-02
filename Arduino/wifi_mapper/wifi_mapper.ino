// WiFi Mapper — Arduino UNO R4 WiFi
// MPU-6050: SDA→SDA, SCL→SCL (基板右上専用ピン), VCC→5V, GND→GND
// 依存: ArduinoJson (Library Manager)

#include <WiFiS3.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_FIFO_EN     0x23
#define MPU_USER_CTRL   0x6A
#define MPU_FIFO_CNT_H  0x72
#define MPU_FIFO_CNT_L  0x73
#define MPU_FIFO_RW     0x74
#define MPU_WHO_AM_I    0x75

#define GYRO_SCALE   (1.0 / 131.0)    // ±250°/s: 131 LSB/(°/s)
#define ACCEL_SCALE  (1.0 / 16384.0)  // ±2g: 16384 LSB/g
#define GRAVITY      9.80665

#define MOVE_THRESHOLD 0.35
#define CALIB_SAMPLES  200

#define IMU_INTERVAL_MS   10
#define SCAN_INTERVAL_MS  2000
#define MAX_APS_PER_SCAN  20

// FIFO layout: AX_H AX_L AY_H AY_L AZ_H AZ_L GZ_H GZ_L (8 bytes/sample)
// 512 bytes / 8 = 64 samples = 2.56 s @ 25 Hz
#define FIFO_BYTES_PER_SAMPLE  8
#define FIFO_MAX_SAMPLES       64
#define FIFO_ODR_MS            40

float gzBias = 0, axBias = 0, ayBias = 0;
unsigned long lastImuMs   = 0;
unsigned long lastScanMs  = 0;
unsigned long scanStartMs = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool mpuCheckConnection(uint8_t &who) {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) { who = 0xFF; return false; }
  who = mpuReadByte(MPU_WHO_AM_I);
  return who == 0x68 || who == 0x70;  // 0x70: MPU-6500 / clone
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // TEMP reg
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void calibrateMPU() {
  long sumGz = 0, sumAx = 0, sumAy = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sumGz += gz; sumAx += ax; sumAy += ay;
    delay(5);
  }
  gzBias = sumGz / (float)CALIB_SAMPLES;
  axBias = sumAx / (float)CALIB_SAMPLES;
  ayBias = sumAy / (float)CALIB_SAMPLES;
}

void startFifo() {
  mpuWrite(MPU_SMPLRT_DIV, 0x27);  // 1kHz / (1+39) = 25 Hz ODR
  mpuWrite(MPU_CONFIG,     0x01);  // DLPF_CFG=1: enables 1 kHz internal clock
  mpuWrite(MPU_FIFO_EN,    0x18);  // bit3=ACCEL, bit4=ZG
  mpuWrite(MPU_USER_CTRL,  0x44);  // FIFO_EN + FIFO_RESET
  scanStartMs = millis();
}

void stopFifo() {
  mpuWrite(MPU_USER_CTRL,  0x00);
  mpuWrite(MPU_FIFO_EN,    0x00);
  mpuWrite(MPU_SMPLRT_DIV, 0x00);
  mpuWrite(MPU_CONFIG,     0x00);
}

void drainAndSendFifo(unsigned long startMs) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_FIFO_CNT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  uint16_t fifoCount = ((uint16_t)Wire.read() << 8) | Wire.read();

  int numSamples = fifoCount / FIFO_BYTES_PER_SAMPLE;
  if (numSamples > FIFO_MAX_SAMPLES) numSamples = FIFO_MAX_SAMPLES;

  unsigned long elapsed = millis() - startMs;
  int overflowed = (elapsed > (unsigned long)(FIFO_MAX_SAMPLES * FIFO_ODR_MS)) ? 1 : 0;

  static StaticJsonDocument<2048> doc;  // static: keep off stack (2 KB)
  doc.clear();
  doc["t"]  = "fb";
  doc["ts"] = (long)startMs;
  doc["te"] = (long)millis();
  doc["dt"] = FIFO_ODR_MS;
  doc["n"]  = numSamples;
  doc["ov"] = overflowed;
  JsonArray d = doc.createNestedArray("d");

  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_FIFO_RW);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 8);
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // AZ (unused)
    int16_t gz = (Wire.read() << 8) | Wire.read();

    d.add((float)((ax - axBias) * ACCEL_SCALE * GRAVITY));
    d.add((float)((ay - ayBias) * ACCEL_SCALE * GRAVITY));
    d.add((float)((gz - gzBias) * GYRO_SCALE * PI / 180.0));
  }

  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
}

void sendBootError(const char *message, uint8_t who) {
  StaticJsonDocument<128> doc;
  doc["t"]   = "err";
  doc["msg"] = message;
  doc["who"] = who;
  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz for faster FIFO drain
  uint8_t who = 0xFF;
  if (!mpuCheckConnection(who)) {
    sendBootError("mpu6050_not_found", who);
    while (true) delay(1000);
  }

  mpuWrite(MPU_PWR_MGMT_1, 0x00);  // wake from sleep
  mpuWrite(MPU_GYRO_CFG,   0x00);  // ±250°/s
  mpuWrite(MPU_ACCEL_CFG,  0x00);  // ±2g
  delay(100);

  calibrateMPU();
  WiFi.disconnect();
}

void sendScanStatus(const char *state, int n) {
  StaticJsonDocument<96> doc;
  doc["t"]     = "scan";
  doc["state"] = state;
  doc["t_ms"]  = millis();
  if (n >= 0) doc["n"] = n;
  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
}

void sendIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  float axMs2 = (ax - axBias) * ACCEL_SCALE * GRAVITY;
  float ayMs2 = (ay - ayBias) * ACCEL_SCALE * GRAVITY;
  float mag   = sqrt(axMs2 * axMs2 + ayMs2 * ayMs2);
  int moving  = (mag > MOVE_THRESHOLD) ? 1 : 0;

  if (moving == 0) {
    gzBias = gzBias * 0.995f + gz * 0.005f;  // EMA bias correction, τ≈2s@100Hz
  }

  float gzRad = (gz - gzBias) * GYRO_SCALE * PI / 180.0;

  StaticJsonDocument<128> doc;
  doc["t"]    = "i";
  doc["t_ms"] = millis();
  doc["gz"]   = gzRad;
  doc["ax"]   = axMs2;
  doc["ay"]   = ayMs2;
  doc["mv"]   = moving;
  serializeJson(doc, Serial);
  Serial.println();
  Serial.flush();
}

int sendScan() {
  int n = WiFi.scanNetworks();
  if (n <= 0) return n;

  int count = min(n, MAX_APS_PER_SCAN);

  static StaticJsonDocument<1536> doc;  // static: keep off stack (1.5 KB)
  doc.clear();
  doc["t"]    = "s";
  doc["t_ms"] = millis();
  JsonArray aps = doc.createNestedArray("a");

  for (int i = 0; i < count; i++) {
    uint8_t bssidBuf[6];
    WiFi.BSSID(i, bssidBuf);
    char bssidStr[18];
    snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             bssidBuf[0], bssidBuf[1], bssidBuf[2],
             bssidBuf[3], bssidBuf[4], bssidBuf[5]);

    JsonObject ap = aps.createNestedObject();
    ap["n"] = WiFi.SSID(i);
    ap["b"] = bssidStr;
    ap["r"] = WiFi.RSSI(i);
    ap["c"] = WiFi.channel(i);
  }

  serializeJson(doc, Serial);
  Serial.println();
  return n;
}

void loop() {
  unsigned long now = millis();

  if (now - lastImuMs >= IMU_INTERVAL_MS) {
    lastImuMs = now;
    sendIMU();
  }

  if (now - lastScanMs >= SCAN_INTERVAL_MS) {
    sendScanStatus("begin", -1);
    startFifo();
    int n = sendScan();
    stopFifo();
    drainAndSendFifo(scanStartMs);
    sendScanStatus("end", n);
    lastScanMs = millis();  // スキャン完了後にリセット → 次まで必ず2秒のIMU期間確保
    lastImuMs  = millis();
  }
}
