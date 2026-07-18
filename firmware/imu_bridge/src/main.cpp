// XIAO ESP32-S3 — ICM-42688-P USB timestamping bridge
// -----------------------------------------------------------------------------
// Why this exists: LIO needs IMU samples stamped AT MEASUREMENT TIME, not at USB
// arrival time. This MCU services the IMU's data-ready interrupt, stamps each
// sample with esp_timer microseconds, and streams fixed 32-byte binary packets
// over USB-CDC. The laptop node (ros2/imu_bridge_node) maps MCU time -> ROS time.
//
// Wiring (XIAO ESP32-S3 <-> ICM-42688-P breakout, 3.3 V logic):
//   3V3->VCC  GND->GND  D8/SCK->SPC  D10/MOSI->SDA(SDI)  D9/MISO->SDO(AD0)
//   D7->CS    D6->INT1
// VERIFY against your breakout silkscreen + the ICM-42688-P datasheet before power.
// MPU6050 stand-in: use the i2c_mpu6050 branch (TODO) — this file is ICM/SPI only.
//
// Packet format (little-endian, 32 bytes):
//   0xAA 0x55 | uint64 t_us | int16 ax ay az | int16 gx gy gz | int16 temp
//   | uint16 seq | uint16 crc16-ccitt(bytes 2..29)
// Scales: accel ±8 g (4096 LSB/g), gyro ±1000 dps (32.8 LSB/dps) — must match
// the REG config below and the laptop node's constants.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>

static const int PIN_CS = D7;
static const int PIN_INT1 = D6;
static const uint32_t SPI_HZ = 8000000;

// ICM-42688-P registers (User Bank 0) — verify against datasheet rev on hand.
enum : uint8_t {
  REG_WHO_AM_I    = 0x75,  // expect 0x47
  REG_DEVICE_CFG  = 0x11,
  REG_PWR_MGMT0   = 0x4E,
  REG_GYRO_CFG0   = 0x4F,
  REG_ACCEL_CFG0  = 0x50,
  REG_INT_CONFIG  = 0x14,
  REG_INT_SOURCE0 = 0x65,
  REG_TEMP_DATA1  = 0x1D,  // temp(2) accel(6) gyro(6) contiguous from here
};

volatile bool dataReady = false;
void IRAM_ATTR onInt() { dataReady = true; }

uint8_t rd(uint8_t reg) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return v;
}
void wr(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
}
void rdBurst(uint8_t reg, uint8_t* buf, size_t n) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (size_t i = 0; i < n; i++) buf[i] = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
}

uint16_t crc16(const uint8_t* d, size_t n) {  // CCITT-FALSE
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
  }
  return c;
}

void setup() {
  Serial.begin(115200);           // USB-CDC; baud is nominal
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT1, INPUT);
  SPI.begin();                    // XIAO default SPI pins
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));

  wr(REG_DEVICE_CFG, 0x01); delay(2);        // soft reset
  while (rd(REG_WHO_AM_I) != 0x47) delay(100);

  wr(REG_GYRO_CFG0,  0x26);  // ±1000 dps, ODR 200 Hz  (verify field encoding!)
  wr(REG_ACCEL_CFG0, 0x26);  // ±8 g,      ODR 200 Hz
  wr(REG_PWR_MGMT0,  0x0F);  // gyro + accel low-noise mode
  delay(45);                 // gyro startup per datasheet
  wr(REG_INT_SOURCE0, 0x08); // data-ready -> INT1
  wr(REG_INT_CONFIG,  0x03); // INT1 push-pull, active high, pulsed

  attachInterrupt(digitalPinToInterrupt(PIN_INT1), onInt, RISING);
}

void loop() {
  static uint16_t seq = 0;
  if (!dataReady) return;
  dataReady = false;

  uint64_t t = (uint64_t)esp_timer_get_time();
  uint8_t raw[14];
  rdBurst(REG_TEMP_DATA1, raw, sizeof(raw));   // temp, ax..az, gx..gz (BE)

  uint8_t pkt[32];
  pkt[0] = 0xAA; pkt[1] = 0x55;
  memcpy(&pkt[2], &t, 8);
  // sensor data is big-endian in registers; swap to LE int16
  for (int i = 0; i < 6; i++) {                // 6 words: temp,ax,ay,az.. reorder below
    pkt[10 + 2*i]     = raw[2 + 2*i + 1];      // accel+gyro start at raw[2]
    pkt[10 + 2*i + 1] = raw[2 + 2*i];
  }
  int16_t temp = (int16_t)((raw[0] << 8) | raw[1]);
  memcpy(&pkt[22], &temp, 2);
  memcpy(&pkt[24], &seq, 2); seq++;
  uint16_t c = crc16(&pkt[2], 24);
  memcpy(&pkt[26], &c, 2);
  pkt[28] = pkt[29] = pkt[30] = pkt[31] = 0;   // pad to 32

  Serial.write(pkt, sizeof(pkt));
}
