// XIAO ESP32-S3 sensor bridge — FIRMWARE v2 (IMU + GPS + PPS)
// -----------------------------------------------------------------------------
// One microsecond clock (esp_timer) stamps everything:
//   * ICM-42688-P samples at 200 Hz (SPI, data-ready interrupt)   -> IMU packets
//   * u-blox M10 PPS rising edges (GPIO interrupt)                -> PPS packets
//   * u-blox M10 NMEA sentences (UART, 9600 8N1)                  -> GPS packets
//
// WIRING (XIAO ESP32-S3 silkscreen names — VERIFY against your boards):
//   ICM-42688-P:  3V3->VCC  GND->GND  D8->SPC/SCK  D10->SDI/MOSI
//                 D9->SDO/MISO  D7->CS  D6->INT1
//   u-blox M10 :  3V3->VCC  GND->GND  M10 TX -> D5   M10 PPS -> D4
//                 (M10 RX unconnected; we never send it commands)
//
// PACKET FORMATS on USB-CDC (all little-endian, CRC16-CCITT-FALSE):
//   IMU  (32 B): AA 55 | u64 t_us | i16 ax ay az gx gy gz | i16 temp
//                | u16 seq | u16 crc(bytes 2..25) | 4 pad zeros
//   PPS  (16 B): AA 57 | u64 t_us | u16 seq | u16 crc(bytes 2..11) | 2 pad
//   GPS  (var) : AA 56 | u8 len | ascii NMEA sentence (no CR/LF, <=90 B)
//                | u16 crc(over len byte + payload)
//
// Scales (must match laptop node): accel ±8 g (4096 LSB/g),
// gyro ±1000 dps (32.8 LSB/dps).
// VERIFY-BEFORE-TRUST: the ICM register values below (GYRO_CFG0/ACCEL_CFG0
// field encodings) were written from the ICM-42688-P datasheet tables —
// confirm against the datasheet revision in hand before first flight use.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>

// ---------- pins ----------
static const int PIN_CS   = D7;
static const int PIN_INT1 = D6;
static const int PIN_GPS_RX = D5;   // XIAO RX  <- M10 TX
static const int PIN_PPS  = D4;
static const uint32_t SPI_HZ = 8000000;

// ---------- ICM-42688-P registers (User Bank 0) ----------
enum : uint8_t {
  REG_WHO_AM_I    = 0x75,  // expect 0x47
  REG_DEVICE_CFG  = 0x11,
  REG_PWR_MGMT0   = 0x4E,
  REG_GYRO_CFG0   = 0x4F,
  REG_ACCEL_CFG0  = 0x50,
  REG_INT_CONFIG  = 0x14,
  REG_INT_SOURCE0 = 0x65,
  REG_TEMP_DATA1  = 0x1D,  // temp(2) accel(6) gyro(6) contiguous
};

// ---------- shared state ----------
volatile bool imuReady = false;
volatile uint64_t ppsTime = 0;      // written in ISR, consumed in loop
volatile bool ppsFlag = false;

void IRAM_ATTR onImuInt() { imuReady = true; }
void IRAM_ATTR onPps() { ppsTime = (uint64_t)esp_timer_get_time(); ppsFlag = true; }

// ---------- SPI helpers ----------
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

// ---------- NMEA line assembly ----------
static char nmeaBuf[100];
static uint8_t nmeaLen = 0;

void sendGpsPacket(const char* line, uint8_t len) {
  // AA 56 | len | payload | crc16(len..payload)
  uint8_t pkt[3 + 100 + 2];
  pkt[0] = 0xAA; pkt[1] = 0x56; pkt[2] = len;
  memcpy(&pkt[3], line, len);
  uint16_t c = crc16(&pkt[2], (size_t)len + 1);
  pkt[3 + len]     = (uint8_t)(c & 0xFF);
  pkt[3 + len + 1] = (uint8_t)(c >> 8);
  Serial.write(pkt, 3 + len + 2);
}

void sendPpsPacket(uint64_t t) {
  static uint16_t seq = 0;
  uint8_t pkt[16] = {0};
  pkt[0] = 0xAA; pkt[1] = 0x57;
  memcpy(&pkt[2], &t, 8);
  memcpy(&pkt[10], &seq, 2); seq++;
  uint16_t c = crc16(&pkt[2], 10);
  memcpy(&pkt[12], &c, 2);
  Serial.write(pkt, sizeof(pkt));
}

void setup() {
  Serial.begin(115200);                        // USB-CDC (baud nominal)
  Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, -1);  // GPS NMEA in, no TX

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT1, INPUT);
  // PULLDOWN, not plain INPUT. PPS has never been wired, so D4 sits
  // unterminated with a RISING interrupt armed on it and rings on
  // electrical transients -- the 2026-08-01 outdoor bag caught 1,809 edges
  // in a 2.6 s burst at ~50 kHz. This fixes that. The u-blox TIMEPULSE
  // output idles low and pulses high, so a pull-down is also the correct
  // termination once PPS IS wired.
  // NOTE: this does NOT protect against a signal being wired to D4 by
  // mistake. Landing GPS TX here instead of D5 gave ~670 Hz on 2026-08-01,
  // and a UART actively drives the line -- no internal pull-down can
  // suppress that. Different fault, wiring fix only.
  pinMode(PIN_PPS, INPUT_PULLDOWN);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));

  wr(REG_DEVICE_CFG, 0x01); delay(2);          // soft reset
  while (rd(REG_WHO_AM_I) != 0x47) delay(100); // halt here if wiring is wrong

  wr(REG_GYRO_CFG0,  0x27);  // ±1000 dps, ODR 200 Hz (DS-000347 verified: FS=001, ODR=0111)
  wr(REG_ACCEL_CFG0, 0x27);  // ±8 g,      ODR 200 Hz (DS-000347 verified)
  wr(REG_PWR_MGMT0,  0x0F);  // gyro+accel low-noise
  delay(45);
  wr(REG_INT_SOURCE0, 0x08); // data-ready -> INT1
  wr(REG_INT_CONFIG,  0x03); // INT1 push-pull, active high, pulsed

  attachInterrupt(digitalPinToInterrupt(PIN_INT1), onImuInt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS),  onPps,    RISING);
}

void loop() {
  // ---- IMU: highest priority ----
  if (imuReady) {
    imuReady = false;
    static uint16_t seq = 0;
    uint64_t t = (uint64_t)esp_timer_get_time();
    uint8_t raw[14];
    rdBurst(REG_TEMP_DATA1, raw, sizeof(raw));   // temp, ax..az, gx..gz (BE)

    uint8_t pkt[32] = {0};
    pkt[0] = 0xAA; pkt[1] = 0x55;
    memcpy(&pkt[2], &t, 8);
    for (int i = 0; i < 6; i++) {                // accel+gyro start at raw[2]
      pkt[10 + 2*i]     = raw[2 + 2*i + 1];      // BE -> LE
      pkt[10 + 2*i + 1] = raw[2 + 2*i];
    }
    int16_t temp = (int16_t)((raw[0] << 8) | raw[1]);
    memcpy(&pkt[22], &temp, 2);
    memcpy(&pkt[24], &seq, 2); seq++;
    uint16_t c = crc16(&pkt[2], 24);
    memcpy(&pkt[26], &c, 2);
    Serial.write(pkt, sizeof(pkt));
  }

  // ---- PPS: forward the captured edge time ----
  if (ppsFlag) {
    noInterrupts();
    uint64_t t = ppsTime;
    ppsFlag = false;
    interrupts();
    sendPpsPacket(t);
  }

  // ---- GPS: assemble NMEA lines, forward complete ones ----
  while (Serial1.available()) {
    char ch = (char)Serial1.read();
    if (ch == '\n' || ch == '\r') {
      if (nmeaLen > 5 && nmeaBuf[0] == '$') sendGpsPacket(nmeaBuf, nmeaLen);
      nmeaLen = 0;
    } else if (nmeaLen < sizeof(nmeaBuf)) {
      nmeaBuf[nmeaLen++] = ch;
    } else {
      nmeaLen = 0;  // overflow: discard malformed line
    }
  }
}
