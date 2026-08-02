// Waveshare ESP32-C6-LCD-1.47 + INA226 -> 12 V deep-cycle battery monitor
// -----------------------------------------------------------------------------
// Shows bus voltage, current, power and accumulated amp-hours for the pack that
// powers the rig, on the board's 172x320 ST7789. Standalone: it needs no laptop,
// so the pack can be checked with everything else switched off.
//
// PINOUT — LCD pins are from Waveshare's own documentation for this board,
// not guessed:
//     MOSI GPIO6   SCLK GPIO7   CS GPIO14   DC GPIO15   RST GPIO21   BL GPIO22
//     RGB LED GPIO8        SD card: CS GPIO4, MISO GPIO5 (unused here)
//
// I2C on GP18/GP19 — CONFIRMED against the board's own pinout diagram, which
// breaks out GP0-GP5 and GP9/12/13/18/19/20/23 on the headers and marks
// GP18 and GP19 as I2C-capable. 3V3(OUT) and GND sit on the left header.
// An I2C scan still runs at boot and reports on screen, so a wiring fault
// shows as a clear message rather than plausible-looking zeroes.
//
// WIRING the INA226, and the one that matters:
//     INA226 VCC -> 3V3(OUT)   left header, 3rd pin down. The part runs
//                              2.7-5.5 V, so 3.3 V needs no level shifter.
//     INA226 GND -> GND        left header, 2nd pin down. Must be common with
//                              the battery negative.
//     INA226 SDA -> GP18       right header
//     INA226 SCL -> GP19       right header
//     IN+ / IN-  -> across the shunt, in the battery POSITIVE lead (high side).
//                   The INA226's 0-36 V common-mode range is what allows this,
//                   and high-side keeps one common ground across laptop, lidar
//                   and converter.
//     *** FUSE AT THE BATTERY TERMINAL. *** A deep-cycle pack will push
//     hundreds of amps into a short and this is new wiring on a moving cart.
//
// SCALE CONSTANTS: bus LSB 1.25 mV and shunt LSB 2.5 uV are from the INA226
// datasheet and are the only two numbers the readings depend on. Current is
// computed as V_shunt / R_shunt directly rather than via the calibration
// register, so there is no CAL word to get wrong. CONFIRM BOTH against the
// datasheet revision in hand before trusting absolute values — the project
// rule is verify, never assume.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>

// ---------- pins ----------
static const int PIN_MOSI = 6;
static const int PIN_SCLK = 7;
static const int PIN_CS   = 14;
static const int PIN_DC   = 15;
static const int PIN_RST  = 21;
static const int PIN_BL   = 22;

static const int PIN_SDA  = 18;   // right header, I2C-capable per pinout
static const int PIN_SCL  = 19;   // right header, I2C-capable per pinout

// ---------- INA226 ----------
static const uint8_t INA226_ADDR   = 0x40;   // A0/A1 both to GND
static const uint8_t REG_CONFIG    = 0x00;
static const uint8_t REG_SHUNT_V   = 0x01;
static const uint8_t REG_BUS_V     = 0x02;
static const uint8_t REG_MANUF_ID  = 0xFE;   // expect 0x5449 ("TI")
static const uint8_t REG_DIE_ID    = 0xFF;   // expect 0x2260

static const float SHUNT_OHMS   = 0.002f;    // R002 on the module
static const float BUS_LSB_V    = 1.25e-3f;  // datasheet
static const float SHUNT_LSB_V  = 2.5e-6f;   // datasheet

// Averaging: 128 samples, 1.1 ms conversion on both channels, continuous.
// Roughly 0.3 s per reading, which is what makes the display readable instead
// of jittering on every motor transient.
static const uint16_t CONFIG_VALUE = 0x4727;

// ---------- battery thresholds: LEAD-ACID DEEP CYCLE ----------
// Confirmed chemistry 2026-08-01. Rested open-circuit voltages:
//     12.7 = 100 %   12.5 = 75 %   12.2 = 50 %   12.0 = 25 %   11.8 = flat
//
// The line that matters is 12.2 V, not 11.8. A deep-cycle pack taken
// repeatedly below 50 % depth of discharge loses cycle life fast, so the
// display goes amber there rather than waiting for the battery to be nearly
// empty. Red at 12.0 means stop now.
//
// THESE ARE RESTING VOLTAGES. Under load the pack sags, so a reading taken
// while the rig is drawing current understates the true state of charge --
// the display flags when that is happening rather than letting you misread a
// sag as a flat battery.
static const float V_FULL  = 12.7f;   // 100 %, rested
static const float V_GOOD  = 12.45f;  // ~65 %
static const float V_HALF  = 12.2f;   // 50 % -- do not habitually go below
static const float V_LOW   = 12.0f;   // 25 % -- stop
static const float V_EMPTY = 11.8f;   // bar floor

Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCLK, PIN_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, PIN_RST, 0 /*rotation*/,
                                      true /*IPS*/, 172, 320, 34, 0, 34, 0);

static bool  inaOk = false;
static float ahUsed = 0.0f, whUsed = 0.0f;
static uint32_t lastMs = 0;

// ---------- I2C helpers ----------
static bool readReg(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)INA226_ADDR, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

static bool writeReg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write(val >> 8);
  Wire.write(val & 0xFF);
  return Wire.endTransmission() == 0;
}

static void banner(const char *l1, const char *l2, uint16_t col) {
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(col);
  gfx->setTextSize(2);
  gfx->setCursor(6, 40);
  gfx->println(l1);
  gfx->setTextSize(1);
  gfx->setCursor(6, 80);
  gfx->println(l2);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  gfx->begin();
  gfx->fillScreen(RGB565_BLACK);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  uint16_t manuf = 0, die = 0;
  inaOk = readReg(REG_MANUF_ID, manuf) && readReg(REG_DIE_ID, die);

  if (!inaOk) {
    // Scan the bus so a wrong pin choice or address is visible immediately
    // rather than showing plausible-looking zeroes.
    String found = "";
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        found += "0x" + String(a, HEX) + " ";
      }
    }
    banner("NO INA226", found.length()
             ? ("found: " + found + "\n\ncheck address").c_str()
             : "nothing on the bus\n\ncheck SDA/SCL pins,\npower and ground", RGB565_RED);
    return;
  }

  writeReg(REG_CONFIG, CONFIG_VALUE);
  lastMs = millis();

  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(1);
  gfx->setCursor(6, 6);
  gfx->printf("INA226 ok  %04X/%04X", manuf, die);
  delay(800);
  gfx->fillScreen(RGB565_BLACK);
}

void loop() {
  if (!inaOk) { delay(1000); return; }

  uint16_t rawBus = 0, rawShunt = 0;
  if (!readReg(REG_BUS_V, rawBus) || !readReg(REG_SHUNT_V, rawShunt)) {
    banner("I2C LOST", "INA226 stopped responding", RGB565_RED);
    delay(1000);
    return;
  }

  float volts = rawBus * BUS_LSB_V;
  float amps  = ((int16_t)rawShunt) * SHUNT_LSB_V / SHUNT_OHMS;  // signed
  float watts = volts * amps;

  uint32_t now = millis();
  float dtH = (now - lastMs) / 3600000.0f;
  lastMs = now;
  ahUsed += amps * dtH;
  whUsed += watts * dtH;

  bool loaded = fabsf(amps) > 0.5f;
  uint16_t col = volts >= V_GOOD ? RGB565_GREEN : (volts >= V_HALF ? RGB565_YELLOW
                : (volts >= V_LOW ? RGB565_ORANGE : RGB565_RED));

  gfx->fillScreen(RGB565_BLACK);

  gfx->setTextColor(col);
  gfx->setTextSize(4);
  gfx->setCursor(8, 18);
  gfx->printf("%.2f", volts);
  gfx->setTextSize(2);
  gfx->print("V");

  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(8, 78);
  gfx->printf("%+.2f A", amps);
  gfx->setCursor(8, 104);
  gfx->printf("%+.1f W", watts);

  gfx->setTextSize(1);
  gfx->setTextColor(RGB565_CYAN);
  gfx->setCursor(8, 140);
  gfx->printf("used  %.3f Ah", ahUsed);
  gfx->setCursor(8, 154);
  gfx->printf("      %.1f Wh", whUsed);

  // A bar is easier to read at a glance than a number while walking.
  int barW = (int)(((volts - V_EMPTY) / (V_FULL - V_EMPTY)) * 156.0f);
  barW = constrain(barW, 0, 156);
  gfx->drawRect(8, 176, 156, 14, RGB565_WHITE);
  if (barW > 0) gfx->fillRect(9, 177, barW, 12, col);

  // Mark the 50 % line on the bar -- the number that protects cycle life.
  int halfX = 9 + (int)(((V_HALF - V_EMPTY) / (V_FULL - V_EMPTY)) * 156.0f);
  gfx->drawFastVLine(halfX, 174, 18, RGB565_WHITE);

  gfx->setTextColor(loaded ? RGB565_ORANGE : RGB565_DARKGREY);
  gfx->setCursor(8, 200);
  gfx->print(loaded ? "UNDER LOAD - reads low" : "rested  | mark = 50%");
  gfx->setTextColor(RGB565_DARKGREY);
  gfx->setCursor(8, 214);
  gfx->print("lead-acid deep cycle");

  Serial.printf("%.3f V  %+.3f A  %+.2f W  %.4f Ah\n", volts, amps, watts, ahUsed);
  delay(500);
}
