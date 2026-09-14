/*
 * Cockpit Gauge Controller
 * https://github.com/sarollahi/CockpitGaugeController
 *
 * Arduino Mega 2560 firmware for an eight-axis physical flight-simulator
 * instrument cluster.
 *
 * Copyright (C) 2026 sarollahi
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Mechanical instrument designs used as the basis for this build are credited
 * in the repository README and NOTICE.md. This firmware is a separate
 * implementation and is not a fork of the MartinRusk/Sixpack firmware.
 */

#include <AccelStepper.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ============================================================
// Feature toggles
// ============================================================
#define ENABLE_TURN_SLIP 1
#define ENABLE_ALT       1
#define ENABLE_BARO      1
#define ENABLE_DG        1   // Gyro compass (card + bug)
#define ENABLE_VSI       1   // Variometer (vertical speed)

// Keep button-state diagnostics available without making the realtime loop
// depend on them. Set to 0 if the serial link should contain protocol data only.
#define ENABLE_BUTTON_STATE_DEBUG 1

// ============================================================
// Display
// ============================================================
class MyST7735 : public Adafruit_ST7735 {
 public:
  using Adafruit_ST77xx::setColRowStart;
  MyST7735(int8_t cs, int8_t dc, int8_t rst)
      : Adafruit_ST7735(cs, dc, rst) {}
};

// Hardware SPI on Mega: SCK=52, MOSI=51
#define TFT_CS   A9
#define TFT_DC   A8
#define TFT_RST  A10
MyST7735 tft(TFT_CS, TFT_DC, TFT_RST);

// ============================================================
// Homing mode
// ============================================================
// 1 = assume current physical positions as zero at boot (no movement)
// 0 = push supported gauges to their mechanical stops at boot (soft-home)
#define ASSUME_HOME_ON_BOOT 1

// ============================================================
// Hardware pins
// ============================================================
const uint8_t ASI_IN1 = 2,  ASI_IN2 = 3,  ASI_IN3 = 4,  ASI_IN4 = 5;
const uint8_t TURN_IN1 = 22, TURN_IN2 = 23, TURN_IN3 = 24, TURN_IN4 = 25;
const uint8_t SLIP_IN1 = 26, SLIP_IN2 = 27, SLIP_IN3 = 28, SLIP_IN4 = 29;
const uint8_t VSI_IN1  = 30, VSI_IN2  = 31, VSI_IN3  = 32, VSI_IN4  = 33;
const uint8_t ALT_IN1  = 34, ALT_IN2  = 35, ALT_IN3  = 36, ALT_IN4  = 37;
const uint8_t BARO_IN1 = 38, BARO_IN2 = 39, BARO_IN3 = 40, BARO_IN4 = 41;
const uint8_t DG_IN1   = 42, DG_IN2   = 43, DG_IN3   = 44, DG_IN4   = 45;
const uint8_t BUG_IN1  = 46, BUG_IN2  = 47, BUG_IN3  = 48, BUG_IN4  = 49;

// Buttons are active LOW with INPUT_PULLUP.
const uint8_t BTN_CW  = 11;
const uint8_t BTN_CCW = 12;
const uint8_t BTN_SEL = 13;

// Mega LED_BUILTIN is also pin 13, so it cannot be used while BTN_SEL is on 13.
// Heartbeat functionality is preserved on spare pin 10. Connect an external LED
// (with a resistor) if you want the indicator. Change this pin if needed.
const uint8_t HEARTBEAT_LED_PIN = 10;

// AccelStepper HALF4WIRE expects pins in 1,3,2,4 order.
AccelStepper stepASI(AccelStepper::HALF4WIRE, ASI_IN1, ASI_IN3, ASI_IN2, ASI_IN4);
#if ENABLE_TURN_SLIP
AccelStepper stepTURN(AccelStepper::HALF4WIRE, TURN_IN1, TURN_IN3, TURN_IN2, TURN_IN4);
AccelStepper stepSLIP(AccelStepper::HALF4WIRE, SLIP_IN1, SLIP_IN3, SLIP_IN2, SLIP_IN4);
#endif
#if ENABLE_ALT
AccelStepper stepALT(AccelStepper::HALF4WIRE, ALT_IN1, ALT_IN3, ALT_IN2, ALT_IN4);
#endif
#if ENABLE_BARO
AccelStepper stepBARO(AccelStepper::HALF4WIRE, BARO_IN1, BARO_IN3, BARO_IN2, BARO_IN4);
#endif
#if ENABLE_DG
AccelStepper stepDG(AccelStepper::HALF4WIRE, DG_IN1, DG_IN3, DG_IN2, DG_IN4);
AccelStepper stepBUG(AccelStepper::HALF4WIRE, BUG_IN1, BUG_IN3, BUG_IN2, BUG_IN4);
#endif
#if ENABLE_VSI
AccelStepper stepVSI(AccelStepper::HALF4WIRE, VSI_IN1, VSI_IN3, VSI_IN2, VSI_IN4);
#endif

// ============================================================
// Mechanics / scaling
// ============================================================
// Keep separate values so each gauge can be calibrated independently later.
// Defaults preserve the original 4096-step behavior.
constexpr float NOMINAL_STEPS_PER_REV = 4096.0f;
constexpr float IAS_STEPS_PER_REV     = 4096.0f;
constexpr float TURN_STEPS_PER_REV    = 4096.0f;
constexpr float SLIP_STEPS_PER_REV    = 4096.0f;
constexpr float ALT_STEPS_PER_REV     = 4096.0f;
constexpr float DG_STEPS_PER_REV      = 4096.0f;
constexpr float BUG_STEPS_PER_REV     = 4096.0f;
constexpr float VSI_STEPS_PER_REV     = 4096.0f;

constexpr float IAS_STEPS_PER_DEG  = IAS_STEPS_PER_REV / 360.0f;
constexpr float TURN_STEPS_PER_DEG = TURN_STEPS_PER_REV / 360.0f;
constexpr float SLIP_STEPS_PER_DEG = SLIP_STEPS_PER_REV / 360.0f;
constexpr float ALT_STEPS_PER_DEG  = ALT_STEPS_PER_REV / 360.0f;
constexpr float DG_STEPS_PER_DEG   = DG_STEPS_PER_REV / 360.0f;
constexpr float BUG_STEPS_PER_DEG  = BUG_STEPS_PER_REV / 360.0f;
constexpr float VSI_STEPS_PER_DEG  = VSI_STEPS_PER_REV / 360.0f;

// Airspeed (per Sixpack): 185.8 units/turn, offset -40 kts
constexpr float IAS_FEED_UNITS_PER_TURN = 185.8f;
constexpr float IAS_ZERO_KTS             = 40.0f;
constexpr float IAS_DEG_PER_KT           = 360.0f / IAS_FEED_UNITS_PER_TURN;
constexpr bool  IAS_ALLOW_4_TURNS        = false;
constexpr float IAS_MAX_FACE_ARC_DEG     = 310.0f;

// Turn + Slip
constexpr float TURN_MAX_DEG = 30.0f;
constexpr float SLIP_GAIN    = 4.0f;
constexpr float SLIP_MAX_DEG = 16.0f;

// Altimeter needle (~1000 ft per motor revolution)
constexpr float ALT_FEET_PER_TURN = 1000.0f;
constexpr float ALT_DEG_PER_FT    = 360.0f / ALT_FEET_PER_TURN;
constexpr long  ALT_WRAP_STEPS    = (long)ALT_STEPS_PER_REV;

// Baro/Kollsman. This experimentally calibrated direct step value is the
// single source of truth; the unused BARO_DEG_PER_HPA value was removed.
constexpr float BARO_STD_INHG      = 29.92f;
constexpr float BARO_STD_HPA       = 1013.25f;
constexpr float BARO_STEPS_PER_HPA = 97.1429f;
constexpr long  BARO_WRAP_STEPS    = (long)(NOMINAL_STEPS_PER_REV * 8.0f);

// DG card and bug
constexpr long DG_WRAP_STEPS  = (long)DG_STEPS_PER_REV;
constexpr long BUG_WRAP_STEPS = (long)BUG_STEPS_PER_REV;

// VSI mapping
constexpr float VSI_FULL_SCALE_FPM = 2000.0f;
constexpr float VSI_SWEEP_DEG      = 250.0f;

// ============================================================
// Directions / trims
// ============================================================
constexpr int DIR_IAS  = -1;
constexpr int DIR_TURN = +1;
constexpr int DIR_SLIP = +1;
constexpr int DIR_ALT  = +1;
constexpr int DIR_BARO = +1;
constexpr int DIR_DG   = +1;
constexpr int DIR_BUG  = +1;
constexpr int DIR_VSI  = +1;

// Trims exist regardless of feature toggles so EEPROM/TFT code remains valid
// and every feature toggle can actually be switched off without compile errors.
long TRIM_IAS  = 0;
long TRIM_TURN = 0;
long TRIM_SLIP = 0;
long TRIM_ALT  = 0;
long TRIM_BARO = 0;
long TRIM_DG   = 0;
long TRIM_BUG  = 0;
long TRIM_VSI  = 0;

enum GaugeId : uint8_t {
  GAUGE_ASI = 0,
  GAUGE_TURN,
  GAUGE_SLIP,
  GAUGE_ALT,
  GAUGE_BARO,
  GAUGE_DG,
  GAUGE_BUG,
  GAUGE_VSI,
  GAUGE_COUNT
};

const char* const gaugeNames[GAUGE_COUNT] = {
    "ASI", "TURN", "SLIP", "ALT", "BARO", "DG", "BUG", "VSI"};

long* const gaugeTrims[GAUGE_COUNT] = {
    &TRIM_IAS, &TRIM_TURN, &TRIM_SLIP, &TRIM_ALT,
    &TRIM_BARO, &TRIM_DG, &TRIM_BUG, &TRIM_VSI};

uint8_t selectedGauge = GAUGE_ASI;

// ============================================================
// Motion / smoothing / parking
// ============================================================
constexpr float MAX_SPEED_STEPS = 1600.0f;
constexpr float ACCEL_STEPS     = 3000.0f;
constexpr float BARO_MAX_SPEED  = 800.0f;
constexpr float BARO_ACCEL      = 1500.0f;

// Fixed-rate control loop. Smoothing no longer depends on CPU loop speed.
constexpr unsigned long CONTROL_INTERVAL_MS = 20;  // 50 Hz
constexpr float IAS_ALPHA  = 0.20f;
constexpr float TURN_ALPHA = 0.20f;
constexpr float SLIP_ALPHA = 0.20f;
constexpr float ALT_ALPHA  = 0.10f;
constexpr float BARO_ALPHA = 0.15f;
constexpr float DG_ALPHA   = 0.15f;
constexpr float BUG_ALPHA  = 0.15f;
constexpr float VSI_ALPHA  = 0.25f;

constexpr unsigned long IDLE_TIMEOUT_MS = 800;
constexpr bool  PARK_ON_IDLE    = false;
constexpr float PARK_IAS_KTS    = 0.0f;
constexpr float PARK_TURN_DEG   = 0.0f;
constexpr float PARK_SLIP_DEG   = 0.0f;
constexpr float PARK_ALT_FT     = 0.0f;
constexpr float PARK_BARO_INHG  = BARO_STD_INHG;
constexpr float PARK_HDG_DEG    = 0.0f;
constexpr float PARK_BUG_DEG    = 0.0f;
constexpr float PARK_VSI_FPM    = 0.0f;

// Zero-hardware position recovery. The Mega cannot physically discover an
// absolute shaft angle, so we persist the open-loop position estimate. A clean
// PARK/SHUTDOWN record is authoritative; journal checkpoints after an unclean
// power loss are deliberately treated as estimates.
constexpr unsigned long POSITION_CHECKPOINT_INTERVAL_MS = 60000UL;   // 1 min, only when stationary
constexpr unsigned long PARK_SETTLE_MS = 250UL;

// ============================================================
// Soft-home (unchanged in concept; no physical sensors added)
// ============================================================
constexpr int   HOME_DIR_ASI  = -1;
constexpr int   HOME_DIR_TURN = -1;
constexpr int   HOME_DIR_SLIP = -1;
constexpr int   HOME_DIR_ALT  = -1;
constexpr int   HOME_DIR_BARO = -1;
constexpr float HOME_REVS_ASI  = 1.5f;
constexpr float HOME_REVS_TURN = 1.0f;
constexpr float HOME_REVS_SLIP = 1.0f;
constexpr float HOME_REVS_ALT  = 2.0f;
constexpr float HOME_REVS_BARO = 1.0f;
constexpr float HOME_SPEED     = 400.0f;
constexpr float HOME_ACCEL     = 1200.0f;

// ============================================================
// Serial input state
// ============================================================
constexpr size_t RX_BUFFER_SIZE = 221;
constexpr uint8_t MAX_SERIAL_BYTES_PER_LOOP = 48;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxPos = 0;
bool rxOverflow = false;
unsigned long lastRxMs = 0;

// BAROSTEP/BAROSET manual debug override. A normal BARO: token releases it.
bool baroManualOverride = false;
long baroManualTarget = 0;

// ============================================================
// Measurement/filter state
// ============================================================
float ias_knots_meas = 0.0f, ias_knots_filt = 0.0f;
#if ENABLE_TURN_SLIP
float turn_deg_meas = 0.0f, turn_deg_filt = 0.0f;
float slip_deg_meas = 0.0f, slip_deg_filt = 0.0f;
#endif
#if ENABLE_ALT
float alt_ft_meas = 0.0f, alt_ft_filt = 0.0f;
#endif
#if ENABLE_BARO
float baro_inhg_meas = BARO_STD_INHG, baro_inhg_filt = BARO_STD_INHG;
#endif
#if ENABLE_DG
float hdg_deg_meas = 0.0f, hdg_deg_filt = 0.0f;
float bug_deg_meas = 0.0f, bug_deg_filt = 0.0f;
#endif
#if ENABLE_VSI
float vsi_fpm_meas = 0.0f, vsi_fpm_filt = 0.0f;
#endif

unsigned long lastControlMs = 0;

// ============================================================
// Button state
// ============================================================
constexpr unsigned long BUTTON_DEBOUNCE_MS        = 50;
constexpr unsigned long BUTTON_HOLD_DELAY         = 500;
constexpr unsigned long CONTINUOUS_MOVE_INTERVAL  = 100;
constexpr long ADJUSTMENT_STEPS                   = 64;

struct DebouncedButton {
  uint8_t pin;
  bool rawPressed;
  bool stablePressed;
  bool previousStablePressed;
  unsigned long rawChangedAt;

  void begin(uint8_t p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
    rawPressed = !digitalRead(pin);
    stablePressed = rawPressed;
    previousStablePressed = stablePressed;
    rawChangedAt = millis();
  }

  void update(unsigned long now) {
    previousStablePressed = stablePressed;
    const bool newRawPressed = !digitalRead(pin);

    if (newRawPressed != rawPressed) {
      rawPressed = newRawPressed;
      rawChangedAt = now;
    }

    if (stablePressed != rawPressed &&
        (unsigned long)(now - rawChangedAt) >= BUTTON_DEBOUNCE_MS) {
      stablePressed = rawPressed;
    }
  }

  bool pressedEdge() const {
    return stablePressed && !previousStablePressed;
  }

  bool releasedEdge() const {
    return !stablePressed && previousStablePressed;
  }
};

DebouncedButton buttonSel;
DebouncedButton buttonCw;
DebouncedButton buttonCcw;

bool btnCwHolding = false;
bool btnCcwHolding = false;
unsigned long btnCwHoldStart = 0;
unsigned long btnCcwHoldStart = 0;
unsigned long lastCwMove = 0;
unsigned long lastCcwMove = 0;

// Non-blocking button-test mode (preserves TEST:BUTTONS without freezing motors)
bool buttonTestActive = false;
unsigned long buttonTestStartedMs = 0;
unsigned long buttonTestLastReportMs = 0;
constexpr unsigned long BUTTON_TEST_DURATION_MS = 10000;
constexpr unsigned long BUTTON_TEST_REPORT_MS   = 100;

// ============================================================
// EEPROM calibration
// ============================================================
constexpr uint32_t CAL_MAGIC = 0x43414C31UL;  // "CAL1"
constexpr uint16_t CAL_VERSION = 1;
constexpr unsigned long EEPROM_SAVE_DELAY_MS = 1000;

struct CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  int32_t trimIAS;
  int32_t trimTurn;
  int32_t trimSlip;
  int32_t trimAlt;
  int32_t trimBaro;
  int32_t trimDG;
  int32_t trimBug;
  int32_t trimVSI;
  uint32_t checksum;
};

bool calibrationDirty = false;
unsigned long calibrationDirtyAt = 0;

// ============================================================
// EEPROM position journal / clean shutdown state
// ============================================================
constexpr uint32_t POS_MAGIC = 0x504F5331UL;  // "POS1"
constexpr uint16_t POS_VERSION = 1;
constexpr uint16_t POS_FLAG_CLEAN = 0x0001;
constexpr uint16_t POS_FLAG_EXACT = 0x0002;

struct PositionData {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint32_t sequence;
  int32_t posASI;
  int32_t posTurn;
  int32_t posSlip;
  int32_t posAlt;
  int32_t posBaro;
  int32_t posDG;
  int32_t posBug;
  int32_t posVSI;
  uint32_t checksum;
};

int positionJournalLatestSlot = -1;
uint32_t positionJournalLatestSequence = 0;
PositionData lastPositionRecord = {};
bool havePositionRecord = false;
bool positionEstimateExact = false;
bool parkedCleanPersisted = false;
bool explicitParkRequested = false;
bool haveSimulatorData = false;
bool parkSettling = false;
unsigned long parkSettlingSince = 0;
unsigned long lastPositionCheckpointMs = 0;

// ============================================================
// Helper functions
// ============================================================
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline long lroundf32(float v) {
  return (long)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

static inline float wrap360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static inline float shortestAngleDelta(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

long wrapToNearest(long current, long target, long period) {
  long cur = (current % period + period) % period;
  long tgt = (target % period + period) % period;
  long diff = tgt - cur;
  if (diff > period / 2) diff -= period;
  if (diff < -period / 2) diff += period;
  return current + diff;
}

bool isGaugeEnabled(uint8_t gauge) {
  switch (gauge) {
    case GAUGE_ASI:
      return true;
    case GAUGE_TURN:
    case GAUGE_SLIP:
#if ENABLE_TURN_SLIP
      return true;
#else
      return false;
#endif
    case GAUGE_ALT:
#if ENABLE_ALT
      return true;
#else
      return false;
#endif
    case GAUGE_BARO:
#if ENABLE_BARO
      return true;
#else
      return false;
#endif
    case GAUGE_DG:
    case GAUGE_BUG:
#if ENABLE_DG
      return true;
#else
      return false;
#endif
    case GAUGE_VSI:
#if ENABLE_VSI
      return true;
#else
      return false;
#endif
    default:
      return false;
  }
}

uint8_t nextEnabledGauge(uint8_t current) {
  for (uint8_t i = 1; i <= GAUGE_COUNT; ++i) {
    const uint8_t candidate = (current + i) % GAUGE_COUNT;
    if (isGaugeEnabled(candidate)) return candidate;
  }
  return GAUGE_ASI;
}

// ============================================================
// Gauge mappings
// ============================================================
long asiStepsFromIAS(float ias) {
  float motorDeg = (ias - IAS_ZERO_KTS) * IAS_DEG_PER_KT;
  motorDeg = IAS_ALLOW_4_TURNS
                 ? clampf(motorDeg, 0.0f, 1440.0f)
                 : clampf(motorDeg, 0.0f, IAS_MAX_FACE_ARC_DEG);
  return lroundf32(TRIM_IAS + DIR_IAS * motorDeg * IAS_STEPS_PER_DEG);
}

#if ENABLE_TURN_SLIP
long turnStepsFromDeg(float turn) {
  const float deg = clampf(turn, -TURN_MAX_DEG, +TURN_MAX_DEG);
  return lroundf32(TRIM_TURN + DIR_TURN * deg * TURN_STEPS_PER_DEG);
}

long slipStepsFromDeg(float slip) {
  const float deg = clampf(slip * SLIP_GAIN, -SLIP_MAX_DEG, +SLIP_MAX_DEG);
  return lroundf32(TRIM_SLIP + DIR_SLIP * deg * SLIP_STEPS_PER_DEG);
}
#endif

#if ENABLE_ALT
long altStepsFromFeet(float feet) {
  feet = clampf(feet, -1000.0f, 60000.0f);
  const float motorDeg = feet * ALT_DEG_PER_FT;
  const long target = lroundf32(TRIM_ALT + DIR_ALT * motorDeg * ALT_STEPS_PER_DEG);
  return wrapToNearest(stepALT.currentPosition(), target, ALT_WRAP_STEPS);
}
#endif

#if ENABLE_BARO
long baroStepsFromInHg(float inhg) {
  const float hpa = inhg * 33.863886f;
  const float dH = hpa - BARO_STD_HPA;
  const long target = lroundf32(TRIM_BARO + DIR_BARO * dH * BARO_STEPS_PER_HPA);
  return wrapToNearest(stepBARO.currentPosition(), target, BARO_WRAP_STEPS);
}
#endif

#if ENABLE_DG
long dgStepsFromDeg(float deg) {
  deg = wrap360(deg);
  const long target = lroundf32(TRIM_DG + DIR_DG * deg * DG_STEPS_PER_DEG);
  return wrapToNearest(stepDG.currentPosition(), target, DG_WRAP_STEPS);
}

long bugStepsFromDeg(float deg) {
  deg = wrap360(deg);
  const long target = lroundf32(TRIM_BUG + DIR_BUG * deg * BUG_STEPS_PER_DEG);
  return wrapToNearest(stepBUG.currentPosition(), target, BUG_WRAP_STEPS);
}
#endif

#if ENABLE_VSI
long vsiStepsFromFPM(float fpm) {
  const float v = clampf(fpm, -VSI_FULL_SCALE_FPM, +VSI_FULL_SCALE_FPM);
  const float motorDeg = (v / VSI_FULL_SCALE_FPM) * (VSI_SWEEP_DEG * 0.5f);
  return lroundf32(TRIM_VSI + DIR_VSI * motorDeg * VSI_STEPS_PER_DEG);
}
#endif

// ============================================================
// TFT
// ============================================================
void tftPrintCentered(int y, const char* s, uint8_t size) {
  tft.setTextSize(size);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  const int x = (int(tft.width()) - (int)w) / 2;
  tft.setCursor(x, y);
  tft.print(s);
}

void tftUpdate(bool force = false) {
  static int lastSel = -1;
  static long lastTrim = 0;

  const int sel = selectedGauge;
  const long trim = gaugeTrims[sel] ? *gaugeTrims[sel] : 0;
  if (!force && sel == lastSel && trim == lastTrim) return;

  char line1[24];
  char line2[24];
  snprintf(line1, sizeof(line1), "SEL: %s", gaugeNames[sel]);
  snprintf(line2, sizeof(line2), "TRIM: %ld", trim);

  const uint8_t size = 2;
  int16_t x1, y1;
  uint16_t w1, h1, w2, h2;
  tft.setTextSize(size);
  tft.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
  tft.getTextBounds(line2, 0, 0, &x1, &y1, &w2, &h2);

  const int gap = 4 * size;
  const int totalH = int(h1) + gap + int(h2);
  const int yStart = (int(tft.height()) - totalH) / 2;

  // Clear only the text area instead of repeatedly clearing the full display.
  const int clearTop = max(0, yStart - 2);
  const int clearBottom = min((int)tft.height(), yStart + totalH + 2);
  tft.fillRect(0, clearTop, tft.width(), clearBottom - clearTop, ST77XX_BLACK);

  tftPrintCentered(yStart, line1, size);
  tftPrintCentered(yStart + h1 + gap, line2, size);

  lastSel = sel;
  lastTrim = trim;
}

void tftInitSmall() {
  tft.initR(INITR_MINI160x80);
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.setColRowStart(1, 26);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tftUpdate(true);
}

// ============================================================
// Homing
// ============================================================
void softHome(AccelStepper& motor,
              int dir,
              float revs,
              float stepsPerRev,
              float normalMaxSpeed,
              float normalAcceleration) {
  const long steps = (long)(revs * stepsPerRev);
  motor.setMaxSpeed(HOME_SPEED);
  motor.setAcceleration(HOME_ACCEL);
  motor.moveTo(motor.currentPosition() + (long)dir * steps);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
  motor.setCurrentPosition(0);
  motor.setMaxSpeed(normalMaxSpeed);
  motor.setAcceleration(normalAcceleration);
}

void assumeHome() {
  stepASI.setCurrentPosition(0);
#if ENABLE_TURN_SLIP
  stepTURN.setCurrentPosition(0);
  stepSLIP.setCurrentPosition(0);
#endif
#if ENABLE_ALT
  stepALT.setCurrentPosition(0);
#endif
#if ENABLE_BARO
  stepBARO.setCurrentPosition(0);
#endif
#if ENABLE_DG
  stepDG.setCurrentPosition(0);
  stepBUG.setCurrentPosition(0);
#endif
#if ENABLE_VSI
  stepVSI.setCurrentPosition(0);
#endif
}

// ============================================================
// EEPROM calibration helpers
// ============================================================
uint32_t calibrationChecksum(const CalibrationData& data) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
  const size_t count = sizeof(CalibrationData) - sizeof(data.checksum);
  uint32_t hash = 2166136261UL;  // FNV-1a
  for (size_t i = 0; i < count; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

CalibrationData makeCalibrationData() {
  CalibrationData data;
  data.magic = CAL_MAGIC;
  data.version = CAL_VERSION;
  data.reserved = 0;
  data.trimIAS  = (int32_t)TRIM_IAS;
  data.trimTurn = (int32_t)TRIM_TURN;
  data.trimSlip = (int32_t)TRIM_SLIP;
  data.trimAlt  = (int32_t)TRIM_ALT;
  data.trimBaro = (int32_t)TRIM_BARO;
  data.trimDG   = (int32_t)TRIM_DG;
  data.trimBug  = (int32_t)TRIM_BUG;
  data.trimVSI  = (int32_t)TRIM_VSI;
  data.checksum = calibrationChecksum(data);
  return data;
}

void applyCalibrationData(const CalibrationData& data) {
  TRIM_IAS  = data.trimIAS;
  TRIM_TURN = data.trimTurn;
  TRIM_SLIP = data.trimSlip;
  TRIM_ALT  = data.trimAlt;
  TRIM_BARO = data.trimBaro;
  TRIM_DG   = data.trimDG;
  TRIM_BUG  = data.trimBug;
  TRIM_VSI  = data.trimVSI;
}

bool loadCalibration() {
  if (sizeof(CalibrationData) > (size_t)EEPROM.length()) {
    Serial.println(F("EEPROM too small for calibration data."));
    return false;
  }

  CalibrationData data;
  EEPROM.get(0, data);

  if (data.magic != CAL_MAGIC || data.version != CAL_VERSION) {
    Serial.println(F("No valid EEPROM calibration; using defaults."));
    return false;
  }

  if (data.checksum != calibrationChecksum(data)) {
    Serial.println(F("EEPROM calibration checksum failed; using defaults."));
    return false;
  }

  applyCalibrationData(data);
  Serial.println(F("Calibration loaded from EEPROM."));
  return true;
}

void saveCalibrationNow() {
  if (sizeof(CalibrationData) > (size_t)EEPROM.length()) return;
  const CalibrationData data = makeCalibrationData();
  EEPROM.put(0, data);  // AVR EEPROM.put() uses update semantics internally.
  calibrationDirty = false;
  Serial.println(F("Calibration saved to EEPROM."));
}

void markCalibrationDirty() {
  calibrationDirty = true;
  calibrationDirtyAt = millis();
}

void serviceCalibrationSave(unsigned long now) {
  if (calibrationDirty &&
      (unsigned long)(now - calibrationDirtyAt) >= EEPROM_SAVE_DELAY_MS) {
    saveCalibrationNow();
  }
}

// ============================================================
// Zero-hardware position persistence
// ============================================================
uint32_t positionChecksum(const PositionData& data) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
  const size_t count = sizeof(PositionData) - sizeof(data.checksum);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < count; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

int positionJournalStart() {
  return (int)sizeof(CalibrationData);
}

int positionJournalSlotCount() {
  const int available = EEPROM.length() - positionJournalStart();
  if (available <= 0) return 0;
  return available / (int)sizeof(PositionData);
}

int positionJournalAddress(int slot) {
  return positionJournalStart() + slot * (int)sizeof(PositionData);
}

void captureCurrentPositions(PositionData& data, bool clean) {
  memset(&data, 0, sizeof(data));
  data.magic = POS_MAGIC;
  data.version = POS_VERSION;
  data.flags = 0;
  if (clean) data.flags |= POS_FLAG_CLEAN;
  if (positionEstimateExact) data.flags |= POS_FLAG_EXACT;
  data.sequence = havePositionRecord ? positionJournalLatestSequence + 1UL : 1UL;

  data.posASI = (int32_t)stepASI.currentPosition();
#if ENABLE_TURN_SLIP
  data.posTurn = (int32_t)stepTURN.currentPosition();
  data.posSlip = (int32_t)stepSLIP.currentPosition();
#endif
#if ENABLE_ALT
  data.posAlt = (int32_t)stepALT.currentPosition();
#endif
#if ENABLE_BARO
  data.posBaro = (int32_t)stepBARO.currentPosition();
#endif
#if ENABLE_DG
  data.posDG = (int32_t)stepDG.currentPosition();
  data.posBug = (int32_t)stepBUG.currentPosition();
#endif
#if ENABLE_VSI
  data.posVSI = (int32_t)stepVSI.currentPosition();
#endif
  data.checksum = positionChecksum(data);
}

bool positionRecordValid(const PositionData& data) {
  if (data.magic != POS_MAGIC || data.version != POS_VERSION) return false;
  return data.checksum == positionChecksum(data);
}

bool sequenceNewer(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) > 0;
}

void applyPositionData(const PositionData& data) {
  stepASI.setCurrentPosition((long)data.posASI);
#if ENABLE_TURN_SLIP
  stepTURN.setCurrentPosition((long)data.posTurn);
  stepSLIP.setCurrentPosition((long)data.posSlip);
#endif
#if ENABLE_ALT
  stepALT.setCurrentPosition((long)data.posAlt);
#endif
#if ENABLE_BARO
  stepBARO.setCurrentPosition((long)data.posBaro);
#endif
#if ENABLE_DG
  stepDG.setCurrentPosition((long)data.posDG);
  stepBUG.setCurrentPosition((long)data.posBug);
#endif
#if ENABLE_VSI
  stepVSI.setCurrentPosition((long)data.posVSI);
#endif
}

bool loadLatestPositionRecord() {
  const int slots = positionJournalSlotCount();
  if (slots <= 0) {
    Serial.println(F("EEPROM has no room for position journal."));
    return false;
  }

  bool found = false;
  PositionData best = {};
  int bestSlot = -1;

  for (int slot = 0; slot < slots; ++slot) {
    PositionData candidate;
    EEPROM.get(positionJournalAddress(slot), candidate);
    if (!positionRecordValid(candidate)) continue;

    if (!found || sequenceNewer(candidate.sequence, best.sequence)) {
      best = candidate;
      bestSlot = slot;
      found = true;
    }
  }

  if (!found) {
    Serial.println(F("No saved motor-position record."));
    return false;
  }

  positionJournalLatestSlot = bestSlot;
  positionJournalLatestSequence = best.sequence;
  lastPositionRecord = best;
  havePositionRecord = true;
  applyPositionData(best);

  const bool clean = (best.flags & POS_FLAG_CLEAN) != 0;
  const bool exactAtSave = (best.flags & POS_FLAG_EXACT) != 0;
  positionEstimateExact = clean && exactAtSave;
  parkedCleanPersisted = clean;

  if (positionEstimateExact) {
    Serial.println(F("Motor positions restored from CLEAN exact record."));
  } else if (clean) {
    Serial.println(F("Motor positions restored from CLEAN record, but absolute origin is estimated."));
  } else {
    Serial.println(F("Motor positions restored from last checkpoint (UNCLEAN shutdown: estimated position)."));
  }
  return true;
}

bool positionsDifferFromLastRecord() {
  if (!havePositionRecord) return true;
  if ((int32_t)stepASI.currentPosition() != lastPositionRecord.posASI) return true;
#if ENABLE_TURN_SLIP
  if ((int32_t)stepTURN.currentPosition() != lastPositionRecord.posTurn) return true;
  if ((int32_t)stepSLIP.currentPosition() != lastPositionRecord.posSlip) return true;
#endif
#if ENABLE_ALT
  if ((int32_t)stepALT.currentPosition() != lastPositionRecord.posAlt) return true;
#endif
#if ENABLE_BARO
  if ((int32_t)stepBARO.currentPosition() != lastPositionRecord.posBaro) return true;
#endif
#if ENABLE_DG
  if ((int32_t)stepDG.currentPosition() != lastPositionRecord.posDG) return true;
  if ((int32_t)stepBUG.currentPosition() != lastPositionRecord.posBug) return true;
#endif
#if ENABLE_VSI
  if ((int32_t)stepVSI.currentPosition() != lastPositionRecord.posVSI) return true;
#endif
  return false;
}

bool savePositionRecord(bool clean) {
  const int slots = positionJournalSlotCount();
  if (slots <= 0) return false;

  PositionData data;
  captureCurrentPositions(data, clean);
  const int nextSlot = havePositionRecord
                           ? (positionJournalLatestSlot + 1) % slots
                           : 0;
  EEPROM.put(positionJournalAddress(nextSlot), data);

  PositionData verify;
  EEPROM.get(positionJournalAddress(nextSlot), verify);
  if (!positionRecordValid(verify) || verify.sequence != data.sequence) {
    Serial.println(F("ERROR: failed to verify motor-position EEPROM record."));
    return false;
  }

  positionJournalLatestSlot = nextSlot;
  positionJournalLatestSequence = data.sequence;
  lastPositionRecord = data;
  havePositionRecord = true;
  lastPositionCheckpointMs = millis();
  return true;
}

void printPositionStatus() {
  Serial.print(F("Position confidence: "));
  Serial.println(positionEstimateExact ? F("EXACT/TRACKED") : F("ESTIMATED"));
  Serial.print(F("Persisted state: "));
  if (!havePositionRecord) {
    Serial.println(F("NONE"));
  } else if ((lastPositionRecord.flags & POS_FLAG_CLEAN) != 0) {
    Serial.println(F("CLEAN"));
  } else {
    Serial.println(F("UNCLEAN/CHECKPOINT"));
  }
  Serial.print(F("ASI pos: ")); Serial.println(stepASI.currentPosition());
#if ENABLE_TURN_SLIP
  Serial.print(F("TURN pos: ")); Serial.println(stepTURN.currentPosition());
  Serial.print(F("SLIP pos: ")); Serial.println(stepSLIP.currentPosition());
#endif
#if ENABLE_ALT
  Serial.print(F("ALT pos: ")); Serial.println(stepALT.currentPosition());
#endif
#if ENABLE_BARO
  Serial.print(F("BARO pos: ")); Serial.println(stepBARO.currentPosition());
#endif
#if ENABLE_DG
  Serial.print(F("DG pos: ")); Serial.println(stepDG.currentPosition());
  Serial.print(F("BUG pos: ")); Serial.println(stepBUG.currentPosition());
#endif
#if ENABLE_VSI
  Serial.print(F("VSI pos: ")); Serial.println(stepVSI.currentPosition());
#endif
}

bool allSteppersAtTarget() {
  if (stepASI.distanceToGo() != 0) return false;
#if ENABLE_TURN_SLIP
  if (stepTURN.distanceToGo() != 0 || stepSLIP.distanceToGo() != 0) return false;
#endif
#if ENABLE_ALT
  if (stepALT.distanceToGo() != 0) return false;
#endif
#if ENABLE_BARO
  if (stepBARO.distanceToGo() != 0) return false;
#endif
#if ENABLE_DG
  if (stepDG.distanceToGo() != 0 || stepBUG.distanceToGo() != 0) return false;
#endif
#if ENABLE_VSI
  if (stepVSI.distanceToGo() != 0) return false;
#endif
  return true;
}

void invalidateCleanRecordBeforeMotion() {
  if (!parkedCleanPersisted) return;
  if (savePositionRecord(false)) {
    parkedCleanPersisted = false;
    Serial.println(F("Position record marked active/unclean before movement."));
  }
}

void servicePositionPersistence(unsigned long now, bool parkMode) {
  if (parkMode) {
    if (!allSteppersAtTarget()) {
      parkSettling = false;
      return;
    }

    if (!parkSettling) {
      parkSettling = true;
      parkSettlingSince = now;
      return;
    }

    if (!parkedCleanPersisted &&
        (unsigned long)(now - parkSettlingSince) >= PARK_SETTLE_MS) {
      if (savePositionRecord(true)) {
        parkedCleanPersisted = true;
        Serial.println(F("PARKED. Position saved CLEAN. Safe to power off."));
      }
    }
    return;
  }

  parkSettling = false;

  if ((unsigned long)(now - lastPositionCheckpointMs) >=
          POSITION_CHECKPOINT_INTERVAL_MS &&
      allSteppersAtTarget() &&
      positionsDifferFromLastRecord()) {
    if (savePositionRecord(true)) {
      parkedCleanPersisted = true;
      Serial.println(F("Stationary motor-position CLEAN checkpoint saved."));
    }
  }
}

// ============================================================
// Target generation / motor service
// ============================================================
void updateTargets(bool parkMode) {
  if (parkMode) {
    stepASI.moveTo(asiStepsFromIAS(PARK_IAS_KTS));
#if ENABLE_TURN_SLIP
    stepTURN.moveTo(turnStepsFromDeg(PARK_TURN_DEG));
    stepSLIP.moveTo(slipStepsFromDeg(PARK_SLIP_DEG));
#endif
#if ENABLE_ALT
    stepALT.moveTo(altStepsFromFeet(PARK_ALT_FT));
#endif
#if ENABLE_BARO
    if (baroManualOverride) {
      stepBARO.moveTo(baroManualTarget);
    } else {
      stepBARO.moveTo(baroStepsFromInHg(PARK_BARO_INHG));
    }
#endif
#if ENABLE_DG
    stepDG.moveTo(dgStepsFromDeg(PARK_HDG_DEG));
    stepBUG.moveTo(bugStepsFromDeg(PARK_BUG_DEG));
#endif
#if ENABLE_VSI
    stepVSI.moveTo(vsiStepsFromFPM(PARK_VSI_FPM));
#endif
    return;
  }

  stepASI.moveTo(asiStepsFromIAS(ias_knots_filt));
#if ENABLE_TURN_SLIP
  stepTURN.moveTo(turnStepsFromDeg(turn_deg_filt));
  stepSLIP.moveTo(slipStepsFromDeg(slip_deg_filt));
#endif
#if ENABLE_ALT
  stepALT.moveTo(altStepsFromFeet(alt_ft_filt));
#endif
#if ENABLE_BARO
  if (baroManualOverride) {
    stepBARO.moveTo(baroManualTarget);
  } else {
    stepBARO.moveTo(baroStepsFromInHg(baro_inhg_filt));
  }
#endif
#if ENABLE_DG
  stepDG.moveTo(dgStepsFromDeg(hdg_deg_filt));
  stepBUG.moveTo(bugStepsFromDeg(bug_deg_filt));
#endif
#if ENABLE_VSI
  stepVSI.moveTo(vsiStepsFromFPM(vsi_fpm_filt));
#endif
}

void updateFilters() {
  ias_knots_filt += IAS_ALPHA * (ias_knots_meas - ias_knots_filt);
#if ENABLE_TURN_SLIP
  turn_deg_filt += TURN_ALPHA * (turn_deg_meas - turn_deg_filt);
  slip_deg_filt += SLIP_ALPHA * (slip_deg_meas - slip_deg_filt);
#endif
#if ENABLE_ALT
  alt_ft_filt += ALT_ALPHA * (alt_ft_meas - alt_ft_filt);
#endif
#if ENABLE_BARO
  baro_inhg_filt += BARO_ALPHA * (baro_inhg_meas - baro_inhg_filt);
#endif
#if ENABLE_DG
  hdg_deg_filt = wrap360(hdg_deg_filt +
                         DG_ALPHA * shortestAngleDelta(hdg_deg_meas, hdg_deg_filt));
  bug_deg_filt = wrap360(bug_deg_filt +
                         BUG_ALPHA * shortestAngleDelta(bug_deg_meas, bug_deg_filt));
#endif
#if ENABLE_VSI
  vsi_fpm_filt += VSI_ALPHA * (vsi_fpm_meas - vsi_fpm_filt);
#endif
}

void serviceControl(unsigned long now) {
  if ((unsigned long)(now - lastControlMs) < CONTROL_INTERVAL_MS) return;

  // Avoid accumulating drift if a long operation happened.
  lastControlMs = now;
  const bool idle = (unsigned long)(now - lastRxMs) > IDLE_TIMEOUT_MS;
  const bool parkMode = explicitParkRequested || (idle && PARK_ON_IDLE);

  // A clean record must be invalidated BEFORE the first post-park movement.
  if (!parkMode && haveSimulatorData && parkedCleanPersisted) {
    invalidateCleanRecordBeforeMotion();
  }

  // After restoring a position, hold it until real simulator data arrives.
  // This prevents default zero-valued measurements from undoing the restore.
  if (!parkMode && !haveSimulatorData) {
    digitalWrite(HEARTBEAT_LED_PIN, LOW);
    return;
  }

  if (!parkMode) {
    updateFilters();
  }
  updateTargets(parkMode);
  digitalWrite(HEARTBEAT_LED_PIN, parkMode ? LOW : HIGH);
}

void runSteppers() {
  stepASI.run();
#if ENABLE_TURN_SLIP
  stepTURN.run();
  stepSLIP.run();
#endif
#if ENABLE_ALT
  stepALT.run();
#endif
#if ENABLE_BARO
  stepBARO.run();
#endif
#if ENABLE_DG
  stepDG.run();
  stepBUG.run();
#endif
#if ENABLE_VSI
  stepVSI.run();
#endif
}

// ============================================================
// Gauge adjustment
// ============================================================
void adjustSelectedGauge(int direction) {
  switch (selectedGauge) {
    case GAUGE_ASI:
      TRIM_IAS += direction * ADJUSTMENT_STEPS;
      Serial.print(F("ASI trim adjusted: "));
      Serial.println(TRIM_IAS);
      break;

#if ENABLE_TURN_SLIP
    case GAUGE_TURN:
      TRIM_TURN += direction * ADJUSTMENT_STEPS;
      Serial.print(F("TURN trim adjusted: "));
      Serial.println(TRIM_TURN);
      break;

    case GAUGE_SLIP:
      TRIM_SLIP += direction * ADJUSTMENT_STEPS;
      Serial.print(F("SLIP trim adjusted: "));
      Serial.println(TRIM_SLIP);
      break;
#endif

#if ENABLE_ALT
    case GAUGE_ALT:
      TRIM_ALT += direction * ADJUSTMENT_STEPS;
      Serial.print(F("ALT trim adjusted: "));
      Serial.println(TRIM_ALT);
      break;
#endif

#if ENABLE_BARO
    case GAUGE_BARO:
      TRIM_BARO += direction * ADJUSTMENT_STEPS;
      Serial.print(F("BARO trim adjusted: "));
      Serial.println(TRIM_BARO);
      break;
#endif

#if ENABLE_DG
    case GAUGE_DG:
      TRIM_DG += direction * ADJUSTMENT_STEPS;
      Serial.print(F("DG trim adjusted: "));
      Serial.println(TRIM_DG);
      break;

    case GAUGE_BUG:
      TRIM_BUG += direction * ADJUSTMENT_STEPS;
      Serial.print(F("BUG trim adjusted: "));
      Serial.println(TRIM_BUG);
      break;
#endif

#if ENABLE_VSI
    case GAUGE_VSI:
      TRIM_VSI += direction * ADJUSTMENT_STEPS;
      Serial.print(F("VSI trim adjusted: "));
      Serial.println(TRIM_VSI);
      break;
#endif

    default:
      return;
  }

  markCalibrationDirty();

  const unsigned long now = millis();
  const bool idle = (unsigned long)(now - lastRxMs) > IDLE_TIMEOUT_MS;
  const bool parkMode = explicitParkRequested || (idle && PARK_ON_IDLE);

  if (!parkMode && parkedCleanPersisted) {
    invalidateCleanRecordBeforeMotion();
  }

  // With simulator data available, recompute the normal absolute target.
  // Before the first simulator packet, preserve the old manual-calibration
  // behavior by nudging only the selected motor by the trim increment.
  if (haveSimulatorData || parkMode) {
    updateTargets(parkMode);
  } else {
    const long delta = direction * ADJUSTMENT_STEPS;
    switch (selectedGauge) {
      case GAUGE_ASI:
        stepASI.moveTo(stepASI.currentPosition() + delta);
        break;
#if ENABLE_TURN_SLIP
      case GAUGE_TURN:
        stepTURN.moveTo(stepTURN.currentPosition() + delta);
        break;
      case GAUGE_SLIP:
        stepSLIP.moveTo(stepSLIP.currentPosition() + delta);
        break;
#endif
#if ENABLE_ALT
      case GAUGE_ALT:
        stepALT.moveTo(stepALT.currentPosition() + delta);
        break;
#endif
#if ENABLE_BARO
      case GAUGE_BARO:
        stepBARO.moveTo(stepBARO.currentPosition() + delta);
        break;
#endif
#if ENABLE_DG
      case GAUGE_DG:
        stepDG.moveTo(stepDG.currentPosition() + delta);
        break;
      case GAUGE_BUG:
        stepBUG.moveTo(stepBUG.currentPosition() + delta);
        break;
#endif
#if ENABLE_VSI
      case GAUGE_VSI:
        stepVSI.moveTo(stepVSI.currentPosition() + delta);
        break;
#endif
      default:
        break;
    }
  }
  tftUpdate();
}

// ============================================================
// Buttons
// ============================================================
void printButtonStates(unsigned long now) {
#if ENABLE_BUTTON_STATE_DEBUG
  static unsigned long lastDebugTime = 0;
  if ((unsigned long)(now - lastDebugTime) < 1000) return;
  lastDebugTime = now;

  // Short enough to minimize serial blocking while preserving diagnostics.
  Serial.print(F("BTN SEL:"));
  Serial.print(buttonSel.stablePressed ? 1 : 0);
  Serial.print(F(" CW:"));
  Serial.print(buttonCw.stablePressed ? 1 : 0);
  Serial.print(F(" CCW:"));
  Serial.print(buttonCcw.stablePressed ? 1 : 0);
  Serial.print(F(" G:"));
  Serial.println(gaugeNames[selectedGauge]);
#else
  (void)now;
#endif
}

void handleButtons(unsigned long now) {
  buttonSel.update(now);
  buttonCw.update(now);
  buttonCcw.update(now);

  printButtonStates(now);

  if (buttonCw.pressedEdge()) {
    Serial.println(F("CW button pressed"));
    adjustSelectedGauge(+1);
    btnCwHoldStart = now;
    lastCwMove = now;
    btnCwHolding = false;
  } else if (buttonCw.stablePressed) {
    if (!btnCwHolding &&
        (unsigned long)(now - btnCwHoldStart) >= BUTTON_HOLD_DELAY) {
      btnCwHolding = true;
      lastCwMove = now;
      Serial.println(F("CW continuous movement started"));
    }
    if (btnCwHolding &&
        (unsigned long)(now - lastCwMove) >= CONTINUOUS_MOVE_INTERVAL) {
      adjustSelectedGauge(+1);
      lastCwMove = now;
    }
  }

  if (buttonCw.releasedEdge()) {
    btnCwHolding = false;
  }

  if (buttonCcw.pressedEdge()) {
    Serial.println(F("CCW button pressed"));
    adjustSelectedGauge(-1);
    btnCcwHoldStart = now;
    lastCcwMove = now;
    btnCcwHolding = false;
  } else if (buttonCcw.stablePressed) {
    if (!btnCcwHolding &&
        (unsigned long)(now - btnCcwHoldStart) >= BUTTON_HOLD_DELAY) {
      btnCcwHolding = true;
      lastCcwMove = now;
      Serial.println(F("CCW continuous movement started"));
    }
    if (btnCcwHolding &&
        (unsigned long)(now - lastCcwMove) >= CONTINUOUS_MOVE_INTERVAL) {
      adjustSelectedGauge(-1);
      lastCcwMove = now;
    }
  }

  if (buttonCcw.releasedEdge()) {
    btnCcwHolding = false;
  }

  if (buttonSel.pressedEdge()) {
    selectedGauge = nextEnabledGauge(selectedGauge);
    Serial.print(F("Selected gauge: "));
    Serial.println(gaugeNames[selectedGauge]);
    tftUpdate();
  }
}

void startButtonTest() {
  buttonTestActive = true;
  buttonTestStartedMs = millis();
  buttonTestLastReportMs = 0;
  Serial.println(F("=== Button Test Mode ==="));
  Serial.println(F("Press buttons to see if they're detected..."));
}

void serviceButtonTest(unsigned long now) {
  if (!buttonTestActive) return;

  if ((unsigned long)(now - buttonTestStartedMs) >= BUTTON_TEST_DURATION_MS) {
    buttonTestActive = false;
    // Reinitialize debounce state so leaving test mode cannot create fake edges.
    buttonSel.begin(BTN_SEL);
    buttonCw.begin(BTN_CW);
    buttonCcw.begin(BTN_CCW);
    Serial.println(F("Button test complete"));
    return;
  }

  if ((unsigned long)(now - buttonTestLastReportMs) < BUTTON_TEST_REPORT_MS) return;
  buttonTestLastReportMs = now;

  const bool sel = !digitalRead(BTN_SEL);
  const bool cw = !digitalRead(BTN_CW);
  const bool ccw = !digitalRead(BTN_CCW);

  if (sel || cw || ccw) {
    Serial.print(F("Button pressed - SEL:"));
    Serial.print(sel ? '1' : '0');
    Serial.print(F(" CW:"));
    Serial.print(cw ? '1' : '0');
    Serial.print(F(" CCW:"));
    Serial.println(ccw ? '1' : '0');
  }
}

// ============================================================
// Serial parser
// ============================================================
bool startsWith(const char* text, const char* prefix) {
  return strncmp(text, prefix, strlen(prefix)) == 0;
}

void parseLine(char* line) {
  // Preserve one-shot commands, but without Arduino String allocations.
  if (strcmp(line, "PARK") == 0 || strcmp(line, "SHUTDOWN") == 0) {
    explicitParkRequested = true;
    baroManualOverride = false;
    parkSettling = false;
    lastRxMs = millis();
    updateTargets(true);
    Serial.println(F("Parking requested. Wait for: PARKED... Safe to power off."));
    return;
  }

  if (strcmp(line, "RESUME") == 0) {
    explicitParkRequested = false;
    parkSettling = false;
    lastRxMs = millis();
    Serial.println(F("Park mode released."));
    return;
  }

  if (strcmp(line, "POS:STATUS") == 0) {
    printPositionStatus();
    lastRxMs = millis();
    return;
  }

  if (strcmp(line, "POS:TRUST") == 0) {
    if (!allSteppersAtTarget()) {
      Serial.println(F("POS:TRUST rejected: wait until all gauges stop moving."));
    } else {
      positionEstimateExact = true;
      if (savePositionRecord(true)) {
        parkedCleanPersisted = true;
        Serial.println(F("Current physical positions marked TRUSTED/EXACT."));
      }
    }
    lastRxMs = millis();
    return;
  }

  if (strcmp(line, "POS:SAVE") == 0) {
    if (!allSteppersAtTarget()) {
      Serial.println(F("POS:SAVE not written: gauges are still moving."));
    } else if (savePositionRecord(true)) {
      parkedCleanPersisted = true;
      Serial.println(F("Motor-position CLEAN snapshot saved manually."));
    }
    lastRxMs = millis();
    return;
  }

  if (strstr(line, "CAL:ZERO") != nullptr) {
    assumeHome();
    // Rebasing software coordinates is not physical position feedback. Keep the
    // confidence conservative until the user visually verifies with POS:TRUST.
    positionEstimateExact = false;
    baroManualOverride = false;
    lastRxMs = millis();
    updateTargets(false);
    const bool cleanNow = allSteppersAtTarget();
    if (savePositionRecord(cleanNow)) {
      parkedCleanPersisted = cleanNow;
    }
    Serial.println(F("All current software positions set as zero. Use POS:TRUST after visual verification."));
    return;
  }

  if (strstr(line, "CAL:SLIPCENTER") != nullptr) {
#if ENABLE_TURN_SLIP
    TRIM_SLIP = stepSLIP.currentPosition();
    markCalibrationDirty();
    tftUpdate();
    Serial.print(F("SLIP center set. Trim: "));
    Serial.println(TRIM_SLIP);
#else
    Serial.println(F("CAL:SLIPCENTER ignored: TURN/SLIP disabled."));
#endif
    lastRxMs = millis();
    return;
  }

  if (strstr(line, "TEST:BUTTONS") != nullptr) {
    startButtonTest();
    lastRxMs = millis();
    return;
  }

  if (strstr(line, "CAL:SAVE") != nullptr) {
    saveCalibrationNow();
    lastRxMs = millis();
    return;
  }

  if (strstr(line, "CAL:LOAD") != nullptr) {
    loadCalibration();
    tftUpdate(true);
    lastRxMs = millis();
    return;
  }

  bool gotGaugeData = false;
  char* savePtr = nullptr;
  for (char* tok = strtok_r(line, " ", &savePtr);
       tok != nullptr;
       tok = strtok_r(nullptr, " ", &savePtr)) {

    if (startsWith(tok, "IAS:")) {
      ias_knots_meas = strtof(tok + 4, nullptr);
      gotGaugeData = true;
    }
#if ENABLE_TURN_SLIP
    else if (startsWith(tok, "T:")) {
      turn_deg_meas = strtof(tok + 2, nullptr);
      gotGaugeData = true;
    }
    else if (startsWith(tok, "S:")) {
      slip_deg_meas = strtof(tok + 2, nullptr);
      gotGaugeData = true;
    }
#endif
#if ENABLE_ALT
    else if (startsWith(tok, "ALT:")) {
      alt_ft_meas = strtof(tok + 4, nullptr);
      gotGaugeData = true;
    }
#endif
#if ENABLE_BARO
    else if (startsWith(tok, "BARO:")) {
      baro_inhg_meas = strtof(tok + 5, nullptr);
      baroManualOverride = false;
      gotGaugeData = true;
    }
#endif
#if ENABLE_DG
    else if (startsWith(tok, "HDG:")) {
      hdg_deg_meas = wrap360(strtof(tok + 4, nullptr));
      gotGaugeData = true;
    }
    else if (startsWith(tok, "BUG:")) {
      bug_deg_meas = wrap360(strtof(tok + 4, nullptr));
      gotGaugeData = true;
    }
#endif
#if ENABLE_VSI
    else if (startsWith(tok, "VSI:") || startsWith(tok, "VVI:")) {
      vsi_fpm_meas = strtof(tok + 4, nullptr);
      gotGaugeData = true;
    }
#endif

#if ENABLE_BARO
    // BARO debug commands remain supported and now persist until a BARO: token.
    else if (startsWith(tok, "BAROSTEP:")) {
      const long steps = strtol(tok + 9, nullptr, 10);
      baroManualTarget = stepBARO.currentPosition() + steps;
      baroManualOverride = true;
      stepBARO.moveTo(baroManualTarget);
    }
    else if (startsWith(tok, "BAROSET:")) {
      const float hpa = strtof(tok + 8, nullptr);
      const float inhg = hpa / 33.863886f;
      baroManualTarget = baroStepsFromInHg(inhg);
      baroManualOverride = true;
      stepBARO.moveTo(baroManualTarget);
    }
#endif
  }

  if (gotGaugeData) {
    haveSimulatorData = true;
  }
  lastRxMs = millis();
}

void readSerialTokens() {
  uint8_t processed = 0;

  while (Serial.available() && processed < MAX_SERIAL_BYTES_PER_LOOP) {
    ++processed;
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxOverflow) {
        Serial.println(F("RX line too long; discarded."));
        rxOverflow = false;
        rxPos = 0;
        continue;
      }

      if (rxPos > 0) {
        rxBuffer[rxPos] = '\0';
        parseLine(rxBuffer);
        rxPos = 0;
      }
      continue;
    }

    if (rxOverflow) continue;

    if (rxPos < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxPos++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  buttonSel.begin(BTN_SEL);
  buttonCw.begin(BTN_CW);
  buttonCcw.begin(BTN_CCW);

  stepASI.setMaxSpeed(MAX_SPEED_STEPS);
  stepASI.setAcceleration(ACCEL_STEPS);
#if ENABLE_TURN_SLIP
  stepTURN.setMaxSpeed(MAX_SPEED_STEPS);
  stepTURN.setAcceleration(ACCEL_STEPS);
  stepSLIP.setMaxSpeed(MAX_SPEED_STEPS);
  stepSLIP.setAcceleration(ACCEL_STEPS);
#endif
#if ENABLE_ALT
  stepALT.setMaxSpeed(MAX_SPEED_STEPS);
  stepALT.setAcceleration(ACCEL_STEPS);
#endif
#if ENABLE_BARO
  stepBARO.setMaxSpeed(BARO_MAX_SPEED);
  stepBARO.setAcceleration(BARO_ACCEL);
#endif
#if ENABLE_DG
  stepDG.setMaxSpeed(MAX_SPEED_STEPS);
  stepDG.setAcceleration(ACCEL_STEPS);
  stepBUG.setMaxSpeed(MAX_SPEED_STEPS);
  stepBUG.setAcceleration(ACCEL_STEPS);
#endif
#if ENABLE_VSI
  stepVSI.setMaxSpeed(MAX_SPEED_STEPS);
  stepVSI.setAcceleration(ACCEL_STEPS);
#endif

  loadCalibration();

  // ----- restore software-tracked motor positions first -----
  const bool restoredPosition = loadLatestPositionRecord();

  if (!restoredPosition) {
    // No software position exists yet. Preserve the original fallback behavior.
#if ASSUME_HOME_ON_BOOT
    assumeHome();
    // This is an assumption, not a physical measurement. CAL:ZERO explicitly
    // establishes a trusted absolute reference when you know the needles are set.
    positionEstimateExact = false;
#else
    softHome(stepASI, HOME_DIR_ASI, HOME_REVS_ASI,
             IAS_STEPS_PER_REV, MAX_SPEED_STEPS, ACCEL_STEPS);
#if ENABLE_TURN_SLIP
    softHome(stepTURN, HOME_DIR_TURN, HOME_REVS_TURN,
             TURN_STEPS_PER_REV, MAX_SPEED_STEPS, ACCEL_STEPS);
    softHome(stepSLIP, HOME_DIR_SLIP, HOME_REVS_SLIP,
             SLIP_STEPS_PER_REV, MAX_SPEED_STEPS, ACCEL_STEPS);
#endif
#if ENABLE_ALT
    softHome(stepALT, HOME_DIR_ALT, HOME_REVS_ALT,
             ALT_STEPS_PER_REV, MAX_SPEED_STEPS, ACCEL_STEPS);
#endif
#if ENABLE_BARO
    softHome(stepBARO, HOME_DIR_BARO, HOME_REVS_BARO,
             NOMINAL_STEPS_PER_REV, BARO_MAX_SPEED, BARO_ACCEL);
#endif
    // DG/BUG are continuous-rotation axes and remain software-estimated.
    positionEstimateExact = false;
#endif
    savePositionRecord(false);
    parkedCleanPersisted = false;
  }

  // Preserve the startup ASI heartbeat/test relative to the restored coordinate.
  const long asiStartupPosition = stepASI.currentPosition();
  const bool startupRecordWasClean = parkedCleanPersisted;
  if (startupRecordWasClean) {
    invalidateCleanRecordBeforeMotion();
  }
  stepASI.moveTo(asiStartupPosition + 120);
  stepASI.runToPosition();
  stepASI.moveTo(asiStartupPosition);
  stepASI.runToPosition();
  if (startupRecordWasClean) {
    if (savePositionRecord(true)) {
      parkedCleanPersisted = true;
    }
  }

  tftInitSmall();

  // Initialize the fixed-rate scheduler from now.
  lastControlMs = millis();

  Serial.println(F("Ready. Send: IAS:<kts> T:<deg/s> S:<deg> ALT:<ft> BARO:<inHg> HDG:<deg> BUG:<deg> VSI:<fpm>"));
  Serial.println(F("Buttons: Pin 13=Select gauge, Pin 11=CW adjust, Pin 12=CCW adjust"));
  Serial.println(F("Commands: CAL:ZERO, CAL:SLIPCENTER, CAL:SAVE, CAL:LOAD, TEST:BUTTONS"));
  Serial.println(F("Position: PARK/SHUTDOWN, RESUME, POS:SAVE, POS:TRUST, POS:STATUS"));
  Serial.println(F("BARO debug: BAROSTEP:<steps>, BAROSET:<hPa>"));
  printPositionStatus();
  Serial.print(F("Selected gauge: "));
  Serial.println(gaugeNames[selectedGauge]);
}

void loop() {
  const unsigned long now = millis();

  // Keep each service short. runSteppers() is called every pass through loop().
  readSerialTokens();

  if (buttonTestActive) {
    serviceButtonTest(now);
  } else {
    // Calibration controls now remain available even while simulator data flows.
    handleButtons(now);
  }

  serviceControl(now);

  // Any command or target change that would move a motor invalidates a clean
  // snapshot before the first step pulse is emitted.
  if (parkedCleanPersisted && !allSteppersAtTarget()) {
    invalidateCleanRecordBeforeMotion();
  }

  runSteppers();

  const bool idle = (unsigned long)(now - lastRxMs) > IDLE_TIMEOUT_MS;
  const bool parkMode = explicitParkRequested || (idle && PARK_ON_IDLE);
  servicePositionPersistence(now, parkMode);
  serviceCalibrationSave(now);
}
