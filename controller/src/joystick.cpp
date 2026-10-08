#include "joystick.h"

#include <Preferences.h>

#include "config.h"

namespace {

constexpr char kNvsNamespace[] = "joycal";
constexpr char kNvsKey[] = "cal";
constexpr uint8_t kCalVersion = 1;

struct StoredCal {
  uint8_t version;
  JoyCalibration cal;
};

constexpr JoyCalibration kDefaultCal = {0, 2048, 4095, 0, 2048, 4095};

}  // namespace

const char* joyDirName(JoyDir d) {
  switch (d) {
    case JoyDir::Forward:  return "FORWARD";
    case JoyDir::Backward: return "BACKWARD";
    case JoyDir::Left:     return "LEFT";
    case JoyDir::Right:    return "RIGHT";
    default:               return "STOP";
  }
}

void Joystick::begin() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_JOY_X, ADC_11db);  // ~0..2.9 V usable on the C3
  analogSetPinAttenuation(PIN_JOY_Y, ADC_11db);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  loadCalibration();

  emaX_ = readAxis(PIN_JOY_X);
  emaY_ = readAxis(PIN_JOY_Y);

  // Center drifts a little between power-ups, so always re-measure it.
  if (!calibrateCenter()) {
    Serial.println("[JOY] stick not at rest during boot - using stored center");
  }
}

uint16_t Joystick::readAxis(uint8_t pin) const {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < JOY_OVERSAMPLE; i++) sum += analogRead(pin);
  return sum / JOY_OVERSAMPLE;
}

int16_t Joystick::normalize(uint16_t raw, uint16_t mn, uint16_t ctr, uint16_t mx) {
  // Each half of the axis is scaled on its own: pots are rarely symmetric and
  // the C3 ADC compresses/saturates near the top of its range.
  int32_t lo = mn + JOY_EDGE_MARGIN;
  int32_t hi = mx - JOY_EDGE_MARGIN;
  int32_t v;
  if (raw >= ctr) {
    int32_t span = max<int32_t>(hi - ctr, 1);
    v = (int32_t)(raw - ctr) * 1000 / span;
  } else {
    int32_t span = max<int32_t>(ctr - lo, 1);
    v = -(int32_t)(ctr - raw) * 1000 / span;
  }
  v = constrain(v, -1000, 1000);

  // Deadzone with rescale, so output ramps smoothly from 0 at its edge.
  int32_t mag = abs(v);
  if (mag <= JOY_DEADZONE) return 0;
  mag = (mag - JOY_DEADZONE) * 1000 / (1000 - JOY_DEADZONE);
  return v < 0 ? -mag : mag;
}

bool Joystick::update() {
  uint32_t now = millis();
  updateButton(now);
  if (now - lastSampleMs_ < JOY_SAMPLE_PERIOD_MS) return false;
  lastSampleMs_ = now;

  emaX_ += JOY_EMA_ALPHA * (readAxis(PIN_JOY_X) - emaX_);
  emaY_ += JOY_EMA_ALPHA * (readAxis(PIN_JOY_Y) - emaY_);
  s_.rawX = (uint16_t)(emaX_ + 0.5f);
  s_.rawY = (uint16_t)(emaY_ + 0.5f);

  if (rangeCal_) {
    sweep_.minX = min(sweep_.minX, s_.rawX);
    sweep_.maxX = max(sweep_.maxX, s_.rawX);
    sweep_.minY = min(sweep_.minY, s_.rawY);
    sweep_.maxY = max(sweep_.maxY, s_.rawY);
  }

  int16_t px = normalize(s_.rawX, cal_.minX, cal_.centerX, cal_.maxX);
  int16_t py = normalize(s_.rawY, cal_.minY, cal_.centerY, cal_.maxY);
  if (JOY_SWAP_XY) std::swap(px, py);
  if (JOY_INVERT_X) px = -px;
  if (JOY_INVERT_Y) py = -py;
  s_.x = px;
  s_.y = py;

  if (abs(s_.x) < JOY_DIR_THRESHOLD && abs(s_.y) < JOY_DIR_THRESHOLD) {
    s_.dir = JoyDir::Stop;
  } else if (abs(s_.y) >= abs(s_.x)) {
    s_.dir = s_.y > 0 ? JoyDir::Forward : JoyDir::Backward;
  } else {
    s_.dir = s_.x > 0 ? JoyDir::Right : JoyDir::Left;
  }
  return true;
}

void Joystick::updateButton(uint32_t now) {
  s_.event = ButtonEvent::None;

  bool raw = digitalRead(PIN_JOY_SW) == LOW;
  if (raw != btnRaw_) {
    btnRaw_ = raw;
    btnChangedMs_ = now;
  }
  if (now - btnChangedMs_ >= BTN_DEBOUNCE_MS && raw != s_.pressed) {
    s_.pressed = raw;
    if (raw) {
      btnDownMs_ = now;
      longFired_ = false;
    } else if (!longFired_) {
      s_.event = ButtonEvent::Click;
    }
  }
  if (s_.pressed && !longFired_ && now - btnDownMs_ >= BTN_LONG_PRESS_MS) {
    longFired_ = true;
    s_.event = ButtonEvent::LongPress;
  }
}

bool Joystick::calibrateCenter() {
  uint16_t mnX = 4095, mxX = 0, mnY = 4095, mxY = 0;
  uint32_t sumX = 0, sumY = 0;
  constexpr int kSamples = 48;
  for (int i = 0; i < kSamples; i++) {
    uint16_t x = readAxis(PIN_JOY_X);
    uint16_t y = readAxis(PIN_JOY_Y);
    sumX += x;
    sumY += y;
    mnX = min(mnX, x); mxX = max(mxX, x);
    mnY = min(mnY, y); mxY = max(mxY, y);
    delay(3);
  }
  uint16_t cx = sumX / kSamples;
  uint16_t cy = sumY / kSamples;

  bool steady = (mxX - mnX) <= JOY_CENTER_MAX_SPREAD && (mxY - mnY) <= JOY_CENTER_MAX_SPREAD;
  bool inWindow = cx >= JOY_CENTER_MIN && cx <= JOY_CENTER_MAX &&
                  cy >= JOY_CENTER_MIN && cy <= JOY_CENTER_MAX;
  // The center must also sit inside the stored range with room on both sides.
  bool inRange = cx > cal_.minX + JOY_MIN_HALF_SPAN / 2 && cx < cal_.maxX - JOY_MIN_HALF_SPAN / 2 &&
                 cy > cal_.minY + JOY_MIN_HALF_SPAN / 2 && cy < cal_.maxY - JOY_MIN_HALF_SPAN / 2;

  Serial.printf("[JOY] center sample X=%u (spread %u)  Y=%u (spread %u)\n",
                cx, mxX - mnX, cy, mxY - mnY);
  if (!steady || !inWindow || !inRange) {
    Serial.printf("[JOY] center rejected:%s%s%s\n", steady ? "" : " noisy/moving",
                  inWindow ? "" : " out of window", inRange ? "" : " outside stored range");
    return false;
  }
  cal_.centerX = cx;
  cal_.centerY = cy;
  emaX_ = cx;
  emaY_ = cy;
  return true;
}

void Joystick::startRangeCalibration() {
  sweep_.minX = sweep_.maxX = s_.rawX;
  sweep_.minY = sweep_.maxY = s_.rawY;
  rangeCal_ = true;
  Serial.println("[JOY] RANGE CAL: sweep the stick around its full edge 3-4 times,");
  Serial.println("[JOY]            release it, then click the button (or send 'c').");
}

bool Joystick::finishRangeCalibration() {
  if (!rangeCal_) return false;
  rangeCal_ = false;

  JoyCalibration c = cal_;
  c.minX = sweep_.minX; c.maxX = sweep_.maxX;
  c.minY = sweep_.minY; c.maxY = sweep_.maxY;

  Serial.printf("[JOY] sweep X %u..%u..%u  Y %u..%u..%u\n", c.minX, c.centerX, c.maxX,
                c.minY, c.centerY, c.maxY);

  // uint16_t operands promote to int, so a center outside the sweep goes negative.
  bool ok = c.centerX - c.minX >= JOY_MIN_HALF_SPAN && c.maxX - c.centerX >= JOY_MIN_HALF_SPAN &&
            c.centerY - c.minY >= JOY_MIN_HALF_SPAN && c.maxY - c.centerY >= JOY_MIN_HALF_SPAN;
  if (!ok) {
    Serial.printf("[JOY] range cal REJECTED - every direction must travel >= %u counts "
                  "from center. Try again, pushing fully to each edge.\n", JOY_MIN_HALF_SPAN);
    return false;
  }

  cal_ = c;
  saveCalibration();
  Serial.println("[JOY] range cal saved");
  return true;
}

void Joystick::cancelRangeCalibration() {
  if (rangeCal_) Serial.println("[JOY] range cal cancelled");
  rangeCal_ = false;
}

void Joystick::resetCalibration() {
  rangeCal_ = false;
  cal_ = kDefaultCal;
  storedRange_ = false;
  Preferences p;
  if (p.begin(kNvsNamespace, false)) {
    p.clear();
    p.end();
  }
  Serial.println("[JOY] calibration reset to defaults (NVS erased)");
  calibrateCenter();
}

void Joystick::loadCalibration() {
  cal_ = kDefaultCal;
  storedRange_ = false;
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) {
    Serial.println("[JOY] no stored calibration - using defaults, run range cal");
    return;
  }
  StoredCal sc{};
  if (p.getBytes(kNvsKey, &sc, sizeof(sc)) == sizeof(sc) && sc.version == kCalVersion) {
    cal_ = sc.cal;
    storedRange_ = true;
    Serial.println("[JOY] loaded calibration from NVS");
  } else {
    Serial.println("[JOY] no stored calibration - using defaults, run range cal");
  }
  p.end();
}

void Joystick::saveCalibration() {
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) {
    Serial.println("[JOY] NVS open failed - calibration not saved");
    return;
  }
  StoredCal sc{kCalVersion, cal_};
  p.putBytes(kNvsKey, &sc, sizeof(sc));
  p.end();
  storedRange_ = true;
}

void Joystick::printCalibration(Print& out) const {
  out.printf("calibration (%s)\n", storedRange_ ? "stored" : "DEFAULTS - run range cal");
  out.printf("  X  min %4u  center %4u  max %4u\n", cal_.minX, cal_.centerX, cal_.maxX);
  out.printf("  Y  min %4u  center %4u  max %4u\n", cal_.minY, cal_.centerY, cal_.maxY);
  out.printf("  deadzone %d/1000  edge margin %d  swapXY %d  invX %d  invY %d\n",
             JOY_DEADZONE, JOY_EDGE_MARGIN, JOY_SWAP_XY, JOY_INVERT_X, JOY_INVERT_Y);
}
