#pragma once
#include <Arduino.h>

// Raw ADC calibration points for each physical axis (VRx / VRy pins).
struct JoyCalibration {
  uint16_t minX, centerX, maxX;
  uint16_t minY, centerY, maxY;
};

enum class JoyDir : uint8_t { Stop, Forward, Backward, Left, Right };
enum class ButtonEvent : uint8_t { None, Down, Click, LongPress };  // Down = press edge

struct JoyState {
  uint16_t rawX = 0, rawY = 0;  // filtered ADC counts (0..4095), physical pins
  int16_t x = 0, y = 0;         // calibrated -1000..1000, +x right, +y forward
  bool pressed = false;         // debounced button level
  ButtonEvent event = ButtonEvent::None;  // edge, valid for one update()
  JoyDir dir = JoyDir::Stop;
};

const char* joyDirName(JoyDir d);

class Joystick {
 public:
  void begin();
  // Samples at JOY_SAMPLE_PERIOD_MS; returns true when a new sample was taken.
  bool update();
  const JoyState& state() const { return s_; }

  // Measure the rest position. Blocking (~150 ms). Stick must be released.
  // Returns false (and keeps the previous center) if the stick looks held.
  bool calibrateCenter();

  // Range calibration: start, sweep the stick around its full edge a few
  // times, then finish. finish() validates, applies and saves to NVS.
  void startRangeCalibration();
  bool finishRangeCalibration();
  void cancelRangeCalibration();
  bool rangeCalibrating() const { return rangeCal_; }

  void resetCalibration();  // back to defaults and erase NVS
  bool hasStoredRange() const { return storedRange_; }
  const JoyCalibration& calibration() const { return cal_; }
  void printCalibration(Print& out) const;

 private:
  uint16_t readAxis(uint8_t pin) const;
  void loadCalibration();
  void saveCalibration();
  void updateButton(uint32_t now);
  static int16_t normalize(uint16_t raw, uint16_t mn, uint16_t ctr, uint16_t mx);

  JoyState s_;
  JoyCalibration cal_{};
  JoyCalibration sweep_{};  // min/max captured during range calibration
  bool rangeCal_ = false;
  bool storedRange_ = false;
  float emaX_ = 2048, emaY_ = 2048;
  uint32_t lastSampleMs_ = 0;

  // button debounce
  bool btnRaw_ = false;
  uint32_t btnChangedMs_ = 0;
  uint32_t btnDownMs_ = 0;
  bool longFired_ = false;
};
