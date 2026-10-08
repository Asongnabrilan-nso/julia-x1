// Julia-x2 Joy — handheld WiFi controller for the julia-X2 robot.
// Firmware v1, step 1: OTA updates + joystick reading and calibration.
//
// Serial console (115200), single-key commands:
//   h  help           i  WiFi/OTA/firmware info
//   s  toggle stream  p  print calibration
//   r  recenter       c  start / finish range calibration
//   x  reset calibration to defaults (erases NVS)
//
// Button: long-press (1.5 s) starts range calibration, click finishes it.

#include <Arduino.h>

#include "config.h"
#include "joystick.h"
#include "ota.h"

namespace {

Joystick joy;
bool streaming = true;
uint32_t lastStreamMs = 0;

void printHelp() {
  Serial.println(
      "commands: h help | i info | s stream on/off | p print cal | r recenter |\n"
      "          c start/finish range cal | x reset cal\n"
      "button:   long-press = start range cal, click = finish it");
}

void toggleRangeCal() {
  if (joy.rangeCalibrating()) {
    joy.finishRangeCalibration();
    joy.printCalibration(Serial);
  } else {
    joy.startRangeCalibration();
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h': printHelp(); break;
      case 'i': ota::printStatus(Serial); break;
      case 's':
        streaming = !streaming;
        Serial.printf("stream %s\n", streaming ? "on" : "off");
        break;
      case 'p': joy.printCalibration(Serial); break;
      case 'r':
        Serial.println("recentering - keep the stick released");
        joy.calibrateCenter();
        break;
      case 'c': toggleRangeCal(); break;
      case 'x': joy.resetCalibration(); break;
      default: break;  // ignore newlines etc.
    }
  }
}

void handleButton() {
  switch (joy.state().event) {
    case ButtonEvent::LongPress:
      if (!joy.rangeCalibrating()) joy.startRangeCalibration();
      break;
    case ButtonEvent::Click:
      if (joy.rangeCalibrating()) {
        toggleRangeCal();
      } else {
        Serial.println("[BTN] click");
      }
      break;
    default: break;
  }
}

void stream() {
  uint32_t now = millis();
  if (!streaming || now - lastStreamMs < STREAM_PERIOD_MS) return;
  lastStreamMs = now;
  const JoyState& s = joy.state();
  if (joy.rangeCalibrating()) {
    Serial.printf("[CAL] raw X=%4u Y=%4u\n", s.rawX, s.rawY);
  } else {
    Serial.printf("raw X=%4u Y=%4u | x=%5d y=%5d | btn=%d | %s\n", s.rawX, s.rawY, s.x,
                  s.y, s.pressed, joyDirName(s.dir));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when running on battery with no USB host
  delay(1500);               // give USB CDC time to enumerate so boot logs aren't lost

  Serial.printf("\n=== Julia-x2 Joy  fw %s ===\n", FW_VERSION);
  joy.begin();
  joy.printCalibration(Serial);
  ota::begin();
  printHelp();
}

void loop() {
  ota::handle();
  if (ota::inProgress()) return;  // ArduinoOTA owns the CPU during an upload

  joy.update();
  handleButton();
  handleSerial();
  stream();
}
