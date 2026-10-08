// Julia-x2 Joy — handheld WiFi controller for the julia-X2 robot.
//
// Two run modes, chosen at power-up:
//
//   DRIVE (default)  Joins the robot's dashboard server and drives the robot
//                    with the stick, in whichever mode the robot is in
//                    (manual or self-balancing; mode, arming and tuning stay on
//                    the web dashboard). OLED shows robot mode, ToF distance,
//                    robot direction and the command being sent.
//
//   TEST             Hold the stick button while powering up, or send 't'.
//                    The bench tools: serial value stream, raw/calibrated
//                    values on the OLED, range calibration. Never sends
//                    anything to the robot.
//
// Serial console (115200), single-key commands:
//   h help   i info   w WiFi scan   t toggle TEST/DRIVE mode
//   TEST mode only:  s stream on/off   p print cal   r recenter
//                    c start/finish range cal   x reset cal
//
// Button: beeps on every press. In TEST mode, long-press (1.5 s) starts range
// calibration and a click finishes it.

#include <Arduino.h>

#include "buzzer.h"
#include "config.h"
#include "display.h"
#include "joystick.h"
#include "ota.h"
#include "robot_link.h"

namespace {

enum class RunMode : uint8_t { Drive, Test };

Joystick joy;
RunMode runMode = RunMode::Drive;
uint32_t lastStatusMs = 0;

// ===========================================================================
// TEST MODE — bench tools from step 1 (joystick values + calibration).
// Only used when testing the controller; nothing is sent to the robot.
// ===========================================================================
bool streaming = true;
uint32_t lastStreamMs = 0;

void toggleRangeCal() {
  if (joy.rangeCalibrating()) {
    joy.finishRangeCalibration();
    joy.printCalibration(Serial);
  } else {
    joy.startRangeCalibration();
  }
}

void testHandleButton() {
  switch (joy.state().event) {
    case ButtonEvent::LongPress:
      if (!joy.rangeCalibrating()) joy.startRangeCalibration();
      break;
    case ButtonEvent::Click:
      if (joy.rangeCalibrating()) toggleRangeCal();
      break;
    default: break;
  }
}

void testStream() {
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

void testHandleKey(char c) {
  switch (c) {
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
    default: break;
  }
}

void runTestMode() {
  testHandleButton();
  testStream();
}

// ===========================================================================
// DRIVE MODE — stick -> robot
// ===========================================================================
// After every (re)connect, commands are held until the stick has been centered
// once, so a stick that is already deflected can't make the robot lurch.
bool driveArmed = false;
bool wasConnected = false;

void runDriveMode() {
  const JoyState& s = joy.state();
  bool connected = robot::connected();

  if (connected != wasConnected) {
    if (connected) buzzer::chirpUp();
    else buzzer::chirpDown();
    wasConnected = connected;
    driveArmed = false;
  }
  if (connected && !driveArmed && s.x == 0 && s.y == 0) driveArmed = true;

  if (driveArmed) robot::setStick(s.x / 1000.0f, s.y / 1000.0f);
  else robot::setStick(0, 0);
}

// ===========================================================================
// Shared
// ===========================================================================
void setRunMode(RunMode m) {
  runMode = m;
  robot::setStick(0, 0);  // leaving DRIVE always stops the robot
  driveArmed = false;
  if (joy.rangeCalibrating()) joy.cancelRangeCalibration();
  Serial.printf("=== %s MODE ===\n", m == RunMode::Test ? "TEST" : "DRIVE");
  if (m == RunMode::Test) buzzer::doubleBeep();
}

void printHelp() {
  Serial.println(
      "commands: h help | i info | w wifi scan | t toggle TEST/DRIVE mode\n"
      "TEST only: s stream | p print cal | r recenter | c start/finish range cal | x reset cal");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h': printHelp(); break;
      case 'i':
        ota::printStatus(Serial);
        robot::printStatus(Serial);
        break;
      case 'w': ota::scan(Serial); break;
      case 't': setRunMode(runMode == RunMode::Test ? RunMode::Drive : RunMode::Test); break;
      case '\r': case '\n': case ' ': break;
      default:
        if (runMode == RunMode::Test) {
          testHandleKey(c);
        } else if (strchr("spxrc", c)) {
          Serial.println("that command is TEST-mode only - send 't' first");
        }
        break;
    }
  }
}

void statusHeartbeat() {
  uint32_t now = millis();
  if (now - lastStatusMs < STATUS_PERIOD_MS) return;
  lastStatusMs = now;
  ota::printStatusLine(Serial);
  if (runMode == RunMode::Drive) robot::printStatus(Serial);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // don't stall when running on battery with no USB host
  delay(1500);               // give USB CDC time to enumerate so boot logs aren't lost

  Serial.printf("\n=== Julia-x2 Joy  fw %s ===\n", FW_VERSION);
  buzzer::begin();
  display::begin();
  joy.begin();
  joy.printCalibration(Serial);

  // Button held at power-up -> TEST mode.
  bool test = digitalRead(PIN_JOY_SW) == LOW;
  setRunMode(test ? RunMode::Test : RunMode::Drive);

  ota::begin();
  robot::begin();
  printHelp();
}

void loop() {
  ota::handle();
  if (ota::inProgress()) {
    robot::setStick(0, 0);
    return;  // ArduinoOTA owns the CPU during an upload
  }

  joy.update();
  buzzer::update();
  if (joy.state().event == ButtonEvent::Down) buzzer::beep();
  handleSerial();

  if (runMode == RunMode::Test) runTestMode();
  else runDriveMode();

  statusHeartbeat();
  display::update(runMode == RunMode::Test ? display::Screen::Test : display::Screen::Drive, joy,
                  runMode == RunMode::Drive && robot::connected() && !driveArmed);
}
