#pragma once
#include <Arduino.h>

// Link to the robot's web dashboard server (Socket.IO v4 on the UNO Q, port 7000).
// The controller joins as one more dashboard client: it sends the same
// "joystick" {x, y} / "stop" messages as the web joystick and listens to the
// same telemetry broadcasts. Runs in its own FreeRTOS task, because a TCP
// connect to an offline robot blocks for seconds.
namespace robot {

// Robot state codes, mirroring enum State / Mode in sketch/sketch.ino.
enum : int8_t { ST_IDLE = 0, ST_ARMED = 1, ST_BALANCING = 2, ST_FAULT = 3 };
enum : int8_t { MODE_MANUAL = 0, MODE_BALANCE = 1 };
// ToF status codes, mirroring TOF_STATUS_TEXT in python/main.py.
enum : int8_t { TOF_MISSING = 0, TOF_OK = 1, TOF_NO_TARGET = 2, TOF_TIMEOUT = 3 };

struct Telemetry {
  bool socket = false;      // Socket.IO connected
  bool fresh = false;       // "balance" telemetry seen within LINK_STALE_MS
  int8_t mode = -1;         // MODE_*, -1 unknown
  int8_t state = -1;        // ST_*, -1 unknown
  uint8_t fault = 0;
  int16_t tofMm = -1;       // -1 = nothing in range
  int8_t tofStatus = -1;    // TOF_*, -1 unknown
  uint8_t tofFactor = 100;  // % forward speed allowed by obstacle avoidance
  float speedScale = 1.0f;  // dashboard max-speed slider
  char action[24] = "";     // e.g. "Driving forward", "Balancing", "FAULT"
  char calibration[12] = "";// IMU: idle / calibrating / calibrated / failed
};

bool configured();                // false if robot_host is empty in platformio.ini
void begin();                     // starts the link task
void setStick(float x, float y);  // -1..1, +x right, +y forward; 0,0 = stop
Telemetry snapshot();             // thread-safe copy for the display
bool connected();                 // Socket.IO connected
bool commanding();                // currently sending a non-zero command
void printStatus(Print& out);

}  // namespace robot
