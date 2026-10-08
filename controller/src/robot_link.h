#pragma once
#include <Arduino.h>
#include <IPAddress.h>

#include "discovery.h"

// Link to the robot's web dashboard server (Socket.IO v4 on the UNO Q, port 7000).
// The controller joins as one more dashboard client: it sends the same
// "joystick" {x, y} / "stop" messages as the web joystick and listens to the
// same telemetry broadcasts. Runs in its own FreeRTOS task, because TCP
// connects and network scans block for seconds.
//
// Which robot: the PAIRED robot (stored in flash as MAC + last IP). If its IP
// changed, the link rescans the network and finds it again by MAC. Pairing is
// started from the controller's menu (or automatically when nothing is paired).
// A non-empty robot_host in platformio.ini overrides all of this (developer use).
namespace robot {

// Robot state codes, mirroring enum State / Mode in sketch/sketch.ino.
enum : int8_t { ST_IDLE = 0, ST_ARMED = 1, ST_BALANCING = 2, ST_FAULT = 3 };
enum : int8_t { MODE_MANUAL = 0, MODE_BALANCE = 1 };
// ToF status codes, mirroring TOF_STATUS_TEXT in python/main.py.
enum : int8_t { TOF_MISSING = 0, TOF_OK = 1, TOF_NO_TARGET = 2, TOF_TIMEOUT = 3 };

enum class LinkState : uint8_t {
  NoWifi,       // waiting for WiFi
  Unpaired,     // no robot paired yet
  Connecting,   // connecting to the paired robot
  Searching,    // paired robot not answering at its last IP: rescanning by MAC
  Connected,    // Socket.IO up
  PairScanning, // pairing: scanning the network
  PairChoose,   // pairing: results ready, waiting for the user to pick
};

struct Telemetry {
  LinkState link = LinkState::NoWifi;
  bool socket = false;      // Socket.IO connected
  bool fresh = false;       // "balance" telemetry seen within LINK_STALE_MS
  int8_t mode = -1;         // MODE_*, -1 unknown
  int8_t state = -1;        // ST_*, -1 unknown
  uint8_t fault = 0;
  int16_t tofMm = -1;       // -1 = nothing in range
  int8_t tofStatus = -1;    // TOF_*, -1 unknown
  uint8_t tofFactor = 100;  // % forward speed allowed by obstacle avoidance
  float speedScale = 1.0f;  // dashboard max-speed slider
  int16_t rttMs = -1;       // last command round trip, -1 unknown
  int16_t rttMaxMs = -1;    // worst round trip over the last few seconds
  char action[24] = "";     // e.g. "Driving forward", "Balancing", "FAULT"
  char calibration[12] = "";// IMU: idle / calibrating / calibrated / failed
};

void begin();                     // starts the link task
void setStick(float x, float y);  // -1..1, +x right, +y forward; 0,0 = stop
Telemetry snapshot();             // thread-safe copy for the display
bool connected();                 // Socket.IO connected
bool commanding();                // currently sending a non-zero command
void printStatus(Print& out);

// Pairing ---------------------------------------------------------------
bool isPaired();
IPAddress pairedIp();
void startPairing();                       // scan; results via pairResults()
uint8_t pairResults(discovery::Robot* out, uint8_t maxOut);  // valid in PairChoose
void choosePaired(const discovery::Robot& r);  // store + connect
void cancelPairing();                      // back to the paired robot (if any)
void forget();                             // erase pairing

}  // namespace robot
