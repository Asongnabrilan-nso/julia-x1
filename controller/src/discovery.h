#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// Finds julia-X2 robots on the local network without any configuration:
// probes every address in the controller's /24 subnet for the dashboard port,
// then confirms it is a julia-X2 by its page title, and reads the robot's
// MAC address from the ARP table (the MAC is the robot's stable identity;
// its IP can change when the router hands out a new lease).
//
// Blocking (several seconds) — call only from the robot-link task.
namespace discovery {

struct Robot {
  IPAddress ip;
  uint8_t mac[6];
  bool hasMac;
};

constexpr uint8_t kMaxRobots = 6;

// Returns how many robots were found (up to kMaxRobots).
uint8_t scan(Robot* out, uint8_t maxOut);

// Confirms a single host is a julia-X2 dashboard; fills `mac` if known.
bool probe(IPAddress ip, uint8_t mac[6], bool* hasMac);

// MAC of a host we've recently talked to (from the ARP table).
bool macOf(IPAddress ip, uint8_t mac[6]);

String macToString(const uint8_t mac[6]);

}  // namespace discovery
