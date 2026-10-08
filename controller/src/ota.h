#pragma once
#include <Arduino.h>

// WiFi station + ArduinoOTA. Non-blocking: the controller keeps working
// (joystick, display) when WiFi is down, and reconnects automatically.
namespace ota {

void begin();
void handle();             // call every loop iteration
bool wifiConnected();
bool inProgress();         // true while a firmware upload is being received
void printStatus(Print& out);
void printStatusLine(Print& out);  // compact one-line heartbeat
void scan(Print& out);     // list visible networks, marking the configured SSID

// WiFi setup portal (phone-based network setup, no rebuild needed)
void startPortal();
void stopPortal();
bool portalActive();
void forgetWifi();          // erase the portal-saved network
String currentSsid();

}  // namespace ota
