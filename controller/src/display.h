#pragma once
#include <Arduino.h>

#include "joystick.h"

// 128x64 I2C OLED. If no display answers on the bus the module disables
// itself and the rest of the firmware runs normally.
namespace display {

enum class Screen : uint8_t { Drive, Test };

bool begin();  // returns false if no OLED was found
// Rate-limited internally, call every loop. `waitingForCenter`: link is up but
// commands are held until the stick has been released once (drive screen).
void update(Screen screen, const Joystick& joy, bool waitingForCenter);
void showMessage(const char* line1, const char* line2 = nullptr);  // e.g. OTA

// Menu screens (drawn immediately; the caller rate-limits).
// List with a highlighted row; scrolls when there are more rows than fit.
void drawList(const char* title, const char* const* items, uint8_t n, uint8_t sel);
// Title bar + up to 4 text lines.
void drawPage(const char* title, const char* l1, const char* l2 = nullptr,
              const char* l3 = nullptr, const char* l4 = nullptr);
bool present();

}  // namespace display
