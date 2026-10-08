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
bool present();

}  // namespace display
