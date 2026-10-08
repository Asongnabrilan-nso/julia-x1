#pragma once
#include <Arduino.h>

#include "joystick.h"

// On-device menu: everything a user needs without a PC or the code.
// Open: hold the stick button for MENU_HOLD_MS in DRIVE mode.
// Navigate: push the stick up/down. Select: click the stick.
// The robot is always sent "stop" while the menu is open.
namespace menu {

enum class Action : uint8_t { None, EnterTestMode };

void open();         // main list
void openPairing();  // jump straight to the pairing screens (e.g. nothing paired)
bool active();
Action update(Joystick& joy);  // input + drawing; call every loop while active()

}  // namespace menu
