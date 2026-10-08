#include "menu.h"

#include <WiFi.h>

#include "buzzer.h"
#include "config.h"
#include "display.h"
#include "ota.h"
#include "robot_link.h"

namespace menu {
namespace {

enum class View : uint8_t {
  Closed,
  Main,
  CalRelease,  // "let go of the stick" countdown, then measure center
  CalSweep,    // sweep the edges, click to save
  CalResult,
  PairWait,    // scanning
  PairList,    // pick a robot
  Wifi,        // setup portal open
  Info,
};

View view = View::Closed;
uint8_t sel = 0;
uint32_t viewStartMs = 0;
uint32_t lastDrawMs = 0;
uint32_t lastNavMs = 0;
bool calOk = false;

discovery::Robot found[discovery::kMaxRobots];
uint8_t foundCount = 0;

const char* const kMain[] = {"Drive (exit menu)", "Calibrate stick", "Pair robot",
                             "WiFi setup", "Info", "Test mode"};
constexpr uint8_t kMainCount = sizeof(kMain) / sizeof(kMain[0]);

void go(View v) {
  view = v;
  sel = 0;
  viewStartMs = millis();
  lastDrawMs = 0;  // draw immediately
}

// Stick up/down moves the cursor (edge-triggered with auto-repeat).
void navigate(const JoyState& s, uint8_t count) {
  if (count == 0) return;
  uint32_t now = millis();
  int dir = s.y > MENU_NAV_THRESHOLD ? -1 : s.y < -MENU_NAV_THRESHOLD ? 1 : 0;
  if (dir == 0) {
    lastNavMs = 0;
    return;
  }
  if (lastNavMs && now - lastNavMs < MENU_NAV_REPEAT_MS) return;
  lastNavMs = now;
  sel = (sel + count + dir) % count;
  lastDrawMs = 0;
}

bool clicked(const JoyState& s) { return s.event == ButtonEvent::Click; }

bool drawDue() {
  uint32_t now = millis();
  if (lastDrawMs && now - lastDrawMs < OLED_PERIOD_MS) return false;
  lastDrawMs = now;
  return true;
}

// ---- Pairing ------------------------------------------------------------
char pairLabels[discovery::kMaxRobots + 2][24];
const char* pairItems[discovery::kMaxRobots + 2];

uint8_t buildPairItems() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < foundCount; i++) {
    bool current = robot::isPaired() && found[i].ip == robot::pairedIp();
    snprintf(pairLabels[n], sizeof(pairLabels[n]), "%s%s", found[i].ip.toString().c_str(),
             current ? " *" : "");
    pairItems[n] = pairLabels[n];
    n++;
  }
  pairItems[n++] = "Scan again";
  pairItems[n++] = "Cancel";
  return n;
}

Action updatePairing(const JoyState& s) {
  robot::Telemetry t = robot::snapshot();
  if (view == View::PairWait) {
    if (t.link == robot::LinkState::PairChoose) {
      foundCount = robot::pairResults(found, discovery::kMaxRobots);
      go(View::PairList);
      if (foundCount) buzzer::beep(0, 40);
    } else if (drawDue()) {
      char dots[8] = "";
      for (uint32_t i = 0; i < ((millis() - viewStartMs) / 500) % 4; i++) strcat(dots, ".");
      char l[24];
      snprintf(l, sizeof(l), "searching%s", dots);
      display::drawPage("PAIR ROBOT", "Looking for julia-X2", "on this WiFi network", l,
                        "(takes ~10 s)");
    }
    return Action::None;
  }

  // PairList
  uint8_t n = buildPairItems();
  navigate(s, n);
  if (clicked(s)) {
    if (sel < foundCount) {
      robot::choosePaired(found[sel]);
      buzzer::chirpUp();
      char l[24];
      snprintf(l, sizeof(l), "%s", found[sel].ip.toString().c_str());
      display::drawPage("PAIRED", l, found[sel].hasMac ? discovery::macToString(found[sel].mac).c_str()
                                                       : "(no MAC)",
                        "Saved - reconnects", "automatically.");
      delay(1500);
      go(View::Closed);
    } else if (sel == foundCount) {  // scan again
      robot::startPairing();
      go(View::PairWait);
    } else {  // cancel
      robot::cancelPairing();
      go(View::Closed);
    }
    return Action::None;
  }
  if (drawDue()) {
    if (foundCount == 0) {
      // Keep the list (Scan again / Cancel) but explain.
      const char* items[] = {"Scan again", "Cancel"};
      display::drawList("NO ROBOT FOUND", items, 2, sel);
    } else {
      display::drawList("PICK ROBOT (* = current)", pairItems, n, sel);
    }
  }
  return Action::None;
}

// ---- Calibration ----------------------------------------------------------
void updateCalibration(Joystick& joy) {
  const JoyState& s = joy.state();
  char l1[24], l2[24];
  switch (view) {
    case View::CalRelease: {
      uint32_t el = millis() - viewStartMs;
      if (el >= 2000) {
        if (joy.calibrateCenter()) {
          joy.startRangeCalibration();
          buzzer::beep(0, 40);
          go(View::CalSweep);
        } else {
          buzzer::beep(900, 200);
          go(View::CalRelease);  // stick was moving: count down again
        }
        return;
      }
      if (drawDue()) {
        snprintf(l2, sizeof(l2), "measuring in %lu...", (unsigned long)(2 - el / 1000));
        display::drawPage("CALIBRATE 1/2", "Let go of the stick", "and don't touch it.", l2);
      }
      return;
    }
    case View::CalSweep:
      if (clicked(s)) {
        calOk = joy.finishRangeCalibration();
        if (calOk) buzzer::doubleBeep();
        else buzzer::beep(900, 400);
        go(View::CalResult);
        return;
      }
      if (drawDue()) {
        snprintf(l1, sizeof(l1), "raw X %4u  Y %4u", s.rawX, s.rawY);
        display::drawPage("CALIBRATE 2/2", "Sweep the stick round", "its edge 3-4 times,",
                          "then click the stick.", l1);
      }
      return;
    case View::CalResult:
      if (millis() - viewStartMs > 2500 || clicked(s)) {
        go(View::Main);
        return;
      }
      if (drawDue()) {
        if (calOk) {
          display::drawPage("CALIBRATION SAVED", "Stored in the", "controller - used", "every time you drive.");
        } else {
          display::drawPage("CALIBRATION FAILED", "Push fully to every", "edge, then try again.");
        }
      }
      return;
    default:
      return;
  }
}

// ---- WiFi / Info --------------------------------------------------------
void updateWifi(const JoyState& s) {
  if (clicked(s)) {
    ota::stopPortal();
    go(View::Main);
    return;
  }
  if (!ota::portalActive()) {  // saved or timed out
    if (WiFi.status() == WL_CONNECTED) buzzer::chirpUp();
    go(View::Main);
    return;
  }
  if (drawDue()) {
    char l2[28];
    snprintf(l2, sizeof(l2), "pw: %s", PORTAL_PASSWORD);
    display::drawPage("WIFI SETUP", "Phone: join WiFi", PORTAL_SSID, l2,
                      "open 192.168.4.1");
  }
}

void updateInfo(const JoyState& s) {
  if (clicked(s)) {
    go(View::Main);
    return;
  }
  if (!drawDue()) return;
  robot::Telemetry t = robot::snapshot();
  char l1[28], l2[28], l3[28], l4[28];
  snprintf(l1, sizeof(l1), "%.20s %ddBm", ota::currentSsid().c_str(),
           WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  snprintf(l2, sizeof(l2), "me    %s", WiFi.localIP().toString().c_str());
  snprintf(l3, sizeof(l3), "robot %s", robot::isPaired() ? robot::pairedIp().toString().c_str() : "not paired");
  if (t.rttMs >= 0) snprintf(l4, sizeof(l4), "rtt %d ms  max %d ms", t.rttMs, t.rttMaxMs);
  else snprintf(l4, sizeof(l4), "rtt --");
  display::drawPage("INFO  fw " FW_VERSION, l1, l2, l3, l4);
}

}  // namespace

void open() {
  go(View::Main);
  buzzer::beep(0, 40);
}

void openPairing() {
  go(robot::snapshot().link == robot::LinkState::PairChoose ? View::PairList : View::PairWait);
  if (view == View::PairList) foundCount = robot::pairResults(found, discovery::kMaxRobots);
}

bool active() { return view != View::Closed; }

Action update(Joystick& joy) {
  const JoyState& s = joy.state();
  switch (view) {
    case View::Closed:
      return Action::None;

    case View::Main:
      navigate(s, kMainCount);
      if (clicked(s)) {
        switch (sel) {
          case 0: go(View::Closed); break;
          case 1: go(View::CalRelease); break;
          case 2:
            robot::startPairing();
            go(View::PairWait);
            break;
          case 3:
            ota::startPortal();
            go(View::Wifi);
            break;
          case 4: go(View::Info); break;
          case 5:
            go(View::Closed);
            return Action::EnterTestMode;
        }
        return Action::None;
      }
      if (drawDue()) display::drawList("MENU  (click = select)", kMain, kMainCount, sel);
      return Action::None;

    case View::CalRelease:
    case View::CalSweep:
    case View::CalResult:
      updateCalibration(joy);
      return Action::None;

    case View::PairWait:
    case View::PairList:
      return updatePairing(s);

    case View::Wifi:
      updateWifi(s);
      return Action::None;

    case View::Info:
      updateInfo(s);
      return Action::None;
  }
  return Action::None;
}

}  // namespace menu
