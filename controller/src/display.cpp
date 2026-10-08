#include "display.h"

#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>

#include "config.h"
#include "robot_link.h"

namespace display {
namespace {

// Switch to U8G2_SH1106_128X64_NONAME_F_HW_I2C for 1.3" SH1106 modules.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

bool found = false;
uint32_t lastDrawMs = 0;

uint8_t probeAddress() {
  for (uint8_t addr : {OLED_I2C_ADDR, (uint8_t)0x3C, (uint8_t)0x3D}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return addr;
  }
  return 0;
}

void drawWifi(int x, int y) {
  // 4 signal bars, or "--" when disconnected
  if (WiFi.status() != WL_CONNECTED) {
    oled.drawStr(x + 4, y + 8, "--");
    return;
  }
  int rssi = WiFi.RSSI();
  int bars = rssi > -60 ? 4 : rssi > -70 ? 3 : rssi > -80 ? 2 : 1;
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    if (i < bars) oled.drawBox(x + i * 4, y + 8 - h, 3, h);
    else oled.drawFrame(x + i * 4, y + 8 - h, 3, h);
  }
}

// ---------------------------------------------------------------------------
// Drive screen: robot mode, ToF distance, robot direction, our command
// ---------------------------------------------------------------------------
//   MANUAL                    ▂▄▆█
//   ─────────────────────────────
//   ToF  342 mm          fwd 60%
//   Robot: Driving forward
//   Cmd:   FORWARD 80%
//   max speed 100%

const char* headerText(const robot::Telemetry& t) {
  switch (t.link) {
    case robot::LinkState::NoWifi: return "No WiFi";
    case robot::LinkState::Unpaired: return "NOT PAIRED";
    case robot::LinkState::Connecting: return "Connecting...";
    case robot::LinkState::Searching: return "Finding robot...";
    case robot::LinkState::PairScanning:
    case robot::LinkState::PairChoose: return "Pairing...";
    default: break;
  }
  if (!t.fresh) return "Robot silent";
  if (t.state == robot::ST_FAULT) return "FAULT";
  if (t.mode == robot::MODE_BALANCE) {
    if (t.state == robot::ST_BALANCING) return "BALANCE: on";
    if (t.state == robot::ST_ARMED) return "BALANCE: armed";
    return "BALANCE: off";
  }
  if (t.mode == robot::MODE_MANUAL) return "MANUAL";
  return "Robot ?";
}

void drawTof(const robot::Telemetry& t) {
  char buf[20];
  oled.setFont(u8g2_font_7x14B_tr);
  if (!t.fresh || t.tofStatus < 0) snprintf(buf, sizeof(buf), "ToF   --");
  else if (t.tofStatus == robot::TOF_MISSING) snprintf(buf, sizeof(buf), "ToF  n/a");
  else if (t.tofStatus == robot::TOF_TIMEOUT) snprintf(buf, sizeof(buf), "ToF  err");
  else if (t.tofMm < 0) snprintf(buf, sizeof(buf), "ToF >1.2m");
  else snprintf(buf, sizeof(buf), "ToF %4d mm", t.tofMm);
  oled.drawStr(0, 26, buf);

  // Obstacle avoidance is limiting forward speed
  oled.setFont(u8g2_font_6x10_tr);
  if (t.fresh && t.tofStatus == robot::TOF_OK && t.tofFactor < 100) {
    if (t.tofFactor == 0) {
      snprintf(buf, sizeof(buf), "STOP");
    } else {
      snprintf(buf, sizeof(buf), "%u%%", t.tofFactor);
    }
    oled.drawStr(128 - oled.getStrWidth(buf), 26, buf);
  }
}

void drawCommand(const JoyState& s, const robot::Telemetry& t, bool waitingForCenter) {
  char buf[28];
  if (waitingForCenter) {
    oled.drawStr(0, 52, "Cmd: release stick");
    return;
  }
  float x = s.x / 1000.0f, y = s.y / 1000.0f;
  float mag = sqrtf(x * x + y * y);
  if (!t.socket || !robot::commanding() || mag < ROBOT_DEADZONE) {
    oled.drawStr(0, 52, "Cmd: --");
    return;
  }
  const char* dir = fabsf(y) >= fabsf(x) ? (y > 0 ? "FORWARD" : "BACK") : (x > 0 ? "RIGHT" : "LEFT");
  snprintf(buf, sizeof(buf), "Cmd: %s %d%%", dir, (int)(min(mag, 1.0f) * 100 + 0.5f));
  oled.drawStr(0, 52, buf);
}

void drawDrive(const Joystick& joy, bool waitingForCenter) {
  robot::Telemetry t = robot::snapshot();
  char buf[32];

  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 8, headerText(t));
  drawWifi(110, 0);
  oled.drawHLine(0, 10, 128);

  drawTof(t);

  oled.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "Robot: %s", t.fresh && t.action[0] ? t.action : "--");
  oled.drawStr(0, 40, buf);

  drawCommand(joy.state(), t, waitingForCenter);

  // Footer: why commands may be ignored, else the dashboard speed limit
  if (t.fresh && t.mode == robot::MODE_BALANCE && t.state != robot::ST_BALANCING &&
      t.state != robot::ST_ARMED) {
    oled.drawStr(0, 63, "arm on dashboard");
  } else if (t.fresh && t.state == robot::ST_FAULT) {
    oled.drawStr(0, 63, "disarm on dashboard");
  } else if (t.fresh) {
    // Dashboard speed limit + command round-trip latency (worst of last 5 s)
    if (t.rttMs >= 0) {
      snprintf(buf, sizeof(buf), "spd %d%%  %dms (%d)", (int)(t.speedScale * 100 + 0.5f), t.rttMs,
               t.rttMaxMs);
    } else {
      snprintf(buf, sizeof(buf), "max speed %d%%", (int)(t.speedScale * 100 + 0.5f));
    }
    oled.drawStr(0, 63, buf);
  } else if (t.link == robot::LinkState::Unpaired) {
    oled.drawStr(0, 63, "hold btn 3s: menu");
  } else if (robot::isPaired()) {
    oled.drawStr(0, 63, robot::pairedIp().toString().c_str());
  }
}

// ---------------------------------------------------------------------------
// Test screen (bench tools): stick position box + raw/calibrated values
// ---------------------------------------------------------------------------
constexpr int kBoxX = 0, kBoxY = 12, kBoxSize = 52;

void drawStick(const JoyState& s, bool calibrating) {
  oled.drawFrame(kBoxX, kBoxY, kBoxSize, kBoxSize);
  int cx = kBoxX + kBoxSize / 2, cy = kBoxY + kBoxSize / 2;
  oled.drawHLine(cx - 2, cy, 5);
  oled.drawVLine(cx, cy - 2, 5);

  int px, py;
  if (calibrating) {
    // raw ADC position, so the full sweep is visible before calibration exists
    px = map(s.rawX, 0, 4095, kBoxX + 3, kBoxX + kBoxSize - 4);
    py = map(s.rawY, 0, 4095, kBoxY + kBoxSize - 4, kBoxY + 3);
  } else {
    px = map(s.x, -1000, 1000, kBoxX + 3, kBoxX + kBoxSize - 4);
    py = map(s.y, -1000, 1000, kBoxY + kBoxSize - 4, kBoxY + 3);  // +y = up
  }
  if (s.pressed) oled.drawDisc(px, py, 3);
  else oled.drawCircle(px, py, 3);
}

void drawTest(const Joystick& joy) {
  const JoyState& s = joy.state();
  bool cal = joy.rangeCalibrating();
  char buf[24];

  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 8, cal ? "CALIBRATING" : joy.hasStoredRange() ? "TEST MODE" : "TEST (no cal)");
  drawWifi(110, 0);
  oled.drawHLine(0, 10, 128);

  drawStick(s, cal);

  const int tx = 58;
  if (cal) {
    snprintf(buf, sizeof(buf), "rawX %4u", s.rawX);
    oled.drawStr(tx, 22, buf);
    snprintf(buf, sizeof(buf), "rawY %4u", s.rawY);
    oled.drawStr(tx, 33, buf);
    oled.drawStr(tx, 47, "sweep edges");
    oled.drawStr(tx, 58, "then click");
  } else {
    snprintf(buf, sizeof(buf), "X %5d", s.x);
    oled.drawStr(tx, 22, buf);
    snprintf(buf, sizeof(buf), "Y %5d", s.y);
    oled.drawStr(tx, 33, buf);
    snprintf(buf, sizeof(buf), "BTN %s", s.pressed ? "ON" : "off");
    oled.drawStr(tx, 44, buf);
    oled.setFont(u8g2_font_7x14B_tr);
    oled.drawStr(tx, 61, joyDirName(s.dir));
  }
}

}  // namespace

bool begin() {
  Wire.begin();  // SDA = D4 / GPIO6, SCL = D5 / GPIO7
  uint8_t addr = probeAddress();
  if (!addr) {
    Serial.println("[OLED] no display found on I2C (0x3C/0x3D) - check wiring, display disabled");
    return false;
  }
  oled.setI2CAddress(addr << 1);  // U8g2 takes the 8-bit address
  oled.setBusClock(400000);
  found = oled.begin();
  Serial.printf("[OLED] %s at 0x%02X\n", found ? "ready" : "init failed", addr);
  if (found) showMessage("Julia-x2 Joy", "fw " FW_VERSION);
  return found;
}

bool present() { return found; }

void drawList(const char* title, const char* const* items, uint8_t n, uint8_t sel) {
  if (!found) return;
  constexpr uint8_t kRows = 4;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 8, title);
  oled.drawHLine(0, 10, 128);
  uint8_t first = sel >= kRows ? sel - kRows + 1 : 0;
  for (uint8_t r = 0; r < kRows && first + r < n; r++) {
    uint8_t i = first + r;
    int y = 22 + r * 13;
    if (i == sel) {
      oled.drawBox(0, y - 10, 128, 12);
      oled.setDrawColor(0);
    }
    oled.drawStr(4, y, items[i]);
    oled.setDrawColor(1);
  }
  if (first > 0) oled.drawStr(120, 22, "^");
  if (first + kRows < n) oled.drawStr(120, 61, "v");
  oled.sendBuffer();
}

void drawPage(const char* title, const char* l1, const char* l2, const char* l3, const char* l4) {
  if (!found) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 8, title);
  oled.drawHLine(0, 10, 128);
  const char* lines[] = {l1, l2, l3, l4};
  for (int i = 0; i < 4; i++) {
    if (lines[i]) oled.drawStr(0, 23 + i * 13, lines[i]);
  }
  oled.sendBuffer();
}

void showMessage(const char* line1, const char* line2) {
  if (!found) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x14B_tr);
  oled.drawStr(0, 24, line1);
  if (line2) {
    oled.setFont(u8g2_font_6x10_tr);
    oled.drawStr(0, 44, line2);
  }
  oled.sendBuffer();
}

void update(Screen screen, const Joystick& joy, bool waitingForCenter) {
  if (!found) return;
  uint32_t now = millis();
  if (now - lastDrawMs < OLED_PERIOD_MS) return;
  lastDrawMs = now;

  oled.clearBuffer();
  if (screen == Screen::Test) drawTest(joy);
  else drawDrive(joy, waitingForCenter);
  oled.sendBuffer();
}

}  // namespace display
