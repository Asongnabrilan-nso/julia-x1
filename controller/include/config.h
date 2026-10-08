#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin map — Seeed XIAO ESP32C3
// ---------------------------------------------------------------------------
// Joystick axes MUST be on ADC1 (GPIO0-4). ADC2 (GPIO5 / A3 / D3) cannot be
// read while WiFi is active. GPIO2 (A0 / D0) is a boot strapping pin, so the
// axes avoid it too. Power the joystick module from 3V3, NOT 5V — the ADC
// pins are not 5 V tolerant.
constexpr uint8_t PIN_JOY_X  = A1;   // D1 / GPIO3  <- joystick VRx
constexpr uint8_t PIN_JOY_Y  = A2;   // D2 / GPIO4  <- joystick VRy
constexpr uint8_t PIN_JOY_SW = D3;   // D3 / GPIO5  <- joystick SW (active low, internal pull-up)

// OLED 128x64 (I2C): SDA = D4 / GPIO6, SCL = D5 / GPIO7 (Wire defaults)
constexpr uint8_t  OLED_I2C_ADDR  = 0x3C;  // 0x3D also probed automatically
constexpr uint32_t OLED_PERIOD_MS = 100;   // 10 Hz redraw (~25 ms per frame at 400 kHz)

// Buzzer on D6 / GPIO21. GPIO21 is also UART0 TX, so the boot ROM log may make
// it click briefly at power-up - harmless.
constexpr uint8_t  PIN_BUZZER          = D6;
constexpr bool     BUZZER_ACTIVE       = false; // true: active buzzer (beeps on plain DC)
                                                // false: passive buzzer (needs a tone)
constexpr uint16_t BUZZER_FREQ_HZ      = 2700;  // typical passive-buzzer resonance
constexpr uint8_t  BUZZER_LEDC_CHANNEL = 3;

// ---------------------------------------------------------------------------
// Joystick processing
// ---------------------------------------------------------------------------
// Calibrated output is -1000..1000 per axis: +x = right, +y = forward.
constexpr uint32_t JOY_SAMPLE_PERIOD_MS = 10;   // 100 Hz update
constexpr uint8_t  JOY_OVERSAMPLE       = 8;    // ADC reads averaged per sample
constexpr float    JOY_EMA_ALPHA        = 0.35f;// smoothing (1 = none)
constexpr int16_t  JOY_DEADZONE         = 80;   // of 1000, around center
constexpr int16_t  JOY_EDGE_MARGIN      = 40;   // ADC counts trimmed off each calibrated
                                                // extreme so full throw reliably hits ±1000
constexpr int16_t  JOY_DIR_THRESHOLD    = 300;  // of 1000, to report a direction

// Range-calibration sanity: each half-axis (min..center, center..max) must
// span at least this many counts or the calibration is rejected.
constexpr uint16_t JOY_MIN_HALF_SPAN    = 400;

// Center measurement at boot / recenter: samples must agree within this
// spread, and sit within this window, or the stick is assumed to be held.
constexpr uint16_t JOY_CENTER_MAX_SPREAD = 120;
constexpr uint16_t JOY_CENTER_MIN        = 900;
constexpr uint16_t JOY_CENTER_MAX        = 3200;

// Orientation fixups — set these after the first test so that pushing the
// stick away from you reads +y (forward) and right reads +x.
constexpr bool JOY_SWAP_XY  = true;
constexpr bool JOY_INVERT_X = true;
constexpr bool JOY_INVERT_Y = true;

// Button
constexpr uint32_t BTN_DEBOUNCE_MS   = 30;
constexpr uint32_t BTN_LONG_PRESS_MS = 1500;

// ---------------------------------------------------------------------------
// Serial streaming
// ---------------------------------------------------------------------------
constexpr uint32_t STREAM_PERIOD_MS = 50;   // 20 Hz
// One-line status heartbeat (fw, WiFi, IP, RSSI, OTA). The boot log is gone
// by the time a monitor attaches, so this is how the IP is always visible.
constexpr uint32_t STATUS_PERIOD_MS = 10000;

// ---------------------------------------------------------------------------
// Robot link (host/port come from [joy] in platformio.ini)
// ---------------------------------------------------------------------------
// The robot zeroes its setpoints after 500 ms without a command and rate-limits
// joystick input to 20 Hz, so send a little slower than that while the stick
// is deflected.
constexpr uint32_t LINK_SEND_PERIOD_MS = 60;
constexpr uint32_t LINK_STALE_MS       = 1000;  // no "balance" telemetry -> link shown as down
constexpr uint32_t LINK_RECONNECT_MS   = 2000;
constexpr float    ROBOT_DEADZONE      = 0.15f; // must match DEADZONE in python/main.py
