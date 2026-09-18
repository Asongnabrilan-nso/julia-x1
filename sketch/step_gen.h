// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// Timer-driven step-pulse generator for the two A4988 drivers.
//
// Why this replaces AccelStepper::runSpeed() in loop(): a balancing robot needs *steady* step
// timing at several kHz while the same CPU is doing 400 kHz I2C reads and Bridge UART traffic.
// Polling from loop() stalls the pulses during every one of those. Here a 10 kHz kernel timer
// interrupt (higher priority than every thread) emits the pulses, so I2C, the control loop and
// the Bridge can take as long as they like without ever disturbing the wheels.
//
// Algorithm: each 100 us tick a Bresenham accumulator adds |steps/s|; every time it passes
// 10 000 one step is due. That gives an exactly-correct average rate (1 step/s resolution) with
// at most +-100 us of jitter per step -- far below what a wheel + 1/16 microstepping can resolve.
// Up to two steps per tick per wheel are emitted, which supports 20 000 steps/s.
//
// The ISR uses integer maths only (cheap FPU-less context, no lazy-stacking cost).
//
// Dead-man watchdog: if the control thread stops calling feed() for WATCHDOG_TICKS the ISR
// forces both wheels to zero. A hung I2C bus or a crashed control loop therefore stops the
// robot instead of leaving it driving at the last commanded speed.

#pragma once

#include <Arduino.h>

namespace stepgen {

constexpr int32_t TICK_HZ = 10000;                  // must match the k_timer period below
constexpr uint32_t WATCHDOG_TICKS = 300;            // 30 ms without feed() -> stop
constexpr int32_t MAX_STEPS_PER_SEC = 2 * TICK_HZ;  // 2 steps per tick

struct Wheel {
  uint8_t stepPin;
  uint8_t dirPin;
  volatile int32_t speed;  // steps/s, sign = direction (already includes motor inversion)
  int32_t acc;
  int8_t dirLevel;  // last level written to the DIR pin, -1 = unknown
};

static Wheel wheels[2];
static volatile uint32_t watchdog = 0;
static volatile bool running = false;
static struct k_timer stepTimer;

// Called from the control thread every cycle.
inline void feed() { watchdog = 0; }

inline void setSpeeds(int32_t left, int32_t right) {
  if (left > MAX_STEPS_PER_SEC) left = MAX_STEPS_PER_SEC;
  if (left < -MAX_STEPS_PER_SEC) left = -MAX_STEPS_PER_SEC;
  if (right > MAX_STEPS_PER_SEC) right = MAX_STEPS_PER_SEC;
  if (right < -MAX_STEPS_PER_SEC) right = -MAX_STEPS_PER_SEC;
  wheels[0].speed = left;
  wheels[1].speed = right;
}

static void onTick(struct k_timer*) {
  if (!running) return;
  bool starved = (++watchdog > WATCHDOG_TICKS);
  if (watchdog > 1000000u) watchdog = WATCHDOG_TICKS + 1;  // never wrap back to "healthy"

  int8_t n[2] = {0, 0};
  for (int i = 0; i < 2; ++i) {
    Wheel& w = wheels[i];
    int32_t s = starved ? 0 : w.speed;
    if (s != 0) {
      int8_t level = (s > 0) ? HIGH : LOW;  // same convention as AccelStepper: +speed -> DIR high
      if (level != w.dirLevel) {
        digitalWrite(w.dirPin, level);
        w.dirLevel = level;
      }
      w.acc += (s > 0) ? s : -s;
      while (w.acc >= TICK_HZ && n[i] < 2) {
        w.acc -= TICK_HZ;
        n[i]++;
      }
      if (w.acc >= TICK_HZ) w.acc = TICK_HZ - 1;  // over-speed request: drop, don't burst later
    } else {
      w.acc = 0;
    }
  }

  // Up to two rounds; both wheels' STEP edges are shared so the pulse-width wait is paid once.
  for (int round = 0; round < 2; ++round) {
    bool any = false;
    for (int i = 0; i < 2; ++i) {
      if (n[i] > round) {
        digitalWrite(wheels[i].stepPin, HIGH);
        any = true;
      }
    }
    if (!any) break;
    delayMicroseconds(3);  // A4988 needs >= 1 us high; margin for slow GPIO
    for (int i = 0; i < 2; ++i) {
      if (n[i] > round) digitalWrite(wheels[i].stepPin, LOW);
    }
    if (round == 0) delayMicroseconds(3);  // >= 1 us low before the next pulse
  }
}

inline void begin(uint8_t leftStep, uint8_t leftDir, uint8_t rightStep, uint8_t rightDir) {
  wheels[0] = {leftStep, leftDir, 0, 0, -1};
  wheels[1] = {rightStep, rightDir, 0, 0, -1};
  for (int i = 0; i < 2; ++i) {
    pinMode(wheels[i].stepPin, OUTPUT);
    pinMode(wheels[i].dirPin, OUTPUT);
    digitalWrite(wheels[i].stepPin, LOW);
  }
  watchdog = 0;
  running = true;
  k_timer_init(&stepTimer, onTick, NULL);
  k_timer_start(&stepTimer, K_USEC(100), K_USEC(100));
}

}  // namespace stepgen
