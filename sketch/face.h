// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// julia-X2 face: spring-animated eyes (after Grobot_Animations) on a 2.8" ILI9341 320x240 SPI TFT.
//
// The face runs in its own low-priority thread (same priority as loop(), far below the balance
// thread). loop() hands it a snapshot of what the robot is doing via observe(); the face picks a
// mood, colour and gaze from that, and Python can override the mood or fire one-shot animations
// through the Bridge ("face_mood" / "face_anim").
//
// Only the display is used: the module's SD card and touch controller are left unconnected.

#pragma once

#include <Arduino.h>

namespace face {

// Wiring (see README, "TFT face display"). SCK = D13, MOSI = D11 and MISO = D12 are the UNO Q's
// hardware SPI pins and are fixed. D8 is NOT available: it is the CNC shield's stepper ENABLE.
const uint8_t PIN_CS = 10;
const uint8_t PIN_DC = 9;
const uint8_t PIN_RST = 7;  // CNC shield Z-axis DIR: free because no Z driver is fitted

// Moods for "face_mood". AUTO = follow the robot's state. Must match FACE_MOODS in python/main.py.
// SHOWCASE cycles through all the others on its own.
enum Mood : uint8_t {
  MOOD_AUTO = 0, MOOD_NEUTRAL, MOOD_HAPPY, MOOD_TIRED, MOOD_ANGRY,
  MOOD_SAD, MOOD_SURPRISED, MOOD_SUSPICIOUS, MOOD_LOVE, MOOD_SHOWCASE, MOOD_COUNT
};
// One-shot animations for "face_anim". Must match FACE_ANIMS in python/main.py.
// INTRO replays the boot animation.
enum Anim : uint8_t {
  ANIM_NONE = 0, ANIM_BLINK, ANIM_LAUGH, ANIM_CONFUSED,
  ANIM_WINK, ANIM_LOOK_AROUND, ANIM_NOD, ANIM_DIZZY, ANIM_SURPRISE, ANIM_INTRO, ANIM_COUNT
};
// Robot state as reported to observe(). Must match enum State in sketch.ino.
enum RobotState : uint8_t { ROBOT_IDLE = 0, ROBOT_ARMED = 1, ROBOT_BALANCING = 2, ROBOT_FAULT = 3 };

// Starts the face thread (the TFT is initialised there and the boot intro plays, so setup() is not
// delayed).
// Returns false if no thread could be created; then call service() from loop() instead.
bool begin();
void service();

// Called from loop(). fwd/steer: drive command -1..1 (+fwd = forward, +steer = turn right), 0 when
// the joystick timed out. near: something is inside the ToF slow-down zone. blocked: the operator is
// driving forward but obstacle avoidance holds the robot back.
void observe(uint8_t robotState, float fwd, float steer, bool near, bool blocked);

void setMood(uint8_t mood);  // Bridge "face_mood"; out-of-range values are ignored
void play(uint8_t anim);     // Bridge "face_anim"

// One-off status line for the serial log (display init result), or nullptr if nothing new.
const char* takeLog();

}  // namespace face
