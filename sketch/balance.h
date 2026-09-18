// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// Hardware-independent balancing core: attitude filter + cascaded controller.
// Deliberately free of Arduino/Zephyr includes so the *exact same code* runs in the
// host-side simulator (tools/sim/) and on the MCU.
//
// Conventions (everything is expressed from the robot's point of view):
//   angle  > 0  : robot leans FORWARD          (deg)
//   rate   > 0  : robot is falling forward     (deg/s)
//   speed  > 0  : wheels roll forward          (microsteps/s)
//
// Control structure (after the "high speed balancing robot" design by Kloppertje):
//
//   joystick fwd --> [slew] --> v_set
//                                  |  (v_set - v)
//                                  v
//                       speed PI  -->  target lean angle
//                                  |
//        angle - trim - lean ----->|
//                                  v
//              angle PD (P on angle, D on GYRO rate)  -->  wheel ACCELERATION
//                                  |
//                                  v  integrate (limited by amax / vmax)
//                              wheel speed v --> step generator
//
// Why acceleration and not "output = speed": a stepper is a velocity source, and the
// inverted-pendulum dynamics are driven by wheel *acceleration*
// (theta'' = g/L * theta - a/L). Integrating the PD output turns the loop into the
// physically correct form, keeps the wheel speed continuous (no missed steps) and lets
// the speed loop hold the robot in place without a steady-state lean.

#pragma once

#include <math.h>
#include <stdint.h>

namespace jb {

static const float kRadToDeg = 57.29577951f;
// Gains in Params are expressed for this wheel diameter and rescaled to the configured one.
static const float kRefWheelMm = 80.0f;

inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

inline float slew(float current, float target, float maxDelta) {
  float d = target - current;
  if (d > maxDelta) return current + maxDelta;
  if (d < -maxDelta) return current - maxDelta;
  return target;
}

// Every value here can be changed at run time from the dashboard (see PARAM_TABLE in
// sketch.ino for names and limits). Defaults are a *starting point* -- see docs/BALANCING.md.
struct Params {
  // --- angle loop (inner, fast) ---
  float angle_kp = 3200.0f;   // steps/s^2 per degree of angle error
  float angle_kd = 250.0f;    // steps/s^2 per deg/s of gyro rate
  // --- speed loop (outer, slow) ---
  float speed_kp = 0.2f;      // degrees of lean per 1000 steps/s of speed error
  float speed_ki = 0.10f;     // degrees of lean per (1000 steps/s * s)
  float max_lean = 6.0f;      // deg, cap on the lean the speed loop may ask for
  // --- geometry / calibration ---
  float trim = 0.0f;          // deg, pitch reading at which the robot balances by itself
  float wheel_mm = 80.0f;     // mm, drive-wheel diameter; gains above are rescaled from an 80 mm reference
  float cf_tau = 0.6f;        // s, complementary filter time constant (gyro <-> accel)
  // --- limits ---
  float vmax = 10000.0f;      // steps/s, wheel speed limit while balancing
  float amax = 60000.0f;      // steps/s^2, wheel acceleration limit
  float fall_deg = 30.0f;     // deg, beyond this the robot is considered fallen -> motors off
  float arm_deg = 2.0f;       // deg, how upright the robot must be to engage after arming
  // --- driving ---
  float fwd_max = 4000.0f;    // steps/s at full-forward joystick
  float steer_max = 2500.0f;  // steps/s differential at full-turn joystick
  float ramp = 12000.0f;      // steps/s^2, slew of joystick setpoints (balance mode)
  float manual_ramp = 40000.0f;  // steps/s^2, wheel acceleration limit in manual mode
  // --- mounting (integers stored as float so one table handles everything) ---
  float axis = 0.0f;          // 0: pitch about IMU X axis, 1: pitch about IMU Y axis
  float invert = 0.0f;        // 1: flip pitch sign (so "lean forward" reads positive)
  float invert_left = 0.0f;   // 1: reverse left motor direction
  float invert_right = 0.0f;  // 1: reverse right motor direction
};

// ---------------------------------------------------------------------------------------
// Attitude: complementary filter. Gyro integrates (smooth, drifts), accelerometer
// corrects (noisy, drift-free). tau is the crossover time constant.
// ---------------------------------------------------------------------------------------
struct AttitudeFilter {
  float angle = 0.0f;  // deg, filtered pitch (before trim)
  bool initialised = false;

  void reset() { initialised = false; }

  float update(float gyroDps, float accelAngleDeg, float dt, float tau) {
    if (!initialised) {
      angle = accelAngleDeg;
      initialised = true;
      return angle;
    }
    float a = tau / (tau + dt);
    angle = a * (angle + gyroDps * dt) + (1.0f - a) * accelAngleDeg;
    return angle;
  }
};

// Pitch from raw accelerometer (any unit) for the chosen mounting axis.
inline float accelPitchDeg(float ax, float ay, float az, int axis) {
  // Pitch about X uses Y/Z, pitch about Y uses X/Z. atan2(.., az) is well-behaved through
  // +-90 deg, unlike the asin/atan(y/sqrt()) forms.
  return (axis == 0 ? atan2f(ay, az) : atan2f(-ax, az)) * kRadToDeg;
}

// ---------------------------------------------------------------------------------------
// Balance controller.
// ---------------------------------------------------------------------------------------
class Balancer {
 public:
  float v = 0.0f;         // steps/s, common (mean) wheel speed -- this is also the *measured*
                          // speed, since steppers do not slip when driven within limits
  float v_set = 0.0f;     // steps/s, slewed forward setpoint
  float turn = 0.0f;      // steps/s, slewed differential setpoint
  float lean_i = 0.0f;    // deg, integral part of the speed loop
  float lean_target = 0.0f;  // deg, output of the speed loop (telemetry)
  float accel = 0.0f;     // steps/s^2, output of the angle loop (telemetry)
  float sat_time = 0.0f;  // s, how long the wheel speed has been pinned at vmax

  void reset() {
    v = v_set = turn = lean_i = lean_target = accel = sat_time = 0.0f;
  }

  // angle: filtered pitch minus nothing (raw, trim is applied here); rate: gyro deg/s.
  // fwd, steer in -1..1. Returns via vL/vR in steps/s.
  void update(const Params& p, float angle, float rate, float dt, float fwd, float steer,
              float* vL, float* vR) {
    // --- setpoints (slew-limited so a joystick jump never shocks the angle loop) ---
    v_set = slew(v_set, clampf(fwd, -1.0f, 1.0f) * p.fwd_max, p.ramp * dt);
    turn = slew(turn, clampf(steer, -1.0f, 1.0f) * p.steer_max, p.ramp * dt);

    // A given wheel acceleration in steps/s^2 moves the robot proportionally to the wheel size,
    // so for the same physical behaviour: angle gains scale with 1/diameter, speed gains with
    // diameter. (Verified in tools/sim: 60 mm and 120 mm robots match the 80 mm reference.)
    const float gs = kRefWheelMm / clampf(p.wheel_mm, 20.0f, 400.0f);

    // --- outer loop: speed error -> desired lean ---
    float vErr = (v_set - v) * 0.001f;
    lean_i = clampf(lean_i + (p.speed_ki / gs) * vErr * dt, -p.max_lean, p.max_lean);
    lean_target = clampf((p.speed_kp / gs) * vErr + lean_i, -p.max_lean, p.max_lean);

    // --- inner loop: angle PD on gyro rate -> wheel acceleration ---
    float err = angle - (p.trim + lean_target);
    accel = clampf(gs * (p.angle_kp * err + p.angle_kd * rate), -p.amax, p.amax);

    // --- integrate to wheel speed ---
    v = clampf(v + accel * dt, -p.vmax, p.vmax);
    if (fabsf(v) >= p.vmax) {
      sat_time += dt;
    } else {
      sat_time = 0.0f;
    }

    // Differential steering rides on top of the common speed. Clamped so a turn at speed
    // can never ask for more than the wheel limit.
    *vL = clampf(v + turn, -p.vmax, p.vmax);
    *vR = clampf(v - turn, -p.vmax, p.vmax);
  }
};

}  // namespace jb
