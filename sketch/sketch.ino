// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

// julia-X2 -- self-balancing robot firmware (MCU side).
//
// Thread / interrupt layout (highest priority first):
//   1. k_timer ISR, 10 kHz ........ step_gen.h  : emits STEP/DIR pulses, dead-man watchdog
//   2. "balance" thread, 200 Hz ... this file   : IMU read -> attitude filter -> controller ->
//                                                 wheel speeds. NEVER talks to the Bridge, so a
//                                                 slow UART / lost WiFi cannot upset balance.
//   3. "bridge" thread ............ library     : services RPCs from Python (params, commands)
//   4. loop() (main thread) ....... this file   : publishes telemetry to Python, prints events
//
// The balance loop is fully autonomous: once armed, the robot keeps itself upright with no help
// from the Linux side. If Python or WiFi dies the joystick command simply times out to zero.
//
// Control maths lives in balance.h (host-testable, see tools/sim/). Tuning: docs/BALANCING.md.

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "balance.h"
#include "mpu6050.h"
#include "step_gen.h"

// ---------------------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------------------
// CNC shield wiring: left wheel driver on the X-axis header, right wheel driver on the Y-axis header
const uint8_t LEFT_STEP_PIN = 2;
const uint8_t LEFT_DIR_PIN = 5;
const uint8_t RIGHT_STEP_PIN = 3;
const uint8_t RIGHT_DIR_PIN = 6;
// CNC shield v3 routes every driver's /EN pin to D8 (active LOW). We drive it so the motors can
// be de-energised after a fall. If your shield does not use D8 this is harmless.
const uint8_t MOTOR_EN_PIN = 8;

// Motors: hybrid NEMA17, 1.8 deg/step, A4988 at 1/16 microstepping -> 3200 microsteps/rev.
// MS1/MS2/MS3 jumpers (all HIGH) and the A4988 current-limit trimpot are hardware settings this
// sketch cannot make: see docs/BALANCING.md, "Hardware checklist".
const uint8_t MICROSTEPS = 16;
const long STEPS_PER_REV = 200L * MICROSTEPS;
const long MANUAL_MAX_STEPS_PER_SEC = 16000;  // 5 rev/s, cap for the plain (non-balancing) drive mode

// IMU (MPU6050) on the Qwiic connector, wired to Wire1.
Mpu6050 imu(Wire1);

// VL53L0X time-of-flight distance sensor on A4/A5, which is the Wire2 bus (the IMU owns Wire1, so
// the two never share a bus). Forward-facing: used for obstacle avoidance only.
VL53L0X tof;

// Datasheet operating envelope (default ranging profile): reliable 30 mm..~1200 mm indoors on a
// light target, +-3% typical accuracy, 25 deg field of view, 8190/8191 mm = "nothing seen".
// Dark, glass or shiny targets and direct sunlight shorten/corrupt the reading.
const uint16_t TOF_OUT_OF_RANGE_MM = 8190;   // >= this from the library means "no target"
const uint16_t TOF_MAX_RELIABLE_MM = 1200;
const uint16_t TOF_PERIOD_MS = 50;           // 20 Hz continuous ranging
const uint32_t TOF_TIMING_BUDGET_US = 33000; // library default; must stay < period
const unsigned long TOF_STALE_MS = 400;      // no fresh reading this long -> treat forward as blocked
const unsigned long TOF_RETRY_MS = 2000;     // sensor missing: retry init this often
const float AVOID_SLOW_BAND = 0.5f;          // forward speed ramps down from thr*(1+band) to thr

// ---------------------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------------------
const uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
const float CONTROL_DT = CONTROL_PERIOD_US * 1e-6f;
const unsigned long COMMAND_TIMEOUT_MS = 500;       // joystick silence -> setpoints to zero
const unsigned long TELEMETRY_BALANCE_MS = 50;      // 20 Hz
const unsigned long TELEMETRY_TOF_MS = 100;         // 10 Hz
const unsigned long TELEMETRY_IMU_MS = 200;         // 5 Hz (raw readouts for the dashboard)
const int CONTROL_STACK_SIZE = 3072;
const int CONTROL_PRIORITY = 3;                     // preempts the Bridge thread (5) and loop() (14)
const float ARM_HOLD_S = 0.4f;                      // upright this long before motors engage
const float ARM_MAX_RATE_DPS = 25.0f;               // ...and not swinging faster than this
const float RUNAWAY_SAT_S = 1.0f;                   // wheel pinned at vmax this long -> fault
const int IMU_FAIL_LIMIT = 5;                       // consecutive failed reads (25 ms) -> fault
const int CALIB_SAMPLES = 300;                      // 1.5 s of gyro averaging
const float CALIB_MAX_STDDEV_DPS = 1.5f;            // reject calibration if the robot was moving
const int TRIM_SAMPLES = 100;                       // 0.5 s pitch average for "set balance point"

// ---------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------
enum State : uint8_t { ST_IDLE = 0, ST_ARMED = 1, ST_BALANCING = 2, ST_FAULT = 3 };
enum Fault : uint8_t { F_NONE = 0, F_FALL = 1, F_RUNAWAY = 2, F_IMU = 3 };
enum Mode : uint8_t { MODE_MANUAL = 0, MODE_BALANCE = 1 };
enum Event : uint8_t { EV_NONE = 0, EV_ARMED, EV_ENGAGED, EV_DISARMED, EV_FALL, EV_RUNAWAY, EV_IMU };

// Obstacle avoidance settings (Bridge "set_param"); read by the balance thread.
float avoidEnabled = 1.0f;  // 0/1
float avoidMm = 300.0f;     // stop distance; forward motion is scaled to 0 at this distance
const float AVOID_MIN_MM = 100.0f;  // below this the robot could not stop in time (and 30 mm is the sensor floor)
const float AVOID_MAX_MM = 800.0f;  // thr*1.5 slow-down band must stay inside the 1200 mm reliable range

jb::Params P;  // live parameters; written by the Bridge thread, read by the balance thread

struct ParamEntry {
  const char* name;
  float* value;
  float minV, maxV;
};

// Single source of truth for names + limits on the MCU side. python/main.py holds the
// matching UI metadata; names must stay identical.
const ParamEntry PARAM_TABLE[] = {
  {"angle_kp",     &P.angle_kp,     100.0f,  30000.0f},
  {"angle_kd",     &P.angle_kd,     0.0f,    3000.0f},
  {"speed_kp",     &P.speed_kp,     0.0f,    5.0f},
  {"speed_ki",     &P.speed_ki,     0.0f,    5.0f},
  {"max_lean",     &P.max_lean,     0.5f,    15.0f},
  {"trim",         &P.trim,         -30.0f,  30.0f},
  {"wheel_mm",     &P.wheel_mm,     30.0f,   250.0f},
  {"cf_tau",       &P.cf_tau,       0.1f,    5.0f},
  {"vmax",         &P.vmax,         1000.0f, (float)MANUAL_MAX_STEPS_PER_SEC},
  {"amax",         &P.amax,         2000.0f, 400000.0f},
  {"fall_deg",     &P.fall_deg,     10.0f,   60.0f},
  {"arm_deg",      &P.arm_deg,      0.5f,    10.0f},
  {"fwd_max",      &P.fwd_max,      0.0f,    (float)MANUAL_MAX_STEPS_PER_SEC},
  {"steer_max",    &P.steer_max,    0.0f,    (float)MANUAL_MAX_STEPS_PER_SEC},
  {"ramp",         &P.ramp,         500.0f,  200000.0f},
  {"manual_ramp",  &P.manual_ramp,  2000.0f, 400000.0f},
  {"axis",         &P.axis,         0.0f,    1.0f},
  {"invert",       &P.invert,       0.0f,    1.0f},
  {"invert_left",  &P.invert_left,  0.0f,    1.0f},
  {"invert_right", &P.invert_right, 0.0f,    1.0f},
  {"avoid_en",     &avoidEnabled,   0.0f,    1.0f},
  {"avoid_mm",     &avoidMm,        AVOID_MIN_MM, AVOID_MAX_MM},
};
const size_t PARAM_COUNT = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

volatile uint32_t paramWrites = 0;  // lets Python detect an MCU reboot (count back to 0)
volatile bool filterResetRequested = false;

volatile uint8_t state = ST_IDLE;
volatile uint8_t mode = MODE_MANUAL;  // boots in manual: nothing balances until the operator asks
volatile uint8_t faultCode = F_NONE;
volatile uint8_t pendingEvent = EV_NONE;
volatile bool imuReady = false;
volatile bool tofReady = false;      // sensor initialised at least once
volatile int tofMm = -1;             // filtered distance; -1 = no target in range
volatile unsigned long tofStampMs = 0;
volatile bool imuCalibrated = false;  // gates every drive/balance command until "calibrate_imu" ran

float gyroBias[3] = {0, 0, 0};  // deg/s, from calibration

// Commands from the Bridge thread (single-word volatile writes are atomic on Cortex-M)
volatile float cmdFwd = 0.0f, cmdSteer = 0.0f;
volatile unsigned long cmdStampMs = 0;
volatile long manualL = 0, manualR = 0;  // plain drive mode (IDLE only), steps/s
volatile unsigned long manualStampMs = 0;
volatile bool disarmRequested = false;
volatile bool armRequested = false;

// Blocking-request accumulator shared between the Bridge thread (waits) and balance thread (fills)
enum AccumMode : uint8_t { AC_NONE = 0, AC_CALIB = 1, AC_TRIM = 2 };
volatile uint8_t accumMode = AC_NONE;
volatile bool accumDone = false;
int accumN = 0;
float accumG[3] = {0, 0, 0}, accumG2[3] = {0, 0, 0}, accumPitch = 0;
float accumResultTrim = 0;
bool accumResultOk = false;

// Telemetry snapshot: written by the balance thread, read by loop() (seqlock: retry if odd/changed)
struct Snapshot {
  uint8_t state, fault, mode;
  float pitch, rate, lean, v, accel;
  float roll, gx, gy, gz, ax, ay, az, tempC;
  uint32_t loopUsMax, overruns;
};
Snapshot snap;
volatile uint32_t snapSeq = 0;

// ---------------------------------------------------------------------------------------
// Balance thread
// ---------------------------------------------------------------------------------------
namespace {
jb::AttitudeFilter filt;
jb::Balancer balancer;
float uprightS = 0.0f;
int imuFails = 0;
uint32_t lastUs = 0;
uint32_t loopUsMax = 0, overruns = 0;
float lastRate = 0.0f;
float manV[2] = {0.0f, 0.0f};  // slewed manual-mode wheel speeds, steps/s
}  // namespace

static void setMotorsEnabled(bool on) {
  digitalWrite(MOTOR_EN_PIN, on ? LOW : HIGH);  // active low
}

static void enterFault(uint8_t code, uint8_t event) {
  stepgen::setSpeeds(0, 0);
  setMotorsEnabled(false);
  faultCode = code;
  state = ST_FAULT;
  pendingEvent = event;
}

static void goIdle() {
  manV[0] = manV[1] = 0.0f;
  balancer.reset();
  uprightS = 0.0f;
  cmdFwd = cmdSteer = 0.0f;
  stepgen::setSpeeds(0, 0);
  setMotorsEnabled(true);  // hold position like the original tele-op mode did
  faultCode = F_NONE;
  state = ST_IDLE;
}

static void publishSnapshot(const ImuSample& s, float pitch, float rate) {
  Snapshot n;
  n.state = state;
  n.fault = faultCode;
  n.mode = mode;
  n.pitch = pitch - P.trim;  // deg from the balance point: what the dashboard scope shows
  n.rate = rate;
  n.lean = balancer.lean_target;
  n.v = balancer.v;
  n.accel = balancer.accel;
  n.roll = atan2f(-s.ax, s.az) * jb::kRadToDeg;  // coarse, for the tilt bubble only
  n.gx = s.gx; n.gy = s.gy; n.gz = s.gz;
  n.ax = s.ax; n.ay = s.ay; n.az = s.az;
  n.tempC = s.tempC;
  n.loopUsMax = loopUsMax;
  n.overruns = overruns;
  snapSeq = snapSeq + 1;  // odd = write in progress
  __sync_synchronize();
  snap = n;
  __sync_synchronize();
  snapSeq = snapSeq + 1;
}

// Forward-speed multiplier 0..1 from the ToF sensor. Only ever reduces FORWARD motion; reverse and
// turning are never restricted, so the operator can always back away. No sensor at all -> 1 (the robot
// is usable without one); a sensor that was working but went silent -> 0 (fail safe).
static float avoidFactor() {
  if (avoidEnabled < 0.5f || !tofReady) return 1.0f;
  if (millis() - tofStampMs > TOF_STALE_MS) return 0.0f;
  int mm = tofMm;
  if (mm < 0) return 1.0f;  // nothing in range
  float stop = avoidMm;
  float slow = stop * (1.0f + AVOID_SLOW_BAND);
  if (mm <= stop) return 0.0f;
  if (mm >= slow) return 1.0f;
  return (mm - stop) / (slow - stop);
}

// Manual mode: raw wheel speeds from the joystick, slew-limited so a sudden stick jump can't
// exceed what the steppers can accelerate (which would skip steps). Times out to zero.
static void runManual(const jb::Params& p, float dt) {
  float tl = (float)manualL, tr = (float)manualR;
  if (millis() - manualStampMs > COMMAND_TIMEOUT_MS) tl = tr = 0.0f;
  float fwdPart = 0.5f * (tl + tr), turnPart = 0.5f * (tl - tr);
  if (fwdPart > 0.0f) {  // obstacle ahead: scale the forward component only, keep the turn
    fwdPart *= avoidFactor();
    tl = fwdPart + turnPart;
    tr = fwdPart - turnPart;
  }
  manV[0] = jb::slew(manV[0], tl, p.manual_ramp * dt);
  manV[1] = jb::slew(manV[1], tr, p.manual_ramp * dt);
  stepgen::setSpeeds((int32_t)((p.invert_left >= 0.5f) ? -manV[0] : manV[0]),
                     (int32_t)((p.invert_right >= 0.5f) ? -manV[1] : manV[1]));
}

static void controlStep() {
  uint32_t t0 = micros();
  float dt = (uint32_t)(t0 - lastUs) * 1e-6f;
  lastUs = t0;
  if (dt < CONTROL_DT * 0.5f) dt = CONTROL_DT * 0.5f;
  if (dt > CONTROL_DT * 3.0f) dt = CONTROL_DT * 3.0f;

  jb::Params p = P;  // consistent copy for this cycle
  const int axis = (p.axis >= 0.5f) ? 1 : 0;
  const float sign = (p.invert >= 0.5f) ? -1.0f : 1.0f;

  if (filterResetRequested) {
    filt.reset();
    filterResetRequested = false;
  }

  // Disarm / fault-reset is handled before touching the IMU on purpose: a dead sensor must never
  // stop the operator from getting the robot out of FAULT.
  if (disarmRequested) {
    disarmRequested = false;
    goIdle();
    pendingEvent = EV_DISARMED;
  }

  ImuSample s;
  if (!imuReady || !imu.read(s)) {
    if (++imuFails >= IMU_FAIL_LIMIT && (state == ST_BALANCING || state == ST_ARMED)) {
      enterFault(F_IMU, EV_IMU);
    }
    if (mode == MODE_MANUAL && state == ST_IDLE) {
      runManual(p, dt);  // manual driving needs no IMU
    } else if (state != ST_FAULT && state != ST_BALANCING) {
      stepgen::setSpeeds(0, 0);
    }
    stepgen::feed();
    return;
  }
  imuFails = 0;

  float gyroAxis[3] = {s.gx, s.gy, s.gz};
  float rate = sign * (gyroAxis[axis] - gyroBias[axis]);
  float accAngle = sign * jb::accelPitchDeg(s.ax, s.ay, s.az, axis);
  float pitch = filt.update(rate, accAngle, dt, p.cf_tau);
  lastRate = rate;

  // ---- blocking requests from the Bridge thread: calibration and balance-point capture ----
  if (accumMode != AC_NONE && !accumDone) {
    if (accumMode == AC_CALIB) {
      for (int i = 0; i < 3; ++i) {
        accumG[i] += gyroAxis[i];
        accumG2[i] += gyroAxis[i] * gyroAxis[i];
      }
      if (++accumN >= CALIB_SAMPLES) {
        bool still = true;
        float mean[3];
        for (int i = 0; i < 3; ++i) {
          mean[i] = accumG[i] / accumN;
          float var = accumG2[i] / accumN - mean[i] * mean[i];
          if (var < 0) var = 0;
          if (sqrtf(var) > CALIB_MAX_STDDEV_DPS) still = false;
        }
        if (still) {
          for (int i = 0; i < 3; ++i) gyroBias[i] = mean[i];
          filt.reset();
        }
        accumResultOk = still;
        accumDone = true;
      }
    } else {  // AC_TRIM
      accumPitch += pitch;
      if (++accumN >= TRIM_SAMPLES) {
        accumResultTrim = accumPitch / accumN;
        accumResultOk = true;
        accumDone = true;
      }
    }
  }

  // ---- state machine ----
  if (armRequested) {
    armRequested = false;
    if (state == ST_IDLE && imuCalibrated && mode == MODE_BALANCE) {
      balancer.reset();
      uprightS = 0.0f;
      loopUsMax = 0;  // worst-case loop time is reported "since arming"
      overruns = 0;
      state = ST_ARMED;
      pendingEvent = EV_ARMED;
    }
  }

  float fromBalance = pitch - p.trim;  // deg away from the balance point, + = leaning forward
  unsigned long nowMs = millis();

  switch (state) {
    case ST_IDLE: {
      if (mode == MODE_MANUAL) {
        runManual(p, dt);
      } else {
        manV[0] = manV[1] = 0.0f;
        stepgen::setSpeeds(0, 0);  // balance mode, not armed: wheels hold still
      }
      break;
    }
    case ST_ARMED: {
      stepgen::setSpeeds(0, 0);
      if (fabsf(fromBalance) < p.arm_deg && fabsf(rate) < ARM_MAX_RATE_DPS) {
        uprightS += dt;
      } else {
        uprightS = 0.0f;
      }
      if (uprightS >= ARM_HOLD_S) {
        balancer.reset();
        cmdFwd = cmdSteer = 0.0f;
        state = ST_BALANCING;
        pendingEvent = EV_ENGAGED;
      }
      break;
    }
    case ST_BALANCING: {
      if (fabsf(fromBalance) > p.fall_deg) {
        enterFault(F_FALL, EV_FALL);
        break;
      }
      float fwd = cmdFwd, steer = cmdSteer;
      if (nowMs - cmdStampMs > COMMAND_TIMEOUT_MS) fwd = steer = 0.0f;  // stay upright, stop moving
      if (fwd > 0.0f) fwd *= avoidFactor();  // the speed loop brakes the robot back to standstill
      float vL, vR;
      balancer.update(p, pitch, rate, dt, fwd, steer, &vL, &vR);
      if (balancer.sat_time > RUNAWAY_SAT_S) {
        enterFault(F_RUNAWAY, EV_RUNAWAY);
        break;
      }
      stepgen::setSpeeds((int32_t)((p.invert_left >= 0.5f) ? -vL : vL),
                         (int32_t)((p.invert_right >= 0.5f) ? -vR : vR));
      break;
    }
    case ST_FAULT:
    default:
      stepgen::setSpeeds(0, 0);
      break;
  }

  stepgen::feed();
  publishSnapshot(s, pitch, rate);

  uint32_t used = (uint32_t)(micros() - t0);
  if (used > loopUsMax) loopUsMax = used;
}

static void controlEntry(void*, void*, void*) {
  uint32_t next = micros();
  for (;;) {
    controlStep();
    next += CONTROL_PERIOD_US;
    int32_t remain = (int32_t)(next - micros());
    if (remain < 0) {  // overran the 5 ms budget: resynchronise instead of bursting
      overruns++;
      next = micros();
      remain = 0;
    }
    if (remain > 0) k_usleep(remain);
  }
}

static bool useLoopFallback = false;

// ---------------------------------------------------------------------------------------
// Bridge API (called from Python)
// ---------------------------------------------------------------------------------------
static long clampL(long v, long lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

// Plain tele-op drive (kept from the original app): independent wheel speeds in steps/s.
// Only honoured in IDLE -- while armed/balancing the controller owns the wheels.
void drive(long leftStepsPerSec, long rightStepsPerSec) {
  if (mode != MODE_MANUAL || state != ST_IDLE) return;  // raw drive is manual-mode only; no IMU needed
  manualL = clampL(leftStepsPerSec, MANUAL_MAX_STEPS_PER_SEC);
  manualR = clampL(rightStepsPerSec, MANUAL_MAX_STEPS_PER_SEC);
  manualStampMs = millis();
}

void stopDriving() {
  manualL = manualR = 0;
  manualStampMs = millis();
  cmdFwd = cmdSteer = 0.0f;
  cmdStampMs = millis();
}

// Blocks up to `timeoutMs` for the balance thread to finish an accumulation. Runs on the Bridge
// thread, so it never stalls balancing.
static bool runAccum(uint8_t mode, unsigned long timeoutMs) {
  if (!imuReady || accumMode != AC_NONE) return false;
  accumN = 0;
  accumPitch = 0;
  for (int i = 0; i < 3; ++i) accumG[i] = accumG2[i] = 0;
  accumResultOk = false;
  accumDone = false;
  accumMode = mode;
  unsigned long start = millis();
  while (!accumDone && millis() - start < timeoutMs) {
    delay(10);
  }
  bool ok = accumDone && accumResultOk;
  accumMode = AC_NONE;
  accumDone = false;
  return ok;
}

// "calibrate_imu": average the gyro for 1.5 s to remove its zero-rate offset. The robot must be
// stationary -- calibration is REJECTED (returns false) if the gyro noise shows it was moving.
// The accelerometer is deliberately not zeroed: the balance point is captured separately with
// "capture_trim", which also absorbs mounting tilt and centre-of-gravity offset.
bool calibrateImu() {
  if (state == ST_BALANCING || state == ST_ARMED) return false;
  Serial.println(F("Calibrating gyro -- keep the robot still"));
  bool ok = runAccum(AC_CALIB, 4000);
  if (ok) imuCalibrated = true;
  Serial.println(ok ? F("Gyro calibration done") : F("Gyro calibration FAILED (moving or IMU error)"));
  return ok;
}

bool imuCalibrationStatus() { return imuCalibrated; }

// "capture_trim": hold the robot at its balance point (by hand) and call this; returns the
// averaged pitch in degrees, or -999 on failure. Python stores it as the `trim` parameter.
float captureTrim() {
  if (state == ST_BALANCING || !imuCalibrated) return -999.0f;
  bool ok = runAccum(AC_TRIM, 3000);
  return ok ? accumResultTrim : -999.0f;
}

// "balance_arm": true = arm (engages automatically once held upright), false = disarm / clear
// a fault. Returns the resulting state, or -1 if refused (not calibrated / not idle).
int balanceArm(bool on) {
  if (on) {
    if (mode != MODE_BALANCE || !imuReady || !imuCalibrated || state != ST_IDLE) return -1;
    armRequested = true;
    return ST_ARMED;
  }
  disarmRequested = true;
  return ST_IDLE;
}

// "set_mode": 0 = manual (direct wheel control, no balancing), 1 = self-balancing.
// Leaving balance mode disarms first (and clears any fault). Returns the new mode, or -1 if invalid.
int setMode(int m) {
  if (m != MODE_MANUAL && m != MODE_BALANCE) return -1;
  manualL = manualR = 0;
  manualStampMs = millis();
  cmdFwd = cmdSteer = 0.0f;
  if (m == MODE_MANUAL) disarmRequested = true;
  mode = (uint8_t)m;
  return m;
}

// "balance_cmd": normalised joystick, fwd/steer in -1..1 (+fwd = forward, +steer = turn right).
void balanceCmd(float fwd, float steer) {
  cmdFwd = fwd;
  cmdSteer = steer;
  cmdStampMs = millis();
}

// "set_param": name must match PARAM_TABLE. Returns false for unknown names / out-of-range values.
bool setParam(String name, float value) {
  for (size_t i = 0; i < PARAM_COUNT; ++i) {
    if (name == PARAM_TABLE[i].name) {
      if (isnan(value) || value < PARAM_TABLE[i].minV || value > PARAM_TABLE[i].maxV) return false;
      *PARAM_TABLE[i].value = value;
      if (name == "axis" || name == "invert") filterResetRequested = true;
      paramWrites = paramWrites + 1;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------------
// ToF sensor (runs on loop(); the balance thread only reads tofMm / tofStampMs)
// ---------------------------------------------------------------------------------------
enum TofStatus : uint8_t { TOF_MISSING = 0, TOF_OK = 1, TOF_NO_TARGET = 2, TOF_TIMEOUT = 3 };
uint8_t tofStatus = TOF_MISSING;
unsigned long tofLastInitMs = 0;

static bool tofInit() {
  tofLastInitMs = millis();
  if (!tof.init()) return false;
  tof.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
  tof.startContinuous(TOF_PERIOD_MS);
  tofReady = true;
  Serial.println(F("VL53L0X ready on Wire2 (A4/A5)"));
  return true;
}

static int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { b = c; }
  return a > b ? a : b;
}

// Reads at most one sample per call (blocks <= ~50 ms waiting for it, which is fine on loop()).
static void tofPoll() {
  static int h[3] = {-1, -1, -1};
  if (!tofReady) {
    tofStatus = TOF_MISSING;
    if (millis() - tofLastInitMs >= TOF_RETRY_MS || tofLastInitMs == 0) tofInit();
    return;
  }
  uint16_t raw = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) {
    tofStatus = TOF_TIMEOUT;  // tofStampMs is not refreshed, so avoidFactor() goes fail-safe
    return;
  }
  // No target: map to a huge value so the median treats it as "far" and the stop check stays simple.
  int mm = (raw >= TOF_OUT_OF_RANGE_MM) ? 9999 : (int)raw;
  h[0] = h[1]; h[1] = h[2]; h[2] = mm;
  int m = (h[0] < 0) ? mm : median3(h[0], h[1], h[2]);  // 3-sample median rejects single-frame spikes
  if (m > TOF_MAX_RELIABLE_MM) {
    tofMm = -1;
    tofStatus = TOF_NO_TARGET;
  } else {
    tofMm = m;
    tofStatus = TOF_OK;
  }
  tofStampMs = millis();
}

// ---------------------------------------------------------------------------------------
// setup / loop (loop = telemetry + event log only)
// ---------------------------------------------------------------------------------------
void setup() {
  pinMode(MOTOR_EN_PIN, OUTPUT);
  setMotorsEnabled(false);  // stay de-energised until the IMU is up
  stepgen::begin(LEFT_STEP_PIN, LEFT_DIR_PIN, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

  Bridge.begin();
  Bridge.provide("drive", drive);
  Bridge.provide("stop", stopDriving);
  Bridge.provide("calibrate_imu", calibrateImu);
  Bridge.provide("imu_status", imuCalibrationStatus);
  Bridge.provide("capture_trim", captureTrim);
  Bridge.provide("balance_arm", balanceArm);
  Bridge.provide("balance_cmd", balanceCmd);
  Bridge.provide("set_mode", setMode);
  Bridge.provide("set_param", setParam);

  Serial.begin(115200);
  Wire2.begin();
  tof.setBus(&Wire2);
  tof.setTimeout(100);
  imuReady = imu.begin();
  if (imuReady) {
    Serial.print(F("MPU6050 ready (WHO_AM_I=0x"));
    Serial.print(imu.whoAmI(), HEX);
    Serial.println(F(") -- calibrate from the dashboard"));
  } else {
    Serial.println(F("MPU6050 NOT found on Wire1 -- balancing disabled"));
  }

  lastUs = micros();
  goIdle();

  k_thread_stack_t* stack = k_thread_stack_alloc(CONTROL_STACK_SIZE, 0);
  static struct k_thread controlThread;
  if (stack) {
    k_tid_t tid = k_thread_create(&controlThread, stack, CONTROL_STACK_SIZE, controlEntry, NULL,
                                  NULL, NULL, CONTROL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(tid, "balance");
  } else {
    useLoopFallback = true;
    Serial.println(F("WARNING: no stack for the balance thread -- running degraded in loop()"));
  }
}

static const char* eventText(uint8_t e) {
  switch (e) {
    case EV_ARMED: return "ARMED -- hold the robot upright at its balance point";
    case EV_ENGAGED: return "BALANCING engaged";
    case EV_DISARMED: return "Disarmed";
    case EV_FALL: return "FAULT: tilt beyond fall_deg -- motors off";
    case EV_RUNAWAY: return "FAULT: wheel speed pinned at vmax (runaway) -- motors off";
    case EV_IMU: return "FAULT: IMU read failed -- motors off";
    default: return "";
  }
}

void loop() {
  static unsigned long lastBalanceMs = 0, lastImuMs = 0;

  if (useLoopFallback) {
    static uint32_t nextUs = micros();
    if ((int32_t)(micros() - nextUs) >= 0) {
      nextUs += CONTROL_PERIOD_US;
      controlStep();
    }
  }

  uint8_t ev = pendingEvent;
  if (ev != EV_NONE) {
    pendingEvent = EV_NONE;
    Serial.println(eventText(ev));
  }

  tofPoll();

  unsigned long now = millis();
  static unsigned long lastTofMs = 0;
  if (now - lastTofMs >= TELEMETRY_TOF_MS) {
    lastTofMs = now;
    // mm (-1 = no target), status, forward-speed factor in percent
    Bridge.notify("tof", (int)tofMm, (int)tofStatus, (int)(avoidFactor() * 100.0f + 0.5f));
  }
  bool dueBalance = now - lastBalanceMs >= TELEMETRY_BALANCE_MS;
  bool dueImu = now - lastImuMs >= TELEMETRY_IMU_MS;
  if (dueBalance || dueImu) {
    Snapshot c;
    uint32_t seq;
    do {  // seqlock read: retry while the balance thread is mid-write
      seq = snapSeq;
      __sync_synchronize();
      c = snap;
      __sync_synchronize();
    } while ((seq & 1u) || seq != snapSeq);

    if (seq != 0) {  // at least one control cycle has run
      if (dueBalance) {
        lastBalanceMs = now;
        Bridge.notify("balance", (int)c.state, (int)c.fault, c.pitch, c.rate, c.lean, c.v, c.accel,
                      (long)c.loopUsMax, (long)c.overruns, (long)paramWrites, (int)c.mode);
      }
      if (dueImu) {
        lastImuMs = now;
        // Same 9-value layout as the original "imu" stream: angleX/Y (deg), gyro XYZ (deg/s),
        // accel XYZ (g), temp (C). angleX is now the filtered balance pitch.
        Bridge.notify("imu", c.pitch, c.roll, c.gx, c.gy, c.gz, c.ax, c.ay, c.az, c.tempC);
      }
    }
  }
  delay(5);
}
