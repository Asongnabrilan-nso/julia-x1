# 🤖 julia-X2

Tele-operated self-balancing robot on Arduino UNO Q + CNC shield.

## Step 1 — motor control interface (this commit)

- `sketch/sketch.ino`: drives the two CNC-shield steppers (left STEP=D2/DIR=D5,
  right STEP=D3/DIR=D6) via `AccelStepper`, exposed to Python as
  `Bridge.provide("drive", leftStepsPerSec, rightStepsPerSec)` and
  `Bridge.provide("stop", ...)`. Auto-stops if no command arrives for 500 ms
  (WiFi teleop safety).
- Motors: hybrid NEMA17, 1.8°/step (200 full steps/rev), 1.2 A/phase. Drivers:
  A4988, run in 1/16 microstepping (3200 microsteps/rev) — set the CNC
  shield's MS1/MS2/MS3 jumpers and each A4988's current-limit trimpot in
  hardware; the sketch only converts speed commands, it can't set either.
- `python/main.py`: WebUI joystick backend. Mixes joystick `{x, y}` into
  left/right wheel speeds (arcade drive) and forwards them to the sketch.
- `assets/`: dark joystick web UI (drag to drive, release or press STOP to stop).

## Step 2 — faster drive + IMU telemetry (this commit)

- `sketch/sketch.ino`: wheel speed clamp raised from 1.0 to 5.0 rev/s (300 RPM,
  16000 microsteps/sec at 1/16 microstepping) — re-check A4988 pull-out torque
  on the real robot before raising it further.
- MPU6050 IMU wired to the Qwiic connector on `Wire1`, read via `MPU6050_light`
  (same library/version already proven on this board in the sibling `imu` app).
  Streams `angleX/Y` (complementary-filtered tilt, deg), `gyroX/Y/Z` (deg/s),
  `accX/Y/Z` (g) and temperature to Python at ~20 Hz via `Bridge.notify("imu", ...)`.
  **Axis note:** `angleX`/`angleY` are raw tilt angles from the sensor's own
  frame — which one is the fore/aft "pitch" that matters for balancing depends
  on how the MPU6050 is physically mounted on the chassis. Verify against the
  real mount and relabel ("Tilt X"/"Tilt Y" in the dashboard) if swapped.
- `python/main.py`: receives the IMU stream via `Bridge.provide("imu", ...)`
  and rebroadcasts it to the web UI as an `imu` socket message.
- `assets/`: IMU panel pinned to the top-right of the dashboard — tilt-bubble
  indicator plus numeric gyro/accel/temperature readouts, live-updated from
  the IMU stream.

## Step 3 — dashboard-confirmed IMU calibration gate (this commit)

- `sketch/sketch.ino`: no more auto-calibration at boot. `mpu.begin()` only
  detects the sensor; offsets are computed exclusively by `calibrateImu()`,
  exposed as `Bridge.provide("calibrate_imu", ...)` and triggered from the
  dashboard's **Calibrate** button. **`drive()` now refuses every command
  until calibration has completed** (`imuCalibrated` flag) — the robot cannot
  be driven, including at first boot, before the IMU has been calibrated with
  it sitting still and level. A second RPC, `Bridge.provide("imu_status", ...)`,
  lets the dashboard resync its button/lock state if the Python app restarts
  while the MCU stays up.
- `python/main.py`: `on_calibrate_imu` runs the ~1 s blocking `Bridge.call
  ("calibrate_imu")` on a background thread (so it doesn't stall the socket
  server) and broadcasts `calibration_status` (`idle` / `calibrating` /
  `calibrated` / `failed`) to all dashboard clients. `on_joystick` also checks
  this status and drops input before it ever reaches the sketch, mirroring the
  firmware-side gate.
- `assets/`: the IMU panel now has a **Calibrate** button and status readout;
  the joystick is visually locked (dimmed, non-interactive) and a banner
  reminds the user to calibrate — both clear once `calibration_status` reports
  `calibrated`. Recalibrating any time (e.g. after moving the robot) just
  means pressing the button again.

## Step 4 — self-balancing (this commit)

Two control modes, switched from the dashboard (boots in **Manual**): **Manual** = direct wheel
control without balancing (no IMU needed, slew-limited), **Self-balancing** = the robot balances
and can be driven around while staying upright.

The robot now balances itself. **Full setup, tuning and run guide: [`docs/BALANCING.md`](docs/BALANCING.md).**

- `sketch/balance.h`: hardware-independent controller — complementary attitude filter plus a
  cascade of an **angle PD loop** (P on angle, D on the raw gyro rate, output = wheel
  acceleration, integrated to speed) inside a **speed PI loop** (holds position, turns joystick
  speed into lean angle). Gains rescale automatically with the configured wheel diameter.
- `sketch/step_gen.h`: 10 kHz timer-interrupt STEP/DIR generator with a 30 ms dead-man
  watchdog. Replaces `AccelStepper.runSpeed()` polling, which stalled during I²C/Bridge
  traffic. `AccelStepper` and `MPU6050_light` are no longer used.
- `sketch/mpu6050.h`: raw-register IMU driver (400 kHz, on-chip low-pass ~94 Hz).
- `sketch/sketch.ino`: state machine (Idle -> Armed -> Balancing, Fault), a 200 Hz `balance`
  thread that never touches the Bridge (balance survives a dead WiFi/Python app), fall /
  runaway / IMU-failure protection, and the Bridge API (`set_param`, `balance_arm`,
  `balance_cmd`, `capture_trim`, ...).
- Calibration is now **gyro-bias only**; the balance point is captured separately
  ("Set balance point"), because a balancing robot at rest is not level.
- `python/main.py`: routes the joystick to the balancer, live-tunes every parameter over the
  Bridge, persists tuning to `data/tuning.json` and re-sends it after any MCU or app restart.
- `assets/`: single-page dashboard that never scrolls (mode switch + **Start/Stop balancing** at the
  top-left, joystick in the centre, IMU/speed/motors/STOP on the right), and a **Tune** overlay with a
  live scope (pitch / lean command / wheel speed), calibration buttons and every parameter.
  The joystick now re-sends while held (it used to stop after 500 ms of stillness), and the
  layout adapts to portrait, landscape and desktop.
- `tools/sim/balance_sim.py`: pure-Python simulator of the same controller against a wheeled
  inverted-pendulum plant with noise, delay and trim error; used to pick and stress-test the
  default gains (`python3 tools/sim/balance_sim.py`).

**Verification status:** firmware compiles and links for the UNO Q with no warnings; Python
backend logic and the dashboard's JavaScript are exercised by tests against a fake Bridge and a
headless browser; the controller is validated in simulation. **Not yet run on the physical
robot** — do the guided bring-up in `docs/BALANCING.md` first.
