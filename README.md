# 🤖 julia-X2

Tele-operated self-balancing robot on Arduino UNO Q + CNC shield.

> **Handheld controller:** the Julia-x2 Joy WiFi joystick (XIAO ESP32C3, PlatformIO) lives in
> [`controller/`](controller/README.md). Its README holds its own changelog, setup and test steps.

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

## Step 5 — handheld controller support (Julia-x2 Joy)

The Julia-x2 Joy handheld ([`controller/`](controller/README.md)) connects to the dashboard's
Socket.IO server on port 7000 as one more client, sending the same `joystick` / `stop` messages
as the web joystick. Robot-side changes in `python/main.py`:

- **Latest-wins joystick.** `on_joystick` no longer drops a command that arrives inside the
  20 Hz rate window. It stores the newest `{x, y}`, and a `_joystick_worker` thread applies it at
  most 20×/s. When a laggy WiFi link delivers several packets at once, the robot now acts on the
  *most recent* one instead of an older one. A `stop` cancels any queued move, and a lock ensures
  it is always applied after an in-flight move. The web dashboard benefits too.
- **Latency echo.** New `joy_ping` → `joy_pong` handler (reply only to the sender), so the
  handheld can show the command round-trip time.

**After updating, restart the app in App Lab.** If driving feels laggy, check the UNO Q's WiFi
power saving: `iw dev wlan0 get power_save`, and turn it off as described in the controller
README, section "If driving feels laggy".

## Step 6 — TFT face display (personality)

Julia now has a face: animated robot eyes ([FluxGarage RoboEyes](https://github.com/FluxGarage/RoboEyes))
on a [2.8" ILI9341 320×240 SPI TFT](https://done.land/components/humaninterface/display/tft/ili9341/2.8inch320x240/).
The eyes react to what the robot is doing, and the operator can set a mood or play an animation from
the dashboard. **Only the display is used.** The module's SD card and touch controller stay unconnected.

### Wiring

The UNO Q's hardware SPI is on **D11 (MOSI), D12 (MISO), D13 (SCK)**. The STM32 runs at 3.3 V
logic, which is what the ILI9341 needs, so no level shifter or series resistors are required. The
CNC shield covers the UNO Q headers, so every signal is taken from a spare shield header:

| TFT pin    | UNO Q pin | Where to connect with the CNC shield fitted | Notes |
|------------|-----------|---------------------------------------------|-------|
| VCC        | **5V**    | any 5V pin                                  | leave jumper **J1 open** (onboard 3.3 V regulator in use) |
| GND        | GND       | any GND pin                                 | |
| CS         | **D10**   | Y-axis endstop header (Y-/Y+), signal pin   | |
| RESET      | **D7**    | DIR pin of the **empty Z-axis driver socket** | free only because no Z driver is fitted |
| DC         | **D9**    | X-axis endstop header (X-/X+), signal pin   | |
| SDI (MOSI) | **D11**   | Z-axis endstop header (Z-/Z+), signal pin   | hardware SPI |
| SCK        | **D13**   | SpnDir header                               | hardware SPI |
| LED        | **3.3V**  | UNO Q 3.3V pin                              | backlight always on, full brightness |
| SDO (MISO) | **D12**   | SpnEn header                                | optional for drawing; needed for the boot self-test |
| T_* / SD_* | —         | not connected                               | touch and SD card are not used |

```
   2.8" ILI9341 TFT                         Arduino UNO Q  (+ CNC shield V3)
  ┌───────────────┐
  │ VCC  ●────────┼──────────────────────── 5V
  │ GND  ●────────┼──────────────────────── GND
  │ CS   ●────────┼──────────────────────── D10   (Y endstop header)
  │ RESET●────────┼──────────────────────── D7    (Z driver socket, DIR pin)
  │ DC   ●────────┼──────────────────────── D9    (X endstop header)
  │ SDI  ●────────┼──────────────────────── D11   (Z endstop header)   MOSI
  │ SCK  ●────────┼──────────────────────── D13   (SpnDir header)      SCK
  │ LED  ●────────┼──────────────────────── 3.3V
  │ SDO  ●────────┼──────────────────────── D12   (SpnEn header)       MISO
  │ T_CLK T_CS T_DIN T_DO T_IRQ  ○ ○ ○ ○ ○  not connected (touch)
  │ SD_CS SD_MOSI SD_MISO SD_CLK ○ ○ ○ ○    not connected (SD card)
  └───────────────┘
```

**Changes from the original wiring table:**

- **TFT_RST moved from D8 to D7.** D8 is the CNC shield's stepper **ENABLE** line, shared by *all*
  driver sockets including X and Y. The firmware uses it to switch the motors off after a fall, so the
  display must never drive it. D7 is the Z-axis DIR line and is free only while **no Z driver is
  fitted**. Never fit a Z driver while the TFT is connected.
- **LED goes to 3.3 V, not 5 V.** This module has no backlight series resistor.
- **VCC = 5 V with J1 open.** If you power it from 3.3 V instead, close J1. Otherwise the regulator
  drops the supply below 3 V and the backlight gets dim.
- **For the future (not wired now):** SD_CS on D4 is fine (Z STEP, free with no Z driver). **T_CS on D5
  is not**: D5 is the left wheel's DIR pin. Use A1 (the shield's "Hold" header) for touch instead.

> **Check your shield before wiring.** Some CNC shield V3 clones swap the "Z limit" and "SpnEn" header
> pins (GRBL 0.9 layout). With the robot unpowered, use a multimeter in continuity mode to confirm that
> each header pin above really reaches the UNO Q pin listed.

### What the face does

| Robot situation                         | Eyes |
|-----------------------------------------|------|
| Idle, nobody driving                    | cyan, blinks, looks around on its own after 4 s |
| Driving                                 | looks into the turn, ahead when going forward, down when reversing |
| Someone/something in front (ToF sensor) | looks up; when the robot is standing still it laughs once to greet (at most every 8 s) |
| Driving forward into an obstacle        | confused shake (avoidance is holding it back) |
| Armed, waiting to be held upright       | amber, looks up |
| Balancing                               | mint, happy eyes; laughs once when balancing engages |
| Fault (fell over, runaway, IMU error)   | red-orange, tired, sweating, looks down |

**Dashboard → Face** overrides the mood (Neutral / Happy / Tired / Angry, each with its own colour)
until you select **Auto** again. **Play once** fires Blink, Laugh or Confused. The override is held by
the Python app and re-sent if the MCU restarts.

### Implementation

- `sketch/face.h`, `sketch/face.cpp`: the face runs in its own `face` thread at the same priority as
  `loop()`, far below the 200 Hz balance thread. RoboEyes draws into a 1-bit 320×240 canvas in RAM.
  Each frame is compared with what is already on the glass, and only the changed areas are sent.
  - **Why it streams in chunks:** on the UNO Q, SPI2 is interrupt-driven, with no DMA and no FIFO in
    this board's devicetree, so every byte costs a CPU interrupt. A full 16-bit frame (150 KB) sent in
    one go would starve the balance loop. The changed pixels go out in 256-byte bursts with a 0.5 ms
    sleep between bursts. A typical eye movement is about 6 KB per frame.
  - **Why the face is a `.cpp`:** RoboEyes defines short global macros (`N`, `E`, `S`, `W`, `ON`, `DEFAULT`, …),
    so it lives in its own `.cpp` to keep them out of `sketch.ino`.
  - **The D10 chip-select fix:** D10 doubles as SPI2's hardware NSS pin, and `SPI.begin()` re-claims it.
    A small SPI wrapper hands D10 back to GPIO so the Adafruit driver's chip-select works.
- `sketch/sketch.ino`: `loop()` passes the robot state, drive command and ToF reading to the face.
  New Bridge calls: `face_mood(int)` (0 = auto, 1–4 = neutral/happy/tired/angry) and
  `face_anim(int)` (1 = blink, 2 = laugh, 3 = confused).
- `sketch/sketch.yaml`: adds `Adafruit BusIO`, `Adafruit GFX Library`, `Adafruit ILI9341`,
  `FluxGarage RoboEyes`.
- `python/main.py`: `face_mood` / `face_anim` dashboard messages, forwarded to the MCU.
- `assets/`: **Face** button in the header opens the face sheet.

Settings at the top of `sketch/face.cpp`: `ROTATION` (1 or 3 if the face is upside down),
`SPI_HZ` (24 MHz), `MIRROR_GAZE` (flip if the eyes look the wrong way when turning), eye size and
colours, and the `CHUNK_BYTES` / `CHUNK_GAP_US` pair that limits how much CPU the display may take.

### Bring-up

1. Wire the display with the robot unpowered, then restart the app in App Lab.
2. Within a few seconds of boot the eyes should open in the centre of the screen. In the serial
   monitor (`arduino-app-cli monitor`), look for:
   `TFT face ready (ILI9341 on D10/D9/D7): power mode 0x94, self-diag 0xC0 ...`.
   0x94 / 0xC0 are the usual values. **0x00 or 0xFF** means nothing answered on MISO: check D12, or
   ignore it if you left MISO unconnected.
3. **Check that the display does not disturb balancing:** arm balancing, open **Tune** and watch
   **Loop** (µs). The counters reset on every arm. Loop must stay well under 5000 µs, and the overrun
   count must stay at 0 while the eyes move. If
   overruns appear, raise `CHUNK_GAP_US` or lower `CHUNK_BYTES` in `face.cpp`.

| Symptom | Fix |
|---------|-----|
| White screen | Display not initialised: check CS (D10), DC (D9), RESET (D7), SCK (D13), MOSI (D11), and that VCC is 5 V |
| Snow, wrong colours, shifted image | Lower `SPI_HZ` to 12 MHz (long jumper wires through the shield headers) |
| Face upside down | `ROTATION = 3` |
| Very dim | Backlight (LED) on 3.3 V? J1 open when VCC is 5 V? |

**Verification status:** compiled with the App Lab library profile for the UNO Q (`face.cpp` and
`sketch.ino` build without warnings; the remaining warnings come from inside the RoboEyes library).
The partial-redraw logic was checked against 3000 random frames in a Python mirror of the C++ code.
The Python handlers were tested against a fake Bridge, and the dashboard's Face sheet in headless
Chromium. **Not yet run on the real display or robot.** In particular, the SPI throughput and the
effect on the balance loop's timing are estimates until step 3 of the bring-up has been done.
