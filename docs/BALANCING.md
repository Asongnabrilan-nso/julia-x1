<!--
SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies

SPDX-License-Identifier: MPL-2.0
-->

# Self-balancing julia-X2 — setup, tuning and running

The robot balances itself with a fast cascaded controller that runs **entirely on the
STM32 (MCU) side**, inspired by the
[high-speed balancing robot](https://elexperiment.nl/2018/11/high-speed-balancing-robot-introduction/)
design ([source](https://gitlab.com/kloppertje/balancingrobot/-/tree/ps3control)). The Linux
side (the web dashboard) is only a remote control, a scope and a tuning panel: **if WiFi or
the Python app dies, the robot keeps standing.**

> **Honest status.** The firmware compiles and links for the UNO Q, the dashboard and Python
> logic are tested, and the controller is validated in simulation (see
> [Simulate first](#9-simulate-first)). It has **not yet been run on your physical robot** —
> your first balance attempt is the real test, which is why this guide walks you through
> hand-held, low-risk steps first.

---

## Contents

0. [The two control modes](#0-the-two-control-modes)
1. [How it works](#1-how-it-works)
2. [Safety](#2-safety)
3. [Hardware checklist (one time)](#3-hardware-checklist-one-time)
4. [Start the app](#4-start-the-app)
5. [First-time bring-up (10 minutes)](#5-first-time-bring-up)
6. [Your first balance](#6-your-first-balance)
7. [Tuning](#7-tuning)
8. [Driving, saving, faults](#8-driving-saving-faults)
9. [Simulate first](#9-simulate-first)
10. [Reference: parameters, Bridge API, files](#10-reference)
11. [Troubleshooting](#11-troubleshooting)

---

## 0. The two control modes

The dashboard has a switch at its top-left (see the [dashboard tour](#the-dashboard-at-a-glance)). The robot **always boots in Manual**,
so nothing can start balancing (and the wheels can't surprise you) until you ask for it.

| | **Manual** | **Self-balancing** |
|---|---|---|
| What it is | You drive the two wheels directly, like a normal rover. **No balancing.** | The robot balances itself and you drive it around while it stays upright. |
| Joystick | Up/down = both wheels forward/back; left/right = turn (arcade mixing). Always live. | Up/down = forward/back **speed**, left/right = **turning**. Live once balancing has started. |
| Needs the IMU / calibration | No — works even if the IMU is unplugged. | Yes — calibrate the gyro first. |
| Used for | Bring-up (motor directions), testing the drivers, driving it on a stand or as a rover, moving it around safely. | The real thing. |
| Acceleration | Wheel acceleration is limited by *Manual accel. limit* so sudden stick moves don't stall the motors. | Governed by the controller and *Max lean* / *Joystick ramp*. |
| Start / Stop button | Hidden. | **Start balancing** → *Armed* → *Balancing*; **Stop balancing** / **Reset fault**. |

Switching **Manual → Self-balancing** leaves the robot idle (wheels held still) until you press
*Start balancing*. Switching **Self-balancing → Manual** disarms the controller first (and clears
any fault), so if it was balancing **it will fall — catch it** (the dashboard asks you to confirm).
The **Max Speed** slider and the red **STOP** button work in both modes.

### The dashboard at a glance

Everything is on **one page that never scrolls** — on a phone (portrait or landscape) and on a
laptop. Only the **Tune** sheet, which opens on top of the dashboard, scrolls internally.

```
Landscape / desktop
+-----------------------------------------------------------+
| Julia X2  [Connected]  [action badge]              [Tune] |
+------------------+-------------------+--------------------+
| [Manual|Self-bal]|                   | IMU  tilt gyro acc |
| +--------------+ |                   | [Calibrate] status |
| | state  pitch | |     joystick      | Max Speed  [-|+]   |
| |[Start balanc]| |        ( )        | L ====   R ====    |
| +--------------+ |    direction      |                    |
|                  |                   | [ STOP ]           |
+------------------+-------------------+--------------------+

Portrait phone
+----------------------------------+
| Julia X2 [Connected] [badge] Tune|
+-----------------+----------------+
| [Manual|Self-b] | IMU            |
| state    pitch  | Max Speed      |
| [Start balanc.] | L / R  [STOP]  |
+-----------------+----------------+
|            joystick              |
|               ( )                |
+----------------------------------+
```

| Where | What |
|-------|------|
| **Top-left** | **Manual / Self-balancing** switch, and — in self-balancing mode — the balance panel: state (*Idle / Armed / Balancing / Fault*), live pitch in degrees from the balance point, and the **Start balancing** / **Stop balancing** / **Reset fault** button with a one-line hint. |
| **Header** | Connection status, the action badge (*Stopped*, *Driving forward*, *Balancing*, *FAULT*…) and the **Tune** button. |
| **Centre** (below the header on portrait) | The joystick, with a direction label under it. |
| **Right** | The **IMU** readout (tilt, gyro, accel, temperature) with the **Calibrate** button and its status; the **Max Speed** slider; the **L / R** wheel-speed gauges; the red **STOP** bar. |

In Manual mode the balance panel is hidden; in self-balancing mode a yellow banner appears until the
IMU is calibrated, and the joystick stays dimmed until balancing has started.

## 1. How it works

```
 IMU (MPU6050) @ 200 Hz
   gyro rate ─────────────────────────────┐
   accel ──► pitch ──► complementary ─────┤
                       filter (τ = 0.6 s) │
                                          ▼
 joystick ─► v_set ─► speed PI ─► lean ─► angle PD ─► wheel ─∫─► wheel speed ─► step pulses
                          ▲       target   (P on angle,  accel.            (10 kHz timer ISR
                          └── measured      D on gyro)                        + watchdog)
                              wheel speed
```

* **Inner loop (angle PD).** Error = `pitch − balance point − lean target`. Output is a wheel
  **acceleration**, which is integrated into wheel speed. This is the physically correct
  quantity to command: for an inverted pendulum, `θ̈ = g/L·θ − a/L`. It keeps the wheel speed
  continuous, so steppers do not skip steps. The **D term uses the raw gyro**, not a
  differentiated angle, so it is clean and fast.
* **Outer loop (speed PI).** Compares the wheel speed against what the joystick wants and
  outputs a **lean angle**. This is what holds the robot in place (no drift) and what makes
  it accelerate (lean forward) and brake (lean back). Its integral term also absorbs small
  errors in the balance point.
* **Step generation.** A 10 kHz timer interrupt produces the STEP/DIR pulses, so slow I²C
  reads or Bridge traffic can never jitter the wheels. A **dead-man watchdog** zeroes the
  wheels if the control loop stalls for 30 ms.
* **Threads.** Timer ISR > `balance` thread (200 Hz, priority 3) > `bridge` thread >
  `loop()` (telemetry only). The balance thread never touches the Bridge.

Why this structure works (and what the simulation checks) is in
[Simulate first](#9-simulate-first).

## 2. Safety

* A balancing robot is **unstable by design** — until you have tuned it, expect falls.
  **Always keep a hand near it** for the first runs, and work over a soft surface (carpet,
  foam) or a table with a lip.
* Keep fingers, hair and cables away from the wheels. Motors can accelerate very quickly.
* **Motors switch off automatically** when: tilt exceeds the *Fall limit* (30°), a wheel is
  pinned at its speed limit for 1 s (runaway), the IMU stops answering, or the control loop
  stalls. The dashboard shows the reason; press **Reset fault** to re-enable.
* Two ways to stop: **Stop balancing** (disarms; robot will fall — catch it) and the
  red **STOP** bar (zeroes the joystick command; the robot keeps balancing).
* Test with the **wheels off the ground** wherever this guide says so.
* Use a battery/PSU with a fuse or current limit. Never hot-plug stepper drivers.

## 3. Hardware checklist (one time)

The firmware **cannot** set any of these — they are physical settings.

| # | Check | Why |
|---|-------|-----|
| 1 | **A4988 microstepping jumpers**: MS1, MS2, MS3 all **HIGH** (jumpers fitted) under *both* drivers → 1/16 microstepping (3200 steps/rev). | The controller and all default speeds assume 3200 steps/rev. |
| 2 | **A4988 current limit** for 1.2 A/phase motors: `Vref = I_max × 8 × R_sense`. Measure Vref between the trimpot's metal top and GND with the board's logic powered and the motors not moving (adjust with a small screwdriver). `R_sense` is printed/known for your driver: 0.068 Ω → ≈ 0.65 V; 0.10 Ω → ≈ 0.96 V. | Too low: skipped steps and falls. Too high: hot drivers. Heatsinks + airflow recommended above ~1 A. |
| 3 | **Motor supply ≥ 12 V**, ideally 12–16 V (3S–4S Li-ion/LiPo). Higher voltage = more torque at speed. Add a **100 µF+ electrolytic** across the CNC-shield motor power terminals. | A4988 torque collapses at speed on low voltage; balancing needs fast corrections. |
| 4 | **Common ground** between the motor supply and the UNO Q. | Otherwise STEP signals are unreliable. |
| 5 | **EN pin**: firmware drives the shield's enable line on **D8** (active low). Leave the shield's EN jumper/routing standard. | Lets the firmware de-energise the motors after a fall. |
| 6 | **IMU**: MPU6050 on the Qwiic connector (`Wire1`). Mount it **rigidly and flat** to the chassis (double-sided foam tape is fine; loose = noise), as **low** as practical (near the axle line reduces acceleration pollution), away from motor magnets. Any orientation works — you set axis/sign in the dashboard. | The pitch estimate is only as good as the mounting. |
| 7 | **Mechanics**: no slop in the wheel couplers/grub screws, wheels with grip, chassis stiff, **heavy parts (battery) high up** — a taller centre of mass falls slower and is *easier* to balance. Symmetric left/right. | Backlash and flex look like control problems that no tuning can fix. |
| 8 | **Measure the wheel diameter** (mm). You enter it once; all gains rescale. | Default gains assume 80 mm. On 60 mm wheels the *unscaled* defaults make the robot fall (simulated). |

## 4. Start the app

> Only one app runs at a time — starting this one stops whichever app is running.

On the board (or via `adb shell` / SSH / App Lab):

```bash
arduino-app-cli app start ~/ArduinoApps/julia-x2
arduino-app-cli app logs  ~/ArduinoApps/julia-x2 --follow   # Python log
arduino-app-cli monitor                                     # MCU log: state changes, calibration
```

Find the board's address (`hostname -I`) and open the dashboard on your phone/laptop
(same WiFi): **`http://<board-ip>:7000`**.

On the MCU log you should see `MPU6050 ready (WHO_AM_I=0x68) -- calibrate from the dashboard`.
If you see `MPU6050 NOT found`, check the Qwiic cable before anything else.

The first build after editing the sketch takes about 1½ minutes. If a build ever seems stale:
`arduino-app-cli app clean-cache user:julia-x2 --force`, then restart.

## 5. First-time bring-up

Do this once, with the robot **on a stand, wheels in the air**, dashboard open.
Tap **Tune** (top-right of the header) to open the tuning panel, where most steps below live; the
IMU panel on the right also has a **Calibrate** button.

### 5.1 Calibrate the gyro
Put the robot down **completely still** and press **1 · Calibrate gyro** (or the IMU panel's
*Calibrate*). It averages 1.5 s of gyro data. If the robot moved, calibration is **rejected**
("failed") — just retry. Until this succeeds, self-balancing is locked (Manual mode does not need it).
Recalibrate whenever the robot has warmed up a lot or the readout at rest drifts.

### 5.2 Wheel diameter
*Mounting → Wheel diameter*: enter your wheels' diameter in mm.

### 5.3 Motor directions
Close the panel and make sure the switch says **Manual**. Push the joystick **forward**: both wheels must turn in the same direction as
"forward" for your robot (the direction *you* decide is its front). If one wheel is reversed,
*Mounting → Invert left motor* / *Invert right motor*. Repeat until both agree.

### 5.4 Pitch axis and sign
Open **Tune** and watch **Pitch**. Tilt the robot **forward** (toward the direction you chose in
5.3) by hand ~20°:

| What you see | Fix (*Mounting*) |
|--------------|------------------|
| Pitch does not change | Toggle **Pitch axis** (0 ↔ 1) |
| Pitch changes but goes **negative** when tilting forward | Toggle **Invert pitch** |
| Pitch goes **positive** when tilting forward | Correct ✓ |

> Rule of thumb: **forward lean = positive pitch = wheels must roll forward.** Steps 5.3 and
> 5.4 together guarantee that. Getting this wrong is the #1 cause of "it just runs away".

### 5.5 Set the balance point (coarse)
Hold the robot upright **by hand, as vertical as you can judge by eye** (centre of mass
above the axle), and still. Press **2 · Set balance point**. This is good to a degree or two
and the controller tolerates that (simulated up to ±2.5°). You will fine-tune it in 6.3.

### 5.6 Dry run (wheels still in the air)
Switch to **Self-balancing**, press **Start balancing**; hold the robot upright and still → it engages
(state **Balancing**). Now, holding the robot by the chassis, **tilt it forward**: the wheels
must spin **forward**, harder the more you tilt. Tilt back → wheels spin backward. Press
**Stop balancing**. If the wheels react the wrong way, revisit 5.3/5.4.

## 6. Your first balance

### 6.1 Start
Put the robot on the floor, switch to **Self-balancing**, hold it upright at its balance point, press **Start balancing**
(button turns amber: *Armed*). Hold it still and upright — after **0.4 s within ±2°** the
motors engage (*Balancing*, green). Let go gently **but keep your hands next to it**.

Good first result: it stands, wobbling by less than about a degree. If it immediately runs
away or oscillates, press **Stop balancing** (or just let it fall — the fall limit cuts the
motors) and go to [Tuning](#7-tuning). You'll rarely need more than a few iterations.

### 6.2 Read the scope (Tune panel)
Traces show the last 12 s:

* **Pitch** (cyan) — degrees from the balance point. Healthy: hovering within ±0.5° of 0.
* **Lean cmd** (amber) — what the speed loop asks for. Should be small and slow.
* **Speed** (magenta, ÷1000 steps/s) — wheel speed. Small back-and-forth movements are normal.
* **Loop** readout — worst control-cycle time since arming. Healthy: **< 2000 µs, 0 late**.
  Red means the MCU is overloaded (see Troubleshooting).

### 6.3 Fine-tune the balance point
Let it stand in place for **10–15 s**. Read **Pitch** once it has settled.

* If it settles at, say, **+0.8°**, the true balance point is 0.8° further forward: add 0.8 to
  *Calibration → Balance point*.
* If **−0.5°**, subtract 0.5.
* Repeat until it stands still with **|Pitch| < 0.3°** and stops creeping around the floor.

(Why it works: at rest the speed loop needs a constant lean to cancel a balance-point error, and
that lean is exactly the error you read.)

## 7. Tuning

**Rules:** change **one** value at a time by 20–30 %; test with a **light nudge** at the top of
the robot after each change; keep notes. Values apply instantly and are saved automatically.

### 7.1 The order

1. **Angle Kp** — the "stiffness". Start from the default.
   * *Too low*: the robot **falls slowly in one direction without fighting back**, or can't
     recover from a nudge. There is a hard floor: at ½ × the default it never recovers, and at
     ~0.7 × it only survives with extra damping (simulated) — the correction has to outrun
     gravity, and below the floor no amount of Kd helps.
   * *Raise* until nudges are answered promptly. *Too high*: fast buzzing/shaking, motors
     sound harsh.
2. **Angle Kd** — the damping (uses the gyro).
   * *Too low*: after a nudge it **overshoots and rings** (visible as a decaying wave in
     Pitch).
   * *Raise* until the ringing dies in ~1 cycle. *Too high*: growling/chattering motors,
     jittery scope trace (gyro noise amplified).
   * The simulation is forgiving here: any Kd from ½× to 2× the default stayed stable once Kp
     was adequate. Kp is the critical one.
3. **Speed Kp / Speed Ki** — position holding.
   * *Slow, large rolling back-and-forth (period several seconds)*: Speed Kp/Ki too **high** —
     lower them (Ki first).
   * *It creeps away and takes long to come back*: raise Speed Ki a little, then Speed Kp.
   * Leave them alone if it already holds position well.
4. **Filter time constant** (*Calibration*) — rarely needed. Lower (0.3–0.5 s) if pitch lags
   during fast wobble; raise (0.8–1.5 s) if pitch is noisy from motor vibration.
5. **Max wheel accel.** — if you *hear or see* skipped steps (wheel jerks/stalls) during hard
   recoveries, reduce it; then consider more supply voltage or driver current.

### 7.2 What "good" looks like
Stands for minutes; a firm push at the top is recovered in about a second with a few degrees of
excursion; wheel speed stays small at rest; loop time steady.

### 7.3 Physical intuition (why these numbers)
* Taller robot (higher centre of mass) → slower fall → needs **lower** gains, tolerates lag.
  Low, squat robot → falls fast → needs **higher** Kp/Kd and a fast loop.
* Bigger wheels move the robot further per step, so the software needs **fewer** steps/s² for
  the same effect — *Wheel diameter* handles this for you.
* A stepper is a speed source with limited torque at high speed: if a hard recovery needs more
  than the motors can give, no gain setting helps — lower the mass, raise the supply voltage,
  or lower *Max wheel speed* so it never enters the torque-cliff region.

## 8. Driving, saving, faults

**Driving in Self-balancing mode.** Once the state is *Balancing*, the joystick's up/down asks for
forward/back speed and left/right for turning. The controller leans to accelerate and brakes by
leaning back, so it stays upright the whole time. The **Max Speed** slider scales forward speed.
If the joystick goes quiet for 0.5 s the target returns to zero (the robot stays upright; the
dashboard re-sends the held stick position, so holding still is fine). Tunables (*Driving* group):
*Forward speed*, *Turn speed*, *Joystick ramp*, and *Balance → Max lean*. Increase speeds in small
steps; fast driving needs a well-tuned, well-powered robot.

**Driving in Manual mode.** The stick maps straight to wheel speeds: full forward = 16000 steps/s
× the Max Speed slider, smoothed by *Manual accel. limit*. There is no balancing, so use it on a
stand, on a rover-style chassis, or to reposition the robot.

**Saving.** Every change is saved after ~1.5 s to `data/tuning.json` (only values that differ
from the defaults). It is re-sent to the MCU whenever the app or the MCU restarts. *Reset to
defaults* clears everything, including the balance point and mounting flags. Back the file up
once you have a good tune.

**Faults** (red *Fault* state, motors off — pick the robot up, press *Reset fault*):

| Message | Meaning | Usual cause |
|---------|---------|-------------|
| Fell over | Tilt beyond *Fall limit* | Normal after a fall; if immediate, pitch axis/sign wrong (5.4) |
| Wheel speed runaway | Wheel pinned at *Max wheel speed* for 1 s | Robot leaning far off the balance point, wrong motor direction, gains far too low |
| IMU read failure | 5 consecutive I²C reads failed | Loose Qwiic cable / brown-out / noise on the bus |

## 9. Simulate first

Because a real tune costs falls, the exact controller maths can be exercised on the board
(or any computer with Python 3, no extra packages):

```bash
python3 tools/sim/balance_sim.py                    # nominal run + robustness sweeps (~20 s)
python3 tools/sim/balance_sim.py kp=4000 kd=300 nosweep
python3 tools/sim/balance_sim.py wheel=65 L=0.15 nosweep csv=/tmp/run.csv
```

Options: `kp kd skp ski tau vmax amax trim cg push tilt`, geometry `L` (centre-of-mass
height above the axle, m) and `wheel` (mm). `wheel=` tells both the plant and the controller;
`wheelcfg=` overrides what the controller is told (to test a wrong entry).

The plant is a wheeled inverted pendulum with a **realistic sensor path**: gyro noise and
residual bias, accelerometer noise, ~4 ms of IMU filter delay, accelerometer readings polluted
by the wheel's own acceleration, a balance point deliberately off by 1°, and step-quantised
wheel speed. What it showed with the shipped defaults (80 mm wheels, `L = 0.12 m`):

| Test | Result |
|------|--------|
| Start 4° off, then 60°/s shove, then drive, then turn | Never falls; ~0.06° RMS wobble once the balance point is right; recovers from the shove in ≈ 1 s |
| Kp × 1 … × 2 with Kd × 0.5 … × 2 | Stable in every combination |
| Kp × 0.5 | Always falls (correction can't outrun gravity) → the "Kp floor" |
| Kp × 0.7 | Marginal: falls at the default Kd, survives with Kd ≥ 1.4 × |
| Centre-of-mass height 0.06 – 0.25 m | Stable; the 0.25 m robot saturates the wheel-speed limit → raise Max wheel speed / voltage |
| Shove up to 140°/s | Recovered (peak ≈ 10° excursion) |
| Balance point off by ±2.5° | Stable; the speed loop slowly compensates; drift shrinks as you correct it (6.3) |
| Wheel diameter 60 / 120 mm with matching setting | Identical behaviour to 80 mm |
| Wheel diameter entered ±10 % wrong | Still fine (entering it too **large** by 25 % starts to oscillate) |

**Limits of this validation:** the model is idealised — no wheel/ground slip, no motor torque
curve or resonance, no chassis flex, perfect velocity source. The simulator proves the
*structure and default gains are sound and robust*; it cannot replace tuning on the real robot.
`tools/sim/balance_sim.py` mirrors `sketch/balance.h` line for line; if you change the
controller, change both.

## 10. Reference

### Parameters: what each one does

Open **Tune** to change them; every change is applied immediately and saved automatically. The
small **↺** button next to a value restores that parameter's default. "Steps" are microsteps
(3200 per wheel revolution).

#### Balance — the two control loops

**Angle Kp** (`angle_kp`, default 3200, range 100–30000, steps/s² per °) — *stiffness*. For every
degree the robot is away from where it should be, the wheels accelerate this hard toward the lean.
**Too low:** it can't out-run gravity and falls slowly without fighting back (below ~½ of the
default it can never recover). **Too high:** fast buzzing/shaking and harsh motor noise. The most
important gain: raise until the robot recovers promptly from a nudge. Scales with wheel diameter
automatically.

**Angle Kd** (`angle_kd`, 250, 0–3000, steps/s² per °/s) — *damping*. Reacts to how fast the robot
is falling, measured directly by the gyro. **Too low:** the robot overshoots and rings after a
nudge. **Too high:** growling/chattering motors and a jittery scope trace (gyro noise amplified).
Forgiving: ½×–2× the default all worked in simulation once Kp was adequate.

**Speed Kp** (`speed_kp`, 0.2, 0–5, ° of lean per 1000 steps/s of speed error) — *position holding,
fast part*. If the robot is rolling faster than it should, it leans back this many degrees per
1000 steps/s to slow itself. **Too low:** it drifts away and returns slowly. **Too high:** slow
rocking back and forth (seconds per swing).

**Speed Ki** (`speed_ki`, 0.10, 0–5, ° per (1000 steps/s·s)) — *position holding, slow part*.
Accumulates the speed error over time, removing steady creeping and soaking up a small error in the
balance point. **Too high:** slow hunting. Leave at the default unless it creeps at rest.

**Max lean** (`max_lean`, 6, 0.5–15 °) — the biggest angle the speed loop may ask the robot to lean
away from its balance point. It limits how hard the robot can accelerate and brake when driving:
larger = snappier, but a bigger step for the angle loop to follow.

#### Calibration — what the sensor tells the robot

**Balance point** (`trim`, 0, −30…30 °) — the pitch reading at which the robot stands on its own
(centre of mass above the axle). Set it with **Set balance point** (hold the robot there) and refine
it as in step 6.3. A wrong value makes the robot creep in one direction.

**Filter time const.** (`cf_tau`, 0.6, 0.1–5 s) — how the gyro and accelerometer are blended. Short
= trust the accelerometer more (drift corrected quicker, but noisier and disturbed by wheel
acceleration); long = trust the gyro more (smooth, but drifts longer before being corrected).
Rarely needs changing.

#### Limits — protection and drive-train capability

**Max wheel speed** (`vmax`, 10000, 1000–16000 steps/s ≈ 3 rev/s) — the wheel-speed ceiling while
balancing. Keep it below where your motors lose torque (see the A4988 notes in the hardware
checklist). If a wheel stays pinned here for 1 s the runaway protection cuts the motors.

**Max wheel accel.** (`amax`, 60000, 2000–400000 steps/s²) — the fastest the wheels may speed up or
slow down. Lower it if you hear or see steps being skipped in hard recoveries; too low and the robot
can't catch itself.

**Fall limit** (`fall_deg`, 30, 10–60 °) — if the robot tilts this far from its balance point the
motors switch off (Fault) so it doesn't thrash on the floor.

**Engage window** (`arm_deg`, 2, 0.5–10 °) — after *Start balancing*, the robot must be held within
this angle of upright (and nearly still) for 0.4 s before the motors engage.

#### Driving

**Forward speed** (`fwd_max`, 4000, 0–16000 steps/s) — balance mode: wheel speed at full forward
joystick (4000 ≈ 1.25 rev/s). Raise gradually; fast driving needs a well-tuned, well-powered robot.

**Turn speed** (`steer_max`, 2500, 0–16000 steps/s) — balance mode: how much faster one wheel runs
than the other at full sideways joystick.

**Joystick ramp** (`ramp`, 12000, 500–200000 steps/s²) — balance mode: how quickly the requested
speed follows the joystick. Lower = smoother, more gradual starts and stops.

**Manual accel. limit** (`manual_ramp`, 40000, 2000–400000 steps/s²) — manual mode: the fastest the
wheels may speed up. A full-stick jump to 16000 steps/s takes 0.4 s at the default. Lower it if the
motors stall or squeal on abrupt stick moves; raise it for a more direct feel.

#### Mounting — how things are fitted

**Wheel diameter** (`wheel_mm`, 80, 30–250 mm) — measure your wheels and enter it once. The gains
(Angle Kp/Kd, Speed Kp/Ki) are defined for 80 mm wheels and rescaled from this value, so a different
wheel size needs no re-tuning. Within ±10 % is fine; err on the smaller side.

**Pitch axis** (`axis`, 0 or 1) — which IMU axis the robot falls about (0 = X, 1 = Y). Tilt the
robot forward: the **Pitch** readout must respond; if not, flip this.

**Invert pitch** (`invert`, 0/1) — set to 1 if tilting the robot **forward** makes Pitch go
negative. Forward lean must read positive.

**Invert left motor / Invert right motor** (`invert_left`, `invert_right`, 0/1) — reverse a wheel
whose direction is wrong. Both wheels must roll the same way when the joystick is pushed forward in
Manual mode. Applies to both modes.

#### Quick table

| Name | Default | Range | Unit |
|------|---------|-------|------|
| `angle_kp` | 3200 | 100–30000 | steps/s² per ° |
| `angle_kd` | 250 | 0–3000 | steps/s² per °/s |
| `speed_kp` | 0.2 | 0–5 | ° per 1000 steps/s |
| `speed_ki` | 0.10 | 0–5 | ° per (1000 steps/s·s) |
| `max_lean` | 6 | 0.5–15 | ° |
| `trim` | 0 | −30…30 | ° |
| `cf_tau` | 0.6 | 0.1–5 | s |
| `vmax` | 10000 | 1000–16000 | steps/s |
| `amax` | 60000 | 2000–400000 | steps/s² |
| `fall_deg` | 30 | 10–60 | ° |
| `arm_deg` | 2 | 0.5–10 | ° |
| `fwd_max` | 4000 | 0–16000 | steps/s |
| `steer_max` | 2500 | 0–16000 | steps/s |
| `ramp` | 12000 | 500–200000 | steps/s² |
| `manual_ramp` | 40000 | 2000–400000 | steps/s² |
| `wheel_mm` | 80 | 30–250 | mm |
| `axis` / `invert` / `invert_left` / `invert_right` | 0 | 0/1 | – |

The defaults live in `sketch/balance.h` (`jb::Params`); names and limits are duplicated in
`PARAM_TABLE` (`sketch/sketch.ino`) and `PARAMS` (`python/main.py`).

### Bridge API

| Direction | Name | Purpose |
|-----------|------|---------|
| Python → MCU | `set_param(name, value)` | Set one parameter (returns false if unknown/out of range) |
| | `balance_arm(on)` | Arm (waits for upright; balance mode only) / disarm + clear fault |
| | `balance_cmd(fwd, steer)` | Normalised joystick, −1…1 |
| | `calibrate_imu()` | 1.5 s gyro-bias calibration (rejected if moving) |
| | `capture_trim()` | 0.5 s averaged pitch, for *Set balance point* |
| | `imu_status()` | Whether calibration has happened (resync) |
| | `set_mode(mode)` | 0 = manual, 1 = self-balancing (leaving balance mode disarms first) |
| | `drive(l, r)`, `stop()` | Raw wheel drive; **manual mode only** |
| MCU → Python | `balance(state, fault, pitch, rate, lean, speed, accel, loop_us, overruns, param_writes, mode)` | 20 Hz |
| | `imu(angleX, angleY, gyroXYZ, accXYZ, temp)` | 5 Hz, IMU panel |

`param_writes` counts `set_param` calls since MCU boot; if it goes down, the MCU rebooted and
Python re-sends every parameter.

### Files

```
sketch/balance.h     attitude filter + cascaded controller (no hardware includes)
sketch/step_gen.h    10 kHz timer step generator + watchdog
sketch/mpu6050.h     raw-register IMU driver (400 kHz, DLPF ~94 Hz)
sketch/sketch.ino    state machine, balance thread, Bridge API, telemetry
python/main.py       dashboard backend, parameter sync, persistence (data/tuning.json)
assets/              dashboard: single-page driving screen + Tune sheet (scope, parameters)
tools/sim/           host-side simulator
```

## 11. Troubleshooting

| Symptom | Likely cause → fix |
|---------|--------------------|
| Start button missing / disabled | You are in Manual mode (switch to Self-balancing), or the IMU is not calibrated (5.1) |
| Calibration "failed" | Robot moved during the 1.5 s; put it down and retry; check the IMU cable |
| Engages, then instantly runs away and faults | Pitch axis/sign or motor direction wrong (5.3, 5.4); wheel diameter far off (5.2) |
| Falls slowly the same way every time | *Balance point* off (6.3), or Kp too low |
| Fast buzzing / motors squeal | Kp or Kd too high; IMU loose; try a longer *Filter time const.* |
| Slow, big rocking (seconds) | Speed Kp/Ki too high |
| Creeps across the floor at rest | Balance point slightly off (6.3), or Speed Ki too low |
| Wheels jerk/stall in hard recoveries | Skipped steps: lower *Max wheel accel.* / *Max wheel speed*; raise driver current or supply voltage; check A4988 heat |
| *Loop* readout red / "late" cycles | MCU overloaded — the balance loop should take < 2 ms. Check the MCU log for `WARNING: no stack for the balance thread` (it then runs degraded in `loop()`) |
| `MPU6050 NOT found` | Qwiic cable/connector, IMU power; the sketch expects `Wire1` |
| Settings vanished | `data/tuning.json` missing/unreadable; the file is only written after a change |
| Nothing works after editing the sketch | `arduino-app-cli app clean-cache user:julia-x2 --force` then restart |
