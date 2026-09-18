#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
#
# SPDX-License-Identifier: MPL-2.0
"""Host-side simulator for the balancing controller (pure Python, no dependencies).

`Balancer` and `AttitudeFilter` below are a line-for-line mirror of sketch/balance.h -- if you
change the controller there, change it here too (there is no C++ compiler on the UNO Q, so the
header itself cannot be run on the board; it *is* compile-checked with the ARM toolchain).

The plant is a wheeled inverted pendulum with realistic imperfections:
  * accelerometer polluted by wheel acceleration (the classic complementary-filter enemy)
  * gyro noise + residual bias, accelerometer noise
  * ~4 ms IMU low-pass delay
  * a true balance point that differs from the IMU's "0 deg" (trim error)
  * wheel speed quantised to whole steps/s (stepper velocity source)

Usage:
  python3 tools/sim/balance_sim.py                       # nominal run + robustness sweeps
  python3 tools/sim/balance_sim.py kp=4000 kd=120 nosweep
  python3 tools/sim/balance_sim.py nosweep csv=/tmp/run.csv
Overridable: kp kd skp ski tau vmax amax trim L cg push tilt wheel (physical wheel diameter, mm; default 80) wheelcfg (diameter the controller is told)
"""
import math
import random
import sys
from collections import deque
from dataclasses import dataclass, replace

RAD2DEG = 57.29577951
REF_WHEEL_MM = 80.0


def clampf(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def slew(cur, tgt, max_delta):
    d = tgt - cur
    if d > max_delta:
        return cur + max_delta
    if d < -max_delta:
        return cur - max_delta
    return tgt


@dataclass
class Params:  # keep defaults identical to jb::Params in sketch/balance.h
    angle_kp: float = 3200.0
    angle_kd: float = 250.0
    speed_kp: float = 0.2
    speed_ki: float = 0.10
    max_lean: float = 6.0
    trim: float = 0.0
    wheel_mm: float = 80.0
    cf_tau: float = 0.6
    vmax: float = 10000.0
    amax: float = 60000.0
    fall_deg: float = 30.0
    arm_deg: float = 2.0
    fwd_max: float = 4000.0
    steer_max: float = 2500.0
    ramp: float = 12000.0


class AttitudeFilter:
    def __init__(self):
        self.angle = 0.0
        self.initialised = False

    def update(self, gyro_dps, accel_angle_deg, dt, tau):
        if not self.initialised:
            self.angle = accel_angle_deg
            self.initialised = True
            return self.angle
        a = tau / (tau + dt)
        self.angle = a * (self.angle + gyro_dps * dt) + (1.0 - a) * accel_angle_deg
        return self.angle


class Balancer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.v = self.v_set = self.turn = self.lean_i = 0.0
        self.lean_target = self.accel = self.sat_time = 0.0

    def update(self, p, angle, rate, dt, fwd, steer):
        gs = REF_WHEEL_MM / clampf(p.wheel_mm, 20.0, 400.0)
        self.v_set = slew(self.v_set, clampf(fwd, -1, 1) * p.fwd_max, p.ramp * dt)
        self.turn = slew(self.turn, clampf(steer, -1, 1) * p.steer_max, p.ramp * dt)

        v_err = (self.v_set - self.v) * 0.001
        self.lean_i = clampf(self.lean_i + (p.speed_ki / gs) * v_err * dt, -p.max_lean, p.max_lean)
        self.lean_target = clampf((p.speed_kp / gs) * v_err + self.lean_i, -p.max_lean, p.max_lean)

        err = angle - (p.trim + self.lean_target)
        self.accel = clampf(gs * (p.angle_kp * err + p.angle_kd * rate), -p.amax, p.amax)

        self.v = clampf(self.v + self.accel * dt, -p.vmax, p.vmax)
        self.sat_time = self.sat_time + dt if abs(self.v) >= p.vmax else 0.0
        return (clampf(self.v + self.turn, -p.vmax, p.vmax),
                clampf(self.v - self.turn, -p.vmax, p.vmax))


@dataclass
class Robot:
    L: float = 0.12               # m, effective COM height above the axle
    m_per_step: float = 7.854e-5  # pi * 0.08 m / 3200 microsteps
    cg_offset_deg: float = 1.0    # true balance point vs the IMU's zero
    sensor_height: float = 0.05   # m above the axle
    gyro_bias_dps: float = 0.25   # residual after calibration
    gyro_noise_dps: float = 0.12  # rms per sample
    accel_noise_rad: float = 0.004
    imu_delay_s: float = 0.004
    g: float = 9.81


@dataclass
class Scenario:
    initial_tilt_deg: float = 4.0
    push_at_s: float = 6.0
    push_dps: float = 60.0
    drive_from: float = 10.0
    drive_to: float = 14.0
    drive_fwd: float = 1.0
    turn_from: float = 15.0
    turn_to: float = 17.0
    end_s: float = 20.0


@dataclass
class Result:
    fell: bool = False
    max_abs_deg: float = 0.0
    rms_still_deg: float = 0.0
    vmax: float = 0.0
    drift_m: float = 0.0
    settle_s: float = 0.0
    sat_s: float = 0.0


def simulate(p, R, sc, seed=1, csv=None):
    rng = random.Random(seed)
    dt_plant, dt_ctl = 0.0005, 1.0 / 200.0
    sub = int(dt_ctl / dt_plant + 0.5)

    phi = math.radians(sc.initial_tilt_deg)  # rad from true vertical
    phi_dot = 0.0
    base_v = base_x = base_acc = phi_ddot = 0.0
    filt, bal = AttitudeFilter(), Balancer()
    line = deque()
    delay_n = int(R.imu_delay_s / dt_ctl + 0.5)

    res = Result()
    sum_sq, n_still, hold_x0, hold_started, last_oob = 0.0, 0, 0.0, False, 0.0
    t, pushed = 0.0, False

    while t < sc.end_s:
        acc_ang = math.degrees(math.atan2(
            R.g * math.sin(phi) - base_acc * math.cos(phi) + R.sensor_height * phi_ddot,
            R.g * math.cos(phi) + base_acc * math.sin(phi))) \
            + R.cg_offset_deg + math.degrees(R.accel_noise_rad * rng.gauss(0, 1))
        gyro = math.degrees(phi_dot) + R.gyro_bias_dps + R.gyro_noise_dps * rng.gauss(0, 1)
        line.append((gyro, acc_ang))
        while len(line) > delay_n + 1:
            line.popleft()
        m_gyro, m_acc = line[0]

        angle = filt.update(m_gyro, m_acc, dt_ctl, p.cf_tau)
        fwd = sc.drive_fwd if sc.drive_from <= t < sc.drive_to else 0.0
        steer = 1.0 if sc.turn_from <= t < sc.turn_to else 0.0
        vl, vr = bal.update(p, angle, m_gyro, dt_ctl, fwd, steer)
        applied_v = round(0.5 * (vl + vr))

        for _ in range(sub):
            prev = base_v
            base_v = applied_v * R.m_per_step
            base_acc = (base_v - prev) / dt_plant
            phi_ddot = (R.g * math.sin(phi) - base_acc * math.cos(phi)) / R.L
            phi_dot += phi_ddot * dt_plant
            phi += phi_dot * dt_plant
            base_x += base_v * dt_plant
            if not pushed and t >= sc.push_at_s:
                phi_dot += math.radians(sc.push_dps)
                pushed = True
        t += dt_ctl

        err_deg = math.degrees(phi)
        if abs(err_deg) > p.fall_deg:
            res.fell = True
            break
        if t > 2.0:
            res.max_abs_deg = max(res.max_abs_deg, abs(err_deg))
        res.vmax = max(res.vmax, abs(applied_v))
        if sc.push_at_s < t < sc.drive_from and abs(err_deg) > 1.0:  # recovery window only
            last_oob = t - sc.push_at_s
        if 3.0 < t < sc.push_at_s - 0.5:
            sum_sq += err_deg * err_deg
            n_still += 1
            if not hold_started:
                hold_started, hold_x0 = True, base_x
            res.drift_m = base_x - hold_x0
        res.sat_s = max(res.sat_s, bal.sat_time)
        if csv:
            csv.write(f"{t:.3f},{err_deg:.3f},{angle - p.trim:.3f},{bal.lean_target:.3f},"
                      f"{applied_v:.0f},{base_x:.4f},{fwd}\n")

    res.rms_still_deg = math.sqrt(sum_sq / n_still) if n_still else 0.0
    res.settle_s = last_oob
    return res


def show(label, r):
    print(f"{label:<34} fell={int(r.fell)}  max|ang|={r.max_abs_deg:5.2f} deg  "
          f"rms(still)={r.rms_still_deg:5.3f} deg  vmax={r.vmax:6.0f}  "
          f"drift={r.drift_m:6.3f} m  settle={r.settle_s:4.2f} s")


def main(argv):
    p, R, sc = Params(), Robot(), Scenario()
    csv_path, sweep = None, True
    keymap = {"kp": (p, "angle_kp"), "kd": (p, "angle_kd"), "skp": (p, "speed_kp"),
              "ski": (p, "speed_ki"), "tau": (p, "cf_tau"), "vmax": (p, "vmax"),
              "amax": (p, "amax"), "trim": (p, "trim"), "L": (R, "L"),
              "cg": (R, "cg_offset_deg"), "push": (sc, "push_dps"),
              "tilt": (sc, "initial_tilt_deg")}
    for a in argv:
        if a.startswith("wheel="):  # physical wheel; the controller is told the same value
            R.m_per_step = math.pi * float(a[6:]) / 1000.0 / 3200.0
            p.wheel_mm = float(a[6:])
        elif a.startswith("wheelcfg="):  # what the controller *thinks* (to test a wrong entry)
            p.wheel_mm = float(a[9:])
        elif a == "nosweep":
            sweep = False
        elif a.startswith("csv="):
            csv_path = a[4:]
        elif "=" in a and a.split("=")[0] in keymap:
            k, v = a.split("=")
            obj, attr = keymap[k]
            setattr(obj, attr, float(v))
        else:
            sys.exit(f"unknown argument: {a}")

    print(f"Params: kp={p.angle_kp:.0f} kd={p.angle_kd:.0f} skp={p.speed_kp:.2f} "
          f"ski={p.speed_ki:.2f} tau={p.cf_tau:.2f} vmax={p.vmax:.0f} amax={p.amax:.0f} | "
          f"L={R.L:.2f} m, trim error={R.cg_offset_deg:.1f} deg")
    f = open(csv_path, "w") if csv_path else None
    if f:
        f.write("t,angle_true,angle_meas,lean_target,v,x,fwd\n")
    r0 = simulate(p, R, sc, 1, f)
    if f:
        f.close()
    show("nominal (trim off by 1 deg)", r0)
    if not sweep:
        return 1 if r0.fell else 0

    fails = runs = 0

    def run(label, pp, rr, ss, display=True):
        nonlocal fails, runs
        local, worst = 0, Result()
        for seed in range(1, 4):
            r = simulate(pp, rr, ss, seed)
            runs += 1
            if r.fell:
                fails += 1
                local += 1
            if r.max_abs_deg >= worst.max_abs_deg:
                worst = r
        if display:
            show(f"{label} ({local}/3 fell)", worst)

    print("\n-- Gain robustness (real mass / wheels / motors differ from this model) --")
    scales = (0.5, 0.7, 1.0, 1.4, 2.0)
    for ks in scales:
        for ds in scales:
            run(f"Kp x{ks:.1f}, Kd x{ds:.1f}",
                replace(p, angle_kp=p.angle_kp * ks, angle_kd=p.angle_kd * ds), R, sc,
                display=(ks == 1.0 or ds == 1.0))
    print("\n-- Centre-of-mass height --")
    for L in (0.06, 0.09, 0.12, 0.18, 0.25):
        run(f"L={L:.2f} m", p, replace(R, L=L), sc)
    print("\n-- Disturbances --")
    for push in (30, 60, 100, 140):
        run(f"shove {push} deg/s", p, R, replace(sc, push_dps=float(push)))
    for cg in (0.0, 1.0, 2.5, -2.5):
        run(f"trim error {cg:.1f} deg", p, replace(R, cg_offset_deg=cg), sc)
    print(f"\nTotal: {fails} of {runs} runs fell.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
