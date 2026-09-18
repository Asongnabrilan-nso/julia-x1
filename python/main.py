# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
#
# SPDX-License-Identifier: MPL-2.0

import json
import threading
import time
from pathlib import Path

from arduino.app_utils import App, Bridge, Logger
from arduino.app_bricks.web_ui import WebUI

# The balance loop itself runs entirely on the MCU (sketch/sketch.ino). This app is the
# operator interface: joystick, live telemetry, tuning, and persistence of the tuned values.

MAX_STEPS_PER_SEC = 16000  # 5 rev/sec (300 RPM) at 200 steps/rev x 1/16 microstepping (A4988); must not exceed the sketch's MANUAL_MAX_STEPS_PER_SEC clamp
JOYSTICK_SEND_HZ = 20     # rate limit for outgoing drive commands while dragging
DEADZONE = 0.15           # matches the client-side joystick deadzone
MIN_SPEED_SCALE = 0.1     # floor so the "-" button/slider can never make the robot unresponsive
MAX_SPEED_SCALE = 1.0
BALANCE_TELEMETRY_HZ = 20  # the sketch pushes 20 Hz; forward every frame to the scope

TUNING_FILE = Path(__file__).resolve().parent.parent / "data" / "tuning.json"
SAVE_DEBOUNCE_S = 1.5

# MCU state / fault codes (must match enum State / Fault in sketch.ino)
ST_IDLE, ST_ARMED, ST_BALANCING, ST_FAULT = 0, 1, 2, 3
MODE_MANUAL, MODE_BALANCE = 0, 1  # control modes (must match enum Mode in sketch.ino)
FAULT_TEXT = {0: "", 1: "Fell over (tilt beyond fall limit)", 2: "Wheel speed runaway", 3: "IMU read failure"}

# Tunable parameters. Names/limits must match PARAM_TABLE in sketch.ino and defaults must match
# jb::Params in sketch/balance.h. `group` and `help` only drive the tuning panel.
PARAMS = [
    # name, group, label, unit, default, min, max, step, help
    ("angle_kp", "Balance", "Angle Kp", "steps/s² per °", 3200.0, 100.0, 30000.0, 50.0,
     "Stiffness. Raise until the robot recovers from a push; too high = fast buzzing/oscillation."),
    ("angle_kd", "Balance", "Angle Kd", "steps/s² per °/s", 250.0, 0.0, 3000.0, 5.0,
     "Damping on the gyro rate. Raise to calm wobble; too high = harsh/noisy motor sound."),
    ("speed_kp", "Balance", "Speed Kp", "° per 1000 steps/s", 0.2, 0.0, 5.0, 0.01,
     "Fights drift: how hard it leans back when it rolls away. Too high = slow big oscillation."),
    ("speed_ki", "Balance", "Speed Ki", "° per (1000 steps/s·s)", 0.10, 0.0, 5.0, 0.01,
     "Removes long-term drift and small trim errors. Too high = slow hunting."),
    ("max_lean", "Balance", "Max lean", "°", 6.0, 0.5, 15.0, 0.5,
     "Largest lean the speed loop may command. Limits acceleration when driving."),
    ("trim", "Calibration", "Balance point", "°", 0.0, -30.0, 30.0, 0.05,
     "Pitch reading at which the robot balances hands-off. Use 'Set balance point' rather than typing."),
    ("wheel_mm", "Mounting", "Wheel diameter", "mm", 80.0, 30.0, 250.0, 1.0,
     "Diameter of the drive wheels. All gains rescale automatically, so enter it once and leave it."),
    ("cf_tau", "Calibration", "Filter time const.", "s", 0.6, 0.1, 5.0, 0.05,
     "Complementary filter: lower trusts the accelerometer more (faster, noisier)."),
    ("vmax", "Limits", "Max wheel speed", "steps/s", 10000.0, 1000.0, 16000.0, 250.0,
     "Hard wheel speed limit while balancing. Keep below where the motors lose torque."),
    ("amax", "Limits", "Max wheel accel.", "steps/s²", 60000.0, 2000.0, 400000.0, 1000.0,
     "Lower it if motors skip steps (you hear/see it) during hard recoveries."),
    ("fall_deg", "Limits", "Fall limit", "°", 30.0, 10.0, 60.0, 1.0,
     "Beyond this tilt from the balance point the motors switch off."),
    ("arm_deg", "Limits", "Engage window", "°", 2.0, 0.5, 10.0, 0.5,
     "How upright the robot must be (and held for 0.4 s) before the motors engage."),
    ("fwd_max", "Driving", "Forward speed", "steps/s", 4000.0, 0.0, 16000.0, 100.0,
     "Wheel speed at full joystick. Increase gradually."),
    ("steer_max", "Driving", "Turn speed", "steps/s", 2500.0, 0.0, 16000.0, 100.0,
     "Wheel speed difference at full sideways joystick."),
    ("ramp", "Driving", "Joystick ramp", "steps/s²", 12000.0, 500.0, 200000.0, 500.0,
     "Balance mode: how fast setpoints follow the joystick. Lower = smoother."),
    ("manual_ramp", "Driving", "Manual accel. limit", "steps/s²", 40000.0, 2000.0, 400000.0, 1000.0,
     "Manual mode: wheel acceleration limit. Lower if the motors stall on sudden stick moves."),
    ("axis", "Mounting", "Pitch axis", "0 = X, 1 = Y", 0.0, 0.0, 1.0, 1.0,
     "Which IMU axis the robot falls about. Tilt forward: the Pitch readout must move."),
    ("invert", "Mounting", "Invert pitch", "0/1", 0.0, 0.0, 1.0, 1.0,
     "Set to 1 if leaning FORWARD makes Pitch go negative."),
    ("invert_left", "Mounting", "Invert left motor", "0/1", 0.0, 0.0, 1.0, 1.0,
     "Set to 1 if the left wheel drives backward when it should go forward."),
    ("invert_right", "Mounting", "Invert right motor", "0/1", 0.0, 0.0, 1.0, 1.0,
     "Set to 1 if the right wheel drives backward when it should go forward."),
]
PARAM_META = {p[0]: p for p in PARAMS}
PARAM_DEFAULTS = {p[0]: p[4] for p in PARAMS}

logger = Logger("julia-x2")
ui = WebUI()
_last_sent = 0.0
_current_action = "Idle"
_speed_scale = 1.0  # user-adjustable fraction of MAX_STEPS_PER_SEC / fwd_max applied to every drive command

# Mirrors the sketch's imuCalibrated flag; the sketch also enforces this independently, but
# gating here too avoids sending drive commands the MCU will silently ignore.
# One of: "idle" (never calibrated this boot), "calibrating", "calibrated", "failed".
_calibration_status = "idle"

# Latest MPU6050 reading pushed from the sketch (Bridge.notify("imu", ...)); see sketch.ino.
_imu = {
    "angle_x": 0.0,
    "angle_y": 0.0,
    "gyro_x": 0.0,
    "gyro_y": 0.0,
    "gyro_z": 0.0,
    "accel_x": 0.0,
    "accel_y": 0.0,
    "accel_z": 0.0,
    "temp": 0.0,
}

# Latest balance telemetry (Bridge.notify("balance", ...)).
_bal = {
    "state": ST_IDLE,
    "fault": 0,
    "mode": MODE_MANUAL,
    "pitch": 0.0,     # deg from the balance point, + = leaning forward
    "rate": 0.0,      # deg/s
    "lean": 0.0,      # deg, lean requested by the speed loop
    "speed": 0.0,     # steps/s, common wheel speed
    "accel": 0.0,     # steps/s^2
    "loop_us": 0,     # worst-case control-cycle duration since arming, us (budget: 5000)
    "overruns": 0,    # control cycles that missed the 5 ms deadline since arming
}
_bal_lock = threading.Lock()
_mcu_pver = None  # MCU's count of set_param calls since boot; a decrease means the MCU rebooted

# Live parameter values (what the MCU is running). Persisted overrides live in TUNING_FILE.
_params = dict(PARAM_DEFAULTS)
_params_lock = threading.Lock()
_params_dirty = False
_params_saved_at = 0.0
_resync_needed = threading.Event()


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------------------
def _load_params():
    try:
        saved = json.loads(TUNING_FILE.read_text())
    except FileNotFoundError:
        return
    except Exception as e:
        logger.warning(f"Ignoring unreadable {TUNING_FILE}: {e}")
        return
    for name, value in saved.items():
        meta = PARAM_META.get(name)
        if meta is None:
            continue
        try:
            _params[name] = _clamp(float(value), meta[5], meta[6])
        except (TypeError, ValueError):
            continue
    logger.info(f"Loaded tuning from {TUNING_FILE}")


def _save_params():
    global _params_dirty, _params_saved_at
    with _params_lock:
        snapshot = {k: v for k, v in _params.items() if v != PARAM_DEFAULTS[k]}
    try:
        TUNING_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = TUNING_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(snapshot, indent=2, sort_keys=True))
        tmp.replace(TUNING_FILE)  # atomic: a power cut never leaves half a file
    except Exception as e:
        logger.exception(f"Could not save tuning: {e}")
        return False
    _params_dirty = False
    _params_saved_at = time.monotonic()
    return True


# ---------------------------------------------------------------------------------------
# Parameter sync with the MCU
# ---------------------------------------------------------------------------------------
def _push_param(name: str, value: float) -> bool:
    try:
        return bool(Bridge.call("set_param", name, float(value)))
    except Exception as e:
        logger.warning(f"set_param({name}) failed: {e}")
        return False


def _push_all_params():
    with _params_lock:
        items = list(_params.items())
    bad = [n for n, v in items if not _push_param(n, v)]
    if bad:
        logger.warning(f"MCU rejected/failed parameters: {bad}")
    else:
        logger.info(f"Pushed {len(items)} tuning parameters to the MCU")


def _params_payload():
    with _params_lock:
        values = dict(_params)
    return {
        "values": values,
        "defaults": PARAM_DEFAULTS,
        "meta": [
            {"name": p[0], "group": p[1], "label": p[2], "unit": p[3], "min": p[5],
             "max": p[6], "step": p[7], "help": p[8]}
            for p in PARAMS
        ],
        "saved": not _params_dirty,
    }


def _broadcast_params(sid: str | None = None):
    ui.send_message("params", _params_payload(), room=sid)


def _notice(text: str, level: str = "info", sid: str | None = None):
    ui.send_message("notice", {"text": text, "level": level}, room=sid)


def _sync_worker():
    """Background housekeeping: re-push parameters after an MCU reboot and debounce-save."""
    global _params_dirty
    while True:
        if _resync_needed.wait(timeout=0.5):
            _resync_needed.clear()
            _push_all_params()
            try:  # a rebooted MCU has lost its calibration; a restarted Python app has lost ours
                calibrated = bool(Bridge.call("imu_status"))
            except Exception:
                calibrated = False
            _set_calibration_status("calibrated" if calibrated else "idle")
            _broadcast_params()
        if _params_dirty and time.monotonic() - _params_saved_at > SAVE_DEBOUNCE_S:
            if _save_params():
                ui.send_message("params_saved", {"saved": True})


# ---------------------------------------------------------------------------------------
# Driving
# ---------------------------------------------------------------------------------------
def _drive(left_speed: float, right_speed: float):
    Bridge.notify("drive", int(left_speed), int(right_speed))


def _action_label(x: float, y: float) -> str:
    if (x * x + y * y) ** 0.5 < DEADZONE:
        return "Stopped"
    if abs(y) >= abs(x):
        return "Driving forward" if y > 0 else "Driving backward"
    return "Turning right" if x > 0 else "Turning left"


def _set_action(label: str, sid: str | None = None):
    global _current_action
    if label == _current_action and sid is None:
        return
    _current_action = label
    ui.send_message("action", {"text": label}, room=sid)


def _set_speed_scale(scale: float, sid: str | None = None):
    global _speed_scale
    scale = _clamp(scale, MIN_SPEED_SCALE, MAX_SPEED_SCALE)
    if scale == _speed_scale and sid is None:
        return
    _speed_scale = scale
    ui.send_message("speed_scale", {"scale": _speed_scale}, room=sid)


def _broadcast_speed_status(left: float, right: float):
    ui.send_message(
        "speed_status",
        {
            "left": round(left),
            "right": round(right),
            "left_pct": round(left / MAX_STEPS_PER_SEC * 100),
            "right_pct": round(right / MAX_STEPS_PER_SEC * 100),
        },
    )


def on_joystick(sid, data):
    global _last_sent
    mode, state = _bal["mode"], _bal["state"]
    if mode == MODE_BALANCE and _calibration_status != "calibrated":
        return  # balancing needs a calibrated IMU; manual driving does not

    now = time.monotonic()
    if now - _last_sent < 1.0 / JOYSTICK_SEND_HZ:
        return
    _last_sent = now

    x = _clamp(float(data.get("x", 0.0)), -1.0, 1.0)  # -1 left, +1 right
    y = _clamp(float(data.get("y", 0.0)), -1.0, 1.0)  # -1 back, +1 forward
    if (x * x + y * y) ** 0.5 < DEADZONE:
        x = y = 0.0

    if mode == MODE_BALANCE:
        if state not in (ST_ARMED, ST_BALANCING):
            return  # balance mode but not started: the wheels hold still

        # The MCU's speed loop turns these normalised commands into lean angles.
        Bridge.notify("balance_cmd", float(y * _speed_scale), float(x))
        _set_action(_action_label(x, y))
        return

    # Manual mode: direct differential drive of the wheels. Arcade mixing.
    left = _clamp(y + x, -1.0, 1.0) * MAX_STEPS_PER_SEC * _speed_scale
    right = _clamp(y - x, -1.0, 1.0) * MAX_STEPS_PER_SEC * _speed_scale
    _drive(left, right)
    _set_action(_action_label(x, y))
    _broadcast_speed_status(left, right)


def on_stop(sid, data):
    _drive(0, 0)
    Bridge.notify("balance_cmd", 0.0, 0.0)
    _set_action("Stopped")
    _broadcast_speed_status(0, 0)
    logger.info("Stop requested from joystick UI")


def on_speed(sid, data):
    scale = data.get("scale")
    if scale is None:
        return
    _set_speed_scale(float(scale))
    logger.info(f"Max speed set to {_speed_scale:.0%}")


# ---------------------------------------------------------------------------------------
# Calibration + balance control
# ---------------------------------------------------------------------------------------
def _set_calibration_status(status: str, sid: str | None = None):
    global _calibration_status
    _calibration_status = status
    ui.send_message("calibration_status", {"status": status}, room=sid)


# Websocket handler: dashboard "Calibrate" button. Runs the ~1.5s blocking Bridge.call off the
# socket-handling thread so it doesn't stall other clients/messages while it waits.
def on_calibrate_imu(sid, data):
    if _calibration_status == "calibrating" or _bal["state"] in (ST_ARMED, ST_BALANCING):
        return
    _set_calibration_status("calibrating")
    logger.info("IMU calibration requested from dashboard -- robot must stay still")

    def _run():
        try:
            ok = bool(Bridge.call("calibrate_imu"))
        except Exception as e:
            logger.exception(f"IMU calibration call failed: {e}")
            ok = False
        _set_calibration_status("calibrated" if ok else "failed")
        if not ok:
            _notice("Calibration failed: keep the robot completely still and try again.", "error")
        logger.info(f"IMU calibration {'succeeded' if ok else 'failed'}")

    threading.Thread(target=_run, daemon=True).start()


def _set_param(name: str, value: float, *, broadcast: bool = True) -> bool:
    meta = PARAM_META.get(name)
    if meta is None:
        return False
    value = _clamp(float(value), meta[5], meta[6])
    if not _push_param(name, value):
        return False
    global _params_dirty
    with _params_lock:
        _params[name] = value
    _params_dirty = True
    if broadcast:
        ui.send_message("params_saved", {"saved": False})
        ui.send_message("param", {"name": name, "value": value})
    return True


def on_set_param(sid, data):
    name, value = data.get("name"), data.get("value")
    if name is None or value is None:
        return
    if not _set_param(str(name), float(value)):
        _notice(f"Could not set {name}", "error", sid=sid)


def on_reset_params(sid, data):
    if _bal["state"] in (ST_ARMED, ST_BALANCING):
        _notice("Disarm before resetting the tuning.", "error", sid=sid)
        return
    for name, default in PARAM_DEFAULTS.items():
        _set_param(name, default, broadcast=False)
    _save_params()
    _broadcast_params()
    _notice("Tuning reset to defaults (balance point cleared: set it again).")


def on_save_params(sid, data):
    ok = _save_params()
    ui.send_message("params_saved", {"saved": ok})
    _notice("Tuning saved." if ok else "Could not write the tuning file.", "info" if ok else "error")


def on_capture_trim(sid, data):
    if _calibration_status != "calibrated":
        _notice("Calibrate the IMU first.", "error", sid=sid)
        return
    if _bal["state"] == ST_BALANCING:
        return

    def _run():
        try:
            pitch = float(Bridge.call("capture_trim"))
        except Exception as e:
            logger.exception(f"capture_trim failed: {e}")
            pitch = -999.0
        if pitch < -900:
            _notice("Could not read the balance point.", "error")
            return
        pitch = round(pitch, 2)
        _set_param("trim", pitch)  # the firmware returns the filtered pitch *before* trim is applied
        _notice(f"Balance point set to {pitch:.2f}°")
        _broadcast_params()

    _notice("Hold the robot still at its balance point…")
    threading.Thread(target=_run, daemon=True).start()


def on_set_mode(sid, data):
    want = MODE_BALANCE if data.get("mode") == "balance" else MODE_MANUAL

    def _run():
        try:
            result = int(Bridge.call("set_mode", want))
        except Exception as e:
            logger.exception(f"set_mode failed: {e}")
            result = -1
        if result < 0:
            _notice("Could not change mode.", "error")
        else:
            _bal["mode"] = result  # optimistic; the next telemetry frame confirms it
            _set_action("Stopped")
            _drive(0, 0)
            ui.send_message("balance", _balance_payload())
        logger.info(f"set_mode({want}) -> {result}")

    threading.Thread(target=_run, daemon=True).start()


def on_balance_arm(sid, data):
    want = bool(data.get("on", False))
    if want and _bal["mode"] != MODE_BALANCE:
        _notice("Switch to self-balancing mode first.", "error", sid=sid)
        return
    if want and _calibration_status != "calibrated":
        _notice("Calibrate the IMU first.", "error", sid=sid)
        return

    def _run():
        try:
            result = int(Bridge.call("balance_arm", want))
        except Exception as e:
            logger.exception(f"balance_arm failed: {e}")
            result = -1
        if result < 0:
            _notice("Cannot arm now (calibrate first, and clear any fault by disarming).", "error")
        logger.info(f"balance_arm({want}) -> {result}")

    threading.Thread(target=_run, daemon=True).start()


def on_connect(sid):
    _set_action(_current_action, sid=sid)
    _set_speed_scale(_speed_scale, sid=sid)
    ui.send_message("imu", _imu, room=sid)
    ui.send_message("balance", _balance_payload(), room=sid)
    ui.send_message("calibration_status", {"status": _calibration_status}, room=sid)
    _broadcast_params(sid)


# ---------------------------------------------------------------------------------------
# Telemetry from the MCU
# ---------------------------------------------------------------------------------------
def _balance_payload():
    return dict(_bal, fault_text=FAULT_TEXT.get(_bal["fault"], "Unknown fault"))


# Bridge handler: called from the sketch via Bridge.notify("balance", ...) at 20 Hz.
def on_balance(state, fault, pitch, rate, lean, speed, accel, loop_us, overruns, pver, mode):
    global _mcu_pver
    prev_state = _bal["state"]
    with _bal_lock:
        _bal.update(
            state=int(state), fault=int(fault), mode=int(mode), pitch=round(pitch, 2), rate=round(rate, 1),
            lean=round(lean, 2), speed=round(speed), accel=round(accel),
            loop_us=int(loop_us), overruns=int(overruns),
        )
    pver = int(pver)
    if _mcu_pver is None or pver < _mcu_pver:
        # First telemetry after this app started, or the MCU rebooted: (re)load its parameters.
        _resync_needed.set()
    _mcu_pver = pver
    if int(state) != prev_state:
        if int(state) == ST_FAULT:
            _set_action("FAULT")
            _notice(f"Motors off: {FAULT_TEXT.get(int(fault), 'fault')}. Press Disarm to reset.", "error")
        elif int(state) == ST_BALANCING:
            _set_action("Balancing")
        elif int(state) == ST_IDLE:
            _set_action("Stopped")
    ui.send_message("balance", _balance_payload())


# Bridge handler: called from the sketch via Bridge.notify("imu", ...) at ~5 Hz.
def on_imu(angle_x, angle_y, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp):
    global _imu
    _imu = {
        "angle_x": round(angle_x, 2),
        "angle_y": round(angle_y, 2),
        "gyro_x": round(gyro_x, 2),
        "gyro_y": round(gyro_y, 2),
        "gyro_z": round(gyro_z, 2),
        "accel_x": round(accel_x, 3),
        "accel_y": round(accel_y, 3),
        "accel_z": round(accel_z, 3),
        "temp": round(temp, 1),
    }
    ui.send_message("imu", _imu)


_load_params()

ui.on_message("joystick", on_joystick)
ui.on_message("stop", on_stop)
ui.on_message("speed", on_speed)
ui.on_message("calibrate_imu", on_calibrate_imu)
ui.on_message("set_param", on_set_param)
ui.on_message("reset_params", on_reset_params)
ui.on_message("save_params", on_save_params)
ui.on_message("capture_trim", on_capture_trim)
ui.on_message("balance_arm", on_balance_arm)
ui.on_message("set_mode", on_set_mode)
ui.on_connect(on_connect)
Bridge.provide("imu", on_imu)
Bridge.provide("balance", on_balance)

threading.Thread(target=_sync_worker, daemon=True).start()

App.run()
