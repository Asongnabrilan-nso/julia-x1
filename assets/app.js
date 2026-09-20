// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

const ui = new WebUI();

const statusEl = document.querySelector('#status');
const actionEl = document.querySelector('#action');
const directionEl = document.querySelector('#direction');
const base = document.querySelector('#joystick-base');
const knob = document.querySelector('#joystick-knob');
const stopBtn = document.querySelector('#stop-btn');
const speedSlider = document.querySelector('#speed-slider');
const speedValueEl = document.querySelector('#speed-value');
const speedDownBtn = document.querySelector('#speed-down');
const speedUpBtn = document.querySelector('#speed-up');
const leftFillEl = document.querySelector('#left-fill');
const rightFillEl = document.querySelector('#right-fill');
const leftValueEl = document.querySelector('#left-value');
const rightValueEl = document.querySelector('#right-value');
const tiltDotEl = document.querySelector('#tilt-dot');
const imuAngleXEl = document.querySelector('#imu-angle-x');
const imuAngleYEl = document.querySelector('#imu-angle-y');
const imuGyroXEl = document.querySelector('#imu-gyro-x');
const imuGyroYEl = document.querySelector('#imu-gyro-y');
const imuGyroZEl = document.querySelector('#imu-gyro-z');
const imuAccelEl = document.querySelector('#imu-accel');
const imuTempEl = document.querySelector('#imu-temp');
const calibrateBtn = document.querySelector('#calibrate-btn');
const calibrateStatusEl = document.querySelector('#calibrate-status');
const calibBannerEl = document.querySelector('#calib-banner');

let controlMode = 'manual'; // 'manual' | 'balance', mirrors the MCU (telemetry is authoritative)
let balStateForLocks = 0;
let imuCalibrated = false; // mirrors the sketch's imuCalibrated gate; blocks joystick input client-side too

const SEND_INTERVAL_MS = 50; // client-side throttle for outgoing joystick updates
const SPEED_SEND_INTERVAL_MS = 100; // client-side throttle while dragging the speed slider
const SPEED_STEP = 5; // percentage points per +/- button tap

const KEEPALIVE_MS = 100; // re-send the held stick position; the MCU zeroes commands after 500 ms of silence

let dragging = false;
let pointerId = null;
let lastSent = 0;
let lastSpeedSent = 0;
let lastXY = { x: 0, y: 0 };
let keepAliveTimer = null;

ui.on_connect(() => {
  statusEl.textContent = 'Connected';
  statusEl.className = 'status status--online';
});

ui.on_disconnect(() => {
  statusEl.textContent = 'Disconnected';
  statusEl.className = 'status status--offline';
  resetKnob();
  updateGauge(leftFillEl, leftValueEl, 0, 0);
  updateGauge(rightFillEl, rightValueEl, 0, 0);
});

// Server-confirmed robot action, pushed by the Python app from the actual drive command sent to the MCU.
ui.on_message('action', (data) => {
  const text = data?.text ?? 'Idle';
  actionEl.textContent = text;
  actionEl.classList.toggle('action-badge--active', text !== 'Stopped' && text !== 'Idle');
});

function setSpeedDisplay(percent) {
  speedSlider.value = percent;
  speedValueEl.textContent = `${percent}%`;
}

function sendSpeed(percent, { immediate = false } = {}) {
  setSpeedDisplay(percent);
  const now = performance.now();
  if (!immediate && now - lastSpeedSent < SPEED_SEND_INTERVAL_MS) return;
  lastSpeedSent = now;
  ui.send_message('speed', { scale: percent / 100 });
}

speedSlider.addEventListener('input', () => sendSpeed(Number(speedSlider.value)));
speedSlider.addEventListener('change', () => sendSpeed(Number(speedSlider.value), { immediate: true }));

speedDownBtn.addEventListener('click', () => {
  const next = Math.max(Number(speedSlider.min), Number(speedSlider.value) - SPEED_STEP);
  sendSpeed(next, { immediate: true });
});

speedUpBtn.addEventListener('click', () => {
  const next = Math.min(Number(speedSlider.max), Number(speedSlider.value) + SPEED_STEP);
  sendSpeed(next, { immediate: true });
});

// Authoritative max-speed setting, pushed by the Python app (covers changes from other connected clients too).
ui.on_message('speed_scale', (data) => {
  const percent = Math.round((data?.scale ?? 1) * 100);
  setSpeedDisplay(percent);
});

function updateGauge(fillEl, valueEl, pct, steps) {
  const clamped = Math.max(-100, Math.min(100, pct ?? 0));
  const width = Math.abs(clamped) / 2;
  const left = clamped >= 0 ? 50 : 50 + clamped / 2;
  fillEl.style.left = `${left}%`;
  fillEl.style.width = `${width}%`;
  fillEl.classList.toggle('motor-gauge__fill--reverse', clamped < 0);
  valueEl.textContent = `${steps ?? 0}`;
}

// Live left/right motor speed dashboard (steps/sec), pushed by the Python app from the actual drive command.
ui.on_message('speed_status', (data) => {
  updateGauge(leftFillEl, leftValueEl, data?.left_pct, data?.left);
  updateGauge(rightFillEl, rightValueEl, data?.right_pct, data?.right);
});

// Tilt dot reaches the edge of its ring at this many degrees.
const TILT_MAX_DEG = 45;
const TILT_RING_RADIUS_PX = 38;

function updateTiltDot(angleX, angleY) {
  const clampedX = Math.max(-TILT_MAX_DEG, Math.min(TILT_MAX_DEG, angleX ?? 0));
  const clampedY = Math.max(-TILT_MAX_DEG, Math.min(TILT_MAX_DEG, angleY ?? 0));
  const dx = (clampedX / TILT_MAX_DEG) * TILT_RING_RADIUS_PX;
  const dy = (clampedY / TILT_MAX_DEG) * TILT_RING_RADIUS_PX;
  tiltDotEl.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
}

const CALIBRATE_LABELS = {
  idle: { status: 'Not calibrated', btn: 'Calibrate', btnDisabled: false },
  calibrating: { status: 'Calibrating…', btn: 'Calibrating…', btnDisabled: true },
  calibrated: { status: 'Calibrated ✓', btn: 'Recalibrate', btnDisabled: false },
  failed: { status: 'Calibration failed', btn: 'Retry', btnDisabled: false },
};

// Calibration state, driven by the sketch's IMU calibration gate on drive commands.
ui.on_message('calibration_status', (data) => {
  const state = CALIBRATE_LABELS[data?.status] ?? CALIBRATE_LABELS.idle;
  imuCalibrated = data?.status === 'calibrated';

  calibrateStatusEl.textContent = state.status;
  calibrateStatusEl.className = `calibrate-status calibrate-status--${data?.status ?? 'idle'}`;
  calibrateBtn.textContent = state.btn;
  calibrateBtn.disabled = state.btnDisabled;

  updateLocks();
});

calibrateBtn.addEventListener('click', () => {
  ui.send_message('calibrate_imu');
});

// Live IMU dashboard (MPU6050 on Wire1), pushed by the Python app from Bridge.notify("imu", ...).
ui.on_message('imu', (data) => {
  const angleX = data?.angle_x ?? 0;
  const angleY = data?.angle_y ?? 0;
  imuAngleXEl.textContent = `${angleX.toFixed(1)}°`;
  imuAngleYEl.textContent = `${angleY.toFixed(1)}°`;
  imuGyroXEl.textContent = `${(data?.gyro_x ?? 0).toFixed(1)}°/s`;
  imuGyroYEl.textContent = `${(data?.gyro_y ?? 0).toFixed(1)}°/s`;
  imuGyroZEl.textContent = `${(data?.gyro_z ?? 0).toFixed(1)}°/s`;
  imuAccelEl.textContent = `${(data?.accel_x ?? 0).toFixed(2)}/${(data?.accel_y ?? 0).toFixed(2)}/${(data?.accel_z ?? 0).toFixed(2)} g`;
  imuTempEl.textContent = `${(data?.temp ?? 0).toFixed(1)}°C`;
  updateTiltDot(angleX, angleY);
});

function directionLabel(x, y) {
  if (Math.hypot(x, y) < 0.15) return 'STOP';
  if (Math.abs(y) >= Math.abs(x)) return y > 0 ? 'FORWARD' : 'BACKWARD';
  return x > 0 ? 'RIGHT' : 'LEFT';
}

function sendJoystick(x, y) {
  lastXY = { x, y };
  directionEl.textContent = directionLabel(x, y);

  const now = performance.now();
  if (now - lastSent < SEND_INTERVAL_MS) return;
  lastSent = now;
  ui.send_message('joystick', { x, y });
}

function resetKnob() {
  clearInterval(keepAliveTimer);
  keepAliveTimer = null;
  lastXY = { x: 0, y: 0 };
  knob.style.transform = 'translate(-50%, -50%)';
  knob.classList.remove('active');
  directionEl.textContent = 'STOP';
  ui.send_message('stop');
}

function updateKnob(clientX, clientY) {
  const rect = base.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const maxRadius = rect.width / 2 - knob.offsetWidth / 2;

  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxRadius) {
    dx = (dx / dist) * maxRadius;
    dy = (dy / dist) * maxRadius;
  }

  knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

  const x = dx / maxRadius;
  const y = -dy / maxRadius; // screen Y grows downward; forward is "up"
  sendJoystick(x, y);
}

base.addEventListener('pointerdown', (e) => {
  dragging = true;
  pointerId = e.pointerId;
  base.setPointerCapture(pointerId);
  knob.classList.add('active');
  updateKnob(e.clientX, e.clientY);
  clearInterval(keepAliveTimer);
  keepAliveTimer = setInterval(() => ui.send_message('joystick', lastXY), KEEPALIVE_MS);
});

base.addEventListener('pointermove', (e) => {
  if (!dragging || e.pointerId !== pointerId) return;
  updateKnob(e.clientX, e.clientY);
});

function endDrag(e) {
  if (!dragging || (pointerId !== null && e.pointerId !== pointerId)) return;
  dragging = false;
  pointerId = null;
  resetKnob();
}

base.addEventListener('pointerup', endDrag);
base.addEventListener('pointercancel', endDrag);
base.addEventListener('pointerleave', (e) => {
  if (dragging) endDrag(e);
});

stopBtn.addEventListener('click', resetKnob);


// =====================================================================================
// Balancing: control bar, live scope, tuning panel
// =====================================================================================
const ST = { IDLE: 0, ARMED: 1, BALANCING: 2, FAULT: 3 };

const balStateEl = document.querySelector('#bal-state');
const balPitchEl = document.querySelector('#bal-pitch');
const balHintEl = document.querySelector('#bal-hint');
const armBtn = document.querySelector('#arm-btn');
const tuneEl = document.querySelector('#tune');
const tuneOpenBtn = document.querySelector('#tune-open');
const tuneCloseBtn = document.querySelector('#tune-close');
const tuneSavedEl = document.querySelector('#tune-saved');
const paramGroupsEl = document.querySelector('#param-groups');
const scopeEl = document.querySelector('#scope');
const toastEl = document.querySelector('#toast');
const lv = {
  pitch: document.querySelector('#lv-pitch'),
  rate: document.querySelector('#lv-rate'),
  lean: document.querySelector('#lv-lean'),
  speed: document.querySelector('#lv-speed'),
  loop: document.querySelector('#lv-loop'),
};

let balState = ST.IDLE;

const STATE_UI = {
  [ST.IDLE]: { label: 'Idle', cls: 'idle', btn: 'Start balancing', hint: 'Hold the robot upright at its balance point, then press Start.' },
  [ST.ARMED]: { label: 'Armed', cls: 'armed', btn: 'Cancel', hint: 'Hold it upright and still — motors engage automatically.' },
  [ST.BALANCING]: { label: 'Balancing', cls: 'balancing', btn: 'Stop balancing', hint: 'Balancing. Drive with the joystick; STOP just halts motion.' },
  [ST.FAULT]: { label: 'Fault', cls: 'fault', btn: 'Reset fault', hint: 'Motors are off. Pick the robot up and press Reset.' },
};

function renderBalanceState(data) {
  balState = data?.state ?? ST.IDLE;
  const ui_ = STATE_UI[balState] ?? STATE_UI[ST.IDLE];
  balStateEl.textContent = ui_.label;
  balStateEl.className = `bal-state bal-state--${ui_.cls}`;
  armBtn.textContent = ui_.btn;
  armBtn.classList.toggle('arm-btn--stop', balState !== ST.IDLE);
  armBtn.disabled = balState === ST.IDLE && !imuCalibrated;
  balHintEl.textContent = balState === ST.FAULT && data?.fault_text ? `${data.fault_text}. ${ui_.hint}` : ui_.hint;
}

armBtn.addEventListener('click', () => {
  ui.send_message('balance_arm', { on: balState === ST.IDLE });
});

// ---- scope -------------------------------------------------------------------------
const SCOPE_POINTS = 240; // 12 s at 20 Hz
const scope = { pitch: [], lean: [], speed: [] };
const SCOPE_RANGE_DEG = 15;

function pushScope(key, value) {
  const arr = scope[key];
  arr.push(value);
  if (arr.length > SCOPE_POINTS) arr.shift();
}

function drawScope() {
  const ctx = scopeEl.getContext('2d');
  const w = scopeEl.width;
  const h = scopeEl.height;
  const mid = h / 2;
  const scaleY = (h / 2 - 6) / SCOPE_RANGE_DEG;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = '#2e2e2e';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#8a8a8a';
  ctx.font = '11px system-ui, sans-serif';
  for (const deg of [-10, -5, 0, 5, 10]) {
    const y = mid - deg * scaleY;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.strokeStyle = deg === 0 ? '#545b62' : '#2a2a2a';
    ctx.stroke();
    ctx.fillText(`${deg}`, 4, y - 2);
  }

  const series = [
    ['pitch', '#00b8be', 1],
    ['lean', '#e6b800', 1],
    ['speed', '#d66bd6', 1], // already scaled to "thousands of steps/s" on push
  ];
  for (const [key, color] of series) {
    const arr = scope[key];
    if (arr.length < 2) continue;
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = key === 'pitch' ? 2 : 1.4;
    arr.forEach((v, i) => {
      const x = (i / (SCOPE_POINTS - 1)) * w;
      const y = mid - Math.max(-SCOPE_RANGE_DEG, Math.min(SCOPE_RANGE_DEG, v)) * scaleY;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

// ---- telemetry ---------------------------------------------------------------------
ui.on_message('balance', (data) => {
  if (!data) return;
  renderBalanceState(data);
  balPitchEl.textContent = `${(data.pitch ?? 0).toFixed(1)}°`;

  pushScope('pitch', data.pitch ?? 0);
  pushScope('lean', data.lean ?? 0);
  pushScope('speed', (data.speed ?? 0) / 1000);

  if (!tuneEl.hidden) {
    lv.pitch.textContent = `${(data.pitch ?? 0).toFixed(2)}°`;
    lv.rate.textContent = `${Math.round(data.rate ?? 0)}°/s`;
    lv.lean.textContent = `${(data.lean ?? 0).toFixed(2)}°`;
    lv.speed.textContent = `${Math.round(data.speed ?? 0)}`;
    const loopUs = data.loop_us ?? 0;
    lv.loop.textContent = `${loopUs} µs${data.overruns ? ` (${data.overruns} late)` : ''}`;
    lv.loop.classList.toggle('live-metric--bad', loopUs > 3500 || data.overruns > 0);
    drawScope();
  }
});

// The arm button also depends on calibration; re-render when that changes.
ui.on_message('calibration_status', () => renderBalanceState({ state: balState }));

// ---- notices -----------------------------------------------------------------------
let toastTimer = null;
ui.on_message('notice', (data) => {
  if (!data?.text) return;
  toastEl.textContent = data.text;
  toastEl.className = `toast toast--${data.level ?? 'info'}`;
  toastEl.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toastEl.hidden = true; }, 4500);
});

// ---- tuning panel ------------------------------------------------------------------
const paramInputs = {}; // name -> { range, number }
const PARAM_SEND_INTERVAL_MS = 80;
const lastParamSent = {};

function fmt(value, step) {
  const decimals = step >= 1 ? 0 : Math.min(3, Math.max(0, Math.ceil(-Math.log10(step))));
  return Number(value).toFixed(decimals);
}

function sendParam(name, value, immediate = false) {
  const now = performance.now();
  if (!immediate && now - (lastParamSent[name] ?? 0) < PARAM_SEND_INTERVAL_MS) return;
  lastParamSent[name] = now;
  ui.send_message('set_param', { name, value: Number(value) });
}

function setSavedIndicator(saved) {
  tuneSavedEl.textContent = saved ? 'Saved ✓' : 'Unsaved…';
  tuneSavedEl.classList.toggle('tune__saved--dirty', !saved);
}

function buildParams(payload) {
  paramGroupsEl.textContent = '';
  const groups = new Map();
  for (const m of payload.meta) {
    if (!groups.has(m.group)) groups.set(m.group, []);
    groups.get(m.group).push(m);
  }
  for (const [group, items] of groups) {
    const box = document.createElement('details');
    box.className = 'pgroup';
    box.open = group === 'Balance';
    const title = document.createElement('summary');
    title.textContent = group;
    box.append(title);

    for (const m of items) {
      const row = document.createElement('div');
      row.className = 'prow';

      const head = document.createElement('div');
      head.className = 'prow__head';
      const label = document.createElement('span');
      label.className = 'prow__label';
      label.textContent = m.label;
      const unit = document.createElement('span');
      unit.className = 'prow__unit';
      unit.textContent = m.unit;
      head.append(label, unit);

      const controls = document.createElement('div');
      controls.className = 'prow__controls';
      const range = document.createElement('input');
      range.type = 'range';
      range.min = m.min;
      range.max = m.max;
      range.step = m.step;
      const number = document.createElement('input');
      number.type = 'number';
      number.min = m.min;
      number.max = m.max;
      number.step = m.step;
      number.inputMode = 'decimal';
      const reset = document.createElement('button');
      reset.type = 'button';
      reset.className = 'prow__reset';
      reset.textContent = '↺';
      reset.title = `Default: ${fmt(payload.defaults[m.name], m.step)}`;
      controls.append(range, number, reset);

      const help = document.createElement('p');
      help.className = 'prow__help';
      help.textContent = m.help;

      const value = payload.values[m.name];
      range.value = value;
      number.value = fmt(value, m.step);

      range.addEventListener('input', () => {
        number.value = fmt(range.value, m.step);
        sendParam(m.name, range.value);
      });
      range.addEventListener('change', () => sendParam(m.name, range.value, true));
      number.addEventListener('change', () => {
        const v = Math.max(m.min, Math.min(m.max, Number(number.value)));
        if (Number.isNaN(v)) return;
        range.value = v;
        number.value = fmt(v, m.step);
        sendParam(m.name, v, true);
      });
      reset.addEventListener('click', () => {
        const v = payload.defaults[m.name];
        range.value = v;
        number.value = fmt(v, m.step);
        sendParam(m.name, v, true);
      });

      paramInputs[m.name] = { range, number, step: m.step };
      row.append(head, controls, help);
      box.append(row);
    }
    paramGroupsEl.append(box);
  }
  setSavedIndicator(payload.saved);
}

ui.on_message('params', (payload) => {
  if (!payload?.meta) return;
  syncAvoid('avoid_en', payload.values?.avoid_en);
  syncAvoid('avoid_mm', payload.values?.avoid_mm);
  // Rebuild only when the panel is empty; otherwise just refresh values so open groups stay open.
  if (!Object.keys(paramInputs).length) {
    buildParams(payload);
    return;
  }
  for (const [name, value] of Object.entries(payload.values)) {
    const p = paramInputs[name];
    if (!p) continue;
    p.range.value = value;
    p.number.value = fmt(value, p.step);
  }
  setSavedIndicator(payload.saved);
});

// Authoritative single-value update (also covers "Set balance point" and other connected clients).
ui.on_message('param', (data) => {
  syncAvoid(data?.name, data?.value);
  const p = paramInputs[data?.name];
  if (!p) return;
  p.range.value = data.value;
  p.number.value = fmt(data.value, p.step);
});

ui.on_message('params_saved', (data) => setSavedIndicator(!!data?.saved));

tuneOpenBtn.addEventListener('click', () => {
  tuneEl.hidden = false;
  drawScope();
});
tuneCloseBtn.addEventListener('click', () => { tuneEl.hidden = true; });
document.querySelector('#tune-calibrate').addEventListener('click', () => ui.send_message('calibrate_imu'));
document.querySelector('#tune-trim').addEventListener('click', () => ui.send_message('capture_trim'));
document.querySelector('#tune-save').addEventListener('click', () => ui.send_message('save_params'));
document.querySelector('#tune-reset').addEventListener('click', () => {
  if (confirm('Reset ALL tuning values (including the balance point) to defaults?')) {
    ui.send_message('reset_params');
  }
});

renderBalanceState({ state: ST.IDLE });


// =====================================================================================
// Control modes: Manual (direct wheel control) vs Self-balancing
// =====================================================================================
const modeManualBtn = document.querySelector('#mode-manual');
const modeBalanceBtn = document.querySelector('#mode-balance');
const modeHintEl = document.querySelector('#mode-hint');
const balanceBarEl = document.querySelector('#balance-bar');

const MODE_HINTS = {
  manual: 'Direct wheel control. The robot does not balance — keep it on a stand or use it as a rover.',
  balance: 'The robot balances itself. Calibrate, set the balance point, then Start balancing and drive.',
};

// The joystick is live when: manual mode (always), or balance mode while balancing/armed.
function updateLocks() {
  const balancing = controlMode === 'balance';
  const live = !balancing || (imuCalibrated && (balStateForLocks === ST.BALANCING || balStateForLocks === ST.ARMED));
  base.classList.toggle('joystick-base--locked', !live);
  calibBannerEl.hidden = !(balancing && !imuCalibrated);
  if (!live && dragging) endDrag({ pointerId });
}

function renderMode(mode) {
  controlMode = mode === 'balance' ? 'balance' : 'manual';
  modeManualBtn.classList.toggle('mode-switch__btn--active', controlMode === 'manual');
  modeBalanceBtn.classList.toggle('mode-switch__btn--active', controlMode === 'balance');
  modeManualBtn.setAttribute('aria-selected', controlMode === 'manual');
  modeBalanceBtn.setAttribute('aria-selected', controlMode === 'balance');
  modeHintEl.textContent = MODE_HINTS[controlMode];
  balanceBarEl.hidden = controlMode !== 'balance';
  updateLocks();
}

function requestMode(mode) {
  if (mode === controlMode) return;
  if (mode === 'manual' && balState === ST.BALANCING &&
      !confirm('Switching to manual stops balancing — the robot will fall. Continue?')) return;
  ui.send_message('set_mode', { mode });
}

modeManualBtn.addEventListener('click', () => requestMode('manual'));
modeBalanceBtn.addEventListener('click', () => requestMode('balance'));

// Telemetry is the source of truth for both mode and state (also covers other connected clients).
ui.on_message('balance', (data) => {
  if (!data) return;
  balStateForLocks = data.state ?? 0;
  renderMode(data.mode === 1 ? 'balance' : 'manual');
});

renderMode('manual');


// =====================================================================================
// VL53L0X obstacle sensor: live reading + avoidance settings. The avoidance itself runs on the
// MCU; these controls only edit the avoid_en / avoid_mm parameters (persisted like all tuning).
// =====================================================================================
const TOF_SCALE_MM = 1200; // sensor's reliable range, right edge of the bar
const TOF_SLOW_BAND = 1.5; // must match AVOID_SLOW_BAND in sketch.ino (1 + 0.5)
const tofStatusEl = document.querySelector('#tof-status');
const tofMmEl = document.querySelector('#tof-mm');
const tofUnitEl = document.querySelector('#tof-unit');
const tofFactorEl = document.querySelector('#tof-factor');
const tofSlowEl = document.querySelector('#tof-slow');
const tofStopEl = document.querySelector('#tof-stop');
const tofMarkerEl = document.querySelector('#tof-marker');
const tofEnableEl = document.querySelector('#tof-enable');
const tofThrEl = document.querySelector('#tof-thr');
const tofThrValEl = document.querySelector('#tof-thr-val');

let avoidThr = 300;
let tofLast = null;

function renderTofBar() {
  const pct = (mm) => `${Math.min(100, (mm / TOF_SCALE_MM) * 100)}%`;
  tofStopEl.style.width = pct(avoidThr);
  tofSlowEl.style.width = pct(avoidThr * TOF_SLOW_BAND);
  const mm = tofLast?.mm ?? -1;
  tofMarkerEl.hidden = mm < 0;
  if (mm >= 0) tofMarkerEl.style.left = pct(mm);
}

function syncAvoid(name, value) {
  if (value === undefined || value === null) return;
  if (name === 'avoid_mm') {
    avoidThr = Number(value);
    tofThrEl.value = avoidThr;
    tofThrValEl.textContent = avoidThr;
    renderTofBar();
  } else if (name === 'avoid_en') {
    tofEnableEl.checked = Number(value) >= 0.5;
  }
}

tofThrEl.addEventListener('input', () => {
  avoidThr = Number(tofThrEl.value);
  tofThrValEl.textContent = avoidThr;
  renderTofBar();
  sendParam('avoid_mm', avoidThr);
});
tofThrEl.addEventListener('change', () => sendParam('avoid_mm', tofThrEl.value, true));
tofEnableEl.addEventListener('change', () => sendParam('avoid_en', tofEnableEl.checked ? 1 : 0, true));

ui.on_message('tof', (data) => {
  if (!data) return;
  tofLast = data;
  const ok = data.status === 1 || data.status === 2;
  tofStatusEl.textContent = data.status_text ?? '';
  tofStatusEl.className = `tof-status ${ok ? 'tof-status--ok' : 'tof-status--bad'}`;
  tofMmEl.textContent = data.status === 1 ? data.mm : (data.status === 2 ? '>1200' : '--');
  const blocking = ok && data.factor < 100 && tofEnableEl.checked;
  tofFactorEl.textContent = !ok && data.status !== 0 && tofEnableEl.checked
    ? 'forward blocked'
    : (blocking ? (data.factor === 0 ? 'STOP' : `fwd ${data.factor}%`) : '');
  tofFactorEl.classList.toggle('tof-panel__factor--stop', data.factor === 0 && tofEnableEl.checked && data.status !== 0);
  tofMmEl.classList.toggle('tof-mm--stop', blocking && data.factor === 0);
  renderTofBar();
});
renderTofBar();
