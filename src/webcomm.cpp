// module:  webcomm.cpp
// layer:   7 — network I/O only
// purpose: WiFi AP/STA bring-up, HTTP server, WebSocket telemetry + command parsing
// date:    2025

#include "webcomm.h"
#include "project_wide_defs.h"
#include <WiFi.h>
#include <Arduino.h>
#include <cmath>

// ── Embedded HTML ─────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>BB-8 Controller</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:        #0a0c10;
    --surface:   #13161d;
    --surface2:  #1c2030;
    --border:    #2a2f40;
    --accent:    #3b8ef3;
    --accent2:   #f0a500;
    --ok:        #22c55e;
    --warn:      #f59e0b;
    --err:       #ef4444;
    --text:      #e2e8f0;
    --text-dim:  #64748b;
    --radius:    12px;
    --font:      'Inter', system-ui, sans-serif;
  }

  body {
    background: var(--bg); color: var(--text); font-family: var(--font);
    font-size: 14px; min-height: 100dvh; display: flex; flex-direction: column;
    padding: 12px; gap: 12px; -webkit-tap-highlight-color: transparent;
  }

  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 14px 16px; background: var(--surface); border: 1px solid var(--border);
    border-radius: var(--radius);
  }

  .logo { display: flex; align-items: center; gap: 10px; font-weight: 700; font-size: 18px; letter-spacing: 0.5px; }
  .logo svg { flex-shrink: 0; }

  .status-pill {
    display: flex; align-items: center; gap: 6px; font-size: 12px; color: var(--text-dim);
    background: var(--surface2); border: 1px solid var(--border); border-radius: 999px;
    padding: 5px 12px; user-select: none;
  }

  .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--err); transition: background 0.4s; }
  .dot.ok   { background: var(--ok); }
  .dot.warn { background: var(--warn); }

  .card { background: var(--surface); border: 1px solid var(--border); border-radius: var(--radius); overflow: hidden; }

  .card-header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 14px 16px; cursor: pointer; user-select: none; -webkit-user-select: none;
  }
  .card-header:active { background: var(--surface2); }

  .card-title {
    display: flex; align-items: center; gap: 8px; font-weight: 600; font-size: 13px;
    text-transform: uppercase; letter-spacing: 0.8px; color: var(--text-dim);
  }

  .card-title .badge {
    font-size: 10px; background: var(--accent); color: #fff; border-radius: 4px;
    padding: 2px 6px; letter-spacing: 0; text-transform: none; font-weight: 700;
  }

  .chevron { color: var(--text-dim); transition: transform 0.25s; font-size: 18px; line-height: 1; }
  .chevron.open { transform: rotate(180deg); }

  .card-body { border-top: 1px solid var(--border); padding: 16px; display: none; }
  .card-body.open { display: block; }

  .imu-section { margin-bottom: 16px; }
  .imu-section:last-child { margin-bottom: 0; }

  .imu-section-label {
    font-size: 11px; font-weight: 700; text-transform: uppercase;
    letter-spacing: 1px; color: var(--text-dim); margin-bottom: 10px;
  }

  .imu-row { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }

  .imu-cell {
    background: var(--surface2); border: 1px solid var(--border); border-radius: 8px;
    padding: 10px 12px; display: flex; flex-direction: column; gap: 4px;
  }

  .imu-axis { font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.8px; color: var(--text-dim); }
  .imu-val  { font-size: 16px; font-weight: 600; font-variant-numeric: tabular-nums; color: var(--text); transition: color 0.2s; }
  .imu-val.hot { color: var(--accent2); }
  .imu-unit { font-size: 10px; color: var(--text-dim); }

  .bar-track { height: 4px; background: var(--surface2); border-radius: 2px; margin-top: 6px; overflow: hidden; }
  .bar-fill  { height: 100%; border-radius: 2px; background: var(--accent); width: 50%; transition: width 0.1s linear; }

  .divider { height: 1px; background: var(--border); margin: 14px 0; }

  .speed-row { display: flex; align-items: center; gap: 12px; padding: 4px 0; }
  .speed-row input[type="range"] { flex: 1; height: 4px; accent-color: var(--accent); cursor: pointer; }

  .speed-number {
    width: 56px; background: var(--surface2); border: 1px solid var(--border);
    border-radius: 6px; color: var(--text); font-size: 14px; font-weight: 600;
    font-variant-numeric: tabular-nums; text-align: center; padding: 6px 4px;
    -moz-appearance: textfield;
  }
  .speed-number::-webkit-inner-spin-button,
  .speed-number::-webkit-outer-spin-button { -webkit-appearance: none; }
  .speed-pct-label { font-size: 12px; color: var(--text-dim); white-space: nowrap; }

  .joystick-wrap { display: flex; flex-direction: column; align-items: center; gap: 12px; padding: 8px 0; }

  #joystickCanvas { touch-action: none; cursor: grab; border-radius: 50%; display: block; }

  .joy-readout { display: flex; gap: 24px; font-size: 12px; color: var(--text-dim); font-variant-numeric: tabular-nums; }
  .joy-readout span { color: var(--text); font-weight: 600; }

  #stopBtn {
    width: 100%; padding: 16px; border: none; border-radius: 10px;
    background: var(--err); color: #fff; font-size: 15px; font-weight: 700;
    letter-spacing: 0.5px; text-transform: uppercase; cursor: pointer;
    transition: opacity 0.15s, transform 0.1s; -webkit-tap-highlight-color: transparent;
  }
  #stopBtn:active { opacity: 0.8; transform: scale(0.98); }

  .sysinfo { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .sysinfo-item { background: var(--surface2); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; }
  .sysinfo-label { font-size: 10px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.8px; }
  .sysinfo-val   { font-size: 14px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 2px; }

  /* ── Balance panel ── */
  #balEnableBtn {
    padding: 10px 20px; border: 1px solid var(--border); border-radius: 8px;
    background: var(--surface2); color: var(--text); font-size: 13px; font-weight: 600;
    cursor: pointer; transition: background 0.2s, color 0.2s;
  }
  #balEnableBtn.active { background: var(--ok); color: #fff; border-color: var(--ok); }

  #sendGainsBtn {
    width: 100%; padding: 10px; border: none; border-radius: 8px;
    background: var(--accent); color: #fff; font-size: 13px; font-weight: 700;
    cursor: pointer; margin-bottom: 16px; transition: opacity 0.15s;
  }
  #sendGainsBtn:active { opacity: 0.8; }

  .gain-label { font-size: 12px; color: var(--text-dim); width: 42px; flex-shrink: 0; }

  footer { text-align: center; font-size: 11px; color: var(--text-dim); padding: 4px 0 8px; }
</style>
</head>
<body>

<!-- Header -->
<header>
  <div class="logo">
    <svg width="28" height="28" viewBox="0 0 28 28" fill="none">
      <circle cx="14" cy="17" r="10" stroke="#3b8ef3" stroke-width="2"/>
      <circle cx="14" cy="8"  r="5"  stroke="#f0a500" stroke-width="2"/>
      <line x1="14" y1="13" x2="14" y2="7" stroke="#64748b" stroke-width="1"/>
    </svg>
    BB-8
  </div>
  <div class="status-pill">
    <div class="dot" id="wsDot"></div>
    <span id="wsLabel">Connecting…</span>
  </div>
</header>

<!-- IMU Panel -->
<div class="card" id="imuCard">
  <div class="card-header" onclick="toggleCard('imuCard')">
    <div class="card-title">IMU Readings <span class="badge" id="imuBadge">●</span></div>
    <div class="chevron open" id="imuChevron">▾</div>
  </div>
  <div class="card-body open" id="imuBody">
    <div class="imu-section">
      <div class="imu-section-label">Accelerometer — m/s²</div>
      <div class="imu-row">
        <div class="imu-cell"><div class="imu-axis">X</div><div class="imu-val" id="ax">—</div><div class="bar-track"><div class="bar-fill" id="axBar"></div></div></div>
        <div class="imu-cell"><div class="imu-axis">Y</div><div class="imu-val" id="ay">—</div><div class="bar-track"><div class="bar-fill" id="ayBar"></div></div></div>
        <div class="imu-cell"><div class="imu-axis">Z</div><div class="imu-val" id="az">—</div><div class="bar-track"><div class="bar-fill" id="azBar"></div></div></div>
      </div>
    </div>
    <div class="divider"></div>
    <div class="imu-section">
      <div class="imu-section-label">Gyroscope — rad/s</div>
      <div class="imu-row">
        <div class="imu-cell"><div class="imu-axis">X</div><div class="imu-val" id="gx">—</div><div class="bar-track"><div class="bar-fill" id="gxBar"></div></div></div>
        <div class="imu-cell"><div class="imu-axis">Y</div><div class="imu-val" id="gy">—</div><div class="bar-track"><div class="bar-fill" id="gyBar"></div></div></div>
        <div class="imu-cell"><div class="imu-axis">Z</div><div class="imu-val" id="gz">—</div><div class="bar-track"><div class="bar-fill" id="gzBar"></div></div></div>
      </div>
    </div>
    <div class="divider"></div>
    <div class="imu-section">
      <div class="imu-section-label">State Estimate — complementary filter</div>
      <div class="imu-row" style="grid-template-columns:1fr 1fr;">
        <div class="imu-cell">
          <div class="imu-axis">Pitch</div><div class="imu-val" id="estPitch">—</div>
          <div class="imu-unit">deg</div>
          <div class="bar-track"><div class="bar-fill" id="estPitchBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Roll</div><div class="imu-val" id="estRoll">—</div>
          <div class="imu-unit">deg</div>
          <div class="bar-track"><div class="bar-fill" id="estRollBar"></div></div>
        </div>
      </div>
    </div>
  </div>
</div>

<!-- Speed Panel -->
<div class="card" id="speedCard">
  <div class="card-header" onclick="toggleCard('speedCard')">
    <div class="card-title">Speed <span class="badge" id="speedBadge">100%</span></div>
    <div class="chevron open" id="speedChevron">▾</div>
  </div>
  <div class="card-body open" id="speedBody">
    <div class="speed-row">
      <span class="speed-pct-label">0%</span>
      <input type="range" id="speedSlider" min="0" max="100" value="100" step="1">
      <span class="speed-pct-label">100%</span>
      <input type="number" class="speed-number" id="speedInput" min="0" max="100" value="100">
      <span class="speed-pct-label">%</span>
    </div>
  </div>
</div>

<!-- Drive Panel -->
<div class="card" id="driveCard">
  <div class="card-header" onclick="toggleCard('driveCard')">
    <div class="card-title">Drive Control</div>
    <div class="chevron open" id="driveChevron">▾</div>
  </div>
  <div class="card-body open" id="driveBody">
    <div class="joystick-wrap">
      <canvas id="joystickCanvas" width="200" height="200"></canvas>
      <div class="joy-readout">X&nbsp;<span id="joyX">0.00</span>&nbsp;&nbsp;Y&nbsp;<span id="joyY">0.00</span></div>
    </div>
    <div style="margin-top:14px;">
      <button id="stopBtn" onclick="sendStop()">⏹ Emergency Stop</button>
    </div>
  </div>
</div>

<!-- Balance Control Panel (closed by default — enable only after sign verification) -->
<div class="card" id="balCard">
  <div class="card-header" onclick="toggleCard('balCard')">
    <div class="card-title">
      Balance Control
      <span class="badge" id="balBadge" style="background:var(--err);">OFF</span>
    </div>
    <div class="chevron" id="balChevron">▾</div>
  </div>
  <div class="card-body" id="balBody">

    <!-- Enable + E-stop status -->
    <div style="display:flex; align-items:center; gap:12px; margin-bottom:16px; flex-wrap:wrap;">
      <button id="balEnableBtn" onclick="toggleBalance()">Enable Balance</button>
      <div style="display:flex; align-items:center; gap:6px;">
        <div class="dot" id="estopDot"></div>
        <span id="estopLabel" style="font-size:12px; color:var(--text-dim);">E-stop: —</span>
      </div>
    </div>

    <!-- Pitch gains -->
    <div class="imu-section-label">Pitch Gains</div>
    <div class="speed-row" style="margin-bottom:8px; margin-top:10px;">
      <span class="gain-label">Kp</span>
      <input type="range" id="kpPitch" min="0" max="3.0" step="0.05" value="0.50"
             oninput="syncGainNum('kpPitch','kpPitchN')">
      <input type="number" class="speed-number" id="kpPitchN"
             min="0" max="3.0" step="0.05" value="0.50"
             onchange="syncGainSlider('kpPitch','kpPitchN')">
    </div>
    <div class="speed-row" style="margin-bottom:16px;">
      <span class="gain-label">Kd</span>
      <input type="range" id="kdPitch" min="0" max="2.0" step="0.02" value="0.00"
             oninput="syncGainNum('kdPitch','kdPitchN')">
      <input type="number" class="speed-number" id="kdPitchN"
             min="0" max="2.0" step="0.02" value="0.00"
             onchange="syncGainSlider('kdPitch','kdPitchN')">
    </div>

    <!-- Roll gains -->
    <div class="imu-section-label">Roll Gains</div>
    <div class="speed-row" style="margin-bottom:8px; margin-top:10px;">
      <span class="gain-label">Kp</span>
      <input type="range" id="kpRoll" min="0" max="3.0" step="0.05" value="0.30"
             oninput="syncGainNum('kpRoll','kpRollN')">
      <input type="number" class="speed-number" id="kpRollN"
             min="0" max="3.0" step="0.05" value="0.30"
             onchange="syncGainSlider('kpRoll','kpRollN')">
    </div>
    <div class="speed-row" style="margin-bottom:16px;">
      <span class="gain-label">Kd</span>
      <input type="range" id="kdRoll" min="0" max="2.0" step="0.02" value="0.00"
             oninput="syncGainNum('kdRoll','kdRollN')">
      <input type="number" class="speed-number" id="kdRollN"
             min="0" max="2.0" step="0.02" value="0.00"
             onchange="syncGainSlider('kdRoll','kdRollN')">
    </div>

    <button id="sendGainsBtn" onclick="sendGains()">↑ Send Gains to Robot</button>

    <div class="divider"></div>

    <!-- Live balance telemetry -->
    <div class="imu-section-label">Live Telemetry</div>
    <div class="imu-row" style="margin-top:10px;">
      <div class="imu-cell">
        <div class="imu-axis">Pitch Err</div>
        <div class="imu-val" id="balPitchErr">—</div>
        <div class="imu-unit">deg</div>
        <div class="bar-track"><div class="bar-fill" id="balPitchErrBar"></div></div>
      </div>
      <div class="imu-cell">
        <div class="imu-axis">Pitch Rate</div>
        <div class="imu-val" id="balPitchRate">—</div>
        <div class="imu-unit">°/s</div>
        <div class="bar-track"><div class="bar-fill" id="balPitchRateBar"></div></div>
      </div>
      <div class="imu-cell">
        <div class="imu-axis">Correction</div>
        <div class="imu-val" id="balCorrY">—</div>
        <div class="imu-unit">vel</div>
        <div class="bar-track"><div class="bar-fill" id="balCorrYBar"></div></div>
      </div>
    </div>

  </div>
</div>

<!-- System Panel (closed by default) -->
<div class="card" id="sysCard">
  <div class="card-header" onclick="toggleCard('sysCard')">
    <div class="card-title">System</div>
    <div class="chevron" id="sysChevron">▾</div>
  </div>
  <div class="card-body" id="sysBody">
    <div class="sysinfo">
      <div class="sysinfo-item"><div class="sysinfo-label">Uptime</div><div class="sysinfo-val" id="uptime">—</div></div>
      <div class="sysinfo-item"><div class="sysinfo-label">Clients</div><div class="sysinfo-val" id="clients">—</div></div>
      <div class="sysinfo-item"><div class="sysinfo-label">IMU</div><div class="sysinfo-val" id="imuStatus">—</div></div>
      <div class="sysinfo-item"><div class="sysinfo-label">WS Msgs</div><div class="sysinfo-val" id="msgCount">0</div></div>
    </div>
  </div>
</div>

<footer>BB-8 Controller · ESP32-S3</footer>

<script>
// ── WebSocket ────────────────────────────────────────────────────────────────
let ws, msgCount = 0, reconnectTimer;

function connect() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);
  ws.onopen  = () => { setStatus('ok', 'Connected'); clearTimeout(reconnectTimer); };
  ws.onclose = () => { setStatus('warn', 'Reconnecting…'); reconnectTimer = setTimeout(connect, 2000); };
  ws.onerror = () => setStatus('err', 'Error');
  ws.onmessage = (e) => {
    msgCount++;
    document.getElementById('msgCount').textContent = msgCount;
    try { handlePacket(JSON.parse(e.data)); }
    catch (err) { console.error('WS parse error:', err, e.data); }
  };
}

function toFiniteNumber(v, fallback = 0) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

function setStatus(state, label) {
  document.getElementById('wsDot').className = 'dot ' + (state === 'ok' ? 'ok' : state === 'warn' ? 'warn' : '');
  document.getElementById('wsLabel').textContent = label;
}

// ── Packet handler ────────────────────────────────────────────────────────────
function handlePacket(p) {
  if (p.type === 'welcome') {
    if (p.speed_max !== undefined) {
      const m = Number(p.speed_max);
      if (Number.isFinite(m) && m > 0) speedScaleMax = m;
    }
    if (p.speed_scale !== undefined) {
      const s = Number(p.speed_scale);
      if (Number.isFinite(s)) syncSpeed(Math.round((s / speedScaleMax) * 100));
    }
  }

  if (p.type === 'imu') {
    const ax = toFiniteNumber(p.ax), ay = toFiniteNumber(p.ay), az = toFiniteNumber(p.az);
    const gx = toFiniteNumber(p.gx ?? p.gyro_x_rads, 0);
    const gy = toFiniteNumber(p.gy ?? p.gyro_y_rads, 0);
    const gz = toFiniteNumber(p.gz ?? p.gyro_z_rads, 0);
    const pitchDeg = (p.pitch_deg == null) ? NaN : toFiniteNumber(p.pitch_deg, NaN);
    const rollDeg  = (p.roll_deg  == null) ? NaN : toFiniteNumber(p.roll_deg,  NaN);

    setImu('ax', ax, 20); setImu('ay', ay, 20); setImu('az', az, 20);
    setImu('gx', gx, 10, 3); setImu('gy', gy, 10, 3); setImu('gz', gz, 10, 3);

    const badge = document.getElementById('imuBadge');
    badge.style.color = p.valid ? '#22c55e' : '#ef4444';

    setImu('estPitch', pitchDeg, 30, 1);
    setImu('estRoll',  rollDeg,  30, 1);

    document.getElementById('imuStatus').textContent = p.valid ? 'OK' : 'ERR';
    document.getElementById('uptime').textContent    = fmtUptime(p.uptime_ms);
    document.getElementById('clients').textContent   = p.clients ?? '—';

    // ── Balance telemetry ─────────────────────────────────────────────────────
    // Sync enable button if server state differs (e.g., after reconnect)
    if (p.bal_enabled !== undefined) {
      const serverEnabled = !!p.bal_enabled;
      if (serverEnabled !== balanceEnabled) syncBalanceButton(serverEnabled);
    }

    // E-stop indicator
    const estopActive = !!p.bal_estop;
    document.getElementById('estopDot').className    = 'dot ' + (estopActive ? 'warn' : 'ok');
    document.getElementById('estopLabel').textContent = 'E-stop: ' + (estopActive ? 'ACTIVE' : 'OK');

    // Balance telemetry bars — scales match controller reference values
    setImu('balPitchErr',  toFiniteNumber(p.pitch_deg,       0), 30,  1);
    setImu('balPitchRate', toFiniteNumber(p.pitch_rate_degs, 0), 180, 0);
    setImu('balCorrY',     toFiniteNumber(p.bal_corr_y,      0), 1.0, 2);
  }
}

// ── IMU display helpers ───────────────────────────────────────────────────────
function setImu(id, val, maxAbs, decimals = 2) {
  const el  = document.getElementById(id);
  const bar = document.getElementById(id + 'Bar');
  const n   = toFiniteNumber(val, NaN);
  if (!Number.isFinite(n)) {
    el.textContent = '—'; el.classList.remove('hot');
    if (bar) bar.style.width = '50%';
    return;
  }
  el.textContent = n.toFixed(decimals);
  el.classList.toggle('hot', Math.abs(n) > maxAbs * 0.5);
  if (bar) {
    const pct = 50 + (n / maxAbs) * 50;
    bar.style.width = Math.min(100, Math.max(0, pct)) + '%';
  }
}

function fmtUptime(ms) {
  const s = Math.floor(ms / 1000), m = Math.floor(s / 60), h = Math.floor(m / 60);
  if (h > 0) return `${h}h ${m % 60}m`;
  if (m > 0) return `${m}m ${s % 60}s`;
  return `${s}s`;
}

// ── Card collapse ─────────────────────────────────────────────────────────────
function toggleCard(id) {
  const body    = document.getElementById(id.replace('Card', 'Body'));
  const chevron = document.getElementById(id.replace('Card', 'Chevron'));
  const open    = body.classList.toggle('open');
  chevron.classList.toggle('open', open);
}

// ── Joystick ──────────────────────────────────────────────────────────────────
const canvas = document.getElementById('joystickCanvas');
const ctx    = canvas.getContext('2d');
const R = canvas.width / 2, rKnob = 28;
let joyActive = false, joyX = 0, joyY = 0, sendTimer;

function drawJoy() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath(); ctx.arc(R, R, R - 4, 0, Math.PI * 2);
  ctx.strokeStyle = '#2a2f40'; ctx.lineWidth = 2; ctx.stroke();
  ctx.strokeStyle = '#2a2f40'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(R, 4); ctx.lineTo(R, canvas.height - 4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(4, R); ctx.lineTo(canvas.width - 4, R); ctx.stroke();
  const kx = R + joyX * (R - rKnob - 4), ky = R + joyY * (R - rKnob - 4);
  const grad = ctx.createRadialGradient(kx - 4, ky - 4, 2, kx, ky, rKnob);
  grad.addColorStop(0, '#5aa8ff'); grad.addColorStop(1, '#1a5aaa');
  ctx.beginPath(); ctx.arc(kx, ky, rKnob, 0, Math.PI * 2);
  ctx.fillStyle = grad; ctx.fill();
  ctx.strokeStyle = '#3b8ef3'; ctx.lineWidth = 2; ctx.stroke();
}

function joyFromEvent(e) {
  const rect = canvas.getBoundingClientRect();
  const touch = e.touches ? e.touches[0] : e;
  const cx = touch.clientX - rect.left - R, cy = touch.clientY - rect.top - R;
  const max = R - rKnob - 4, dist = Math.sqrt(cx * cx + cy * cy);
  const scale = dist > max ? max / dist : 1;
  joyX = Math.max(-1, Math.min(1, (cx * scale) / max));
  joyY = Math.max(-1, Math.min(1, (cy * scale) / max));
}

function startJoy(e) { e.preventDefault(); joyActive = true; joyFromEvent(e); drawJoy(); scheduleSend(); }
function moveJoy(e)  { if (!joyActive) return; e.preventDefault(); joyFromEvent(e); drawJoy(); updateReadout(); }
function endJoy()    { joyActive = false; joyX = 0; joyY = 0; drawJoy(); updateReadout(); sendDrive(0, 0); }

canvas.addEventListener('mousedown',  startJoy);
canvas.addEventListener('mousemove',  moveJoy);
canvas.addEventListener('mouseup',    endJoy);
canvas.addEventListener('touchstart', startJoy, {passive: false});
canvas.addEventListener('touchmove',  moveJoy,  {passive: false});
canvas.addEventListener('touchend',   endJoy);

function updateReadout() {
  document.getElementById('joyX').textContent = joyX.toFixed(2);
  document.getElementById('joyY').textContent = (-joyY).toFixed(2);
}

function scheduleSend() {
  if (!joyActive) return;
  sendDrive(joyX, -joyY);
  updateReadout();
  sendTimer = setTimeout(scheduleSend, 80);
}

function sendDrive(x, y) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(`CMD:DRIVE:x=${x.toFixed(3)},y=${y.toFixed(3)}`);
}

function sendStop() {
  joyX = 0; joyY = 0; drawJoy(); updateReadout(); clearTimeout(sendTimer);
  if (ws && ws.readyState === WebSocket.OPEN) ws.send('CMD:STOP');
}

// ── Speed control ─────────────────────────────────────────────────────────────
let speedScaleMax = 1.0;

function applySpeed(pct) {
  const v = Math.min(100, Math.max(0, Math.round(pct)));
  document.getElementById('speedSlider').value      = v;
  document.getElementById('speedInput').value       = v;
  document.getElementById('speedBadge').textContent = v + '%';
  if (ws && ws.readyState === WebSocket.OPEN)
    ws.send('CMD:SPEED:v=' + ((v / 100) * speedScaleMax).toFixed(3));
}

function syncSpeed(pct) {
  const v = Math.min(100, Math.max(0, Math.round(pct)));
  document.getElementById('speedSlider').value      = v;
  document.getElementById('speedInput').value       = v;
  document.getElementById('speedBadge').textContent = v + '%';
}

document.getElementById('speedSlider').addEventListener('input',  e => applySpeed(+e.target.value));
document.getElementById('speedInput').addEventListener('change',  e => applySpeed(+e.target.value));

// ── Balance Control ───────────────────────────────────────────────────────────
let balanceEnabled = false;

function syncBalanceButton(en) {
  balanceEnabled = en;
  const btn   = document.getElementById('balEnableBtn');
  const badge = document.getElementById('balBadge');
  if (en) {
    btn.textContent = 'Disable Balance';
    btn.classList.add('active');
    badge.textContent = 'ON';
    badge.style.background = 'var(--ok)';
  } else {
    btn.textContent = 'Enable Balance';
    btn.classList.remove('active');
    badge.textContent = 'OFF';
    badge.style.background = 'var(--err)';
  }
}

function toggleBalance() {
  syncBalanceButton(!balanceEnabled);
  if (ws && ws.readyState === WebSocket.OPEN)
    ws.send('CMD:BALANCE:enable=' + (balanceEnabled ? '1' : '0'));
}

function sendGains() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const kp  = parseFloat(document.getElementById('kpPitch').value).toFixed(3);
  const kd  = parseFloat(document.getElementById('kdPitch').value).toFixed(3);
  const kpr = parseFloat(document.getElementById('kpRoll').value).toFixed(3);
  const kdr = parseFloat(document.getElementById('kdRoll').value).toFixed(3);
  ws.send(`CMD:BALANCE_TUNE:Kp=${kp},Kd=${kd},Kpr=${kpr},Kdr=${kdr}`);
}

function syncGainNum(sliderId, numId) {
  document.getElementById(numId).value = document.getElementById(sliderId).value;
}

function syncGainSlider(sliderId, numId) {
  const slider = document.getElementById(sliderId);
  const v = Math.min(parseFloat(slider.max), Math.max(parseFloat(slider.min),
                parseFloat(document.getElementById(numId).value)));
  slider.value = v;
  document.getElementById(numId).value = v;
}

// ── Init ──────────────────────────────────────────────────────────────────────
drawJoy();
connect();
</script>
</body>
</html>
)rawliteral";

// ── WebComm implementation ────────────────────────────────────────────────────

static WebComm* _instance = nullptr;

bool WebComm::begin(const char* ssid, const char* pass) {
    _instance = this;

    if (ssid == nullptr || ssid[0] == '\0') {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BB8-Robot", "bb8robot1");
        Serial.printf("[WEBCOMM] AP mode. IP: %s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.printf("[WEBCOMM] Connecting to SSID: %s\n", ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, pass);

        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED) {
            delay(250);
            Serial.print('.');
            if (millis() - t0 > 15000) {
                Serial.println("\n[WEBCOMM] WiFi timeout — falling back to AP mode");
                WiFi.mode(WIFI_AP);
                WiFi.softAP("BB8-Robot", "bb8robot1");
                Serial.printf("[WEBCOMM] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
                break;
            }
        }
        if (WiFi.status() == WL_CONNECTED)
            Serial.printf("\n[WEBCOMM] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    }

    _ws.onEvent(_wsEventStatic);
    _server.addHandler(&_ws);

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", INDEX_HTML);
    });
    _server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "Not found");
    });

    _server.begin();
    _connected = true;
    Serial.println("[WEBCOMM] HTTP server started");
    return true;
}

void WebComm::update() {
    _ws.cleanupClients();
}

void WebComm::broadcastState(const RawIMUData& raw, const IMUState& est, const BalanceOutput& bal) {
    if (_ws.count() == 0) return;

    auto safeF = [](float v, int dec) -> String {
        if (std::isnan(v) || std::isinf(v)) return String("0");
        return String(v, dec);
    };

    JsonDocument doc;
    doc["type"]       = "imu";
    doc["ax"]         = serialized(safeF(raw.accel_x_ms2, 3));
    doc["ay"]         = serialized(safeF(raw.accel_y_ms2, 3));
    doc["az"]         = serialized(safeF(raw.accel_z_ms2, 3));
    doc["gx"]         = serialized(safeF(raw.gyro_x_rads, 4));
    doc["gy"]         = serialized(safeF(raw.gyro_y_rads, 4));
    doc["gz"]         = serialized(safeF(raw.gyro_z_rads, 4));
    doc["valid"]      = raw.valid;
    doc["pitch_deg"]  = est.valid ? serialized(safeF(est.pitch_deg, 1)) : serialized(String("null"));
    doc["roll_deg"]   = est.valid ? serialized(safeF(est.roll_deg,  1)) : serialized(String("null"));
    doc["est_valid"]  = est.valid;

    // Balance telemetry — always included; zero when balance is disabled or invalid
    doc["bal_enabled"]     = bal.balance_enabled;
    doc["bal_estop"]       = bal.estop_active;
    doc["bal_corr_y"]      = serialized(safeF(bal.correction_vel_y, 3));
    doc["bal_corr_x"]      = serialized(safeF(bal.correction_vel_x, 3));
    // pitch_rate_degs from balance output (already converted rad→deg in balance_controller.cpp)
    doc["pitch_rate_degs"] = serialized(safeF(bal.pitch_rate_degs, 1));

    doc["uptime_ms"] = millis();
    doc["clients"]   = (uint8_t)_ws.count();

    String out;
    out.reserve(280);
    serializeJson(doc, out);
    _ws.textAll(out);
}

uint8_t WebComm::clientCount() const { return (uint8_t)_ws.count(); }

String WebComm::ipAddress() const {
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    return WiFi.softAPIP().toString();
}

// ── WebSocket event handling ──────────────────────────────────────────────────
void WebComm::_wsEventStatic(AsyncWebSocket* server, AsyncWebSocketClient* client,
                              AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (_instance) _instance->_wsEvent(server, client, type, arg, data, len);
}

void WebComm::_wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                        AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
    case WS_EVT_CONNECT:
        Serial.printf("[WEBCOMM] Client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        _sendWelcome(client);
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("[WEBCOMM] Client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA: {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            char buf[128];
            size_t copyLen = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
            memcpy(buf, data, copyLen);
            buf[copyLen] = '\0';
            _parseCommand(buf, copyLen);
        }
        break;
    }
    case WS_EVT_ERROR:
        Serial.printf("[WEBCOMM] Client #%u error\n", client->id());
        break;
    default: break;
    }
}

// ── Command parser ────────────────────────────────────────────────────────────
// Protocol (text frames):
//   CMD:DRIVE:x=<f>,y=<f>                        — drive velocity [-1.0, +1.0]
//   CMD:STOP                                      — zero drive immediately
//   CMD:SPEED:v=<f>                               — speed scale [0, DRIVE_SPEED_MAX]
//   CMD:BALANCE:enable=<0|1>                      — enable/disable balance
//   CMD:BALANCE_TUNE:Kp=<f>,Kd=<f>,Kpr=<f>,Kdr=<f>  — update gains live
void WebComm::_parseCommand(const char* msg, size_t len) {

    if (strncmp(msg, "CMD:STOP", 8) == 0) {
        Serial.println("[WEBCOMM] CMD:STOP");
        if (_driveCb) _driveCb(0.0f, 0.0f);
        return;
    }

    if (strncmp(msg, "CMD:SPEED:", 10) == 0) {
        float v = DRIVE_SPEED_DEFAULT;
        if (sscanf(msg + 10, "v=%f", &v) == 1) {
            v = v < 0.0f ? 0.0f : (v > DRIVE_SPEED_MAX ? DRIVE_SPEED_MAX : v);
            if (_speedCb) _speedCb(v);
            _currentSpeed = v;
        } else {
            Serial.printf("[WEBCOMM] Malformed SPEED: %s\n", msg);
        }
        return;
    }

    if (strncmp(msg, "CMD:DRIVE:", 10) == 0) {
        float x = 0.0f, y = 0.0f;
        if (sscanf(msg + 10, "x=%f,y=%f", &x, &y) == 2) {
            if (_driveCb) _driveCb(_clamp1(x), _clamp1(y));
        } else {
            Serial.printf("[WEBCOMM] Malformed DRIVE: %s\n", msg);
        }
        return;
    }

    if (strncmp(msg, "CMD:BALANCE:", 12) == 0) {
        int enable = -1;
        if (sscanf(msg + 12, "enable=%d", &enable) == 1) {
            if (_balanceEnableCb) _balanceEnableCb(enable != 0);
            Serial.printf("[WEBCOMM] Balance %s\n", enable ? "ENABLED" : "DISABLED");
        } else {
            Serial.printf("[WEBCOMM] Malformed BALANCE cmd: %s\n", msg);
        }
        return;
    }

    if (strncmp(msg, "CMD:BALANCE_TUNE:", 17) == 0) {
        float kp  = SPHERE_BALANCE_KP_PITCH_DEFAULT;
        float kd  = SPHERE_BALANCE_KD_PITCH_DEFAULT;
        float kpr = SPHERE_BALANCE_KP_ROLL_DEFAULT;
        float kdr = SPHERE_BALANCE_KD_ROLL_DEFAULT;
        // Accept 2-4 parameters — kpr/kdr are optional for quick pitch-only tuning
        int parsed = sscanf(msg + 17, "Kp=%f,Kd=%f,Kpr=%f,Kdr=%f", &kp, &kd, &kpr, &kdr);
        if (parsed >= 2) {
            // Clamp at WebSocket boundary — second line of defence after UI slider max
            auto cl = [](float v, float mx) { return v < 0.0f ? 0.0f : (v > mx ? mx : v); };
            kp  = cl(kp,  SPHERE_BALANCE_KP_MAX);
            kd  = cl(kd,  SPHERE_BALANCE_KD_MAX);
            kpr = cl(kpr, SPHERE_BALANCE_KP_MAX);
            kdr = cl(kdr, SPHERE_BALANCE_KD_MAX);
            if (_balanceTuneCb) _balanceTuneCb(kp, kd, kpr, kdr);
            Serial.printf("[WEBCOMM] Balance gains: Kp=%.3f Kd=%.3f Kpr=%.3f Kdr=%.3f\n",
                          kp, kd, kpr, kdr);
        } else {
            Serial.printf("[WEBCOMM] Malformed BALANCE_TUNE (need >=2 params): %s\n", msg);
        }
        return;
    }

    Serial.printf("[WEBCOMM] Unknown command: %s\n", msg);
}

// ── Welcome packet ────────────────────────────────────────────────────────────
void WebComm::_sendWelcome(AsyncWebSocketClient* client) {
    JsonDocument doc;
    doc["type"]        = "welcome";
    doc["version"]     = "BB8-v1";
    doc["ip"]          = ipAddress();
    doc["speed_scale"] = _currentSpeed;
    doc["speed_max"]   = DRIVE_SPEED_MAX;
    // Balance starts disabled on boot; UI will sync from bal_enabled in next broadcast.
    doc["bal_enabled"] = false;
    String out;
    serializeJson(doc, out);
    client->text(out);
}