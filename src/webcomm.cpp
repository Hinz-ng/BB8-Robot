// module:  webcomm.cpp
// layer:   7 — network I/O only
// purpose: WiFi AP/STA bring-up, HTTP server, WebSocket telemetry + command parsing
// date:    2025

#include "webcomm.h"
#include "project_wide_defs.h"
#include <WiFi.h>
#include <Arduino.h>

// ── Embedded HTML ─────────────────────────────────────────────────────────────
// Single-file SPA: HTML + CSS + JS served from flash.
// Stored in PROGMEM to keep it off the heap.
// Edit freely — only the WebSocket message format must stay in sync with
// the JSON keys emitted by broadcastIMU() below.
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>BB-8 Controller</title>
<style>
  /* ── Reset & base ───────────────────────────────────────── */
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
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    font-size: 14px;
    min-height: 100dvh;
    display: flex;
    flex-direction: column;
    padding: 12px;
    gap: 12px;
    -webkit-tap-highlight-color: transparent;
  }

  /* ── Header ─────────────────────────────────────────────── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 16px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
  }

  .logo {
    display: flex;
    align-items: center;
    gap: 10px;
    font-weight: 700;
    font-size: 18px;
    letter-spacing: 0.5px;
  }

  .logo svg { flex-shrink: 0; }

  .status-pill {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 12px;
    color: var(--text-dim);
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 999px;
    padding: 5px 12px;
    user-select: none;
  }

  .dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--err);
    transition: background 0.4s;
  }
  .dot.ok   { background: var(--ok); }
  .dot.warn { background: var(--warn); }

  /* ── Cards ──────────────────────────────────────────────── */
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    overflow: hidden;
  }

  .card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 16px;
    cursor: pointer;
    user-select: none;
    -webkit-user-select: none;
  }

  .card-header:active { background: var(--surface2); }

  .card-title {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 600;
    font-size: 13px;
    text-transform: uppercase;
    letter-spacing: 0.8px;
    color: var(--text-dim);
  }

  .card-title .badge {
    font-size: 10px;
    background: var(--accent);
    color: #fff;
    border-radius: 4px;
    padding: 2px 6px;
    letter-spacing: 0;
    text-transform: none;
    font-weight: 700;
  }

  .chevron {
    color: var(--text-dim);
    transition: transform 0.25s;
    font-size: 18px;
    line-height: 1;
  }
  .chevron.open { transform: rotate(180deg); }

  .card-body {
    border-top: 1px solid var(--border);
    padding: 16px;
    display: none;   /* toggled by JS */
  }
  .card-body.open { display: block; }

  /* ── IMU grid ────────────────────────────────────────────── */
  .imu-section { margin-bottom: 16px; }
  .imu-section:last-child { margin-bottom: 0; }

  .imu-section-label {
    font-size: 11px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 1px;
    color: var(--text-dim);
    margin-bottom: 10px;
  }

  .imu-row {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 8px;
  }

  .imu-cell {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 12px;
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .imu-axis {
    font-size: 10px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.8px;
    color: var(--text-dim);
  }

  .imu-val {
    font-size: 16px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    color: var(--text);
    transition: color 0.2s;
  }

  .imu-val.hot { color: var(--accent2); }

  .imu-unit {
    font-size: 10px;
    color: var(--text-dim);
  }

  /* ── Bar indicators ──────────────────────────────────────── */
  .bar-track {
    height: 4px;
    background: var(--surface2);
    border-radius: 2px;
    margin-top: 6px;
    overflow: hidden;
  }
  .bar-fill {
    height: 100%;
    border-radius: 2px;
    background: var(--accent);
    width: 50%;           /* centred at 50% = zero; JS maps [-max, +max] → [0,100] */
    transition: width 0.1s linear;
  }

  /* ── Divider ─────────────────────────────────────────────── */
  .divider {
    height: 1px;
    background: var(--border);
    margin: 14px 0;
  }

  /* ── Joystick pad ────────────────────────────────────────── */
  .joystick-wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 12px;
    padding: 8px 0;
  }

  #joystickCanvas {
    touch-action: none;
    cursor: grab;
    border-radius: 50%;
    display: block;
  }

  .joy-readout {
    display: flex;
    gap: 24px;
    font-size: 12px;
    color: var(--text-dim);
    font-variant-numeric: tabular-nums;
  }

  .joy-readout span { color: var(--text); font-weight: 600; }

  /* ── Stop button ─────────────────────────────────────────── */
  #stopBtn {
    width: 100%;
    padding: 16px;
    border: none;
    border-radius: 10px;
    background: var(--err);
    color: #fff;
    font-size: 15px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
    cursor: pointer;
    transition: opacity 0.15s, transform 0.1s;
    -webkit-tap-highlight-color: transparent;
  }

  #stopBtn:active { opacity: 0.8; transform: scale(0.98); }

  /* ── System info ─────────────────────────────────────────── */
  .sysinfo {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .sysinfo-item {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 12px;
  }

  .sysinfo-label { font-size: 10px; color: var(--text-dim); text-transform: uppercase; letter-spacing: 0.8px; }
  .sysinfo-val   { font-size: 14px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 2px; }

  /* ── Footer ─────────────────────────────────────────────── */
  footer {
    text-align: center;
    font-size: 11px;
    color: var(--text-dim);
    padding: 4px 0 8px;
  }
</style>
</head>
<body>

<!-- Header -->
<header>
  <div class="logo">
    <!-- BB8 icon -->
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

<!-- IMU Panel (collapsible) -->
<div class="card" id="imuCard">
  <div class="card-header" onclick="toggleCard('imuCard')">
    <div class="card-title">
      IMU Readings
      <span class="badge" id="imuBadge">●</span>
    </div>
    <div class="chevron open" id="imuChevron">▾</div>
  </div>
  <div class="card-body open" id="imuBody">

    <!-- Accelerometer -->
    <div class="imu-section">
      <div class="imu-section-label">Accelerometer — m/s²</div>
      <div class="imu-row">
        <div class="imu-cell">
          <div class="imu-axis">X</div>
          <div class="imu-val" id="ax">—</div>
          <div class="bar-track"><div class="bar-fill" id="axBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Y</div>
          <div class="imu-val" id="ay">—</div>
          <div class="bar-track"><div class="bar-fill" id="ayBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Z</div>
          <div class="imu-val" id="az">—</div>
          <div class="bar-track"><div class="bar-fill" id="azBar"></div></div>
        </div>
      </div>
    </div>

    <div class="divider"></div>

    <!-- Gyroscope -->
    <div class="imu-section">
      <div class="imu-section-label">Gyroscope — rad/s</div>
      <div class="imu-row">
        <div class="imu-cell">
          <div class="imu-axis">X</div>
          <div class="imu-val" id="gx">—</div>
          <div class="bar-track"><div class="bar-fill" id="gxBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Y</div>
          <div class="imu-val" id="gy">—</div>
          <div class="bar-track"><div class="bar-fill" id="gyBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Z</div>
          <div class="imu-val" id="gz">—</div>
          <div class="bar-track"><div class="bar-fill" id="gzBar"></div></div>
        </div>
      </div>
    </div>

<div class="divider"></div>

    <!-- Complementary Filter State Estimate -->
    <div class="imu-section">
      <div class="imu-section-label">State Estimate — complementary filter</div>
      <div class="imu-row" style="grid-template-columns: 1fr 1fr;">
        <div class="imu-cell">
          <div class="imu-axis">Pitch</div>
          <div class="imu-val" id="estPitch">—</div>
          <div class="imu-unit">deg</div>
          <div class="bar-track"><div class="bar-fill" id="estPitchBar"></div></div>
        </div>
        <div class="imu-cell">
          <div class="imu-axis">Roll</div>
          <div class="imu-val" id="estRoll">—</div>
          <div class="imu-unit">deg</div>
          <div class="bar-track"><div class="bar-fill" id="estRollBar"></div></div>
        </div>
      </div>
    </div>

  </div><!-- /imuBody -->
</div>

<!-- Drive Panel (collapsible) -->
<div class="card" id="driveCard">
  <div class="card-header" onclick="toggleCard('driveCard')">
    <div class="card-title">Drive Control</div>
    <div class="chevron open" id="driveChevron">▾</div>
  </div>
  <div class="card-body open" id="driveBody">
    <div class="joystick-wrap">
      <canvas id="joystickCanvas" width="200" height="200"></canvas>
      <div class="joy-readout">
        X&nbsp;<span id="joyX">0.00</span>&nbsp;&nbsp;
        Y&nbsp;<span id="joyY">0.00</span>
      </div>
    </div>
    <div style="margin-top:14px;">
      <button id="stopBtn" onclick="sendStop()">⏹ Emergency Stop</button>
    </div>
  </div>
</div>

<!-- System Panel (collapsible, closed by default) -->
<div class="card" id="sysCard">
  <div class="card-header" onclick="toggleCard('sysCard')">
    <div class="card-title">System</div>
    <div class="chevron" id="sysChevron">▾</div>
  </div>
  <div class="card-body" id="sysBody">
    <div class="sysinfo">
      <div class="sysinfo-item">
        <div class="sysinfo-label">Uptime</div>
        <div class="sysinfo-val" id="uptime">—</div>
      </div>
      <div class="sysinfo-item">
        <div class="sysinfo-label">Clients</div>
        <div class="sysinfo-val" id="clients">—</div>
      </div>
      <div class="sysinfo-item">
        <div class="sysinfo-label">IMU</div>
        <div class="sysinfo-val" id="imuStatus">—</div>
      </div>
      <div class="sysinfo-item">
        <div class="sysinfo-label">WS Msgs</div>
        <div class="sysinfo-val" id="msgCount">0</div>
      </div>
    </div>
  </div>
</div>

<footer>BB-8 Controller · ESP32-S3</footer>

<script>
// ── WebSocket ────────────────────────────────────────────────────────────────
let ws, msgCount = 0, reconnectTimer;

function connect() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);

  ws.onopen = () => {
    setStatus('ok', 'Connected');
    clearTimeout(reconnectTimer);
  };

  ws.onclose = () => {
    setStatus('warn', 'Reconnecting…');
    reconnectTimer = setTimeout(connect, 2000);
  };

  ws.onerror = () => setStatus('err', 'Error');

  ws.onmessage = (e) => {
    msgCount++;
    document.getElementById('msgCount').textContent = msgCount;
    try { handlePacket(JSON.parse(e.data)); } catch(_) {}
  };
}

function setStatus(state, label) {
  const dot   = document.getElementById('wsDot');
  const lbl   = document.getElementById('wsLabel');
  dot.className = 'dot ' + (state === 'ok' ? 'ok' : state === 'warn' ? 'warn' : '');
  lbl.textContent = label;
}

// ── Packet handler ───────────────────────────────────────────────────────────
function handlePacket(p) {
  if (p.type === 'imu') {
    // Accel
    setImu('ax', p.ax, 20);
    setImu('ay', p.ay, 20);
    setImu('az', p.az, 20);
    // Gyro
    setImu('gx', p.gx, 10, 3);
    setImu('gy', p.gy, 10, 3);
    setImu('gz', p.gz, 10, 3);

    // Badge colour
    const badge = document.getElementById('imuBadge');
    badge.style.color = p.valid ? '#22c55e' : '#ef4444';

    // State estimate — bar scale matches SPHERE_ESTOP_PITCH_DEG (30°)
    // 'hot' highlight triggers at ±15° (50% of estop threshold)
    if (p.pitch_deg !== undefined) {
      setImu('estPitch', parseFloat(p.pitch_deg), 30, 1);
      setImu('estRoll',  parseFloat(p.roll_deg),  30, 1);
    }

    document.getElementById('imuStatus').textContent  = p.valid ? 'OK' : 'ERR';
    document.getElementById('uptime').textContent     = fmtUptime(p.uptime_ms);
    document.getElementById('clients').textContent    = p.clients ?? '—';
  }
}

// ── IMU helpers ──────────────────────────────────────────────────────────────
function setImu(id, val, maxAbs, decimals = 2) {
  const el  = document.getElementById(id);
  const bar = document.getElementById(id + 'Bar');

  const formatted = val.toFixed(decimals);
  el.textContent = formatted;

  // Highlight if magnitude > 50% of scale
  el.classList.toggle('hot', Math.abs(val) > maxAbs * 0.5);

  // Bar: maps [-maxAbs, +maxAbs] → [0%, 100%], centred at 50%
  if (bar) {
    const pct = 50 + (val / maxAbs) * 50;
    bar.style.width = Math.min(100, Math.max(0, pct)) + '%';
  }
}

function fmtUptime(ms) {
  const s = Math.floor(ms / 1000);
  const m = Math.floor(s / 60);
  const h = Math.floor(m / 60);
  if (h > 0) return `${h}h ${m % 60}m`;
  if (m > 0) return `${m}m ${s % 60}s`;
  return `${s}s`;
}

// ── Card collapse ────────────────────────────────────────────────────────────
function toggleCard(id) {
  const body    = document.getElementById(id.replace('Card', 'Body'));
  const chevron = document.getElementById(id.replace('Card', 'Chevron'));
  const open    = body.classList.toggle('open');
  chevron.classList.toggle('open', open);
}

// ── Joystick ─────────────────────────────────────────────────────────────────
const canvas  = document.getElementById('joystickCanvas');
const ctx     = canvas.getContext('2d');
const R       = canvas.width / 2;   // outer radius
const rKnob   = 28;                 // knob radius
let joyActive = false;
let joyX = 0, joyY = 0;            // normalised [-1, +1]
let sendTimer;

function drawJoy() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Outer ring
  ctx.beginPath();
  ctx.arc(R, R, R - 4, 0, Math.PI * 2);
  ctx.strokeStyle = '#2a2f40';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Crosshair
  ctx.strokeStyle = '#2a2f40';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(R, 4);    ctx.lineTo(R, canvas.height - 4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(4, R);    ctx.lineTo(canvas.width - 4, R);  ctx.stroke();

  // Knob position
  const kx = R + joyX * (R - rKnob - 4);
  const ky = R + joyY * (R - rKnob - 4);

  // Knob fill
  const grad = ctx.createRadialGradient(kx - 4, ky - 4, 2, kx, ky, rKnob);
  grad.addColorStop(0, '#5aa8ff');
  grad.addColorStop(1, '#1a5aaa');
  ctx.beginPath();
  ctx.arc(kx, ky, rKnob, 0, Math.PI * 2);
  ctx.fillStyle = grad;
  ctx.fill();
  ctx.strokeStyle = '#3b8ef3';
  ctx.lineWidth = 2;
  ctx.stroke();
}

function joyFromEvent(e) {
  const rect = canvas.getBoundingClientRect();
  const touch = e.touches ? e.touches[0] : e;
  const cx = touch.clientX - rect.left - R;
  const cy = touch.clientY - rect.top  - R;
  const max = R - rKnob - 4;
  const dist = Math.sqrt(cx * cx + cy * cy);
  const scale = dist > max ? max / dist : 1;
  joyX = Math.max(-1, Math.min(1, (cx * scale) / max));
  joyY = Math.max(-1, Math.min(1, (cy * scale) / max));
}

function startJoy(e) { e.preventDefault(); joyActive = true; joyFromEvent(e); drawJoy(); scheduleSend(); }
function moveJoy(e)  { if (!joyActive) return; e.preventDefault(); joyFromEvent(e); drawJoy(); updateReadout(); }
function endJoy(e)   { joyActive = false; joyX = 0; joyY = 0; drawJoy(); updateReadout(); sendDrive(0, 0); }

canvas.addEventListener('mousedown',  startJoy);
canvas.addEventListener('mousemove',  moveJoy);
canvas.addEventListener('mouseup',    endJoy);
canvas.addEventListener('touchstart', startJoy, {passive: false});
canvas.addEventListener('touchmove',  moveJoy,  {passive: false});
canvas.addEventListener('touchend',   endJoy);

function updateReadout() {
  document.getElementById('joyX').textContent = joyX.toFixed(2);
  // Invert Y for intuitive "up = forward"
  document.getElementById('joyY').textContent = (-joyY).toFixed(2);
}

function scheduleSend() {
  if (!joyActive) return;
  sendDrive(joyX, -joyY);           // invert Y so up = forward
  updateReadout();
  sendTimer = setTimeout(scheduleSend, 80); // 12.5 Hz drive rate
}

function sendDrive(x, y) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(`CMD:DRIVE:x=${x.toFixed(3)},y=${y.toFixed(3)}`);
}

function sendStop() {
  joyX = 0; joyY = 0;
  drawJoy(); updateReadout();
  clearTimeout(sendTimer);
  if (ws && ws.readyState === WebSocket.OPEN) ws.send('CMD:STOP');
}

// ── Init ─────────────────────────────────────────────────────────────────────
drawJoy();
connect();
</script>
</body>
</html>
)rawliteral";

// ── WebComm implementation ────────────────────────────────────────────────────

// Pointer used by the static WebSocket event trampoline
static WebComm* _instance = nullptr;

bool WebComm::begin(const char* ssid, const char* pass) {
    _instance = this;

    // WiFi bring-up: AP-first if ssid is empty, STA with AP fallback otherwise.
    // Passing ssid="" from main.cpp bypasses the 15 s STA timeout entirely.
    if (ssid == nullptr || ssid[0] == '\0') {
        // ── AP mode (no credentials supplied) ────────────────────────────────
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BB8-Robot", "bb8robot1");
        Serial.printf("[WEBCOMM] AP mode. IP: %s\n",
                      WiFi.softAPIP().toString().c_str());
    } else {
        // ── STA mode with AP fallback ─────────────────────────────────────────
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
                Serial.printf("[WEBCOMM] AP IP: %s\n",
                              WiFi.softAPIP().toString().c_str());
                break;
            }
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[WEBCOMM] Connected. IP: %s\n",
                          WiFi.localIP().toString().c_str());
        }
    }

    // Attach WebSocket handler
    _ws.onEvent(_wsEventStatic);
    _server.addHandler(&_ws);

    // Serve HTML root
    // send_P is deprecated in mathieucarbou/ESPAsyncWebServer v3 — use send().
    // INDEX_HTML is in PROGMEM; the v3 send() overload handles PROGMEM correctly.
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    // 404 handler
    _server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "Not found");
    });

    _server.begin();
    _connected = true;
    Serial.println("[WEBCOMM] HTTP server started");
    return true;
}

void WebComm::update() {
    // AsyncWebServer is fully interrupt-driven — this call just handles
    // WebSocket stale-client cleanup which must run from the main loop
    _ws.cleanupClients();
}

// ── REPLACE the entire broadcastIMU() function with this ─────────────────────

void WebComm::broadcastState(const RawIMUData& raw, const IMUState& est) {
    if (_ws.count() == 0) return;

    JsonDocument doc;
    doc["type"]      = "imu";
    // Raw IMU
    doc["ax"]        = serialized(String(raw.accel_x_ms2, 3));
    doc["ay"]        = serialized(String(raw.accel_y_ms2, 3));
    doc["az"]        = serialized(String(raw.accel_z_ms2, 3));
    doc["gx"]        = serialized(String(raw.gyro_x_rads, 4));
    doc["gy"]        = serialized(String(raw.gyro_y_rads, 4));
    doc["gz"]        = serialized(String(raw.gyro_z_rads, 4));
    doc["valid"]     = raw.valid;
    // State estimate
    doc["pitch_deg"] = serialized(String(est.pitch_deg, 1));
    doc["roll_deg"]  = serialized(String(est.roll_deg,  1));
    doc["est_valid"] = est.valid;
    // System
    doc["uptime_ms"] = millis();
    doc["clients"]   = (uint8_t)_ws.count();

    String out;
    out.reserve(220);
    serializeJson(doc, out);
    _ws.textAll(out);
}

uint8_t WebComm::clientCount() const {
    return (uint8_t)_ws.count();
}

String WebComm::ipAddress() const {
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    return WiFi.softAPIP().toString();
}

// ── WebSocket event handling ──────────────────────────────────────────────────
void WebComm::_wsEventStatic(AsyncWebSocket* server,
                              AsyncWebSocketClient* client,
                              AwsEventType type,
                              void* arg,
                              uint8_t* data,
                              size_t len)
{
    if (_instance) _instance->_wsEvent(server, client, type, arg, data, len);
}

void WebComm::_wsEvent(AsyncWebSocket* server,
                        AsyncWebSocketClient* client,
                        AwsEventType type,
                        void* arg,
                        uint8_t* data,
                        size_t len)
{
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
        // Only handle complete, single-frame text messages
        if (info->final && info->index == 0 && info->len == len
            && info->opcode == WS_TEXT)
        {
            // Null-terminate for safe string ops
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

    default:
        break;
    }
}

// ── Command parser ────────────────────────────────────────────────────────────
// Protocol (text frames):
//   CMD:DRIVE:x=<float>,y=<float>   — drive velocity, range [-1.0, +1.0]
//   CMD:STOP                         — zero drive immediately
//
// Format is intentionally simple (no JSON) to minimise phone-side latency.
void WebComm::_parseCommand(const char* msg, size_t len) {
    if (strncmp(msg, "CMD:STOP", 8) == 0) {
        Serial.println("[WEBCOMM] CMD:STOP received");
        if (_driveCb) _driveCb(0.0f, 0.0f);
        return;
    }

    if (strncmp(msg, "CMD:DRIVE:", 10) == 0) {
        float x = 0.0f, y = 0.0f;
        // sscanf is safe here — buf is null-terminated and bounded
        if (sscanf(msg + 10, "x=%f,y=%f", &x, &y) == 2) {
            x = _clamp1(x);
            y = _clamp1(y);
            if (_driveCb) _driveCb(x, y);
        } else {
            Serial.printf("[WEBCOMM] Malformed DRIVE command: %s\n", msg);
        }
        return;
    }

    Serial.printf("[WEBCOMM] Unknown command: %s\n", msg);
}

// ── Welcome packet ────────────────────────────────────────────────────────────
// Sent immediately on connect so the UI doesn't wait for the next broadcast tick
void WebComm::_sendWelcome(AsyncWebSocketClient* client) {
    JsonDocument doc;
    doc["type"]    = "welcome";
    doc["version"] = "BB8-v1";
    doc["ip"]      = ipAddress();
    String out;
    serializeJson(doc, out);
    client->text(out);
}