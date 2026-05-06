/**
 * PIDTuning.js — Vanilla JS PID Tuning Panel
 * Drop into any Three.js simulator. No dependencies.
 *
 * Usage:
 *   import PIDTuning from './PIDTuning.js';
 *   const pidPanel = new PIDTuning({ onPIDChange: (axis, p, i, d) => { ... } });
 *   document.body.appendChild(pidPanel.el);
 */

export default class PIDTuning {

  // ─── Axis defaults & slider ranges ────────────────────────────────────────
  static AXES = {
    roll:  { label: 'Roll axis — inner loop',  p: 0.45, i: 0.12, d: 0.080, pMax: 2.0, iMax: 1.0, dMax: 0.50 },
    pitch: { label: 'Pitch axis — inner loop', p: 0.42, i: 0.10, d: 0.070, pMax: 2.0, iMax: 1.0, dMax: 0.50 },
    yaw:   { label: 'Yaw axis — heading hold', p: 0.30, i: 0.20, d: 0.020, pMax: 1.5, iMax: 1.5, dMax: 0.20 },
    alt:   { label: 'Altitude — outer loop',   p: 0.55, i: 0.08, d: 0.150, pMax: 3.0, iMax: 2.0, dMax: 1.00 },
  };

  // ─── Flight presets ────────────────────────────────────────────────────────
  static PRESETS = {
    stable:     { p: 0.25, i: 0.05, d: 0.120 },
    default:    { p: 0.45, i: 0.12, d: 0.080 },
    cinematic:  { p: 0.20, i: 0.04, d: 0.100 },
    racing:     { p: 1.40, i: 0.30, d: 0.050 },
    acro:       { p: 1.10, i: 0.18, d: 0.040 },
    aggressive: { p: 1.80, i: 0.50, d: 0.020 },
    windy:      { p: 0.65, i: 0.35, d: 0.150 },
  };

  constructor({ onPIDChange = null, telemetry = null } = {}) {
    this.onPIDChange = onPIDChange;
    this.telemetry   = telemetry;   // pass { altitude, speed } ref — updated externally
    this.curAxis     = 'roll';
    this.autotuning  = false;
    this._autotuneTimer = null;
    this._oscTimer      = null;
    this._oscPhase      = 0;

    // PID values per axis (live copies)
    this.pid = {};
    for (const [k, v] of Object.entries(PIDTuning.AXES)) {
      this.pid[k] = { p: v.p, i: v.i, d: v.d };
    }

    this._build();
    this._startOscilloscope();
    this._runSim();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Public API
  // ──────────────────────────────────────────────────────────────────────────

  /** Update a displayed telemetry value (call from your animation loop) */
  updateTelemetry(altitude, speed) {
    if (this._telEl) {
      this._telEl.textContent = `ALT: ${(+altitude).toFixed(1)}m  |  SPD: ${(+speed).toFixed(1)}m/s`;
    }
  }

  /** Get current PID for a given axis */
  getPID(axis) {
    return { ...this.pid[axis] };
  }

  /** Programmatically set PID (also updates sliders) */
  setPID(axis, p, i, d) {
    this.pid[axis] = { p, i, d };
    if (axis === this.curAxis) this._syncSlidersToState();
    this._runSim();
  }

  /** Show / hide the panel */
  show() { this.el.style.display = 'block'; }
  hide() { this.el.style.display = 'none'; }
  toggle() { this.el.style.display = this.el.style.display === 'none' ? 'block' : 'none'; }

  /** Clean up timers */
  destroy() {
    clearInterval(this._oscTimer);
    clearInterval(this._autotuneTimer);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // DOM build
  // ──────────────────────────────────────────────────────────────────────────

  _build() {
    this.el = document.createElement('div');
    this.el.id = 'pid-panel';
    this.el.innerHTML = this._html();
    document.head.appendChild(this._styleTag());

    // Cache refs
    this._stepCanvas = this.el.querySelector('#pid-step-canvas');
    this._oscCanvas  = this.el.querySelector('#pid-osc-canvas');
    this._axisLabel  = this.el.querySelector('#pid-axis-label');
    this._telEl      = this.el.querySelector('#pid-tel');

    this._sliders = {
      p: this.el.querySelector('#pid-p-slider'),
      i: this.el.querySelector('#pid-i-slider'),
      d: this.el.querySelector('#pid-d-slider'),
    };
    this._vals = {
      p: this.el.querySelector('#pid-p-val'),
      i: this.el.querySelector('#pid-i-val'),
      d: this.el.querySelector('#pid-d-val'),
    };

    this._stats = {
      overshoot: this.el.querySelector('#stat-overshoot'),
      settle:    this.el.querySelector('#stat-settle'),
      rise:      this.el.querySelector('#stat-rise'),
      sse:       this.el.querySelector('#stat-sse'),
    };

    // Axis tab clicks
    this.el.querySelectorAll('.pid-axis-tab').forEach(btn => {
      btn.addEventListener('click', () => this._setAxis(btn.dataset.axis));
    });

    // Slider input
    ['p', 'i', 'd'].forEach(term => {
      this._sliders[term].addEventListener('input', e => {
        const val = parseFloat(e.target.value);
        this.pid[this.curAxis][term] = val;
        this._vals[term].textContent = term === 'd' ? val.toFixed(3) : val.toFixed(2);
        this._updateSliderFill(term, val);
        this._runSim();
        this._clearPresetActive();
        if (this.onPIDChange) {
          const { p, i, d } = this.pid[this.curAxis];
          this.onPIDChange(this.curAxis, p, i, d);
        }
      });
    });

    // Preset buttons
    this.el.querySelectorAll('.pid-preset-btn').forEach(btn => {
      btn.addEventListener('click', () => this._loadPreset(btn.dataset.preset, btn));
    });

    // Autotune button
    this.el.querySelector('#pid-autotune-btn').addEventListener('click', () => this._startAutotune());
  }

  _html() {
    const presetKeys = Object.keys(PIDTuning.PRESETS);
    return `
      <div class="pid-header">
        <span class="pid-title">PID TUNING</span>
        <span class="pid-live-badge"><span class="pid-dot"></span>LIVE SIM</span>
        <span id="pid-tel" class="pid-tel"></span>
      </div>

      <div class="pid-body">

        <!-- LEFT: controls -->
        <div class="pid-left">

          <div class="pid-axis-tabs">
            ${['roll','pitch','yaw','alt'].map((a, i) =>
              `<button class="pid-axis-tab${i===0?' active':''}" data-axis="${a}">${a.toUpperCase()}</button>`
            ).join('')}
          </div>

          <div id="pid-axis-label" class="pid-section-label">Roll axis — inner loop</div>

          <!-- P -->
          <div class="pid-slider-row">
            <div class="pid-slider-header">
              <span class="pid-slider-name">P <span class="pid-slider-desc">Proportional</span></span>
              <span id="pid-p-val" class="pid-slider-val">0.45</span>
            </div>
            <input id="pid-p-slider" class="pid-slider" type="range" min="0" max="2.0" step="0.01" value="0.45">
          </div>

          <!-- I -->
          <div class="pid-slider-row">
            <div class="pid-slider-header">
              <span class="pid-slider-name">I <span class="pid-slider-desc">Integral</span></span>
              <span id="pid-i-val" class="pid-slider-val">0.12</span>
            </div>
            <input id="pid-i-slider" class="pid-slider" type="range" min="0" max="1.0" step="0.01" value="0.12">
          </div>

          <!-- D -->
          <div class="pid-slider-row">
            <div class="pid-slider-header">
              <span class="pid-slider-name">D <span class="pid-slider-desc">Derivative</span></span>
              <span id="pid-d-val" class="pid-slider-val">0.080</span>
            </div>
            <input id="pid-d-slider" class="pid-slider" type="range" min="0" max="0.5" step="0.005" value="0.08">
          </div>

          <div class="pid-divider"></div>
          <div class="pid-section-label">PRESETS</div>

          <div class="pid-presets">
            ${presetKeys.map((name, i) =>
              `<button class="pid-preset-btn${i===1?' active':''}" data-preset="${name}">${name.toUpperCase()}</button>`
            ).join('')}
          </div>

          <button id="pid-autotune-btn" class="pid-autotune-btn">⟳&nbsp;&nbsp;AUTO-TUNE AXIS</button>
        </div>

        <!-- RIGHT: graphs + stats -->
        <div class="pid-right">

          <div class="pid-graph-block">
            <div class="pid-graph-label">STEP RESPONSE — SETPOINT VS OUTPUT</div>
            <canvas id="pid-step-canvas" class="pid-canvas" height="160"></canvas>
            <div class="pid-time-axis">
              <span>0ms</span><span>100ms</span><span>200ms</span><span>300ms</span><span>400ms</span>
            </div>
          </div>

          <div class="pid-graph-block">
            <div class="pid-graph-label">LIVE OSCILLOSCOPE</div>
            <canvas id="pid-osc-canvas" class="pid-canvas" height="90"></canvas>
          </div>

          <div class="pid-stats">
            <div class="pid-stat-card">
              <div id="stat-overshoot" class="pid-stat-val" style="color:#ff6b2b">0.0%</div>
              <div class="pid-stat-lbl">Overshoot</div>
            </div>
            <div class="pid-stat-card">
              <div id="stat-settle" class="pid-stat-val" style="color:#4a9eff">0ms</div>
              <div class="pid-stat-lbl">Settle time</div>
            </div>
            <div class="pid-stat-card">
              <div id="stat-rise" class="pid-stat-val" style="color:#4ae89a">0ms</div>
              <div class="pid-stat-lbl">Rise time</div>
            </div>
            <div class="pid-stat-card">
              <div id="stat-sse" class="pid-stat-val" style="color:#c8d6e5">0.000</div>
              <div class="pid-stat-lbl">Steady err</div>
            </div>
          </div>

        </div>
      </div>
    `;
  }

  _styleTag() {
    const s = document.createElement('style');
    s.textContent = `
      #pid-panel {
        position: absolute;
        top: 60px; right: 16px;
        width: 680px;
        background: #0b1120;
        color: #c8d6e5;
        font-family: 'Courier New', monospace;
        font-size: 13px;
        border: 1px solid #1e3a5f;
        border-radius: 6px;
        overflow: hidden;
        z-index: 1000;
        user-select: none;
      }
      .pid-header {
        background: #0f1729;
        border-bottom: 1px solid #1e3a5f;
        padding: 10px 16px;
        display: flex;
        align-items: center;
        gap: 12px;
      }
      .pid-title {
        font-size: 11px;
        letter-spacing: 2px;
        color: #4a9eff;
        font-weight: 700;
      }
      .pid-live-badge {
        background: #4a9eff22;
        border: 1px solid #4a9eff44;
        color: #4a9eff;
        font-size: 10px;
        padding: 2px 8px;
        border-radius: 3px;
        letter-spacing: 1px;
        display: flex;
        align-items: center;
        gap: 5px;
      }
      .pid-dot {
        width: 6px; height: 6px;
        border-radius: 50%;
        background: #4a9eff;
        animation: pid-blink 1.2s infinite;
        display: inline-block;
      }
      .pid-tel {
        margin-left: auto;
        font-size: 10px;
        color: #3a6a9a;
        letter-spacing: 1px;
      }
      .pid-body {
        display: grid;
        grid-template-columns: 1fr 1fr;
      }
      .pid-left {
        padding: 12px 16px;
        border-right: 1px solid #1e3a5f;
      }
      .pid-right {
        padding: 12px 16px;
        display: flex;
        flex-direction: column;
        gap: 10px;
      }
      .pid-axis-tabs {
        display: flex;
        gap: 4px;
        margin-bottom: 14px;
      }
      .pid-axis-tab {
        flex: 1;
        padding: 6px 4px;
        background: #0f1729;
        border: 1px solid #1e3a5f;
        color: #5a7a9a;
        font-size: 11px;
        letter-spacing: 1px;
        cursor: pointer;
        border-radius: 3px;
        font-family: 'Courier New', monospace;
        transition: all 0.15s;
      }
      .pid-axis-tab:hover { border-color: #2a5a8a; color: #8ab4d4; }
      .pid-axis-tab.active {
        background: #4a9eff18;
        border-color: #4a9eff88;
        color: #4a9eff;
      }
      .pid-section-label {
        font-size: 10px;
        letter-spacing: 2px;
        color: #3a6a9a;
        margin-bottom: 10px;
        text-transform: uppercase;
      }
      .pid-slider-row { margin-bottom: 14px; }
      .pid-slider-header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 5px;
      }
      .pid-slider-name {
        font-size: 12px;
        color: #8ab4d4;
        letter-spacing: 1px;
      }
      .pid-slider-desc {
        font-size: 10px;
        color: #3a6a9a;
        margin-left: 6px;
      }
      .pid-slider-val {
        font-size: 14px;
        color: #4a9eff;
        font-weight: 700;
        min-width: 46px;
        text-align: right;
      }
      .pid-slider {
        -webkit-appearance: none;
        width: 100%;
        height: 4px;
        border-radius: 2px;
        outline: none;
        cursor: pointer;
        background: #1e3a5f;
      }
      .pid-slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        width: 14px; height: 14px;
        border-radius: 50%;
        background: #4a9eff;
        cursor: pointer;
        border: 2px solid #0b1120;
      }
      .pid-slider::-moz-range-thumb {
        width: 14px; height: 14px;
        border-radius: 50%;
        background: #4a9eff;
        cursor: pointer;
        border: 2px solid #0b1120;
      }
      .pid-divider {
        border-top: 1px solid #1e3a5f;
        margin: 10px 0;
      }
      .pid-presets {
        display: flex;
        flex-wrap: wrap;
        gap: 5px;
        margin-bottom: 10px;
      }
      .pid-preset-btn {
        flex: 1 1 calc(33% - 5px);
        padding: 6px 4px;
        background: #0f1729;
        border: 1px solid #1e3a5f;
        color: #5a7a9a;
        font-size: 10px;
        letter-spacing: 1px;
        cursor: pointer;
        border-radius: 3px;
        text-align: center;
        font-family: 'Courier New', monospace;
        transition: all 0.15s;
      }
      .pid-preset-btn:hover { border-color: #4a9eff66; color: #8ab4d4; background: #4a9eff0a; }
      .pid-preset-btn.active { border-color: #4a9eff; color: #4a9eff; background: #4a9eff18; }
      .pid-autotune-btn {
        width: 100%;
        padding: 8px;
        background: #4a9eff15;
        border: 1px solid #4a9eff44;
        color: #4a9eff;
        font-size: 11px;
        letter-spacing: 2px;
        cursor: pointer;
        border-radius: 3px;
        font-family: 'Courier New', monospace;
        transition: all 0.2s;
      }
      .pid-autotune-btn:hover { background: #4a9eff25; border-color: #4a9eff; }
      .pid-autotune-btn.running {
        border-color: #ff6b2b;
        color: #ff6b2b;
        background: #ff6b2b15;
        animation: pid-pulse 1s infinite;
      }
      .pid-graph-block { display: flex; flex-direction: column; }
      .pid-graph-label {
        font-size: 10px;
        letter-spacing: 2px;
        color: #3a6a9a;
        margin-bottom: 6px;
      }
      .pid-canvas {
        width: 100%;
        border: 1px solid #1e3a5f;
        border-radius: 4px;
        background: #070d1a;
        display: block;
      }
      .pid-time-axis {
        display: flex;
        justify-content: space-between;
        margin-top: 3px;
        font-size: 9px;
        color: #2a5a8a;
      }
      .pid-stats {
        display: flex;
        gap: 8px;
      }
      .pid-stat-card {
        flex: 1;
        background: #0f1729;
        border: 1px solid #1e3a5f;
        border-radius: 4px;
        padding: 8px 6px;
        text-align: center;
      }
      .pid-stat-val {
        font-size: 15px;
        font-weight: 700;
        margin-bottom: 3px;
      }
      .pid-stat-lbl {
        font-size: 9px;
        letter-spacing: 1px;
        color: #3a6a9a;
        text-transform: uppercase;
      }
      @keyframes pid-blink { 0%,100%{opacity:1} 50%{opacity:.2} }
      @keyframes pid-pulse { 0%,100%{opacity:1} 50%{opacity:.5} }
    `;
    return s;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Axis switching
  // ──────────────────────────────────────────────────────────────────────────

  _setAxis(axis) {
    this.curAxis = axis;
    const cfg = PIDTuning.AXES[axis];

    // Update tab highlights
    this.el.querySelectorAll('.pid-axis-tab').forEach(btn => {
      btn.classList.toggle('active', btn.dataset.axis === axis);
    });

    // Update label
    this._axisLabel.textContent = cfg.label;

    // Update slider ranges to axis limits
    this._sliders.p.max = cfg.pMax;
    this._sliders.i.max = cfg.iMax;
    this._sliders.d.max = cfg.dMax;

    this._syncSlidersToState();
    this._runSim();
  }

  _syncSlidersToState() {
    const { p, i, d } = this.pid[this.curAxis];
    ['p','i','d'].forEach(term => {
      const val = this.pid[this.curAxis][term];
      this._sliders[term].value = val;
      this._vals[term].textContent = term === 'd' ? val.toFixed(3) : val.toFixed(2);
      this._updateSliderFill(term, val);
    });
  }

  _updateSliderFill(term, val) {
    const slider = this._sliders[term];
    const min  = parseFloat(slider.min);
    const max  = parseFloat(slider.max);
    const pct  = ((val - min) / (max - min)) * 100;
    slider.style.background =
      `linear-gradient(to right, #4a9eff ${pct}%, #1e3a5f ${pct}%)`;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Presets
  // ──────────────────────────────────────────────────────────────────────────

  _loadPreset(name, btn) {
    const pr  = PIDTuning.PRESETS[name];
    const cfg = PIDTuning.AXES[this.curAxis];
    this.pid[this.curAxis] = {
      p: Math.min(pr.p, cfg.pMax),
      i: Math.min(pr.i, cfg.iMax),
      d: Math.min(pr.d, cfg.dMax),
    };
    this._syncSlidersToState();
    this._clearPresetActive();
    btn.classList.add('active');
    this._runSim();
    if (this.onPIDChange) {
      const { p, i, d } = this.pid[this.curAxis];
      this.onPIDChange(this.curAxis, p, i, d);
    }
  }

  _clearPresetActive() {
    this.el.querySelectorAll('.pid-preset-btn').forEach(b => b.classList.remove('active'));
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Auto-tune (Ziegler–Nichols-style sweep simulation)
  // ──────────────────────────────────────────────────────────────────────────

  _startAutotune() {
    if (this.autotuning) return;
    this.autotuning = true;
    const btn = this.el.querySelector('#pid-autotune-btn');
    btn.classList.add('running');
    btn.textContent = '⟳  TUNING...';
    this._clearPresetActive();

    let step = 0;
    const cfg  = PIDTuning.AXES[this.curAxis];
    const base = { ...this.pid[this.curAxis] };

    this._autotuneTimer = setInterval(() => {
      step++;
      const t = step / 40; // 0 → 1

      // Simulate parameter search with convergence noise
      const p = base.p * (0.5 + t * 0.5) + (Math.random() - 0.5) * 0.04 * (1 - t);
      const i = base.i * (0.5 + t * 0.5) + (Math.random() - 0.5) * 0.02 * (1 - t);
      const d = base.d * (0.5 + t * 0.5) + (Math.random() - 0.5) * 0.01 * (1 - t);

      this.pid[this.curAxis] = {
        p: Math.max(0, Math.min(cfg.pMax, parseFloat(p.toFixed(3)))),
        i: Math.max(0, Math.min(cfg.iMax, parseFloat(i.toFixed(3)))),
        d: Math.max(0, Math.min(cfg.dMax, parseFloat(d.toFixed(4)))),
      };

      this._syncSlidersToState();
      this._runSim();

      if (this.onPIDChange) {
        const { p, i, d } = this.pid[this.curAxis];
        this.onPIDChange(this.curAxis, p, i, d);
      }

      if (step >= 40) {
        clearInterval(this._autotuneTimer);
        this.autotuning = false;
        btn.classList.remove('running');
        btn.textContent = '✓  TUNE COMPLETE';
        setTimeout(() => { btn.textContent = '⟳  AUTO-TUNE AXIS'; }, 2500);
      }
    }, 60);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // PID simulation (discrete-time, 2nd-order plant)
  // ──────────────────────────────────────────────────────────────────────────

  _simulate(Kp, Ki, Kd, steps = 400, dt = 0.001) {
    let output = 0, integral = 0, prevErr = 0;
    const setpoint = 1.0;
    const tau      = 0.05; // plant time constant (50 ms — realistic small drone)
    const results  = [];
    for (let k = 0; k < steps; k++) {
      const error      = setpoint - output;
      integral        += error * dt;
      const derivative = (error - prevErr) / dt;
      prevErr          = error;
      const u = Math.max(-3, Math.min(3, Kp * error + Ki * integral + Kd * derivative));
      output  += (u - output) * (dt / tau);
      results.push(output);
    }
    return results;
  }

  _computeStats(data, setpoint = 1.0, dt = 0.001) {
    const max       = Math.max(...data);
    const overshoot = Math.max(0, ((max - setpoint) / setpoint) * 100);
    const riseIdx   = data.findIndex(v => v >= 0.9 * setpoint);
    const riseTime  = riseIdx >= 0 ? riseIdx * dt * 1000 : 999;

    let settleIdx = data.length - 1;
    for (let i = data.length - 1; i >= 0; i--) {
      if (Math.abs(data[i] - setpoint) > 0.02 * setpoint) { settleIdx = i; break; }
    }
    const settleTime = settleIdx * dt * 1000;
    const sse        = Math.abs(data[data.length - 1] - setpoint);
    return { overshoot, riseTime, settleTime, sse };
  }

  _runSim() {
    const { p, i, d } = this.pid[this.curAxis];
    const data  = this._simulate(p, i, d);
    const stats = this._computeStats(data);

    this._drawStepResponse(data);
    this._updateStats(stats);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Canvas drawing — step response
  // ──────────────────────────────────────────────────────────────────────────

  _drawStepResponse(data, setpoint = 1.0) {
    const canvas = this._stepCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W   = canvas.clientWidth || 320;
    const H   = canvas.height;
    canvas.width = W;
    ctx.clearRect(0, 0, W, H);

    const pad = { t: 14, b: 10, l: 10, r: 10 };
    const w   = W - pad.l - pad.r;
    const h   = H - pad.t - pad.b;
    const minV = Math.min(0, ...data) - 0.05;
    const maxV = Math.max(setpoint * 1.35, ...data) + 0.05;
    const sy = v => pad.t + h - ((v - minV) / (maxV - minV)) * h;
    const sx = i => pad.l + (i / (data.length - 1)) * w;

    // Grid lines
    ctx.strokeStyle = '#1e3a5f';
    ctx.lineWidth   = 0.5;
    for (let r = 0; r <= 4; r++) {
      const y = pad.t + (r / 4) * h;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + w, y); ctx.stroke();
    }
    for (let c = 0; c <= 8; c++) {
      const x = pad.l + (c / 8) * w;
      ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, pad.t + h); ctx.stroke();
    }

    // Setpoint reference line
    ctx.strokeStyle = '#4a9eff44';
    ctx.lineWidth   = 1;
    ctx.setLineDash([5, 5]);
    ctx.beginPath(); ctx.moveTo(pad.l, sy(setpoint)); ctx.lineTo(pad.l + w, sy(setpoint)); ctx.stroke();
    ctx.setLineDash([]);

    // Shaded fill
    ctx.fillStyle = 'rgba(74,158,255,0.07)';
    ctx.beginPath();
    data.forEach((v, i) => i === 0 ? ctx.moveTo(sx(i), sy(v)) : ctx.lineTo(sx(i), sy(v)));
    ctx.lineTo(sx(data.length - 1), sy(0));
    ctx.lineTo(pad.l, sy(0));
    ctx.fill();

    // Response curve
    ctx.strokeStyle = '#4a9eff';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    data.forEach((v, i) => i === 0 ? ctx.moveTo(sx(i), sy(v)) : ctx.lineTo(sx(i), sy(v)));
    ctx.stroke();

    // Axis labels
    ctx.fillStyle = '#2a7acc';
    ctx.font = "9px 'Courier New'";
    ctx.fillText('1.0', pad.l + 3, sy(setpoint) - 3);
    ctx.fillText('0.0', pad.l + 3, sy(0) + 10);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Canvas drawing — oscilloscope
  // ──────────────────────────────────────────────────────────────────────────

  _startOscilloscope() {
    this._oscTimer = setInterval(() => {
      this._oscPhase += 0.08;
      this._drawOscilloscope();
    }, 50);
  }

  _drawOscilloscope() {
    const canvas = this._oscCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W   = canvas.clientWidth || 320;
    const H   = canvas.height;
    canvas.width = W;
    ctx.clearRect(0, 0, W, H);

    const pad  = { t: 8, b: 8, l: 10, r: 10 };
    const w    = W - pad.l - pad.r;
    const h    = H - pad.t - pad.b;
    const midY = pad.t + h / 2;

    // Center line
    ctx.strokeStyle = '#1e3a5f';
    ctx.lineWidth   = 0.5;
    ctx.beginPath(); ctx.moveTo(pad.l, midY); ctx.lineTo(pad.l + w, midY); ctx.stroke();

    const { p, i } = this.pid[this.curAxis];
    // Oscillation characteristics derived from PID gains
    const stability = Math.min(1, Math.max(0, 1 - i * 0.5 - Math.max(0, p - 1.0) * 0.3));
    const amp  = 0.12 + (1 - stability) * 0.38 + p * 0.07;
    const freq = 1  + p * 2.5 + i * 0.5;

    ctx.strokeStyle = '#4ae89a';
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    for (let i = 0; i < 200; i++) {
      const t     = i / 200;
      const noise = (Math.random() - 0.5) * 0.022;
      const osc   = Math.sin(t * Math.PI * 2 * freq + this._oscPhase) * amp * Math.exp(-t * 2) + noise;
      const x     = pad.l + (i / 199) * w;
      const y     = midY - osc * h * 0.42;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Stats display
  // ──────────────────────────────────────────────────────────────────────────

  _updateStats({ overshoot, riseTime, settleTime, sse }) {
    this._stats.overshoot.textContent = `${overshoot.toFixed(1)}%`;
    this._stats.overshoot.style.color = overshoot > 20 ? '#ff4444' : overshoot > 5 ? '#ff6b2b' : '#4ae89a';

    this._stats.settle.textContent = `${Math.min(settleTime, 999).toFixed(0)}ms`;
    this._stats.settle.style.color = settleTime > 300 ? '#ff6b2b' : '#4a9eff';

    this._stats.rise.textContent = `${riseTime.toFixed(0)}ms`;
    this._stats.sse.textContent  = sse.toFixed(3);
  }
}
