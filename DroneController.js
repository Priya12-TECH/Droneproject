/**
 * DroneController.js — PID Flight Controller for Three.js
 * Handles roll, pitch, yaw, and altitude using real discrete-time PID loops.
 * Designed to be driven by PIDTuning.js but works standalone too.
 *
 * Usage:
 *   import DroneController from './DroneController.js';
 *   const controller = new DroneController();
 *
 *   // Wire up the PID panel:
 *   pidPanel = new PIDTuning({ onPIDChange: (axis, p, i, d) => controller.setPID(axis, p, i, d) });
 *
 *   // In your Three.js animate() loop:
 *   const dt = clock.getDelta();
 *   controller.update(droneMesh, dt);
 */

export default class DroneController {

  constructor() {

    // ── PID state per axis ─────────────────────────────────────────────────
    this._axes = {
      roll: {
        p: 0.45, i: 0.12, d: 0.08,
        integral: 0, prevError: 0,
        setpoint: 0,          // target roll angle (rad)
        maxOutput: Math.PI / 6, // ±30°
      },
      pitch: {
        p: 0.42, i: 0.10, d: 0.07,
        integral: 0, prevError: 0,
        setpoint: 0,
        maxOutput: Math.PI / 6,
      },
      yaw: {
        p: 0.30, i: 0.20, d: 0.02,
        integral: 0, prevError: 0,
        setpoint: 0,
        maxOutput: Math.PI,
      },
      alt: {
        p: 0.55, i: 0.08, d: 0.15,
        integral: 0, prevError: 0,
        setpoint: 5.0,        // default hover altitude (metres)
        maxOutput: 20.0,      // max vertical velocity (m/s)
      },
    };

    // ── Physics state ──────────────────────────────────────────────────────
    this.velocity     = { x: 0, y: 0, z: 0 };   // m/s
    this.angularVel   = { x: 0, y: 0, z: 0 };   // rad/s
    this.gravity      = -9.81;                    // m/s²
    this.mass         = 0.95;                     // kg  (matches your DJI Mavic 3 profile)
    this.drag         = 0.35;                     // linear drag coefficient
    this.angularDrag  = 4.0;
    this.thrustFactor = 1.0;                      // from your Flight Parameters panel
    this.isArmed      = false;
    this.isHovering   = false;

    // ── Input commands (set these from your Controls panel) ───────────────
    this.input = { roll: 0, pitch: 0, yaw: 0, throttle: 0 }; // -1 to +1

    // ── Noise / wind simulation ────────────────────────────────────────────
    this.windEnabled   = false;
    this.windStrength  = 0.0;
    this._windVec      = { x: 0, y: 0, z: 0 };
    this._windTimer    = 0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Public API
  // ──────────────────────────────────────────────────────────────────────────

  /** Set gains from the PID tuning panel */
  setPID(axis, p, i, d) {
    if (!this._axes[axis]) return;
    this._axes[axis].p = p;
    this._axes[axis].i = i;
    this._axes[axis].d = d;
  }

  /** Get all gains for an axis (for display / export) */
  getPID(axis) {
    const a = this._axes[axis];
    return { p: a.p, i: a.i, d: a.d };
  }

  /** Set target altitude (metres) */
  setAltitude(alt) { this._axes.alt.setpoint = alt; }

  /** Set control input (call from your keyboard / gamepad handler, -1 to +1) */
  setInput(roll = 0, pitch = 0, yaw = 0, throttle = 0) {
    this.input.roll     = Math.max(-1, Math.min(1, roll));
    this.input.pitch    = Math.max(-1, Math.min(1, pitch));
    this.input.yaw      = Math.max(-1, Math.min(1, yaw));
    this.input.throttle = Math.max(-1, Math.min(1, throttle));
  }

  /** Arm the drone (enables motors) */
  arm()   { this.isArmed = true;  this._resetIntegrals(); }
  disarm(){ this.isArmed = false; this._resetIntegrals(); this.velocity = {x:0,y:0,z:0}; }

  /** Engage hover mode — altitude hold at current height */
  hover(droneMesh) {
    this.isHovering = true;
    this._axes.alt.setpoint = droneMesh.position.y;
  }

  /** Enable wind turbulence */
  setWind(enabled, strength = 1.0) {
    this.windEnabled  = enabled;
    this.windStrength = strength;
  }

  /**
   * Main update — call every frame from your Three.js animate() loop.
   * @param {THREE.Object3D} droneMesh  — your drone mesh / group
   * @param {number}         dt         — delta time in seconds (from THREE.Clock)
   */
  update(droneMesh, dt) {
    if (!this.isArmed || dt <= 0 || dt > 0.1) return;

    // Clamp dt to avoid physics blow-up on tab switch / resize
    const safeDt = Math.min(dt, 0.033);

    // ── Map stick inputs to setpoints ──────────────────────────────────────
    const maxTilt = Math.PI / 5; // 36° max lean
    this._axes.roll.setpoint  =  this.input.roll  * maxTilt;
    this._axes.pitch.setpoint = -this.input.pitch * maxTilt;
    // yaw is rate-based: add to current heading
    this._axes.yaw.setpoint   = droneMesh.rotation.y + this.input.yaw * 2.0 * safeDt;

    // Throttle adjusts altitude setpoint
    this._axes.alt.setpoint  += this.input.throttle * 8.0 * safeDt;
    this._axes.alt.setpoint   = Math.max(0, this._axes.alt.setpoint);

    // ── Compute PID outputs ────────────────────────────────────────────────
    const rollOut  = this._computePID('roll',  droneMesh.rotation.z, safeDt);
    const pitchOut = this._computePID('pitch', droneMesh.rotation.x, safeDt);
    const yawOut   = this._computePID('yaw',   droneMesh.rotation.y, safeDt);
    const altOut   = this._computePID('alt',   droneMesh.position.y, safeDt);

    // ── Angular velocity (rad/s) ───────────────────────────────────────────
    this.angularVel.z += (rollOut  - this.angularVel.z * this.angularDrag) * safeDt;
    this.angularVel.x += (pitchOut - this.angularVel.x * this.angularDrag) * safeDt;
    this.angularVel.y += (yawOut   - this.angularVel.y * this.angularDrag) * safeDt;

    droneMesh.rotation.z += this.angularVel.z * safeDt;
    droneMesh.rotation.x += this.angularVel.x * safeDt;
    droneMesh.rotation.y += this.angularVel.y * safeDt;

    // ── Clamp tilt (safety) ────────────────────────────────────────────────
    droneMesh.rotation.z = Math.max(-maxTilt, Math.min(maxTilt, droneMesh.rotation.z));
    droneMesh.rotation.x = Math.max(-maxTilt, Math.min(maxTilt, droneMesh.rotation.x));

    // ── Vertical velocity (altitude PID → thrust → gravity) ───────────────
    const thrust   = altOut * this.thrustFactor / this.mass;
    const gravComp = -this.gravity * safeDt;

    this.velocity.y += (thrust - gravComp - this.velocity.y * this.drag) * safeDt;

    // Lateral drift caused by tilt (simplified forward projection)
    const tiltFwd   = Math.sin(droneMesh.rotation.x);
    const tiltRight = Math.sin(droneMesh.rotation.z);
    this.velocity.x += (tiltRight * thrust * 0.6 - this.velocity.x * this.drag) * safeDt;
    this.velocity.z += (tiltFwd   * thrust * 0.6 - this.velocity.z * this.drag) * safeDt;

    // ── Wind disturbance ──────────────────────────────────────────────────
    if (this.windEnabled) {
      this._updateWind(safeDt);
      this.velocity.x += this._windVec.x * safeDt;
      this.velocity.z += this._windVec.z * safeDt;
      this.velocity.y += this._windVec.y * safeDt * 0.3;
    }

    // ── Apply velocity to position ────────────────────────────────────────
    droneMesh.position.x += this.velocity.x * safeDt;
    droneMesh.position.y += this.velocity.y * safeDt;
    droneMesh.position.z += this.velocity.z * safeDt;

    // ── Ground clamp ──────────────────────────────────────────────────────
    if (droneMesh.position.y < 0) {
      droneMesh.position.y = 0;
      this.velocity.y = 0;
      this._axes.alt.integral = 0; // reset windup on touchdown
    }
  }

  /**
   * Returns current telemetry for your HUD / PID panel.
   * Call after update().
   */
  getTelemetry(droneMesh) {
    const spd = Math.sqrt(
      this.velocity.x ** 2 + this.velocity.y ** 2 + this.velocity.z ** 2
    );
    return {
      altitude : parseFloat(droneMesh.position.y.toFixed(2)),
      speed    : parseFloat(spd.toFixed(2)),
      roll     : parseFloat((droneMesh.rotation.z * (180 / Math.PI)).toFixed(1)),
      pitch    : parseFloat((droneMesh.rotation.x * (180 / Math.PI)).toFixed(1)),
      yaw      : parseFloat((droneMesh.rotation.y * (180 / Math.PI)).toFixed(1)),
      vx       : parseFloat(this.velocity.x.toFixed(2)),
      vy       : parseFloat(this.velocity.y.toFixed(2)),
      vz       : parseFloat(this.velocity.z.toFixed(2)),
    };
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Internal helpers
  // ──────────────────────────────────────────────────────────────────────────

  _computePID(axisName, measured, dt) {
    const ax         = this._axes[axisName];
    const error      = ax.setpoint - measured;
    ax.integral     += error * dt;
    // Anti-windup: clamp integral so it doesn't run away
    const maxI       = ax.maxOutput / Math.max(ax.i, 0.001);
    ax.integral      = Math.max(-maxI, Math.min(maxI, ax.integral));
    const derivative = (error - ax.prevError) / dt;
    ax.prevError     = error;
    const output     = ax.p * error + ax.i * ax.integral + ax.d * derivative;
    return Math.max(-ax.maxOutput, Math.min(ax.maxOutput, output));
  }

  _resetIntegrals() {
    for (const ax of Object.values(this._axes)) {
      ax.integral  = 0;
      ax.prevError = 0;
    }
    this.velocity   = { x: 0, y: 0, z: 0 };
    this.angularVel = { x: 0, y: 0, z: 0 };
  }

  _updateWind(dt) {
    this._windTimer += dt;
    // Slowly rotating wind vector with gusty turbulence
    const angle   = this._windTimer * 0.3;
    const gust    = 1 + Math.sin(this._windTimer * 2.7) * 0.4 + (Math.random() - 0.5) * 0.2;
    this._windVec = {
      x: Math.cos(angle) * this.windStrength * gust,
      y: Math.sin(this._windTimer * 1.1) * this.windStrength * 0.15,
      z: Math.sin(angle) * this.windStrength * gust,
    };
  }
}
