import * as THREE from "three";
import { Drone } from "./components/Drone.js";
import {
  createOrbitControls,
  KeyboardFlightController,
} from "./utils/controls.js";
import {
  initWorldEnvironment,
  toggleEnvironmentPanel,
} from "../world-environment.js";
import "./styles.css";

const DRONE_PROFILES = {
  m350: {
    key: "m350",
    name: "DJI Matrice 350 RTK",
    weightKg: 3.8,
    batteryMah: 11700,
    maxSpeed: 13,
    maxAcceleration: 9.2,
    thrustFactor: 1.02,
    linearDamping: 0.91,
    payloadKg: 2.7,
    estimatedFlightTimeMin: 46,
  },
  m3e: {
    key: "m3e",
    name: "DJI Mavic 3 Enterprise",
    weightKg: 0.95,
    batteryMah: 5000,
    maxSpeed: 15,
    maxAcceleration: 13.5,
    thrustFactor: 1.0,
    linearDamping: 0.88,
    payloadKg: 0.35,
    estimatedFlightTimeMin: 42,
  },
  px4_450: {
    key: "px4_450",
    name: "PX4 F450 (Research Build)",
    weightKg: 1.35,
    batteryMah: 5200,
    maxSpeed: 11,
    maxAcceleration: 12,
    thrustFactor: 0.96,
    linearDamping: 0.885,
    payloadKg: 0.7,
    estimatedFlightTimeMin: 18,
  },
  nano: {
    key: "nano",
    name: "Nano Quad (Indoor)",
    weightKg: 0.25,
    batteryMah: 1500,
    maxSpeed: 7.5,
    maxAcceleration: 16,
    thrustFactor: 1.08,
    linearDamping: 0.86,
    payloadKg: 0.05,
    estimatedFlightTimeMin: 9,
  },
};

const DEFAULT_PROFILE_KEY = "m3e";

class DroneSimulatorApp {
  constructor(container) {
    this.container = container;
    this.clock = new THREE.Clock();
    this.elapsed = 0;
    this.followMode = false;
    this._telemetryLastSent = 0;
    this.telemetryInput = new THREE.Vector3();
    this.windVector = new THREE.Vector3();
    this.commandMode = "manual";
    this.scenario = "normal";

    this.obstacles = [];
    this.obstacleHelpers = [];
    this.vslamPoints = [];
    this.leftPanelOpen = true;
    this.rightPanelOpen = true;
    this.cameraDockOpen = true;
    this.environmentPanelOpen = false;
    this.viewportLocked = false;
    this.cameraViewMode = "auto";
    this.lastAutoCameraDirection = "forward";
    this.activeDroneProfileKey = DEFAULT_PROFILE_KEY;
    this.currentDroneProfile = { ...DRONE_PROFILES[DEFAULT_PROFILE_KEY] };
    this.lowBatteryWarned = false;
    this.batteryEmergencyActive = false;
    this.batteryEmergencyLanded = false;
    this._noticeTimer = null;

    this.ui = {
      panel: null,
      hud: null,
      toolbar: null,
      toggleTelemetryBtn: null,
      toggleControlBtn: null,
      toggleCameraBtn: null,
      toggleEnvironmentBtn: null,
      viewportLockBtn: null,
      viewportResetBtn: null,
      cameraDock: null,
      telemetryEls: null,
      log: null,
      sideCameraCanvas: null,
      sideCameraModeLabel: null,
      sideCameraModeButtons: [],
      scenarioSelect: null,
      droneModelSelect: null,
      droneWeightInput: null,
      droneBatteryInput: null,
      droneMaxSpeedInput: null,
      droneMaxAccelInput: null,
      droneThrustInput: null,
      droneDampingInput: null,
      dronePayloadInput: null,
      droneFlightTimeChip: null,
      droneApplyBtn: null,
      droneResetBtn: null,
      missionStatus: null,
      missionSpeed: null,
      missionSpeedValue: null,
      planningBtn: null,
      startMissionBtn: null,
      pauseMissionBtn: null,
      clearMissionBtn: null,
      scenarioBadge: null,
      introOverlay: null,
      notice: null,
      noticeText: null,
    };

    this.sideCamera = null;
    this.sideRenderer = null;

    this.audio = {
      context: null,
      master: null,
      engineOsc: null,
      engineGain: null,
      unlocked: false,
      collisionCooldown: 0,
    };

    this.mavlinkLogs = [];
    this.maxLogs = 150;
    this.mavlinkTick = {
      heartbeat: 0,
      attitude: 0,
      gps: 0,
      battery: 0,
      vision: 0,
      obstacle: 0,
    };

    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -0.55);

    this.waypoints = [];
    this.waypointMeshes = [];
    this.missionLine = null;
    this.planningMode = false;
    this.missionActive = false;
    this.missionPaused = false;
    this.missionIndex = 0;
    this.missionSpeed = 0.75;

    this.telemetry = {
      battery: 100,
      gps: 100,
      cpu: 12,
      status: "IDLE",
    };

    this.hudState = {
      alt: 0,
      spd: 0,
      pitch: 0,
      roll: 0,
      bat: 100,
      gps: 100,
      cpu: 12,
      status: "IDLE",
      flying: false,
    };

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1220);
    this.scene.fog = new THREE.Fog(0x0b1220, 20, 120);

    this.camera = new THREE.PerspectiveCamera(
      60,
      window.innerWidth / window.innerHeight,
      0.1,
      500,
    );
    this.camera.position.set(8, 6, 10);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.tabIndex = 0;
    this.container.appendChild(this.renderer.domElement);

    this.controls = createOrbitControls(this.camera, this.renderer.domElement);
    this.keyboard = new KeyboardFlightController();

    this.drone = new Drone();
    this.drone.setFlightConfig(this.currentDroneProfile);
    this.drone.group.position.set(0, 2, 0);
    this.scene.add(this.drone.group);

    this.followOffset = new THREE.Vector3(8, 5, 9);

    this._setupScene();
    this._setupEvents();
    this._setupViewportToolbar();
    this._setupSystemNotice();
    this._setupHud();
    this._setupControlPanel();
    this._setupCameraDock();
    this._setupIntroOverlay();
    this._setupAudio();
    this._setupTelemetryMessaging();
    this._setScenario("normal");
    this._logMavlink("SYSTEM", "DroneSimX ready. Select scenario and fly.");

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  _setupScene() {
    // Use shared 3D environment system (ground, fog, lighting, and env switch UI).
    initWorldEnvironment(this.scene, this.renderer);

    const axes = new THREE.AxesHelper(2.5);
    this.scene.add(axes);

    // Demo obstacles, also used by avoidance-force placeholder.
    const obstacleMaterial = new THREE.MeshStandardMaterial({
      color: 0x3c4c66,
      roughness: 0.65,
      metalness: 0.1,
    });
    const obstacleA = new THREE.Mesh(
      new THREE.BoxGeometry(2, 4, 2),
      obstacleMaterial,
    );
    obstacleA.position.set(5, 2, -3);
    obstacleA.castShadow = true;
    obstacleA.receiveShadow = true;

    const obstacleB = new THREE.Mesh(
      new THREE.CylinderGeometry(1.1, 1.1, 3.5, 20),
      obstacleMaterial,
    );
    obstacleB.position.set(-4, 1.75, 4);
    obstacleB.castShadow = true;
    obstacleB.receiveShadow = true;

    const obstacleC = new THREE.Mesh(
      new THREE.BoxGeometry(3.2, 2.8, 2.4),
      obstacleMaterial,
    );
    obstacleC.position.set(16, 1.4, 12);
    obstacleC.castShadow = true;
    obstacleC.receiveShadow = true;

    const obstacleD = new THREE.Mesh(
      new THREE.CylinderGeometry(1.4, 1.4, 4.2, 20),
      obstacleMaterial,
    );
    obstacleD.position.set(-18, 2.1, -14);
    obstacleD.castShadow = true;
    obstacleD.receiveShadow = true;

    const obstacleE = new THREE.Mesh(
      new THREE.BoxGeometry(2.6, 3.6, 2.6),
      obstacleMaterial,
    );
    obstacleE.position.set(10, 1.8, -18);
    obstacleE.castShadow = true;
    obstacleE.receiveShadow = true;

    this.scene.add(obstacleA, obstacleB, obstacleC, obstacleD, obstacleE);
    this.obstacles = [obstacleA, obstacleB, obstacleC, obstacleD, obstacleE];
    this.drone.setObstacles(this.obstacles);
    this.drone.setAvoidanceEnabled(false);

    this.obstacles.forEach((obstacle) => {
      const helper = new THREE.BoxHelper(obstacle, 0x00ffd5);
      helper.visible = false;
      this.scene.add(helper);
      this.obstacleHelpers.push(helper);
    });
  }

  _setupEvents() {
    this.renderer.domElement.addEventListener("pointerdown", (event) => {
      this.renderer.domElement.focus();
      this._unlockAudio();
      this._handleMissionPointing(event);
    });

    window.addEventListener("resize", () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(window.innerWidth, window.innerHeight);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this._resizeSideCamera();
    });

    window.addEventListener("keydown", (event) => {
      this._unlockAudio();

      if (event.code === "KeyC") {
        this.followMode = !this.followMode;
        this._playUiBeep(720, 0.06, 0.016);
        this._logMavlink(
          "COMMAND_LONG",
          `CAMERA_MODE ${this.followMode ? "FOLLOW" : "FREE"}`,
        );
      }

      if (event.code === "Space") {
        event.preventDefault();
        this.commandMode = "hover";
        this._playUiBeep(640, 0.08, 0.018);
        this._logMavlink("COMMAND_LONG", "SET_MODE HOVER");
      }

      if (event.code === "KeyL") {
        event.preventDefault();
        this.commandMode = "land";
        this._playUiBeep(310, 0.12, 0.02);
        this._logMavlink("COMMAND_LONG", "NAV_LAND");
      }

      if (event.code === "KeyM") {
        event.preventDefault();
        this._togglePlanningMode();
      }

      if (event.code === "Enter") {
        event.preventDefault();
        this._startMission();
      }
    });
  }

  _setupViewportToolbar() {
    const toolbar = document.createElement("div");
    toolbar.className = "viewport-toolbar";
    toolbar.innerHTML = `
      <button type="button" class="toolbar-btn toolbar-btn--active" data-action="telemetry">Telemetry</button>
      <button type="button" class="toolbar-btn toolbar-btn--active" data-action="controls">Controls</button>
      <button type="button" class="toolbar-btn toolbar-btn--active" data-action="camera">Camera</button>
      <button type="button" class="toolbar-btn" data-action="environment">Environment</button>
      <button type="button" class="toolbar-btn" data-action="lock">Lock Viewport</button>
      <button type="button" class="toolbar-btn" data-action="reset">Reset View</button>
    `.trim();

    document.body.appendChild(toolbar);
    this.ui.toolbar = toolbar;

    this.ui.toggleTelemetryBtn = toolbar.querySelector(
      '[data-action="telemetry"]',
    );
    this.ui.toggleControlBtn = toolbar.querySelector(
      '[data-action="controls"]',
    );
    this.ui.toggleCameraBtn = toolbar.querySelector('[data-action="camera"]');
    this.ui.toggleEnvironmentBtn = toolbar.querySelector(
      '[data-action="environment"]',
    );
    this.ui.viewportLockBtn = toolbar.querySelector('[data-action="lock"]');
    this.ui.viewportResetBtn = toolbar.querySelector('[data-action="reset"]');

    this.ui.toggleTelemetryBtn.addEventListener("click", () =>
      this._toggleTelemetryPanel(),
    );
    this.ui.toggleControlBtn.addEventListener("click", () =>
      this._toggleControlPanel(),
    );
    this.ui.toggleCameraBtn.addEventListener("click", () =>
      this._toggleCameraDock(),
    );
    if (this.ui.toggleEnvironmentBtn) {
      this.ui.toggleEnvironmentBtn.addEventListener("click", () =>
        this._toggleEnvironmentPanel(),
      );
    }
    this.ui.viewportLockBtn.addEventListener("click", () =>
      this._toggleViewportLock(),
    );
    this.ui.viewportResetBtn.addEventListener("click", () =>
      this._resetViewport(),
    );
  }

  _setupSystemNotice() {
    const notice = document.createElement("div");
    notice.className = "sim-notice";
    notice.innerHTML = `<span class="sim-notice__text"></span>`;
    document.body.appendChild(notice);

    this.ui.notice = notice;
    this.ui.noticeText = notice.querySelector(".sim-notice__text");
  }

  _showSystemNotice(message, tone = "info", durationMs = 3000) {
    if (!this.ui.notice || !this.ui.noticeText) return;

    this.ui.noticeText.textContent = message;
    this.ui.notice.classList.remove(
      "sim-notice--warn",
      "sim-notice--danger",
      "sim-notice--show",
    );
    if (tone === "warn") this.ui.notice.classList.add("sim-notice--warn");
    if (tone === "danger") this.ui.notice.classList.add("sim-notice--danger");
    this.ui.notice.classList.add("sim-notice--show");

    if (this._noticeTimer) {
      window.clearTimeout(this._noticeTimer);
    }
    this._noticeTimer = window.setTimeout(() => {
      if (this.ui.notice) {
        this.ui.notice.classList.remove("sim-notice--show");
      }
    }, durationMs);
  }

  _toggleTelemetryPanel() {
    this.leftPanelOpen = !this.leftPanelOpen;
    if (this.ui.hud) {
      this.ui.hud.classList.toggle("hud--collapsed", !this.leftPanelOpen);
    }
    if (this.ui.toggleTelemetryBtn) {
      this.ui.toggleTelemetryBtn.classList.toggle(
        "toolbar-btn--active",
        this.leftPanelOpen,
      );
    }
  }

  _toggleControlPanel() {
    this.rightPanelOpen = !this.rightPanelOpen;
    if (this.ui.panel) {
      this.ui.panel.classList.toggle(
        "sim-panel--collapsed",
        !this.rightPanelOpen,
      );
    }
    if (this.ui.toggleControlBtn) {
      this.ui.toggleControlBtn.classList.toggle(
        "toolbar-btn--active",
        this.rightPanelOpen,
      );
    }
  }

  _toggleCameraDock() {
    this.cameraDockOpen = !this.cameraDockOpen;
    if (this.ui.cameraDock) {
      this.ui.cameraDock.classList.toggle(
        "camera-dock--collapsed",
        !this.cameraDockOpen,
      );
    }
    if (this.ui.toggleCameraBtn) {
      this.ui.toggleCameraBtn.classList.toggle(
        "toolbar-btn--active",
        this.cameraDockOpen,
      );
    }
  }

  _toggleEnvironmentPanel() {
    this.environmentPanelOpen = toggleEnvironmentPanel();
    if (this.ui.toggleEnvironmentBtn) {
      this.ui.toggleEnvironmentBtn.classList.toggle(
        "toolbar-btn--active",
        this.environmentPanelOpen,
      );
    }
  }

  _toggleViewportLock() {
    this.viewportLocked = !this.viewportLocked;
    this.controls.enabled = !this.viewportLocked;
    if (this.ui.viewportLockBtn) {
      this.ui.viewportLockBtn.textContent = this.viewportLocked
        ? "Unlock Viewport"
        : "Lock Viewport";
      this.ui.viewportLockBtn.classList.toggle(
        "toolbar-btn--active",
        this.viewportLocked,
      );
    }
    this._logMavlink(
      "COMMAND_LONG",
      this.viewportLocked ? "VIEWPORT_LOCK ON" : "VIEWPORT_LOCK OFF",
    );
  }

  _resetViewport() {
    this.followMode = false;
    this.viewportLocked = false;
    this.controls.enabled = true;
    this.camera.position.set(8, 6, 10);
    this.controls.target.set(0, 2, 0);
    this.controls.update();
    if (this.ui.viewportLockBtn) {
      this.ui.viewportLockBtn.textContent = "Lock Viewport";
      this.ui.viewportLockBtn.classList.remove("toolbar-btn--active");
    }
    this._playUiBeep(520, 0.07, 0.015);
    this._logMavlink("COMMAND_LONG", "VIEWPORT_RESET");
  }

  _setupHud() {
    const hud = document.createElement("div");
    hud.className = "hud hud--telemetry";
    hud.innerHTML = `
      <div class="telemetry-panel" id="telemetry-panel">
        <div class="telemetry-header">
          <span class="telemetry-title">Telemetry</span>
          <div class="telemetry-status-badge">
            <span class="telemetry-status-dot" id="tel-status-dot"></span>
            <span id="tel-status-text">IDLE</span>
          </div>
        </div>

        <div class="telemetry-metric-grid">
          <div class="telemetry-metric-cell">
            <div class="telemetry-metric-label">Altitude</div>
            <div class="telemetry-metric-value" id="tel-alt">0</div>
            <div class="telemetry-metric-unit">meters</div>
          </div>
          <div class="telemetry-metric-cell">
            <div class="telemetry-metric-label">Speed</div>
            <div class="telemetry-metric-value" id="tel-spd">0.0</div>
            <div class="telemetry-metric-unit">m/s</div>
          </div>
          <div class="telemetry-metric-cell">
            <div class="telemetry-metric-label">Pitch</div>
            <div class="telemetry-metric-value" id="tel-pitch">0</div>
            <div class="telemetry-metric-unit">deg</div>
          </div>
          <div class="telemetry-metric-cell">
            <div class="telemetry-metric-label">Roll</div>
            <div class="telemetry-metric-value" id="tel-roll">0</div>
            <div class="telemetry-metric-unit">deg</div>
          </div>
        </div>

        <div class="telemetry-bars">
          <div class="telemetry-bar-row">
            <div class="telemetry-bar-meta">
              <span class="telemetry-bar-label">Battery</span>
              <span class="telemetry-bar-val" id="tel-bat-val">100%</span>
            </div>
            <div class="telemetry-bar-track"><div class="telemetry-bar-fill telemetry-bar-battery" id="tel-bat" style="width:100%"></div></div>
          </div>
          <div class="telemetry-bar-row">
            <div class="telemetry-bar-meta">
              <span class="telemetry-bar-label">GPS Signal</span>
              <span class="telemetry-bar-val" id="tel-gps-val">100%</span>
            </div>
            <div class="telemetry-bar-track"><div class="telemetry-bar-fill telemetry-bar-gps" id="tel-gps" style="width:100%"></div></div>
          </div>
          <div class="telemetry-bar-row">
            <div class="telemetry-bar-meta">
              <span class="telemetry-bar-label">CPU Load</span>
              <span class="telemetry-bar-val" id="tel-cpu-val">12%</span>
            </div>
            <div class="telemetry-bar-track"><div class="telemetry-bar-fill telemetry-bar-cpu" id="tel-cpu" style="width:12%"></div></div>
          </div>
        </div>

        <div class="telemetry-controls">
          <div class="telemetry-controls-label">Controls</div>
          <div class="telemetry-btn-grid">
            <button type="button" class="telemetry-btn" data-tel-action="forward">▲ Fwd</button>
            <button type="button" class="telemetry-btn" data-tel-action="backward">▼ Back</button>
            <button type="button" class="telemetry-btn" data-tel-action="up">↑ Up</button>
            <button type="button" class="telemetry-btn" data-tel-action="down">↓ Down</button>
            <button type="button" class="telemetry-btn" data-tel-action="left">◀ Left</button>
            <button type="button" class="telemetry-btn" data-tel-action="right">▶ Right</button>
            <button type="button" class="telemetry-btn telemetry-btn--hover" data-tel-action="hover">⏸ Hover</button>
            <button type="button" class="telemetry-btn telemetry-btn--land" data-tel-action="land">⏬ Land</button>
          </div>
        </div>

        <div class="telemetry-footer">
          <span class="telemetry-footer-info">Drone v1.0 | UID-7E3F</span>
          <span class="telemetry-footer-time" id="tel-clock">00:00:00</span>
        </div>
      </div>
    `.trim();

    hud.addEventListener("click", (event) => {
      const button = event.target.closest("[data-tel-action]");
      if (!button) return;
      this._handleTelemetryAction(button.dataset.telAction);
      button.classList.add("telemetry-btn--pressed");
      window.setTimeout(
        () => button.classList.remove("telemetry-btn--pressed"),
        180,
      );
    });

    document.body.appendChild(hud);
    this.ui.hud = hud;
    this.ui.telemetryEls = {
      statusText: hud.querySelector("#tel-status-text"),
      statusDot: hud.querySelector("#tel-status-dot"),
      alt: hud.querySelector("#tel-alt"),
      spd: hud.querySelector("#tel-spd"),
      pitch: hud.querySelector("#tel-pitch"),
      roll: hud.querySelector("#tel-roll"),
      batVal: hud.querySelector("#tel-bat-val"),
      gpsVal: hud.querySelector("#tel-gps-val"),
      cpuVal: hud.querySelector("#tel-cpu-val"),
      bat: hud.querySelector("#tel-bat"),
      gps: hud.querySelector("#tel-gps"),
      cpu: hud.querySelector("#tel-cpu"),
      clock: hud.querySelector("#tel-clock"),
    };
  }

  _setupControlPanel() {
    const profileOptions = Object.values(DRONE_PROFILES)
      .map(
        (profile) =>
          `<option value="${profile.key}">${profile.name}</option>`,
      )
      .join("");

    const panel = document.createElement("aside");
    panel.className = "sim-panel";
    panel.innerHTML = `
      <div class="sim-panel__header">
        <div>
          <h2>DroneSimX</h2>
          <p>Pixhawk + Jetson Demo Stack</p>
        </div>
        <span class="sim-panel__badge" id="scenario-badge">NORMAL</span>
      </div>

      <section class="sim-section">
        <div class="sim-row">
          <label for="drone-model-select" class="sim-label sim-label--tight">Drone Profile</label>
          <span id="drone-flight-time" class="sim-chip">42 min</span>
        </div>
        <select id="drone-model-select" class="sim-select">
          ${profileOptions}
          <option value="custom">Custom Manual</option>
        </select>

        <p class="sim-note">Choose a real-world profile or switch to Custom Manual for tuning.</p>

        <div class="sim-row sim-row--gap">
          <button id="btn-reset-drone" class="sim-btn">Load Profile</button>
        </div>
      </section>

      <section class="sim-section">
        <div class="sim-row">
          <span class="sim-label">Flight Parameters</span>
          <span class="sim-chip">Editable</span>
        </div>
        <div class="sim-input-grid">
          <label class="sim-label sim-label--tight" for="drone-weight">Weight (kg)</label>
          <input id="drone-weight" class="sim-input" type="number" min="0.1" max="12" step="0.05" />

          <label class="sim-label sim-label--tight" for="drone-battery">Battery (mAh)</label>
          <input id="drone-battery" class="sim-input" type="number" min="400" max="30000" step="50" />

          <label class="sim-label sim-label--tight" for="drone-max-speed">Max Speed (m/s)</label>
          <input id="drone-max-speed" class="sim-input" type="number" min="3" max="30" step="0.1" />

          <label class="sim-label sim-label--tight" for="drone-max-accel">Max Accel (m/s²)</label>
          <input id="drone-max-accel" class="sim-input" type="number" min="4" max="28" step="0.1" />

          <label class="sim-label sim-label--tight" for="drone-thrust">Thrust Factor</label>
          <input id="drone-thrust" class="sim-input" type="number" min="0.60" max="1.60" step="0.01" />

          <label class="sim-label sim-label--tight" for="drone-damping">Damping (0-1)</label>
          <input id="drone-damping" class="sim-input" type="number" min="0.70" max="0.98" step="0.005" />

          <label class="sim-label sim-label--tight" for="drone-payload">Payload (kg)</label>
          <input id="drone-payload" class="sim-input" type="number" min="0" max="6" step="0.05" />
        </div>

        <p class="sim-note">Higher thrust improves climb and punch-out; lower thrust feels heavy and sluggish.</p>

        <div class="sim-row sim-row--gap">
          <button id="btn-apply-drone" class="sim-btn sim-btn--accent">Apply Profile</button>
          <button id="btn-reset-drone-params" class="sim-btn">Reset Params</button>
        </div>
      </section>

      <section class="sim-section">
        <label for="scenario-select" class="sim-label">Scenario</label>
        <select id="scenario-select" class="sim-select">
          <option value="normal">Normal</option>
          <option value="wind">Wind Disturbance</option>
          <option value="gps-denied">GPS Denied + VSLAM</option>
          <option value="motor-failure">Motor Failure</option>
          <option value="obstacle">Obstacle Avoidance</option>
        </select>
      </section>

      <section class="sim-section">
        <div class="sim-row">
          <span class="sim-label">Mission</span>
          <span class="sim-chip" id="mission-status">IDLE</span>
        </div>
        <div class="sim-row sim-row--gap">
          <button id="btn-plan" class="sim-btn">Add Waypoint (M)</button>
          <button id="btn-start-mission" class="sim-btn sim-btn--accent">Start (Enter)</button>
        </div>
        <div class="sim-row sim-row--gap">
          <button id="btn-pause-mission" class="sim-btn">Pause</button>
          <button id="btn-clear-mission" class="sim-btn sim-btn--danger">Clear</button>
        </div>
        <label for="mission-speed" class="sim-label">Mission Speed</label>
        <div class="sim-row">
          <input id="mission-speed" class="sim-range" type="range" min="0.3" max="1.4" step="0.05" value="0.75" />
          <span id="mission-speed-value" class="sim-chip">0.75x</span>
        </div>
      </section>

      <section class="sim-section sim-section--log">
        <div class="sim-row">
          <span class="sim-label">MAVLink Stream</span>
          <span class="sim-chip">LIVE</span>
        </div>
        <pre id="mavlink-log" class="sim-log"></pre>
      </section>
    `.trim();

    document.body.appendChild(panel);

    this.ui.panel = panel;
    this.ui.log = panel.querySelector("#mavlink-log");
    this.ui.droneModelSelect = panel.querySelector("#drone-model-select");
    this.ui.droneWeightInput = panel.querySelector("#drone-weight");
    this.ui.droneBatteryInput = panel.querySelector("#drone-battery");
    this.ui.droneMaxSpeedInput = panel.querySelector("#drone-max-speed");
    this.ui.droneMaxAccelInput = panel.querySelector("#drone-max-accel");
    this.ui.droneThrustInput = panel.querySelector("#drone-thrust");
    this.ui.droneDampingInput = panel.querySelector("#drone-damping");
    this.ui.dronePayloadInput = panel.querySelector("#drone-payload");
    this.ui.droneFlightTimeChip = panel.querySelector("#drone-flight-time");
    this.ui.droneApplyBtn = panel.querySelector("#btn-apply-drone");
    this.ui.droneResetBtn = panel.querySelector("#btn-reset-drone");
    this.ui.scenarioSelect = panel.querySelector("#scenario-select");
    this.ui.missionStatus = panel.querySelector("#mission-status");
    this.ui.missionSpeed = panel.querySelector("#mission-speed");
    this.ui.missionSpeedValue = panel.querySelector("#mission-speed-value");
    this.ui.planningBtn = panel.querySelector("#btn-plan");
    this.ui.startMissionBtn = panel.querySelector("#btn-start-mission");
    this.ui.pauseMissionBtn = panel.querySelector("#btn-pause-mission");
    this.ui.clearMissionBtn = panel.querySelector("#btn-clear-mission");
    this.ui.scenarioBadge = panel.querySelector("#scenario-badge");

    if (this.ui.droneModelSelect) {
      this.ui.droneModelSelect.value = DEFAULT_PROFILE_KEY;
    }

    this.ui.scenarioSelect.addEventListener("change", (event) => {
      this._setScenario(event.target.value);
    });

    this.ui.droneModelSelect.addEventListener("change", () => {
      const selected = this.ui.droneModelSelect.value;
      if (selected === "custom") {
        this._showSystemNotice(
          "Custom mode: edit parameters and click Apply Profile.",
          "info",
          2600,
        );
        return;
      }
      this._applyDroneProfile(selected, DRONE_PROFILES[selected]);
    });

    this.ui.droneApplyBtn.addEventListener("click", () => {
      const customProfile = this._getDroneProfileFromInputs();
      this._applyDroneProfile("custom", customProfile);
      if (this.ui.droneModelSelect) {
        this.ui.droneModelSelect.value = "custom";
      }
    });

    this.ui.droneResetBtn.addEventListener("click", () => {
      const selected = this.ui.droneModelSelect?.value;
      const fallbackKey =
        selected && selected !== "custom" ? selected : DEFAULT_PROFILE_KEY;
      this._applyDroneProfile(fallbackKey, DRONE_PROFILES[fallbackKey]);
      if (this.ui.droneModelSelect) {
        this.ui.droneModelSelect.value = fallbackKey;
      }
    });

    panel.querySelector("#btn-reset-drone-params")?.addEventListener("click", () => {
      const selected = this.ui.droneModelSelect?.value;
      const fallbackKey =
        selected && selected !== "custom" ? selected : DEFAULT_PROFILE_KEY;
      this._applyDroneProfile(fallbackKey, DRONE_PROFILES[fallbackKey]);
      if (this.ui.droneModelSelect) {
        this.ui.droneModelSelect.value = fallbackKey;
      }
    });

    this.ui.planningBtn.addEventListener("click", () =>
      this._togglePlanningMode(),
    );
    this.ui.startMissionBtn.addEventListener("click", () =>
      this._startMission(),
    );
    this.ui.pauseMissionBtn.addEventListener("click", () =>
      this._togglePauseMission(),
    );
    this.ui.clearMissionBtn.addEventListener("click", () =>
      this._clearMission(),
    );
    this.ui.missionSpeed.addEventListener("input", () => {
      this.missionSpeed = Number(this.ui.missionSpeed.value);
      this.ui.missionSpeedValue.textContent = `${this.missionSpeed.toFixed(2)}x`;
    });

    this._applyDroneProfile(DEFAULT_PROFILE_KEY, DRONE_PROFILES[DEFAULT_PROFILE_KEY]);
  }

  _syncDroneProfileInputs(profile) {
    if (!profile) return;
    if (this.ui.droneWeightInput)
      this.ui.droneWeightInput.value = profile.weightKg.toFixed(2);
    if (this.ui.droneBatteryInput)
      this.ui.droneBatteryInput.value = String(Math.round(profile.batteryMah));
    if (this.ui.droneMaxSpeedInput)
      this.ui.droneMaxSpeedInput.value = profile.maxSpeed.toFixed(1);
    if (this.ui.droneMaxAccelInput)
      this.ui.droneMaxAccelInput.value = profile.maxAcceleration.toFixed(1);
    if (this.ui.droneThrustInput)
      this.ui.droneThrustInput.value = profile.thrustFactor.toFixed(2);
    if (this.ui.droneDampingInput)
      this.ui.droneDampingInput.value = profile.linearDamping.toFixed(3);
    if (this.ui.dronePayloadInput)
      this.ui.dronePayloadInput.value = profile.payloadKg.toFixed(2);
    if (this.ui.droneFlightTimeChip) {
      this.ui.droneFlightTimeChip.textContent = `${Math.round(profile.estimatedFlightTimeMin)} min`;
    }
  }

  _getDroneProfileFromInputs() {
    const numberValue = (input, fallback) => {
      const n = Number(input?.value);
      return Number.isFinite(n) ? n : fallback;
    };

    const baseline = this.currentDroneProfile || DRONE_PROFILES[DEFAULT_PROFILE_KEY];
    const weightKg = THREE.MathUtils.clamp(
      numberValue(this.ui.droneWeightInput, baseline.weightKg),
      0.1,
      12,
    );
    const batteryMah = THREE.MathUtils.clamp(
      numberValue(this.ui.droneBatteryInput, baseline.batteryMah),
      400,
      30000,
    );
    const maxSpeed = THREE.MathUtils.clamp(
      numberValue(this.ui.droneMaxSpeedInput, baseline.maxSpeed),
      3,
      30,
    );
    const maxAcceleration = THREE.MathUtils.clamp(
      numberValue(this.ui.droneMaxAccelInput, baseline.maxAcceleration),
      4,
      28,
    );
    const thrustFactor = THREE.MathUtils.clamp(
      numberValue(this.ui.droneThrustInput, baseline.thrustFactor ?? 1.0),
      0.6,
      1.6,
    );
    const linearDamping = THREE.MathUtils.clamp(
      numberValue(this.ui.droneDampingInput, baseline.linearDamping),
      0.7,
      0.98,
    );
    const payloadKg = THREE.MathUtils.clamp(
      numberValue(this.ui.dronePayloadInput, baseline.payloadKg),
      0,
      6,
    );

    const estimatedFlightTimeMin = THREE.MathUtils.clamp(
      (batteryMah / 220) * (1 / Math.max(0.35, weightKg + payloadKg * 0.5)),
      5,
      70,
    );

    return {
      key: "custom",
      name: "Custom Manual",
      weightKg,
      batteryMah,
      maxSpeed,
      maxAcceleration,
      thrustFactor,
      linearDamping,
      payloadKg,
      estimatedFlightTimeMin,
    };
  }

  _applyDroneProfile(profileKey, profileData) {
    const source = profileData || DRONE_PROFILES[profileKey];
    if (!source) return;

    const profile = {
      key: profileKey,
      name: source.name,
      weightKg: THREE.MathUtils.clamp(Number(source.weightKg) || 1.0, 0.1, 12),
      batteryMah: THREE.MathUtils.clamp(
        Number(source.batteryMah) || 5200,
        400,
        30000,
      ),
      maxSpeed: THREE.MathUtils.clamp(Number(source.maxSpeed) || 11, 3, 30),
      maxAcceleration: THREE.MathUtils.clamp(
        Number(source.maxAcceleration) || 13,
        4,
        28,
      ),
      thrustFactor: THREE.MathUtils.clamp(
        Number(source.thrustFactor) || 1.0,
        0.6,
        1.6,
      ),
      linearDamping: THREE.MathUtils.clamp(
        Number(source.linearDamping) || 0.88,
        0.7,
        0.98,
      ),
      payloadKg: THREE.MathUtils.clamp(Number(source.payloadKg) || 0, 0, 6),
      estimatedFlightTimeMin: THREE.MathUtils.clamp(
        Number(source.estimatedFlightTimeMin) || 20,
        5,
        70,
      ),
    };

    this.currentDroneProfile = profile;
    this.activeDroneProfileKey = profileKey;
    this.drone.setFlightConfig({
      maxSpeed: profile.maxSpeed,
      maxAcceleration: profile.maxAcceleration,
      thrustFactor: profile.thrustFactor,
      linearDamping: profile.linearDamping,
      weightKg: profile.weightKg,
      payloadKg: profile.payloadKg,
    });
    this._syncDroneProfileInputs(profile);

    this.telemetry.battery = 100;
    this.lowBatteryWarned = false;
    this.batteryEmergencyActive = false;
    this.batteryEmergencyLanded = false;
    if (this.commandMode === "emergency-land") this.commandMode = "hover";

    this._showSystemNotice(`Profile applied: ${profile.name}`, "info", 2200);
    this._logMavlink(
      "PARAM_VALUE",
      `PROFILE=${profile.name} MASS=${profile.weightKg.toFixed(2)}kg BAT=${Math.round(profile.batteryMah)}mAh VMAX=${profile.maxSpeed.toFixed(1)} ACC=${profile.maxAcceleration.toFixed(1)} THR=${profile.thrustFactor.toFixed(2)}`,
    );
  }

  _setupIntroOverlay() {
    const overlay = document.createElement("div");
    overlay.className = "intro-overlay";
    overlay.innerHTML = `
      <div class="intro-card">
        <h1>DroneSimX Demo</h1>
        <p>Browser-based Pixhawk + Jetson simulation</p>
        <ul>
          <li>5 real-world scenarios</li>
          <li>MAVLink telemetry stream</li>
          <li>Waypoint mission planning</li>
        </ul>
        <button id="btn-start-demo" class="sim-btn sim-btn--accent">Launch Simulation</button>
      </div>
    `.trim();
    document.body.appendChild(overlay);
    this.ui.introOverlay = overlay;

    overlay.querySelector("#btn-start-demo").addEventListener("click", () => {
      overlay.classList.add("intro-overlay--hide");
      this._unlockAudio();
      this.renderer.domElement.focus();
    });
  }

  _setupCameraDock() {
    const dock = document.createElement("aside");
    dock.className = "camera-dock";
    dock.innerHTML = `
      <div class="camera-dock__header">
        <span class="sim-label">Camera View</span>
        <span id="camera-mode-label" class="sim-chip">AUTO</span>
      </div>
      <canvas id="side-camera" class="side-camera"></canvas>
      <div class="camera-modes">
        <button type="button" class="camera-mode-btn camera-mode-btn--active" data-camera="auto">Auto</button>
        <button type="button" class="camera-mode-btn" data-camera="forward">Forward</button>
        <button type="button" class="camera-mode-btn" data-camera="backward">Backward</button>
        <button type="button" class="camera-mode-btn" data-camera="left">Left</button>
        <button type="button" class="camera-mode-btn" data-camera="right">Right</button>
      </div>
    `.trim();
    document.body.appendChild(dock);

    this.ui.cameraDock = dock;
    this.ui.sideCameraCanvas = dock.querySelector("#side-camera");
    this.ui.sideCameraModeLabel = dock.querySelector("#camera-mode-label");
    this.ui.sideCameraModeButtons = Array.from(
      dock.querySelectorAll(".camera-mode-btn"),
    );

    this.ui.sideCameraModeButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        this._setCameraMode(btn.dataset.camera || "forward");
      });
    });

    this.sideCamera = new THREE.PerspectiveCamera(75, 1.7, 0.1, 220);
    this.sideRenderer = new THREE.WebGLRenderer({
      canvas: this.ui.sideCameraCanvas,
      antialias: true,
      alpha: true,
    });
    this.sideRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this._resizeSideCamera();
    this._setCameraMode("auto");
  }

  _resizeSideCamera() {
    if (!this.sideRenderer || !this.ui.sideCameraCanvas || !this.sideCamera)
      return;

    const rect = this.ui.sideCameraCanvas.getBoundingClientRect();
    const width = Math.max(220, Math.floor(rect.width || 340));
    const height = Math.max(140, Math.floor(rect.height || width * 0.58));
    this.sideRenderer.setSize(width, height, false);
    this.sideCamera.aspect = width / height;
    this.sideCamera.updateProjectionMatrix();
  }

  _setCameraMode(mode) {
    const allowed = new Set(["auto", "forward", "backward", "left", "right"]);
    this.cameraViewMode = allowed.has(mode) ? mode : "auto";

    if (this.ui.sideCameraModeLabel) {
      this.ui.sideCameraModeLabel.textContent =
        this.cameraViewMode === "auto"
          ? `AUTO • ${this.lastAutoCameraDirection.toUpperCase()}`
          : this.cameraViewMode.toUpperCase();
    }
    if (this.ui.sideCameraModeButtons?.length) {
      this.ui.sideCameraModeButtons.forEach((btn) => {
        const active = btn.dataset.camera === this.cameraViewMode;
        btn.classList.toggle("camera-mode-btn--active", active);
      });
    }
    this._playUiBeep(560, 0.06, 0.012);
    this._logMavlink(
      "COMMAND_LONG",
      `CAMERA_VIEW ${this.cameraViewMode.toUpperCase()}`,
    );
  }

  _updateSideCamera() {
    if (!this.sideCamera || !this.sideRenderer) return;

    const velocityFlat = this.drone.velocity.clone();
    velocityFlat.y = 0;

    let motionDir = new THREE.Vector3(0, 0, -1);
    if (velocityFlat.length() > 0.08) {
      motionDir.copy(velocityFlat.normalize());
      if (Math.abs(motionDir.x) > Math.abs(motionDir.z)) {
        this.lastAutoCameraDirection = motionDir.x >= 0 ? "right" : "left";
      } else {
        this.lastAutoCameraDirection =
          motionDir.z >= 0 ? "backward" : "forward";
      }
    }

    const axisDir = {
      forward: new THREE.Vector3(0, 0, -1),
      backward: new THREE.Vector3(0, 0, 1),
      left: new THREE.Vector3(-1, 0, 0),
      right: new THREE.Vector3(1, 0, 0),
    };

    const activeDirection =
      this.cameraViewMode === "auto"
        ? this.lastAutoCameraDirection
        : this.cameraViewMode;

    if (this.cameraViewMode === "auto" && velocityFlat.length() <= 0.08) {
      motionDir = axisDir[activeDirection].clone();
    }

    const viewDirection =
      this.cameraViewMode === "auto"
        ? motionDir.clone().normalize()
        : axisDir[activeDirection].clone();

    const upOffset = new THREE.Vector3(0, 0.42, 0);
    const anchor = this.drone.group.position.clone().add(upOffset);
    const camPos = anchor.clone().addScaledVector(viewDirection, -0.45);
    const lookAt = anchor.clone().addScaledVector(viewDirection, 8);

    this.sideCamera.position.copy(camPos);
    this.sideCamera.lookAt(lookAt);

    if (this.ui.sideCameraModeLabel && this.cameraViewMode === "auto") {
      this.ui.sideCameraModeLabel.textContent = `AUTO • ${activeDirection.toUpperCase()}`;
    }

    this.sideRenderer.render(this.scene, this.sideCamera);
  }

  _setupAudio() {
    this.audio.context = null;
  }

  _unlockAudio() {
    if (this.audio.unlocked) {
      if (this.audio.context?.state === "suspended") {
        this.audio.context.resume();
      }
      return;
    }

    const AudioContextRef = window.AudioContext || window.webkitAudioContext;
    if (!AudioContextRef) return;

    this.audio.context = new AudioContextRef();
    const ctx = this.audio.context;
    const master = ctx.createGain();
    master.gain.value = 0.18;
    master.connect(ctx.destination);

    const osc = ctx.createOscillator();
    osc.type = "sawtooth";
    osc.frequency.value = 90;

    const engineGain = ctx.createGain();
    engineGain.gain.value = 0.0001;

    osc.connect(engineGain);
    engineGain.connect(master);
    osc.start();

    this.audio.master = master;
    this.audio.engineOsc = osc;
    this.audio.engineGain = engineGain;
    this.audio.unlocked = true;
    if (ctx.state === "suspended") {
      ctx.resume();
    }
  }

  _playUiBeep(freq = 640, duration = 0.08, gain = 0.016) {
    if (!this.audio.unlocked || !this.audio.context || !this.audio.master)
      return;
    const ctx = this.audio.context;
    const now = ctx.currentTime;

    const osc = ctx.createOscillator();
    osc.type = "triangle";
    osc.frequency.setValueAtTime(freq, now);

    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, now);
    g.gain.exponentialRampToValueAtTime(gain, now + 0.012);
    g.gain.exponentialRampToValueAtTime(0.0001, now + duration);

    osc.connect(g);
    g.connect(this.audio.master);
    osc.start(now);
    osc.stop(now + duration + 0.02);
  }

  _playCollisionThud() {
    if (!this.audio.unlocked || !this.audio.context || !this.audio.master)
      return;
    const ctx = this.audio.context;
    const now = ctx.currentTime;

    const osc = ctx.createOscillator();
    osc.type = "square";
    osc.frequency.setValueAtTime(160, now);
    osc.frequency.exponentialRampToValueAtTime(60, now + 0.08);

    const g = ctx.createGain();
    g.gain.setValueAtTime(0.0001, now);
    g.gain.exponentialRampToValueAtTime(0.028, now + 0.01);
    g.gain.exponentialRampToValueAtTime(0.0001, now + 0.12);

    osc.connect(g);
    g.connect(this.audio.master);
    osc.start(now);
    osc.stop(now + 0.14);
  }

  _updateAudio(speed, altitude, delta) {
    if (
      !this.audio.unlocked ||
      !this.audio.context ||
      !this.audio.engineOsc ||
      !this.audio.engineGain
    )
      return;

    this.audio.collisionCooldown = Math.max(
      0,
      this.audio.collisionCooldown - delta,
    );

    const targetGain =
      altitude > 0.05
        ? THREE.MathUtils.clamp(0.01 + speed * 0.005, 0.008, 0.09)
        : 0.002;
    const targetFreq = THREE.MathUtils.clamp(
      88 + speed * 16 + altitude * 2,
      85,
      260,
    );

    this.audio.engineGain.gain.value = THREE.MathUtils.lerp(
      this.audio.engineGain.gain.value,
      targetGain,
      0.08,
    );
    this.audio.engineOsc.frequency.value = THREE.MathUtils.lerp(
      this.audio.engineOsc.frequency.value,
      targetFreq,
      0.08,
    );
  }

  _postTelemetry() {
    if (!this.ui.telemetryEls) return;

    const now = performance.now();
    if (now - this._telemetryLastSent < 50) return; // ~20Hz
    this._telemetryLastSent = now;

    const alt = Math.max(0, this.drone.group.position.y - 0.55);
    const spd = this.drone.velocity.length();

    const pitchDeg = THREE.MathUtils.radToDeg(this.drone.group.rotation.x);
    const rollDeg = THREE.MathUtils.radToDeg(this.drone.group.rotation.z);
    const pitch = THREE.MathUtils.clamp(pitchDeg, -30, 30);
    const roll = THREE.MathUtils.clamp(rollDeg, -30, 30);

    const flying =
      alt > 0.05 && (spd > 0.05 || Math.abs(pitch) > 1 || Math.abs(roll) > 1);

    const t = this.hudState;
    const lerp = THREE.MathUtils.lerp;
    t.alt = lerp(t.alt, alt, 0.1);
    t.spd = lerp(t.spd, spd, 0.12);
    t.pitch = lerp(t.pitch, pitch, 0.14);
    t.roll = lerp(t.roll, roll, 0.14);
    t.bat = lerp(t.bat, this.telemetry.battery, 0.14);
    t.gps = lerp(t.gps, this.telemetry.gps, 0.14);
    t.cpu = lerp(t.cpu, this.telemetry.cpu, 0.16);
    t.flying = flying;
    t.status = this.telemetry.status;

    const els = this.ui.telemetryEls;
    els.alt.textContent = `${Math.round(t.alt)}`;
    els.spd.textContent = `${Math.abs(t.spd).toFixed(1)}`;
    els.pitch.textContent = `${Math.round(t.pitch)}`;
    els.roll.textContent = `${Math.round(t.roll)}`;
    els.batVal.textContent = `${Math.round(t.bat)}%`;
    els.gpsVal.textContent = `${Math.round(t.gps)}%`;
    els.cpuVal.textContent = `${Math.round(t.cpu)}%`;
    els.bat.style.width = `${THREE.MathUtils.clamp(t.bat, 0, 100)}%`;
    els.gps.style.width = `${THREE.MathUtils.clamp(t.gps, 0, 100)}%`;
    els.cpu.style.width = `${THREE.MathUtils.clamp(t.cpu, 0, 100)}%`;

    this.ui.hud.classList.toggle("hud--flying", t.flying);
    this._setTelemetryStatus(t.flying ? "FLYING" : t.status || "IDLE");

    const bat = Math.round(t.bat);
    if (bat < 20) {
      els.bat.classList.add("telemetry-bar-battery--low");
      els.bat.classList.remove("telemetry-bar-battery--warn");
    } else if (bat < 50) {
      els.bat.classList.add("telemetry-bar-battery--warn");
      els.bat.classList.remove("telemetry-bar-battery--low");
    } else {
      els.bat.classList.remove(
        "telemetry-bar-battery--warn",
        "telemetry-bar-battery--low",
      );
    }

    const d = new Date();
    const pad = (n) => String(n).padStart(2, "0");
    els.clock.textContent = `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
  }

  _setupTelemetryMessaging() {
    // Telemetry commands are handled directly from the injected HUD buttons.
  }

  _setTelemetryStatus(status) {
    if (!this.ui.telemetryEls) return;
    const label = status || "IDLE";
    const dot = this.ui.telemetryEls.statusDot;
    this.ui.telemetryEls.statusText.textContent = label;
    dot.className = "telemetry-status-dot";
    if (label === "IDLE") dot.classList.add("telemetry-status-dot--red");
    else if (label === "LANDING")
      dot.classList.add("telemetry-status-dot--yellow");
  }

  _handleTelemetryAction(action) {
    switch (action) {
      case "up":
        this.commandMode = "manual";
        this.telemetryInput.set(0, 1, 0);
        this._playUiBeep(680, 0.08, 0.014);
        this._logMavlink("COMMAND_LONG", "THRUST UP");
        break;
      case "down":
        this.commandMode = "manual";
        this.telemetryInput.set(0, -1, 0);
        this._playUiBeep(430, 0.08, 0.014);
        this._logMavlink("COMMAND_LONG", "THRUST DOWN");
        break;
      case "land":
        this.commandMode = "land";
        this.telemetryInput.set(0, 0, 0);
        this._playUiBeep(300, 0.12, 0.02);
        this._logMavlink("COMMAND_LONG", "NAV_LAND");
        break;
      case "forward":
        this.commandMode = "manual";
        this.telemetryInput.set(0, 0, -1);
        this._logMavlink("COMMAND_LONG", "SET_FORWARD");
        break;
      case "backward":
        this.commandMode = "manual";
        this.telemetryInput.set(0, 0, 1);
        this._logMavlink("COMMAND_LONG", "SET_BACKWARD");
        break;
      case "left":
        this.commandMode = "manual";
        this.telemetryInput.set(-1, 0, 0);
        this._logMavlink("COMMAND_LONG", "SET_LEFT");
        break;
      case "right":
        this.commandMode = "manual";
        this.telemetryInput.set(1, 0, 0);
        this._logMavlink("COMMAND_LONG", "SET_RIGHT");
        break;
      case "hover":
        this.commandMode = "hover";
        this.telemetryInput.set(0, 0, 0);
        this._playUiBeep(620, 0.08, 0.015);
        this._logMavlink("COMMAND_LONG", "SET_MODE HOVER");
        break;
      default:
        break;
    }
  }

  _setScenario(name) {
    this.scenario = name;
    this.drone.setAvoidanceEnabled(name === "obstacle");

    const label = name.replace("-", " ").toUpperCase();
    if (this.ui.scenarioBadge) this.ui.scenarioBadge.textContent = label;
    if (this.ui.scenarioSelect && this.ui.scenarioSelect.value !== name) {
      this.ui.scenarioSelect.value = name;
    }

    this._logMavlink("SCENARIO", `${label} enabled`);
  }

  _togglePlanningMode() {
    this.planningMode = !this.planningMode;
    if (this.ui.planningBtn) {
      this.ui.planningBtn.classList.toggle(
        "sim-btn--active",
        this.planningMode,
      );
      this.ui.planningBtn.textContent = this.planningMode
        ? "Click Ground To Add"
        : "Add Waypoint (M)";
    }
    this._logMavlink(
      "MISSION",
      this.planningMode ? "Waypoint planning ON" : "Waypoint planning OFF",
    );
  }

  _startMission() {
    if (!this.waypoints.length) {
      this._logMavlink("MISSION", "No waypoints available");
      return;
    }
    this.missionActive = true;
    this.missionPaused = false;
    this.missionIndex = 0;
    this.planningMode = false;
    this._refreshMissionUi();
    this._logMavlink(
      "MISSION",
      `Auto mission started (${this.waypoints.length} WPs)`,
    );
  }

  _togglePauseMission() {
    if (!this.missionActive) return;
    this.missionPaused = !this.missionPaused;
    this._refreshMissionUi();
    this._logMavlink(
      "MISSION",
      this.missionPaused ? "Mission paused" : "Mission resumed",
    );
  }

  _clearMission() {
    this.waypoints = [];
    this.missionIndex = 0;
    this.missionActive = false;
    this.missionPaused = false;
    this.waypointMeshes.forEach((mesh) => this.scene.remove(mesh));
    this.waypointMeshes = [];
    if (this.missionLine) {
      this.scene.remove(this.missionLine);
      this.missionLine.geometry.dispose();
      this.missionLine.material.dispose();
      this.missionLine = null;
    }
    this._refreshMissionUi();
    this._logMavlink("MISSION", "Mission cleared");
  }

  _refreshMissionUi() {
    if (!this.ui.missionStatus) return;

    let status = "IDLE";
    if (this.missionActive && this.missionPaused) status = "PAUSED";
    else if (this.missionActive)
      status = `AUTO ${this.missionIndex + 1}/${Math.max(this.waypoints.length, 1)}`;
    else if (this.planningMode) status = "PLANNING";

    this.ui.missionStatus.textContent = status;
    if (this.ui.pauseMissionBtn) {
      this.ui.pauseMissionBtn.textContent = this.missionPaused
        ? "Resume"
        : "Pause";
    }
    if (this.ui.planningBtn) {
      this.ui.planningBtn.classList.toggle(
        "sim-btn--active",
        this.planningMode,
      );
      this.ui.planningBtn.textContent = this.planningMode
        ? "Click Ground To Add"
        : "Add Waypoint (M)";
    }
  }

  _handleMissionPointing(event) {
    if (!this.planningMode) return;

    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    this.raycaster.setFromCamera(this.pointer, this.camera);
    const hit = new THREE.Vector3();
    const intersects = this.raycaster.ray.intersectPlane(this.groundPlane, hit);
    if (!intersects) return;

    const waypoint = new THREE.Vector3(hit.x, 2.0, hit.z);
    this.waypoints.push(waypoint);

    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 14, 14),
      new THREE.MeshStandardMaterial({
        color: 0x29f0ff,
        emissive: 0x0a4d55,
        emissiveIntensity: 0.65,
      }),
    );
    marker.position.copy(waypoint);
    marker.castShadow = true;
    marker.receiveShadow = true;
    this.scene.add(marker);
    this.waypointMeshes.push(marker);

    this._updateMissionLine();
    this._refreshMissionUi();
    this._logMavlink(
      "MISSION",
      `Waypoint ${this.waypoints.length} @ ${waypoint.x.toFixed(1)},${waypoint.z.toFixed(1)}`,
    );
  }

  _updateMissionLine() {
    if (this.missionLine) {
      this.scene.remove(this.missionLine);
      this.missionLine.geometry.dispose();
      this.missionLine.material.dispose();
      this.missionLine = null;
    }

    if (this.waypoints.length < 2) return;

    const points = this.waypoints.map((w) => w.clone());
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({
      color: 0x33d9ff,
      linewidth: 2,
    });
    this.missionLine = new THREE.Line(geometry, material);
    this.scene.add(this.missionLine);
  }

  _applyMissionAutopilot(input) {
    if (!this.missionActive || this.missionPaused || !this.waypoints.length)
      return;

    const target = this.waypoints[this.missionIndex];
    if (!target) return;

    const toTarget = target.clone().sub(this.drone.group.position);
    const distance = toTarget.length();

    if (distance < 0.75) {
      this.missionIndex += 1;
      if (this.missionIndex >= this.waypoints.length) {
        this.missionActive = false;
        this.missionPaused = false;
        this._refreshMissionUi();
        this._logMavlink("MISSION", "Mission completed");
        return;
      }
    }

    const nextTarget = this.waypoints[this.missionIndex];
    const direction = nextTarget.clone().sub(this.drone.group.position);
    if (direction.lengthSq() > 0.0001) {
      direction.normalize().multiplyScalar(this.missionSpeed);
      input.copy(direction);
      this.commandMode = "manual";
      this.telemetryInput.set(0, 0, 0);
    }
    this._refreshMissionUi();
  }

  _nearestObstacleDistance() {
    if (!this.obstacles.length) return Infinity;
    let min = Infinity;
    this.obstacles.forEach((obstacle) => {
      const d = this.drone.group.position.distanceTo(obstacle.position);
      if (d < min) min = d;
    });
    return min;
  }

  _applyScenarioEffects(delta, input, altitude) {
    const effects = {
      vslamActive: false,
      obstacleDistance: this._nearestObstacleDistance(),
      windStrength: 0,
      motorFailureActive: false,
    };

    if (this.scenario === "wind") {
      this.windVector.set(
        Math.sin(this.elapsed * 0.8) * 0.3 + (Math.random() - 0.5) * 0.04,
        0,
        Math.cos(this.elapsed * 0.55) * 0.22 + (Math.random() - 0.5) * 0.04,
      );
      effects.windStrength = this.windVector.length();
      this.drone.velocity.addScaledVector(this.windVector, delta * 2.2);
    }

    if (this.scenario === "gps-denied") {
      effects.vslamActive = true;
      this.drone.velocity.x += (Math.random() - 0.5) * 0.18 * delta;
      this.drone.velocity.z += (Math.random() - 0.5) * 0.18 * delta;
      if (Math.random() < 0.45) {
        this._spawnVslamPoint();
      }
    }

    if (this.scenario === "motor-failure" && altitude > 0.12) {
      effects.motorFailureActive = true;
      input.x += 0.35;
      input.y *= 0.78;
      this.drone.velocity.x += 0.55 * delta;
    }

    if (this.scenario === "obstacle") {
      this.drone.setAvoidanceEnabled(true);
    } else {
      this.drone.setAvoidanceEnabled(false);
    }

    return effects;
  }

  _updateJetsonVisuals(delta, effects) {
    const showDetection =
      this.scenario === "obstacle" && effects.obstacleDistance < 4.8;
    this.obstacleHelpers.forEach((helper, index) => {
      if (showDetection) {
        helper.visible = true;
        helper.update();
        const dist = this.drone.group.position.distanceTo(
          this.obstacles[index].position,
        );
        helper.material.color.set(dist < 3 ? 0xff6444 : 0x00ffd5);
      } else {
        helper.visible = false;
      }
    });

    for (let i = this.vslamPoints.length - 1; i >= 0; i -= 1) {
      const point = this.vslamPoints[i];
      point.life -= delta;
      point.mesh.material.opacity = Math.max(0, point.life);
      if (point.life <= 0) {
        this.scene.remove(point.mesh);
        point.mesh.geometry.dispose();
        point.mesh.material.dispose();
        this.vslamPoints.splice(i, 1);
      }
    }

    if (!effects.vslamActive && this.vslamPoints.length > 60) {
      const overflow = this.vslamPoints.splice(0, this.vslamPoints.length - 60);
      overflow.forEach((entry) => {
        this.scene.remove(entry.mesh);
        entry.mesh.geometry.dispose();
        entry.mesh.material.dispose();
      });
    }
  }

  _spawnVslamPoint() {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.05, 8, 8),
      new THREE.MeshBasicMaterial({
        color: 0x57f0ff,
        transparent: true,
        opacity: 0.8,
      }),
    );
    mesh.position
      .copy(this.drone.group.position)
      .add(
        new THREE.Vector3(
          (Math.random() - 0.5) * 1.4,
          (Math.random() - 0.5) * 0.8,
          (Math.random() - 0.5) * 1.4,
        ),
      );
    this.scene.add(mesh);
    this.vslamPoints.push({ mesh, life: 1.0 });
  }

  _updateTelemetryState(delta, altitude, speed, effects) {
    const inFlight = speed > 0.06 || altitude > 0.08;
    const profile = this.currentDroneProfile || DRONE_PROFILES[DEFAULT_PROFILE_KEY];

    if (inFlight) {
      let drain = 0.22 + Math.max(0, speed - 0.2) * 0.07 + altitude * 0.012;
      if (this.scenario === "wind") drain += effects.windStrength * 0.05;
      if (effects.vslamActive) drain += 0.06;
      if (effects.motorFailureActive) drain += 0.03;

      const batteryScale = THREE.MathUtils.clamp(5200 / profile.batteryMah, 0.25, 2.2);
      const massScale = THREE.MathUtils.clamp(profile.weightKg / 1.0, 0.6, 2.5);
      const payloadScale = THREE.MathUtils.clamp(1 + profile.payloadKg * 0.18, 1, 2.1);
      const speedEnvelope = THREE.MathUtils.clamp(speed / Math.max(3, profile.maxSpeed), 0.35, 1.35);
      const thrustScale = THREE.MathUtils.clamp(0.78 + (profile.thrustFactor || 1.0) * 0.35, 0.7, 1.45);
      drain *= batteryScale * massScale * payloadScale * speedEnvelope * thrustScale;

      this.telemetry.battery = Math.max(
        0,
        this.telemetry.battery - drain * delta,
      );
    }

    if (this.scenario === "gps-denied") {
      this.telemetry.gps = THREE.MathUtils.clamp(
        18 + Math.sin(this.elapsed * 1.6) * 5,
        8,
        30,
      );
    } else {
      this.telemetry.gps = THREE.MathUtils.clamp(
        100 - altitude * 0.25,
        70,
        100,
      );
    }

    this.telemetry.cpu = THREE.MathUtils.clamp(
      10 +
        speed * 6 +
        (profile.thrustFactor || 1.0) * 2.6 +
        (profile.maxSpeed > 12 ? 2 : 0) +
        profile.payloadKg * 2.2 +
        (this.missionActive ? 8 : 0) +
        (this.commandMode === "hover" ? 2 : 0) +
        (this.commandMode === "land" ? 4 : 0) +
        (effects.vslamActive ? 24 : 0) +
        (this.scenario === "obstacle" && effects.obstacleDistance < 5 ? 10 : 0),
      6,
      98,
    );

    if (!this.lowBatteryWarned && this.telemetry.battery <= 10) {
      this.lowBatteryWarned = true;
      this._showSystemNotice(
        "Battery low (10%). Return to base immediately.",
        "warn",
        4200,
      );
      this._logMavlink(
        "STATUSTEXT",
        "LOW_BATTERY 10%: return-to-home recommended",
      );
    }

    if (!this.batteryEmergencyActive && this.telemetry.battery <= 0) {
      this.batteryEmergencyActive = true;
      this.commandMode = "emergency-land";
      this.telemetryInput.set(0, 0, 0);
      this.missionActive = false;
      this.missionPaused = false;
      this._refreshMissionUi();

      this._showSystemNotice(
        "Battery depleted (0%). Emergency landing initiated.",
        "danger",
        5000,
      );
      this._logMavlink(
        "STATUSTEXT",
        "BATTERY_DEPLETED 0%: emergency landing (failsafe)",
      );
      this._logMavlink("COMMAND_LONG", "NAV_LAND reason=BATTERY_FAILSAFE");
    }

    if (this.batteryEmergencyActive) {
      this.telemetry.status = altitude > 0.05 ? "EMERG_LAND" : "BATT_EMPTY";
    } else if (effects.motorFailureActive) {
      this.telemetry.status = "MOTOR_FAIL";
    } else if (this.missionActive && !this.missionPaused) {
      this.telemetry.status = "AUTO";
    } else if (this.commandMode === "land") {
      this.telemetry.status = altitude > 0.05 ? "LANDING" : "IDLE";
    } else if (this.commandMode === "hover" && altitude > 0.05) {
      this.telemetry.status = "HOVER";
    } else if (effects.vslamActive && altitude > 0.05) {
      this.telemetry.status = "VSLAM";
    } else if (inFlight) {
      this.telemetry.status = "FLYING";
    } else {
      this.telemetry.status = "IDLE";
    }
  }

  _emitMavlink(delta, effects, altitude, speed) {
    this.mavlinkTick.heartbeat += delta;
    this.mavlinkTick.attitude += delta;
    this.mavlinkTick.gps += delta;
    this.mavlinkTick.battery += delta;
    this.mavlinkTick.vision += delta;
    this.mavlinkTick.obstacle += delta;

    if (this.mavlinkTick.heartbeat >= 1.0) {
      this.mavlinkTick.heartbeat = 0;
      this._logMavlink(
        "HEARTBEAT",
        `mode=${this.telemetry.status} arm=${altitude > 0.08 ? 1 : 0}`,
      );
    }

    if (this.mavlinkTick.attitude >= 0.18) {
      this.mavlinkTick.attitude = 0;
      this._logMavlink(
        "ATTITUDE",
        `roll=${this.drone.group.rotation.z.toFixed(2)} pitch=${this.drone.group.rotation.x.toFixed(2)}`,
      );
    }

    if (this.mavlinkTick.gps >= 0.5) {
      this.mavlinkTick.gps = 0;
      this._logMavlink(
        "GPS_RAW_INT",
        `fix=${this.telemetry.gps > 40 ? 3 : 1} sats=${Math.round(3 + this.telemetry.gps / 10)} alt=${Math.round(altitude * 100)}`,
      );
    }

    if (this.mavlinkTick.battery >= 1.1) {
      this.mavlinkTick.battery = 0;
      this._logMavlink(
        "BATTERY_STATUS",
        `remaining=${Math.round(this.telemetry.battery)}% current=${(3 + speed * 0.8).toFixed(1)}A`,
      );
    }

    if (effects.vslamActive && this.mavlinkTick.vision >= 0.35) {
      this.mavlinkTick.vision = 0;
      this._logMavlink(
        "VISION_POSITION_ESTIMATE",
        `x=${this.drone.group.position.x.toFixed(2)} y=${this.drone.group.position.y.toFixed(2)} z=${this.drone.group.position.z.toFixed(2)}`,
      );
    }

    if (
      this.scenario === "obstacle" &&
      effects.obstacleDistance < 6 &&
      this.mavlinkTick.obstacle >= 0.25
    ) {
      this.mavlinkTick.obstacle = 0;
      this._logMavlink(
        "OBSTACLE_DISTANCE",
        `min=${Math.max(0.1, effects.obstacleDistance - 1).toFixed(2)}m`,
      );
    }
  }

  _logMavlink(type, payload) {
    const timestamp = new Date().toLocaleTimeString("en-GB", {
      hour12: false,
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
    this.mavlinkLogs.push(`[${timestamp}] ${type}  ${payload}`);
    if (this.mavlinkLogs.length > this.maxLogs) {
      this.mavlinkLogs.splice(0, this.mavlinkLogs.length - this.maxLogs);
    }

    if (this.ui.log) {
      this.ui.log.textContent = this.mavlinkLogs.join("\n");
      this.ui.log.scrollTop = this.ui.log.scrollHeight;
    }
  }

  _updateFollowCamera(delta) {
    const desiredPosition = this.drone.group.position
      .clone()
      .add(this.followOffset);
    this.camera.position.lerp(desiredPosition, 1 - Math.exp(-4 * delta));
    this.controls.target.lerp(
      this.drone.group.position,
      1 - Math.exp(-6 * delta),
    );
  }

  animate() {
    const delta = Math.min(this.clock.getDelta(), 0.05);
    this.elapsed += delta;

    const keyboardInput = this.keyboard.getInputVector();

    // Keyboard should always be able to take control back from LAND/HOVER modes.
    if (keyboardInput.lengthSq() > 0 && !this.batteryEmergencyActive) {
      this.commandMode = "manual";
      this.telemetryInput.set(0, 0, 0);
      if (this.missionActive) {
        this.missionActive = false;
        this.missionPaused = false;
        this._refreshMissionUi();
        this._logMavlink("MISSION", "Mission interrupted by manual input");
      }
    }

    let input = keyboardInput.add(this.telemetryInput);
    if (input.lengthSq() > 1) input.normalize();

    const altitude = Math.max(0, this.drone.group.position.y - 0.55);

    if (this.commandMode === "hover") {
      input.set(0, 0, 0);
      this.drone.velocity.multiplyScalar(Math.pow(0.9, delta * 60));
    } else if (this.commandMode === "emergency-land") {
      input.set(0, -2.2, 0);
      this.drone.velocity.x *= Math.pow(0.72, delta * 60);
      this.drone.velocity.z *= Math.pow(0.72, delta * 60);
      this.drone.velocity.y = Math.min(this.drone.velocity.y, -7.0);

      if (altitude <= 0.03) {
        this.commandMode = "hover";
        this.telemetryInput.set(0, 0, 0);
        if (!this.batteryEmergencyLanded) {
          this.batteryEmergencyLanded = true;
          this._showSystemNotice(
            "Emergency landing complete. Battery exhausted.",
            "danger",
            4500,
          );
          this._logMavlink(
            "STATUSTEXT",
            "LAND_COMPLETE reason=BATTERY_FAILSAFE",
          );
        }
      }
    } else if (this.commandMode === "land") {
      input.set(0, -0.65, 0);
      this.drone.velocity.x *= Math.pow(0.85, delta * 60);
      this.drone.velocity.z *= Math.pow(0.85, delta * 60);
      if (altitude <= 0.03) {
        this.commandMode = "hover";
        this.telemetryInput.set(0, 0, 0);
      }
    }

    this._applyMissionAutopilot(input);

    const effects = this._applyScenarioEffects(delta, input, altitude);

    this.drone.update(delta, input);

    const spd = this.drone.velocity.length();

    this._updateAudio(spd, altitude, delta);
    if (
      this.drone.collisionsThisFrame > 0 &&
      this.audio.collisionCooldown <= 0
    ) {
      this._playCollisionThud();
      this.audio.collisionCooldown = 0.18;
      this._logMavlink("OBSTACLE", "Collision prevented (solid obstacle)");
    }

    this._updateTelemetryState(delta, altitude, spd, effects);
    this._updateJetsonVisuals(delta, effects);
    this._emitMavlink(delta, effects, altitude, spd);
    this._refreshMissionUi();

    if (this.followMode && !this.viewportLocked) {
      this._updateFollowCamera(delta);
    }

    if (!this.viewportLocked) {
      this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
    this._updateSideCamera();
    this._postTelemetry();
    requestAnimationFrame(this.animate);
  }
}

const root = document.getElementById("app");
new DroneSimulatorApp(root);
