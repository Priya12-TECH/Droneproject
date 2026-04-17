import * as THREE from "three";
import { Drone } from "./components/Drone.js";
import {
  createOrbitControls,
  KeyboardFlightController,
} from "./utils/controls.js";
import "./styles.css";

class DroneSimulatorApp {
  constructor(container) {
    this.container = container;
    this.clock = new THREE.Clock();
    this.elapsed = 0;
    this.followMode = false;
    this.telemetryIframe = null;
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

    this.ui = {
      panel: null,
      hud: null,
      hudToggle: null,
      panelToggle: null,
      log: null,
      sideCameraCanvas: null,
      scenarioSelect: null,
      missionStatus: null,
      missionSpeed: null,
      missionSpeedValue: null,
      planningBtn: null,
      startMissionBtn: null,
      pauseMissionBtn: null,
      clearMissionBtn: null,
      scenarioBadge: null,
      introOverlay: null,
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
    this.drone.group.position.set(0, 2, 0);
    this.scene.add(this.drone.group);

    this.followOffset = new THREE.Vector3(8, 5, 9);

    this._setupScene();
    this._setupEvents();
    this._setupHud();
    this._setupControlPanel();
    this._setupIntroOverlay();
    this._setupAudio();
    this._setupTelemetryMessaging();
    this._setScenario("normal");
    this._logMavlink("SYSTEM", "DroneSimX ready. Select scenario and fly.");

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  _setupScene() {
    const ambient = new THREE.AmbientLight(0xffffff, 0.32);
    this.scene.add(ambient);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
    dirLight.position.set(12, 20, 8);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.set(2048, 2048);
    dirLight.shadow.camera.left = -35;
    dirLight.shadow.camera.right = 35;
    dirLight.shadow.camera.top = 35;
    dirLight.shadow.camera.bottom = -35;
    dirLight.shadow.camera.near = 1;
    dirLight.shadow.camera.far = 120;
    this.scene.add(dirLight);

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(220, 220),
      new THREE.MeshStandardMaterial({
        color: 0x101a2d,
        metalness: 0.02,
        roughness: 0.95,
      }),
    );
    ground.rotation.x = -Math.PI * 0.5;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(220, 70, 0x3d6aa2, 0x243755);
    grid.position.y = 0.01;
    this.scene.add(grid);

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

  _setupHud() {
    const hud = document.createElement("div");
    hud.className = "hud hud--telemetry";
    hud.innerHTML = `
      <iframe
        class="telemetry-iframe"
        title="Drone Telemetry HUD"
        src="/drone_telemetry.html"
      ></iframe>
    `.trim();
    this.telemetryIframe = hud.querySelector("iframe");
    document.body.appendChild(hud);
    this.ui.hud = hud;

    const toggle = document.createElement("button");
    toggle.className = "panel-toggle panel-toggle--left panel-toggle--active";
    toggle.type = "button";
    toggle.textContent = "Telemetry";
    toggle.addEventListener("click", () => {
      this.leftPanelOpen = !this.leftPanelOpen;
      hud.classList.toggle("hud--collapsed", !this.leftPanelOpen);
      toggle.classList.toggle("panel-toggle--active", this.leftPanelOpen);
    });
    document.body.appendChild(toggle);
    this.ui.hudToggle = toggle;
  }

  _setupControlPanel() {
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
          <span class="sim-label">Camera View</span>
          <span class="sim-chip">FPV</span>
        </div>
        <canvas id="side-camera" class="side-camera"></canvas>
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
    this.ui.sideCameraCanvas = panel.querySelector("#side-camera");
    this.ui.scenarioSelect = panel.querySelector("#scenario-select");
    this.ui.missionStatus = panel.querySelector("#mission-status");
    this.ui.missionSpeed = panel.querySelector("#mission-speed");
    this.ui.missionSpeedValue = panel.querySelector("#mission-speed-value");
    this.ui.planningBtn = panel.querySelector("#btn-plan");
    this.ui.startMissionBtn = panel.querySelector("#btn-start-mission");
    this.ui.pauseMissionBtn = panel.querySelector("#btn-pause-mission");
    this.ui.clearMissionBtn = panel.querySelector("#btn-clear-mission");
    this.ui.scenarioBadge = panel.querySelector("#scenario-badge");

    this.ui.scenarioSelect.addEventListener("change", (event) => {
      this._setScenario(event.target.value);
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

    const toggle = document.createElement("button");
    toggle.className = "panel-toggle panel-toggle--right panel-toggle--active";
    toggle.type = "button";
    toggle.textContent = "Control Panel";
    toggle.addEventListener("click", () => {
      this.rightPanelOpen = !this.rightPanelOpen;
      panel.classList.toggle("sim-panel--collapsed", !this.rightPanelOpen);
      toggle.classList.toggle("panel-toggle--active", this.rightPanelOpen);
    });
    document.body.appendChild(toggle);
    this.ui.panelToggle = toggle;

    this._setupSideCamera();
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

  _setupSideCamera() {
    if (!this.ui.sideCameraCanvas) return;

    this.sideCamera = new THREE.PerspectiveCamera(75, 1.7, 0.1, 220);
    this.sideRenderer = new THREE.WebGLRenderer({
      canvas: this.ui.sideCameraCanvas,
      antialias: true,
      alpha: true,
    });
    this.sideRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.sideRenderer.setSize(330, 190, false);
  }

  _updateSideCamera() {
    if (!this.sideCamera || !this.sideRenderer) return;

    const speed = this.drone.velocity.length();
    const forward = this.drone.velocity.clone();
    if (speed < 0.05) {
      forward.set(0, 0, -1);
    } else {
      forward.normalize();
    }

    const camPos = this.drone.group.position
      .clone()
      .addScaledVector(forward, -0.45)
      .add(new THREE.Vector3(0, 0.4, 0));
    const lookAt = this.drone.group.position
      .clone()
      .addScaledVector(forward, 8)
      .add(new THREE.Vector3(0, 0.15, 0));

    this.sideCamera.position.copy(camPos);
    this.sideCamera.lookAt(lookAt);
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
    if (!this.telemetryIframe || !this.telemetryIframe.contentWindow) return;

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

    this.telemetryIframe.contentWindow.postMessage(
      {
        type: "telemetry",
        alt,
        spd,
        pitch,
        roll,
        flying,
        battery: this.telemetry.battery,
        gps: this.telemetry.gps,
        cpu: this.telemetry.cpu,
        status: this.telemetry.status,
      },
      "*",
    );
  }

  _setupTelemetryMessaging() {
    window.addEventListener("message", (event) => {
      const data = event.data;
      if (!data || data.type !== "telemetry-cmd") return;

      // Telemetry HUD buttons are discrete clicks; keep the last command "active"
      // until user clicks another command (e.g. Hover or Land).
      switch (data.action) {
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
    });
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

    if (inFlight) {
      let drain = 0.22 + Math.max(0, speed - 0.2) * 0.07 + altitude * 0.012;
      if (this.scenario === "wind") drain += effects.windStrength * 0.05;
      if (effects.vslamActive) drain += 0.06;
      if (effects.motorFailureActive) drain += 0.03;
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
        (this.missionActive ? 8 : 0) +
        (this.commandMode === "hover" ? 2 : 0) +
        (this.commandMode === "land" ? 4 : 0) +
        (effects.vslamActive ? 24 : 0) +
        (this.scenario === "obstacle" && effects.obstacleDistance < 5 ? 10 : 0),
      6,
      98,
    );

    if (effects.motorFailureActive) {
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
    if (keyboardInput.lengthSq() > 0) {
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

    if (this.followMode) {
      this._updateFollowCamera(delta);
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this._updateSideCamera();
    this._postTelemetry();
    requestAnimationFrame(this.animate);
  }
}

const root = document.getElementById("app");
new DroneSimulatorApp(root);
