import * as THREE from "three";

// ═══════════════════════════════════════════════════════════════
//  DRONE SIMULATOR — World Environment System
//  Drop this file into your project and call initWorldEnvironment(scene, renderer)
//  Then use switchEnvironment('day') / switchEnvironment('night') etc.
// ═══════════════════════════════════════════════════════════════

const ENVIRONMENTS = {
  // ── 1. Bright Day ─────────────────────────────────────────────
  day: {
    name: "Day",
    fogColor: 0xc9e8ff,
    fogNear: 80,
    fogFar: 400,
    bgColor: 0x87ceeb, // sky blue
    ambientColor: 0xffffff,
    ambientIntensity: 0.7,
    sunColor: 0xfff5e0,
    sunIntensity: 1.4,
    sunPosition: [100, 200, 100],
    groundColor: 0x4a7c3f, // grass green
    gridColor: 0x3a6b30,
    gridCenterColor: 0x2d5426,
    hemiSkyColor: 0x87ceeb,
    hemiGroundColor: 0x4a7c3f,
    hemiIntensity: 0.5,
  },

  // ── 2. Desert / Arid ─────────────────────────────────────────
  desert: {
    name: "Desert",
    fogColor: 0xe8c88a,
    fogNear: 70,
    fogFar: 350,
    bgColor: 0xd4a855,
    ambientColor: 0xffe0a0,
    ambientIntensity: 0.8,
    sunColor: 0xffd060,
    sunIntensity: 1.6,
    sunPosition: [80, 180, 60],
    groundColor: 0xc2944a,
    gridColor: 0xb08040,
    gridCenterColor: 0x8a6230,
    hemiSkyColor: 0xd4a855,
    hemiGroundColor: 0xc2944a,
    hemiIntensity: 0.5,
  },

  // ── 3. Snowy Arctic ───────────────────────────────────────────
  snow: {
    name: "Arctic",
    fogColor: 0xddeeff,
    fogNear: 50,
    fogFar: 250,
    bgColor: 0xcce4ff,
    ambientColor: 0xddeeff,
    ambientIntensity: 0.9,
    sunColor: 0xffffff,
    sunIntensity: 0.7,
    sunPosition: [60, 80, 80],
    groundColor: 0xeef4ff,
    gridColor: 0xc8d8f0,
    gridCenterColor: 0xa0b8e0,
    hemiSkyColor: 0xcce4ff,
    hemiGroundColor: 0xeef4ff,
    hemiIntensity: 0.7,
  },

  // ── 4. Night / City ───────────────────────────────────────────
  night: {
    name: "Night",
    fogColor: 0x050a14,
    fogNear: 40,
    fogFar: 200,
    bgColor: 0x050a14,
    ambientColor: 0x102040,
    ambientIntensity: 0.3,
    sunColor: 0x4488ff,
    sunIntensity: 0.2,
    sunPosition: [0, 100, 0],
    groundColor: 0x0a1020,
    gridColor: 0x0a2040,
    gridCenterColor: 0x0055aa,
    hemiSkyColor: 0x050a14,
    hemiGroundColor: 0x0a1020,
    hemiIntensity: 0.2,
  },
};

// ── Globals ──────────────────────────────────────────────────────
let _scene, _renderer, _ground, _grid, _sunLight, _ambientLight, _hemiLight;
let _currentEnv = "day";
let _envPanel = null;
let _envOpen = false;

// ═══════════════════════════════════════════════════════════════
//  INIT — call once after you create your scene
//  Pass in your Three.js scene and renderer
// ═══════════════════════════════════════════════════════════════
function initWorldEnvironment(scene, renderer) {
  _scene = scene;
  _renderer = renderer;

  // Enable shadows on renderer
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;

  // ── Ground plane ──────────────────────────────────────────────
  const groundGeo = new THREE.PlaneGeometry(600, 600);
  const groundMat = new THREE.MeshLambertMaterial({ color: 0x4a7c3f });
  _ground = new THREE.Mesh(groundGeo, groundMat);
  _ground.rotation.x = -Math.PI / 2;
  _ground.position.y = 0;
  _ground.receiveShadow = true;
  scene.add(_ground);

  // ── Grid on top of ground ─────────────────────────────────────
  _grid = new THREE.GridHelper(600, 60, 0x3a6b30, 0x3a6b30);
  _grid.position.y = 0.05;
  scene.add(_grid);

  // ── Ambient light ─────────────────────────────────────────────
  _ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(_ambientLight);

  // ── Hemisphere light (sky + ground bounce) ────────────────────
  _hemiLight = new THREE.HemisphereLight(0x87ceeb, 0x4a7c3f, 0.5);
  scene.add(_hemiLight);

  // ── Sun / directional light ───────────────────────────────────
  _sunLight = new THREE.DirectionalLight(0xfff5e0, 1.4);
  _sunLight.position.set(100, 200, 100);
  _sunLight.castShadow = true;
  _sunLight.shadow.mapSize.width = 2048;
  _sunLight.shadow.mapSize.height = 2048;
  _sunLight.shadow.camera.near = 0.5;
  _sunLight.shadow.camera.far = 600;
  _sunLight.shadow.camera.left = -150;
  _sunLight.shadow.camera.right = 150;
  _sunLight.shadow.camera.top = 150;
  _sunLight.shadow.camera.bottom = -150;
  _sunLight.shadow.bias = -0.0005;
  scene.add(_sunLight);

  // Make existing obstacles cast + receive shadows
  scene.traverse((obj) => {
    if (obj.isMesh && obj !== _ground) {
      obj.castShadow = true;
      obj.receiveShadow = true;
    }
  });

  // Apply default environment
  applyEnvironment("day");

  // Build UI switcher
  buildEnvUI();
}

// ═══════════════════════════════════════════════════════════════
//  SWITCH ENVIRONMENT — call this any time
//  e.g. switchEnvironment('sunset')
// ═══════════════════════════════════════════════════════════════
function switchEnvironment(envKey) {
  if (!ENVIRONMENTS[envKey]) return console.warn("Unknown env:", envKey);
  _currentEnv = envKey;
  applyEnvironment(envKey);
  updateEnvUI(envKey);
}

function applyEnvironment(envKey) {
  const e = ENVIRONMENTS[envKey];

  // Background + fog
  _scene.background = new THREE.Color(e.bgColor);
  _scene.fog = new THREE.Fog(e.fogColor, e.fogNear, e.fogFar);

  // Lights
  _ambientLight.color.setHex(e.ambientColor);
  _ambientLight.intensity = e.ambientIntensity;

  _sunLight.color.setHex(e.sunColor);
  _sunLight.intensity = e.sunIntensity;
  _sunLight.position.set(...e.sunPosition);

  _hemiLight.color.setHex(e.hemiSkyColor);
  _hemiLight.groundColor.setHex(e.hemiGroundColor);
  _hemiLight.intensity = e.hemiIntensity;

  // Ground
  _ground.material.color.setHex(e.groundColor);

  // Grid
  _grid.material.color = new THREE.Color(e.gridColor);

  // Renderer exposure tweak for night
  if (_renderer) {
    _renderer.toneMappingExposure = envKey === "night" ? 2.0 : 1.0;
  }
}

// ═══════════════════════════════════════════════════════════════
//  UI — floating environment switcher (top-right corner)
// ═══════════════════════════════════════════════════════════════
function buildEnvUI() {
  // Remove existing if any
  const old = document.getElementById("env-switcher");
  if (old) old.remove();

  const panel = document.createElement("div");
  panel.id = "env-switcher";
  panel.className = "sim-panel environment-panel environment-panel--collapsed";

  const header = document.createElement("div");
  header.className = "sim-panel__header";
  header.innerHTML = `
    <div>
      <h2>Environment</h2>
      <p>Visual world profile</p>
    </div>
    <span class="sim-panel__badge">ACTIVE</span>
  `;
  panel.appendChild(header);

  const section = document.createElement("section");
  section.className = "sim-section";

  const label = document.createElement("div");
  label.className = "sim-label";
  label.textContent = "Select Preset";
  section.appendChild(label);

  const btnRow = document.createElement("div");
  btnRow.className = "environment-grid";

  Object.entries(ENVIRONMENTS).forEach(([key, env]) => {
    const btn = document.createElement("button");
    btn.id = "env-btn-" + key;
    btn.textContent = env.name;
    btn.type = "button";
    btn.onclick = () => switchEnvironment(key);
    btn.className = "sim-btn environment-btn";
    btnRow.appendChild(btn);
  });

  section.appendChild(btnRow);
  panel.appendChild(section);

  document.body.appendChild(panel);
  _envPanel = panel;
  _envOpen = false;
  setEnvironmentPanelOpen(false);
  updateEnvUI("day");
}

function updateEnvUI(activeKey) {
  Object.keys(ENVIRONMENTS).forEach((key) => {
    const btn = document.getElementById("env-btn-" + key);
    if (!btn) return;
    if (key === activeKey) {
      btn.classList.add("environment-btn--active");
    } else {
      btn.classList.remove("environment-btn--active");
    }
  });
}

function setEnvironmentPanelOpen(open) {
  _envOpen = Boolean(open);
  if (_envPanel) {
    _envPanel.classList.toggle("environment-panel--collapsed", !_envOpen);
  }
  return _envOpen;
}

function toggleEnvironmentPanel() {
  return setEnvironmentPanelOpen(!_envOpen);
}

function isEnvironmentPanelOpen() {
  return _envOpen;
}

// ═══════════════════════════════════════════════════════════════
//  EXPORT (if using modules)
// ═══════════════════════════════════════════════════════════════
// ES module export (used by src/main.js)
export {
  initWorldEnvironment,
  switchEnvironment,
  ENVIRONMENTS,
  setEnvironmentPanelOpen,
  toggleEnvironmentPanel,
  isEnvironmentPanelOpen,
};
