import * as THREE from "three";

// ═══════════════════════════════════════════════════════════════
//  DRONE SIMULATOR — World Environment System
//  Drop this file into your project and call initWorldEnvironment(scene, renderer)
//  Then use switchEnvironment('day') / switchEnvironment('night') etc.
// ═══════════════════════════════════════════════════════════════

const ENVIRONMENTS = {

  // ── 1. Bright Day ─────────────────────────────────────────────
  day: {
    name: 'Daytime',
    fogColor: 0xc9e8ff,
    fogNear: 80,
    fogFar: 400,
    bgColor: 0x87ceeb,           // sky blue
    ambientColor: 0xffffff,
    ambientIntensity: 0.7,
    sunColor: 0xfff5e0,
    sunIntensity: 1.4,
    sunPosition: [100, 200, 100],
    groundColor: 0x4a7c3f,       // grass green
    gridColor: 0x3a6b30,
    gridCenterColor: 0x2d5426,
    hemiSkyColor: 0x87ceeb,
    hemiGroundColor: 0x4a7c3f,
    hemiIntensity: 0.5,
  },

  // ── 2. Golden Hour ────────────────────────────────────────────
  sunset: {
    name: 'Sunset',
    fogColor: 0xff9966,
    fogNear: 60,
    fogFar: 300,
    bgColor: 0xff7733,
    ambientColor: 0xff9955,
    ambientIntensity: 0.6,
    sunColor: 0xff6600,
    sunIntensity: 1.2,
    sunPosition: [200, 40, -100],
    groundColor: 0x8b5e3c,
    gridColor: 0x7a4e2d,
    gridCenterColor: 0x5c3a20,
    hemiSkyColor: 0xff7733,
    hemiGroundColor: 0x8b5e3c,
    hemiIntensity: 0.4,
  },

  // ── 3. Clean White Studio ─────────────────────────────────────
  studio: {
    name: 'Studio',
    fogColor: 0xffffff,
    fogNear: 100,
    fogFar: 500,
    bgColor: 0xf0f4f8,
    ambientColor: 0xffffff,
    ambientIntensity: 1.0,
    sunColor: 0xffffff,
    sunIntensity: 0.8,
    sunPosition: [50, 200, 50],
    groundColor: 0xe8ecf0,
    gridColor: 0xc0c8d0,
    gridCenterColor: 0x9aa5b0,
    hemiSkyColor: 0xffffff,
    hemiGroundColor: 0xdddddd,
    hemiIntensity: 0.6,
  },

  // ── 4. Desert / Arid ─────────────────────────────────────────
  desert: {
    name: 'Desert',
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

  // ── 5. Snowy Arctic ───────────────────────────────────────────
  snow: {
    name: 'Arctic',
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

  // ── 6. Night / City ───────────────────────────────────────────
  night: {
    name: 'Night',
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
let _currentEnv = 'day';

// ═══════════════════════════════════════════════════════════════
//  INIT — call once after you create your scene
//  Pass in your Three.js scene and renderer
// ═══════════════════════════════════════════════════════════════
function initWorldEnvironment(scene, renderer) {
  _scene    = scene;
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
  _sunLight.shadow.mapSize.width  = 2048;
  _sunLight.shadow.mapSize.height = 2048;
  _sunLight.shadow.camera.near   = 0.5;
  _sunLight.shadow.camera.far    = 600;
  _sunLight.shadow.camera.left   = -150;
  _sunLight.shadow.camera.right  =  150;
  _sunLight.shadow.camera.top    =  150;
  _sunLight.shadow.camera.bottom = -150;
  _sunLight.shadow.bias = -0.0005;
  scene.add(_sunLight);

  // Make existing obstacles cast + receive shadows
  scene.traverse(obj => {
    if (obj.isMesh && obj !== _ground) {
      obj.castShadow    = true;
      obj.receiveShadow = true;
    }
  });

  // Apply default environment
  applyEnvironment('day');

  // Build UI switcher
  buildEnvUI();
}

// ═══════════════════════════════════════════════════════════════
//  SWITCH ENVIRONMENT — call this any time
//  e.g. switchEnvironment('sunset')
// ═══════════════════════════════════════════════════════════════
function switchEnvironment(envKey) {
  if (!ENVIRONMENTS[envKey]) return console.warn('Unknown env:', envKey);
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
    _renderer.toneMappingExposure = envKey === 'night' ? 2.0 : 1.0;
  }
}

// ═══════════════════════════════════════════════════════════════
//  UI — floating environment switcher (top-right corner)
// ═══════════════════════════════════════════════════════════════
function buildEnvUI() {
  // Remove existing if any
  const old = document.getElementById('env-switcher');
  if (old) old.remove();

  const panel = document.createElement('div');
  panel.id = 'env-switcher';
  panel.style.cssText = `
    position: fixed;
    top: 16px;
    right: 16px;
    z-index: 1000;
    background: rgba(5, 10, 20, 0.85);
    border: 1px solid rgba(56, 189, 248, 0.25);
    border-radius: 10px;
    padding: 10px 12px;
    font-family: 'Segoe UI', sans-serif;
    backdrop-filter: blur(8px);
  `;

  const label = document.createElement('div');
  label.style.cssText = 'font-size:10px; color:#4b9fd4; letter-spacing:0.1em; text-transform:uppercase; margin-bottom:8px; font-weight:600;';
  label.textContent = 'Environment';
  panel.appendChild(label);

  const btnRow = document.createElement('div');
  btnRow.style.cssText = 'display:flex; flex-wrap:wrap; gap:5px; max-width:220px;';

  Object.entries(ENVIRONMENTS).forEach(([key, env]) => {
    const btn = document.createElement('button');
    btn.id = 'env-btn-' + key;
    btn.textContent = env.name;
    btn.onclick = () => switchEnvironment(key);
    btn.style.cssText = `
      font-size: 11px;
      padding: 5px 10px;
      border-radius: 5px;
      border: 1px solid rgba(56,189,248,0.2);
      background: rgba(255,255,255,0.05);
      color: #a0c4d8;
      cursor: pointer;
      transition: all 0.15s;
      font-family: inherit;
    `;
    btn.onmouseenter = () => {
      if (btn.id !== 'env-btn-' + _currentEnv)
        btn.style.background = 'rgba(56,189,248,0.1)';
    };
    btn.onmouseleave = () => {
      if (btn.id !== 'env-btn-' + _currentEnv)
        btn.style.background = 'rgba(255,255,255,0.05)';
    };
    btnRow.appendChild(btn);
  });

  panel.appendChild(btnRow);
  document.body.appendChild(panel);
  updateEnvUI('day');
}

function updateEnvUI(activeKey) {
  Object.keys(ENVIRONMENTS).forEach(key => {
    const btn = document.getElementById('env-btn-' + key);
    if (!btn) return;
    if (key === activeKey) {
      btn.style.background = 'rgba(56,189,248,0.25)';
      btn.style.color = '#38bdf8';
      btn.style.borderColor = 'rgba(56,189,248,0.6)';
    } else {
      btn.style.background = 'rgba(255,255,255,0.05)';
      btn.style.color = '#a0c4d8';
      btn.style.borderColor = 'rgba(56,189,248,0.2)';
    }
  });
}

// ═══════════════════════════════════════════════════════════════
//  EXPORT (if using modules)
// ═══════════════════════════════════════════════════════════════
// ES module export (used by src/main.js)
export { initWorldEnvironment, switchEnvironment, ENVIRONMENTS };
