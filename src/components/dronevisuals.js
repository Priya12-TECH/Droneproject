// drone-visual.js
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

/**
 * Lightweight, realistic quadcopter model for Three.js simulations.
 * - Procedural fallback model (performance-friendly)
 * - Optional GLTF/GLB model loading
 * - Propeller animation driven by throttle
 * - FPV camera mount object aligned to drone front
 */
export class DroneVisual {
  constructor({ scene = null, useGLTF = false, gltfUrl = null, baseScale = 1.0 } = {}) {

    this.scene = scene;
    this.group = new THREE.Group();
    this.group.name = 'DroneRoot';

    this.droneMesh = new THREE.Group();
    this.droneMesh.name = 'DroneMesh';
    this.group.add(this.droneMesh);

    this.fpvcamMount = new THREE.Object3D();
    this.fpvcamMount.name = 'FPVCameraMount';
    this.group.add(this.fpvcamMount);

    this.propellers = [];
    this.throttle = 0;
    this.propSpin = 0;

    this._tmpQuat = new THREE.Quaternion();
    this._tmpPos = new THREE.Vector3();
    this._forward = new THREE.Vector3(0, 0, -1);

    if (this.scene) this.scene.add(this.group);

    if (useGLTF && gltfUrl) {
      this._loadGLTF(gltfUrl, baseScale).catch(() => {
        this._buildProcedural(baseScale);
      });
    } else {
      this._buildProcedural(baseScale);
    }
  }

  _makeMaterialLibrary() {
    return {
      body: new THREE.MeshStandardMaterial({
        color: 0x2b2f36,
        roughness: 0.45,
        metalness: 0.4
      }),
      carbon: new THREE.MeshStandardMaterial({
        color: 0x121417,
        roughness: 0.65,
        metalness: 0.15
      }),
      accent: new THREE.MeshStandardMaterial({
        color: 0x2fd9ff,
        emissive: 0x0a2f38,
        roughness: 0.35,
        metalness: 0.65
      }),
      camera: new THREE.MeshStandardMaterial({
        color: 0x0f1114,
        roughness: 0.25,
        metalness: 0.5
      }),
      lens: new THREE.MeshPhysicalMaterial({
        color: 0x88d8ff,
        transmission: 0.15,
        roughness: 0.08,
        metalness: 0.2,
        clearcoat: 1
      }),
      prop: new THREE.MeshStandardMaterial({
        color: 0x15181c,
        roughness: 0.55,
        metalness: 0.05
      }),
      leg: new THREE.MeshStandardMaterial({
        color: 0x1f2329,
        roughness: 0.6,
        metalness: 0.2
      })
    };
  }

  _buildProcedural(baseScale) {
    this._disposeDroneMesh();

    const m = this._makeMaterialLibrary();
    const drone = new THREE.Group();

    const body = new THREE.Mesh(
      new THREE.CapsuleGeometry(0.22, 0.26, 10, 18),
      m.body
    );
    body.rotation.x = Math.PI * 0.5;
    body.scale.set(1.0, 0.72, 1.22);
    drone.add(body);

    const topPlate = new THREE.Mesh(
      new THREE.CylinderGeometry(0.18, 0.2, 0.05, 24),
      m.accent
    );
    topPlate.position.y = 0.08;
    drone.add(topPlate);

    const armLength = 0.64;
    const armRadius = 0.035;
    const motorOffsetY = 0.03;
    const motorRadius = 0.055;

    const armGeometry = new THREE.CylinderGeometry(armRadius, armRadius * 1.05, armLength, 16);
    const motorGeometry = new THREE.CylinderGeometry(motorRadius, motorRadius * 0.95, 0.07, 20);

    const armAngles = [45, 135, 225, 315].map((d) => THREE.MathUtils.degToRad(d));

    for (let i = 0; i < 4; i += 1) {
      const armPivot = new THREE.Group();
      armPivot.rotation.y = armAngles[i];

      const arm = new THREE.Mesh(armGeometry, m.carbon);
      arm.rotation.z = Math.PI * 0.5;
      arm.position.x = armLength * 0.5;
      arm.position.y = 0.01;
      armPivot.add(arm);

      const motor = new THREE.Mesh(motorGeometry, m.body);
      motor.rotation.x = Math.PI * 0.5;
      motor.position.set(armLength, motorOffsetY, 0);
      armPivot.add(motor);

      const propHub = new THREE.Group();
      propHub.position.set(armLength, 0.075, 0);
      propHub.userData.spinDir = i % 2 === 0 ? 1 : -1;

      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(0.022, 0.022, 0.02, 14),
        m.carbon
      );
      hub.position.y = 0.01;
      propHub.add(hub);

      const bladeGeom = new THREE.BoxGeometry(0.22, 0.006, 0.028);
      const blade1 = new THREE.Mesh(bladeGeom, m.prop);
      blade1.position.x = 0.11;
      blade1.position.y = 0.004;
      blade1.rotation.z = THREE.MathUtils.degToRad(4);
      propHub.add(blade1);

      const blade2 = blade1.clone();
      blade2.rotation.y = Math.PI;
      blade2.rotation.z = THREE.MathUtils.degToRad(-4);
      propHub.add(blade2);

      armPivot.add(propHub);
      this.propellers.push(propHub);

      drone.add(armPivot);
    }

    const camHousing = new THREE.Mesh(
      new THREE.BoxGeometry(0.16, 0.08, 0.1, 3, 2, 2),
      m.camera
    );
    camHousing.position.set(0, -0.03, -0.28);
    drone.add(camHousing);

    const camLens = new THREE.Mesh(
      new THREE.SphereGeometry(0.034, 16, 14),
      m.lens
    );
    camLens.position.set(0, -0.025, -0.334);
    camLens.scale.z = 0.72;
    drone.add(camLens);

    const ledStrip = new THREE.Mesh(
      new THREE.BoxGeometry(0.22, 0.015, 0.018),
      m.accent
    );
    ledStrip.position.set(0, 0.045, -0.17);
    drone.add(ledStrip);

    // Optional landing legs
    const legGeom = new THREE.CylinderGeometry(0.012, 0.014, 0.24, 10);
    const skidGeom = new THREE.CylinderGeometry(0.012, 0.012, 0.24, 10);

    [-1, 1].forEach((side) => {
      const leg = new THREE.Mesh(legGeom, m.leg);
      leg.position.set(side * 0.13, -0.16, -0.02);
      leg.rotation.z = side * THREE.MathUtils.degToRad(10);
      drone.add(leg);

      const skid = new THREE.Mesh(skidGeom, m.leg);
      skid.rotation.x = Math.PI * 0.5;
      skid.position.set(side * 0.12, -0.28, 0.06);
      drone.add(skid);
    });

    drone.scale.setScalar(baseScale);
    this.droneMesh.add(drone);

    // FPV camera mount in front of lens.
    this.fpvcamMount.position.set(0, -0.025 * baseScale, -0.38 * baseScale);
  }

  async _loadGLTF(url, baseScale) {
    this._disposeDroneMesh();
    this.propellers = [];

    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);
    const model = gltf.scene;

    model.traverse((obj) => {
      if (obj.isMesh) {
        obj.castShadow = true;
        obj.receiveShadow = true;
      }
    });

    // Normalize scale to a compact quadcopter footprint.
    const box = new THREE.Box3().setFromObject(model);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxSize = Math.max(size.x, size.y, size.z) || 1;
    const targetMax = 1.45 * baseScale;
    const s = targetMax / maxSize;
    model.scale.setScalar(s);

    // Recompute bounds after scale and center near origin.
    const centeredBox = new THREE.Box3().setFromObject(model);
    const center = new THREE.Vector3();
    centeredBox.getCenter(center);
    model.position.sub(center);
    model.position.y += 0.05 * baseScale;

    // Optional naming convention for propellers in GLTF:
    // prop_fl, prop_fr, prop_rl, prop_rr
    model.traverse((obj) => {
      if (!obj.name) return;
      const n = obj.name.toLowerCase();
      if (n.includes('prop') || n.includes('rotor')) {
        obj.userData.spinDir = n.includes('fl') || n.includes('rr') ? 1 : -1;
        this.propellers.push(obj);
      }
    });

    this.droneMesh.add(model);

    // A sensible default FPV mount for imported models.
    this.fpvcamMount.position.set(0, -0.02 * baseScale, -0.35 * baseScale);
  }

  _disposeDroneMesh() {
    while (this.droneMesh.children.length) {
      const c = this.droneMesh.children.pop();
      c.traverse?.((obj) => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) {
          if (Array.isArray(obj.material)) obj.material.forEach((mat) => mat.dispose());
          else obj.material.dispose();
        }
      });
    }
  }

  setThrottle(rawThrottle) {
    const t = Number.isFinite(rawThrottle) ? rawThrottle : 0;
    this.throttle = THREE.MathUtils.clamp(t, 0, 1);
  }

  setPose({ position, quaternion, rotationEuler }) {
    if (position) this.group.position.copy(position);
    if (quaternion) this.group.quaternion.copy(quaternion);
    if (!quaternion && rotationEuler) this.group.rotation.copy(rotationEuler);
  }

  /**
   * Updates propeller animation and optionally aligns an FPV camera.
   * @param {number} dt - delta time in seconds
   * @param {THREE.Camera} fpvCamera - existing simulation FPV camera
   */
  update(dt, fpvCamera = null) {
    const idle = 12;
    const max = 140;
    const target = idle + this.throttle * (max - idle);

    // Smooth response so spin speed changes naturally.
    const response = 1 - Math.exp(-8 * dt);
    this.propSpin += (target - this.propSpin) * response;

    const angle = this.propSpin * dt;
    for (let i = 0; i < this.propellers.length; i += 1) {
      const p = this.propellers[i];
      const dir = p.userData.spinDir || 1;
      p.rotation.y += angle * dir;
    }

    // Attach existing FPV camera logic to front mount, preserving world transform behavior.
    if (fpvCamera) {
      this.fpvcamMount.getWorldPosition(this._tmpPos);
      this.fpvcamMount.getWorldQuaternion(this._tmpQuat);
      fpvCamera.position.copy(this._tmpPos);
      fpvCamera.quaternion.copy(this._tmpQuat);

      // Slight forward look for stable FPV framing.
      const lookTarget = this._forward.clone().applyQuaternion(this._tmpQuat).multiplyScalar(8).add(this._tmpPos);
      fpvCamera.lookAt(lookTarget);
    }
  }

  dispose() {
    this._disposeDroneMesh();
    this.scene?.remove(this.group);
  }
}
