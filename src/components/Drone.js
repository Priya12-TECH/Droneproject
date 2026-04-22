import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

const ARM_LENGTH = 0.95;
const DEFAULT_FLIGHT_CONFIG = {
  maxAcceleration: 13.0,
  maxSpeed: 11.0,
  thrustFactor: 1.0,
  linearDamping: 0.88,
  weightKg: 1.0,
  payloadKg: 0.2,
};

/**
 * Reusable drone entity with mesh, propeller animation, and movement physics.
 */
export class Drone {
  constructor() {
    this.group = new THREE.Group();
    this.group.name = "Drone";

    this.velocity = new THREE.Vector3();
    this.acceleration = new THREE.Vector3();
    this._targetAcceleration = new THREE.Vector3();

    this.propellers = [];
    this.obstacles = [];
    this.avoidanceEnabled = false;
    this.collisionsThisFrame = 0;
    this._collisionPadding = 0.05;
    this._obstacleBounds = new THREE.Box3();
    this._droneBounds = new THREE.Box3();
    this.flightConfig = { ...DEFAULT_FLIGHT_CONFIG };

    this._buildDroneGeometry();
  }

  setFlightConfig(config = {}) {
    this.flightConfig = {
      ...this.flightConfig,
      ...config,
    };

    this.flightConfig.maxAcceleration = THREE.MathUtils.clamp(
      Number(this.flightConfig.maxAcceleration) ||
        DEFAULT_FLIGHT_CONFIG.maxAcceleration,
      4,
      28,
    );
    this.flightConfig.maxSpeed = THREE.MathUtils.clamp(
      Number(this.flightConfig.maxSpeed) || DEFAULT_FLIGHT_CONFIG.maxSpeed,
      3,
      30,
    );
    this.flightConfig.thrustFactor = THREE.MathUtils.clamp(
      Number(this.flightConfig.thrustFactor) ||
        DEFAULT_FLIGHT_CONFIG.thrustFactor,
      0.6,
      1.6,
    );
    this.flightConfig.linearDamping = THREE.MathUtils.clamp(
      Number(this.flightConfig.linearDamping) ||
        DEFAULT_FLIGHT_CONFIG.linearDamping,
      0.7,
      0.98,
    );
    this.flightConfig.weightKg = THREE.MathUtils.clamp(
      Number(this.flightConfig.weightKg) || DEFAULT_FLIGHT_CONFIG.weightKg,
      0.1,
      12,
    );
    this.flightConfig.payloadKg = THREE.MathUtils.clamp(
      Number(this.flightConfig.payloadKg) || DEFAULT_FLIGHT_CONFIG.payloadKg,
      0,
      6,
    );
  }

  getFlightConfig() {
    return { ...this.flightConfig };
  }

  _buildDroneGeometry() {
    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: 0x202835,
      metalness: 0.58,
      roughness: 0.32,
    });
    const accentMaterial = new THREE.MeshStandardMaterial({
      color: 0x2f3f58,
      metalness: 0.45,
      roughness: 0.4,
    });
    const matteMaterial = new THREE.MeshStandardMaterial({
      color: 0x171d27,
      metalness: 0.2,
      roughness: 0.72,
    });
    const motorMaterial = new THREE.MeshStandardMaterial({
      color: 0x252f3f,
      metalness: 0.55,
      roughness: 0.35,
    });
    const propMaterial = new THREE.MeshStandardMaterial({
      color: 0x2a3446,
      metalness: 0.15,
      roughness: 0.78,
    });

    // Main fuselage.
    const fuselage = new THREE.Mesh(
      new THREE.BoxGeometry(1.25, 0.26, 0.82),
      bodyMaterial,
    );
    fuselage.castShadow = true;
    fuselage.receiveShadow = true;
    this.group.add(fuselage);

    // Top canopy.
    const canopy = new THREE.Mesh(
      new THREE.SphereGeometry(0.38, 24, 16, 0, Math.PI * 2, 0, Math.PI * 0.55),
      new THREE.MeshStandardMaterial({
        color: 0x344966,
        metalness: 0.35,
        roughness: 0.22,
      }),
    );
    canopy.position.set(0, 0.14, -0.02);
    canopy.scale.set(1.2, 0.55, 1.05);
    canopy.castShadow = true;
    this.group.add(canopy);

    // Battery pack detail.
    const batteryPack = new THREE.Mesh(
      new THREE.BoxGeometry(0.6, 0.1, 0.38),
      accentMaterial,
    );
    batteryPack.position.set(0, 0.2, 0.18);
    batteryPack.castShadow = true;
    batteryPack.receiveShadow = true;
    this.group.add(batteryPack);

    // GPS puck.
    const gpsPuck = new THREE.Mesh(
      new THREE.CylinderGeometry(0.1, 0.1, 0.04, 20),
      matteMaterial,
    );
    gpsPuck.position.set(0, 0.28, 0.2);
    gpsPuck.castShadow = true;
    this.group.add(gpsPuck);

    // Front camera module.
    const cameraPod = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, 0.14, 0.2),
      matteMaterial,
    );
    cameraPod.position.set(0, -0.08, -0.5);
    cameraPod.castShadow = true;
    cameraPod.receiveShadow = true;
    this.group.add(cameraPod);

    const cameraLens = new THREE.Mesh(
      new THREE.CylinderGeometry(0.05, 0.05, 0.06, 18),
      new THREE.MeshStandardMaterial({
        color: 0x11151d,
        metalness: 0.72,
        roughness: 0.18,
      }),
    );
    cameraLens.rotation.x = Math.PI * 0.5;
    cameraLens.position.set(0, -0.08, -0.62);
    cameraLens.castShadow = true;
    this.group.add(cameraLens);

    // Landing gear (two skids with supports).
    const skidGeometry = new THREE.CylinderGeometry(0.025, 0.025, 1.0, 12);
    const leftSkid = new THREE.Mesh(skidGeometry, accentMaterial);
    leftSkid.rotation.z = Math.PI * 0.5;
    leftSkid.position.set(-0.28, -0.36, 0.02);
    const rightSkid = leftSkid.clone();
    rightSkid.position.x = 0.28;

    const gearStrutGeometry = new THREE.CylinderGeometry(
      0.018,
      0.018,
      0.24,
      10,
    );
    const strutA = new THREE.Mesh(gearStrutGeometry, accentMaterial);
    strutA.position.set(-0.28, -0.22, -0.2);
    const strutB = strutA.clone();
    strutB.position.set(-0.28, -0.22, 0.24);
    const strutC = strutA.clone();
    strutC.position.set(0.28, -0.22, -0.2);
    const strutD = strutA.clone();
    strutD.position.set(0.28, -0.22, 0.24);

    [leftSkid, rightSkid, strutA, strutB, strutC, strutD].forEach((mesh) => {
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.group.add(mesh);
    });

    // Arms.
    const armGeometry = new THREE.CylinderGeometry(
      0.05,
      0.05,
      ARM_LENGTH * 2.05,
      14,
    );
    const arm1 = new THREE.Mesh(armGeometry, bodyMaterial);
    arm1.rotation.z = Math.PI * 0.5;
    arm1.rotation.y = Math.PI * 0.25;
    arm1.castShadow = true;
    this.group.add(arm1);

    const arm2 = new THREE.Mesh(armGeometry, bodyMaterial);
    arm2.rotation.z = Math.PI * 0.5;
    arm2.rotation.y = -Math.PI * 0.25;
    arm2.castShadow = true;
    this.group.add(arm2);

    const rotorPoints = [
      new THREE.Vector3(ARM_LENGTH, 0.08, ARM_LENGTH),
      new THREE.Vector3(-ARM_LENGTH, 0.08, ARM_LENGTH),
      new THREE.Vector3(ARM_LENGTH, 0.08, -ARM_LENGTH),
      new THREE.Vector3(-ARM_LENGTH, 0.08, -ARM_LENGTH),
    ];

    rotorPoints.forEach((position, index) => {
      const motor = new THREE.Mesh(
        new THREE.CylinderGeometry(0.115, 0.125, 0.16, 20),
        motorMaterial,
      );
      motor.position.copy(position);
      motor.castShadow = true;
      motor.receiveShadow = true;
      this.group.add(motor);

      // Protective duct ring around rotor.
      const duct = new THREE.Mesh(
        new THREE.TorusGeometry(0.27, 0.02, 12, 42),
        accentMaterial,
      );
      duct.rotation.x = Math.PI * 0.5;
      duct.position.copy(position).add(new THREE.Vector3(0, 0.06, 0));
      duct.castShadow = true;
      this.group.add(duct);

      const propeller = new THREE.Group();
      propeller.position.copy(position).add(new THREE.Vector3(0, 0.105, 0));

      const bladeGeometry = new THREE.BoxGeometry(0.58, 0.014, 0.055);
      const blade1 = new THREE.Mesh(bladeGeometry, propMaterial);
      blade1.castShadow = true;

      const blade2 = blade1.clone();
      blade2.rotation.y = Math.PI * 0.5;

      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(0.042, 0.042, 0.022, 16),
        matteMaterial,
      );
      hub.castShadow = true;

      propeller.add(blade1, blade2, hub);
      this.group.add(propeller);
      this.propellers.push(propeller);

      // Navigation lights: front green, rear red.
      const isFront = index >= 2;
      const light = new THREE.Mesh(
        new THREE.SphereGeometry(0.028, 10, 10),
        new THREE.MeshStandardMaterial({
          color: isFront ? 0x39ff6e : 0xff4a4a,
          emissive: isFront ? 0x1e7d46 : 0x7d1e1e,
          emissiveIntensity: 0.8,
          metalness: 0.15,
          roughness: 0.45,
        }),
      );
      light.position
        .copy(position)
        .add(new THREE.Vector3(0, 0.11, isFront ? -0.08 : 0.08));
      this.group.add(light);
    });
  }

  /**
   * Optional obstacle list for future obstacle-avoidance behaviors.
   * @param {THREE.Object3D[]} obstacles
   */
  setObstacles(obstacles) {
    this.obstacles = obstacles;
  }

  setAvoidanceEnabled(enabled) {
    this.avoidanceEnabled = Boolean(enabled);
  }

  /**
   * Optional model loader to replace procedural drone body with a glTF model.
   */
  async loadVisualModel(url) {
    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync(url);

    gltf.scene.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });

    this.group.add(gltf.scene);
    return gltf.scene;
  }

  /**
   * Updates drone motion and propeller rotation.
   * @param {number} delta Seconds since last frame
   * @param {THREE.Vector3} input Movement input vector
   */
  update(delta, input) {
    const thrustBoost = this.flightConfig.thrustFactor;
    this._targetAcceleration
      .copy(input)
      .multiplyScalar(this.flightConfig.maxAcceleration * thrustBoost);

    // Placeholder for future obstacle-avoidance force blending.
    if (this.avoidanceEnabled && this.obstacles.length) {
      this._targetAcceleration.add(this._computeAvoidanceForce());
    }

    // Smooth transition toward target acceleration.
    const accelLerp = 1 - Math.exp(-12 * delta);
    this.acceleration.lerp(this._targetAcceleration, accelLerp);

    this.velocity.addScaledVector(this.acceleration, delta);

    // Clamp max speed.
    if (this.velocity.length() > this.flightConfig.maxSpeed) {
      this.velocity.setLength(this.flightConfig.maxSpeed);
    }

    // Exponential damping for stable feel at different frame rates.
    const damping = Math.pow(this.flightConfig.linearDamping, delta * 60);
    this.velocity.multiplyScalar(damping);

    this.group.position.addScaledVector(this.velocity, delta);
    this.group.position.y = Math.max(0.55, this.group.position.y);
    this._resolveObstacleCollisions();

    // Optional tilt effect based on horizontal velocity.
    const tiltAmount = 0.045;
    const targetTiltX = THREE.MathUtils.clamp(
      this.velocity.z * tiltAmount,
      -0.35,
      0.35,
    );
    const targetTiltZ = THREE.MathUtils.clamp(
      -this.velocity.x * tiltAmount,
      -0.35,
      0.35,
    );
    this.group.rotation.x = THREE.MathUtils.lerp(
      this.group.rotation.x,
      targetTiltX,
      1 - Math.exp(-10 * delta),
    );
    this.group.rotation.z = THREE.MathUtils.lerp(
      this.group.rotation.z,
      targetTiltZ,
      1 - Math.exp(-10 * delta),
    );

    // Propeller speed scales with current acceleration magnitude.
    const spinSpeed = 18 + this.acceleration.length() * 1.35;
    for (let i = 0; i < this.propellers.length; i += 1) {
      const direction = i % 2 === 0 ? 1 : -1;
      this.propellers[i].rotation.y += spinSpeed * direction * delta;
    }
  }

  _computeAvoidanceForce() {
    const force = new THREE.Vector3();
    const threshold = 2.8;

    this.obstacles.forEach((obstacle) => {
      const away = this.group.position.clone().sub(obstacle.position);
      const distance = away.length();
      if (distance > 0.0001 && distance < threshold) {
        const strength =
          ((threshold - distance) / threshold) *
          this.flightConfig.maxAcceleration *
          0.5;
        force.add(away.normalize().multiplyScalar(strength));
      }
    });

    return force;
  }

  _resolveObstacleCollisions() {
    this.collisionsThisFrame = 0;
    if (!this.obstacles.length) return;

    for (let i = 0; i < this.obstacles.length; i += 1) {
      const obstacle = this.obstacles[i];
      this._obstacleBounds
        .setFromObject(obstacle)
        .expandByScalar(this._collisionPadding);
      this._droneBounds.setFromObject(this.group);

      if (!this._droneBounds.intersectsBox(this._obstacleBounds)) continue;

      this.collisionsThisFrame += 1;

      const overlapX =
        Math.min(this._droneBounds.max.x, this._obstacleBounds.max.x) -
        Math.max(this._droneBounds.min.x, this._obstacleBounds.min.x);
      const overlapY =
        Math.min(this._droneBounds.max.y, this._obstacleBounds.max.y) -
        Math.max(this._droneBounds.min.y, this._obstacleBounds.min.y);
      const overlapZ =
        Math.min(this._droneBounds.max.z, this._obstacleBounds.max.z) -
        Math.max(this._droneBounds.min.z, this._obstacleBounds.min.z);

      const overlaps = { x: overlapX, y: overlapY, z: overlapZ };

      let axis = "x";
      let minOverlap = overlaps.x;
      Object.entries(overlaps).forEach(([name, value]) => {
        if (value < minOverlap) {
          minOverlap = value;
          axis = name;
        }
      });

      const epsilon = 0.001;
      const p = this.group.position;
      const droneCenter = this._droneBounds.getCenter(new THREE.Vector3());
      const obstacleCenter = this._obstacleBounds.getCenter(
        new THREE.Vector3(),
      );

      if (axis === "x") {
        const dir = droneCenter.x >= obstacleCenter.x ? 1 : -1;
        p.x += (minOverlap + epsilon) * dir;
        this.velocity.x *= 0.2;
      } else if (axis === "y") {
        const dir = droneCenter.y >= obstacleCenter.y ? 1 : -1;
        p.y = Math.max(0.55, p.y + (minOverlap + epsilon) * dir);
        this.velocity.y *= 0.2;
      } else {
        const dir = droneCenter.z >= obstacleCenter.z ? 1 : -1;
        p.z += (minOverlap + epsilon) * dir;
        this.velocity.z *= 0.2;
      }
    }
  }
}
