import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DroneVisual } from "./dronevisuals.js";

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
    this.visual = null;
    this._droneBounds = new THREE.Box3();
    this.flightConfig = { ...DEFAULT_FLIGHT_CONFIG };

    this._buildDroneGeometry();
    this.setFlightConfig(this.flightConfig);
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
    this.visual = new DroneVisual({ baseScale: 1.0 });
    this.group.add(this.visual.group);
    this.propellers = this.visual.propellers;
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
    const throttle = THREE.MathUtils.clamp(
      this.acceleration.length() / this.flightConfig.maxAcceleration,
      0,
      1,
    );
    this.visual?.setThrottle(throttle);
    this.visual?.update(delta);
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
