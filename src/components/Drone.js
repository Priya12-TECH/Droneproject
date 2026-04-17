import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

const ARM_LENGTH = 0.95;
const MAX_ACCELERATION = 13.0;
const MAX_SPEED = 11.0;
const LINEAR_DAMPING = 0.88;

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
    this._collisionRadius = 0.62;
    this._obstacleBounds = new THREE.Box3();

    this._buildDroneGeometry();
  }

  _buildDroneGeometry() {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(1.3, 0.35, 1.0),
      new THREE.MeshStandardMaterial({
        color: 0x2a2f3b,
        metalness: 0.55,
        roughness: 0.35,
      }),
    );
    body.castShadow = true;
    body.receiveShadow = true;
    this.group.add(body);

    const armGeometry = new THREE.CylinderGeometry(
      0.065,
      0.065,
      ARM_LENGTH * 2,
      16,
    );
    const armMaterial = new THREE.MeshStandardMaterial({
      color: 0x2a2f3b,
      metalness: 0.6,
      roughness: 0.25,
    });

    const arm1 = new THREE.Mesh(armGeometry, armMaterial);
    arm1.rotation.z = Math.PI * 0.5;
    arm1.rotation.y = Math.PI * 0.25;
    arm1.castShadow = true;
    this.group.add(arm1);

    const arm2 = new THREE.Mesh(armGeometry, armMaterial);
    arm2.rotation.z = Math.PI * 0.5;
    arm2.rotation.y = -Math.PI * 0.25;
    arm2.castShadow = true;
    this.group.add(arm2);

    const motorMaterial = new THREE.MeshStandardMaterial({
      color: 0x2a2f3b,
      metalness: 0.4,
      roughness: 0.4,
    });
    const propMaterial = new THREE.MeshStandardMaterial({
      color: 0x2a2f3b,
      metalness: 0.2,
      roughness: 0.7,
    });

    const rotorPoints = [
      new THREE.Vector3(ARM_LENGTH, 0.08, ARM_LENGTH),
      new THREE.Vector3(-ARM_LENGTH, 0.08, ARM_LENGTH),
      new THREE.Vector3(ARM_LENGTH, 0.08, -ARM_LENGTH),
      new THREE.Vector3(-ARM_LENGTH, 0.08, -ARM_LENGTH),
    ];

    rotorPoints.forEach((position) => {
      const motor = new THREE.Mesh(
        new THREE.CylinderGeometry(0.12, 0.12, 0.14, 20),
        motorMaterial,
      );
      motor.position.copy(position);
      motor.castShadow = true;
      this.group.add(motor);

      const propeller = new THREE.Group();
      propeller.position.copy(position).add(new THREE.Vector3(0, 0.09, 0));

      const bladeGeometry = new THREE.BoxGeometry(0.65, 0.018, 0.08);
      const blade1 = new THREE.Mesh(bladeGeometry, propMaterial);
      blade1.castShadow = true;

      const blade2 = blade1.clone();
      blade2.rotation.y = Math.PI * 0.5;

      propeller.add(blade1, blade2);
      this.group.add(propeller);
      this.propellers.push(propeller);
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
    this._targetAcceleration.copy(input).multiplyScalar(MAX_ACCELERATION);

    // Placeholder for future obstacle-avoidance force blending.
    if (this.avoidanceEnabled && this.obstacles.length) {
      this._targetAcceleration.add(this._computeAvoidanceForce());
    }

    // Smooth transition toward target acceleration.
    const accelLerp = 1 - Math.exp(-12 * delta);
    this.acceleration.lerp(this._targetAcceleration, accelLerp);

    this.velocity.addScaledVector(this.acceleration, delta);

    // Clamp max speed.
    if (this.velocity.length() > MAX_SPEED) {
      this.velocity.setLength(MAX_SPEED);
    }

    // Exponential damping for stable feel at different frame rates.
    const damping = Math.pow(LINEAR_DAMPING, delta * 60);
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
    const spinSpeed = 20 + this.acceleration.length() * 1.4;
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
          ((threshold - distance) / threshold) * MAX_ACCELERATION * 0.5;
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
        .expandByScalar(this._collisionRadius);

      const p = this.group.position;
      if (!this._obstacleBounds.containsPoint(p)) continue;

      this.collisionsThisFrame += 1;

      const distances = {
        minX: Math.abs(p.x - this._obstacleBounds.min.x),
        maxX: Math.abs(this._obstacleBounds.max.x - p.x),
        minY: Math.abs(p.y - this._obstacleBounds.min.y),
        maxY: Math.abs(this._obstacleBounds.max.y - p.y),
        minZ: Math.abs(p.z - this._obstacleBounds.min.z),
        maxZ: Math.abs(this._obstacleBounds.max.z - p.z),
      };

      let side = "minX";
      let minDistance = distances.minX;
      Object.entries(distances).forEach(([name, value]) => {
        if (value < minDistance) {
          minDistance = value;
          side = name;
        }
      });

      const epsilon = 0.001;
      switch (side) {
        case "minX":
          p.x = this._obstacleBounds.min.x - epsilon;
          this.velocity.x = Math.min(0, this.velocity.x) * 0.2;
          break;
        case "maxX":
          p.x = this._obstacleBounds.max.x + epsilon;
          this.velocity.x = Math.max(0, this.velocity.x) * 0.2;
          break;
        case "minY":
          p.y = Math.max(0.55, this._obstacleBounds.min.y - epsilon);
          this.velocity.y = Math.min(0, this.velocity.y) * 0.2;
          break;
        case "maxY":
          p.y = this._obstacleBounds.max.y + epsilon;
          this.velocity.y = Math.max(0, this.velocity.y) * 0.2;
          break;
        case "minZ":
          p.z = this._obstacleBounds.min.z - epsilon;
          this.velocity.z = Math.min(0, this.velocity.z) * 0.2;
          break;
        case "maxZ":
          p.z = this._obstacleBounds.max.z + epsilon;
          this.velocity.z = Math.max(0, this.velocity.z) * 0.2;
          break;
        default:
          break;
      }
    }
  }
}
