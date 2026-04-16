import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

/**
 * Creates and configures OrbitControls.
 */
export function createOrbitControls(camera, domElement) {
  const controls = new OrbitControls(camera, domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.enableZoom = true;
  controls.enableRotate = true;
  controls.minDistance = 2;
  controls.maxDistance = 80;
  controls.maxPolarAngle = Math.PI * 0.49;
  return controls;
}

/**
 * Keyboard input handler for smooth drone motion.
 * W/S -> forward/backward, A/D -> left/right, Q/E -> up/down
 */
export class KeyboardFlightController {
  constructor() {
    this.keys = new Set();
    this.enabled = true;
    this._movementKeys = new Set(['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);

    this._onKeyDown = (event) => {
      const normalizedKey = this._normalizeMovementKey(event);
      if (normalizedKey) {
        event.preventDefault();
        this.keys.add(normalizedKey);
      }
    };
    this._onKeyUp = (event) => {
      const normalizedKey = this._normalizeMovementKey(event);
      if (normalizedKey) {
        event.preventDefault();
        this.keys.delete(normalizedKey);
      }
    };

    document.addEventListener('keydown', this._onKeyDown);
    document.addEventListener('keyup', this._onKeyUp);
    window.addEventListener('blur', () => this.keys.clear());
  }

  getInputVector() {
    if (!this.enabled) return new THREE.Vector3();

    const x = ((this.keys.has('KeyD') || this.keys.has('ArrowRight')) ? 1 : 0) - ((this.keys.has('KeyA') || this.keys.has('ArrowLeft')) ? 1 : 0);
    const y = (this.keys.has('KeyE') ? 1 : 0) - (this.keys.has('KeyQ') ? 1 : 0);
    const z = ((this.keys.has('KeyS') || this.keys.has('ArrowDown')) ? 1 : 0) - ((this.keys.has('KeyW') || this.keys.has('ArrowUp')) ? 1 : 0);

    const input = new THREE.Vector3(x, y, z);
    if (input.lengthSq() > 1) input.normalize();
    return input;
  }

  _normalizeMovementKey(event) {
    if (this._movementKeys.has(event.code)) {
      return event.code;
    }

    switch ((event.key || '').toLowerCase()) {
      case 'w':
        return 'KeyW';
      case 's':
        return 'KeyS';
      case 'a':
        return 'KeyA';
      case 'd':
        return 'KeyD';
      case 'q':
        return 'KeyQ';
      case 'e':
        return 'KeyE';
      case 'arrowup':
        return 'ArrowUp';
      case 'arrowdown':
        return 'ArrowDown';
      case 'arrowleft':
        return 'ArrowLeft';
      case 'arrowright':
        return 'ArrowRight';
      default:
        return null;
    }
  }

  dispose() {
    document.removeEventListener('keydown', this._onKeyDown);
    document.removeEventListener('keyup', this._onKeyUp);
  }
}
