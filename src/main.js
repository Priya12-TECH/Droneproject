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
    this.followMode = false;
    this.telemetryIframe = null;
    this._telemetryLastSent = 0;
    this.telemetryInput = new THREE.Vector3();
    this.commandMode = "manual";
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
    this._setupTelemetryMessaging();

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

    this.scene.add(obstacleA, obstacleB);
    this.drone.setObstacles([obstacleA, obstacleB]);
  }

  _setupEvents() {
    this.renderer.domElement.addEventListener("pointerdown", () => {
      this.renderer.domElement.focus();
    });

    window.addEventListener("resize", () => {
      this.camera.aspect = window.innerWidth / window.innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(window.innerWidth, window.innerHeight);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    });

    window.addEventListener("keydown", (event) => {
      if (event.code === "KeyC") {
        this.followMode = !this.followMode;
      }

      if (event.code === "Space") {
        event.preventDefault();
        this.commandMode = "hover";
      }

      if (event.code === "KeyL") {
        event.preventDefault();
        this.commandMode = "land";
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
          break;
        case "down":
          this.commandMode = "manual";
          this.telemetryInput.set(0, -1, 0);
          break;
        case "land":
          this.commandMode = "land";
          this.telemetryInput.set(0, 0, 0);
          break;
        case "forward":
          this.commandMode = "manual";
          this.telemetryInput.set(0, 0, -1);
          break;
        case "backward":
          this.commandMode = "manual";
          this.telemetryInput.set(0, 0, 1);
          break;
        case "left":
          this.commandMode = "manual";
          this.telemetryInput.set(-1, 0, 0);
          break;
        case "right":
          this.commandMode = "manual";
          this.telemetryInput.set(1, 0, 0);
          break;
        case "hover":
          this.commandMode = "hover";
          this.telemetryInput.set(0, 0, 0);
          break;
        default:
          break;
      }
    });
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

    const keyboardInput = this.keyboard.getInputVector();

    // Keyboard should always be able to take control back from LAND/HOVER modes.
    if (keyboardInput.lengthSq() > 0) {
      this.commandMode = "manual";
      this.telemetryInput.set(0, 0, 0);
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

    this.drone.update(delta, input);

    const spd = this.drone.velocity.length();
    const isMoving = spd > 0.06 || altitude > 0.08;

    if (isMoving) {
      const drain = 0.3 + Math.max(0, spd - 0.3) * 0.06 + altitude * 0.01;
      this.telemetry.battery = Math.max(
        0,
        this.telemetry.battery - drain * delta,
      );
    }

    this.telemetry.cpu = THREE.MathUtils.clamp(
      10 +
        spd * 6 +
        (this.commandMode === "hover" ? 2 : 0) +
        (this.commandMode === "land" ? 4 : 0),
      6,
      95,
    );
    this.telemetry.gps = THREE.MathUtils.clamp(100 - altitude * 0.25, 70, 100);

    if (this.commandMode === "land") {
      this.telemetry.status = altitude > 0.05 ? "LANDING" : "IDLE";
    } else if (isMoving) {
      this.telemetry.status = "FLYING";
    } else if (this.commandMode === "hover" && altitude > 0.05) {
      this.telemetry.status = "HOVER";
    } else {
      this.telemetry.status = "IDLE";
    }

    if (this.followMode) {
      this._updateFollowCamera(delta);
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this._postTelemetry();
    requestAnimationFrame(this.animate);
  }
}

const root = document.getElementById("app");
new DroneSimulatorApp(root);
