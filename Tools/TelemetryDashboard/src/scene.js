import * as THREE from "three";

const DEG_TO_RAD = Math.PI / 180;
const FIN_LIMIT_DEG = 35;

function makeMaterial(color, roughness = 0.72, metalness = 0.12) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function createOgiveGeometry(radius, height, radialSegments = 64) {
  const points = [];
  const pointCount = 18;

  for (let i = 0; i <= pointCount; i += 1) {
    const t = i / pointCount;
    const y = t * height;
    const r = radius * Math.cos(t * Math.PI * 0.5);
    points.push(new THREE.Vector2(r, y));
  }

  return new THREE.LatheGeometry(points, radialSegments);
}

function normalizedServos(snapshot) {
  if (Array.isArray(snapshot.servosDeg) && snapshot.servosDeg.length >= 4) {
    return snapshot.servosDeg.slice(0, 4).map((value) => (Number.isFinite(value) ? value : 0));
  }

  return [0, 0, 0, 0];
}

export class VehicleScene {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x0f1416, 1);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100);
    this.camera.position.set(4.1, 2.9, 5.0);
    this.camera.lookAt(0, 1.35, 0);

    this.vehicleGroup = new THREE.Group();
    this.scene.add(this.vehicleGroup);

    this.targetLine = null;
    this.finActuators = [];
    this.buildScene();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
  }

  buildScene() {
    const hemi = new THREE.HemisphereLight(0xd8ffff, 0x192224, 1.35);
    this.scene.add(hemi);

    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(3.5, 5.2, 4.0);
    this.scene.add(key);

    const fill = new THREE.DirectionalLight(0x7ed6c6, 0.65);
    fill.position.set(-3, 2.3, -4);
    this.scene.add(fill);

    const grid = new THREE.GridHelper(7, 14, 0x405055, 0x263236);
    grid.position.y = -0.03;
    this.scene.add(grid);

    const targetGeometry = new THREE.CylinderGeometry(0.012, 0.012, 3.25, 16);
    const targetMaterial = new THREE.MeshBasicMaterial({ color: 0x65d6b2, transparent: true, opacity: 0.45 });
    this.targetLine = new THREE.Mesh(targetGeometry, targetMaterial);
    this.targetLine.position.y = 1.6;
    this.scene.add(this.targetLine);

    const base = new THREE.Mesh(new THREE.CylinderGeometry(1.02, 1.18, 0.16, 72), makeMaterial(0x252d31, 0.8, 0.18));
    base.position.y = 0.02;
    this.scene.add(base);

    const gimbalRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.52, 0.018, 12, 96),
      makeMaterial(0x58686d, 0.58, 0.28)
    );
    gimbalRing.rotation.x = Math.PI / 2;
    gimbalRing.position.y = 0.22;
    this.scene.add(gimbalRing);

    this.buildRocket();
  }

  buildRocket() {
    const bodyMat = makeMaterial(0xdfe8e4, 0.5, 0.18);
    const darkMat = makeMaterial(0x2d3639, 0.7, 0.28);
    const bandMat = makeMaterial(0x215e68, 0.56, 0.24);
    const noseMat = makeMaterial(0xe7b64b, 0.44, 0.18);
    const finMat = makeMaterial(0xe06b4f, 0.48, 0.12);

    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.2, 2.42, 72), bodyMat);
    body.position.y = 1.43;
    this.vehicleGroup.add(body);

    const upperShoulder = new THREE.Mesh(new THREE.CylinderGeometry(0.19, 0.18, 0.12, 72), bodyMat);
    upperShoulder.position.y = 2.70;
    this.vehicleGroup.add(upperShoulder);

    const nose = new THREE.Mesh(createOgiveGeometry(0.19, 0.66), noseMat);
    nose.position.y = 2.76;
    this.vehicleGroup.add(nose);

    const avionicsBand = new THREE.Mesh(new THREE.CylinderGeometry(0.205, 0.205, 0.12, 72), bandMat);
    avionicsBand.position.y = 2.21;
    this.vehicleGroup.add(avionicsBand);

    const lowerBand = new THREE.Mesh(new THREE.CylinderGeometry(0.215, 0.215, 0.08, 72), bandMat);
    lowerBand.position.y = 0.62;
    this.vehicleGroup.add(lowerBand);

    const nozzle = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.18, 0.26, 48), darkMat);
    nozzle.position.y = 0.12;
    this.vehicleGroup.add(nozzle);

    const nozzleLip = new THREE.Mesh(
      new THREE.TorusGeometry(0.15, 0.012, 10, 64),
      makeMaterial(0x738188, 0.52, 0.35)
    );
    nozzleLip.rotation.x = Math.PI / 2;
    nozzleLip.position.y = 0.0;
    this.vehicleGroup.add(nozzleLip);

    const antenna = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.42, 10), makeMaterial(0x65d6b2, 0.5, 0.1));
    antenna.position.set(0.06, 3.46, 0);
    antenna.rotation.z = -0.12;
    this.vehicleGroup.add(antenna);

    [1.02, 1.48, 1.93].forEach((y) => {
      const seam = new THREE.Mesh(
        new THREE.TorusGeometry(0.183, 0.0045, 8, 72),
        makeMaterial(0x8fa19e, 0.56, 0.2)
      );
      seam.rotation.x = Math.PI / 2;
      seam.position.y = y;
      this.vehicleGroup.add(seam);
    });

    this.addFins(finMat);
  }

  addFins(finMat) {
    const finSpecs = [
      { label: "A", azimuthDeg: 0, sign: 1 },
      { label: "B", azimuthDeg: 90, sign: 1 },
      { label: "C", azimuthDeg: 180, sign: 1 },
      { label: "D", azimuthDeg: 270, sign: 1 }
    ];
    const bodyRadius = 0.205;

    this.finActuators = finSpecs.map((spec) => {
      const angleRad = spec.azimuthDeg * DEG_TO_RAD;
      const fin = this.createServoFin(finMat, spec.label);
      fin.mount.position.set(Math.cos(angleRad) * bodyRadius, 0.58, Math.sin(angleRad) * bodyRadius);
      fin.mount.rotation.y = angleRad;
      this.vehicleGroup.add(fin.mount);
      return {
        deflector: fin.deflector,
        sign: spec.sign
      };
    });
  }

  createServoFin(material, label) {
    const mount = new THREE.Group();
    const deflector = new THREE.Group();
    const shape = new THREE.Shape();

    shape.moveTo(0.0, -0.28);
    shape.lineTo(0.54, -0.38);
    shape.lineTo(0.42, 0.22);
    shape.lineTo(0.0, 0.30);
    shape.lineTo(0.0, -0.28);

    const finGeometry = new THREE.ExtrudeGeometry(shape, {
      depth: 0.036,
      bevelEnabled: true,
      bevelThickness: 0.006,
      bevelSize: 0.006,
      bevelSegments: 1
    });
    finGeometry.translate(0.0, 0.0, -0.018);

    const blade = new THREE.Mesh(finGeometry, material);
    blade.position.x = 0.018;
    deflector.add(blade);

    const hinge = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.66, 18), makeMaterial(0x94a2a6, 0.42, 0.36));
    hinge.position.y = -0.02;
    mount.add(hinge);

    const servoHorn = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.045, 0.07), makeMaterial(0x39474b, 0.55, 0.3));
    servoHorn.position.set(0.065, -0.18, 0);
    deflector.add(servoHorn);

    const labelSprite = this.createFinLabel(label);
    labelSprite.position.set(0.37, 0.08, 0.035);
    deflector.add(labelSprite);

    mount.add(deflector);
    return { mount, deflector };
  }

  createFinLabel(label) {
    const canvas = document.createElement("canvas");
    canvas.width = 96;
    canvas.height = 64;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "rgba(15, 20, 22, 0.8)";
    ctx.fillRect(18, 12, 60, 40);
    ctx.fillStyle = "#eaf3ef";
    ctx.font = "700 28px Inter, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, 48, 33);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(0.12, 0.08, 1);
    return sprite;
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width));
    const height = Math.max(1, Math.floor(rect.height));
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  update(snapshot) {
    const servosDeg = normalizedServos(snapshot);

    this.vehicleGroup.rotation.order = "YXZ";
    this.vehicleGroup.rotation.y = snapshot.yawDeg * DEG_TO_RAD;
    this.vehicleGroup.rotation.x = snapshot.pitchDeg * DEG_TO_RAD;
    this.vehicleGroup.rotation.z = -snapshot.rollDeg * DEG_TO_RAD;

    this.finActuators.forEach((actuator, index) => {
      const angleDeg = clamp(servosDeg[index] ?? 0, -FIN_LIMIT_DEG, FIN_LIMIT_DEG);
      actuator.deflector.rotation.y = actuator.sign * angleDeg * DEG_TO_RAD;
    });
  }

  render(timeMs) {
    this.targetLine.material.opacity = 0.28 + (0.12 * Math.sin(timeMs * 0.003));
    this.renderer.render(this.scene, this.camera);
  }
}
