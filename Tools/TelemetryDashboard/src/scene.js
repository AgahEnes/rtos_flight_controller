import * as THREE from "three";

const DEG_TO_RAD = Math.PI / 180;

function makeMaterial(color, roughness = 0.72, metalness = 0.12) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

export class VehicleScene {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x0f1416, 1);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(36, 1, 0.1, 100);
    this.camera.position.set(4.2, 3.1, 5.2);
    this.camera.lookAt(0, 1.2, 0);

    this.vehicleGroup = new THREE.Group();
    this.scene.add(this.vehicleGroup);

    this.targetLine = null;
    this.servoA = null;
    this.servoB = null;
    this.buildScene();
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
  }

  buildScene() {
    const hemi = new THREE.HemisphereLight(0xd8ffff, 0x192224, 1.3);
    this.scene.add(hemi);

    const key = new THREE.DirectionalLight(0xffffff, 2.1);
    key.position.set(3, 5, 4);
    this.scene.add(key);

    const grid = new THREE.GridHelper(7, 14, 0x405055, 0x263236);
    grid.position.y = -0.03;
    this.scene.add(grid);

    const targetGeometry = new THREE.CylinderGeometry(0.012, 0.012, 3.2, 16);
    const targetMaterial = new THREE.MeshBasicMaterial({ color: 0x65d6b2, transparent: true, opacity: 0.45 });
    this.targetLine = new THREE.Mesh(targetGeometry, targetMaterial);
    this.targetLine.position.y = 1.6;
    this.scene.add(this.targetLine);

    const base = new THREE.Mesh(new THREE.CylinderGeometry(1.05, 1.2, 0.16, 64), makeMaterial(0x293238, 0.8, 0.18));
    base.position.y = 0.02;
    this.scene.add(base);

    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.22, 2.35, 48), makeMaterial(0xe8efec, 0.58, 0.08));
    body.position.y = 1.35;
    this.vehicleGroup.add(body);

    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.44, 48), makeMaterial(0xf2b84b, 0.52, 0.1));
    nose.position.y = 2.74;
    this.vehicleGroup.add(nose);

    const deck = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.09, 0.64), makeMaterial(0x2f6f78, 0.7, 0.18));
    deck.position.y = 2.22;
    this.vehicleGroup.add(deck);

    const board = new THREE.Mesh(new THREE.BoxGeometry(0.52, 0.055, 0.38), makeMaterial(0x1d8b6a, 0.65, 0.08));
    board.position.y = 2.31;
    this.vehicleGroup.add(board);

    const finMat = makeMaterial(0xe86f51, 0.55, 0.08);
    this.servoA = this.createFin(finMat);
    this.servoA.position.set(0.36, 0.45, 0);
    this.servoA.rotation.z = -18 * DEG_TO_RAD;
    this.vehicleGroup.add(this.servoA);

    this.servoB = this.createFin(finMat);
    this.servoB.position.set(0, 0.45, 0.36);
    this.servoB.rotation.x = 18 * DEG_TO_RAD;
    this.vehicleGroup.add(this.servoB);

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.46, 0.018, 12, 72),
      makeMaterial(0x617177, 0.62, 0.24)
    );
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 0.42;
    this.vehicleGroup.add(ring);
  }

  createFin(material) {
    const group = new THREE.Group();
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.38, 0.62), material);
    fin.position.y = 0.05;
    group.add(fin);
    const hinge = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.42, 20), makeMaterial(0x8d9da3, 0.44, 0.35));
    hinge.rotation.z = Math.PI / 2;
    hinge.position.y = -0.18;
    group.add(hinge);
    return group;
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
    this.vehicleGroup.rotation.order = "YXZ";
    this.vehicleGroup.rotation.y = snapshot.yawDeg * DEG_TO_RAD;
    this.vehicleGroup.rotation.x = snapshot.pitchDeg * DEG_TO_RAD;
    this.vehicleGroup.rotation.z = -snapshot.rollDeg * DEG_TO_RAD;

    if (this.servoA) {
      this.servoA.rotation.z = (-18 + snapshot.servoA) * DEG_TO_RAD;
    }
    if (this.servoB) {
      this.servoB.rotation.x = (18 + snapshot.servoB) * DEG_TO_RAD;
    }
  }

  render(timeMs) {
    this.targetLine.material.opacity = 0.28 + (0.12 * Math.sin(timeMs * 0.003));
    this.renderer.render(this.scene, this.camera);
  }
}
