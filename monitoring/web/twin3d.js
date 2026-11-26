// Three.js 3D 디지털 트윈 뷰.
// 씬 루트를 x축 -90° 회전해 ROS 좌표계(x 전방, y 좌측, z 위)를 그대로 쓴다.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// simulation/worlds/*.sdf, patrol_path/config/waypoints_*.yaml 과 맞춘 배치
export const WORLDS = {
  corridor: {
    size: [45, 3.5, 4],
    obstacles: [
      { x: 8, y: 0.6, yaw: 0 },
      { x: -6, y: -0.5, yaw: 0 },
    ],
    gauges: [{ name: "gauge_corridor", x: 15, y: 1.55, z: 1.4 }],
    tanks: [],
    waypoints: [
      { name: "home", x: -20, y: 0 },
      { name: "mid_obstacle", x: 0, y: 0 },
      { name: "gauge_corridor", x: 15, y: 0.4 },
      { name: "east_end", x: 20, y: 0 },
    ],
    camera: [-14, -14, 12],
  },
  factory: {
    size: [25, 18, 7],
    obstacles: [
      { x: 6, y: 4, yaw: 0.3 },
      { x: -5, y: -3, yaw: -0.4 },
    ],
    gauges: [
      { name: "gauge_line1", x: 10, y: 8.7, z: 1.5 },
      { name: "gauge_line2", x: -8, y: 8.7, z: 1.5 },
    ],
    tanks: [
      { x: 8, y: -6 },
      { x: 10, y: -6 },
    ],
    // plume_sim 기본 파라미터
    plume: { x: 8, y: -6, sigma: 3, peak: 100 },
    waypoints: [
      { name: "home", x: -10, y: 0 },
      { name: "gauge_line1", x: 10, y: 7.5 },
      { name: "gauge_line2", x: -8, y: 7.5 },
      { name: "gas_tank", x: 8, y: -5 },
    ],
    camera: [-18, -16, 16],
  },
};

const COLORS = {
  floor: 0x1b232d,
  grid: 0x2a3542,
  wall: 0x3a4757,
  obstacle: 0x6b7787,
  tank: 0xb8c2cc,
  gauge: 0xe8eef4,
  robot: 0xe87722,
  leg: 0x2b3440,
  trail: 0x4fa3e0,
  waypoint: 0x8b9aab,
  goal: 0x3dba84,
  rth: 0xd4a017,
  seek: 0xc4453c,
};

function makeLabel(text, color = "#e8eef4") {
  const c = document.createElement("canvas");
  c.width = 256;
  c.height = 64;
  const g = c.getContext("2d");
  g.font = "600 28px 'IBM Plex Sans KR', sans-serif";
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillStyle = "rgba(15,20,25,0.75)";
  const w = Math.min(250, g.measureText(text).width + 24);
  g.fillRect(128 - w / 2, 12, w, 40);
  g.fillStyle = color;
  g.fillText(text, 128, 33);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  s.scale.set(2.4, 0.6, 1);
  s.renderOrder = 10;
  return s;
}

// 플룸 가우시안을 바닥 텍스처로 그린다 (plume_sim 과 같은 식)
function plumeTexture(world) {
  const [L, W] = world.size;
  const p = world.plume;
  const res = 256;
  const c = document.createElement("canvas");
  c.width = res;
  c.height = Math.round((res * W) / L);
  const g = c.getContext("2d");
  const img = g.createImageData(c.width, c.height);
  for (let j = 0; j < c.height; j++) {
    for (let i = 0; i < c.width; i++) {
      const x = -L / 2 + ((i + 0.5) / c.width) * L;
      const y = W / 2 - ((j + 0.5) / c.height) * W;
      const v = Math.exp(-((x - p.x) ** 2 + (y - p.y) ** 2) / (2 * p.sigma ** 2));
      const k = (j * c.width + i) * 4;
      img.data[k] = 255;
      img.data[k + 1] = Math.round(200 * (1 - v));
      img.data[k + 2] = 40;
      img.data[k + 3] = Math.round(200 * v);
    }
  }
  g.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

function buildRobot() {
  // Go2 크기 근사 (0.7 × 0.31 × 0.4 m)
  const robot = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.6, 0.28, 0.14),
    new THREE.MeshStandardMaterial({ color: COLORS.robot, roughness: 0.5 })
  );
  body.position.z = 0.32;
  body.castShadow = true;
  robot.add(body);

  const head = new THREE.Mesh(
    new THREE.BoxGeometry(0.1, 0.18, 0.08),
    new THREE.MeshStandardMaterial({ color: 0x222a33 })
  );
  head.position.set(0.33, 0, 0.34);
  robot.add(head);

  const legMat = new THREE.MeshStandardMaterial({ color: COLORS.leg });
  const legs = [];
  for (const [lx, ly] of [
    [0.22, 0.15],
    [0.22, -0.15],
    [-0.22, 0.15],
    [-0.22, -0.15],
  ]) {
    const hip = new THREE.Group();
    hip.position.set(lx, ly, 0.3);
    const thigh = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.04, 0.3), legMat);
    thigh.position.z = -0.15;
    thigh.castShadow = true;
    hip.add(thigh);
    robot.add(hip);
    legs.push(hip);
  }

  // 가스 농도 링
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.55, 0.65, 48),
    new THREE.MeshBasicMaterial({ color: 0x3dba84, transparent: true, opacity: 0.8, side: THREE.DoubleSide })
  );
  ring.position.z = 0.01;
  robot.add(ring);

  // 전방 표시
  const arrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.08, 0.2, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff })
  );
  arrow.rotation.z = -Math.PI / 2;
  arrow.position.set(0.75, 0, 0.02);
  robot.add(arrow);

  return { robot, body, legs, ring };
}

function goalMarker(color, label) {
  const g = new THREE.Group();
  const pin = new THREE.Mesh(
    new THREE.ConeGeometry(0.18, 0.5, 16),
    new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.4 })
  );
  pin.rotation.x = -Math.PI / 2; // 끝이 아래(−z)를 향하게
  pin.position.z = 0.45;
  g.add(pin);
  const lbl = makeLabel(label, "#" + color.toString(16).padStart(6, "0"));
  lbl.position.z = 1.0;
  g.add(lbl);
  g.visible = false;
  return g;
}

export class Twin3D {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f1419);
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.1, 500);
    this.camera.up.set(0, 1, 0);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.05;

    // ROS(z-up) → three(y-up)
    this.root = new THREE.Group();
    this.root.rotation.x = -Math.PI / 2;
    this.scene.add(this.root);

    this.scene.add(new THREE.HemisphereLight(0xdfe8f2, 0x1a222c, 1.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(-10, 25, 10);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -30, right: 30, top: 30, bottom: -30 });
    this.scene.add(sun);

    this.worldGroup = new THREE.Group();
    this.root.add(this.worldGroup);

    const { robot, body, legs, ring } = buildRobot();
    this.robot = robot;
    this.body = body;
    this.legs = legs;
    this.ring = ring;
    this.root.add(robot);

    this.trailPts = [];
    this.trailGeom = new THREE.BufferGeometry();
    this.trailGeom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * 2000), 3));
    this.trailGeom.setDrawRange(0, 0);
    this.root.add(new THREE.Line(this.trailGeom, new THREE.LineBasicMaterial({ color: COLORS.trail })));

    this.goals = {
      patrol: goalMarker(COLORS.goal, "patrol goal"),
      rth: goalMarker(COLORS.rth, "return home"),
      seek: goalMarker(COLORS.seek, "gas source?"),
    };
    Object.values(this.goals).forEach((g) => this.root.add(g));

    this.speed = 0;
    this.phase = 0;
    this.follow = false;
    this.clock = new THREE.Clock();

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => this.tick());
  }

  setWorld(name) {
    const w = WORLDS[name];
    this.world = w;
    this.worldName = name;
    this.worldGroup.clear();
    const [L, W, H] = w.size;

    const floor = new THREE.Mesh(
      new THREE.BoxGeometry(L, W, 0.1),
      new THREE.MeshStandardMaterial({ color: COLORS.floor, roughness: 0.9 })
    );
    floor.position.z = -0.05;
    floor.receiveShadow = true;
    this.worldGroup.add(floor);

    // 바닥 크기에 맞춘 1 m 격자
    const gp = [];
    for (let x = Math.ceil(-L / 2); x <= L / 2; x++) gp.push(x, -W / 2, 0, x, W / 2, 0);
    for (let y = Math.ceil(-W / 2); y <= W / 2; y++) gp.push(-L / 2, y, 0, L / 2, y, 0);
    const gridGeom = new THREE.BufferGeometry();
    gridGeom.setAttribute("position", new THREE.Float32BufferAttribute(gp, 3));
    const grid = new THREE.LineSegments(
      gridGeom,
      new THREE.LineBasicMaterial({ color: COLORS.grid, transparent: true, opacity: 0.6 })
    );
    grid.position.z = 0.002;
    this.worldGroup.add(grid);

    // 벽: 반투명 + 위쪽 시야 확보를 위해 높이 절반만 진하게
    const wallMat = new THREE.MeshStandardMaterial({ color: COLORS.wall, transparent: true, opacity: 0.35 });
    const walls = [
      [L, 0.1, 0, W / 2 + 0.05],
      [L, 0.1, 0, -W / 2 - 0.05],
      [0.1, W + 0.2, L / 2 + 0.05, 0],
      [0.1, W + 0.2, -L / 2 - 0.05, 0],
    ];
    for (const [sx, sy, x, y] of walls) {
      const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, H), wallMat);
      m.position.set(x, y, H / 2);
      this.worldGroup.add(m);
    }

    for (const o of w.obstacles) {
      const m = new THREE.Mesh(
        new THREE.BoxGeometry(0.6, 0.4, 0.8),
        new THREE.MeshStandardMaterial({ color: COLORS.obstacle })
      );
      m.position.set(o.x, o.y, 0.4);
      m.rotation.z = o.yaw;
      m.castShadow = m.receiveShadow = true;
      this.worldGroup.add(m);
    }

    for (const t of w.tanks) {
      const m = new THREE.Mesh(
        new THREE.CylinderGeometry(0.35, 0.35, 1.5, 32),
        new THREE.MeshStandardMaterial({ color: COLORS.tank, metalness: 0.4, roughness: 0.4 })
      );
      m.rotation.x = Math.PI / 2;
      m.position.set(t.x, t.y, 0.75);
      m.castShadow = true;
      this.worldGroup.add(m);
    }

    this.gaugeMeshes = [];
    for (const gdef of w.gauges) {
      const g = new THREE.Mesh(
        new THREE.BoxGeometry(0.35, 0.04, 0.35),
        new THREE.MeshStandardMaterial({ color: COLORS.gauge, emissive: 0x000000 })
      );
      g.position.set(gdef.x, gdef.y, gdef.z);
      this.worldGroup.add(g);
      const lbl = makeLabel(gdef.name, "#8b9aab");
      lbl.position.set(gdef.x, gdef.y - 0.3, gdef.z + 0.6);
      this.worldGroup.add(lbl);
      this.gaugeMeshes.push({ def: gdef, mesh: g });
    }

    if (w.plume) {
      const p = new THREE.Mesh(
        new THREE.PlaneGeometry(L, W),
        new THREE.MeshBasicMaterial({ map: plumeTexture(w), transparent: true, depthWrite: false })
      );
      p.position.z = 0.01;
      this.worldGroup.add(p);
    }

    // 웨이포인트 + 순찰 경로
    const wpMat = new THREE.MeshBasicMaterial({ color: COLORS.waypoint });
    const pts = [];
    for (const wp of w.waypoints) {
      const d = new THREE.Mesh(new THREE.CircleGeometry(0.2, 24), wpMat);
      d.position.set(wp.x, wp.y, 0.015);
      this.worldGroup.add(d);
      const lbl = makeLabel(wp.name, "#8b9aab");
      lbl.position.set(wp.x, wp.y, 0.6);
      lbl.scale.multiplyScalar(0.7);
      this.worldGroup.add(lbl);
      pts.push(new THREE.Vector3(wp.x, wp.y, 0.02));
    }
    pts.push(pts[0].clone());
    const route = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineDashedMaterial({ color: COLORS.waypoint, dashSize: 0.4, gapSize: 0.3 })
    );
    route.computeLineDistances();
    this.worldGroup.add(route);

    this.clearTrail();
    Object.values(this.goals).forEach((g) => (g.visible = false));
    const home = w.waypoints[0];
    this.setPose(home.x, home.y, 0, 0);
    this.resetView();
  }

  resetView() {
    const [cx, cy, cz] = this.world.camera;
    // ROS (x,y,z) → three (x, z, -y)
    this.camera.position.set(cx, cz, -cy);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
  }

  setPose(x, y, z, yaw) {
    this.robot.position.set(x, y, Math.max(0, z - 0.4));
    this.robot.rotation.z = yaw;
    const last = this.trailPts[this.trailPts.length - 1];
    if (!last || Math.hypot(last[0] - x, last[1] - y) > 0.05) {
      if (this.trailPts.length >= 2000) this.trailPts.shift();
      this.trailPts.push([x, y]);
      const arr = this.trailGeom.attributes.position.array;
      this.trailPts.forEach(([px, py], i) => arr.set([px, py, 0.03], i * 3));
      this.trailGeom.attributes.position.needsUpdate = true;
      this.trailGeom.setDrawRange(0, this.trailPts.length);
    }
  }

  clearTrail() {
    this.trailPts = [];
    this.trailGeom.setDrawRange(0, 0);
  }

  setSpeed(v) {
    this.speed = Math.abs(v);
  }

  setGoal(kind, x, y) {
    const g = this.goals[kind];
    g.position.set(x, y, 0);
    g.visible = true;
  }

  setGas(v) {
    const c = v > 50 ? 0xc4453c : v > 20 ? 0xd4a017 : 0x3dba84;
    this.ring.material.color.setHex(c);
    const s = 1 + Math.min(v, 100) / 50;
    this.ring.scale.set(s, s, 1);
  }

  // 가장 가까운 게이지를 판독 중으로 강조
  setGaugeReading(active) {
    for (const g of this.gaugeMeshes || []) {
      const d = Math.hypot(g.def.x - this.robot.position.x, g.def.y - this.robot.position.y);
      g.mesh.material.emissive.setHex(active && d < 3 ? 0x3dba84 : 0x000000);
    }
  }

  setThermalAlert(on) {
    this.body.material.emissive.setHex(on ? 0xc4453c : 0x000000);
    this.body.material.emissiveIntensity = on ? 0.8 : 0;
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  tick() {
    const dt = this.clock.getDelta();
    // 속도에 비례한 단순 트롯 애니메이션 (대각 다리 쌍 위상 반대)
    this.phase += dt * (2 + this.speed * 10);
    const amp = Math.min(0.5, this.speed * 1.2);
    this.legs.forEach((leg, i) => {
      const s = i === 0 || i === 3 ? 1 : -1;
      leg.rotation.y = s * amp * Math.sin(this.phase);
    });
    this.ring.rotation.z += dt;
    Object.values(this.goals).forEach((g) => {
      if (g.visible) g.children[0].position.z = 0.45 + 0.08 * Math.sin(this.phase * 0.5);
    });

    if (this.follow) {
      const p = new THREE.Vector3();
      this.robot.getWorldPosition(p);
      const delta = p.clone().sub(this.controls.target);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
