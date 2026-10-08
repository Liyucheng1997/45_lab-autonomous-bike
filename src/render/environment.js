// 场景：大气散射天空 + 环境光照 + 草地 + 环形道路（标线/路缘/斑马线）+ 障碍物模型。
import * as THREE from 'three';
import { Sky } from 'three/examples/jsm/objects/Sky.js';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { M, tex, carPaint, facadeMat } from './materials.js';
import { roadCenterline, ROAD } from '../core/world.js';
import { V, merge } from './geomUtils.js';

export function buildSky(scene, renderer) {
  const sky = new Sky();
  sky.scale.setScalar(4000);
  const u = sky.material.uniforms;
  u.turbidity.value = 5.5;
  u.rayleigh.value = 1.2;
  u.mieCoefficient.value = 0.004;
  u.mieDirectionalG.value = 0.82;
  const sun = new THREE.Vector3().setFromSphericalCoords(1, THREE.MathUtils.degToRad(90 - 36), THREE.MathUtils.degToRad(215));
  u.sunPosition.value.copy(sun);
  scene.add(sky);
  // 用天空生成 PMREM 环境贴图 → 金属/漆面反射真实天空
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envScene = new THREE.Scene();
  const sky2 = sky.clone();
  envScene.add(sky2);
  const ground = new THREE.Mesh(new THREE.CircleGeometry(3000, 16).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0x6f6f6a }));
  ground.position.y = -10;
  envScene.add(ground);
  scene.environment = pmrem.fromScene(envScene, 0.02).texture;
  scene.environmentIntensity = 0.85;
  return sun;
}

export function buildLights(scene, sunDir) {
  const hemi = new THREE.HemisphereLight(0xcfe3ff, 0x8a8478, 0.55);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xfff1dc, 3.2);
  sun.position.copy(sunDir).multiplyScalar(40);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const S = 14;
  Object.assign(sun.shadow.camera, { left: -S, right: S, top: S, bottom: -S, near: 1, far: 120 });
  sun.shadow.bias = -0.00015;
  sun.shadow.normalBias = 0.015;
  sun.shadow.radius = 3;
  scene.add(sun);
  scene.add(sun.target);
  return { sun, hemi, sunDir: sunDir.clone() };
}

// 沿闭合中心线生成带状网格：offset ∈ [o0, o1]，高度 h；withSides 生成侧面（路缘石）
function ribbon(center, o0, o1, h = 0, { withSides = false, uvScale = 6, dash = null } = {}) {
  const n = center.length;
  const pos = [], uv = [], idx = [];
  const normals = center.map((p, i) => {
    const a = center[(i - 1 + n) % n], b = center[(i + 1) % n];
    const tx = b[0] - a[0], ty = b[1] - a[1], L = Math.hypot(tx, ty);
    return [-ty / L, tx / L];
  });
  let s = 0;
  const S = [0];
  for (let i = 1; i <= n; i++) { const a = center[i - 1], b = center[i % n]; s += Math.hypot(b[0] - a[0], b[1] - a[1]); S.push(s); }
  const quads = [];
  for (let i = 0; i < n; i++) {
    if (dash) {
      const m = S[i] % (dash[0] + dash[1]);
      if (m > dash[0]) continue;
    }
    quads.push(i);
  }
  const pushV = (x, y, z, u, v) => { pos.push(x, y, z); uv.push(u, v); return pos.length / 3 - 1; };
  for (const i of quads) {
    const j = (i + 1) % n;
    const p = center[i], q = center[j], np = normals[i], nq = normals[j];
    const a = pushV(p[0] + np[0] * o0, h, p[1] + np[1] * o0, S[i] / uvScale, 0);
    const b = pushV(p[0] + np[0] * o1, h, p[1] + np[1] * o1, S[i] / uvScale, (o1 - o0) / uvScale);
    const c = pushV(q[0] + nq[0] * o1, h, q[1] + nq[1] * o1, S[i + 1] / uvScale, (o1 - o0) / uvScale);
    const d = pushV(q[0] + nq[0] * o0, h, q[1] + nq[1] * o0, S[i + 1] / uvScale, 0);
    idx.push(a, b, d, b, c, d);
    if (withSides) {
      for (const o of [o0, o1]) {
        const e = pushV(p[0] + np[0] * o, 0, p[1] + np[1] * o, S[i] / uvScale, 0);
        const f = pushV(p[0] + np[0] * o, h, p[1] + np[1] * o, S[i] / uvScale, h);
        const g = pushV(q[0] + nq[0] * o, h, q[1] + nq[1] * o, S[i + 1] / uvScale, h);
        const k = pushV(q[0] + nq[0] * o, 0, q[1] + nq[1] * o, S[i + 1] / uvScale, 0);
        o === o0 ? idx.push(e, f, k, f, g, k) : idx.push(e, k, f, f, k, g);
      }
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

export function buildGround(scene) {
  const grass = new THREE.Mesh(new THREE.PlaneGeometry(600, 600).rotateX(-Math.PI / 2), M.grass);
  grass.material.map.repeat.set(120, 120);
  grass.receiveShadow = true;
  grass.name = 'ground';
  scene.add(grass);

  const c = roadCenterline(0.5);
  const hw = ROAD.width / 2;
  const road = new THREE.Mesh(ribbon(c, -hw, hw, 0.012, { uvScale: 6 }), M.asphalt);
  road.receiveShadow = true;
  scene.add(road);
  // 路缘石
  for (const [o0, o1] of [[-hw - 0.2, -hw], [hw, hw + 0.2]]) {
    const curb = new THREE.Mesh(ribbon(c, o0, o1, 0.1, { withSides: true }), M.curb);
    curb.receiveShadow = true; curb.castShadow = true;
    scene.add(curb);
  }
  // 标线：两侧白实线、中心黄色虚线
  for (const o of [-hw + 0.25, hw - 0.25]) scene.add(new THREE.Mesh(ribbon(c, o - 0.06, o + 0.06, 0.016), M.paintWhite));
  scene.add(new THREE.Mesh(ribbon(c, -0.07, 0.07, 0.016, { dash: [3, 3] }), M.paintYellow));
  // 斑马线（行人横穿处）
  const zebra = [];
  for (const [x, y] of [[-22, -20], [22, 20]]) {
    for (let k = -5; k <= 5; k++) {
      const b = new THREE.BoxGeometry(0.45, 0.004, 3.2);
      b.translate(x + k * 0.9 * 0 + 0, 0.016, y);
      // 条纹沿道路方向排列（道路沿 x），每条横跨车道
      const g2 = new THREE.BoxGeometry(2.4, 0.004, 0.45);
      g2.translate(x, 0.016, y + k * 0.55);
      zebra.push(g2);
    }
  }
  scene.add(new THREE.Mesh(merge(zebra), M.paintWhite));
  // 停止线前的“慢”字/让行三角省略
  return { grass, road };
}

// ---------------- 障碍物模型 ----------------
export function buildObstacleMeshes(scene, obstacles) {
  const meshes = new Map();
  const rnd = mulberry(5);
  for (const o of obstacles) {
    let g;
    switch (o.kind) {
      case 'cone': g = cone(); break;
      case 'car': g = car(o, false); break;
      case 'van': g = car(o, true); break;
      case 'barrier': g = barrier(o); break;
      case 'crate': g = crate(o); break;
      case 'ped': g = pedestrian(o, rnd); break;
      case 'building': g = building(o); break;
      case 'tree': g = tree(o, rnd); break;
      case 'lamp': g = lamp(o); break;
      default: continue;
    }
    g.position.set(o.x, 0, o.y);
    if (o.shape === 'box') g.rotation.y = -o.yaw;
    if (o.kind === 'lamp') g.rotation.y = -o.face;
    g.traverse((m) => { if (m.isMesh) { m.castShadow = o.kind !== 'building' || true; m.receiveShadow = true; } });
    scene.add(g);
    meshes.set(o, g);
  }
  return meshes;
}

export function addObstacleMesh(scene, meshes, o) {
  const g = o.kind === 'crate' ? crate(o) : cone();
  g.position.set(o.x, 0, o.y);
  if (o.shape === 'box') g.rotation.y = -o.yaw;
  g.traverse((m) => { if (m.isMesh) { m.castShadow = true; m.receiveShadow = true; } });
  scene.add(g);
  meshes.set(o, g);
}

export function updateObstacleMeshes(meshes, t) {
  for (const [o, g] of meshes) {
    if (!o.dyn) continue;
    g.position.set(o.x, 0, o.y);
    g.rotation.y = -o.heading + Math.PI / 2;
    const ph = o.walking ? Math.sin(t * 7.5) : 0;
    const L = g.userData.limbs;
    if (L) {
      L.legL.rotation.z = 0.55 * ph; L.legR.rotation.z = -0.55 * ph;
      L.armL.rotation.z = -0.45 * ph; L.armR.rotation.z = 0.45 * ph;
      L.body.position.y = o.walking ? Math.abs(Math.cos(t * 7.5)) * 0.025 : 0;
    }
  }
}

function cone() {
  const g = new THREE.Group();
  const prof = [[0.0, 0.7], [0.03, 0.7], [0.035, 0.68], [0.15, 0.06], [0.16, 0.04], [0.0, 0.04]].map(([r, y]) => new THREE.Vector2(r, y));
  g.add(new THREE.Mesh(new THREE.LatheGeometry(prof, 32), M.coneOrange));
  for (const [y0, y1] of [[0.42, 0.5], [0.24, 0.31]]) {
    const r0 = 0.15 - ((y0 - 0.06) / 0.62) * 0.115, r1 = 0.15 - ((y1 - 0.06) / 0.62) * 0.115;
    g.add(new THREE.Mesh(new THREE.CylinderGeometry(r1 + 0.002, r0 + 0.002, y1 - y0, 32, 1, true).translate(0, (y0 + y1) / 2, 0), M.reflective));
  }
  g.add(new THREE.Mesh(new RoundedBoxGeometry(0.38, 0.04, 0.38, 2, 0.01).translate(0, 0.02, 0), M.rubber));
  return g;
}

function car(o, van) {
  const g = new THREE.Group();
  const paint = carPaint(o.color);
  const L = o.lx, W = o.ly, H = o.h;
  if (!van) {
    g.add(new THREE.Mesh(new RoundedBoxGeometry(L, 0.62, W, 4, 0.16).translate(0, 0.55, 0), paint));
    const cabin = new THREE.Mesh(new RoundedBoxGeometry(L * 0.52, 0.5, W * 0.88, 4, 0.14).translate(-L * 0.05, 1.05, 0), M.carGlass);
    g.add(cabin);
    g.add(new THREE.Mesh(new RoundedBoxGeometry(L * 0.46, 0.06, W * 0.84, 2, 0.03).translate(-L * 0.06, 1.3, 0), paint));
    for (const s of [-1, 1]) g.add(new THREE.Mesh(new RoundedBoxGeometry(0.1, 0.42, 0.06, 2, 0.02).translate(-L * 0.05 + s * L * 0.08, 1.04, 0).scale(1, 1, 1), paint));
  } else {
    g.add(new THREE.Mesh(new RoundedBoxGeometry(L, H - 0.35, W, 4, 0.18).translate(0, (H - 0.35) / 2 + 0.35, 0), paint));
    g.add(new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.7, W * 0.86, 2, 0.03).translate(L / 2 - 0.02, H - 0.65, 0), M.carGlass));
    for (const s of [-1, 1]) g.add(new THREE.Mesh(new THREE.BoxGeometry(L * 0.18, 0.5, 0.02).translate(L * 0.3, H - 0.6, s * (W / 2 + 0.002)), M.carGlass));
  }
  // 车轮
  const wheel = new THREE.CylinderGeometry(0.33, 0.33, 0.24, 28).rotateX(Math.PI / 2);
  const rim = new THREE.CylinderGeometry(0.2, 0.2, 0.25, 20).rotateX(Math.PI / 2);
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    const w = new THREE.Mesh(wheel, M.rubber);
    w.position.set(sx * L * 0.32, 0.33, sz * (W / 2 - 0.1));
    g.add(w);
    const r = new THREE.Mesh(rim, M.alu);
    r.position.copy(w.position);
    g.add(r);
  }
  // 车灯
  for (const sz of [-1, 1]) {
    g.add(new THREE.Mesh(new RoundedBoxGeometry(0.05, 0.1, 0.3, 2, 0.02).translate(L / 2, van ? 0.75 : 0.65, sz * W * 0.32), M.headlight));
    g.add(new THREE.Mesh(new RoundedBoxGeometry(0.05, 0.1, 0.25, 2, 0.02).translate(-L / 2, van ? 0.9 : 0.68, sz * W * 0.34), M.tailLight));
  }
  return g;
}

function barrier(o) {
  const g = new THREE.Group();
  const s = new THREE.Shape();
  const w = 0.2, h = o.h;
  s.moveTo(-w, 0); s.lineTo(w, 0); s.lineTo(w * 0.55, h * 0.25); s.lineTo(w * 0.3, h); s.lineTo(-w * 0.3, h); s.lineTo(-w * 0.55, h * 0.25); s.closePath();
  const seg = o.ly / 2;
  ['#d81f26', '#f4f4f4'].forEach((c, i) => {
    const geo = new THREE.ExtrudeGeometry(s, { depth: seg - 0.02, bevelEnabled: true, bevelSize: 0.015, bevelThickness: 0.015, bevelSegments: 2 });
    geo.translate(0, 0, -o.ly / 2 + i * seg + 0.01);
    g.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: c, roughness: 0.45 })));
  });
  return g;
}

function crate(o) {
  const g = new THREE.Group();
  g.add(new THREE.Mesh(new THREE.BoxGeometry(o.lx, o.h, o.ly).translate(0, o.h / 2, 0), M.cardboard));
  return g;
}

function pedestrian(o, rnd) {
  const g = new THREE.Group();
  const body = new THREE.Group();
  g.add(body);
  const H = o.h;
  const shirt = new THREE.MeshStandardMaterial({ color: [0x3b6fd8, 0xd8443b, 0x2f9e6e, 0xe0a030][Math.floor(rnd() * 4)], roughness: 0.8 });
  const pants = new THREE.MeshStandardMaterial({ color: [0x2a2f3a, 0x4a3c2c, 0x1f2a44][Math.floor(rnd() * 3)], roughness: 0.85 });
  const hip = H * 0.52, sh = H * 0.82;
  body.add(new THREE.Mesh(new THREE.CapsuleGeometry(0.16, sh - hip - 0.12, 6, 14).scale(1, 1, 0.65).translate(0, (hip + sh) / 2, 0), shirt));
  body.add(new THREE.Mesh(new THREE.SphereGeometry(0.105, 20, 14).translate(0, sh + 0.13, 0), M.skin));
  body.add(new THREE.Mesh(new THREE.SphereGeometry(0.11, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2).translate(0, sh + 0.15, -0.005), new THREE.MeshStandardMaterial({ color: 0x2a1d14, roughness: 0.9 })));
  const limb = (len, r, mat) => {
    const piv = new THREE.Group();
    piv.add(new THREE.Mesh(new THREE.CapsuleGeometry(r, len - 2 * r, 4, 10).translate(0, -len / 2, 0), mat));
    return piv;
  };
  const legL = limb(hip, 0.07, pants), legR = limb(hip, 0.07, pants);
  legL.position.set(0, hip, -0.085); legR.position.set(0, hip, 0.085);
  const armL = limb(sh - hip + 0.05, 0.05, shirt), armR = limb(sh - hip + 0.05, 0.05, shirt);
  armL.position.set(0, sh - 0.02, -0.2); armR.position.set(0, sh - 0.02, 0.2);
  body.add(legL, legR, armL, armR);
  g.userData.limbs = { legL, legR, armL, armR, body };
  // 面向 +x 行走：把身体转 90°（模型正面朝 +x）
  body.rotation.y = -Math.PI / 2;
  return g;
}

function building(o) {
  const g = new THREE.Group();
  const side = facadeMat(o.color);
  side.map = tex.facade.clone();
  side.map.needsUpdate = true;
  side.map.repeat.set(Math.max(1, Math.round(Math.max(o.lx, o.ly) / 4)), Math.max(1, Math.round(o.h / 3.4)));
  const roof = new THREE.MeshStandardMaterial({ color: 0x6d6a66, roughness: 0.9 });
  g.add(new THREE.Mesh(new THREE.BoxGeometry(o.lx, o.h, o.ly).translate(0, o.h / 2, 0), [side, side, roof, roof, side, side]));
  g.add(new THREE.Mesh(new THREE.BoxGeometry(o.lx + 0.3, 0.3, o.ly + 0.3).translate(0, o.h + 0.15, 0), roof));
  return g;
}

function tree(o, rnd) {
  const g = new THREE.Group();
  const th = o.h * 0.55;
  g.add(new THREE.Mesh(new THREE.CylinderGeometry(o.r * 0.6, o.r, th + 0.4, 10).translate(0, (th + 0.4) / 2, 0), M.trunk));
  const cy = o.h - o.crown * 0.6;
  const n = 3 + Math.floor(rnd() * 3);
  for (let i = 0; i < n; i++) {
    const r = o.crown * (0.55 + rnd() * 0.45);
    const m = new THREE.Mesh(new THREE.IcosahedronGeometry(r, 1), M.leaves[Math.floor(rnd() * M.leaves.length)]);
    m.position.set((rnd() - 0.5) * o.crown * 0.8, cy + (rnd() - 0.3) * o.crown * 0.6, (rnd() - 0.5) * o.crown * 0.8);
    m.rotation.set(rnd() * 3, rnd() * 3, 0);
    g.add(m);
  }
  return g;
}

function lamp(o) {
  const g = new THREE.Group();
  g.add(new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.09, o.h, 12).translate(0, o.h / 2, 0), M.darkSteel));
  const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 1.4, 8).rotateX(Math.PI / 2).translate(0, o.h - 0.1, 0.65), M.darkSteel);
  g.add(arm);
  g.add(new THREE.Mesh(new RoundedBoxGeometry(0.3, 0.1, 0.6, 2, 0.03).translate(0, o.h - 0.15, 1.3), M.lampHead));
  return g;
}

function mulberry(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
