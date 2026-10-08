// 自动驾驶自行车三维模型 —— 全部程序化建模，几何由 core/geometry.js 驱动（与动力学同源）。
//
// 层级：
//   root (地面位置 + 航向)
//   └─ rollGroup (横滚，绕后轮触地线)
//      ├─ 车架 / 后轮 / 传动 / 座垫 / 货架 / 电子设备（静止于车架）
//      ├─ rwSpin（动量轮飞轮，绕 x 轴自转）
//      ├─ steerPivot（位于转向轴，倾角 λ）→ steerRot（绕转向轴转 δ）→ 前叉/前轮/车把/大带轮
//      └─ lidarHead（激光雷达旋转头）
// 局部坐标：x 前 / y 上 / z 右，原点为后轮触地点。

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { M } from './materials.js';
import { V, sweep, tube, curveTube, loft, superRing, gearShape, circlePath, beltLoop, merge, alignY, xform } from './geomUtils.js';

const P = (xy, z = 0) => V(xy[0], xy[1], z);

export class BikeModel {
  constructor(sim) {
    this.sim = sim;
    this.root = new THREE.Group();
    this.rollGroup = new THREE.Group();
    this.root.add(this.rollGroup);
    this.labelAnchors = {};
    this.build();
  }

  add(geo, mat, parent = this.frame, shadow = true) {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = shadow;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  }

  build() {
    const G = (this.G = this.sim.plant.body.G);
    const p = this.sim.p;
    this.frame = new THREE.Group();
    this.rollGroup.add(this.frame);

    this.buildFrame(G);
    this.rearWheel = this.buildWheel(G, true);
    this.rearWheel.position.copy(P(G.rearHub));
    this.rollGroup.add(this.rearWheel);
    this.buildDrivetrain(G);
    this.buildSaddle(G);
    this.buildRack(G);
    this.buildFront(G);
    this.buildReactionWheel(G, p);
    this.buildSteerActuator(G);
    this.buildSensors(G);
    this.buildElectronics(G);
    this.buildCables(G);
  }

  // ---------------------------------------------------------------- 车架
  buildFrame(G) {
    const R = G.R;
    const add = (g, m = M.frame) => this.add(g, m);
    // 下管：内置电池 → 大截面椭圆管，两端收细
    const dtCurve = new THREE.CatmullRomCurve3([P(G.bb, 0).add(V(0.005, 0.012, 0)), P([(G.bb[0] + G.dtFront[0]) / 2 + 0.008, (G.bb[1] + G.dtFront[1]) / 2 - 0.006]), P(G.dtFront)]);
    add(sweep(dtCurve, { segments: 40, radial: 24, radius: (t) => {
      const k = Math.sin(Math.PI * Math.min(1, Math.max(0, (t - 0.02) / 0.96)));
      return [0.024 + 0.008 * k ** 0.4, 0.028 + 0.016 * k ** 0.5];
    } }));
    this.dtCurve = dtCurve;
    // 上管
    add(sweep(new THREE.CatmullRomCurve3([P(G.ttRear), P(G.ttFront)]), { segments: 8, radial: 20, radius: (t) => [0.018 + 0.002 * t, 0.02 + 0.003 * t] }));
    // 座管（座管顶略高出上管，带座管夹）
    const stTop = P(G.seatTop).add(V(-0.008, 0.028, 0));
    add(sweep(new THREE.CatmullRomCurve3([P(G.bb), stTop]), { segments: 8, radial: 20, radius: (t) => { const r = 0.0175 + 0.007 * (1 - t) ** 4; return [r, r]; } }));
    const collar = new THREE.CylinderGeometry(0.0205, 0.0205, 0.022, 20);
    alignY(collar, stTop.clone().add(V(0.004, -0.012, 0)), stTop.clone().add(V(-0.003, 0.012, 0)));
    this.add(collar, M.anodBlack);
    // 头管：锥形（下 1.5" / 上 1-1/8"）
    const hb = P(G.axisAtY(G.headBot[1] - 0.012)), ht = P(G.axisAtY(G.headTop[1] + 0.006));
    add(sweep(new THREE.LineCurve3(hb, ht), { segments: 4, radial: 24, radius: (t) => { const r = 0.032 - 0.006 * t; return [r, r]; } }));
    // 碗组上下碗
    for (const [y, r] of [[G.headBot[1] - 0.02, 0.031], [G.headTop[1] + 0.012, 0.025]]) {
      const c = new THREE.CylinderGeometry(r, r, 0.012, 24);
      alignY(c, P(G.axisAtY(y - 0.006)), P(G.axisAtY(y + 0.006)));
      this.add(c, M.anodBlack);
    }
    // 五通
    const bbShell = new THREE.CylinderGeometry(0.023, 0.023, 0.073, 24);
    bbShell.rotateX(Math.PI / 2);
    bbShell.translate(G.bb[0], G.bb[1], 0);
    add(bbShell);
    // 后下叉 / 后上叉（左右）
    const ssTop = P(G.seatTop).add(V(0.02, -0.06, 0));
    for (const s of [-1, 1]) {
      const cs = curveTube([P(G.bb, s * 0.03), P([G.bb[0] - 0.12, G.bb[1] + 0.012], s * 0.058), P([0.09, R - 0.012], s * 0.066), P(G.rearHub, s * 0.068)],
        (t) => { const r = 0.0125 - 0.0035 * t; return [r * 0.85, r]; }, { segments: 30 });
      add(cs);
      const ss = curveTube([ssTop.clone().setZ(s * 0.016), P(lerp(G.seatTop, G.rearHub, 0.45), s * 0.05), P(G.rearHub, s * 0.067).add(V(0.012, 0.02, 0))],
        (t) => { const r = 0.0095 - 0.002 * t; return [r, r]; }, { segments: 30 });
      add(ss);
      // 后叉勾片
      const drop = new RoundedBoxGeometry(0.05, 0.046, 0.007, 2, 0.006);
      drop.translate(0.008, R + 0.006, s * 0.068);
      this.add(drop, M.frame);
    }
    // 后轴（贯穿轴）+ 拉杆
    const axle = new THREE.CylinderGeometry(0.0075, 0.0075, 0.165, 12);
    axle.rotateX(Math.PI / 2);
    axle.translate(0, R, 0);
    this.add(axle, M.anodBlack);
    // 后刹车卡钳（左侧，后下叉上方）
    this.addCaliper(this.frame, P(G.rearHub, -0.052), 2.35);
    // 下管电池盒 + 电量灯 + 充电口
    const dA = P(lerp(G.bb, G.dtFront, 0.2)), dB = P(lerp(G.bb, G.dtFront, 0.88));
    const dir = dB.clone().sub(dA);
    const L = dir.length();
    dir.normalize();
    const nrm = V(dir.y, -dir.x, 0); // 指向下管下方
    const batt = new RoundedBoxGeometry(L, 0.05, 0.058, 4, 0.012);
    const bm = this.add(batt, M.plastic);
    bm.position.copy(dA.clone().add(dB).multiplyScalar(0.5).addScaledVector(nrm, 0.024));
    bm.rotation.z = Math.atan2(dir.y, dir.x);
    for (let i = 0; i < 5; i++) {
      const led = this.add(new THREE.BoxGeometry(0.01, 0.004, 0.002), i < 4 ? M.ledGreen : M.plastic, bm, false);
      led.position.set(-0.05 + i * 0.014, 0.004, 0.0295);
    }
    const port = this.add(new THREE.CylinderGeometry(0.008, 0.008, 0.004, 16).rotateX(Math.PI / 2), M.anodGrey, bm, false);
    port.position.set(0.12, -0.004, 0.0295);
    const decal = this.textDecal('36V · 468Wh', 0.11, 0.016, '#c9ced6');
    decal.position.set(-0.07, -0.01, 0.0302);
    bm.add(decal);
    this.labelAnchors.battery = bm;
  }

  addCaliper(parent, center, angle) {
    // 卡钳跨在碟片边缘：位于半径 0.08 处
    const g = new THREE.Group();
    const body = new RoundedBoxGeometry(0.05, 0.026, 0.032, 3, 0.008);
    this.add(body, M.anodGrey, g);
    const pist = new THREE.CylinderGeometry(0.009, 0.009, 0.034, 14).rotateX(Math.PI / 2);
    this.add(pist, M.alu, g).position.set(0.01, -0.004, 0);
    g.position.copy(center).add(V(Math.cos(angle) * 0.072, Math.sin(angle) * 0.072, 0));
    g.rotation.z = angle + Math.PI / 2;
    parent.add(g);
    return g;
  }

  // ---------------------------------------------------------------- 车轮
  buildWheel(G, rear) {
    const R = G.R;
    const g = new THREE.Group();
    const spin = new THREE.Group();
    g.add(spin);
    g.userData.spin = spin;
    const lathe = (pts, segs = 72) => new THREE.LatheGeometry(pts.map(([r, y]) => new THREE.Vector2(r, y)), segs).rotateX(Math.PI / 2);
    // 外胎：胎面(黑色花纹) + 胎侧(棕色)
    const rc = R - 0.027;
    const prof = (a0, a1, n) => Array.from({ length: n + 1 }, (_, i) => { const a = a0 + ((a1 - a0) * i) / n; return [rc + 0.027 * Math.cos(a), 0.0275 * Math.sin(a)]; });
    this.add(lathe(prof(-0.85, 0.85, 14), 120), M.tire, spin);
    this.add(lathe(prof(0.85, 2.05, 8), 96), M.gumwall, spin);
    this.add(lathe(prof(-2.05, -0.85, 8), 96), M.gumwall, spin);
    // 轮圈（双层箱形截面）
    const rimPts = [[0.266, -0.008], [0.270, -0.0115], [0.287, -0.0128], [0.2945, -0.0128], [0.2945, -0.0108], [0.2905, -0.0098], [0.2905, 0.0098], [0.2945, 0.0108], [0.2945, 0.0128], [0.287, 0.0128], [0.270, 0.0115], [0.266, 0.008], [0.266, -0.008]]
      .map(([r, y]) => [r + (R - 0.34), y]);
    this.add(lathe(rimPts, 96), M.anodBlack, spin);
    // 气门嘴
    const valve = new THREE.CylinderGeometry(0.003, 0.003, 0.03, 8);
    valve.translate(0, R - 0.08, 0);
    this.add(valve, M.alu, spin);
    // 花鼓：后轮 = 轮毂电机（大鼓 + 散热筋）；前轮 = 普通花鼓
    let flangeR, flangeZ, N, cross;
    if (rear) {
      const hubPts = [[0.012, -0.04], [0.035, -0.04], [0.06, -0.036], [0.085, -0.03], [0.092, -0.022], [0.092, 0.022], [0.085, 0.03], [0.06, 0.036], [0.035, 0.04], [0.012, 0.04]];
      this.add(lathe(hubPts, 64), M.anodBlack, spin);
      const ribs = [];
      for (let i = 0; i < 16; i++) {
        for (const s of [-1, 1]) {
          const b = new THREE.BoxGeometry(0.05, 0.004, 0.003);
          b.translate(0.058, 0, s * 0.036);
          b.rotateZ((i / 16) * Math.PI * 2);
          ribs.push(b);
        }
      }
      this.add(merge(ribs), M.anodGrey, spin);
      for (const s of [-1, 1]) {
        const fl = new THREE.CylinderGeometry(0.098, 0.098, 0.004, 48).rotateX(Math.PI / 2);
        fl.translate(0, 0, s * 0.028);
        this.add(fl, M.alu, spin);
      }
      const ring = new THREE.TorusGeometry(0.093, 0.003, 8, 64);
      ring.translate(0, 0, 0.0);
      this.add(ring, M.accent, spin);
      flangeR = 0.092; flangeZ = 0.028; N = 36; cross = 1;
      // 单速飞轮（右侧，链线 48 mm）
      const cog = new THREE.ExtrudeGeometry(gearShape(18, 0.0335, 0.0385, { holes: [circlePath(0.012)] }), { depth: 0.003, bevelEnabled: false });
      cog.translate(0, 0, 0.0465);
      this.add(cog, M.steel, spin);
    } else {
      const hubPts = [[0.006, -0.05], [0.017, -0.05], [0.017, -0.036], [0.03, -0.034], [0.03, -0.03], [0.015, -0.026], [0.014, 0], [0.015, 0.026], [0.03, 0.03], [0.03, 0.034], [0.017, 0.036], [0.017, 0.05], [0.006, 0.05]];
      this.add(lathe(hubPts, 48), M.alu, spin);
      flangeR = 0.028; flangeZ = 0.032; N = 32; cross = 3;
    }
    // 辐条：交叉编法 (rear 1-cross / front 3-cross)，J 弯头在花鼓法兰
    const spokes = [], nipples = [];
    const rimHoleR = R - 0.072;
    for (let i = 0; i < N; i++) {
      const side = i % 2 ? 1 : -1;
      const rimA = (2 * Math.PI * i) / N;
      const dirn = Math.floor(i / 2) % 2 ? 1 : -1;
      const hubA = rimA + dirn * cross * ((4 * Math.PI) / N);
      const a = V(flangeR * Math.cos(hubA), flangeR * Math.sin(hubA), side * flangeZ);
      const b = V(rimHoleR * Math.cos(rimA), rimHoleR * Math.sin(rimA), side * 0.0035);
      const c2 = new THREE.CylinderGeometry(0.00105, 0.00105, a.distanceTo(b), 5, 1, true);
      alignY(c2, a, b);
      spokes.push(c2);
      const nip = new THREE.CylinderGeometry(0.0022, 0.0022, 0.013, 6);
      const d = b.clone().sub(a).normalize();
      alignY(nip, b.clone().addScaledVector(d, -0.013), b.clone().addScaledVector(d, 0.002));
      nipples.push(nip);
    }
    this.add(merge(spokes), M.steel, spin, false);
    this.add(merge(nipples), M.alu, spin, false);
    // 碟刹盘（左侧）
    const rotor = this.makeRotor();
    rotor.translate(0, 0, rear ? -0.047 : -0.044);
    this.add(rotor, M.steel, spin);
    return g;
  }

  makeRotor() {
    const r = 0.08;
    const s = new THREE.Shape();
    s.absarc(0, 0, r, 0, Math.PI * 2, false);
    s.holes.push(circlePath(0.023));
    // 5 个镂空（星形辐臂之间）
    for (let k = 0; k < 5; k++) {
      const a0 = (k / 5) * Math.PI * 2 + 0.22, a1 = a0 + (Math.PI * 2) / 5 - 0.44;
      const h = new THREE.Path();
      h.absarc(0, 0, 0.061, a0, a1, false);
      h.absarc(0, 0, 0.034, a1 - 0.12, a0 + 0.12, true);
      h.closePath();
      s.holes.push(h);
    }
    // 刹车面打孔
    for (let k = 0; k < 24; k++) {
      const a = (k / 24) * Math.PI * 2;
      const rr = k % 2 ? 0.068 : 0.074;
      s.holes.push(circlePath(0.0024, rr * Math.cos(a), rr * Math.sin(a), 8));
    }
    return new THREE.ExtrudeGeometry(s, { depth: 0.0019, bevelEnabled: false, curveSegments: 40 });
  }

  // ---------------------------------------------------------------- 传动：牙盘 / 曲柄 / 脚踏 / 链条
  buildDrivetrain(G) {
    const bb = G.bb, z = 0.048;
    const crank = new THREE.Group();
    crank.position.set(bb[0], bb[1], 0);
    this.frame.add(crank);
    const holes = [circlePath(0.03)];
    for (let k = 0; k < 4; k++) {
      const a = (k / 4) * Math.PI * 2 + Math.PI / 4;
      holes.push(circlePath(0.016, 0.055 * Math.cos(a), 0.055 * Math.sin(a), 16));
    }
    const ring = new THREE.ExtrudeGeometry(gearShape(42, 0.0815, 0.0875, { holes }), { depth: 0.003, bevelEnabled: false, curveSegments: 24 });
    ring.translate(0, 0, z - 0.0015);
    this.add(ring, M.anodBlack, crank);
    // 曲柄臂（锥形挤出），右臂朝前下，左臂相反
    const armShape = new THREE.Shape();
    armShape.absarc(0, 0, 0.02, Math.PI / 2, (3 * Math.PI) / 2, false);
    armShape.lineTo(0.17, -0.012);
    armShape.absarc(0.17, 0, 0.012, -Math.PI / 2, Math.PI / 2, false);
    armShape.lineTo(0, 0.02);
    const ang = -0.6;
    for (const s of [1, -1]) {
      const arm = new THREE.ExtrudeGeometry(armShape, { depth: 0.012, bevelEnabled: true, bevelSize: 0.003, bevelThickness: 0.003, bevelSegments: 2 });
      arm.translate(0, 0, -0.006);
      const m = this.add(arm, M.anodBlack, crank);
      m.rotation.z = s > 0 ? ang : ang + Math.PI;
      m.position.z = s * 0.068;
      // 脚踏
      const pedal = new THREE.Group();
      const ax = Math.cos(m.rotation.z) * 0.17, ay = Math.sin(m.rotation.z) * 0.17;
      pedal.position.set(ax, ay, s * 0.11);
      this.add(new RoundedBoxGeometry(0.1, 0.018, 0.09, 2, 0.006), M.plastic, pedal);
      const pins = [];
      for (let i = 0; i < 5; i++) for (const sy of [-1, 1]) {
        const pin = new THREE.CylinderGeometry(0.002, 0.002, 0.006, 6);
        pin.translate(-0.04 + i * 0.02, sy * 0.011, -0.03 + (i % 2) * 0.06);
        pins.push(pin);
      }
      this.add(merge(pins), M.alu, pedal, false);
      const spindle = new THREE.CylinderGeometry(0.006, 0.006, 0.05, 10).rotateX(Math.PI / 2);
      spindle.translate(0, 0, -s * 0.03);
      this.add(spindle, M.steel, pedal);
      crank.add(pedal);
    }
    const axleG = new THREE.CylinderGeometry(0.012, 0.012, 0.14, 16).rotateX(Math.PI / 2);
    this.add(axleG, M.alu, crank);
    // 链条：沿两轮外公切线回路，节距 12.7 mm，内外链节交替
    const links = beltLoop([bb[0], bb[1]], 0.0845, [G.rearHub[0], G.rearHub[1]], 0.0366, 0.0127);
    const plateG = new THREE.BoxGeometry(0.0165, 0.0068, 0.0011);
    const rollerG = new THREE.CylinderGeometry(0.0038, 0.0038, 0.0078, 8).rotateX(Math.PI / 2);
    const plates = new THREE.InstancedMesh(plateG, M.steel, links.length * 2);
    const plates2 = new THREE.InstancedMesh(plateG, M.darkSteel, links.length * 2);
    const rollers = new THREE.InstancedMesh(rollerG, M.darkSteel, links.length);
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler();
    let i1 = 0, i2 = 0;
    links.forEach((l, i) => {
      const a = Math.atan2(l.t[1], l.t[0]);
      q.setFromEuler(e.set(0, 0, a));
      const mid = V(l.p[0] + l.t[0] * 0.00635, l.p[1] + l.t[1] * 0.00635, z);
      for (const s of [-1, 1]) {
        const off = i % 2 ? 0.0048 : 0.0039;
        m4.compose(mid.clone().setZ(z + s * off), q, V(1, 1, 1));
        if (i % 2) plates.setMatrixAt(i1++, m4); else plates2.setMatrixAt(i2++, m4);
      }
      m4.compose(V(l.p[0], l.p[1], z), q, V(1, 1, 1));
      rollers.setMatrixAt(i, m4);
    });
    plates.count = i1; plates2.count = i2;
    for (const im of [plates, plates2, rollers]) { im.castShadow = true; this.frame.add(im); }
  }

  // ---------------------------------------------------------------- 座垫 + 座杆
  buildSaddle(G) {
    const sd = G.seatDir;
    const postTop = P(G.saddle).add(V(0, -0.035, 0));
    const postBot = P(G.seatTop).add(V(-sd[0] * 0.05, -sd[1] * 0.05, 0));
    const post = new THREE.CylinderGeometry(0.0136, 0.0136, 1, 20);
    alignY(post.scale(1, postTop.distanceTo(postBot), 1), postBot, postTop);
    this.add(post, M.anodBlack);
    const head = new RoundedBoxGeometry(0.05, 0.02, 0.034, 2, 0.006);
    this.add(head, M.anodBlack).position.copy(postTop).add(V(0, 0.006, 0));
    // 座垫：放样曲面（后宽前窄，中部下凹）
    const sections = [];
    const L = 0.27, n = 18;
    for (let i = 0; i <= n; i++) {
      const u = i / n; // 0 = 后, 1 = 鼻端
      const x = -0.12 + u * L;
      const hw = 0.016 + 0.058 * Math.pow(1 - u, 0.55) * (u < 0.06 ? Math.sqrt(u / 0.06) * 0.6 + 0.4 : 1);
      const hh = 0.012 + 0.014 * (1 - u * 0.4);
      const cy = 0.018 + 0.012 * (1 - u) ** 3 - 0.006 * Math.sin(Math.PI * u);
      sections.push(superRing(x, cy, hw, hh, 2.8, 28, 0.35));
    }
    const sad = this.add(loft(sections), M.leather);
    sad.position.copy(postTop).add(V(0.0, 0.0, 0));
    sad.rotation.z = -0.03;
    // 座垫导轨
    for (const s of [-1, 1]) {
      const rail = curveTube([V(-0.09, 0.008, s * 0.03), V(-0.05, -0.006, s * 0.022), V(0.04, -0.006, s * 0.022), V(0.11, 0.012, s * 0.012)], 0.0035, { segments: 16, radial: 8 });
      this.add(rail, M.steel, sad);
    }
  }

  // ---------------------------------------------------------------- 后货架
  buildRack(G) {
    const y = G.rack.y, x0 = G.rack.x0 - 0.02, x1 = G.rack.x1;
    const r = 0.0052, w = 0.068;
    const parts = [];
    for (const s of [-1, 1]) {
      parts.push(tube(V(x0, y, s * w), V(x1, y, s * w), r));
      // 前撑：接后上叉
      const ssPt = P(lerp(G.rearHub, G.seatTop, 0.62), s * 0.05);
      parts.push(curveTube([V(x1, y, s * w), V(x1 + 0.02, y - 0.02, s * w * 0.9), ssPt], r, { segments: 10, radial: 8 }));
      // 后腿：接后叉勾片
      parts.push(curveTube([V(x0 + 0.02, y, s * w), V(-0.12, y - 0.2, s * 0.074), V(-0.012, G.R + 0.02, s * 0.076)], r, { segments: 16, radial: 8 }));
      parts.push(tube(V(0.08, y, s * w), V(0.0, G.R + 0.03, s * 0.076), r));
    }
    for (const x of [x0, -0.1, 0.02, 0.14, x1]) parts.push(tube(V(x, y, -w), V(x, y, w), r));
    this.add(merge(parts), M.anodBlack);
    // 铝合金承载板
    const deck = new RoundedBoxGeometry(x1 - x0 + 0.01, 0.005, 2 * w + 0.016, 2, 0.002);
    this.add(deck, M.alu).position.set((x0 + x1) / 2, y + 0.006, 0);
    // 尾灯（刹车时变亮）
    const tl = new RoundedBoxGeometry(0.014, 0.022, 0.07, 2, 0.004);
    this.tailLight = this.add(tl, M.tailLight.clone(), this.frame, false);
    this.tailLight.position.set(x0 - 0.01, y - 0.015, 0);
  }

  // ---------------------------------------------------------------- 前叉 / 前轮 / 车把（随转向转动）
  buildFront(G) {
    const pivot = new THREE.Group();
    pivot.position.set(G.axisGround[0], 0, 0);
    pivot.rotation.z = G.lam; // 局部 y = 转向轴
    this.rollGroup.add(pivot);
    const rot = new THREE.Group();
    pivot.add(rot);
    this.steerRot = rot;
    // 内层：抵消 pivot 的变换，使子部件仍可用车身坐标建模
    const inner = new THREE.Group();
    inner.rotation.z = -G.lam;
    inner.position.copy(V(-G.axisGround[0], 0, 0).applyAxisAngle(V(0, 0, 1), -G.lam));
    rot.add(inner);
    const F = (this.front = inner);
    const addF = (g, m) => this.add(g, m, F);

    // 前轮
    this.frontWheel = this.buildWheel(G, false);
    this.frontWheel.position.copy(P(G.frontHub));
    F.add(this.frontWheel);
    // 前叉肩
    const crown = new RoundedBoxGeometry(0.034, 0.03, 0.13, 3, 0.01);
    const cm = addF(crown, M.frame);
    cm.position.copy(P(G.crown));
    cm.rotation.z = G.lam;
    // 舵管（头管上方露出部分）+ 垫圈
    const stA = P(G.axisAtY(G.headTop[1] + 0.012)), stB = P(G.steererTop).add(V(0, 0.015, 0));
    const st = new THREE.CylinderGeometry(0.0143, 0.0143, 1, 18);
    alignY(st.scale(1, stA.distanceTo(stB), 1), stA, stB);
    addF(st, M.anodBlack);
    // 前叉腿：从肩部沿转向轴下行，下端前弯形成 46 mm 偏移
    for (const s of [-1, 1]) {
      const top = P(G.crown, s * 0.052);
      const mid = P(G.axisAtY(G.frontHub[1] + 0.17), s * 0.055);
      const low = P(G.axisAtY(G.frontHub[1] + 0.06), s * 0.056).add(V(0.02, 0, 0));
      const end = P(G.frontHub, s * 0.056);
      addF(curveTube([top, mid, low, end], (t) => { const r = 0.0175 - 0.0065 * t; return [r * 0.9, r * 1.08]; }, { segments: 30, radial: 16 }), M.frame);
      const d = new RoundedBoxGeometry(0.03, 0.04, 0.008, 2, 0.005);
      addF(d, M.frame).position.copy(end);
    }
    const ax = new THREE.CylinderGeometry(0.0075, 0.0075, 0.15, 12).rotateX(Math.PI / 2);
    ax.translate(G.frontHub[0], G.frontHub[1], 0);
    addF(ax, M.anodBlack);
    const lever = new RoundedBoxGeometry(0.06, 0.008, 0.008, 2, 0.003);
    addF(lever, M.anodBlack).position.copy(P(G.frontHub, 0.08)).add(V(-0.02, 0.01, 0));
    this.addCaliper(F, P(G.frontHub, -0.049), 2.6);
    // 转向大带轮（45 齿，装在舵管上，随车把转）
    const big = new THREE.ExtrudeGeometry(gearShape(45, 0.0465, 0.0485, { holes: [circlePath(0.015)] }), { depth: 0.011, bevelEnabled: false, curveSegments: 12 });
    big.rotateX(-Math.PI / 2).translate(0, -0.0055, 0);
    const bigG = new THREE.Group();
    bigG.position.copy(P(G.pulley));
    bigG.rotation.z = G.lam;
    this.add(big, M.anodBlack, bigG);
    for (const s of [-1, 1]) {
      const flange = new THREE.CylinderGeometry(0.052, 0.052, 0.0012, 48);
      flange.translate(0, s * 0.0062, 0);
      this.add(flange, M.alu, bigG);
    }
    F.add(bigG);
    // 把立 + 车把
    const steererTop = P(G.steererTop);
    const bar = P(G.barCenter);
    const stem = new RoundedBoxGeometry(1, 0.034, 0.038, 3, 0.012);
    const sm = addF(stem, M.anodBlack);
    const sdir = bar.clone().sub(steererTop);
    sm.scale.x = sdir.length() + 0.03;
    sm.position.copy(steererTop.clone().add(bar).multiplyScalar(0.5));
    sm.rotation.z = Math.atan2(sdir.y, sdir.x);
    const cap = new THREE.CylinderGeometry(0.018, 0.018, 0.008, 20);
    alignY(cap, P(G.steererTop).add(V(0.003 * -G.sl, 0.012, 0)), P(G.steererTop).add(V(-0.008 * G.sl, 0.022, 0)));
    addF(cap, M.alu);
    // 车载状态屏（把立顶部）
    const scr = new RoundedBoxGeometry(0.05, 0.008, 0.036, 2, 0.003);
    const scrM = addF(scr, M.plastic);
    scrM.position.copy(sm.position).add(V(0, 0.022, 0));
    scrM.rotation.z = sm.rotation.z - 0.25;
    this.screen = new THREE.Mesh(new THREE.PlaneGeometry(0.042, 0.028), new THREE.MeshBasicMaterial({ color: 0x18c8ff }));
    this.screen.rotation.x = -Math.PI / 2;
    this.screen.rotation.z = Math.PI / 2;
    this.screen.position.y = 0.0045;
    scrM.add(this.screen);
    // 平把（略后掠、上扬），中段 31.8 mm
    const hw = G.barHalfWidth;
    const pts = [];
    for (let i = 0; i <= 12; i++) {
      const z = -hw + (2 * hw * i) / 12;
      const az = Math.abs(z);
      const back = az > 0.07 ? -0.03 * ((az - 0.07) / (hw - 0.07)) ** 1.2 : 0;
      const up = az > 0.06 ? 0.018 * ((az - 0.06) / (hw - 0.06)) : 0;
      pts.push(V(bar.x + back, bar.y + up, z));
    }
    const barGeo = curveTube(pts, (t) => { const z = Math.abs((t - 0.5) * 2 * hw); const r = z < 0.035 ? 0.0159 : z < 0.06 ? 0.0159 - (z - 0.035) / 0.025 * 0.0048 : 0.0111; return [r, r]; }, { segments: 60, radial: 16, refAxis: V(1, 0, 0) });
    addF(barGeo, M.anodBlack);
    const clamp = new THREE.CylinderGeometry(0.021, 0.021, 0.045, 20).rotateX(Math.PI / 2);
    clamp.translate(bar.x, bar.y, 0);
    addF(clamp, M.anodBlack);
    for (const s of [-1, 1]) {
      const end = pts[s > 0 ? 12 : 0], inner = pts[s > 0 ? 9 : 3];
      // 锁死式握把
      const grip = new THREE.CylinderGeometry(0.0165, 0.0165, 1, 18);
      const gA = end.clone().lerp(inner, 0.0), gB = end.clone().lerp(inner, 0.92);
      alignY(grip.scale(1, gA.distanceTo(gB), 1), gB, gA);
      addF(grip, M.rubber);
      const lockRing = new THREE.CylinderGeometry(0.0172, 0.0172, 0.008, 18);
      alignY(lockRing, gB, gB.clone().lerp(inner, 0.09));
      addF(lockRing, M.accent);
      // 液压刹车把 + 油壶
      const mc = new RoundedBoxGeometry(0.03, 0.026, 0.05, 2, 0.008);
      const mcM = addF(mc, M.anodBlack);
      const lp = end.clone().lerp(inner, 1.12);
      mcM.position.copy(lp).add(V(0.01, 0.012, 0));
      const leverPts = [lp.clone().add(V(0.022, 0.0, 0)), lp.clone().add(V(0.05, -0.012, s * 0.04)), lp.clone().add(V(0.045, -0.02, s * 0.09))];
      addF(curveTube(leverPts, (t) => [0.0035, 0.006 - 0.002 * t], { segments: 12, radial: 8 }), M.anodGrey);
    }
  }

  // ---------------------------------------------------------------- 动量轮模块
  buildReactionWheel(G, p) {
    const c = P(G.rwCenter);
    const rr = p.rwRadius;
    const grp = new THREE.Group();
    grp.position.copy(c);
    this.frame.add(grp);
    this.labelAnchors.rw = grp;
    // 飞轮（旋转部分）：抛光钢轮缘 + 镂空腹板 + 橙色标记
    const spin = (this.rwSpin = new THREE.Group());
    grp.add(spin);
    const rimPts = [[rr - 0.024, -0.016], [rr - 0.003, -0.016], [rr, -0.013], [rr, 0.013], [rr - 0.003, 0.016], [rr - 0.024, 0.016], [rr - 0.026, 0.012], [rr - 0.026, -0.012], [rr - 0.024, -0.016]];
    const rim = new THREE.LatheGeometry(rimPts.map(([r, y]) => new THREE.Vector2(r, y)), 96).rotateZ(-Math.PI / 2);
    this.add(rim, M.chrome, spin);
    const web = new THREE.Shape();
    web.absarc(0, 0, rr - 0.023, 0, Math.PI * 2, false);
    web.holes.push(circlePath(0.012));
    for (let k = 0; k < 6; k++) {
      const a = (k / 6) * Math.PI * 2;
      web.holes.push(circlePath(0.028, 0.072 * Math.cos(a), 0.072 * Math.sin(a), 20));
    }
    const webG = new THREE.ExtrudeGeometry(web, { depth: 0.006, bevelEnabled: true, bevelSize: 0.0015, bevelThickness: 0.0015, bevelSegments: 1, curveSegments: 32 }).translate(0, 0, -0.003).rotateY(Math.PI / 2);
    this.add(webG, M.anodBlack, spin);
    const hub = new THREE.CylinderGeometry(0.026, 0.026, 0.03, 24).rotateZ(Math.PI / 2);
    this.add(hub, M.alu, spin);
    const mark = new THREE.BoxGeometry(0.034, 0.012, 0.022);
    mark.translate(0, rr - 0.012, 0);
    this.add(mark, M.accent, spin);
    // 外转子电机（转子随飞轮转，定子座固定）
    const bell = new THREE.CylinderGeometry(0.056, 0.056, 0.026, 40).rotateZ(Math.PI / 2).translate(-0.03, 0, 0);
    this.add(bell, M.anodBlack, spin);
    const fins = [];
    for (let k = 0; k < 18; k++) {
      const f = new THREE.BoxGeometry(0.024, 0.006, 0.0025).translate(-0.03, 0.058, 0);
      f.rotateX((k / 18) * Math.PI * 2);
      fins.push(f);
    }
    this.add(merge(fins), M.anodGrey, spin);
    // 定子安装板 + 撑杆连到上管、下管、座管
    const plate = new THREE.CylinderGeometry(0.05, 0.05, 0.007, 32).rotateZ(Math.PI / 2).translate(-0.05, 0, 0);
    this.add(plate, M.alu, grp);
    const strutMat = M.accent;
    const toLocal = (xy) => P(xy).sub(c);
    const ttPt = toLocal(lerp(G.ttRear, G.ttFront, (G.rwCenter[0] - 0.05 - G.ttRear[0]) / (G.ttFront[0] - G.ttRear[0])));
    const dtPt = toLocal(lerp(G.bb, G.dtFront, (G.rwCenter[0] - 0.05 - G.bb[0]) / (G.dtFront[0] - G.bb[0])));
    const stPt = toLocal(lerp(G.bb, G.seatTop, 0.62));
    const struts = [
      [V(-0.05, 0.045, 0), ttPt.clone().add(V(0, -0.018, 0))],
      [V(-0.05, -0.045, 0), dtPt.clone().add(V(0, 0.03, 0))],
      [V(-0.054, 0, 0.02), stPt.clone().add(V(0.016, 0, 0.012))],
      [V(-0.054, 0, -0.02), stPt.clone().add(V(0.016, 0, -0.012))],
    ];
    for (const [a, b] of struts) this.add(tube(a, b, 0.0075), strutMat, grp);
    // 管夹
    for (const [pt, r, dirA, dirB] of [[ttPt, 0.024, G.ttRear, G.ttFront], [dtPt, 0.046, G.bb, G.dtFront]]) {
      const d = P(dirB).sub(P(dirA)).normalize();
      const cl = new THREE.CylinderGeometry(r, r, 0.03, 24);
      alignY(cl, pt.clone().addScaledVector(d, -0.015), pt.clone().addScaledVector(d, 0.015));
      this.add(cl, M.anodBlack, grp);
    }
    // 聚碳酸酯护罩：环带 + 前透明盖 + 橙色边圈
    const gr = rr + 0.01;
    const band = new THREE.CylinderGeometry(gr, gr, 0.04, 72, 1, true).rotateZ(Math.PI / 2);
    this.add(band, M.polycarb, grp, false);
    const cover = new THREE.CircleGeometry(gr, 72).rotateY(Math.PI / 2).translate(0.02, 0, 0);
    this.add(cover, M.polycarb, grp, false);
    for (const x of [-0.02, 0.02]) {
      const t = new THREE.TorusGeometry(gr, 0.0035, 8, 96).rotateY(Math.PI / 2).translate(x, 0, 0);
      this.add(t, M.accent, grp);
    }
  }

  // ---------------------------------------------------------------- 转向执行器（电机 + 同步带）
  buildSteerActuator(G) {
    // 在“带轮平面坐标系”里建：原点 = 小带轮中心，局部 y = 平行于转向轴
    const grp = new THREE.Group();
    grp.position.copy(P(G.steerMotor));
    grp.rotation.z = G.lam;
    this.frame.add(grp);
    this.labelAnchors.steer = grp;
    // 小带轮（15 齿）
    const small = new THREE.ExtrudeGeometry(gearShape(15, 0.0145, 0.0165, { holes: [circlePath(0.004)] }), { depth: 0.011, bevelEnabled: false });
    small.rotateX(-Math.PI / 2).translate(0, -0.0055, 0);
    this.smallPulley = this.add(small, M.alu, grp);
    // 电机（在带轮上方）+ 减速箱 + 编码器盖
    const motor = new THREE.CylinderGeometry(0.03, 0.03, 0.048, 32).translate(0, 0.036, 0);
    this.add(motor, M.anodBlack, grp);
    const enc = new THREE.CylinderGeometry(0.024, 0.026, 0.014, 32).translate(0, 0.066, 0);
    this.add(enc, M.accent, grp);
    const finsG = [];
    for (let k = 0; k < 20; k++) {
      const f = new THREE.BoxGeometry(0.004, 0.04, 0.003).translate(0.031, 0.036, 0);
      f.rotateY((k / 20) * Math.PI * 2);
      finsG.push(f);
    }
    this.add(merge(finsG), M.anodGrey, grp);
    // 安装支架：从电机下方连到上管（管夹）
    const brk = new RoundedBoxGeometry(0.05, 0.006, 0.05, 2, 0.003).translate(0, 0.009, 0);
    this.add(brk, M.alu, grp);
    const ttLocalY = -0.035;
    this.add(tube(V(0, 0.008, 0.02), V(0.0, ttLocalY, 0.024), 0.005), M.alu, grp);
    this.add(tube(V(0, 0.008, -0.02), V(0.0, ttLocalY, -0.024), 0.005), M.alu, grp);
    // 同步带（扫掠矩形截面）
    const loop = beltLoop([G.beltC, 0], 0.0475, [0, 0], 0.0155, 0.002);
    const curve = new THREE.CatmullRomCurve3(loop.map((l) => V(l.p[0], 0, -l.p[1])), true);
    const belt = sweep(curve, { segments: 260, radial: 4, radius: () => [0.0055, 0.0015], refAxis: V(0, 1, 0), caps: false });
    this.add(belt, M.rubber, grp);
  }

  // ---------------------------------------------------------------- 传感器：激光雷达 / 深度相机 / 前灯 / GNSS / 急停
  buildSensors(G) {
    const y = G.rack.y;
    // 雷达支架（碳纤维管）+ 状态灯环
    const mastTop = V(G.lidar[0], G.lidar[1] - 0.035, 0);
    this.add(tube(V(G.lidar[0], y, 0), mastTop, 0.0135), M.carbon);
    const base = new THREE.CylinderGeometry(0.03, 0.035, 0.018, 24).translate(G.lidar[0], y + 0.016, 0);
    this.add(base, M.anodBlack);
    this.statusRing = this.add(new THREE.TorusGeometry(0.02, 0.004, 8, 32).rotateX(Math.PI / 2).translate(mastTop.x, mastTop.y - 0.05, 0), M.statusRing.clone(), this.frame, false);
    // 激光雷达本体
    const L = new THREE.Group();
    L.position.set(G.lidar[0], G.lidar[1], 0);
    this.frame.add(L);
    this.labelAnchors.lidar = L;
    this.add(new THREE.CylinderGeometry(0.052, 0.054, 0.022, 48).translate(0, -0.024, 0), M.anodBlack, L);
    const head = (this.lidarHead = new THREE.Group());
    L.add(head);
    this.add(new THREE.CylinderGeometry(0.05, 0.05, 0.036, 48, 1, true).translate(0, 0.005, 0), M.glassDark, head);
    this.add(new THREE.CylinderGeometry(0.052, 0.052, 0.014, 48).translate(0, 0.03, 0), M.anodGrey, head);
    this.add(new THREE.TorusGeometry(0.046, 0.0025, 6, 48).rotateX(Math.PI / 2).translate(0, 0.038, 0), M.accent, head);
    // 内部发射器（透过视窗可见的红色激光光斑）
    const emitter = new THREE.Mesh(new THREE.PlaneGeometry(0.008, 0.026), M.laser);
    emitter.position.set(0.0505, 0.005, 0);
    emitter.rotation.y = Math.PI / 2;
    head.add(emitter);
    // 深度相机（夹在头管前方，下俯 8°）
    const cam = new THREE.Group();
    cam.position.copy(P(G.camera));
    cam.rotation.z = -0.14;
    this.frame.add(cam);
    this.labelAnchors.camera = cam;
    this.add(new RoundedBoxGeometry(0.026, 0.026, 0.092, 3, 0.006), M.anodGrey, cam);
    const face = new THREE.Mesh(new RoundedBoxGeometry(0.002, 0.02, 0.084, 2, 0.001), M.glassDark);
    face.position.x = 0.013;
    cam.add(face);
    for (const [z, r] of [[-0.034, 0.0045], [0.034, 0.0045], [0.012, 0.0055], [-0.012, 0.004]]) {
      const lens = new THREE.CylinderGeometry(r, r, 0.004, 20).rotateZ(Math.PI / 2).translate(0.014, 0, z);
      this.add(lens, M.lens, cam, false);
    }
    // 相机支架：抱箍头管
    const clampR = 0.034;
    const hc = P(G.axisAtY(G.camera[1] - 0.005));
    const ring = new THREE.TorusGeometry(clampR, 0.005, 8, 32).rotateX(Math.PI / 2);
    const rm = this.add(ring, M.anodBlack);
    rm.position.copy(hc);
    rm.rotation.z = G.lam;
    this.add(tube(hc.clone().add(V(clampR * G.cl, clampR * G.sl, 0)), P(G.camera).add(V(-0.012, 0, 0)), 0.006), M.anodBlack);
    // 车载摄像机（用于画中画）：位于 RGB 镜头，看向 +x
    this.onboardCam = new THREE.PerspectiveCamera(58, 16 / 9, 0.05, 150);
    this.onboardCam.position.set(0.02, 0, 0.012);
    this.onboardCam.rotation.y = -Math.PI / 2;
    cam.add(this.onboardCam);
    // 前灯
    const hl = new THREE.Group();
    hl.position.copy(P(G.headlight));
    this.frame.add(hl);
    this.add(new THREE.CylinderGeometry(0.019, 0.016, 0.04, 24).rotateZ(Math.PI / 2), M.anodBlack, hl);
    this.add(new THREE.CircleGeometry(0.0165, 24).rotateY(Math.PI / 2).translate(0.0205, 0, 0), M.headlight, hl, false);
    this.add(tube(V(-0.02, 0, 0), P(G.axisAtY(G.headlight[1])).sub(P(G.headlight)).add(V(0.03, 0, 0)), 0.005), M.anodBlack, hl);
    // GNSS 天线
    const gn = new THREE.Group();
    gn.position.copy(P(G.gnss));
    this.frame.add(gn);
    this.labelAnchors.gnss = gn;
    this.add(new THREE.CylinderGeometry(0.045, 0.047, 0.012, 40), M.white, gn);
    const dome = new THREE.SphereGeometry(0.044, 32, 12, 0, Math.PI * 2, 0, Math.PI / 2).scale(1, 0.35, 1).translate(0, 0.006, 0);
    this.add(dome, M.white, gn);
    // 急停按钮
    const es = new THREE.Group();
    es.position.copy(P(G.estop));
    this.frame.add(es);
    this.labelAnchors.estop = es;
    this.add(new RoundedBoxGeometry(0.05, 0.036, 0.05, 2, 0.006), M.yellow, es);
    this.add(new THREE.CylinderGeometry(0.009, 0.009, 0.012, 16).translate(0, 0.024, 0), M.red, es);
    this.add(new THREE.SphereGeometry(0.019, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2).scale(1, 0.55, 1).translate(0, 0.029, 0), M.red, es);
  }

  // ---------------------------------------------------------------- 计算单元 / 主控
  buildElectronics(G) {
    const cu = new THREE.Group();
    cu.position.copy(P(G.compute));
    this.frame.add(cu);
    this.labelAnchors.compute = cu;
    this.add(new RoundedBoxGeometry(0.2, 0.05, 0.13, 3, 0.008), M.anodGrey, cu);
    const fins = [];
    for (let k = 0; k < 15; k++) fins.push(new THREE.BoxGeometry(0.17, 0.02, 0.0016).translate(0, 0.034, -0.056 + k * 0.008));
    this.add(merge(fins), M.alu, cu);
    // 风扇格栅（后面）+ 接口 + 指示灯
    const fan = new THREE.CylinderGeometry(0.018, 0.018, 0.003, 24).rotateZ(Math.PI / 2).translate(-0.101, 0, 0.03);
    this.add(fan, M.plastic, cu);
    for (let k = 0; k < 4; k++) this.add(new THREE.BoxGeometry(0.012, 0.007, 0.002).translate(-0.06 + k * 0.022, -0.008, 0.066), M.plastic, cu, false);
    this.cuLeds = [];
    for (let k = 0; k < 3; k++) {
      const led = this.add(new THREE.SphereGeometry(0.0025, 8, 6).translate(0.07 + k * 0.009, 0.01, 0.066), k === 0 ? M.ledGreen : M.ledBlue, cu, false);
      this.cuLeds.push(led);
    }
    const lab = this.textDecal('AV-COMPUTE', 0.08, 0.012, '#e8ebf0');
    lab.position.set(-0.02, 0.012, 0.0655);
    cu.add(lab);
    // 数传天线 ×2
    for (const z of [-0.05, 0.05]) {
      this.add(new THREE.CylinderGeometry(0.003, 0.004, 0.12, 8).translate(0.085, 0.085, z), M.plastic, cu);
    }
    // 主控 + IMU + FOC 驱动盒（上管下方）
    const mc = new THREE.Group();
    mc.position.copy(P(G.mcu));
    mc.rotation.z = Math.atan2(G.ttFront[1] - G.ttRear[1], G.ttFront[0] - G.ttRear[0]);
    this.frame.add(mc);
    this.labelAnchors.mcu = mc;
    this.add(new RoundedBoxGeometry(0.12, 0.048, 0.056, 3, 0.008), M.plastic, mc);
    this.add(new THREE.BoxGeometry(0.1, 0.002, 0.04).translate(0, 0.0245, 0), M.pcb, mc, false);
    const lab2 = this.textDecal('MCU · IMU · FOC×3', 0.09, 0.011, '#ff8a3d');
    lab2.position.set(0, 0, 0.0285);
    mc.add(lab2);
    this.add(tube(V(0, 0.024, 0), V(0, 0.058, 0), 0.006), M.anodBlack, mc);
  }

  // ---------------------------------------------------------------- 线束
  buildCables(G) {
    const mcu = P(G.mcu);
    const cables = [
      // MCU → 动量轮电机
      [mcu.clone().add(V(0.05, -0.02, 0.02)), P(G.rwCenter).add(V(-0.08, 0.06, 0.03)), P(G.rwCenter).add(V(-0.06, 0.03, 0.03))],
      // MCU → 转向电机（沿上管）
      [mcu.clone().add(V(0.06, 0.01, -0.02)), P(lerp(G.ttRear, G.ttFront, 0.6)).add(V(0, 0.02, -0.022)), P(G.steerMotor).add(V(-0.02, 0.03, -0.03))],
      // MCU → 轮毂电机（沿后下叉）
      [mcu.clone().add(V(-0.05, -0.02, 0.02)), P(lerp(G.bb, G.seatTop, 0.3)).add(V(-0.02, 0, 0.03)), P(lerp(G.bb, G.rearHub, 0.5)).add(V(0, 0.018, 0.06)), P(G.rearHub).add(V(0.03, 0.01, 0.07))],
      // 计算单元 → 雷达
      [P(G.compute).add(V(-0.09, 0.01, -0.05)), V(G.lidar[0] + 0.02, G.rack.y + 0.15, -0.012), V(G.lidar[0] + 0.012, G.lidar[1] - 0.06, -0.01)],
      // 计算单元 → MCU（沿座管）
      [P(G.compute).add(V(0.1, -0.01, -0.03)), P(G.seatTop).add(V(-0.025, -0.08, -0.02)), mcu.clone().add(V(-0.06, 0, -0.02))],
      // 计算单元 → 相机（沿上管）
      [P(G.compute).add(V(0.1, 0, 0.04)), P(G.seatTop).add(V(0.0, -0.04, 0.024)), P(lerp(G.ttRear, G.ttFront, 0.5)).add(V(0, 0.022, 0.016)), P(G.camera).add(V(-0.03, 0.02, 0.02))],
    ];
    const geos = cables.map((pts) => curveTube(pts, 0.0032, { segments: 40, radial: 6 }));
    this.add(merge(geos), M.cable, this.frame, false);
    // 高压线（电池 → MCU，橙色）
    this.add(curveTube([P(lerp(G.bb, G.dtFront, 0.3)).add(V(0, 0.03, 0.02)), P(lerp(G.bb, G.seatTop, 0.6)).add(V(0.02, 0, 0.026)), P(G.mcu).add(V(-0.05, -0.02, 0.026))], 0.004, { segments: 24, radial: 8 }), M.cableOrange, this.frame, false);
  }

  textDecal(text, w, h, color) {
    const c = document.createElement('canvas');
    c.width = 512; c.height = Math.round((512 * h) / w);
    const ctx = c.getContext('2d');
    ctx.fillStyle = color;
    ctx.font = `600 ${Math.round(c.height * 0.72)}px "Segoe UI", "Microsoft YaHei", sans-serif`;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'center';
    ctx.fillText(text, c.width / 2, c.height / 2 + 2);
    const t = new THREE.CanvasTexture(c);
    t.colorSpace = THREE.SRGBColorSpace;
    t.anisotropy = 8;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshStandardMaterial({ map: t, transparent: true, roughness: 0.5, polygonOffset: true, polygonOffsetFactor: -4 }));
    return m;
  }

  // ---------------------------------------------------------------- 动画
  update(sim, renderT) {
    const s = sim.s;
    this.root.position.set(s.x, 0, s.y);
    this.root.rotation.y = -s.psi;
    let phi = s.phi;
    if (sim.fallen) {
      // 倒地后按重力加速度“摔”到车把着地的角度（≈70°）
      const t = sim.t - sim.fallT;
      const target = 1.22;
      const a0 = Math.abs(sim.fallPhi0 ?? s.phi);
      phi = Math.sign(s.phi) * Math.min(target, a0 + 0.5 * 9 * t * t);
    }
    this.rollGroup.rotation.x = phi;
    this.steerRot.rotation.y = -s.delta;
    this.rearWheel.userData.spin.rotation.z = -s.rearAngle;
    this.frontWheel.userData.spin.rotation.z = -s.frontAngle;
    this.rwSpin.rotation.x = s.rwAngle;
    // 小带轮转速 = 3 × 大带轮（1:3 减速）
    this.smallPulley.rotation.y = -3 * s.delta;
    this.lidarHead.rotation.y = -renderT * 2 * Math.PI * 10 * 0.25; // 10 Hz（视觉上降速 4 倍，避免频闪）
    const braking = sim.act.brake > 0.05;
    this.tailLight.material.emissiveIntensity = braking ? 6 : 1.2 + 0.6 * (Math.sin(renderT * 6) > 0 ? 1 : 0);
    const mode = sim.autopilot.mode;
    const col = sim.fallen ? 0xff2030 : mode === 'ESTOP' ? 0xff2030 : sim.autopilot.emergency ? 0xffa000 : mode === 'IDLE' ? 0x18c8ff : mode === 'MANUAL' ? 0xb070ff : 0x20ff80;
    this.statusRing.material.emissive.setHex(col);
    this.screen.material.color.setHex(col);
    this.cuLeds[1].visible = Math.sin(renderT * 17) > 0;
  }
}

function lerp(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
}
