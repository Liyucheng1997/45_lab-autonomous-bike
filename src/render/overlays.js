// 自动驾驶可视化叠加层：激光点云、占据栅格、规划路径（按速度着色）、路线、目标点、检测框、行人预测轨迹。
import * as THREE from 'three';

const turbo = (t) => {
  // 近似 Turbo 色图
  t = Math.min(1, Math.max(0, t));
  const r = 0.1357 + t * (4.5974 - t * (42.3277 - t * (130.5887 - t * (150.5666 - t * 58.1375))));
  const g = 0.0914 + t * (2.1856 + t * (4.8052 - t * (14.0195 - t * (4.2109 + t * 2.7747))));
  const b = 0.1067 + t * (12.5925 - t * (60.1097 - t * (109.0745 - t * (88.5066 - t * 26.8183))));
  return [Math.min(1, Math.max(0, r)), Math.min(1, Math.max(0, g)), Math.min(1, Math.max(0, b))];
};

export class Overlays {
  constructor(scene, sim) {
    this.scene = scene;
    this.sim = sim;
    this.group = new THREE.Group();
    scene.add(this.group);
    const ap = sim.autopilot;

    // 激光点云
    const n = ap.lidar.points.length / 3;
    const g = new THREE.BufferGeometry();
    this.ptPos = new THREE.BufferAttribute(new Float32Array(n * 3), 3);
    this.ptCol = new THREE.BufferAttribute(new Float32Array(n * 3), 3);
    g.setAttribute('position', this.ptPos);
    g.setAttribute('color', this.ptCol);
    this.points = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.055, vertexColors: true, sizeAttenuation: true, transparent: true, opacity: 0.95 }));
    this.points.frustumCulled = false;
    this.group.add(this.points);

    // 占据栅格
    const grid = ap.grid;
    this.gridData = new Uint8Array(grid.nx * grid.ny * 4);
    this.gridTex = new THREE.DataTexture(this.gridData, grid.nx, grid.ny, THREE.RGBAFormat);
    this.gridTex.magFilter = THREE.NearestFilter;
    const W = grid.nx * grid.res, H = grid.ny * grid.res;
    this.gridMesh = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshBasicMaterial({ map: this.gridTex, transparent: true, depthWrite: false, side: THREE.DoubleSide }));
    this.gridMesh.rotation.x = Math.PI / 2;
    this.gridMesh.position.set(grid.xmin + W / 2, 0.035, grid.ymin + H / 2);
    this.gridMesh.renderOrder = 1;
    this.group.add(this.gridMesh);
    this.gridVersion = -1;
    this.tGrid = 0;

    // 路线（车道中心，黄色虚线）
    const rp = ap.route.map(([x, y]) => new THREE.Vector3(x, 0.04, y));
    rp.push(rp[0].clone());
    const rl = new THREE.Line(new THREE.BufferGeometry().setFromPoints(rp), new THREE.LineDashedMaterial({ color: 0xffd23f, dashSize: 0.6, gapSize: 0.5, transparent: true, opacity: 0.7 }));
    rl.computeLineDistances();
    this.routeLine = rl;
    this.group.add(rl);

    // 规划路径：贴地彩带（颜色 = 速度规划）
    this.pathMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.75, depthWrite: false, side: THREE.DoubleSide }));
    this.pathMesh.renderOrder = 2;
    this.pathMesh.frustumCulled = false;
    this.group.add(this.pathMesh);
    this.lastPath = null;

    // 胡萝卜点 / 跟踪目标点 / 导航目标
    this.carrot = new THREE.Mesh(new THREE.TorusGeometry(0.45, 0.04, 8, 40).rotateX(Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0xffd23f }));
    this.group.add(this.carrot);
    this.ppTarget = new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 8), new THREE.MeshBasicMaterial({ color: 0x40e0ff }));
    this.group.add(this.ppTarget);
    this.ppLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ color: 0x40e0ff }));
    this.group.add(this.ppLine);
    this.goal = new THREE.Group();
    this.goal.add(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 1.6, 8).translate(0, 0.8, 0), new THREE.MeshStandardMaterial({ color: 0xdddddd })));
    this.goal.add(new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.4, 3).rotateZ(-Math.PI / 2).translate(0.2, 1.4, 0), new THREE.MeshStandardMaterial({ color: 0xff3b6b, emissive: 0x801030 })));
    this.goal.add(new THREE.Mesh(new THREE.RingGeometry(0.5, 0.62, 40).rotateX(-Math.PI / 2).translate(0, 0.05, 0), new THREE.MeshBasicMaterial({ color: 0xff3b6b, side: THREE.DoubleSide })));
    this.goal.visible = false;
    this.group.add(this.goal);

    // 紧急制动走廊
    this.corridor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0xff2030, transparent: true, opacity: 0.35, depthWrite: false }));
    this.group.add(this.corridor);

    // 检测框池
    this.boxes = [];
    this.show = { lidar: true, grid: true, path: true, detections: true };
  }

  setVisible(key, v) {
    this.show[key] = v;
    if (key === 'lidar') this.points.visible = v;
    if (key === 'grid') this.gridMesh.visible = v;
    if (key === 'path') [this.pathMesh, this.routeLine, this.carrot, this.ppTarget, this.ppLine].forEach((o) => (o.visible = v));
  }

  update(t) {
    const sim = this.sim, ap = sim.autopilot, s = sim.s;
    // 点云（按高度着色）
    if (this.show.lidar) {
      const L = ap.lidar, P = L.points, n = L.count;
      this.ptPos.array.set(P.subarray(0, n * 3));
      const C = this.ptCol.array;
      for (let k = 0; k < n; k++) {
        const h = P[3 * k + 1];
        const [r, g, b] = turbo(0.08 + h / 3.2);
        C[3 * k] = r; C[3 * k + 1] = g; C[3 * k + 2] = b;
      }
      this.points.geometry.setDrawRange(0, n);
      this.ptPos.needsUpdate = true;
      this.ptCol.needsUpdate = true;
    }
    // 栅格 4 Hz 刷新
    if (this.show.grid && t - this.tGrid > 0.25) {
      this.tGrid = t;
      const grid = ap.grid, D = this.gridData, infl = sim.p.inflate;
      for (let i = 0; i < grid.nx * grid.ny; i++) {
        const o = 4 * i;
        if (grid.occupied(i)) { D[o] = 255; D[o + 1] = 40; D[o + 2] = 60; D[o + 3] = 230; }
        else if (grid.dist[i] < infl) { D[o] = 255; D[o + 1] = 140; D[o + 2] = 30; D[o + 3] = 95; }
        else if (grid.dist[i] < infl + 0.9) { D[o] = 255; D[o + 1] = 210; D[o + 2] = 60; D[o + 3] = 35; }
        else if (grid.seen[i]) { D[o] = 40; D[o + 1] = 190; D[o + 2] = 255; D[o + 3] = 22; }
        else D[o + 3] = 0;
      }
      this.gridTex.needsUpdate = true;
    }
    // 规划路径
    if (ap.path !== this.lastPath) {
      this.lastPath = ap.path;
      this.buildPathRibbon(ap.path, ap.vProfile);
    }
    this.pathMesh.visible = this.show.path && !!ap.path && ap.mode !== 'MANUAL' && ap.mode !== 'IDLE';
    const showC = this.show.path && ap.mode === 'ROUTE' && ap.carrot;
    this.carrot.visible = !!showC;
    if (showC) {
      this.carrot.position.set(ap.carrot[0], 0.06, ap.carrot[1]);
      this.carrot.scale.setScalar(1 + 0.15 * Math.sin(t * 5));
    }
    const pp = ap.pursuit && this.pathMesh.visible ? ap.pursuit.target : null;
    this.ppTarget.visible = this.ppLine.visible = !!pp;
    if (pp) {
      this.ppTarget.position.set(pp[0], 0.15, pp[1]);
      const pos = this.ppLine.geometry.attributes.position;
      pos.setXYZ(0, s.x, 0.15, s.y);
      pos.setXYZ(1, pp[0], 0.15, pp[1]);
      pos.needsUpdate = true;
    }
    this.goal.visible = ap.mode === 'GOTO' && !!ap.goal;
    if (this.goal.visible) this.goal.position.set(ap.goal[0], 0, ap.goal[1]);
    // 紧急制动走廊
    this.corridor.visible = ap.emergency;
    if (ap.emergency) {
      const Lc = 1.1 + sim.p.safetyStop;
      this.corridor.scale.set(Lc, 1, 1.0);
      this.corridor.position.set(s.x + Math.cos(s.psi) * Lc / 2, 0.05, s.y + Math.sin(s.psi) * Lc / 2);
      this.corridor.rotation.y = -s.psi;
      this.corridor.material.opacity = 0.25 + 0.2 * Math.sin(t * 14);
    }
    this.updateDetections(t);
  }

  buildPathRibbon(path, vprof) {
    const geo = this.pathMesh.geometry;
    if (!path || path.length < 2) { geo.setDrawRange(0, 0); return; }
    const pos = [], col = [], idx = [];
    const w = 0.22;
    const vmax = this.sim.p.cruiseSpeed;
    for (let i = 0; i < path.length; i++) {
      const a = path[Math.max(0, i - 1)], b = path[Math.min(path.length - 1, i + 1)];
      const tx = b[0] - a[0], ty = b[1] - a[1], L = Math.hypot(tx, ty) || 1;
      const nx = -ty / L, ny = tx / L;
      const p = path[i];
      pos.push(p[0] + nx * w, 0.045, p[1] + ny * w, p[0] - nx * w, 0.045, p[1] - ny * w);
      const v = vprof ? vprof[i] / vmax : 1;
      // 红(慢) → 黄 → 青(快)
      const c = v < 0.5 ? [1, 0.25 + v * 1.4, 0.2] : [1.6 - v * 1.4, 0.95, 0.3 + (v - 0.5) * 1.6];
      col.push(...c, ...c);
      if (i < path.length - 1) { const k = 2 * i; idx.push(k, k + 1, k + 2, k + 1, k + 3, k + 2); }
    }
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    geo.setIndex(idx);
    geo.setDrawRange(0, idx.length);
    geo.computeBoundingSphere();
  }

  updateDetections(t) {
    const dets = this.show.detections ? this.sim.autopilot.camera.detections : [];
    while (this.boxes.length < dets.length) {
      const g = new THREE.Group();
      const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: 0x40ff90 }));
      g.add(box);
      const arrow = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(1, 0, 0)]), new THREE.LineBasicMaterial({ color: 0xff40c0 }));
      g.userData = { box, arrow };
      this.group.add(g);
      this.group.add(arrow);
      this.boxes.push(g);
    }
    this.boxes.forEach((g, i) => {
      const d = dets[i];
      g.visible = !!d;
      g.userData.arrow.visible = !!d && d.kind === 'ped';
      if (!d) return;
      const o = d.obj;
      const color = d.kind === 'ped' ? 0xff40c0 : d.kind === 'cone' ? 0xffa020 : 0x40ff90;
      g.userData.box.material.color.setHex(color);
      if (o.shape === 'box') { g.scale.set(o.lx + 0.1, o.h + 0.1, o.ly + 0.1); g.rotation.y = -o.yaw; }
      else { g.scale.set(o.r * 2 + 0.25, o.h + 0.1, o.r * 2 + 0.25); g.rotation.y = 0; }
      g.position.set(d.x, (o.h + 0.1) / 2, d.y);
      if (d.kind === 'ped') {
        const a = g.userData.arrow.geometry.attributes.position;
        a.setXYZ(0, d.x, 0.08, d.y);
        a.setXYZ(1, d.x + d.vx * 3, 0.08, d.y + d.vy * 3);
        a.needsUpdate = true;
      }
    });
  }
}
