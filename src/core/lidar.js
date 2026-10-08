// 16 线 360° 激光雷达模型：解析射线求交（地面平面、旋转盒、竖直圆柱、球形树冠）。
// 雷达装在车上，射线随车身横滚一起倾斜（真实效果：过弯时扫描线会“歪”）。
// 输出世界坐标点云 + 每束最近距离，供建图/避障使用。

export class Lidar {
  constructor({ channels = 16, fovUp = 15, fovDown = -15, azSteps = 360, range = 40, noise = 0.015, mountH = 1.16, mountX = -0.12 } = {}) {
    this.channels = channels;
    this.azSteps = azSteps;
    this.range = range;
    this.noise = noise;
    this.mountH = mountH;
    this.mountX = mountX;
    this.elev = Array.from({ length: channels }, (_, i) => ((fovDown + ((fovUp - fovDown) * i) / (channels - 1)) * Math.PI) / 180);
    const n = channels * azSteps;
    this.points = new Float32Array(n * 3);   // 命中点 (three 坐标 x, y, z)
    this.hitKind = new Uint8Array(n);         // 0 无, 1 地面, 2 障碍
    this.count = 0;
    this.origin = [0, 0, 0];
    this.seed = 1;
  }

  // pose: {x, y, psi, phi}；obstacles: world.js 的障碍列表
  scan(pose, obstacles) {
    const { x, y, psi, phi } = pose;
    const cps = Math.cos(psi), sps = Math.sin(psi), cph = Math.cos(phi), sph = Math.sin(phi);
    // 雷达在车身局部的位置 (mx, mh, 0) → 世界
    const lx = this.mountX, ly = this.mountH;
    // 横滚：局部 (lx, ly, 0) 绕 x 轴转 φ → (lx, ly cosφ, ly sinφ)
    const by = ly * cph, bz = ly * sph;
    // 航向：局部 x → (cosψ, sinψ)，局部 z（右）→ (−sinψ, cosψ)（平面坐标）
    const ox = x + lx * cps - bz * sps;
    const oz = y + lx * sps + bz * cps;
    const oy = by;
    this.origin = [ox, oy, oz];
    this.heading = psi;

    // 预筛：范围内的障碍，并按方位角分桶（每束射线只测它所在扇区里的障碍）
    const R = this.range;
    const A = this.azSteps;
    if (!this.bins) this.bins = Array.from({ length: A }, () => []);
    const bins = this.bins;
    for (const b of bins) b.length = 0;
    const all = [];
    const margin = 3 + Math.ceil(Math.abs(phi) * 20);
    for (const o of obstacles) {
      const rr = (o.shape === 'box' ? Math.hypot(o.lx, o.ly) / 2 : (o.crown || o.r)) + 0.2;
      const dx = o.x - ox, dz = o.y - oz;
      const d = Math.hypot(dx, dz);
      if (d > R + rr) continue;
      if (d <= rr + 0.5) { all.push(o); continue; }
      const ang = Math.atan2(dz, dx) - psi;
      const half = Math.asin(Math.min(1, rr / d));
      const b0 = Math.floor(((ang - half) / (2 * Math.PI)) * A) - margin;
      const b1 = Math.ceil(((ang + half) / (2 * Math.PI)) * A) + margin;
      for (let b = b0; b <= b1; b++) bins[((b % A) + A) % A].push(o);
    }

    let k = 0;
    let s = this.seed = (this.seed * 1103515245 + 12345) >>> 0;
    const rnd = () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296 - 0.5);
    for (let c = 0; c < this.channels; c++) {
      const el = this.elev[c];
      const ce = Math.cos(el), se = Math.sin(el);
      for (let a = 0; a < this.azSteps; a++) {
        const az = (a / this.azSteps) * 2 * Math.PI + c * 0.003;
        // 局部方向 (x 前, y 上, z 右)
        const dx0 = ce * Math.cos(az), dy0 = se, dz0 = ce * Math.sin(az);
        // 横滚
        const dy1 = dy0 * cph - dz0 * sph, dz1 = dy0 * sph + dz0 * cph;
        // 航向 → 世界 (平面 x, 高度 y, 平面 y)
        const dx = dx0 * cps - dz1 * sps, dz = dx0 * sps + dz1 * cps, dy = dy1;
        let tBest = R, kind = 0;
        if (dy < -1e-4) {
          const tg = -oy / dy;
          if (tg < tBest) { tBest = tg; kind = 1; }
        }
        const bin = bins[a];
        for (let i = 0; i < bin.length; i++) {
          const t = intersect(bin[i], ox, oy, oz, dx, dy, dz, tBest);
          if (t < tBest) { tBest = t; kind = 2; }
        }
        for (let i = 0; i < all.length; i++) {
          const t = intersect(all[i], ox, oy, oz, dx, dy, dz, tBest);
          if (t < tBest) { tBest = t; kind = 2; }
        }
        const idx = c * this.azSteps + a;
        this.hitKind[idx] = kind;
        if (kind) {
          const t = tBest + rnd() * this.noise * 2;
          this.points[3 * k] = ox + dx * t;
          this.points[3 * k + 1] = oy + dy * t;
          this.points[3 * k + 2] = oz + dz * t;
          k++;
        }
      }
    }
    this.count = k;
    return this;
  }
}

export function intersect(o, ox, oy, oz, dx, dy, dz, tMax) {
  if (o.shape === 'box') {
    // 转到盒子局部（绕竖直轴 yaw）
    const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
    const px = ox - o.x, pz = oz - o.y;
    const lx = px * c + pz * s, lz = -px * s + pz * c;
    const ldx = dx * c + dz * s, ldz = -dx * s + dz * c;
    let t0 = 0, t1 = tMax;
    const slab = (p, d, lo, hi) => {
      if (Math.abs(d) < 1e-9) return p >= lo && p <= hi;
      let a = (lo - p) / d, b = (hi - p) / d;
      if (a > b) [a, b] = [b, a];
      if (a > t0) t0 = a;
      if (b < t1) t1 = b;
      return t0 <= t1;
    };
    if (!slab(lx, ldx, -o.lx / 2, o.lx / 2)) return Infinity;
    if (!slab(lz, ldz, -o.ly / 2, o.ly / 2)) return Infinity;
    if (!slab(oy, dy, 0, o.h)) return Infinity;
    return t0 > 0 ? t0 : Infinity;
  }
  // 竖直圆柱（树干/锥桶/行人/灯杆）
  let best = Infinity;
  const px = ox - o.x, pz = oz - o.y;
  const a = dx * dx + dz * dz;
  if (a > 1e-9) {
    const r = o.kind === 'cone' ? o.r * 0.8 : o.r;
    const b = px * dx + pz * dz, cc = px * px + pz * pz - r * r;
    const disc = b * b - a * cc;
    if (disc >= 0) {
      const t = (-b - Math.sqrt(disc)) / a;
      if (t > 0 && t < tMax) {
        const hy = oy + dy * t;
        if (hy >= 0 && hy <= (o.crown ? o.h * 0.55 : o.h)) best = t;
      }
    }
  }
  if (o.crown) {
    // 树冠：球心在 (x, h − crown·0.6, y)
    const cy = o.h - o.crown * 0.6;
    const qx = ox - o.x, qy = oy - cy, qz = oz - o.y;
    const b = qx * dx + qy * dy + qz * dz;
    const cc = qx * qx + qy * qy + qz * qz - o.crown * o.crown;
    const disc = b * b - cc;
    if (disc >= 0) {
      const t = -b - Math.sqrt(disc);
      if (t > 0 && t < Math.min(best, tMax)) best = t;
    }
  }
  return best;
}
