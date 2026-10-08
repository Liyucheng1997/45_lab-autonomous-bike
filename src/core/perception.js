// 视觉感知（RGB-D 相机 + NPU 上的目标检测网络）的行为级模型：
//   - 只有在相机视场 (87°×58°) 和量程 (≤ 25 m) 内、未被遮挡的目标才能被检测到
//   - 检测置信度随距离下降，带位置噪声；多帧关联后用 α-β 滤波估计目标速度
//   - 输出 {kind, x, y, vx, vy, dist, conf}，供决策层做行人让行预测
// 遮挡判断借用激光雷达距离场：相机到目标的视线被其他障碍挡住则不可见。

import { makeRng } from './mathx.js';

const CLASSES = { ped: '行人', car: '汽车', van: '面包车', cone: '锥桶', barrier: '护栏', crate: '纸箱' };

export class CameraDetector {
  constructor(geom, { hfov = 87, range = 25 } = {}) {
    this.hfov = (hfov * Math.PI) / 180;
    this.range = range;
    this.mount = geom.camera;
    this.tracks = new Map();
    this.rng = makeRng(11);
    this.detections = [];
  }

  update(s, obstacles, dt) {
    const cx = s.x + this.mount[0] * Math.cos(s.psi);
    const cy = s.y + this.mount[0] * Math.sin(s.psi);
    const out = [];
    const seen = new Set();
    for (const o of obstacles) {
      if (!CLASSES[o.kind]) continue;
      const dx = o.x - cx, dy = o.y - cy;
      const d = Math.hypot(dx, dy);
      if (d > this.range || d < 0.3) continue;
      const bearing = Math.atan2(dy, dx) - s.psi;
      const b = Math.atan2(Math.sin(bearing), Math.cos(bearing));
      if (Math.abs(b) > this.hfov / 2) continue;
      if (occluded(obstacles, o, cx, cy)) continue;
      const conf = Math.max(0.35, Math.min(0.99, 1.02 - d / 40 + this.rng.gauss() * 0.03));
      const n = 0.03 + d * 0.006;
      const mx = o.x + this.rng.gauss() * n, my = o.y + this.rng.gauss() * n;
      // α-β 跟踪
      let tr = this.tracks.get(o);
      if (!tr) tr = { x: mx, y: my, vx: 0, vy: 0, age: 0 };
      else {
        const px = tr.x + tr.vx * dt, py = tr.y + tr.vy * dt;
        const rx = mx - px, ry = my - py;
        tr.x = px + 0.5 * rx; tr.y = py + 0.5 * ry;
        tr.vx += (0.25 * rx) / dt; tr.vy += (0.25 * ry) / dt;
        tr.age++;
      }
      this.tracks.set(o, tr);
      seen.add(o);
      out.push({ obj: o, kind: o.kind, label: CLASSES[o.kind], x: tr.x, y: tr.y, vx: tr.vx, vy: tr.vy, dist: d, conf });
    }
    for (const k of [...this.tracks.keys()]) if (!seen.has(k)) this.tracks.delete(k);
    this.detections = out;
    return out;
  }
}

// 粗略遮挡：视线段与其他（更近的）障碍包围圆相交
function occluded(obstacles, target, cx, cy) {
  const tx = target.x, ty = target.y;
  const L = Math.hypot(tx - cx, ty - cy);
  for (const o of obstacles) {
    if (o === target || o.kind === 'lamp') continue;
    const r = o.shape === 'box' ? Math.min(o.lx, o.ly) / 2 : o.crown ? o.r : o.r;
    const vx = (tx - cx) / L, vy = (ty - cy) / L;
    const t = (o.x - cx) * vx + (o.y - cy) * vy;
    if (t < 0.3 || t > L - 0.6) continue;
    const px = cx + vx * t, py = cy + vy * t;
    if (Math.hypot(o.x - px, o.y - py) < r * 0.9 && (o.h || 2) > 1.0) return true;
  }
  return false;
}
