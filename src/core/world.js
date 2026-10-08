// 测试场地（纯数据，渲染和激光雷达都用它）：
//   - 一条环形道路（圆角矩形中心线），自行车靠右行驶
//   - 道路上的障碍：锥桶阵、停放车辆、施工护栏、纸箱、过马路的行人（动态）
//   - 路外：树木、建筑、路灯 —— 雷达同样能扫到
// 平面坐标 (x, y)，y 对应 three.js 的 z。航向 ψ 从 +x 转向 +y。

import { makeRng } from './mathx.js';

export const ROAD = { cx: 0, cy: 0, hw: 34, hh: 20, r: 12, width: 6.0, laneOffset: 1.3 };

// 圆角矩形中心线，逆时针（从 ψ 的角度看是 +x → +y 方向）
export function roadCenterline(step = 0.5, R = ROAD) {
  const pts = [];
  const { hw, hh, r } = R;
  const segs = [];
  // 四条直线 + 四段圆弧，按顺序首尾相接
  const sx = hw - r, sy = hh - r;
  segs.push({ type: 'line', a: [-sx, -hh], b: [sx, -hh] });
  segs.push({ type: 'arc', c: [sx, -sy], a0: -Math.PI / 2, a1: 0 });
  segs.push({ type: 'line', a: [hw, -sy], b: [hw, sy] });
  segs.push({ type: 'arc', c: [sx, sy], a0: 0, a1: Math.PI / 2 });
  segs.push({ type: 'line', a: [sx, hh], b: [-sx, hh] });
  segs.push({ type: 'arc', c: [-sx, sy], a0: Math.PI / 2, a1: Math.PI });
  segs.push({ type: 'line', a: [-hw, sy], b: [-hw, -sy] });
  segs.push({ type: 'arc', c: [-sx, -sy], a0: Math.PI, a1: 1.5 * Math.PI });
  for (const s of segs) {
    if (s.type === 'line') {
      const L = Math.hypot(s.b[0] - s.a[0], s.b[1] - s.a[1]);
      const n = Math.max(1, Math.round(L / step));
      for (let i = 0; i < n; i++) pts.push([s.a[0] + ((s.b[0] - s.a[0]) * i) / n, s.a[1] + ((s.b[1] - s.a[1]) * i) / n]);
    } else {
      const n = Math.max(2, Math.round(((s.a1 - s.a0) * r) / step));
      for (let i = 0; i < n; i++) {
        const a = s.a0 + ((s.a1 - s.a0) * i) / n;
        pts.push([s.c[0] + r * Math.cos(a), s.c[1] + r * Math.sin(a)]);
      }
    }
  }
  return pts.map(([x, y]) => [x + R.cx, y + R.cy]);
}

// 车道线（中心线向右偏移 laneOffset；行驶方向 = 中心线方向，右侧 = 方向左转 −90°）
export function laneRoute(R = ROAD) {
  const c = roadCenterline(0.5, R);
  const n = c.length;
  return c.map((p, i) => {
    const a = c[(i - 1 + n) % n], b = c[(i + 1) % n];
    const tx = b[0] - a[0], ty = b[1] - a[1];
    const L = Math.hypot(tx, ty);
    // ψ 从 x 转向 y；“右侧”在这里为 (ty, −tx)/L 的反向？—— 从上往下看，前进 +x、右侧是 +y 方向
    // （three.js 中 z 向右），故右侧法向 = (−ty, tx)/L
    return [p[0] + (-ty / L) * R.laneOffset, p[1] + (tx / L) * R.laneOffset];
  });
}

// 到道路中心线的距离（用于判断“在路上”）
export function distToRoadCenter(x, y, R = ROAD) {
  const sx = R.hw - R.r, sy = R.hh - R.r;
  const qx = Math.abs(x - R.cx) - sx, qy = Math.abs(y - R.cy) - sy;
  // 圆角矩形 SDF 的“边界”就是中心线
  const ox = Math.max(qx, 0), oy = Math.max(qy, 0);
  const sdf = Math.hypot(ox, oy) + Math.min(Math.max(qx, qy), 0) - R.r;
  return Math.abs(sdf);
}

// ---- 障碍物 ----
// box: {kind, x, y, lx, ly, h, yaw}   cyl: {kind, x, y, r, h}   tree: cyl 树干 + 球冠
export function buildObstacles(seed = 3) {
  const rng = makeRng(seed);
  const obs = [];
  const box = (kind, x, y, lx, ly, h, yaw = 0, extra = {}) => obs.push({ shape: 'box', kind, x, y, lx, ly, h, yaw, ...extra });
  const cyl = (kind, x, y, r, h, extra = {}) => obs.push({ shape: 'cyl', kind, x, y, r, h, ...extra });

  const lane = ROAD.laneOffset;
  // 下边直道（y = −20，向 +x 行驶，右侧车道 y = −20 + 1.3）
  for (let i = 0; i < 4; i++) cyl('cone', -10 + i * 0.9, -20 + lane + 0.2 + i * 0.05, 0.17, 0.7);
  box('car', 8, -20 + lane + 0.3, 4.4, 1.85, 1.5, 0.03, { color: 0x2f6fb5 });
  // 右侧直道（x = 34，向 +y 行驶，右侧车道 x = 34 − 1.3）
  box('barrier', 34 - lane + 0.1, -2, 0.4, 2.6, 1.0, 0);
  box('crate', 34 - lane - 0.6, 5.5, 0.8, 0.8, 0.75, 0.4);
  box('crate', 34 - lane + 0.4, 6.4, 0.7, 0.7, 0.6, -0.2);
  // 上边直道（y = 20，向 −x 行驶，右侧车道 y = 20 − 1.3）
  for (let i = 0; i < 5; i++) cyl('cone', 12 - i * 1.2, 20 - lane + (i % 2 ? 0.5 : -0.5), 0.17, 0.7);
  box('van', -12, 20 - lane - 0.2, 5.0, 2.0, 2.2, -0.02, { color: 0xe8e8e8 });
  // 左侧直道（x = −34，向 −y 行驶）
  box('barrier', -34 + lane - 0.1, 3, 0.4, 2.4, 1.0, 0);

  // 行人（动态，在道路上来回横穿）
  cyl('ped', -22, -20, 0.25, 1.72, { dyn: { ax: -22, ay: -24.5, bx: -22, by: -15.5, speed: 0.9, phase: 0 } });
  cyl('ped', 22, 20, 0.25, 1.65, { dyn: { ax: 22, ay: 24.5, bx: 22, by: 15.5, speed: 0.8, phase: 0.5 } });

  // 路外：建筑
  box('building', 0, 0, 30, 10, 7, 0, { color: 0xc9c2b6 });
  box('building', -2, 34, 26, 8, 10, 0, { color: 0xb8bfc8 });
  box('building', 52, 4, 8, 22, 12, 0, { color: 0xd2c7b5 });
  box('building', -52, -6, 8, 18, 9, 0, { color: 0xa9b4bf });
  box('building', 10, -34, 18, 8, 6, 0, { color: 0xcbbfae });

  // 树：沿道路外侧 + 内侧草坪
  const trees = [];
  const tryTree = (x, y) => {
    if (distToRoadCenter(x, y) < ROAD.width / 2 + 1.8) return;
    for (const o of obs) {
      if (o.kind === 'building' && Math.abs(x - o.x) < o.lx / 2 + 2.5 && Math.abs(y - o.y) < o.ly / 2 + 2.5) return;
    }
    for (const t of trees) if (Math.hypot(t[0] - x, t[1] - y) < 4) return;
    trees.push([x, y]);
  };
  for (let i = 0; i < 260; i++) tryTree((rng() - 0.5) * 120, (rng() - 0.5) * 90);
  for (const [x, y] of trees) {
    const h = 3.2 + rng() * 2.5;
    cyl('tree', x, y, 0.18 + rng() * 0.08, h, { crown: 1.3 + rng() * 1.0 });
  }
  // 路灯（道路外缘）
  const cl = roadCenterline(14);
  cl.forEach((p, i) => {
    if (i % 2) return;
    const a = cl[(i + 1) % cl.length];
    const tx = a[0] - p[0], ty = a[1] - p[1], L = Math.hypot(tx, ty);
    const off = -(ROAD.width / 2 + 0.6);
    cyl('lamp', p[0] + (-ty / L) * off, p[1] + (tx / L) * off, 0.08, 5.5, { face: Math.atan2(ty, tx) });
  });
  return obs;
}

// 更新动态障碍位置
export function updateDynamic(obs, t) {
  for (const o of obs) {
    if (!o.dyn) continue;
    const d = o.dyn;
    const L = Math.hypot(d.bx - d.ax, d.by - d.ay);
    const period = (2 * L) / d.speed + 6; // 两端各停 3 s
    let u = ((t / period + d.phase) % 1) * period;
    let f;
    const tl = L / d.speed;
    if (u < 3) f = 0;
    else if (u < 3 + tl) f = (u - 3) / tl;
    else if (u < 6 + tl) f = 1;
    else f = 1 - (u - 6 - tl) / tl;
    o.x = d.ax + (d.bx - d.ax) * f;
    o.y = d.ay + (d.by - d.ay) * f;
    o.walking = (u > 3 && u < 3 + tl) || u > 6 + tl;
    o.heading = Math.atan2(d.by - d.ay, d.bx - d.ax) + (u > 3 + tl ? Math.PI : 0);
  }
}
