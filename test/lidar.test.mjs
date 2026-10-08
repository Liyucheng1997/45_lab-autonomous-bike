// 分桶加速的激光雷达 vs 暴力全遍历：逐束比较最近命中距离，必须完全一致
import { Lidar, intersect } from '../src/core/lidar.js';
import { buildObstacles } from '../src/core/world.js';
const obs = buildObstacles();
const poses = [{ x: -2, y: -18.7, psi: 0, phi: 0.03 }, { x: 30, y: 10, psi: 1.7, phi: -0.25 }, { x: -20, y: 18, psi: 3.1, phi: 0.15 }, { x: 0, y: 0, psi: -2, phi: 0.4 }];
let bad = 0, total = 0;
for (const pose of poses) { const bad0 = bad;
  const L = new Lidar({ noise: 0 });
  L.scan(pose, obs);
  const [ox, oy, oz] = L.origin;
  const cps = Math.cos(pose.psi), sps = Math.sin(pose.psi), cph = Math.cos(pose.phi), sph = Math.sin(pose.phi);
  let k = 0;
  for (let c = 0; c < L.channels; c++) for (let a = 0; a < L.azSteps; a++) {
    const el = L.elev[c], az = (a / L.azSteps) * 2 * Math.PI + c * 0.003;
    const dx0 = Math.cos(el) * Math.cos(az), dy0 = Math.sin(el), dz0 = Math.cos(el) * Math.sin(az);
    const dy1 = dy0 * cph - dz0 * sph, dz1 = dy0 * sph + dz0 * cph;
    const dx = dx0 * cps - dz1 * sps, dz = dx0 * sps + dz1 * cps, dy = dy1;
    let t = L.range, kind = 0;
    if (dy < -1e-4 && -oy / dy < t) { t = -oy / dy; kind = 1; }
    for (const o of obs) { const ti = intersect(o, ox, oy, oz, dx, dy, dz, t); if (ti < t) { t = ti; kind = 2; } }
    total++;
    if (kind !== L.hitKind[c * L.azSteps + a]) { bad++; continue; }
    if (kind) {
      const e = Math.hypot(L.points[3 * k] - (ox + dx * t), L.points[3 * k + 1] - (oy + dy * t), L.points[3 * k + 2] - (oz + dz * t));
      if (e > 1e-4) bad++; // 点云以 Float32 存储，30 m 处精度约 2e-6
      k++;
    }
  }
  console.log('pose', JSON.stringify(pose), 'mismatch', bad - bad0);
}
console.log(`rays ${total}, mismatches ${bad}`);
const t0 = performance.now(); const L = new Lidar(); for (let i = 0; i < 30; i++) L.scan(poses[1], obs);
console.log('scan ms', ((performance.now() - t0) / 30).toFixed(2));
if (bad) process.exit(1);
