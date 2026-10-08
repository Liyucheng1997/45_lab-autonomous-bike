// 整车闭环：环线巡航一圈（含障碍、行人），统计是否倒地/碰撞/完成。
import { params } from '../src/core/params.js';
import { Simulation } from '../src/core/simulation.js';
const sim = new Simulation({ ...params });
sim.autopilot.setMode('ROUTE');
const T = Number(process.argv[2] || 120);
let minClear = Infinity, lastLog = -10, dist = 0, prev = [sim.s.x, sim.s.y], estops = 0, wasE = false;
const t0 = performance.now();
const obsStatic = sim.obstacles.filter(o => o.kind !== 'tree' && o.kind !== 'building' && o.kind !== 'lamp');
while (sim.t < T && !sim.fallen) {
  sim.advance(0.02);
  const s = sim.s;
  dist += Math.hypot(s.x - prev[0], s.y - prev[1]); prev = [s.x, s.y];
  for (const o of sim.obstacles) {
    let d;
    if (o.shape === 'cyl') d = Math.hypot(o.x - s.x, o.y - s.y) - o.r;
    else { const c = Math.cos(o.yaw), sn = Math.sin(o.yaw); const lx = (s.x-o.x)*c + (s.y-o.y)*sn, ly = -(s.x-o.x)*sn + (s.y-o.y)*c;
      d = Math.hypot(Math.max(Math.abs(lx)-o.lx/2,0), Math.max(Math.abs(ly)-o.ly/2,0)); }
    // 车身是 1.7 m 长的线段，这里用后轮点 + 前轮点近似
    const fx = s.x + 1.04*Math.cos(s.psi), fy = s.y + 1.04*Math.sin(s.psi);
    let d2;
    if (o.shape === 'cyl') d2 = Math.hypot(o.x - fx, o.y - fy) - o.r; else d2 = d;
    minClear = Math.min(minClear, d, d2);
  }
  if (sim.autopilot.emergency && !wasE) estops++;
  wasE = sim.autopilot.emergency;
  if (sim.t - lastLog >= 5) {
    lastLog = sim.t;
    const ap = sim.autopilot;
    console.log(`t=${sim.t.toFixed(0).padStart(3)} pos=(${s.x.toFixed(1)},${s.y.toFixed(1)}) v=${s.v.toFixed(2)} φ=${(s.phi*57.3).toFixed(1)}° δ=${(s.delta*57.3).toFixed(1)}° Ω=${s.Omega.toFixed(0)} ${ap.status} plan=${ap.planMs.toFixed(1)}ms`);
  }
}
const wall = (performance.now() - t0) / 1000;
// 通过条件：不倒、车身与障碍保持 > 0.2 m（行人主动走近停着的车除外）、平均速度 > 2.5 m/s
const ok = !sim.fallen && minClear > 0.2 && dist > 2.5 * T;
console.log(`\n${ok ? 'PASS' : 'FAIL'} fallen=${sim.fallen} dist=${dist.toFixed(1)} m  minClear=${minClear.toFixed(2)} m  estops=${estops}  maxLean=${(sim.maxLean*57.3).toFixed(1)}°  energy=${(sim.energy/3600).toFixed(2)} Wh  wall=${wall.toFixed(1)}s for ${sim.t.toFixed(0)}s sim`);
if (!ok) process.exit(1);
