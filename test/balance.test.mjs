// 平衡控制闭环测试（含 IMU 噪声 + 估计器）。每项都带“期望结果”：
// 静止时动量轮力矩有限，0.5 rad/s 的侧向冲击超出物理极限，必须倒 —— 这正说明模型没有作弊。
import { makeSim } from './harness.mjs';
const deg = (r) => (r * 180 / Math.PI).toFixed(2);
let failed = 0;
function check(name, sim, expectFall, extra = '') {
  const st = sim.stats(), s = sim.s;
  const ok = sim.fallen === expectFall;
  if (!ok) failed++;
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name.padEnd(34)} fallen=${sim.fallen} max|φ|=${deg(st.maxPhi).padStart(6)}° max|Ω|=${st.maxOm.toFixed(0).padStart(3)} max|τ|=${st.maxTau.toFixed(1).padStart(4)} ${extra}`);
}
for (const est of [false, true]) {
  console.log(`--- ${est ? '传感器 + 估计器' : '理想传感器'}`);
  let sim = makeSim({ useEstimator: est }).run(8);
  check('静止，初始倾角 1.7°', sim, false);
  for (const [kick, fall] of [[0.2, false], [0.35, false], [0.5, true]]) {
    sim = makeSim({ useEstimator: est }).run(3);
    sim.s = { ...sim.s, phiDot: sim.s.phiDot + kick };
    sim.resetStats(); sim.run(6);
    check(`静止 + 侧向冲击 ${kick} rad/s`, sim, fall, fall ? '(超出力矩极限，应倒)' : '');
  }
  for (const v of [1, 2, 3.2, 5, 7]) {
    sim = makeSim({ useEstimator: est });
    sim.ctl.vCmd = v; sim.run(14);
    sim.resetStats();
    sim.s = { ...sim.s, phiDot: sim.s.phiDot + 0.6 };
    sim.run(5);
    check(`${v} m/s 行驶 + 冲击 0.6 rad/s`, sim, false);
  }
  for (const [v, R] of [[1, 3], [2, 5], [3.2, 8], [5, 15]]) {
    sim = makeSim({ useEstimator: est });
    sim.ctl.vCmd = v; sim.run(10);
    sim.ctl.kappaCmd = 1 / R; sim.resetStats(); sim.run(10);
    const rAct = sim.s.v / sim.s.psiDot;
    const err = Math.abs(rAct - R) / R;
    if (err > 0.05) failed++;
    check(`转弯 v=${v} R=${R}`, sim, false, `实际 R=${rAct.toFixed(2)} 侧倾 ${deg(sim.s.phi)}°`);
  }
  sim = makeSim({ useEstimator: est }); sim.ctl.vCmd = 4; sim.run(10); sim.ctl.vCmd = 0; sim.ctl.brakeCmd = 1; sim.resetStats(); sim.run(8);
  check('4 m/s 紧急制动到静止', sim, false, `v=${sim.s.v.toFixed(2)}`);
}
if (failed) { console.error(`${failed} FAILED`); process.exit(1); }
console.log('ALL PASS');
