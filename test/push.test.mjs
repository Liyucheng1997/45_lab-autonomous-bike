// 侧向冲量恢复能力。静止时只靠动量轮（力矩有限）；行驶时转向接管平衡，能扛的冲量大得多。
// 这里的数值也是 UI 上“推一把”使用的强度（静止 3 / 低速 6 / 巡航 12 N·s）。
import { params } from '../src/core/params.js';
import { Simulation } from '../src/core/simulation.js';
let failed = 0;
const cases = [
  [0, 3, false], [0, -3, false], [0, 6, true],         // 静止：6 N·s 超出动量轮极限，必须倒
  [1.5, 6, false], [1.5, -6, false],
  [3.2, 12, false], [3.2, -12, false], [5, 15, false],
];
for (const [v, J, expectFall] of cases) {
  const sim = new Simulation({ ...params });
  sim.obstacles.length = 0; // 空旷路面，避免自动刹车干扰
  sim.autopilot.setMode('MANUAL');
  sim.autopilot.manual.v = v;
  sim.advance(v ? 8 : 3);
  sim.push(J);
  sim.advance(5);
  const ok = sim.fallen === expectFall;
  if (!ok) failed++;
  console.log(`${ok ? 'PASS' : 'FAIL'} v=${v} m/s push ${J} N·s → fallen=${sim.fallen} (expect ${expectFall}) maxLean=${(sim.maxLean * 57.3).toFixed(1)}°`);
}
if (failed) process.exit(1);
