// 校验 Whipple 矩阵与论文基准值一致，并找出 weave/capsize 临界速度。
import { whippleMatrices, BENCHMARK } from '../src/core/whipple.js';
import { mat, eigenvalues } from '../src/core/mathx.js';

const ref = {
  M: [[80.81722, 2.31941332208709], [2.31941332208709, 0.29784188199686]],
  C1: [[0, 33.86641391492494], [-0.85035641456978, 1.68540397397560]],
  K0: [[-80.95, -2.59951685249872], [-2.59951685249872, -0.80329488458618]],
  K2: [[0, 76.59734589573222], [0, 2.65431523794604]],
};
const m = whippleMatrices(BENCHMARK);
let worst = 0;
for (const k of ['M', 'C1', 'K0', 'K2'])
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++)
    worst = Math.max(worst, Math.abs(m[k][i][j] - ref[k][i][j]));
console.log('max |err| vs benchmark =', worst.toExponential(2));
if (worst > 1e-6) { console.error('FAIL'); process.exit(1); }

function stateA(m, v, g = 9.81) {
  const Mi = mat.inv(m.M);
  const K = mat.add(mat.scale(m.K0, g), mat.scale(m.K2, v * v));
  const a = mat.scale(mat.mul(Mi, K), -1), b = mat.scale(mat.mul(Mi, m.C1), -v);
  return [[0, 0, 1, 0], [0, 0, 0, 1], [...a[0], ...b[0]], [...a[1], ...b[1]]];
}
// 自稳定区间：论文 weave 速度 4.292 m/s，capsize 速度 6.024 m/s
let lo = null, hi = null;
for (let v = 0; v <= 10; v += 0.001) {
  const st = eigenvalues(stateA(m, v)).every((e) => e.re < 0);
  if (st && lo === null) lo = v;
  if (!st && lo !== null && hi === null) hi = v;
}
console.log('self-stable v ∈', lo?.toFixed(3), '…', hi?.toFixed(3), '(paper: 4.292 … 6.024)');
if (Math.abs(lo - 4.292) > 0.01 || Math.abs(hi - 6.024) > 0.01) { console.error('FAIL'); process.exit(1); }
console.log('PASS');
