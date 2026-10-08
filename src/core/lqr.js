// LQR 求解：连续系统 (A, B) → 零阶保持离散化 → 离散代数 Riccati 方程 (DARE)。
// DARE 用结构保持倍增算法 (SDA, Structure-preserving Doubling Algorithm) 求解，
// 二次收敛，~30 次迭代即可，比朴素 Riccati 迭代快几个数量级，可在浏览器里实时做增益调度。

import { mat } from './mathx.js';

// 零阶保持离散化：Ad = e^{A dt}，Bd = ∫0^dt e^{Aτ} dτ · B（级数展开，小 dt 下快速收敛）
export function discretize(A, B, dt) {
  const n = A.length;
  const Adt = mat.scale(A, dt);
  let term = mat.eye(n);
  let Ad = mat.eye(n);
  let intTerm = mat.scale(mat.eye(n), dt);
  let intSum = mat.copy(intTerm);
  for (let k = 1; k < 24; k++) {
    term = mat.scale(mat.mul(term, Adt), 1 / k);
    Ad = mat.add(Ad, term);
    intTerm = mat.scale(mat.mul(intTerm, Adt), 1 / (k + 1));
    intSum = mat.add(intSum, intTerm);
    if (mat.fro(term) < 1e-16) break;
  }
  return { Ad, Bd: mat.mul(intSum, B) };
}

// SDA 求解 DARE: P = AᵀPA − AᵀPB(R+BᵀPB)⁻¹BᵀPA + Q
export function solveDARE(Ad, Bd, Q, R) {
  const n = Ad.length;
  const I = mat.eye(n);
  let Ak = mat.copy(Ad);
  let Gk = mat.mul(mat.mul(Bd, mat.inv(R)), mat.T(Bd));
  let Hk = mat.copy(Q);
  for (let it = 0; it < 80; it++) {
    const W = mat.inv(mat.add(I, mat.mul(Gk, Hk)));
    const AW = mat.mul(Ak, W);
    const An = mat.mul(AW, Ak);
    const Gn = mat.add(Gk, mat.mul(mat.mul(AW, Gk), mat.T(Ak)));
    const Hn = mat.add(Hk, mat.mul(mat.mul(mat.T(Ak), Hk), mat.mul(W, Ak)));
    const diff = mat.fro(mat.sub(Hn, Hk)) / (1 + mat.fro(Hn));
    Ak = An; Gk = Gn; Hk = Hn;
    if (diff < 1e-13) break;
  }
  // 数值对称化
  return Hk.map((row, i) => row.map((v, j) => 0.5 * (v + Hk[j][i])));
}

// 返回反馈增益 K (m×n)，控制律 u = −K x
export function lqr(A, B, Qdiag, Rdiag, dt) {
  const { Ad, Bd } = discretize(A, B, dt);
  const Q = mat.diag(Qdiag);
  const R = mat.diag(Rdiag);
  const P = solveDARE(Ad, Bd, Q, R);
  const BtP = mat.mul(mat.T(Bd), P);
  const S = mat.add(R, mat.mul(BtP, Bd));
  return mat.mul(mat.inv(S), mat.mul(BtP, Ad));
}
