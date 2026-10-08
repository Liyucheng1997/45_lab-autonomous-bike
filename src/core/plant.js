// 被控对象：Whipple 自行车 + 动量轮。
//
// 广义坐标 q = [φ, δ]，外加飞轮相对车架的转速 Ω（独立自由度）。用拉格朗日法把飞轮自转惯量 Iw
// 从车身横滚惯量里拆出来后：
//   M' q̈ + v·C1 q̇ + (g·K0 + v²·K2) q + D q̇ = [−τ_rw, T_δ]ᵀ,     M' = M − diag(Iw, 0)
//   Iw (φ̈ + Ω̇) = τ_rw − b_w Ω
// τ_rw 是电机作用在飞轮上的力矩，其反作用 −τ_rw 作用在车身横滚上 —— 这就是动量轮平衡的原理。
//
// 线性化状态 x = [φ, δ, φ̇, δ̇, Ω]，输入 u = [τ_rw, T_δ]，用于 LQR 设计；
// 非线性仿真见 physics.js。

import { mat } from './mathx.js';
import { whippleMatrices } from './whipple.js';
import { bodyParams } from './hardware.js';
import { lqr } from './lqr.js';

export function buildPlant(p) {
  const body = bodyParams(p);
  const W = whippleMatrices(body.whipple);
  const Iw = body.Iw;
  const Mp = mat.copy(W.M);
  Mp[0][0] -= Iw;
  const Mpi = mat.inv(Mp);
  const wp = body.whipple;
  // 纵向等效质量（含两个车轮的转动惯量）
  const mEff = W.mT + wp.IRyy / wp.rR ** 2 + wp.IFyy / wp.rF ** 2;
  const plant = { p, body, W, Iw, Mp, Mpi, mEff, mT: W.mT, cgHeight: -W.zT, gains: null };
  plant.gains = gainSchedule(plant);
  return plant;
}

// 在速度 v 处的线性模型
export function linearize(plant, v) {
  const { W, Mpi, Iw, p } = plant;
  const g = p.g;
  const K = mat.add(mat.scale(W.K0, g), mat.scale(W.K2, v * v));
  const Cd = mat.add(mat.scale(W.C1, v), [[0, 0], [0, p.steerDamping]]);
  const a = mat.scale(mat.mul(Mpi, K), -1);   // ∂q̈/∂q
  const b = mat.scale(mat.mul(Mpi, Cd), -1);  // ∂q̈/∂q̇
  const bu = mat.mul(Mpi, [[-1, 0], [0, 1]]); // ∂q̈/∂u
  const bw = p.rwFriction;
  const A = [
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [a[0][0], a[0][1], b[0][0], b[0][1], (bw * Mpi[0][0]) ],
    [a[1][0], a[1][1], b[1][0], b[1][1], (bw * Mpi[1][0]) ],
    [0, 0, 0, 0, 0],
  ];
  // Ω̇ = (τ − bw Ω)/Iw − φ̈
  for (let j = 0; j < 5; j++) A[4][j] = -A[2][j];
  A[4][4] += -bw / Iw;
  const B = [
    [0, 0],
    [0, 0],
    [bu[0][0], bu[0][1]],
    [bu[1][0], bu[1][1]],
    [1 / Iw - bu[0][0], -bu[0][1]],
  ];
  return { A, B };
}

// 增益调度表：在一组速度上离线求 LQR，运行时线性插值。
export const GAIN_SPEEDS = Array.from({ length: 47 }, (_, i) => -1.5 + i * 0.25); // −1.5 … 10 m/s

export function gainSchedule(plant) {
  const { p } = plant;
  return GAIN_SPEEDS.map((v) => {
    const { A, B } = linearize(plant, v);
    return { v, K: lqr(A, B, p.Q, p.R, p.ctrlDt) };
  });
}

export function gainAt(plant, v) {
  const tab = plant.gains;
  const v0 = tab[0].v, dv = tab[1].v - tab[0].v;
  const f = Math.min(Math.max((v - v0) / dv, 0), tab.length - 1.000001);
  const i = Math.floor(f), t = f - i;
  const A = tab[i].K, B = tab[i + 1].K;
  return A.map((row, r) => row.map((k, c) => k + (B[r][c] - k) * t));
}

// 稳态转弯参考：给定曲率 κ 和速度 v，求使横滚方程平衡（τ_rw = 0）的 [φ_ref, δ_ref] 以及所需转向力矩前馈。
export function steadyTurn(plant, kappa, v) {
  const { W, body, p } = plant;
  const g = p.g, lam = body.whipple.lambda, w = body.whipple.w;
  // 地面转角 δg = atan(κ w)，再换算到车把转角
  const dg = Math.atan(kappa * w);
  let delta = Math.atan(Math.tan(dg) / Math.cos(lam));
  const kφφ = g * W.K0[0][0];
  const kφδ = g * W.K0[0][1] + v * v * W.K2[0][1];
  let s = (-kφδ * delta) / kφφ;
  s = Math.max(-0.9, Math.min(0.9, s));
  const phi = Math.asin(s);
  const Td = g * W.K0[1][0] * phi + (g * W.K0[1][1] + v * v * W.K2[1][1]) * delta;
  return { phi, delta, Td };
}
