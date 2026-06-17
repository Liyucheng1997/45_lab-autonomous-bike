// 自行车动力学 + RK4 积分。
// 状态向量 s = {
//   theta, thetaDot,   // 横滚角 / 角速度 (rad, rad/s) —— 倒立摆
//   omega,             // 动量轮转速 (rad/s)
//   psi, x, y,         // 航向 / 地面位置
//   v, delta,          // 前进速度 / 当前转向角
//   rearSpin, frontSpin// 车轮可视旋转角 (仅用于渲染)
// }
// 输入 u = { tau (动量轮力矩), deltaCmd (转向目标), vCmd (速度目标) }

import { params as P } from './params.js';

export function initialState() {
  return {
    theta: 0.04,     // 给一点初始倾角，便于观察平衡起效
    thetaDot: 0,
    omega: 0,
    psi: 0,
    x: 0,
    y: 0,
    v: 0,
    delta: 0,
    rearSpin: 0,
    frontSpin: 0,
  };
}

// 计算状态导数。u 为已经过限幅的执行器指令。
function deriv(s, u, p) {
  const sinT = Math.sin(s.theta);
  const cosT = Math.cos(s.theta);

  // 动量轮提供的反作用力矩（作用在车身上，方向与电机力矩相反）。
  const uWheel = u.tau;

  // 转向产生的回正力矩（仅在有速度时显著）：转向打入跌倒方向产生离心反力使车身回正。
  const tauSteer = -p.m * p.h * (s.v * s.v / p.b) * Math.tan(s.delta) * cosT;

  // 横滚动力学: Ip·θ̈ = m g l sinθ - u_wheel + τ_steer
  const thetaDDot = (p.m * p.g * p.h * sinT - uWheel + tauSteer) / p.Ip;

  // 动量轮: Iw·Ω̇ = u_wheel
  const omegaDot = uWheel / p.Iw;

  // 速度 / 转向：一阶执行器
  const vDot = (u.vCmd - s.v) / p.vTau;
  const deltaDot = clamp((u.deltaCmd - s.delta) / p.steerTau, -p.steerRateMax, p.steerRateMax);

  // 航向 / 位置（自行车运动学）
  const psiDot = (s.v / p.b) * Math.tan(s.delta) * cosT;
  const xDot = s.v * Math.cos(s.psi);
  const yDot = s.v * Math.sin(s.psi);

  // 车轮可视旋转
  const rearSpinDot = s.v / 0.34;  // 后轮半径 ~0.34 m
  const frontSpinDot = s.v / 0.34;

  return {
    theta: s.thetaDot,
    thetaDot: thetaDDot,
    omega: omegaDot,
    psi: psiDot,
    x: xDot,
    y: yDot,
    v: vDot,
    delta: deltaDot,
    rearSpin: rearSpinDot,
    frontSpin: frontSpinDot,
  };
}

function addScaled(s, ds, h) {
  const out = {};
  for (const k in s) out[k] = s[k] + (ds[k] || 0) * h;
  return out;
}

// 一步 RK4
export function step(s, u, dt, p = P) {
  const k1 = deriv(s, u, p);
  const k2 = deriv(addScaled(s, k1, dt / 2), u, p);
  const k3 = deriv(addScaled(s, k2, dt / 2), u, p);
  const k4 = deriv(addScaled(s, k3, dt), u, p);
  const out = {};
  for (const key in s) {
    out[key] = s[key] + (dt / 6) * ((k1[key] || 0) + 2 * (k2[key] || 0) + 2 * (k3[key] || 0) + (k4[key] || 0));
  }
  // 动量轮转速饱和
  out.omega = clamp(out.omega, -p.wheelSpeedMax, p.wheelSpeedMax);
  return out;
}

export function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

// 线性化倒立摆 + 动量轮，给 LQR 用。状态 [θ, θ̇, Ω]。
// θ̈ = (mgl/Ip)θ - u/Ip ;  Ω̇ = u/Iw
export function linearModel(p = P) {
  const A = [
    [0, 1, 0],
    [(p.m * p.g * p.h) / p.Ip, 0, 0],
    [0, 0, 0],
  ];
  const B = [[0], [-1 / p.Ip], [1 / p.Iw]];
  return { A, B };
}
