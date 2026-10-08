// 非线性仿真：Whipple 横滚/转向动力学 + 动量轮 + 纵向动力学 + 平面运动学，RK4 积分。
//
// 相对线性模型保留的非线性：
//  - 重力倾覆项用 sin φ（大角度倒下时正确）
//  - 地面转角 δg = atan(tan δ cos λ / cos φ)，航向 ψ̇ = v tan δg / w
//  - 执行器：电机力矩饱和、动量轮反电动势（转速越高可用力矩越小）、转向机械限位、轮毂电机功率限制
//
// 状态 s: { x, y, psi, phi, delta, phiDot, deltaDot, Omega, v, rwAngle, rearAngle, frontAngle, odo }

import { clamp } from './mathx.js';

export function initialState(x = 0, y = 0, psi = 0) {
  return {
    x, y, psi,
    phi: 0.03, delta: 0, phiDot: 0, deltaDot: 0,
    Omega: 0, v: 0,
    rwAngle: 0, rearAngle: 0, frontAngle: 0, odo: 0,
  };
}

// 执行器层：把控制器指令变成真实可用的力/力矩（硬件限制在这里体现）
export function actuate(plant, s, cmd) {
  const p = plant.p;
  // 动量轮：力矩–转速特性。与转速同向加速时可用力矩随 |Ω| 线性下降，反向制动可用全力矩。
  let tau = clamp(cmd.tauRW, -p.rwTorqueMax, p.rwTorqueMax);
  if (tau * s.Omega > 0) {
    const avail = p.rwTorqueMax * Math.max(0, 1 - Math.abs(s.Omega) / p.rwSpeedMax);
    tau = clamp(tau, -avail, avail);
  }
  const Td = clamp(cmd.Tdelta, -p.steerTorqueMax, p.steerTorqueMax);
  // 轮毂电机：力矩上限 + 功率上限；刹车单独
  let F = clamp(cmd.drive, -p.driveTorqueMax, p.driveTorqueMax) / plant.body.whipple.rR;
  const vAbs = Math.max(Math.abs(s.v), 0.5);
  F = clamp(F, -p.drivePowerMax / vAbs, p.drivePowerMax / vAbs);
  const brake = clamp(cmd.brake || 0, 0, 1) * p.brakeDecelMax * plant.mT;
  return { tau, Td, F, brake };
}

function deriv(plant, s, a, dist) {
  const { W, Mpi, Iw, p, mEff, mT, body } = plant;
  const g = p.g, v = s.v;
  const lam = body.whipple.lambda, w = body.whipple.w, rR = body.whipple.rR;

  // 广义力 = 执行器 + 扰动 − (v C1 q̇ + (gK0 + v²K2) q + D q̇)
  const sphi = Math.sin(s.phi);
  const kq0 = g * W.K0[0][0] * sphi + (g * W.K0[0][1] + v * v * W.K2[0][1]) * s.delta;
  const kq1 = g * W.K0[1][0] * s.phi + (g * W.K0[1][1] + v * v * W.K2[1][1]) * s.delta;
  const cq0 = v * (W.C1[0][0] * s.phiDot + W.C1[0][1] * s.deltaDot);
  const cq1 = v * (W.C1[1][0] * s.phiDot + W.C1[1][1] * s.deltaDot) + p.steerDamping * s.deltaDot;
  const tauNet = a.tau - p.rwFriction * s.Omega;

  // 转向机械限位：刚性弹簧–阻尼
  const dMax = (p.steerMaxDeg * Math.PI) / 180;
  let stop = 0;
  if (s.delta > dMax) stop = -800 * (s.delta - dMax) - 20 * s.deltaDot;
  else if (s.delta < -dMax) stop = -800 * (s.delta + dMax) - 20 * s.deltaDot;

  const f0 = -tauNet + dist.Tphi - cq0 - kq0;
  const f1 = a.Td + stop + dist.Tdelta - cq1 - kq1;
  const phiDD = Mpi[0][0] * f0 + Mpi[0][1] * f1;
  const deltaDD = Mpi[1][0] * f0 + Mpi[1][1] * f1;
  const OmegaDot = tauNet / Iw - phiDD;

  // 纵向：驱动 − 滚阻 − 风阻 − 刹车
  const sgn = Math.abs(v) > 0.02 ? Math.sign(v) : 0;
  const Frr = p.Crr * mT * g * sgn;
  const Faero = 0.5 * 1.2 * p.CdA * v * Math.abs(v);
  let Fbrake = a.brake * sgn;
  let vDot = (a.F - Frr - Faero - Fbrake) / mEff;
  if (sgn === 0 && Math.abs(a.F) < p.Crr * mT * g + a.brake) vDot = -v * 20; // 静摩擦：停住

  // 平面运动学（后轮触地点无侧滑）
  const cphi = Math.max(Math.cos(s.phi), 0.2);
  const dg = Math.atan((Math.tan(s.delta) * Math.cos(lam)) / cphi);
  const psiDot = (v * Math.tan(dg)) / w;
  const rF = body.whipple.rF;
  return {
    x: v * Math.cos(s.psi), y: v * Math.sin(s.psi), psi: psiDot,
    phi: s.phiDot, delta: s.deltaDot, phiDot: phiDD, deltaDot: deltaDD,
    Omega: OmegaDot, v: vDot,
    rwAngle: s.Omega, rearAngle: v / rR, frontAngle: (v / rF) / Math.max(Math.cos(dg), 0.3),
    odo: Math.abs(v),
    // 附带输出（不积分）
    _psiDot: psiDot, _phiDD: phiDD,
  };
}

const KEYS = ['x', 'y', 'psi', 'phi', 'delta', 'phiDot', 'deltaDot', 'Omega', 'v', 'rwAngle', 'rearAngle', 'frontAngle', 'odo'];

function addScaled(s, d, h) {
  const o = {};
  for (const k of KEYS) o[k] = s[k] + d[k] * h;
  return o;
}

const NO_DIST = { Tphi: 0, Tdelta: 0 };

// 一步 RK4，返回新状态，附带 psiDot / phiDD（给传感器模型用）
export function step(plant, s, a, dt, dist = NO_DIST) {
  const k1 = deriv(plant, s, a, dist);
  const k2 = deriv(plant, addScaled(s, k1, dt / 2), a, dist);
  const k3 = deriv(plant, addScaled(s, k2, dt / 2), a, dist);
  const k4 = deriv(plant, addScaled(s, k3, dt), a, dist);
  const o = {};
  for (const k of KEYS) o[k] = s[k] + (dt / 6) * (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]);
  o.psiDot = k4._psiDot;
  o.phiDD = (k1._phiDD + 2 * k2._phiDD + 2 * k3._phiDD + k4._phiDD) / 6;
  return o;
}
