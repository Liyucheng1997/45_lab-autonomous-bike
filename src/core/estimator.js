// 传感器模型 + 状态估计。控制器在真实硬件上看不到“真值”，只能读：
//   - 6 轴 IMU：陀螺 (含零偏 + 白噪声)、加速度计 (含重力、向心加速度、横滚角加速度引起的切向加速度)
//   - 转向电机编码器 (14 bit 量化)、动量轮电机编码器、轮毂电机霍尔测速
// 估计器：带零偏估计的互补滤波 (Mahony 风格) + 向心加速度补偿，输出 [φ, δ, φ̇, δ̇, Ω, v]。

import { makeRng } from './mathx.js';

export class SensorSuite {
  constructor(p, seed = 7) {
    this.p = p;
    this.rng = makeRng(seed);
    this.bias = p.gyroBias * (this.rng() > 0.5 ? 1 : -1);
  }

  read(s, plant) {
    const p = this.p, n = this.rng.gauss, g = p.g;
    const h = p.imuHeight;
    // 机体系比力：重力 + 转弯向心加速度，再加 IMU 高于横滚轴带来的切向/向心项
    const ac = s.v * (s.psiDot || 0);
    const c = Math.cos(s.phi), si = Math.sin(s.phi);
    const fy = g * c + ac * si - h * s.phiDot * s.phiDot;
    const fz = -g * si + ac * c + h * (s.phiDD || 0);
    const q = (2 * Math.PI) / 2 ** p.steerEncBits;
    return {
      gyroX: s.phiDot + this.bias + n() * p.gyroNoise,
      accY: fy + n() * p.accNoise,
      accZ: fz + n() * p.accNoise,
      steerEnc: Math.round(s.delta / q) * q,
      rwSpeed: s.Omega + n() * 0.3,
      wheelSpeed: s.v + n() * 0.01,
    };
  }
}

export class Estimator {
  constructor(p) {
    this.p = p;
    this.reset();
  }
  reset(phi0 = 0) {
    this.phi = phi0;
    this.bias = 0;
    this.phiDot = 0;
    this.delta = 0;
    this.deltaPrev = null;
    this.deltaDot = 0;
    this.Omega = 0;
    this.v = 0;
    this.phiAcc = 0;
  }

  update(m, dt, plant) {
    const g = this.p.g;
    this.v = m.wheelSpeed;
    this.Omega = 0.7 * this.Omega + 0.3 * m.rwSpeed;
    // 转向角速度：编码器差分 + 一阶低通 (~60 Hz)
    if (this.deltaPrev === null) this.deltaPrev = m.steerEnc;
    const rawDd = (m.steerEnc - this.deltaPrev) / dt;
    this.deltaPrev = m.steerEnc;
    const a = dt / (dt + 1 / (2 * Math.PI * 60));
    this.deltaDot += a * (rawDd - this.deltaDot);
    this.delta = m.steerEnc;

    // 运动学估计向心加速度 a_c = v·ψ̇（ψ̇ 由车速 + 转向角得到），补偿加速度计
    const lam = plant.body.whipple.lambda, w = plant.body.whipple.w;
    const psiDot = (this.v * Math.tan(this.delta) * Math.cos(lam)) / w;
    const ac = this.v * psiDot;
    this.phiAcc = Math.atan2(ac, g) - Math.atan2(m.accZ, m.accY);

    // 互补滤波 + 零偏积分
    const kp = 1.5, ki = 0.25;
    const err = this.phiAcc - this.phi;
    this.bias -= ki * err * dt;
    const rate = m.gyroX - this.bias;
    this.phi += (rate + kp * err) * dt;
    this.phiDot = rate;
    return this;
  }
}
