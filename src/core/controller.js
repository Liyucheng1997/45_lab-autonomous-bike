// 底层运动控制（对应真车 MCU 上 500 Hz 的实时环）：
//   1) 平衡 + 转向：增益调度 LQR，u = u_ff − K(v)·(x − x_ref)，同时输出动量轮力矩和转向力矩。
//      低速时主要靠动量轮；速度升高后 LQR 自然把平衡任务交给转向（K 随 v 变化），
//      并会自动产生“反打方向 (countersteer)”来建立转弯侧倾 —— 这和人骑车一模一样。
//   2) 转弯：上层给曲率 κ → 稳态转弯解 [φ_ref, δ_ref, T_δ,ff]，并做速率限制。
//   3) 车速：PI 控制轮毂电机力矩 + 电子刹车。

import { clamp } from './mathx.js';
import { gainAt, steadyTurn } from './plant.js';

export class MotionController {
  constructor(plant) {
    this.setPlant(plant);
    this.reset();
  }
  setPlant(plant) {
    this.plant = plant;
    this.p = plant.p;
  }
  reset() {
    this.kappaCmd = 0;    // 期望曲率 (1/m)
    this.vCmd = 0;        // 期望车速
    this.brakeCmd = 0;    // 0..1 紧急刹车
    this.kappaRef = 0;
    this.vRef = 0;
    this.vInt = 0;
    this.ref = { phi: 0, delta: 0, Td: 0 };
    this.K = null;
    this.out = { tauRW: 0, Tdelta: 0, drive: 0, brake: 0 };
  }

  // x̂ = 估计状态 {phi, delta, phiDot, deltaDot, Omega, v}
  update(xh, dt) {
    const p = this.p, plant = this.plant;
    const v = xh.v;

    // --- 车速参考：加速度限幅 ---
    const dv = clamp(this.vCmd - this.vRef, -p.accelMax * 1.5 * dt, p.accelMax * dt);
    this.vRef += dv;

    // --- 曲率参考：按当前车速可承受的侧倾限幅 + 速率限幅 ---
    const latMax = p.latAccMax;
    const kMaxDyn = latMax / Math.max(v * v, 0.5);
    const kMaxGeo = Math.tan((p.steerMaxDeg - 6) * Math.PI / 180 * Math.cos(plant.body.whipple.lambda)) / plant.body.whipple.w;
    const kLim = Math.min(kMaxDyn, kMaxGeo);
    const kTarget = clamp(this.kappaCmd, -kLim, kLim);
    const kRate = 0.6 + 0.25 * Math.max(v, 0);
    this.kappaRef += clamp(kTarget - this.kappaRef, -kRate * dt, kRate * dt);
    this.ref = steadyTurn(plant, this.kappaRef, Math.max(v, 0));

    // --- 平衡 LQR ---
    const K = (this.K = gainAt(plant, v));
    const e = [
      xh.phi - this.ref.phi,
      xh.delta - this.ref.delta,
      xh.phiDot,
      xh.deltaDot,
      xh.Omega,
    ];
    let tauRW = -(K[0][0] * e[0] + K[0][1] * e[1] + K[0][2] * e[2] + K[0][3] * e[3] + K[0][4] * e[4]);
    let Tdelta = this.ref.Td - (K[1][0] * e[0] + K[1][1] * e[1] + K[1][2] * e[2] + K[1][3] * e[3] + K[1][4] * e[4]);

    // --- 车速 PI ---
    const ev = this.vRef - v;
    this.vInt = clamp(this.vInt + ev * dt, -3, 3);
    let drive = 40 * ev + 25 * this.vInt + (this.vRef > 0.05 ? 3.0 : this.vRef < -0.05 ? -3.0 : 0);
    let brake = 0;
    if (this.brakeCmd > 0) {
      brake = this.brakeCmd;
      drive = 0;
      this.vInt = 0;
      this.vRef = Math.min(this.vRef, Math.max(v, 0));
    } else if (ev < -0.4 && v > 0.3) {
      brake = clamp(-ev * 0.4, 0, 0.6);
    }

    this.out = { tauRW, Tdelta, drive, brake };
    return this.out;
  }
}
