// 控制器：复刻 XUAN 的思路
//  1) 动量轮 LQR 状态反馈 —— 主平衡（静止/低速最关键）。
//  2) 转向控制 —— 高速平衡辅助 + 路径跟踪（航点/航向）。
//  3) 驱动 —— 跟踪目标速度。

import { lqrGain } from './lqr.js';
import { linearModel, clamp } from './physics.js';

export class Controller {
  constructor(params) {
    this.p = params;
    this.K = [0, 0, 0];
    this.recomputeGain();
    this.waypoints = [];   // [{x,y}, ...] 路径跟踪目标
    this.wpIndex = 0;
  }

  // 参数变化（质量/惯量/Q/R）后重新求解 LQR 增益。
  recomputeGain() {
    const { A, B } = linearModel(this.p);
    try {
      this.K = lqrGain(A, B, this.p.Q, this.p.R, 0.001);
    } catch (e) {
      console.warn('LQR 求解失败, 保留旧增益:', e.message);
    }
  }

  setWaypoints(pts) {
    this.waypoints = pts || [];
    this.wpIndex = 0;
  }

  // 计算本步执行器指令。
  compute(s) {
    const p = this.p;

    // --- 动量轮 LQR ---
    let tau = -(this.K[0] * s.theta + this.K[1] * s.thetaDot + this.K[2] * s.omega);
    tau = clamp(tau, -p.wheelTorqueMax, p.wheelTorqueMax);
    // 转速接近饱和时减弱力矩，避免“撞墙”后失稳
    if (Math.abs(s.omega) > 0.92 * p.wheelSpeedMax && Math.sign(tau) === Math.sign(s.omega)) {
      tau *= 0.2;
    }

    // --- 转向：平衡辅助（随速度淡入）+ 路径跟踪 ---
    const speedBlend = clamp((Math.abs(s.v) - p.steerBlendSpeed) / 1.5, 0, 1);
    // 平衡辅助：朝跌倒方向打舵（θ>0 向右倒 → 向右打舵回正）
    const deltaBalance = speedBlend * (p.steerBalanceKp * s.theta + p.steerBalanceKd * s.thetaDot);

    // 路径跟踪：朝目标航点的航向做 PD
    let deltaPath = 0;
    const target = this.currentTarget(s);
    if (target && Math.abs(s.v) > 0.05) {
      let desiredPsi = Math.atan2(target.y - s.y, target.x - s.x);
      let headingErr = wrapAngle(desiredPsi - s.psi);
      // 横向偏差也并入（简单 Stanley 风格）
      deltaPath = p.steerHeadingKp * headingErr - p.steerHeadingKd * (s.v / p.b) * Math.tan(s.delta);
      deltaPath *= speedBlend; // 低速不靠转向导航（靠动量轮稳住）
    }

    let deltaCmd = deltaBalance + deltaPath;
    deltaCmd = clamp(deltaCmd, -p.steerMax, p.steerMax);

    // --- 驱动 ---
    const vCmd = clamp(p.vTarget, -p.vMax, p.vMax);

    return { tau, deltaCmd, vCmd };
  }

  // 纯追踪 (pure pursuit)：沿航点序列推进到一个前视距离外的点，路径循环。
  currentTarget(s) {
    const n = this.waypoints.length;
    if (!n) return null;
    const lookahead = 2.5;
    // 推进 wpIndex 跳过已在前视圈内的点（循环）。
    let guard = 0;
    while (guard++ < n) {
      const wp = this.waypoints[this.wpIndex % n];
      if (Math.hypot(wp.x - s.x, wp.y - s.y) < lookahead) {
        this.wpIndex = (this.wpIndex + 1) % n;
      } else break;
    }
    return this.waypoints[this.wpIndex % n];
  }
}

function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
