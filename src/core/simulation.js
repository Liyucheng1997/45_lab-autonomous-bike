// 整车仿真：把 物理 / 传感器 / 估计器 / 底层控制 / 自动驾驶 / 场景 串起来。
// 浏览器和 node 测试共用这一个类，渲染层只读它的状态。

import { buildPlant } from './plant.js';
import { initialState, actuate, step } from './physics.js';
import { MotionController } from './controller.js';
import { SensorSuite, Estimator } from './estimator.js';
import { Autopilot } from './autopilot.js';
import { buildObstacles, updateDynamic, laneRoute } from './world.js';

export class Simulation {
  constructor(p) {
    this.p = p;
    this.obstacles = buildObstacles();
    this.rebuild();
    this.autopilot = new Autopilot(p, this.plant.body.G);
    this.reset();
  }

  // 几何/质量/LQR 权重变化后重建模型（重新求 47 组 LQR 增益）
  rebuild() {
    const t0 = typeof performance !== 'undefined' ? performance.now() : 0;
    this.plant = buildPlant(this.p);
    this.buildMs = (typeof performance !== 'undefined' ? performance.now() : 0) - t0;
    if (this.ctl) this.ctl.setPlant(this.plant);
    else this.ctl = new MotionController(this.plant);
    this.version = (this.version || 0) + 1;
  }

  reset() {
    const r = laneRoute();
    const i = 40; // 从下边直道起步
    const a = r[i], b = r[i + 1];
    this.s = initialState(a[0], a[1], Math.atan2(b[1] - a[1], b[0] - a[0]));
    this.s.psiDot = 0; this.s.phiDD = 0;
    this.t = 0;
    this.fallen = false;
    this.ctl.reset();
    this.sensors = new SensorSuite(this.p);
    this.est = new Estimator(this.p);
    this.est.reset(this.s.phi); // 上电时用静止加速度计读数初始化倾角（真车做法）
    this.autopilot.reset();
    this.act = { tau: 0, Td: 0, F: 0, brake: 0 };
    this.cmd = { tauRW: 0, Tdelta: 0, drive: 0, brake: 0 };
    this.dist = { Tphi: 0, Tdelta: 0, until: 0 };
    this.meas = null;
    this.energy = 0;
    this.maxLean = 0;
  }

  // 侧向推一把：冲量 J (N·s) 作用在车座高度，持续 0.1 s
  push(J) {
    const h = 0.9;
    this.dist = { Tphi: (J * h) / 0.1, Tdelta: 0, until: this.t + 0.1 };
  }

  stepOnce() {
    const p = this.p, dt = p.dt;
    updateDynamic(this.obstacles, this.t);
    if (this.fallen) { this.t += dt; return; }

    // 上层决策 50 Hz（计算单元），底层平衡 500 Hz（MCU）
    if ((this.tick = (this.tick || 0) + 1) % 10 === 1) this.autopilot.update(this.t, this.s, this.ctl, this.obstacles);

    // 传感器 → 估计器 → 控制器（与真车 MCU 一致：控制器看不到真值）
    this.meas = this.sensors.read(this.s, this.plant);
    this.est.update(this.meas, dt, this.plant);
    const xh = p.useEstimator ? this.est : this.s;
    this.cmd = this.ctl.update(xh, dt);
    this.act = actuate(this.plant, this.s, this.cmd);

    const dist = this.t < this.dist.until ? this.dist : undefined;
    this.s = step(this.plant, this.s, this.act, dt, dist);
    this.t += dt;

    // 能耗（电功率粗估：机械功率 / 效率 + 待机功耗）
    const Pm = Math.abs(this.act.tau * this.s.Omega) + Math.abs(this.act.F * this.s.v) + Math.abs(this.act.Td * this.s.deltaDot);
    this.power = Pm / 0.82 + 28;
    this.energy += this.power * dt;
    this.maxLean = Math.max(this.maxLean, Math.abs(this.s.phi));

    if (Math.abs(this.s.phi) > p.maxFallAngle || !Number.isFinite(this.s.phi)) {
      this.fallen = true;
      this.fallSide = Math.sign(this.s.phi) || 1;
      this.fallT = this.t;
    }
  }

  advance(T) {
    const n = Math.round(T / this.p.dt);
    for (let i = 0; i < n; i++) this.stepOnce();
  }
}
