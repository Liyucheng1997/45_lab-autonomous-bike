// 自动驾驶决策层（对应真车计算单元上的 ROS 2 节点，10–20 Hz）：
//   感知：激光雷达 10 Hz → 占据栅格 → 距离场
//   规划：A*（在线地图 + 高精地图先验）→ 平滑 → 速度规划（曲率限速 + 近障限速 + 减速度约束）
//   跟踪：纯追踪 (Pure Pursuit) → 期望曲率 κ 交给底层 MotionController
//   安全：前向走廊检测 → 紧急制动；路径被堵 → 立即重规划；无路可走 → 原地平衡等待
//
// 模式：IDLE 原地平衡 / ROUTE 环线巡航 / GOTO 点击导航 / MANUAL 遥控 / ESTOP 急停

import { clamp, wrapAngle } from './mathx.js';
import { Lidar } from './lidar.js';
import { OccupancyGrid, planPath } from './mapping.js';
import { laneRoute, distToRoadCenter, ROAD } from './world.js';
import { CameraDetector } from './perception.js';

export const MODE_NAMES = {
  IDLE: '原地平衡', ROUTE: '环线巡航', GOTO: '目标导航', MANUAL: '手动遥控', ESTOP: '急停',
};

export class Autopilot {
  constructor(p, geometry) {
    this.p = p;
    this.lidar = new Lidar({ mountH: geometry.lidar[1], mountX: geometry.lidar[0] });
    this.camera = new CameraDetector(geometry);
    this.grid = new OccupancyGrid();
    this.grid.setPrior((x, y) => (distToRoadCenter(x, y) > ROAD.width / 2 - 0.3 ? 2.5 : 0));
    this.route = laneRoute();
    this.reset();
  }

  reset() {
    this.mode = 'IDLE';
    this.goal = null;
    this.path = null;
    this.vProfile = null;
    this.routeIdx = -1;
    this.carrot = null;
    this.pursuit = null;
    this.tScan = -1;
    this.tPlan = 0;
    this.status = '待命';
    this.blocked = false;
    this.emergency = false;
    this.nearestObs = Infinity;
    this.planMs = 0;
    this.planFail = 0;
    this.manual = { v: 0, k: 0 };
    this.yielding = false;
    this.camera.tracks.clear();
    this.camera.detections = [];
    this.grid.clear();
  }

  setMode(mode, goal = null) {
    this.mode = mode;
    this.goal = goal;
    this.path = null;
    this.vProfile = null;
    this.tPlan = 0;
    if (mode === 'ROUTE') this.routeIdx = -1;
    if (mode === 'MANUAL') this.manual = { v: 0, k: 0 };
  }

  // 每个物理步调用；内部按自己的频率做感知/规划
  update(t, s, ctl, obstacles) {
    const p = this.p;
    // ---- 感知 10 Hz ----
    if (t - this.tScan >= 0.1) {
      this.tScan = t;
      this.lidar.scan(s, obstacles);
      this.grid.integrate(this.lidar);
      this.grid.computeDistance(4);
      this.checkCorridor(s, ctl.kappaCmd);
      this.camera.update(s, obstacles, 0.1);
    }

    if (this.mode === 'ESTOP') {
      ctl.vCmd = 0; ctl.kappaCmd = 0; ctl.brakeCmd = 1;
      this.status = '急停：电机输出限制，仅保持平衡';
      return;
    }
    if (this.mode === 'IDLE') {
      ctl.vCmd = 0; ctl.kappaCmd = 0; ctl.brakeCmd = Math.abs(s.v) > 0.05 ? 0.6 : 0;
      this.status = Math.abs(s.v) > 0.05 ? '减速停车' : '静止 · 动量轮平衡';
      return;
    }
    if (this.mode === 'MANUAL') {
      ctl.vCmd = this.manual.v; ctl.kappaCmd = this.manual.k;
      ctl.brakeCmd = this.emergency && this.manual.v > 0 ? 1 : 0;
      this.status = this.emergency ? '前方障碍！自动刹车' : '手动遥控 (WASD)';
      return;
    }

    // ---- 规划 ~4 Hz，或路径被堵时立即 ----
    const pathBlocked = this.path && this.pathBlocked(s);
    if (t - this.tPlan >= 0.25 || !this.path || pathBlocked) {
      this.tPlan = t;
      this.replan(s);
    }

    // ---- 跟踪 ----
    if (!this.path || this.path.length < 2) {
      ctl.vCmd = 0; ctl.kappaCmd = 0; ctl.brakeCmd = Math.abs(s.v) > 0.1 ? 0.8 : 0;
      this.status = this.planFail > 0 ? '无可行路径 · 原地平衡等待' : '规划中…';
      return;
    }
    const { kappa, idx, distToEnd } = this.purePursuit(s);
    ctl.kappaCmd = kappa;
    let v = this.vProfile[Math.min(idx + 2, this.vProfile.length - 1)];
    if (this.mode === 'GOTO' && distToEnd < 0.8) {
      this.setMode('IDLE');
      this.status = '已到达目标';
      return;
    }
    // 起步阶段航向偏差大时限速（低速靠动量轮也能转向）
    if (this.pursuit && Math.abs(this.pursuit.alpha) > 1.0) v = Math.min(v, 0.8);
    // 行人让行：预测行人轨迹与本车路径冲突
    const yv = this.yieldSpeed(s, idx);
    this.yielding = yv < v;
    v = Math.min(v, yv);
    ctl.vCmd = v;
    ctl.brakeCmd = this.emergency ? 1 : 0;
    this.status = this.emergency ? '前方障碍！紧急制动' : this.yielding ? (yv < 0.1 ? '礼让行人 · 停车等待' : '前方行人 · 减速') : this.blocked ? '避障重规划' : this.mode === 'ROUTE' ? '沿车道巡航' : '前往目标点';
  }

  // 碰撞走廊：沿“即将走的轨迹”扫掠车宽，雷达点落在其中 → 紧急制动。
  // 有规划路径时沿路径；手动模式沿当前指令曲率的圆弧。长度 = 安全距离 + 刹车距离。
  checkCorridor(s, kappaCmd = 0) {
    const L = 1.1 + this.p.safetyStop * Math.min(1, 0.4 + Math.max(s.v, 0) / 3) + (s.v * s.v) / (2 * this.p.brakeDecelMax);
    const halfW = 0.5;
    // 走廊中心线采样点
    const pts = [];
    if (this.path && this.mode !== 'MANUAL') {
      let i = this.closestIdx(s), acc = 0;
      pts.push([s.x, s.y]);
      for (; i < this.path.length && acc < L; i++) {
        const q = this.path[i], pr = pts[pts.length - 1];
        acc += Math.hypot(q[0] - pr[0], q[1] - pr[1]);
        pts.push(q);
      }
    } else {
      for (let d = 0; d <= L; d += 0.3) {
        const th = s.psi + kappaCmd * d;
        const k = Math.abs(kappaCmd) > 1e-3 ? kappaCmd : 0;
        pts.push(k ? [s.x + (Math.sin(th) - Math.sin(s.psi)) / k, s.y - (Math.cos(th) - Math.cos(s.psi)) / k]
          : [s.x + d * Math.cos(s.psi), s.y + d * Math.sin(s.psi)]);
      }
    }
    const P = this.lidar.points;
    const c = Math.cos(s.psi), sn = Math.sin(s.psi);
    let hit = false, nearest = Infinity;
    for (let k = 0; k < this.lidar.count; k++) {
      const h = P[3 * k + 1];
      if (h < 0.15 || h > 1.9) continue;
      const px = P[3 * k], py = P[3 * k + 2];
      const dx = px - s.x, dy = py - s.y;
      const d = Math.hypot(dx, dy);
      if (d < nearest) nearest = d;
      if (hit || d > L + 1 || dx * c + dy * sn < 0.2) continue;
      for (let i = 1; i < pts.length; i++) {
        if (segDist(px, py, pts[i - 1], pts[i]) < halfW) { hit = true; break; }
      }
    }
    this.nearestObs = nearest;
    this.emergency = hit && s.v > -0.05 && this.mode !== 'IDLE';
  }

  // 对每个检测到的行人做 4 s 匀速预测；与本车沿路径的预测位置距离 < 1.6 m 视为冲突
  yieldSpeed(s, i0) {
    const peds = this.camera.detections.filter((d) => d.kind === 'ped');
    if (!peds.length) return Infinity;
    const path = this.path;
    const cum = [0];
    for (let i = i0 + 1; i < path.length; i++) cum.push(cum[cum.length - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
    const at = (d) => {
      let k = 0;
      while (k < cum.length - 1 && cum[k + 1] < d) k++;
      return path[Math.min(i0 + k, path.length - 1)];
    };
    let cap = Infinity;
    const vb = Math.max(s.v, 1.2);
    for (const d of peds) {
      const sp = Math.hypot(d.vx, d.vy);
      for (let t = 0; t <= 4; t += 0.2) {
        const q = at(vb * t);
        const px = d.x + d.vx * t, py = d.y + d.vy * t;
        const gap = Math.hypot(px - q[0], py - q[1]);
        if (gap < 1.6 && (sp > 0.25 || t < 1.5)) {
          cap = Math.min(cap, t < 2.6 ? 0 : 1.2);
          break;
        }
      }
      if (d.dist < 7) cap = Math.min(cap, 2.0);
    }
    return cap;
  }

  pathBlocked(s) {
    const path = this.path;
    let i0 = this.closestIdx(s);
    let acc = 0;
    for (let i = i0; i < path.length - 1 && acc < 10; i++) {
      acc += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
      if (this.grid.distAt(path[i][0], path[i][1]) < 0.45) return true;
    }
    return false;
  }

  replan(s) {
    const t0 = performance.now();
    let candidates;
    if (this.mode === 'ROUTE') {
      const n = this.route.length;
      // 在当前位置附近搜索路线投影（只向前搜索，避免跳到环线另一侧）
      if (this.routeIdx < 0) {
        let best = 0, bd = Infinity;
        this.route.forEach((q, i) => { const d = Math.hypot(q[0] - s.x, q[1] - s.y); if (d < bd) { bd = d; best = i; } });
        this.routeIdx = best;
      } else {
        let best = this.routeIdx, bd = Infinity;
        for (let k = 0; k < 80; k++) {
          const i = (this.routeIdx + k) % n;
          const d = Math.hypot(this.route[i][0] - s.x, this.route[i][1] - s.y);
          if (d < bd) { bd = d; best = i; }
        }
        this.routeIdx = best;
      }
      // 前视 16 m 的“胡萝卜”点；落在障碍上就往前挪。
      // 注意：雷达只能看到障碍的外表面，车辆“内部”看起来是被包围的空洞，A* 到不了 →
      // 规划失败时也继续沿路线往前挪目标点重试。
      candidates = [];
      let ahead = 32;
      while (candidates.length < 4 && ahead < 110) {
        const gi = (this.routeIdx + ahead) % n;
        if (this.grid.distAt(...this.route[gi]) >= this.p.inflate + 0.15) candidates.push(this.route[gi]);
        ahead += candidates.length ? 12 : 2;
      }
    } else candidates = [this.goal];

    // 起点取车前方 0.6 m（让路径和当前航向更一致）
    const start = [s.x + 0.6 * Math.cos(s.psi), s.y + 0.6 * Math.sin(s.psi)];
    let path = null;
    for (const goal of candidates) {
      this.carrot = goal;
      path = planPath(this.grid, start, goal, { inflate: this.p.inflate, maxExpand: 40000 });
      if (path) break;
    }
    this.planMs = performance.now() - t0;
    if (!path) {
      this.planFail++;
      if (this.planFail > 2) { this.path = null; this.vProfile = null; }
      return;
    }
    this.planFail = 0;
    this.blocked = !!this.path && this.pathBlockedFlag(path);
    path.unshift([s.x, s.y]);
    this.path = path;
    this.vProfile = this.speedProfile(path, this.mode === 'GOTO');
  }

  pathBlockedFlag(path) {
    // 新路径如果明显偏离车道中心 → 标记“正在避障”
    return path.some(([x, y]) => this.mode === 'ROUTE' && this.grid.distAt(x, y) < 1.6);
  }

  speedProfile(path, stopAtEnd) {
    const p = this.p, n = path.length;
    const v = new Array(n).fill(p.cruiseSpeed);
    for (let i = 1; i < n - 1; i++) {
      const a = path[i - 1], b = path[i], c = path[i + 1];
      const k = curvature(a, b, c);
      v[i] = Math.min(v[i], Math.sqrt(p.latAccMax / Math.max(k, 1e-3)));
      const d = this.grid.distAt(b[0], b[1]);
      if (d < 1.6) v[i] = Math.min(v[i], 1.2 + (d - 0.6) * 2.0);
      v[i] = Math.max(v[i], 0.6);
    }
    if (stopAtEnd) v[n - 1] = 0;
    // 反向传递：减速度约束 1.2 m/s²
    for (let i = n - 2; i >= 0; i--) {
      const ds = Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
      v[i] = Math.min(v[i], Math.sqrt(v[i + 1] ** 2 + 2 * 1.2 * ds));
    }
    return v;
  }

  closestIdx(s) {
    let best = 0, bd = Infinity;
    const path = this.path;
    for (let i = 0; i < path.length; i++) {
      const d = (path[i][0] - s.x) ** 2 + (path[i][1] - s.y) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  purePursuit(s) {
    const p = this.p, path = this.path;
    const Ld = p.lookaheadMin + p.lookaheadGain * Math.max(s.v, 0);
    const i0 = this.closestIdx(s);
    let target = path[path.length - 1], ti = path.length - 1;
    for (let i = i0; i < path.length; i++) {
      if (Math.hypot(path[i][0] - s.x, path[i][1] - s.y) >= Ld) { target = path[i]; ti = i; break; }
    }
    const alpha = wrapAngle(Math.atan2(target[1] - s.y, target[0] - s.x) - s.psi);
    const kappa = clamp((2 * Math.sin(alpha)) / Ld, -0.6, 0.6);
    let distToEnd = 0;
    for (let i = i0; i < path.length - 1; i++) distToEnd += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
    this.pursuit = { target, alpha, Ld };
    return { kappa, idx: i0, distToEnd };
  }
}

function segDist(px, py, a, b) {
  const vx = b[0] - a[0], vy = b[1] - a[1];
  const L2 = vx * vx + vy * vy;
  let t = L2 > 1e-9 ? ((px - a[0]) * vx + (py - a[1]) * vy) / L2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(px - a[0] - t * vx, py - a[1] - t * vy);
}

function curvature(a, b, c) {
  const abx = b[0] - a[0], aby = b[1] - a[1], bcx = c[0] - b[0], bcy = c[1] - b[1];
  const cross = abx * bcy - aby * bcx;
  const la = Math.hypot(abx, aby), lb = Math.hypot(bcx, bcy), lc = Math.hypot(c[0] - a[0], c[1] - a[1]);
  return la * lb * lc > 1e-9 ? Math.abs(2 * cross) / (la * lb * lc) : 0;
}
