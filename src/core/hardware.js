// 硬件物料清单 (BOM) —— 每个部件的质量、安装位置、形状。
// 由它求出 Whipple 模型四个刚体 (R 后轮 / B 后车架 / H 前叉车把 / F 前轮) 的质量、质心和惯量张量，
// 所以在 UI 里加大电池、换更重的雷达，动力学会跟着变。
//
// 坐标：车身局部 x 前 / y 上 / z 右，原点为后轮触地点（与 geometry.js 相同）。

import { computeGeometry, lerp2 } from './geometry.js';

// ---- 形状 → 绕自身质心的惯量张量 (three 坐标) ----
function boxI(m, [lx, ly, lz]) {
  return [
    [(m * (ly * ly + lz * lz)) / 12, 0, 0],
    [0, (m * (lx * lx + lz * lz)) / 12, 0],
    [0, 0, (m * (lx * lx + ly * ly)) / 12],
  ];
}
function rodI(m, a, b) {
  // 细杆：I = mL²/12 (E − u uᵀ)
  const d = [b[0] - a[0], b[1] - a[1], (b[2] || 0) - (a[2] || 0)];
  const L = Math.hypot(...d);
  const u = d.map((v) => v / L);
  const k = (m * L * L) / 12;
  return [0, 1, 2].map((i) => [0, 1, 2].map((j) => k * ((i === j ? 1 : 0) - u[i] * u[j])));
}
// 圆环/圆盘，axis = 'x' | 'z'；k = 回转半径系数（薄环 1，实心盘 0.5）
function wheelI(m, r, axis, k = 1) {
  const Ia = m * r * r * k, Id = Ia / 2;
  return axis === 'x'
    ? [[Ia, 0, 0], [0, Id, 0], [0, 0, Id]]
    : [[Id, 0, 0], [0, Id, 0], [0, 0, Ia]];
}

// ---- BOM ----
export function buildBOM(p) {
  const G = computeGeometry(p);
  const P3 = (xy, z = 0) => [xy[0], xy[1], z];
  const items = [];
  const add = (it) => items.push({ z: 0, ...it });
  const rod = (id, name, assy, mass, a, b, extra = {}) =>
    add({ id, name, assy, mass, pos: P3(lerp2(a, b, 0.5)), I: rodI(mass, P3(a), P3(b)), ...extra });

  // ===== 后车架 B =====
  rod('dt', '下管 (内置电池仓)', 'B', 0.75, G.bb, G.dtFront);
  rod('tt', '上管', 'B', 0.42, G.ttRear, G.ttFront);
  rod('st', '座管', 'B', 0.38, G.bb, G.seatTop);
  rod('ht', '头管 + 碗组', 'B', 0.32, G.headBot, G.headTop);
  rod('cs', '后下叉 ×2', 'B', 0.36, G.bb, G.rearHub);
  rod('ss', '后上叉 ×2', 'B', 0.30, lerp2(G.bb, G.seatTop, 0.9), G.rearHub);
  add({ id: 'battery', name: '锂电池 36V 13Ah (下管内置)', assy: 'B', mass: 2.9, pos: P3(G.battery),
    I: rodI(2.9, P3(lerp2(G.bb, G.dtFront, 0.2)), P3(lerp2(G.bb, G.dtFront, 0.9))),
    spec: '10S4P 21700 · 468 Wh · BMS 30A', label: true });
  add({ id: 'rw', name: '动量轮飞轮', assy: 'B', mass: p.rwMass, pos: P3(G.rwCenter),
    I: wheelI(p.rwMass, p.rwRadius * 0.9, 'x', 0.92), spinAxis: 'x',
    spec: `钢制轮缘 Ø${(p.rwRadius * 2000).toFixed(0)} mm · ${p.rwMass} kg`, label: true });
  add({ id: 'rwMotor', name: '动量轮电机', assy: 'B', mass: 1.15, pos: [G.rwCenter[0] - 0.045, G.rwCenter[1], 0],
    I: boxI(1.15, [0.07, 0.1, 0.1]),
    spec: `BLDC 外转子 + FOC · 峰值 ${p.rwTorqueMax} N·m · ${(p.rwSpeedMax * 60 / 2 / Math.PI).toFixed(0)} rpm`, label: true });
  add({ id: 'rwGuard', name: '动量轮护罩/支架', assy: 'B', mass: 0.55, pos: P3(G.rwCenter), I: wheelI(0.55, p.rwRadius + 0.01, 'x', 1) });
  add({ id: 'steerMotor', name: '转向执行器', assy: 'B', mass: 0.85, pos: P3(G.steerMotor, 0),
    I: boxI(0.85, [0.07, 0.09, 0.07]), spec: `BLDC + 1:3 同步带 · 峰值 ${p.steerTorqueMax} N·m · 14-bit 编码器`, label: true });
  add({ id: 'camera', name: 'RGB-D 深度相机', assy: 'B', mass: 0.32, pos: P3(G.camera), I: boxI(0.32, [0.03, 0.03, 0.1]),
    spec: '双目结构光 · 87°×58° FOV · 0.3–10 m', label: true });
  add({ id: 'headlight', name: '前灯', assy: 'B', mass: 0.12, pos: P3(G.headlight), I: boxI(0.12, [0.05, 0.04, 0.05]) });
  add({ id: 'rack', name: '后货架', assy: 'B', mass: 0.75, pos: [0, G.rack.y - 0.08, 0], I: boxI(0.75, [0.42, 0.2, 0.14]) });
  add({ id: 'compute', name: '计算单元', assy: 'B', mass: 1.25, pos: P3(G.compute), I: boxI(1.25, [0.2, 0.07, 0.15]),
    spec: 'ARM 8 核 + 100 TOPS NPU · ROS 2 · 感知 / 定位 / 规划', label: true });
  add({ id: 'mast', name: '雷达支架', assy: 'B', mass: 0.28, pos: [G.lidar[0], (G.rack.y + G.lidar[1]) / 2, 0],
    I: rodI(0.28, [G.lidar[0], G.rack.y, 0], [G.lidar[0], G.lidar[1], 0]) });
  add({ id: 'lidar', name: '360° 激光雷达', assy: 'B', mass: 0.85, pos: P3(G.lidar), I: boxI(0.85, [0.1, 0.07, 0.1]),
    spec: '16 线 · 360°×30° · 10 Hz · 100 m', label: true });
  add({ id: 'gnss', name: 'GNSS RTK 天线', assy: 'B', mass: 0.18, pos: P3(G.gnss), I: boxI(0.18, [0.09, 0.03, 0.09]),
    spec: '多频 RTK · 厘米级定位', label: true });
  add({ id: 'estop', name: '急停按钮', assy: 'B', mass: 0.1, pos: P3(G.estop), I: boxI(0.1, [0.04, 0.04, 0.04]),
    spec: '硬线切断三路电机驱动', label: true });
  add({ id: 'mcu', name: '主控 + IMU + FOC 驱动', assy: 'B', mass: 0.45, pos: P3(G.mcu), I: boxI(0.45, [0.12, 0.05, 0.06]),
    spec: 'MCU 500 Hz 平衡环 · 6 轴 IMU 1 kHz · CAN 总线', label: true });
  add({ id: 'saddle', name: '座垫 + 座杆', assy: 'B', mass: 0.45, pos: P3(G.saddle), I: boxI(0.45, [0.26, 0.15, 0.12]) });
  add({ id: 'crank', name: '曲柄 / 牙盘 / 脚踏', assy: 'B', mass: 0.95, pos: P3(G.bb), I: boxI(0.95, [0.35, 0.35, 0.3]) });
  add({ id: 'harness', name: '线束 / 连接器', assy: 'B', mass: 0.35, pos: [0.45, 0.62, 0], I: boxI(0.35, [0.6, 0.3, 0.05]) });

  // ===== 前叉 + 车把 H =====
  const steerTopP = G.steererTop;
  rod('fork', '前叉 (含刹车卡钳)', 'H', 0.85, G.crown, G.frontHub);
  rod('steerer', '舵管 + 大带轮', 'H', 0.28, G.crown, steerTopP);
  add({ id: 'bar', name: '车把 + 把立 + 握把', assy: 'H', mass: 0.62, pos: P3(G.barCenter),
    I: rodI(0.62, [G.barCenter[0], G.barCenter[1], -G.barHalfWidth], [G.barCenter[0], G.barCenter[1], G.barHalfWidth]) });

  // ===== 车轮 =====
  const rimR = p.wheelR - 0.045;
  // 后轮：轮圈+外胎 1.45 kg（环）+ 轮毂电机 2.9 kg（实心盘 r≈0.09）+ 碟片
  const IR = addI(wheelI(1.45, rimR + 0.02, 'z', 1), wheelI(2.9, 0.09, 'z', 0.5), wheelI(0.12, 0.08, 'z', 1));
  add({ id: 'rearWheel', name: '后轮 + 轮毂电机', assy: 'R', mass: 1.45 + 2.9 + 0.12, pos: P3(G.rearHub), I: IR,
    spec: `轮毂电机 · 峰值 ${p.driveTorqueMax} N·m · ${p.drivePowerMax} W · 霍尔测速`, label: true });
  const IF = addI(wheelI(1.4, rimR + 0.02, 'z', 1), wheelI(0.25, 0.03, 'z', 0.5), wheelI(0.12, 0.08, 'z', 1));
  add({ id: 'frontWheel', name: '前轮', assy: 'F', mass: 1.4 + 0.25 + 0.12, pos: P3(G.frontHub), I: IF });

  return { items, G };
}

function addI(...Is) {
  return [0, 1, 2].map((i) => [0, 1, 2].map((j) => Is.reduce((s, I) => s + I[i][j], 0)));
}

// 合成一个刚体：质量、质心、绕质心的惯量张量 (three 坐标)
function combine(items) {
  const m = items.reduce((s, it) => s + it.mass, 0);
  const c = [0, 1, 2].map((k) => items.reduce((s, it) => s + it.mass * it.pos[k], 0) / m);
  const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (const it of items) {
    const d = it.pos.map((v, k) => v - c[k]);
    const d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2;
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) I[i][j] += it.I[i][j] + it.mass * ((i === j ? d2 : 0) - d[i] * d[j]);
  }
  return { m, c, I };
}

// BOM → Whipple 参数（坐标转换：Whipple x=前, y=右(three z), z=下(−three y)）
export function bodyParams(p) {
  const { items, G } = buildBOM(p);
  const group = (a) => combine(items.filter((it) => it.assy === a));
  const B = group('B'), H = group('H'), Rw = group('R'), Fw = group('F');
  const toW = (body) => ({
    m: body.m, x: body.c[0], z: -body.c[1],
    Ixx: body.I[0][0], Iyy: body.I[2][2], Izz: body.I[1][1],
    Ixz: -body.I[0][1], // I_xz(W) = −∫x z_W dm = +∫x y dm = −I_xy(three)
  });
  const b = toW(B), h = toW(H);
  const total = items.reduce((s, it) => s + it.mass, 0);
  const cgY = items.reduce((s, it) => s + it.mass * it.pos[1], 0) / total;
  const cgX = items.reduce((s, it) => s + it.mass * it.pos[0], 0) / total;
  return {
    whipple: {
      w: G.w, c: G.c, lambda: G.lam, g: p.g,
      rR: G.R, mR: Rw.m, IRxx: Rw.I[0][0], IRyy: Rw.I[2][2],
      xB: b.x, zB: b.z, mB: b.m, IBxx: b.Ixx, IByy: b.Iyy, IBzz: b.Izz, IBxz: b.Ixz,
      xH: h.x, zH: h.z, mH: h.m, IHxx: h.Ixx, IHyy: h.Iyy, IHzz: h.Izz, IHxz: h.Ixz,
      rF: G.R, mF: Fw.m, IFxx: Fw.I[0][0], IFyy: Fw.I[2][2],
    },
    // 飞轮绕自转轴的转动惯量（单独作为一个自由度）
    Iw: items.find((it) => it.id === 'rw').I[0][0],
    totalMass: total, cg: [cgX, cgY], items, G,
  };
}
