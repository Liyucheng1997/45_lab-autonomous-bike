// 物理参数 — 取 XUAN 量级（一辆真实尺寸自行车 + 动量轮）。
// 单位: 米 / 千克 / 秒 / 弧度。所有数值可在 UI 中实时调整。

export const params = {
  g: 9.81,            // 重力加速度

  // 整车（不含动量轮转动惯量）
  m: 15.0,            // 总质量 (kg)
  h: 0.55,            // 质心离地高度 (m) —— 倒立摆摆长 l
  b: 1.02,            // 轴距 (m)
  Ip: 5.0,            // 车身绕轮触地线的转动惯量 (kg·m^2)

  // 动量轮 (reaction wheel) —— 力矩必须大于峰值重力力矩 m·g·l·sinθ 才能回正，
  // 故取强力 FOC 电机 + 减速量级（XUAN 的动量轮力矩很大）。
  Iw: 0.09,           // 动量轮转动惯量 (kg·m^2)
  wheelTorqueMax: 38.0,// 动量轮电机最大力矩 (N·m)
  wheelSpeedMax: 600, // 动量轮最大转速 (rad/s)

  // 驱动
  vTarget: 0.0,       // 目标前进速度 (m/s)
  vTau: 0.6,          // 驱动一阶时间常数 (s)
  vMax: 6.0,

  // 转向舵机
  steerMax: 0.5,      // 最大转向角 (rad ≈ 28.6°)
  steerRateMax: 6.0,  // 转向角速度上限 (rad/s)
  steerTau: 0.06,     // 舵机一阶时间常数 (s)

  // LQR 权重 (动量轮平衡): 状态 [θ, θ̇, Ω]
  Q: [800, 30, 0.15], // 越大越激进 (第三项约束动量轮转速回零)
  R: 1.0,             // 控制代价

  // 转向控制增益（高速平衡辅助 + 路径跟踪）
  steerBalanceKp: 1.2,
  steerBalanceKd: 0.25,
  steerHeadingKp: 0.9,
  steerHeadingKd: 0.6,
  steerBlendSpeed: 1.2, // 速度超过该值后转向平衡逐渐介入 (m/s)

  // 仿真
  dt: 0.002,          // 物理积分步长 (s)
  maxFallAngle: 1.2,  // 倒地判定 (rad ≈ 68.8°)
};

// 由几何/质量推导的初始 LQR 量级提示（用于注释/调试）。
export function naturalFrequency(p = params) {
  return Math.sqrt((p.m * p.g * p.h) / p.Ip); // rad/s, 倒立摆失稳速度
}
