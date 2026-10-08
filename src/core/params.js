// 全部可调参数。单位：米 / 千克 / 秒 / 弧度（标注 Deg 的除外）。
// 几何与质量会传到 geometry.js → hardware.js → Whipple 动力学，三维模型也用同一套几何，
// 保证“看到的车”和“仿真的车”是同一辆。

export const params = {
  g: 9.81,

  // ---------- 车架几何（27.5" 城市车级别）----------
  wheelR: 0.34,          // 车轮滚动半径 (27.5×2.1)
  wheelbase: 1.04,       // 轴距 w
  trail: 0.068,          // 拖曳距 c（前轮触地点到转向轴地面交点）
  headAngleDeg: 71,      // 头管角 → 转向轴后倾 λ = 90° − 71°
  chainstay: 0.43,       // 后下叉长
  bbDrop: 0.055,         // 中轴下沉
  seatAngleDeg: 73.5,
  seatTube: 0.50,
  headTubeBottomY: 0.73, // 头管下端离地高
  headTubeLen: 0.14,

  // ---------- 动量轮 (reaction wheel) ----------
  rwRadius: 0.14,        // 飞轮外半径（护罩 +1 cm）
  rwMass: 2.8,           // 飞轮（钢制轮缘）质量
  rwTorqueMax: 18,       // 电机峰值力矩 N·m（BLDC + FOC）
  rwSpeedMax: 420,       // 空载最高转速 rad/s（≈4000 rpm），反电动势使力矩随转速线性下降
  rwFriction: 0.0015,    // 轴承粘滞摩擦 N·m·s/rad

  // ---------- 转向执行器 ----------
  steerTorqueMax: 16,    // 转向电机经同步带减速后的峰值力矩 N·m
  steerMaxDeg: 42,       // 机械限位
  steerDamping: 0.35,    // 头碗轴承 + 减速器粘滞阻尼 N·m·s/rad

  // ---------- 驱动（后轮轮毂电机）----------
  driveTorqueMax: 32,    // 轮毂电机峰值力矩 N·m
  drivePowerMax: 500,    // W
  brakeDecelMax: 4.0,    // 电子刹车最大减速度 m/s²
  Crr: 0.007,            // 滚阻系数
  CdA: 0.28,             // 风阻面积 m²
  vMax: 8.0,

  // ---------- 平衡控制（2 输入 LQR，按速度增益调度）----------
  // 状态 x = [φ, δ, φ̇, δ̇, Ω]，输入 u = [τ_rw, T_δ]
  Q: [3000, 40, 120, 1.5, 0.01],
  R: [0.6, 1.2],
  ctrlDt: 0.002,          // 控制周期 (500 Hz)，与 MCU 实时环一致
  useEstimator: true,     // true: 控制器只看“传感器+估计器”；false: 读真值（理想传感器）

  // ---------- 传感器噪声 ----------
  gyroNoise: 0.004,       // rad/s (1σ, 每采样)
  gyroBias: 0.003,        // rad/s 零偏
  accNoise: 0.04,         // m/s²
  steerEncBits: 14,       // 转向编码器分辨率
  imuHeight: 0.62,        // IMU 安装高度 (m)

  // ---------- 自动驾驶 ----------
  cruiseSpeed: 3.2,       // 巡航速度 m/s
  latAccMax: 2.0,         // 允许的最大横向加速度 m/s²（≈ 11.5° 侧倾）
  accelMax: 1.0,
  lookaheadMin: 2.2,
  lookaheadGain: 0.9,     // 前视距离 = min + gain·v
  safetyStop: 2.4,        // 前方此距离内有障碍 → 紧急制动 (m)
  inflate: 0.75,          // 规划时障碍膨胀半径 (m)

  // ---------- 仿真 ----------
  dt: 0.002,
  maxFallAngle: 1.05,     // 倒地判定 (≈60°)
  timeScale: 1.0,
};
