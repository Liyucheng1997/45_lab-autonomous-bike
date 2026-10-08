// Whipple–Carvallo 自行车线性化动力学 —— 按 Meijaard, Papadopoulos, Ruina & Schwab (2007)
// "Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review",
// Proc. R. Soc. A 463, 1955–1982，附录 A 的公式由物理参数构造系数矩阵：
//
//     M q̈ + v·C1 q̇ + (g·K0 + v²·K2) q = f,     q = [φ 横滚, δ 转向]ᵀ,  f = [Tφ, Tδ]ᵀ
//
// 坐标约定（与论文一致）：原点在后轮触地点，x 向前、y 向右、z 向下；φ>0 向右倒，δ>0 向右打。
// 四个刚体：R 后轮、B 后车架(含车载设备)、H 前叉+车把、F 前轮。

export function whippleMatrices(p) {
  const { w, c, lambda: lam, rR, rF } = p;
  const sl = Math.sin(lam), cl = Math.cos(lam);

  // 整车
  const mT = p.mR + p.mB + p.mH + p.mF;
  const xT = (p.xB * p.mB + p.xH * p.mH + w * p.mF) / mT;
  const zT = (-rR * p.mR + p.zB * p.mB + p.zH * p.mH - rF * p.mF) / mT;
  const ITxx = p.IRxx + p.IBxx + p.IHxx + p.IFxx + p.mR * rR * rR + p.mB * p.zB * p.zB + p.mH * p.zH * p.zH + p.mF * rF * rF;
  const ITxz = p.IBxz + p.IHxz - p.mB * p.xB * p.zB - p.mH * p.xH * p.zH + p.mF * w * rF;
  const IRzz = p.IRxx, IFzz = p.IFxx;
  const ITzz = IRzz + p.IBzz + p.IHzz + IFzz + p.mB * p.xB * p.xB + p.mH * p.xH * p.xH + p.mF * w * w;

  // 前组件 A = H + F
  const mA = p.mH + p.mF;
  const xA = (p.xH * p.mH + w * p.mF) / mA;
  const zA = (p.zH * p.mH - rF * p.mF) / mA;
  const IAxx = p.IHxx + p.IFxx + p.mH * (p.zH - zA) ** 2 + p.mF * (rF + zA) ** 2;
  const IAxz = p.IHxz - p.mH * (p.xH - xA) * (p.zH - zA) + p.mF * (w - xA) * (rF + zA);
  const IAzz = p.IHzz + IFzz + p.mH * (p.xH - xA) ** 2 + p.mF * (w - xA) ** 2;
  const uA = (xA - w - c) * cl - zA * sl; // 前组件质心到转向轴的垂距
  const IAll = mA * uA * uA + IAxx * sl * sl + 2 * IAxz * sl * cl + IAzz * cl * cl;
  const IAlx = -mA * uA * zA + IAxx * sl + IAxz * cl;
  const IAlz = mA * uA * xA + IAxz * sl + IAzz * cl;

  const mu = (c / w) * cl;
  const SR = p.IRyy / rR, SF = p.IFyy / rF, ST = SR + SF; // 车轮陀螺系数
  const SA = mA * uA + mu * mT * xT;

  const M = [
    [ITxx, IAlx + mu * ITxz],
    [IAlx + mu * ITxz, IAll + 2 * mu * IAlz + mu * mu * ITzz],
  ];
  const K0 = [
    [mT * zT, -SA],
    [-SA, -SA * sl],
  ];
  const K2 = [
    [0, ((ST - mT * zT) / w) * cl],
    [0, ((SA + SF * sl) / w) * cl],
  ];
  const C1 = [
    [0, mu * ST + SF * cl + (ITxz / w) * cl - mu * mT * zT],
    [-(mu * ST + SF * cl), (IAlz / w) * cl + mu * (SA + (ITzz / w) * cl)],
  ];
  return { M, C1, K0, K2, mT, xT, zT, ITxx, ITxz, ITzz };
}

// 论文表 1 的基准自行车参数（含骑手），用于单元验证。
export const BENCHMARK = {
  w: 1.02, c: 0.08, lambda: Math.PI / 10, g: 9.81,
  rR: 0.3, mR: 2, IRxx: 0.0603, IRyy: 0.12,
  xB: 0.3, zB: -0.9, mB: 85, IBxx: 9.2, IByy: 11, IBzz: 2.8, IBxz: 2.4,
  xH: 0.9, zH: -0.7, mH: 4, IHxx: 0.05892, IHyy: 0.06, IHzz: 0.00708, IHxz: -0.00756,
  rF: 0.35, mF: 3, IFxx: 0.1405, IFyy: 0.28,
};
