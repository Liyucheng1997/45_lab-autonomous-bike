// 车架几何：由少量设计参数（轴距、拖曳距、头管角、后下叉长…）推导出所有关键节点。
// 坐标（车身局部，侧视）：原点 = 后轮触地点，x 向前，y 向上，z 向右（three.js 右手系）。
// 动力学（Whipple，z 向下）与三维模型共用这一份几何。

import { rad } from './mathx.js';

export function computeGeometry(p) {
  const R = p.wheelR;
  const w = p.wheelbase;
  const c = p.trail;
  const lam = rad(90 - p.headAngleDeg);          // 转向轴后倾角 λ
  const sl = Math.sin(lam), cl = Math.cos(lam);

  // 转向轴：地面交点 (w + c, 0)，单位方向（向上）axisUp = (−sinλ, cosλ)
  const axisGround = [w + c, 0];
  const axisUp = [-sl, cl];
  const axisAtY = (y) => [w + c - y * Math.tan(lam), y]; // 轴线上高度为 y 的点
  const forkOffset = R * sl - c * cl;                    // 前叉偏移（前轴到转向轴的垂距）

  const rearHub = [0, R];
  const frontHub = [w, R];
  const bbY = R - p.bbDrop;
  const bb = [Math.sqrt(p.chainstay ** 2 - p.bbDrop ** 2), bbY];
  const sa = rad(p.seatAngleDeg);
  const seatTop = [bb[0] - p.seatTube * Math.cos(sa), bb[1] + p.seatTube * Math.sin(sa)];
  const headBot = axisAtY(p.headTubeBottomY);
  const headTopY = p.headTubeBottomY + p.headTubeLen * cl;
  const headTop = axisAtY(headTopY);
  const crown = axisAtY(p.headTubeBottomY - 0.025);
  const steererTop = axisAtY(headTopY + 0.075);          // 头碗上方：同步带轮 + 把立
  const pulleyY = headTopY + 0.022;
  const pulley = axisAtY(pulleyY);

  // 上管从座管顶略下方 → 头管上端略下方
  const ttRear = lerp2(bb, seatTop, 0.94);
  const ttFront = lerp2(headTop, headBot, 0.22);
  const dtFront = lerp2(headBot, headTop, 0.22);

  // 把立 + 车把（在“转向坐标系”里定义，跟随转向转动）
  const stemLen = 0.085, stemRise = 0.02;
  const barCenter = [steererTop[0] + stemLen * cl + (-sl) * stemRise, steererTop[1] + stemLen * sl + cl * stemRise];

  // 座杆 + 座垫
  const seatDir = [seatTop[0] - bb[0], seatTop[1] - bb[1]].map((v) => v / p.seatTube);
  const saddle = [seatTop[0] + seatDir[0] * 0.2, seatTop[1] + seatDir[1] * 0.2 + 0.03];

  // 动量轮：放在前三角内，盘面垂直于前进方向（转轴 // x 轴 = 横滚轴）
  // 盘面垂直于车架平面，护罩有 4 cm 厚度，所以要在 x ± 0.02 范围内同时避开倾斜的下管和上管
  const rwX = bb[0] + 0.065;
  const dtTop = yOnLine(bb, dtFront, rwX + 0.02) + 0.064;  // 下管（含电池加粗）上表面
  const ttBot = yOnLine(ttRear, ttFront, rwX - 0.02) - 0.022;
  const rwCenter = [rwX, (dtTop + ttBot) / 2];
  const rwGap = (ttBot - dtTop) / 2;                        // 护罩能用的最大半径

  // 转向电机：在转向轴后方，轴线平行于转向轴，同步带连到舵管上的大带轮
  const beltC = 0.105;
  const steerMotor = [pulley[0] - beltC * cl, pulley[1] - beltC * sl];

  // 车载传感器/设备安装点
  const rackY = R + 0.40;
  const rack = { x0: -0.2, x1: 0.2, y: rackY };
  const lidar = [-0.12, 1.16];
  const camera = [headTop[0] + 0.075, (headTop[1] + headBot[1]) / 2 + 0.01];
  const headlight = [headBot[0] + 0.07, headBot[1] + 0.035];
  const compute = [0.02, rackY + 0.05];
  const gnss = [0.13, rackY + 0.035];
  const estop = [-0.16, rackY + 0.06];
  const mcu = [lerp2(ttRear, ttFront, 0.13)[0], yOnLine(ttRear, ttFront, lerp2(ttRear, ttFront, 0.13)[0]) - 0.07];
  const battery = lerp2(bb, dtFront, 0.55);

  return {
    R, w, c, lam, sl, cl, forkOffset,
    axisGround, axisUp, axisAtY,
    rearHub, frontHub, bb, seatTop, headBot, headTop, crown, steererTop, pulley, pulleyY,
    ttRear, ttFront, dtFront, barCenter, saddle, seatDir,
    rwCenter, rwGap, steerMotor, beltC,
    rack, lidar, camera, headlight, compute, gnss, estop, mcu, battery,
    barHalfWidth: 0.34,
  };
}

export function lerp2(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
}
function yOnLine(a, b, x) {
  return a[1] + ((x - a[0]) / (b[0] - a[0])) * (b[1] - a[1]);
}
