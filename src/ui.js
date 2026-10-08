// 界面：HUD 遥测、实时曲线、参数面板 (lil-gui)、硬件 BOM / 动力学分析面板。
import GUI from 'lil-gui';
import { MODE_NAMES } from './core/autopilot.js';
import { linearize } from './core/plant.js';
import { eigenvalues } from './core/mathx.js';

const deg = (r) => (r * 180) / Math.PI;
const $ = (id) => document.getElementById(id);

export class HUD {
  constructor(sim) {
    this.sim = sim;
    this.el = $('hud');
    this.chart = $('chart');
    this.ctx = this.chart.getContext('2d');
    this.N = 360;
    this.series = {
      phi: new Float32Array(this.N), phiHat: new Float32Array(this.N), delta: new Float32Array(this.N),
      tau: new Float32Array(this.N), Td: new Float32Array(this.N), v: new Float32Array(this.N), vCmd: new Float32Array(this.N),
    };
    this.head = 0;
    this.tText = 0;
  }

  sample() {
    const s = this.sim.s, S = this.series, i = this.head;
    S.phi[i] = deg(s.phi); S.phiHat[i] = deg(this.sim.est.phi); S.delta[i] = deg(s.delta);
    S.tau[i] = this.sim.act.tau; S.Td[i] = this.sim.act.Td; S.v[i] = s.v; S.vCmd[i] = this.sim.ctl.vRef;
    this.head = (i + 1) % this.N;
  }

  draw() {
    const c = this.ctx, W = this.chart.width, H = this.chart.height;
    c.clearRect(0, 0, W, H);
    const rows = [
      { title: '横滚 φ 真值 / 估计 (°)', keys: ['phi', 'phiHat'], colors: ['#5ce1ff', '#ff9f43'], range: 12 },
      { title: '转向 δ (°)', keys: ['delta'], colors: ['#7dff9b'], range: 30 },
      { title: '动量轮 τ / 转向 T (N·m)', keys: ['tau', 'Td'], colors: ['#ff5f6d', '#c38bff'], range: this.sim.p.rwTorqueMax },
      { title: '车速 v / 指令 (m/s)', keys: ['v', 'vCmd'], colors: ['#ffd23f', 'rgba(255,255,255,0.45)'], range: 4, positive: true },
    ];
    const h = H / rows.length;
    rows.forEach((r, k) => {
      const y0 = k * h;
      c.fillStyle = 'rgba(255,255,255,0.03)';
      c.fillRect(0, y0 + 1, W, h - 2);
      const mid = r.positive ? y0 + h - 4 : y0 + h / 2;
      const sc = r.positive ? (h - 16) / r.range : (h / 2 - 8) / r.range;
      c.strokeStyle = 'rgba(255,255,255,0.12)';
      c.beginPath(); c.moveTo(0, mid); c.lineTo(W, mid); c.stroke();
      r.keys.forEach((key, j) => {
        const a = this.series[key];
        c.strokeStyle = r.colors[j];
        c.lineWidth = 1.4;
        c.beginPath();
        for (let x = 0; x < this.N; x++) {
          const v = a[(this.head + x) % this.N];
          const yy = Math.max(y0 + 2, Math.min(y0 + h - 2, mid - v * sc));
          const px = (x / (this.N - 1)) * W;
          x ? c.lineTo(px, yy) : c.moveTo(px, yy);
        }
        c.stroke();
      });
      c.fillStyle = '#8b93a7';
      c.font = '10px ui-monospace, Consolas, monospace';
      c.fillText(r.title, 6, y0 + 12);
    });
  }

  text(t, fps) {
    if (t - this.tText < 0.08) return;
    this.tText = t;
    const sim = this.sim, s = sim.s, ap = sim.autopilot, p = sim.p;
    const K = sim.ctl.K;
    const mode = sim.fallen ? '<span class="bad">已倒地</span>' : `<span class="mode-${ap.mode}">${MODE_NAMES[ap.mode]}</span>`;
    const status = sim.fallen ? '按 R 复位' : ap.status;
    const share = K ? Math.abs(sim.act.Td) / (Math.abs(sim.act.Td) + Math.abs(sim.act.tau) + 1e-6) : 0;
    const bar = (v, max, cls = '') => `<span class="bar ${cls}"><i style="width:${Math.min(100, (Math.abs(v) / max) * 100).toFixed(0)}%"></i></span>`;
    const soc = Math.max(0, 100 - (sim.energy / 3600 / 468) * 100);
    this.el.innerHTML = `
      <div class="hud-title">XUAN-AV <small>自动驾驶自行车 · 数字孪生</small></div>
      <div class="row big"><span>模式</span><b>${mode}</b></div>
      <div class="row"><span>状态</span><span class="${ap.emergency || ap.yielding ? 'warn' : ''}">${status}</span></div>
      <div class="sep"></div>
      <div class="row"><span>车速</span><b>${s.v.toFixed(2)} m/s</b><em>${(s.v * 3.6).toFixed(1)} km/h</em></div>
      <div class="row"><span>横滚 φ</span><b>${deg(s.phi).toFixed(1)}°</b>${bar(s.phi, 0.35)}</div>
      <div class="row"><span>估计 φ̂</span><b>${deg(sim.est.phi).toFixed(1)}°</b><em>误差 ${deg(sim.est.phi - s.phi).toFixed(2)}°</em></div>
      <div class="row"><span>转向 δ</span><b>${deg(s.delta).toFixed(1)}°</b>${bar(s.delta, p.steerMaxDeg * Math.PI / 180)}</div>
      <div class="row"><span>动量轮</span><b>${(s.Omega * 60 / 2 / Math.PI).toFixed(0)} rpm</b>${bar(s.Omega, p.rwSpeedMax, 'rw')}</div>
      <div class="row"><span>动量轮力矩</span><b>${sim.act.tau.toFixed(1)} N·m</b>${bar(sim.act.tau, p.rwTorqueMax, 'rw')}</div>
      <div class="row"><span>转向力矩</span><b>${sim.act.Td.toFixed(1)} N·m</b>${bar(sim.act.Td, p.steerTorqueMax)}</div>
      <div class="row"><span>平衡分配</span><em>动量轮 ${(100 * (1 - share)).toFixed(0)}% · 转向 ${(100 * share).toFixed(0)}%</em></div>
      <div class="sep"></div>
      <div class="row"><span>最近障碍</span><b>${Number.isFinite(ap.nearestObs) ? ap.nearestObs.toFixed(1) + ' m' : '—'}</b><em>检测 ${ap.camera.detections.length} 个</em></div>
      <div class="row"><span>规划耗时</span><b>${ap.planMs.toFixed(1)} ms</b><em>栅格 ${ap.grid.nx}×${ap.grid.ny}</em></div>
      <div class="row"><span>功率 / 电量</span><b>${(sim.power || 0).toFixed(0)} W</b><em>SOC ${soc.toFixed(1)}%</em></div>
      <div class="row"><span>里程</span><b>${s.odo.toFixed(0)} m</b><em>t = ${sim.t.toFixed(0)} s · ${fps.toFixed(0)} fps</em></div>`;
  }
}

// ---------------- 参数面板 ----------------
export function buildGUI(sim, hooks) {
  const p = sim.p;
  const gui = new GUI({ title: '参数 · 调试', width: 300 });
  gui.close();
  const rebuildPhys = () => hooks.rebuild(false);
  const rebuildGeo = () => hooks.rebuild(true);

  const fA = gui.addFolder('自动驾驶');
  fA.add(p, 'cruiseSpeed', 0.5, 6, 0.1).name('巡航速度 m/s');
  fA.add(p, 'latAccMax', 0.5, 4, 0.1).name('最大横向加速度');
  fA.add(p, 'lookaheadGain', 0.3, 2, 0.05).name('前视距离增益');
  fA.add(p, 'safetyStop', 1, 5, 0.1).name('安全制动距离 m');
  fA.add(p, 'inflate', 0.4, 1.5, 0.05).name('障碍膨胀半径 m');

  const fB = gui.addFolder('平衡控制 LQR');
  fB.add(p.Q, '0', 10, 20000, 10).name('Q φ').onFinishChange(rebuildPhys);
  fB.add(p.Q, '1', 0.1, 1000, 0.1).name('Q δ').onFinishChange(rebuildPhys);
  fB.add(p.Q, '2', 0.1, 2000, 0.1).name('Q φ̇').onFinishChange(rebuildPhys);
  fB.add(p.Q, '3', 0.01, 100, 0.01).name('Q δ̇').onFinishChange(rebuildPhys);
  fB.add(p.Q, '4', 0, 1, 0.001).name('Q Ω (动量轮去饱和)').onFinishChange(rebuildPhys);
  fB.add(p.R, '0', 0.01, 20, 0.01).name('R 动量轮').onFinishChange(rebuildPhys);
  fB.add(p.R, '1', 0.01, 20, 0.01).name('R 转向').onFinishChange(rebuildPhys);
  fB.add(p, 'useEstimator').name('使用 IMU 估计 (真车)');

  const fH = gui.addFolder('执行器硬件');
  fH.add(p, 'rwTorqueMax', 2, 40, 0.5).name('动量轮峰值力矩 N·m');
  fH.add(p, 'rwSpeedMax', 100, 800, 10).name('动量轮极限转速 rad/s');
  fH.add(p, 'rwMass', 0.5, 6, 0.1).name('飞轮质量 kg').onFinishChange(rebuildGeo);
  fH.add(p, 'steerTorqueMax', 2, 40, 0.5).name('转向峰值力矩 N·m');
  fH.add(p, 'driveTorqueMax', 5, 60, 1).name('轮毂电机力矩 N·m');
  fH.add(p, 'brakeDecelMax', 1, 7, 0.1).name('刹车减速度 m/s²');

  const fG = gui.addFolder('车架几何 (改动会重建三维模型)');
  fG.add(p, 'headAngleDeg', 64, 76, 0.5).name('头管角 °').onFinishChange(rebuildGeo);
  fG.add(p, 'trail', 0.0, 0.14, 0.002).name('拖曳距 m').onFinishChange(rebuildGeo);
  fG.add(p, 'wheelbase', 0.9, 1.25, 0.01).name('轴距 m').onFinishChange(rebuildGeo);
  fG.add(p, 'chainstay', 0.38, 0.5, 0.005).name('后下叉 m').onFinishChange(rebuildGeo);
  fG.close();

  const fS = gui.addFolder('传感器噪声');
  fS.add(p, 'gyroNoise', 0, 0.05, 0.001).name('陀螺噪声 rad/s');
  fS.add(p, 'gyroBias', -0.03, 0.03, 0.001).name('陀螺零偏 rad/s').onChange(() => (sim.sensors.bias = p.gyroBias));
  fS.add(p, 'accNoise', 0, 0.5, 0.01).name('加计噪声 m/s²');
  fS.close();

  const fSim = gui.addFolder('仿真');
  fSim.add(p, 'timeScale', 0.1, 3, 0.05).name('时间倍率');
  return gui;
}

// ---------------- 硬件 / 动力学面板 ----------------
export function renderBOM(sim) {
  const plant = sim.plant, b = plant.body, W = plant.W, wp = b.whipple;
  const groups = { B: '后车架 + 车载设备 (B)', H: '前叉 + 车把 (H)', R: '后轮 (R)', F: '前轮 (F)' };
  const rows = Object.entries(groups).map(([k, name]) => {
    const items = b.items.filter((it) => it.assy === k);
    const m = items.reduce((s, it) => s + it.mass, 0);
    return `<tr class="grp"><td colspan="3">${name}</td><td>${m.toFixed(2)} kg</td></tr>` +
      items.map((it) => `<tr><td>${it.name}</td><td class="spec">${it.spec || ''}</td><td>${it.pos[0].toFixed(2)}, ${it.pos[1].toFixed(2)}</td><td>${it.mass.toFixed(2)}</td></tr>`).join('');
  }).join('');
  // 自稳定速度区间（不加控制的裸车）
  let lo = null, hi = null;
  const eig = [];
  for (let v = 0; v <= 10.001; v += 0.05) {
    const { A } = linearize(plant, v);
    const ev = eigenvalues(A);
    const st = ev.filter((e) => Math.abs(e.re) > 1e-3 || Math.abs(e.im) > 1e-3).every((e) => e.re < 0);
    if (st && lo === null) lo = v;
    if (!st && lo !== null && hi === null) hi = v;
    if (Math.abs(v * 2 - Math.round(v * 2)) < 1e-6) eig.push({ v, ev });
  }
  const m2 = (A) => `<table class="mat">${A.map((r) => `<tr>${r.map((v) => `<td>${v.toFixed(3)}</td>`).join('')}</tr>`).join('')}</table>`;
  const K = sim.ctl.K || plant.gains[6].K;
  const kRow = (r) => r.map((v) => v.toFixed(1)).join(', ');
  return `
    <div class="bom-head"><h2>硬件清单 & 动力学模型</h2><button id="bomClose">✕</button></div>
    <div class="bom-grid">
      <div>
        <h3>物料清单（质量/位置决定动力学参数）</h3>
        <table class="bom"><tr><th>部件</th><th>规格</th><th>位置 x,y (m)</th><th>kg</th></tr>${rows}</table>
        <p class="note">总质量 <b>${b.totalMass.toFixed(2)} kg</b> · 质心 (${b.cg[0].toFixed(3)}, ${b.cg[1].toFixed(3)}) m · 飞轮转动惯量 I<sub>w</sub> = ${plant.Iw.toFixed(4)} kg·m²</p>
      </div>
      <div>
        <h3>Whipple–Carvallo 线性模型（Meijaard 2007）</h3>
        <p class="eq">M q̈ + v·C₁ q̇ + (g·K₀ + v²·K₂) q = [−τ<sub>rw</sub>, T<sub>δ</sub>]ᵀ, q = [φ, δ]ᵀ</p>
        <div class="mats"><div>M ${m2(W.M)}</div><div>C₁ ${m2(W.C1)}</div><div>K₀ ${m2(W.K0)}</div><div>K₂ ${m2(W.K2)}</div></div>
        <p class="note">几何：轴距 ${wp.w.toFixed(3)} m · 拖曳距 ${wp.c.toFixed(3)} m · 转向轴后倾 ${(wp.lambda * 57.3).toFixed(1)}° · 前叉偏移 ${(b.G.forkOffset * 1000).toFixed(0)} mm</p>
        <p class="note">无控制裸车的自稳定区间：<b>${lo !== null ? `${lo.toFixed(2)} – ${hi !== null ? hi.toFixed(2) : '>10'} m/s` : '无'}</b>（weave 模式在下界失稳，capsize 模式在上界失稳）</p>
        <h3>特征值（裸车，实部 max）</h3>
        <div class="eigs">${eig.map(({ v, ev }) => { const m = Math.max(...ev.filter((e) => Math.abs(e.re) > 1e-3 || Math.abs(e.im) > 1e-3).map((e) => e.re)); return `<span class="${m < 0 ? 'ok' : 'bad'}" title="v=${v}">${v.toFixed(1)}<i>${m.toFixed(2)}</i></span>`; }).join('')}</div>
        <h3>当前 LQR 增益 K(v = ${sim.s.v.toFixed(2)} m/s)</h3>
        <p class="eq">τ<sub>rw</sub> = −[${kRow(K[0])}]·x̃<br>T<sub>δ</sub> = T<sub>ff</sub> − [${kRow(K[1])}]·x̃<br><small>x̃ = [φ−φ<sub>ref</sub>, δ−δ<sub>ref</sub>, φ̇, δ̇, Ω]</small></p>
      </div>
    </div>`;
}
