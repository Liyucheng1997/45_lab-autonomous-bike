import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { params } from './core/params.js';
import { Simulation } from './core/simulation.js';
import { MODE_NAMES } from './core/autopilot.js';
import { buildTextures, buildMaterials } from './render/materials.js';
import { BikeModel } from './render/bikeModel.js';
import { buildSky, buildLights, buildGround, buildObstacleMeshes, updateObstacleMeshes, addObstacleMesh } from './render/environment.js';
import { Overlays } from './render/overlays.js';
import { HUD, buildGUI, renderBOM } from './ui.js';

const $ = (id) => document.getElementById(id);

// ---------------- 渲染器 / 场景 ----------------
const app = $('app');
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.92;
app.appendChild(renderer.domElement);


const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0xc4d6ea, 90, 520);
const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.03, 1500);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 0.6;
controls.maxDistance = 80;

buildTextures();
buildMaterials();
const sunDir = buildSky(scene, renderer);
const lights = buildLights(scene, sunDir);
buildGround(scene);

// ---------------- 仿真 + 模型 ----------------
const sim = new Simulation(params);
const obstacleMeshes = buildObstacleMeshes(scene, sim.obstacles);
let bike = new BikeModel(sim);
scene.add(bike.root);
const overlays = new Overlays(scene, sim);
const hud = new HUD(sim);

// 硬件标注：工程图式引出线。锚点随车身运动，标签在屏幕两侧排成两列，自动避让。
const callouts = $('callouts');
const calloutSvg = callouts.querySelector('svg');
let labels = [];
let labelsOn = false;
function buildLabels() {
  callouts.querySelectorAll('.hwlabel').forEach((e) => e.remove());
  labels = sim.plant.body.items.filter((it) => it.label).map((it) => {
    const div = document.createElement('div');
    div.className = 'hwlabel';
    div.innerHTML = `<b>${it.name}</b><em>${it.mass.toFixed(2)} kg</em><span>${it.spec || ''}</span>`;
    callouts.appendChild(div);
    return { it, div, local: new THREE.Vector3(...it.pos) };
  });
}
buildLabels();
function updateLabels() {
  callouts.style.display = labelsOn ? '' : 'none';
  if (!labelsOn) return;
  const W = window.innerWidth, H = window.innerHeight;
  const v = new THREE.Vector3();
  const c = new THREE.Vector3(sim.plant.body.cg[0], sim.plant.body.cg[1], 0);
  bike.rollGroup.localToWorld(c).project(camera);
  const cx = ((c.x + 1) / 2) * W;
  const pts = labels.map((l) => {
    v.copy(l.local);
    bike.rollGroup.localToWorld(v).project(camera);
    return { l, x: ((v.x + 1) / 2) * W, y: ((1 - v.y) / 2) * H, ok: v.z < 1 };
  }).filter((p) => p.ok);
  let svg = '';
  for (const side of [-1, 1]) {
    const grp = pts.filter((p) => (side < 0 ? p.x < cx : p.x >= cx)).sort((a, b) => a.y - b.y);
    const boxH = 46, gap = 6;
    const ys = grp.map((p) => p.y);
    for (let i = 1; i < ys.length; i++) ys[i] = Math.max(ys[i], ys[i - 1] + boxH + gap);
    const over = ys.length ? ys[ys.length - 1] + boxH / 2 - (H - 250) : 0;
    if (over > 0) for (let i = 0; i < ys.length; i++) ys[i] -= over;
    for (let i = ys.length - 2; i >= 0; i--) ys[i] = Math.min(ys[i], ys[i + 1] - boxH - gap);
    const colX = Math.min(W - 250, Math.max(250, cx + side * Math.min(420, W * 0.27)));
    grp.forEach((p, i) => {
      const y = Math.max(70, ys[i]);
      const d = p.l.div;
      d.style.left = (side < 0 ? colX - 230 : colX) + 'px';
      d.style.top = y - boxH / 2 + 'px';
      d.classList.toggle('left', side < 0);
      const ex = colX;
      const mx = ex - side * 24;
      svg += `<polyline points="${p.x.toFixed(1)},${p.y.toFixed(1)} ${mx.toFixed(1)},${y.toFixed(1)} ${ex.toFixed(1)},${y.toFixed(1)}" />`;
      svg += `<circle cx="${p.x.toFixed(1)}" cy="${p.y.toFixed(1)}" r="3.5" />`;
    });
  }
  calloutSvg.innerHTML = svg;
}

function rebuild(geometry) {
  sim.rebuild();
  if (geometry) {
    scene.remove(bike.root);
    bike.root.traverse((o) => { if (o.isMesh) o.geometry.dispose(); });
    bike = new BikeModel(sim);
    scene.add(bike.root);
    const G = sim.plant.body.G;
    sim.autopilot.lidar.mountH = G.lidar[1];
    sim.autopilot.lidar.mountX = G.lidar[0];
    sim.autopilot.camera.mount = G.camera;
    buildLabels();
  }
  toast(geometry ? '几何已变更：三维模型、质量属性与 47 组 LQR 增益已重建' : `LQR 增益表已重算（${sim.buildMs.toFixed(0)} ms）`);
  if ($('bom').classList.contains('show')) openBOM();
}
const gui = buildGUI(sim, { rebuild });

// ---------------- UI 交互 ----------------
let camMode = 'orbit';
function setMode(m, goal = null) {
  if (sim.fallen && m !== 'IDLE') { toast('车已倒地，请先复位 (R)'); return; }
  sim.autopilot.setMode(m, goal);
  if (m === 'GOTO' && !goal) { sim.autopilot.setMode('IDLE'); pendingGoal = true; toast('点击地面设置导航目标'); }
  else pendingGoal = false;
  syncButtons();
}
let pendingGoal = false;
function syncButtons() {
  document.querySelectorAll('[data-mode]').forEach((b) => b.classList.toggle('on', pendingGoal ? b.dataset.mode === 'GOTO' : b.dataset.mode === sim.autopilot.mode));
  document.querySelectorAll('[data-cam]').forEach((b) => b.classList.toggle('on', b.dataset.cam === camMode));
  $('btnLabels').classList.toggle('on', labelsOn);
  $('btnSensors').classList.toggle('on', overlays.show.lidar);
}
document.querySelectorAll('[data-mode]').forEach((b) => b.addEventListener('click', () => setMode(b.dataset.mode)));
document.querySelectorAll('[data-cam]').forEach((b) => b.addEventListener('click', () => setCam(b.dataset.cam)));
$('btnPush').addEventListener('click', push);
$('btnReset').addEventListener('click', reset);
$('btnLabels').addEventListener('click', () => {
  labelsOn = !labelsOn;
  document.body.classList.toggle('labels-on', labelsOn);
  if (labelsOn && camMode !== 'orbit') setCam('orbit');
  syncButtons();
});
$('btnSensors').addEventListener('click', () => {
  const v = !overlays.show.lidar;
  ['lidar', 'grid', 'path', 'detections'].forEach((k) => overlays.setVisible(k, v));
  syncButtons();
});
$('btnBOM').addEventListener('click', () => ($('bom').classList.contains('show') ? $('bom').classList.remove('show') : openBOM()));
function openBOM() {
  $('bom').innerHTML = renderBOM(sim);
  $('bom').classList.add('show');
  $('bomClose').onclick = () => $('bom').classList.remove('show');
}

function push() {
  // 可恢复极限（实测）：静止 ≈4 N·s（动量轮力矩有限），1.5 m/s ≈8，≥3 m/s >20（转向接管平衡）
  const v = Math.abs(sim.s.v);
  const J = (Math.random() > 0.5 ? 1 : -1) * (v < 0.8 ? 3 : v < 2.2 ? 6 : 12);
  sim.push(J);
  toast(`侧向冲量 ${Math.abs(J).toFixed(1)} N·s（${J > 0 ? '向右' : '向左'}）`);
}
function reset() {
  sim.reset();
  pendingGoal = false;
  overlays.lastPath = undefined;
  syncButtons();
  autoStart = sim.t + 2.5;
}

let toastTimer = 0;
function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.style.opacity = 1;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (t.style.opacity = 0), 2600);
}

const keys = new Set();
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  keys.add(e.code);
  const map = { Digit1: 'IDLE', Digit2: 'ROUTE', Digit3: 'GOTO', Digit4: 'MANUAL', KeyE: 'ESTOP' };
  if (map[e.code]) setMode(map[e.code]);
  if (e.code === 'Space') { e.preventDefault(); push(); }
  if (e.code === 'KeyR') reset();
  if (e.code === 'KeyC') setCam({ chase: 'orbit', orbit: 'top', top: 'fpv', fpv: 'chase' }[camMode]);
  if (e.code === 'Escape') $('bom').classList.remove('show');
});
window.addEventListener('keyup', (e) => keys.delete(e.code));

// 点击地面：导航目标 / Shift 放锥桶
const ray = new THREE.Raycaster();
let downXY = null;
renderer.domElement.addEventListener('pointerdown', (e) => (downXY = [e.clientX, e.clientY]));
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downXY || Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]) > 5) return;
  const ndc = new THREE.Vector2((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  ray.setFromCamera(ndc, camera);
  const hit = new THREE.Vector3();
  if (!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), hit)) return;
  if (e.shiftKey) {
    const o = { shape: 'cyl', kind: 'cone', x: hit.x, y: hit.z, r: 0.17, h: 0.7 };
    sim.obstacles.push(o);
    addObstacleMesh(scene, obstacleMeshes, o);
    toast('已放置锥桶 —— 激光雷达扫描到后会自动绕行');
  } else if (pendingGoal || sim.autopilot.mode === 'GOTO') {
    setMode('GOTO', [hit.x, hit.z]);
    toast(`导航目标 (${hit.x.toFixed(1)}, ${hit.z.toFixed(1)})`);
  }
});

// ---------------- 相机 ----------------
const camState = { pos: new THREE.Vector3(), look: new THREE.Vector3(), topH: 34 };
function bikePos() {
  return new THREE.Vector3(sim.s.x, 0, sim.s.y);
}
function setCam(m) {
  camMode = m;
  controls.enabled = m === 'orbit';
  const b = bikePos();
  if (m === 'orbit') {
    const fwd = new THREE.Vector3(Math.cos(sim.s.psi), 0, Math.sin(sim.s.psi));
    const right = new THREE.Vector3(-fwd.z, 0, fwd.x);
    camera.position.copy(b).addScaledVector(fwd, 2.1).addScaledVector(right, 2.0).add(new THREE.Vector3(0, 1.2, 0));
    controls.target.copy(b).add(new THREE.Vector3(Math.cos(sim.s.psi) * 0.5, 0.6, Math.sin(sim.s.psi) * 0.5));
  }
  camera.fov = m === 'fpv' ? 70 : 50;
  camera.updateProjectionMatrix();
  syncButtons();
}
renderer.domElement.addEventListener('wheel', (e) => { if (camMode === 'top') camState.topH = Math.min(120, Math.max(8, camState.topH * (e.deltaY > 0 ? 1.1 : 0.9))); }, { passive: true });

let lastBike = bikePos();
function updateCamera(dt) {
  const b = bikePos();
  const s = sim.s;
  const fwd = new THREE.Vector3(Math.cos(s.psi), 0, Math.sin(s.psi));
  const right = new THREE.Vector3(-fwd.z, 0, fwd.x);
  if (camMode === 'orbit') {
    const d = b.clone().sub(lastBike);
    camera.position.add(d);
    controls.target.add(d);
    controls.update();
  } else if (camMode === 'chase') {
    const want = b.clone().addScaledVector(fwd, -3.6).addScaledVector(right, 1.1).add(new THREE.Vector3(0, 1.65, 0));
    const look = b.clone().addScaledVector(fwd, 2.0).add(new THREE.Vector3(0, 0.55, 0));
    const k = 1 - Math.exp(-dt * 3.5);
    camState.pos.lerp(want, camState.pos.lengthSq() ? k : 1);
    camState.look.lerp(look, camState.look.lengthSq() ? k * 1.5 : 1);
    camera.position.copy(camState.pos);
    camera.lookAt(camState.look);
  } else if (camMode === 'top') {
    const want = b.clone().add(new THREE.Vector3(0, camState.topH, 0.01));
    camera.position.lerp(want, 1 - Math.exp(-dt * 4));
    camera.lookAt(camera.position.x, 0, camera.position.z - 0.01);
  } else if (camMode === 'fpv') {
    bike.onboardCam.getWorldPosition(camera.position);
    bike.onboardCam.getWorldQuaternion(camera.quaternion);
  }
  lastBike = b;
  // 阴影相机跟随车辆
  lights.sun.position.copy(b).addScaledVector(lights.sunDir, 40);
  lights.sun.target.position.copy(b);
}

// ---------------- 画中画：车载相机 + 检测框 ----------------
const pipHole = $('pipHole');
const pipCanvas = $('pipOverlay');
const pctx = pipCanvas.getContext('2d');
function renderPiP() {
  const r = pipHole.getBoundingClientRect();
  if (r.width < 10 || getComputedStyle($('pipWrap')).display === 'none' || camMode === 'fpv') return;
  const cam = bike.onboardCam;
  cam.aspect = r.width / r.height;
  cam.updateProjectionMatrix();
  cam.updateMatrixWorld();
  const vis = overlays.group.visible;
  overlays.group.visible = false;
  renderer.setScissorTest(true);
  const y = window.innerHeight - r.bottom;
  renderer.setScissor(r.left, y, r.width, r.height);
  renderer.setViewport(r.left, y, r.width, r.height);
  renderer.render(scene, cam);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
  overlays.group.visible = vis;
  // 2D 检测框
  if (pipCanvas.width !== r.width * 2) { pipCanvas.width = r.width * 2; pipCanvas.height = r.height * 2; }
  const W = pipCanvas.width, H = pipCanvas.height;
  pctx.clearRect(0, 0, W, H);
  pctx.lineWidth = 3;
  pctx.font = '600 20px "Segoe UI", sans-serif';
  const v = new THREE.Vector3();
  for (const d of sim.autopilot.camera.detections) {
    const o = d.obj;
    const hw = o.shape === 'box' ? Math.max(o.lx, o.ly) / 2 : o.r + 0.05;
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, ok = true;
    for (const dx of [-hw, hw]) for (const dz of [-hw, hw]) for (const h of [0, o.h]) {
      v.set(o.x + dx, h, o.y + dz).project(cam);
      if (v.z > 1) { ok = false; break; }
      const px = ((v.x + 1) / 2) * W, py = ((1 - v.y) / 2) * H;
      x0 = Math.min(x0, px); x1 = Math.max(x1, px); y0 = Math.min(y0, py); y1 = Math.max(y1, py);
    }
    if (!ok || x1 < 0 || x0 > W) continue;
    const col = d.kind === 'ped' ? '#ff40c0' : d.kind === 'cone' ? '#ffa020' : '#40ff90';
    pctx.strokeStyle = col;
    pctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    const txt = `${d.label} ${(d.conf * 100).toFixed(0)}% · ${d.dist.toFixed(1)}m`;
    const tw = pctx.measureText(txt).width + 10;
    pctx.fillStyle = col;
    pctx.fillRect(x0, y0 - 26, tw, 26);
    pctx.fillStyle = '#0b0e14';
    pctx.fillText(txt, x0 + 5, y0 - 7);
  }
  $('pipInfo').textContent = `${sim.autopilot.camera.detections.length} 个目标`;
}

// ---------------- 主循环 ----------------
const clock = new THREE.Clock();
let acc = 0, fps = 60, renderT = 0, autoStart = 2.5, wasFallen = false;
function frame() {
  requestAnimationFrame(frame);
  tick(Math.min(clock.getDelta(), 0.1));
}
function tick(dt) {
  renderT += dt;
  fps += (1 / Math.max(dt, 1e-3) - fps) * 0.05;

  // 手动遥控
  if (sim.autopilot.mode === 'MANUAL') {
    const m = sim.autopilot.manual;
    if (keys.has('KeyW') || keys.has('ArrowUp')) m.v = Math.min(5, m.v + 1.6 * dt);
    if (keys.has('KeyS') || keys.has('ArrowDown')) m.v = Math.max(-0.8, m.v - 2.4 * dt);
    const turn = (keys.has('KeyD') || keys.has('ArrowRight') ? 1 : 0) - (keys.has('KeyA') || keys.has('ArrowLeft') ? 1 : 0);
    m.k += ((turn * 0.35) - m.k) * Math.min(1, dt * 4);
  }
  // 开场：先原地平衡 2.5 s，再自动出发巡航
  if (autoStart !== null && sim.t > autoStart && sim.autopilot.mode === 'IDLE' && !pendingGoal) {
    autoStart = null;
    setMode('ROUTE');
    toast('自检完成 → 开始沿车道自动巡航');
  }

  acc += dt * params.timeScale;
  let steps = 0;
  while (acc >= params.dt && steps < 120) { sim.stepOnce(); acc -= params.dt; steps++; }
  if (steps >= 120) acc = 0;
  if (sim.fallen && !wasFallen) toast('倾角超过极限，车辆倒地。按 R 复位');
  wasFallen = sim.fallen;
  if (sim.autopilot.mode !== (syncButtons.last || '')) { syncButtons.last = sim.autopilot.mode; syncButtons(); }

  bike.update(sim, renderT);
  updateObstacleMeshes(obstacleMeshes, sim.t);
  overlays.update(renderT);
  updateCamera(dt);

  renderer.render(scene, camera);
  updateLabels();
  renderPiP();

  hud.sample();
  hud.draw();
  hud.text(renderT, fps);
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

setCam('orbit');
syncButtons();
$('loading').style.opacity = 0;
setTimeout(() => $('loading').remove(), 600);
frame();
window.__sim = sim; // 调试用
window.__dbg = { sim, camera, controls, setCam, bike: () => bike, overlays, tick };
