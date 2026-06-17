import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { params } from './params.js';
import { initialState, step, clamp } from './physics.js';
import { Controller } from './controller.js';
import { Bike } from './bike.js';
import { buildGUI } from './ui.js';

// ---------- 场景 ----------
const app = document.getElementById('app');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
app.appendChild(renderer.domElement);

const scene = new THREE.Scene();
// 白天天空：上蓝下浅，淡雾衔接地平线
const skyTop = new THREE.Color(0x77b0ec);
const skyBottom = new THREE.Color(0xdfeefb);
scene.background = skyTop.clone();
scene.fog = new THREE.Fog(0xcfe2f4, 35, 120);

// 渐变天空穹顶
const sky = new THREE.Mesh(
  new THREE.SphereGeometry(300, 32, 16),
  new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: {
      top: { value: skyTop }, bottom: { value: skyBottom }, offset: { value: 0.18 },
    },
    vertexShader: `varying vec3 vP; void main(){ vP = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0);} `,
    fragmentShader: `varying vec3 vP; uniform vec3 top; uniform vec3 bottom; uniform float offset;
      void main(){ float h = normalize(vP).y * 0.5 + 0.5 + offset; gl_FragColor = vec4(mix(bottom, top, clamp(h,0.0,1.0)), 1.0);} `,
  })
);
scene.add(sky);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 200);
camera.position.set(3.2, 2.2, 3.6);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.5, 0);

// ---------- 光照（白天）----------
scene.add(new THREE.HemisphereLight(0xbcd6f5, 0x9a9488, 1.1)); // 天空蓝 / 地面暖
const sun = new THREE.DirectionalLight(0xfff6e6, 2.6);          // 正午暖白阳光
sun.position.set(8, 14, 6);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -15; sun.shadow.camera.right = 15;
sun.shadow.camera.top = 15; sun.shadow.camera.bottom = -15;
sun.shadow.camera.near = 0.5; sun.shadow.camera.far = 60;
sun.shadow.bias = -0.0003;
scene.add(sun);
scene.add(new THREE.AmbientLight(0xffffff, 0.25)); // 轻微补光，避免暗部死黑

// ---------- 地面（浅色沥青/水泥）----------
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(400, 400),
  new THREE.MeshStandardMaterial({ color: 0x9aa0a8, roughness: 0.95, metalness: 0 })
);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(400, 400, 0xc4cad2, 0xb2b8c0);
grid.material.opacity = 0.5; grid.material.transparent = true;
scene.add(grid);

// ---------- 自行车 + 控制器 + 状态 ----------
const bike = new Bike(params);
bike.rollGroup.traverse((o) => { if (o.isMesh) { o.castShadow = true; } });
scene.add(bike.root);

const controller = new Controller(params);
let state = initialState();

// 路径可视化
let pathLine = null;
function drawPath(pts) {
  if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); pathLine = null; }
  if (!pts || !pts.length) return;
  const g = new THREE.BufferGeometry().setFromPoints(
    pts.map((p) => new THREE.Vector3(p.x, 0.02, p.y))
  );
  pathLine = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xf9e2af }));
  scene.add(pathLine);
}

// ---------- 交互动作 ----------
const actions = {
  // 横向冲击扰动：静止时动量轮可回正的量级约 ±1.5 rad/s；行驶中转向辅助可承受更大。
  kick() { state.thetaDot += (Math.random() > 0.5 ? 1 : -1) * (0.8 + Math.random() * 0.6); },
  reset() { state = initialState(); controller.setWaypoints([]); drawPath([]); params.vTarget = 0; },
  figureEight() {
    const pts = [];
    const R = 6;
    for (let i = 0; i <= 120; i++) {
      const t = (i / 120) * Math.PI * 2;
      pts.push({ x: R * Math.sin(t), y: R * Math.sin(t) * Math.cos(t) }); // 8 字 (lemniscate-ish)
    }
    controller.setWaypoints(pts);
    drawPath(pts);
    params.vTarget = 2.5;
  },
  clearPath() { controller.setWaypoints([]); drawPath([]); },
};

window.addEventListener('keydown', (e) => {
  if (e.code === 'Space') { e.preventDefault(); actions.kick(); }
  if (e.code === 'KeyR') actions.reset();
});

buildGUI(params, controller, actions);

// ---------- HUD + 示波器 ----------
const hud = document.getElementById('hud');
const scope = document.getElementById('scope');
const sctx = scope.getContext('2d');
const rollHist = new Array(scope.width).fill(0);

function updateHUD(u, fallen) {
  const deg = (r) => (r * 180 / Math.PI).toFixed(1);
  const statusCls = fallen ? 'warn' : (Math.abs(state.theta) < 0.05 ? 'ok' : '');
  const statusTxt = fallen ? '已倒地 (按 R 复位)' : (Math.abs(state.theta) < 0.05 ? '平衡中' : '调整中');
  hud.innerHTML = `
    <div id="title">🚲 自动驾驶自行车 · XUAN 方法</div>
    <div class="row"><span>状态</span><span class="${statusCls}">${statusTxt}</span></div>
    <div class="row"><span>倾角 θ</span><span>${deg(state.theta)}°</span></div>
    <div class="row"><span>倾角速度</span><span>${deg(state.thetaDot)}°/s</span></div>
    <div class="row"><span>动量轮 Ω</span><span>${state.omega.toFixed(0)} rad/s</span></div>
    <div class="row"><span>动量轮力矩</span><span>${u.tau.toFixed(2)} N·m</span></div>
    <div class="row"><span>速度 v</span><span>${state.v.toFixed(2)} m/s</span></div>
    <div class="row"><span>转向 δ</span><span>${deg(state.delta)}°</span></div>
    <div class="row"><span>LQR K</span><span>${controller.K.map((k) => k.toFixed(1)).join(', ')}</span></div>
  `;
}

function drawScope() {
  rollHist.push(state.theta);
  rollHist.shift();
  const w = scope.width, h = scope.height;
  sctx.clearRect(0, 0, w, h);
  // 零线
  sctx.strokeStyle = '#313244'; sctx.beginPath(); sctx.moveTo(0, h / 2); sctx.lineTo(w, h / 2); sctx.stroke();
  // ±倒地阈值
  const scale = (h / 2) / params.maxFallAngle;
  sctx.strokeStyle = '#45475a'; sctx.setLineDash([3, 3]);
  for (const s of [0.3, -0.3]) { sctx.beginPath(); const y = h / 2 - s * scale; sctx.moveTo(0, y); sctx.lineTo(w, y); sctx.stroke(); }
  sctx.setLineDash([]);
  // 倾角曲线
  sctx.strokeStyle = '#89dceb'; sctx.lineWidth = 1.5; sctx.beginPath();
  for (let i = 0; i < w; i++) {
    const y = clamp(h / 2 - rollHist[i] * scale, 1, h - 1);
    i === 0 ? sctx.moveTo(i, y) : sctx.lineTo(i, y);
  }
  sctx.stroke();
  sctx.fillStyle = '#7f849c'; sctx.font = '10px monospace';
  sctx.fillText('倾角 θ (时间→)', 6, 12);
}

// ---------- 固定步长物理 + 渲染循环 ----------
const clock = new THREE.Clock();
let acc = 0;
let fallen = false;
let lastU = { tau: 0, deltaCmd: 0, vCmd: 0 };

function animate() {
  requestAnimationFrame(animate);
  let frameDt = Math.min(clock.getDelta(), 0.05); // 防止卡顿后大跳
  acc += frameDt;

  while (acc >= params.dt) {
    if (!fallen) {
      const u = controller.compute(state);
      state = step(state, u, params.dt, params);
      lastU = u;
      if (Math.abs(state.theta) > params.maxFallAngle) fallen = true;
    }
    acc -= params.dt;
  }

  bike.update(state);
  // 倒地时把车身压到地面
  if (fallen) bike.rollGroup.rotation.x = Math.sign(state.theta) * Math.PI / 2 * 0.9;

  // 相机柔和跟随
  controls.target.lerp(new THREE.Vector3(state.x, 0.5, state.y), 0.05);
  controls.update();

  updateHUD(lastU, fallen);
  drawScope();
  renderer.render(scene, camera);
}
animate();

// ---------- resize ----------
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// 复位时也重置倒地标志
const origReset = actions.reset;
actions.reset = function () { origReset(); fallen = false; };
