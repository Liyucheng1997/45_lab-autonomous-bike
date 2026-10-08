// PBR 材质库 + 程序化贴图（用 canvas 生成，零外部资源）。
import * as THREE from 'three';

function canvasTex(w, h, draw, { repeat = [1, 1], srgb = true } = {}) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  draw(ctx, w, h);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(...repeat);
  t.anisotropy = 8;
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function noise(ctx, w, h, base, amp, n = 1) {
  const img = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    let v = base;
    for (let k = 0; k < n; k++) v += (Math.random() - 0.5) * amp;
    img.data[4 * i] = img.data[4 * i + 1] = img.data[4 * i + 2] = Math.max(0, Math.min(255, v));
    img.data[4 * i + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

export const tex = {};

export function buildTextures() {
  // 胎面花纹（凹凸贴图）：城市/砂石混合胎的人字块
  tex.tread = canvasTex(256, 64, (ctx, w, h) => {
    ctx.fillStyle = '#202020'; ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = '#d0d0d0';
    for (let i = 0; i < 8; i++) {
      const x = i * 32;
      ctx.beginPath(); ctx.moveTo(x + 4, 6); ctx.lineTo(x + 16, 30); ctx.lineTo(x + 4, 58); ctx.lineTo(x + 12, 58); ctx.lineTo(x + 24, 30); ctx.lineTo(x + 12, 6); ctx.fill();
      ctx.fillRect(x + 24, 4, 6, 12); ctx.fillRect(x + 24, 48, 6, 12);
    }
  }, { repeat: [10, 1], srgb: false });
  // 沥青：颗粒 + 斑块
  tex.asphalt = canvasTex(512, 512, (ctx, w, h) => {
    noise(ctx, w, h, 78, 70, 2);
    ctx.globalAlpha = 0.08;
    for (let i = 0; i < 70; i++) {
      ctx.fillStyle = Math.random() > 0.5 ? '#000' : '#888';
      ctx.beginPath(); ctx.arc(Math.random() * w, Math.random() * h, 10 + Math.random() * 50, 0, 7); ctx.fill();
    }
    ctx.globalAlpha = 1;
  }, { repeat: [1, 1] });
  tex.asphaltRough = canvasTex(256, 256, (ctx, w, h) => noise(ctx, w, h, 220, 60, 2), { repeat: [1, 1], srgb: false });
  // 草地
  tex.grass = canvasTex(512, 512, (ctx, w, h) => {
    ctx.fillStyle = '#4d7a35'; ctx.fillRect(0, 0, w, h);
    for (let i = 0; i < 14000; i++) {
      const g = 90 + Math.random() * 70;
      ctx.fillStyle = `rgba(${40 + Math.random() * 50},${g},${30 + Math.random() * 25},0.55)`;
      const x = Math.random() * w, y = Math.random() * h;
      ctx.fillRect(x, y, 1.5, 3 + Math.random() * 4);
    }
    ctx.globalAlpha = 0.12;
    for (let i = 0; i < 40; i++) {
      ctx.fillStyle = Math.random() > 0.5 ? '#2f4f1f' : '#8aa05a';
      ctx.beginPath(); ctx.arc(Math.random() * w, Math.random() * h, 20 + Math.random() * 60, 0, 7); ctx.fill();
    }
  }, { repeat: [40, 40] });
  // 碳纤维编织
  tex.carbon = canvasTex(64, 64, (ctx, w, h) => {
    ctx.fillStyle = '#101114'; ctx.fillRect(0, 0, w, h);
    for (let y = 0; y < 8; y++)
      for (let x = 0; x < 8; x++) {
        const g = ctx.createLinearGradient(x * 8, y * 8, x * 8 + ((x + y) % 2 ? 8 : 0), y * 8 + ((x + y) % 2 ? 0 : 8));
        g.addColorStop(0, '#1c1e23'); g.addColorStop(0.5, '#3a3d45'); g.addColorStop(1, '#1c1e23');
        ctx.fillStyle = g; ctx.fillRect(x * 8 + 0.5, y * 8 + 0.5, 7, 7);
      }
  }, { repeat: [6, 2] });
  // 建筑立面窗户
  tex.facade = canvasTex(256, 256, (ctx, w, h) => {
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, w, h);
    for (let y = 0; y < 4; y++)
      for (let x = 0; x < 4; x++) {
        const lit = Math.random();
        ctx.fillStyle = lit > 0.85 ? '#5d6f80' : '#3a4a5a';
        ctx.fillRect(x * 64 + 10, y * 64 + 12, 44, 38);
        ctx.fillStyle = 'rgba(255,255,255,0.18)';
        ctx.fillRect(x * 64 + 10, y * 64 + 12, 44, 6);
        ctx.fillStyle = '#d8d4cc';
        ctx.fillRect(x * 64 + 31, y * 64 + 12, 2, 38);
      }
  }, { repeat: [1, 1] });
  // 纸箱
  tex.cardboard = canvasTex(128, 128, (ctx, w, h) => {
    noise(ctx, w, h, 0, 0);
    ctx.fillStyle = '#b48a56'; ctx.fillRect(0, 0, w, h);
    for (let i = 0; i < 800; i++) { ctx.fillStyle = `rgba(90,60,30,${Math.random() * 0.15})`; ctx.fillRect(Math.random() * w, Math.random() * h, 2, 2); }
    ctx.fillStyle = 'rgba(220,200,160,0.85)'; ctx.fillRect(56, 0, 16, h);
  });
  // 锥桶反光条
  return tex;
}

export const M = {};

export function buildMaterials() {
  const std = (o) => new THREE.MeshStandardMaterial(o);
  const phy = (o) => new THREE.MeshPhysicalMaterial(o);
  // 车架：珍珠白烤漆 + 清漆层
  M.frame = phy({ color: 0xeceff1, metalness: 0.15, roughness: 0.32, clearcoat: 1, clearcoatRoughness: 0.08 });
  // 强调色：机器人部件的亮橙
  M.accent = phy({ color: 0xff5f1f, metalness: 0.2, roughness: 0.35, clearcoat: 0.8, clearcoatRoughness: 0.15 });
  M.anodBlack = std({ color: 0x1a1c20, metalness: 0.75, roughness: 0.38 });
  M.anodGrey = std({ color: 0x5b6069, metalness: 0.85, roughness: 0.3 });
  M.alu = std({ color: 0xc9ced6, metalness: 1, roughness: 0.28 });
  M.chrome = std({ color: 0xf2f4f7, metalness: 1, roughness: 0.08 });
  M.steel = std({ color: 0xa9aeb6, metalness: 1, roughness: 0.22 });
  M.darkSteel = std({ color: 0x3a3e45, metalness: 0.9, roughness: 0.35 });
  M.rubber = std({ color: 0x141414, metalness: 0, roughness: 0.92 });
  M.tire = std({ color: 0x1b1b1b, metalness: 0, roughness: 0.88, bumpMap: tex.tread, bumpScale: 2.5 });
  M.gumwall = std({ color: 0x8b5a32, metalness: 0, roughness: 0.75 });
  M.plastic = std({ color: 0x23262b, metalness: 0.05, roughness: 0.55 });
  M.plasticGrey = std({ color: 0x8d939b, metalness: 0.05, roughness: 0.5 });
  M.carbon = phy({ color: 0xffffff, map: tex.carbon, metalness: 0.3, roughness: 0.35, clearcoat: 1, clearcoatRoughness: 0.05 });
  M.leather = std({ color: 0x2a2421, metalness: 0, roughness: 0.62 });
  M.glassDark = phy({ color: 0x0b0d12, metalness: 0.1, roughness: 0.05, clearcoat: 1, transmission: 0, reflectivity: 0.9 });
  M.lens = phy({ color: 0x06070a, metalness: 0.4, roughness: 0.02, clearcoat: 1, iridescence: 0.6 });
  M.polycarb = phy({ color: 0xcfe6ff, metalness: 0, roughness: 0.05, transmission: 0.92, thickness: 0.003, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false });
  M.pcb = std({ color: 0x0f5c3a, metalness: 0.3, roughness: 0.6 });
  M.copper = std({ color: 0xb87333, metalness: 1, roughness: 0.3 });
  M.cable = std({ color: 0x111214, metalness: 0.1, roughness: 0.6 });
  M.cableOrange = std({ color: 0xff7a1a, metalness: 0.1, roughness: 0.5 });
  M.yellow = std({ color: 0xf2c200, metalness: 0.1, roughness: 0.45 });
  M.red = phy({ color: 0xd8141a, metalness: 0.05, roughness: 0.3, clearcoat: 1 });
  M.white = std({ color: 0xf5f5f2, metalness: 0.05, roughness: 0.45 });
  M.ledGreen = std({ color: 0x0a2, emissive: 0x00ff66, emissiveIntensity: 2.2 });
  M.ledBlue = std({ color: 0x024, emissive: 0x2a8cff, emissiveIntensity: 2.4 });
  M.headlight = std({ color: 0xffffff, emissive: 0xfff4dd, emissiveIntensity: 3 });
  M.tailLight = std({ color: 0x400, emissive: 0xff1010, emissiveIntensity: 1.2 });
  M.statusRing = std({ color: 0x013, emissive: 0x18c8ff, emissiveIntensity: 2.5 });
  M.laser = new THREE.MeshBasicMaterial({ color: 0xff3355, transparent: true, opacity: 0.85 });

  // 场景
  M.asphalt = std({ map: tex.asphalt, roughnessMap: tex.asphaltRough, color: 0x8a8d92, roughness: 0.95, metalness: 0 });
  M.grass = std({ map: tex.grass, roughness: 1, metalness: 0, color: 0xc8d8b0 });
  M.curb = std({ color: 0xb9b7b0, roughness: 0.9 });
  M.paintWhite = std({ color: 0xf2f2ee, roughness: 0.7, polygonOffset: true, polygonOffsetFactor: -2 });
  M.paintYellow = std({ color: 0xf0c020, roughness: 0.7, polygonOffset: true, polygonOffsetFactor: -2 });
  M.coneOrange = std({ color: 0xff5a10, roughness: 0.55 });
  M.reflective = std({ color: 0xf4f4f4, roughness: 0.25, metalness: 0.3 });
  M.trunk = std({ color: 0x5a4030, roughness: 0.95 });
  M.leaves = [0x4f7f3a, 0x5d8c3f, 0x3f6e33, 0x6c9a48].map((c) => std({ color: c, roughness: 0.9, flatShading: true }));
  M.cardboard = std({ map: tex.cardboard, roughness: 0.9 });
  M.carGlass = phy({ color: 0x1a2430, metalness: 0.2, roughness: 0.05, clearcoat: 1 });
  M.lampHead = std({ color: 0x333, emissive: 0xfff2d0, emissiveIntensity: 0.4 });
  M.skin = std({ color: 0xe0b896, roughness: 0.7 });
  return M;
}

export function carPaint(color) {
  return new THREE.MeshPhysicalMaterial({ color, metalness: 0.6, roughness: 0.3, clearcoat: 1, clearcoatRoughness: 0.05 });
}
export function facadeMat(color) {
  return new THREE.MeshStandardMaterial({ color, map: tex.facade, roughness: 0.85 });
}
