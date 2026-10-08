// 程序化建模工具：变截面扫掠管、放样曲面、齿轮轮廓、皮带/链条路径、合并。
import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);

// 沿曲线扫掠椭圆截面。radius(t) → [ra, rb]：ra 沿 refAxis（通常为车身横向 z），rb 沿另一法向。
export function sweep(curve, { segments = 32, radial = 14, radius = () => [0.01, 0.01], refAxis = V(0, 0, 1), caps = true } = {}) {
  const pos = [], nor = [], uv = [], idx = [];
  const A = new THREE.Vector3(), B = new THREE.Vector3(), T = new THREE.Vector3(), P = new THREE.Vector3();
  const ref = refAxis.clone().normalize();
  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    curve.getPointAt(t, P);
    curve.getTangentAt(t, T);
    A.copy(ref).addScaledVector(T, -ref.dot(T));
    if (A.lengthSq() < 1e-8) A.set(0, 1, 0).addScaledVector(T, -T.y);
    A.normalize();
    B.crossVectors(T, A).normalize();
    const [ra, rb] = radius(t);
    for (let j = 0; j <= radial; j++) {
      const th = (j / radial) * Math.PI * 2;
      const c = Math.cos(th), s = Math.sin(th);
      pos.push(P.x + A.x * ra * c + B.x * rb * s, P.y + A.y * ra * c + B.y * rb * s, P.z + A.z * ra * c + B.z * rb * s);
      const nx = A.x * c / ra + B.x * s / rb, ny = A.y * c / ra + B.y * s / rb, nz = A.z * c / ra + B.z * s / rb;
      const nl = Math.hypot(nx, ny, nz);
      nor.push(nx / nl, ny / nl, nz / nl);
      uv.push(t, j / radial);
    }
  }
  const row = radial + 1;
  for (let i = 0; i < segments; i++)
    for (let j = 0; j < radial; j++) {
      const a = i * row + j, b = a + row, c = b + 1, d = a + 1;
      idx.push(a, d, b, b, d, c);
    }
  if (caps) {
    for (const end of [0, segments]) {
      const t = end / segments;
      curve.getPointAt(t, P);
      curve.getTangentAt(t, T);
      const sgn = end === 0 ? -1 : 1;
      const ci = pos.length / 3;
      pos.push(P.x, P.y, P.z); nor.push(T.x * sgn, T.y * sgn, T.z * sgn); uv.push(t, 0.5);
      const base = pos.length / 3;
      for (let j = 0; j <= radial; j++) {
        const k = (end * row + j) * 3;
        pos.push(pos[k], pos[k + 1], pos[k + 2]); nor.push(T.x * sgn, T.y * sgn, T.z * sgn); uv.push(t, j / radial);
      }
      for (let j = 0; j < radial; j++) sgn > 0 ? idx.push(ci, base + j, base + j + 1) : idx.push(ci, base + j + 1, base + j);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  return g;
}

// 直线管（两点）
export function tube(a, b, r0, r1 = r0, opts = {}) {
  const curve = new THREE.LineCurve3(a, b);
  const rr = (t) => {
    const r = r0 + (r1 - r0) * t;
    return [r, r];
  };
  return sweep(curve, { segments: 2, radial: opts.radial || 16, radius: opts.radius || rr, refAxis: opts.refAxis, caps: opts.caps !== false });
}

// 平滑曲线管
export function curveTube(points, radius, opts = {}) {
  const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal');
  const rf = typeof radius === 'function' ? radius : () => [radius, radius];
  return sweep(curve, { segments: opts.segments || 40, radial: opts.radial || 12, radius: rf, refAxis: opts.refAxis, caps: opts.caps !== false });
}

// 放样：sections 为若干个闭合环（每环点数相同），按顺序连成曲面，可选封口
export function loft(sections, { capStart = true, capEnd = true } = {}) {
  const M = sections[0].length;
  const pos = [], idx = [], uv = [];
  sections.forEach((ring, i) => ring.forEach((p, j) => { pos.push(p.x, p.y, p.z); uv.push(i / (sections.length - 1), j / M); }));
  for (let i = 0; i < sections.length - 1; i++)
    for (let j = 0; j < M; j++) {
      const a = i * M + j, b = i * M + ((j + 1) % M), c = (i + 1) * M + ((j + 1) % M), d = (i + 1) * M + j;
      idx.push(a, d, b, b, d, c);
    }
  const cap = (ring, flip) => {
    const c = ring.reduce((s, p) => s.add(p), V()).multiplyScalar(1 / ring.length);
    const ci = pos.length / 3;
    pos.push(c.x, c.y, c.z); uv.push(0.5, 0.5);
    const base = pos.length / 3;
    ring.forEach((p) => { pos.push(p.x, p.y, p.z); uv.push(0, 0); });
    for (let j = 0; j < M; j++) flip ? idx.push(ci, base + ((j + 1) % M), base + j) : idx.push(ci, base + j, base + ((j + 1) % M));
  };
  if (capStart) cap(sections[0], false);
  if (capEnd) cap(sections[sections.length - 1], true);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// 超椭圆截面环（在 y-z 平面，位于 x 处）
export function superRing(x, cy, hw, hh, n = 2.6, M = 28, bottomFlat = 1) {
  const ring = [];
  for (let j = 0; j < M; j++) {
    const th = (j / M) * Math.PI * 2;
    const c = Math.cos(th), s = Math.sin(th);
    const z = hw * Math.sign(c) * Math.abs(c) ** (2 / n);
    let y = hh * Math.sign(s) * Math.abs(s) ** (2 / n);
    if (y < 0) y *= bottomFlat;
    ring.push(V(x, cy + y, z));
  }
  return ring;
}

// 齿轮/链轮/同步带轮外形（Shape，在 x-y 平面）
export function gearShape(teeth, rRoot, rTip, { toothFrac = 0.45, holes = [] } = {}) {
  const s = new THREE.Shape();
  const step = (Math.PI * 2) / teeth;
  for (let i = 0; i < teeth; i++) {
    const a0 = i * step;
    const a1 = a0 + step * (0.5 - toothFrac / 2);
    const a2 = a0 + step * (0.5 - toothFrac / 4);
    const a3 = a0 + step * (0.5 + toothFrac / 4);
    const a4 = a0 + step * (0.5 + toothFrac / 2);
    const pts = [[rRoot, a0], [rRoot, a1], [rTip, a2], [rTip, a3], [rRoot, a4]];
    pts.forEach(([r, a], k) => {
      const x = r * Math.cos(a), y = r * Math.sin(a);
      i === 0 && k === 0 ? s.moveTo(x, y) : s.lineTo(x, y);
    });
  }
  s.closePath();
  holes.forEach((h) => s.holes.push(h));
  return s;
}

export function circlePath(r, cx = 0, cy = 0, n = 32, clockwise = true) {
  const p = new THREE.Path();
  p.absarc(cx, cy, r, 0, Math.PI * 2, clockwise);
  return p;
}

// 两圆外公切线组成的开口皮带回路（2D），返回按弧长均匀分布的点 [{p:[x,y], t:[tx,ty]}]
export function beltLoop(A, r1, B, r2, spacing) {
  const dx = B[0] - A[0], dy = B[1] - A[1];
  const d = Math.hypot(dx, dy);
  const thu = Math.atan2(dy, dx);
  const s = (r1 - r2) / d;
  const a = Math.acos(Math.max(-1, Math.min(1, s)));
  const pts = [];
  const arc = (c, r, from, to, n) => { for (let i = 0; i <= n; i++) { const t = from + ((to - from) * i) / n; pts.push([c[0] + r * Math.cos(t), c[1] + r * Math.sin(t)]); } };
  // 大圆 A：从 thu + a 绕远侧到 thu − a + 2π；小圆 B：从 thu − a 到 thu + a
  arc(A, r1, thu + a, thu + 2 * Math.PI - a, 60);
  arc(B, r2, thu - a, thu + a, 30);
  pts.push(pts[0]);
  // 均匀重采样
  const out = [];
  let acc = 0, next = 0;
  for (let i = 1; i < pts.length; i++) {
    const p = pts[i - 1], q = pts[i];
    const L = Math.hypot(q[0] - p[0], q[1] - p[1]);
    while (next <= acc + L) {
      const t = (next - acc) / L;
      out.push({ p: [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t], t: [(q[0] - p[0]) / L, (q[1] - p[1]) / L] });
      next += spacing;
    }
    acc += L;
  }
  return out;
}

export function merge(geos) {
  const prepared = geos.map((g) => (g.index ? g.toNonIndexed() : g)).map((g) => {
    if (!g.attributes.uv) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array((g.attributes.position.count) * 2), 2));
    if (!g.attributes.normal) g.computeVertexNormals();
    for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv'].includes(k)) g.deleteAttribute(k);
    return g;
  });
  return mergeGeometries(prepared, false);
}

// 把几何体放到 a→b 的线段上（几何体本身沿 +Y、中心在原点）
export function alignY(geo, a, b) {
  const dir = b.clone().sub(a);
  const L = dir.length();
  const q = new THREE.Quaternion().setFromUnitVectors(V(0, 1, 0), dir.normalize());
  const m = new THREE.Matrix4().compose(a.clone().add(b).multiplyScalar(0.5), q, V(1, 1, 1));
  geo.applyMatrix4(m);
  return { geo, L };
}

export function xform(geo, { pos = [0, 0, 0], rot = [0, 0, 0], scale = [1, 1, 1] } = {}) {
  const m = new THREE.Matrix4().compose(V(...pos), new THREE.Quaternion().setFromEuler(new THREE.Euler(...rot)), V(...scale));
  return geo.applyMatrix4(m);
}
