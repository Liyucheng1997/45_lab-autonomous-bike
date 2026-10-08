// 在线建图 + 路径规划：
//   OccupancyGrid  —— 由激光雷达点云更新的对数几率 (log-odds) 占据栅格，射线穿过的格子被清除，
//                     所以移动的行人离开后，地图也会“放行”。
//   distance field —— 两遍倒角距离变换，得到每格到最近障碍的距离，用于膨胀和代价。
//   A*             —— 8 邻域，代价 = 步长 × (1 + 贴近障碍惩罚 + 离开道路惩罚)。
//   平滑           —— 视线裁剪 (line-of-sight shortcut) + 重采样 + 梯度平滑。

export class OccupancyGrid {
  constructor({ res = 0.25, xmin = -64, ymin = -48, xmax = 64, ymax = 48 } = {}) {
    this.res = res;
    this.xmin = xmin; this.ymin = ymin;
    this.nx = Math.ceil((xmax - xmin) / res);
    this.ny = Math.ceil((ymax - ymin) / res);
    const N = this.nx * this.ny;
    this.logodds = new Float32Array(N);
    this.dist = new Float32Array(N);       // 到最近占据格的距离 (m)
    this.prior = new Float32Array(N);      // 先验代价（高精地图：路外更贵）
    this.seen = new Uint8Array(N);
    this.version = 0;
  }
  idx(ix, iy) { return iy * this.nx + ix; }
  toCell(x, y) { return [Math.floor((x - this.xmin) / this.res), Math.floor((y - this.ymin) / this.res)]; }
  toWorld(ix, iy) { return [this.xmin + (ix + 0.5) * this.res, this.ymin + (iy + 0.5) * this.res]; }
  inside(ix, iy) { return ix >= 0 && iy >= 0 && ix < this.nx && iy < this.ny; }
  occupied(i) { return this.logodds[i] > 0.85; }

  setPrior(fn) {
    for (let iy = 0; iy < this.ny; iy++)
      for (let ix = 0; ix < this.nx; ix++) {
        const [x, y] = this.toWorld(ix, iy);
        this.prior[this.idx(ix, iy)] = fn(x, y);
      }
  }
  clear() { this.logodds.fill(0); this.seen.fill(0); this.version++; }

  // 用一帧点云更新。origin = 雷达位置 (three 坐标 [x, h, z])。
  integrate(lidar) {
    const [ox, , oz] = lidar.origin;
    const [cx, cy] = this.toCell(ox, oz);
    const P = lidar.points;
    const HIT = 0.9, MISS = -0.35, LMAX = 3.5, LMIN = -2.0;
    const lo = this.logodds;
    // 1) 只用障碍物高度范围内的点 (0.12 – 2.2 m) 作为“命中”
    const hits = new Map();
    for (let k = 0; k < lidar.count; k++) {
      const h = P[3 * k + 1];
      if (h < 0.12 || h > 2.2) continue;
      const [ix, iy] = this.toCell(P[3 * k], P[3 * k + 2]);
      if (!this.inside(ix, iy)) continue;
      hits.set(this.idx(ix, iy), [ix, iy]);
    }
    // 2) 地面点和远处的点：沿射线清除（Bresenham）
    const az = lidar.azSteps;
    const clearRay = (tx, ty, excl) => {
      let x0 = cx, y0 = cy;
      const dx = Math.abs(tx - x0), dy = -Math.abs(ty - y0);
      const sx = x0 < tx ? 1 : -1, sy = y0 < ty ? 1 : -1;
      let err = dx + dy;
      for (let guard = 0; guard < 400; guard++) {
        if (x0 === tx && y0 === ty) break;
        if (this.inside(x0, y0)) {
          const i = this.idx(x0, y0);
          if (!hits.has(i)) { lo[i] = Math.max(LMIN, lo[i] + MISS); this.seen[i] = 1; }
        }
        const e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
      }
    };
    // 清除终点：该方位最近的障碍命中；没有障碍就清到 12 m
    const nearest = new Float32Array(az).fill(1e9);
    const nearXY = new Array(az);
    let k = 0;
    for (let c = 0; c < lidar.channels; c++)
      for (let a = 0; a < az; a++) {
        const kind = lidar.hitKind[c * az + a];
        if (!kind) continue;
        const px = P[3 * k], ph = P[3 * k + 1], pz = P[3 * k + 2];
        k++;
        if (kind !== 2 || ph < 0.12 || ph > 2.2) continue;
        const d = Math.hypot(px - ox, pz - oz);
        if (d < nearest[a]) { nearest[a] = d; nearXY[a] = [px, pz]; }
      }
    for (let a = 0; a < az; a++) {
      let tx, ty;
      const th = (a / az) * 2 * Math.PI + lidar.heading;
      const L = nearXY[a] ? Math.min(nearest[a], 20) : 12;
      [tx, ty] = this.toCell(ox + L * Math.cos(th), oz + L * Math.sin(th));
      clearRay(tx, ty);
    }
    for (const [i] of hits) { lo[i] = Math.min(LMAX, lo[i] + HIT); this.seen[i] = 1; }
    this.version++;
  }

  // 两遍倒角距离变换 (3-4 chamfer)，单位米
  computeDistance(maxD = 4) {
    const { nx, ny, res } = this;
    const D = this.dist;
    const INF = 1e6;
    for (let i = 0; i < D.length; i++) D[i] = this.occupied(i) ? 0 : INF;
    const a = res, b = res * Math.SQRT2;
    for (let y = 0; y < ny; y++)
      for (let x = 0; x < nx; x++) {
        const i = y * nx + x;
        let d = D[i];
        if (d === 0) continue;
        if (x > 0) d = Math.min(d, D[i - 1] + a);
        if (y > 0) {
          d = Math.min(d, D[i - nx] + a);
          if (x > 0) d = Math.min(d, D[i - nx - 1] + b);
          if (x < nx - 1) d = Math.min(d, D[i - nx + 1] + b);
        }
        D[i] = d;
      }
    for (let y = ny - 1; y >= 0; y--)
      for (let x = nx - 1; x >= 0; x--) {
        const i = y * nx + x;
        let d = D[i];
        if (d === 0) continue;
        if (x < nx - 1) d = Math.min(d, D[i + 1] + a);
        if (y < ny - 1) {
          d = Math.min(d, D[i + nx] + a);
          if (x < nx - 1) d = Math.min(d, D[i + nx + 1] + b);
          if (x > 0) d = Math.min(d, D[i + nx - 1] + b);
        }
        D[i] = Math.min(d, maxD);
      }
  }

  distAt(x, y) {
    const [ix, iy] = this.toCell(x, y);
    if (!this.inside(ix, iy)) return 0;
    return this.dist[this.idx(ix, iy)];
  }
}

// ---------------- A* ----------------
class MinHeap {
  constructor() { this.k = []; this.v = []; }
  get size() { return this.k.length; }
  push(key, val) {
    const k = this.k, v = this.v;
    let i = k.length;
    k.push(key); v.push(val);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (k[p] <= key) break;
      k[i] = k[p]; v[i] = v[p]; i = p;
    }
    k[i] = key; v[i] = val;
  }
  pop() {
    const k = this.k, v = this.v;
    const top = v[0];
    const lk = k.pop(), lv = v.pop();
    if (k.length) {
      let i = 0;
      const n = k.length;
      for (;;) {
        let l = 2 * i + 1, r = l + 1, m = i;
        let mk = lk;
        if (l < n && k[l] < mk) { m = l; mk = k[l]; }
        if (r < n && k[r] < mk) { m = r; }
        if (m === i) break;
        k[i] = k[m]; v[i] = v[m]; i = m;
      }
      k[i] = lk; v[i] = lv;
    }
    return top;
  }
}

export function planPath(grid, start, goal, { inflate = 0.7, maxExpand = 120000 } = {}) {
  const { nx, ny, res } = grid;
  const [sx, sy] = grid.toCell(start[0], start[1]);
  let [gx, gy] = grid.toCell(goal[0], goal[1]);
  if (!grid.inside(sx, sy) || !grid.inside(gx, gy)) return null;
  const D = grid.dist, prior = grid.prior;
  const blocked = (i) => D[i] < inflate;
  const sI = grid.idx(sx, sy);
  const gI = grid.idx(gx, gy);
  if (blocked(gI)) return null;
  const N = nx * ny;
  const g = new Float32Array(N).fill(Infinity);
  const from = new Int32Array(N).fill(-1);
  const closed = new Uint8Array(N);
  const open = new MinHeap();
  g[sI] = 0;
  const h = (ix, iy) => Math.hypot(ix - gx, iy - gy) * res;
  open.push(h(sx, sy), sI);
  const NB = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  let expanded = 0, found = false;
  while (open.size) {
    const i = open.pop();
    if (closed[i]) continue;
    closed[i] = 1;
    if (i === gI) { found = true; break; }
    if (++expanded > maxExpand) break;
    const ix = i % nx, iy = (i / nx) | 0;
    for (const [dx, dy, c] of NB) {
      const jx = ix + dx, jy = iy + dy;
      if (jx < 0 || jy < 0 || jx >= nx || jy >= ny) continue;
      const j = jy * nx + jx;
      if (closed[j]) continue;
      // 起点附近允许穿过膨胀区（否则车贴着障碍时无路可走）
      if (blocked(j) && Math.hypot(jx - sx, jy - sy) * res > inflate + 0.3) continue;
      if (D[j] < 0.2) continue;
      const prox = D[j] < 2.2 ? (2.2 - D[j]) * 1.6 : 0;
      const ng = g[i] + c * res * (1 + prox + prior[j]);
      if (ng < g[j]) {
        g[j] = ng;
        from[j] = i;
        open.push(ng + h(jx, jy), j);
      }
    }
  }
  if (!found) return null;
  const cells = [];
  for (let i = gI; i !== -1; i = from[i]) cells.push(i);
  cells.reverse();
  const pts = cells.map((i) => grid.toWorld(i % nx, (i / nx) | 0));
  pts[0] = [start[0], start[1]];
  pts[pts.length - 1] = [goal[0], goal[1]];
  return smoothPath(grid, pts, inflate);
}

// 视线检查：线段上每个采样点都离障碍足够远，且不更多地离开道路
function lineFree(grid, a, b, clearance) {
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.ceil(L / (grid.res * 0.5));
  let prior = 0;
  for (let k = 0; k <= n; k++) {
    const t = k / n;
    const x = a[0] + (b[0] - a[0]) * t, y = a[1] + (b[1] - a[1]) * t;
    if (grid.distAt(x, y) < clearance) return false;
    const [ix, iy] = grid.toCell(x, y);
    if (grid.inside(ix, iy)) prior = Math.max(prior, grid.prior[grid.idx(ix, iy)]);
  }
  return prior < 0.5;
}

export function smoothPath(grid, pts, inflate) {
  if (pts.length < 3) return pts;
  // 1) 视线裁剪
  const out = [pts[0]];
  let i = 0;
  while (i < pts.length - 1) {
    let j = Math.min(pts.length - 1, i + 60);
    while (j > i + 1 && !lineFree(grid, pts[i], pts[j], inflate + 0.25)) j--;
    out.push(pts[j]);
    i = j;
  }
  // 2) 0.4 m 重采样
  const rs = resample(out, 0.4);
  // 3) 梯度平滑（保持端点）
  const P = rs.map((p) => p.slice());
  const minD = inflate + 0.1;
  for (let it = 0; it < 60; it++) {
    for (let k = 1; k < P.length - 1; k++) {
      const ox = P[k][0], oy = P[k][1];
      for (let d = 0; d < 2; d++) {
        const lap = P[k - 1][d] + P[k + 1][d] - 2 * P[k][d];
        P[k][d] += 0.25 * lap + 0.08 * (rs[k][d] - P[k][d]);
      }
      // 平滑不能把路径拉进膨胀区（弯道“切角”撞障碍）
      if (grid.distAt(P[k][0], P[k][1]) < Math.min(minD, grid.distAt(ox, oy))) { P[k][0] = ox; P[k][1] = oy; }
    }
  }
  return P;
}

export function resample(pts, step) {
  const out = [pts[0].slice()];
  let carry = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i];
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    let s = step - carry;
    while (s <= L) {
      const t = s / L;
      out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
      s += step;
    }
    carry = L - (s - step);
  }
  const last = pts[pts.length - 1];
  const e = out[out.length - 1];
  if (Math.hypot(e[0] - last[0], e[1] - last[1]) > step * 0.3) out.push(last.slice());
  return out;
}
