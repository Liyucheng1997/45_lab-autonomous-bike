// 轻量数学工具：稠密小矩阵运算 + 常用标量函数。
// 系统规模很小（≤ 6 维），二维数组 row-major 足够且易读。

export const mat = {
  zeros(r, c) {
    return Array.from({ length: r }, () => new Array(c).fill(0));
  },
  eye(n) {
    const m = mat.zeros(n, n);
    for (let i = 0; i < n; i++) m[i][i] = 1;
    return m;
  },
  diag(v) {
    const m = mat.zeros(v.length, v.length);
    v.forEach((x, i) => (m[i][i] = x));
    return m;
  },
  copy(A) {
    return A.map((r) => r.slice());
  },
  T(A) {
    const r = A.length, c = A[0].length, B = mat.zeros(c, r);
    for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) B[j][i] = A[i][j];
    return B;
  },
  mul(A, B) {
    const r = A.length, n = B.length, c = B[0].length;
    const C = mat.zeros(r, c);
    for (let i = 0; i < r; i++)
      for (let k = 0; k < n; k++) {
        const a = A[i][k];
        if (a === 0) continue;
        for (let j = 0; j < c; j++) C[i][j] += a * B[k][j];
      }
    return C;
  },
  mulVec(A, x) {
    return A.map((row) => row.reduce((s, a, j) => s + a * x[j], 0));
  },
  add(A, B) {
    return A.map((row, i) => row.map((v, j) => v + B[i][j]));
  },
  sub(A, B) {
    return A.map((row, i) => row.map((v, j) => v - B[i][j]));
  },
  scale(A, s) {
    return A.map((row) => row.map((v) => v * s));
  },
  // Gauss-Jordan 求逆（部分主元）
  inv(A) {
    const n = A.length;
    const M = A.map((row, i) => [...row, ...mat.eye(n)[i]]);
    for (let col = 0; col < n; col++) {
      let piv = col;
      for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
      if (Math.abs(M[piv][col]) < 1e-14) throw new Error('矩阵奇异，无法求逆');
      [M[col], M[piv]] = [M[piv], M[col]];
      const d = M[col][col];
      for (let j = 0; j < 2 * n; j++) M[col][j] /= d;
      for (let r = 0; r < n; r++) {
        if (r === col) continue;
        const f = M[r][col];
        if (f === 0) continue;
        for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[col][j];
      }
    }
    return M.map((row) => row.slice(n));
  },
  fro(A) {
    let s = 0;
    for (const row of A) for (const v of row) s += v * v;
    return Math.sqrt(s);
  },
  // 2×2 特征值（用于自稳定性分析）
};

// 实矩阵特征值（QR 迭代 + Hessenberg，够用于 ≤6 维的稳定性分析）。返回 [{re, im}]
export function eigenvalues(A0) {
  const n = A0.length;
  const A = mat.copy(A0);
  // 化为上 Hessenberg
  for (let m = 1; m < n - 1; m++) {
    let x = 0, i = m;
    for (let j = m; j < n; j++) if (Math.abs(A[j][m - 1]) > Math.abs(x)) { x = A[j][m - 1]; i = j; }
    if (i !== m) {
      for (let j = m - 1; j < n; j++) [A[i][j], A[m][j]] = [A[m][j], A[i][j]];
      for (let j = 0; j < n; j++) [A[j][i], A[j][m]] = [A[j][m], A[j][i]];
    }
    if (x !== 0) {
      for (let i2 = m + 1; i2 < n; i2++) {
        let y = A[i2][m - 1];
        if (y !== 0) {
          y /= x;
          A[i2][m - 1] = y;
          for (let j = m; j < n; j++) A[i2][j] -= y * A[m][j];
          for (let j = 0; j < n; j++) A[j][m] += y * A[j][i2];
        }
      }
    }
  }
  for (let i = 2; i < n; i++) for (let j = 0; j < i - 1; j++) A[i][j] = 0;
  // Hessenberg QR（Numerical Recipes hqr）
  const wr = new Array(n).fill(0), wi = new Array(n).fill(0);
  let nn = n - 1, t = 0;
  const anorm = A.reduce((s, r) => s + r.reduce((a, b) => a + Math.abs(b), 0), 0);
  while (nn >= 0) {
    let its = 0, l;
    do {
      for (l = nn; l >= 1; l--) {
        const s = Math.abs(A[l - 1][l - 1]) + Math.abs(A[l][l]) || anorm;
        if (Math.abs(A[l][l - 1]) + s === s) { A[l][l - 1] = 0; break; }
      }
      const x = A[nn][nn];
      if (l === nn) { wr[nn] = x + t; wi[nn--] = 0; }
      else {
        const y = A[nn - 1][nn - 1], w = A[nn][nn - 1] * A[nn - 1][nn];
        if (l === nn - 1) {
          const p = 0.5 * (y - x), q = p * p + w, z = Math.sqrt(Math.abs(q));
          const xx = x + t;
          if (q >= 0) {
            const zz = p + (p >= 0 ? Math.abs(z) : -Math.abs(z));
            wr[nn - 1] = wr[nn] = xx + zz;
            if (zz) wr[nn] = xx - w / zz;
            wi[nn - 1] = wi[nn] = 0;
          } else {
            wr[nn - 1] = wr[nn] = xx + p;
            wi[nn - 1] = -(wi[nn] = z);
          }
          nn -= 2;
        } else {
          if (its === 60) throw new Error('eigenvalues: 不收敛');
          let xx = x, yy = y, ww = w;
          if (its === 10 || its === 20) {
            t += xx;
            for (let i = 0; i <= nn; i++) A[i][i] -= xx;
            const s = Math.abs(A[nn][nn - 1]) + Math.abs(A[nn - 1][nn - 2]);
            yy = xx = 0.75 * s;
            ww = -0.4375 * s * s;
          }
          ++its;
          let m, p, q, r, z;
          for (m = nn - 2; m >= l; m--) {
            z = A[m][m];
            r = xx - z;
            const s2 = yy - z;
            p = (r * s2 - ww) / A[m + 1][m] + A[m][m + 1];
            q = A[m + 1][m + 1] - z - r - s2;
            r = A[m + 2][m + 1];
            const s = Math.abs(p) + Math.abs(q) + Math.abs(r);
            p /= s; q /= s; r /= s;
            if (m === l) break;
            const u = Math.abs(A[m][m - 1]) * (Math.abs(q) + Math.abs(r));
            const v = Math.abs(p) * (Math.abs(A[m - 1][m - 1]) + Math.abs(z) + Math.abs(A[m + 1][m + 1]));
            if (u + v === v) break;
          }
          for (let i = m + 2; i <= nn; i++) {
            A[i][i - 2] = 0;
            if (i !== m + 2) A[i][i - 3] = 0;
          }
          for (let k = m; k <= nn - 1; k++) {
            if (k !== m) {
              p = A[k][k - 1]; q = A[k + 1][k - 1]; r = 0;
              if (k !== nn - 1) r = A[k + 2][k - 1];
              if ((xx = Math.abs(p) + Math.abs(q) + Math.abs(r)) !== 0) { p /= xx; q /= xx; r /= xx; }
            }
            const s = Math.sqrt(p * p + q * q + r * r) * (p >= 0 ? 1 : -1);
            if (s !== 0) {
              if (k === m) { if (l !== m) A[k][k - 1] = -A[k][k - 1]; }
              else A[k][k - 1] = -s * xx;
              p += s; xx = p / s; yy = q / s; z = r / s; q /= p; r /= p;
              for (let j = k; j <= nn; j++) {
                p = A[k][j] + q * A[k + 1][j];
                if (k !== nn - 1) { p += r * A[k + 2][j]; A[k + 2][j] -= p * z; }
                A[k + 1][j] -= p * yy; A[k][j] -= p * xx;
              }
              const mmin = nn < k + 3 ? nn : k + 3;
              for (let i = l; i <= mmin; i++) {
                p = xx * A[i][k] + yy * A[i][k + 1];
                if (k !== nn - 1) { p += z * A[i][k + 2]; A[i][k + 2] -= p * r; }
                A[i][k + 1] -= p * q; A[i][k] -= p;
              }
            }
          }
        }
      }
    } while (l < nn - 1);
  }
  return wr.map((re, i) => ({ re, im: wi[i] }));
}

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;
export const deg = (r) => (r * 180) / Math.PI;
export const rad = (d) => (d * Math.PI) / 180;

export function wrapAngle(a) {
  a = (a + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
}

// 可复现的随机数（mulberry32）+ 高斯
export function makeRng(seed = 1) {
  let s = seed >>> 0;
  const rand = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  let spare = null;
  rand.gauss = () => {
    if (spare !== null) { const v = spare; spare = null; return v; }
    let u, v, r;
    do { u = rand() * 2 - 1; v = rand() * 2 - 1; r = u * u + v * v; } while (r >= 1 || r === 0);
    const f = Math.sqrt((-2 * Math.log(r)) / r);
    spare = v * f;
    return u * f;
  };
  return rand;
}
