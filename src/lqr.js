// 轻量矩阵运算 + 离散 Riccati 迭代求解 LQR 反馈增益 K。
// 系统规模很小（3 维），用稠密矩阵 + 简单算法即可，数值稳定。

// --- 基础矩阵工具 (二维数组, row-major) ---
export const mat = {
  zeros(r, c) {
    return Array.from({ length: r }, () => new Array(c).fill(0));
  },
  eye(n) {
    const m = mat.zeros(n, n);
    for (let i = 0; i < n; i++) m[i][i] = 1;
    return m;
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
  add(A, B) {
    return A.map((row, i) => row.map((v, j) => v + B[i][j]));
  },
  sub(A, B) {
    return A.map((row, i) => row.map((v, j) => v - B[i][j]));
  },
  scale(A, s) {
    return A.map((row) => row.map((v) => v * s));
  },
  // 通用矩阵求逆 (Gauss-Jordan with partial pivoting)
  inv(A) {
    const n = A.length;
    const M = A.map((row, i) => [...row, ...mat.eye(n)[i]]);
    for (let col = 0; col < n; col++) {
      // pivot
      let piv = col;
      for (let r = col + 1; r < n; r++)
        if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
      if (Math.abs(M[piv][col]) < 1e-12) throw new Error('矩阵奇异，无法求逆');
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
};

// 连续时间系统 (A, B) 用零阶保持近似离散化到步长 dt。
// 用矩阵指数级数: Ad = exp(A·dt), Bd = A^{-1}(Ad - I)B （A 奇异时退化为级数）。
function discretize(A, B, dt) {
  const n = A.length;
  // Ad = I + A dt + (A dt)^2/2! + ...  (级数, 小 dt 收敛快)
  const Adt = mat.scale(A, dt);
  let term = mat.eye(n);
  let Ad = mat.eye(n);
  // Bd 用 (∫ exp(Aτ)dτ) B ≈ (I dt + A dt^2/2! + ...) B
  let intTerm = mat.scale(mat.eye(n), dt);
  let intSum = mat.scale(mat.eye(n), dt);
  for (let k = 1; k < 20; k++) {
    term = mat.scale(mat.mul(term, Adt), 1 / k);
    Ad = mat.add(Ad, term);
    // 积分级数: 下一项 = 上一项 · (A dt) /(k+1)
    intTerm = mat.scale(mat.mul(intTerm, Adt), 1 / (k + 1));
    intSum = mat.add(intSum, intTerm);
  }
  const Bd = mat.mul(intSum, B);
  return { Ad, Bd };
}

// 离散 LQR: 迭代求解离散代数 Riccati 方程, 返回反馈增益 K (1×n)。
// 控制律 u = -K x （连续使用时直接代入, dt 取得足够小即等价于连续 LQR）。
export function lqrGain(A, B, Qdiag, R, dt = 0.001) {
  const { Ad, Bd } = discretize(A, B, dt);
  const n = A.length;
  const Q = mat.zeros(n, n);
  for (let i = 0; i < n; i++) Q[i][i] = Qdiag[i];
  const Rm = [[R]];
  const AdT = mat.T(Ad);
  const BdT = mat.T(Bd);

  let P = Q.map((row) => row.slice());
  for (let iter = 0; iter < 2000; iter++) {
    // P_next = Ad^T P Ad - Ad^T P Bd (R + Bd^T P Bd)^-1 Bd^T P Ad + Q
    const PAd = mat.mul(P, Ad);
    const AdTPAd = mat.mul(AdT, PAd);
    const BdTP = mat.mul(BdT, P);
    const S = mat.add(Rm, mat.mul(BdTP, Bd)); // 1x1
    const Sinv = mat.inv(S);
    const K1 = mat.mul(mat.mul(mat.mul(AdT, mat.mul(P, Bd)), Sinv), mat.mul(BdT, PAd));
    const Pn = mat.add(mat.sub(AdTPAd, K1), Q);
    if (mat.fro(mat.sub(Pn, P)) < 1e-9) { P = Pn; break; }
    P = Pn;
  }
  // K = (R + Bd^T P Bd)^-1 Bd^T P Ad
  const S = mat.add(Rm, mat.mul(mat.mul(BdT, P), Bd));
  const K = mat.mul(mat.mul(mat.inv(S), mat.mul(BdT, P)), Ad);
  return K[0]; // 长度 n 的数组
}
