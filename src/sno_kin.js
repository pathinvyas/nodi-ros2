/* ============================== SnoBots: kinematics (no 3D library needed) ==============================
   Robots are described like a URDF: links + joints (origin xyz/rpy, axis, limits). Transforms are 3x4
   row-major Float64Arrays [r00 r01 r02 tx | r10 r11 r12 ty | r20 r21 r22 tz]. Frames follow ROS: x forward,
   y left, z up, and joint rotations follow the right-hand rule. */
const K3 = {
  I() { return new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]); },
  // URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  fromRPY(x, y, z, r, p, w) {
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(w), sy = Math.sin(w);
    return new Float64Array([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y, -sp, cp * sr, cp * cr, z]);
  },
  mul(a, b, o) {
    o = o || new Float64Array(12);
    const a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3], a4 = a[4], a5 = a[5], a6 = a[6], a7 = a[7], a8 = a[8], a9 = a[9], a10 = a[10], a11 = a[11];
    const b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3], b4 = b[4], b5 = b[5], b6 = b[6], b7 = b[7], b8 = b[8], b9 = b[9], b10 = b[10], b11 = b[11];
    o[0] = a0 * b0 + a1 * b4 + a2 * b8; o[1] = a0 * b1 + a1 * b5 + a2 * b9; o[2] = a0 * b2 + a1 * b6 + a2 * b10; o[3] = a0 * b3 + a1 * b7 + a2 * b11 + a3;
    o[4] = a4 * b0 + a5 * b4 + a6 * b8; o[5] = a4 * b1 + a5 * b5 + a6 * b9; o[6] = a4 * b2 + a5 * b6 + a6 * b10; o[7] = a4 * b3 + a5 * b7 + a6 * b11 + a7;
    o[8] = a8 * b0 + a9 * b4 + a10 * b8; o[9] = a8 * b1 + a9 * b5 + a10 * b9; o[10] = a8 * b2 + a9 * b6 + a10 * b10; o[11] = a8 * b3 + a9 * b7 + a10 * b11 + a11;
    return o;
  },
  // rotation about a unit axis (Rodrigues), no translation
  axisAngle(x, y, z, t, o) {
    o = o || new Float64Array(12);
    const c = Math.cos(t), s = Math.sin(t), C = 1 - c;
    o[0] = c + x * x * C; o[1] = x * y * C - z * s; o[2] = x * z * C + y * s; o[3] = 0;
    o[4] = y * x * C + z * s; o[5] = c + y * y * C; o[6] = y * z * C - x * s; o[7] = 0;
    o[8] = z * x * C - y * s; o[9] = z * y * C + x * s; o[10] = c + z * z * C; o[11] = 0;
    return o;
  },
  inv(a) {
    return new Float64Array([a[0], a[4], a[8], -(a[0] * a[3] + a[4] * a[7] + a[8] * a[11]), a[1], a[5], a[9], -(a[1] * a[3] + a[5] * a[7] + a[9] * a[11]), a[2], a[6], a[10], -(a[2] * a[3] + a[6] * a[7] + a[10] * a[11])]);
  },
  apply(T, x, y, z) { return [T[0] * x + T[1] * y + T[2] * z + T[3], T[4] * x + T[5] * y + T[6] * z + T[7], T[8] * x + T[9] * y + T[10] * z + T[11]]; },
  rot(T, x, y, z) { return [T[0] * x + T[1] * y + T[2] * z, T[4] * x + T[5] * y + T[6] * z, T[8] * x + T[9] * y + T[10] * z]; },
  pos(T) { return [T[3], T[7], T[11]]; },
  rpy(T) { const p = Math.asin(Math.max(-1, Math.min(1, -T[8]))); return [Math.atan2(T[9], T[10]), p, Math.atan2(T[4], T[0])]; },
  quat(T) {
    const m00 = T[0], m11 = T[5], m22 = T[10], tr = m00 + m11 + m22; let x, y, z, w;
    if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (T[9] - T[6]) / s; y = (T[2] - T[8]) / s; z = (T[4] - T[1]) / s; }
    else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (T[9] - T[6]) / s; x = 0.25 * s; y = (T[1] + T[4]) / s; z = (T[2] + T[8]) / s; }
    else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (T[2] - T[8]) / s; x = (T[1] + T[4]) / s; y = 0.25 * s; z = (T[6] + T[9]) / s; }
    else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (T[4] - T[1]) / s; x = (T[2] + T[8]) / s; y = (T[6] + T[9]) / s; z = 0.25 * s; }
    return { x, y, z, w };
  },
  pose(x, y, z, roll, pitch, yaw) { return K3.fromRPY(x, y, z, roll, pitch, yaw); },
};
const wrapPi = (a) => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };
const clampN = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
const lerpN = (a, b, t) => a + (b - a) * t;
const approach = (v, t, d) => v < t ? Math.min(t, v + d) : Math.max(t, v - d);
const smooth01 = (t) => { t = clampN(t, 0, 1); return t * t * (3 - 2 * t); };
const ease01 = (t) => 0.5 - 0.5 * Math.cos(Math.PI * clampN(t, 0, 1));

// Solve A x = b in place (n x n, row-major array of arrays). Gaussian elimination with partial pivoting.
function solveLin(A, b) {
  const n = b.length;
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (p !== c) { const t = A[p]; A[p] = A[c]; A[c] = t; const u = b[p]; b[p] = b[c]; b[c] = u; }
    const d = A[c][c] || 1e-12;
    for (let r = c + 1; r < n; r++) { const f = A[r][c] / d; if (!f) continue; for (let k = c; k < n; k++) A[r][k] -= f * A[c][k]; b[r] -= f * b[c]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = b[r]; for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k]; x[r] = s / (A[r][r] || 1e-12); }
  return x;
}

class KinRobot {
  constructor(desc) {
    this.desc = desc; this.root = desc.root || 'base_link';
    const AX = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1], '-x': [-1, 0, 0], '-y': [0, -1, 0], '-z': [0, 0, -1] };
    this.joints = desc.joints.map((j, i) => {
      const ax = Array.isArray(j.axis) ? j.axis : AX[j.axis || 'z'], L = Math.hypot(...ax) || 1;
      const xyz = j.xyz || [0, 0, 0], rpy = j.rpy || [0, 0, 0];
      return Object.assign({}, j, { i, type: j.type || 'revolute', ax: ax.map(v => v / L), O: K3.fromRPY(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]), offset: j.offset || 0, lo: j.lo != null ? j.lo : -Math.PI, hi: j.hi != null ? j.hi : Math.PI });
    });
    this.J = {}; this.byChild = {}; this.kids = {};
    this.joints.forEach(j => { this.J[j.name] = j; this.byChild[j.child] = j; (this.kids[j.parent] = this.kids[j.parent] || []).push(j); });
    this.order = []; const q = [this.root];
    while (q.length) { const l = q.shift(); (this.kids[l] || []).forEach(j => { this.order.push(j); q.push(j.child); }); }
    this.links = [this.root, ...this.order.map(j => j.child)];
    this.movable = this.order.filter(j => j.type !== 'fixed');
    this.names = this.movable.map(j => j.name);
    this.T = {}; this._R = new Float64Array(12); this._A = new Float64Array(12);
  }
  // world transforms of every link for joint values q (object name -> rad) and base transform B
  fk(q, B) {
    const T = this.T; T[this.root] = B || K3.I();
    for (const j of this.order) {
      const P = T[j.parent], M = K3.mul(P, j.O, this._A);
      if (j.type === 'fixed') { T[j.child] = new Float64Array(M); continue; }
      const v = (q[j.name] || 0);
      if (j.type === 'prismatic') { const o = new Float64Array(M); o[3] += M[0] * j.ax[0] * v + M[1] * j.ax[1] * v + M[2] * j.ax[2] * v; o[7] += M[4] * j.ax[0] * v + M[5] * j.ax[1] * v + M[6] * j.ax[2] * v; o[11] += M[8] * j.ax[0] * v + M[9] * j.ax[1] * v + M[10] * j.ax[2] * v; T[j.child] = o; continue; }
      K3.axisAngle(j.ax[0], j.ax[1], j.ax[2], v + j.offset, this._R);
      T[j.child] = K3.mul(M, this._R);
    }
    return T;
  }
  // joints from link `from` down to link `to`
  chain(from, to) {
    const out = []; let l = to;
    while (l !== from) { const j = this.byChild[l]; if (!j) throw new Error(`no chain ${from} -> ${to}`); out.unshift(j); l = j.parent; }
    return out;
  }
  // transform of `to` relative to `from` (both in the same tree) using current this.T
  rel(from, to) { return K3.mul(K3.inv(this.T[from]), this.T[to]); }
  clamp(q, j) { return clampN(q, j.lo, j.hi); }
  /* Damped least squares IK. chain: joints from rootT's link to the tip. rootT: world transform of the chain's
     parent link. target: world transform the tip (optionally offset by tool) should reach.
     opts: { tool: 3x4 tip->tool, wp: position weight, wr: rotation weight, iters, lambda, tolP, tolR } */
  ik(chain, q, rootT, target, opts = {}) {
    const n = chain.length, wp = opts.wp != null ? opts.wp : 1, wr = opts.wr != null ? opts.wr : 0.3, lam = opts.lambda || 0.02;
    const iters = opts.iters || 20, tolP = opts.tolP || 1e-4, tolR = opts.tolR || 1e-3, maxStep = opts.maxStep || 0.35;
    const Tj = new Array(n), A = new Float64Array(12), R = new Float64Array(12);
    let err = 1e9, ep = [0, 0, 0], er = [0, 0, 0];
    for (let it = 0; it <= iters; it++) {
      let T = rootT;
      for (let k = 0; k < n; k++) {
        const j = chain[k]; const M = K3.mul(T, j.O, A); Tj[k] = new Float64Array(M);
        if (j.type === 'fixed') { T = Tj[k]; continue; }
        K3.axisAngle(j.ax[0], j.ax[1], j.ax[2], (q[j.name] || 0) + j.offset, R); T = K3.mul(M, R);
      }
      const E = opts.tool ? K3.mul(T, opts.tool) : T;
      ep = [target[3] - E[3], target[7] - E[7], target[11] - E[11]];
      // orientation error: 0.5 * sum(col_e x col_t)
      er = [0, 0, 0];
      for (let c = 0; c < 3; c++) {
        const ex = E[c], ey = E[4 + c], ez = E[8 + c], tx = target[c], ty = target[4 + c], tz = target[8 + c];
        er[0] += 0.5 * (ey * tz - ez * ty); er[1] += 0.5 * (ez * tx - ex * tz); er[2] += 0.5 * (ex * ty - ey * tx);
      }
      err = Math.hypot(...ep) + (wr ? 0.2 * Math.hypot(...er) : 0);
      if (Math.hypot(...ep) < tolP && (!wr || Math.hypot(...er) < tolR)) break;
      if (it === iters) break;
      // Jacobian rows: 3 position + 3 rotation (weighted)
      const cols = [];
      for (let k = 0; k < n; k++) {
        const j = chain[k]; if (j.type === 'fixed') { cols.push(null); continue; }
        const M = Tj[k], z = K3.rot(M, j.ax[0], j.ax[1], j.ax[2]), px = E[3] - M[3], py = E[7] - M[7], pz = E[11] - M[11];
        cols.push([wp * (z[1] * pz - z[2] * py), wp * (z[2] * px - z[0] * pz), wp * (z[0] * py - z[1] * px), wr * z[0], wr * z[1], wr * z[2]]);
      }
      const e = [wp * ep[0], wp * ep[1], wp * ep[2], wr * er[0], wr * er[1], wr * er[2]];
      const JJ = [0, 1, 2, 3, 4, 5].map(r => [0, 1, 2, 3, 4, 5].map(c => { let s = r === c ? lam * lam : 0; cols.forEach(col => { if (col) s += col[r] * col[c]; }); return s; }));
      const y = solveLin(JJ, e.slice());
      let big = 0; const dq = cols.map(col => col ? col.reduce((s, v, r) => s + v * y[r], 0) : 0);
      dq.forEach(d => { big = Math.max(big, Math.abs(d)); });
      const sc = big > maxStep ? maxStep / big : 1;
      chain.forEach((j, k) => { if (j.type !== 'fixed') q[j.name] = clampN((q[j.name] || 0) + dq[k] * sc, j.lo, j.hi); });
    }
    return { err, ep: Math.hypot(...ep), er: Math.hypot(...er) };
  }
}
