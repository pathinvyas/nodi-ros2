/* ============================== SnoBots: rigid-body physics engine ==============================
   A small MuJoCo-style engine written for these simulators.
   - Robots are articulated bodies in generalized coordinates: a floating base (6 DOF) plus revolute and
     prismatic joints. Bias forces come from Featherstone's recursive Newton-Euler algorithm and the
     joint-space mass matrix from the composite-rigid-body algorithm. Spatial vectors are world-aligned
     and taken about the base origin, which keeps the numbers well conditioned anywhere in the world.
   - Joint servos (position, velocity or torque mode) are integrated implicitly: their stiffness and
     damping go into the system matrix, so stiff servos stay stable at a 2 ms step. Torque is limited by
     a torque-speed line; rotor armature, viscous damping and dry friction are modelled.
   - Contacts between boxes, spheres, capsules and wheels, the ground, static obstacles and loose objects
     are soft constraints with Coulomb friction (anisotropic for mecanum rollers), rolling friction and
     restitution, solved at the velocity level by projected Gauss-Seidel with warm starting. */
const PH_G = 9.81;

function phMat(q, p, o) {
  const x = q[0], y = q[1], z = q[2], w = q[3];
  o[0] = 1 - 2 * (y * y + z * z); o[1] = 2 * (x * y - z * w); o[2] = 2 * (x * z + y * w); o[3] = p[0];
  o[4] = 2 * (x * y + z * w); o[5] = 1 - 2 * (x * x + z * z); o[6] = 2 * (y * z - x * w); o[7] = p[1];
  o[8] = 2 * (x * z - y * w); o[9] = 2 * (y * z + x * w); o[10] = 1 - 2 * (x * x + y * y); o[11] = p[2];
  return o;
}
function phQuat(T) { const q = K3.quat(T), n = Math.hypot(q.x, q.y, q.z, q.w) || 1; return [q.x / n, q.y / n, q.z / n, q.w / n]; }
function phQuatStep(q, wx, wy, wz, h) {
  const a = Math.sqrt(wx * wx + wy * wy + wz * wz); if (a * h < 1e-12) return;
  const ha = a * h / 2, s = Math.sin(ha) / a, dx = wx * s, dy = wy * s, dz = wz * s, dw = Math.cos(ha);
  const x = q[0], y = q[1], z = q[2], w = q[3];
  const nx = dw * x + dx * w + dy * z - dz * y, ny = dw * y - dx * z + dy * w + dz * x, nz = dw * z + dx * y - dy * x + dz * w, nw = dw * w - dx * x - dy * y - dz * z;
  const n = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz + nw * nw); q[0] = nx * n; q[1] = ny * n; q[2] = nz * n; q[3] = nw * n;
}
// Cholesky L L^T in place (lower triangle), then solve in place
function phChol(A, n) {
  for (let j = 0; j < n; j++) {
    const jj = j * n; let s = A[jj + j];
    for (let k = 0; k < j; k++) s -= A[jj + k] * A[jj + k];
    const d = Math.sqrt(s > 1e-12 ? s : 1e-12); A[jj + j] = d; const id = 1 / d;
    for (let i = j + 1; i < n; i++) { const ii = i * n; let t = A[ii + j]; for (let k = 0; k < j; k++) t -= A[ii + k] * A[jj + k]; A[ii + j] = t * id; }
  }
}
function phSolve(L, n, x, o) {
  o = o || 0;
  for (let i = 0; i < n; i++) { const ii = i * n; let s = x[o + i]; for (let k = 0; k < i; k++) s -= L[ii + k] * x[o + k]; x[o + i] = s / L[ii + i]; }
  for (let i = n - 1; i >= 0; i--) { let s = x[o + i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[o + k]; x[o + i] = s / L[i * n + i]; }
}

/* ---------- one articulated system (a robot or a loose object) ---------- */
class PhysSys {
  constructor(name) {
    Object.assign(this, { name, nb: 0, par: [], jt: [], Xj: [], ax: [], jn: [], qoff: [], mass: [], com: [], Ib: [], geoms: [], linkBody: {}, linkOff: {}, ji: {}, pos: [0, 0, 0], quat: [0, 0, 0, 1], forces: [], payload: null, off: false });
  }
  addBody(parent, type, Xj, axis, name, qoff) {
    const b = this.nb++;
    this.par.push(parent); this.jt.push(type); this.Xj.push(Xj ? new Float64Array(Xj) : K3.I()); this.ax.push(axis ? axis.slice() : [0, 0, 1]); this.jn.push(name || null); this.qoff.push(qoff || 0);
    this.mass.push(0); this.com.push([0, 0, 0]); this.Ib.push([0, 0, 0, 0, 0, 0, 0, 0, 0]);
    if (name) this.ji[name] = b - 1;
    return b;
  }
  init() {
    const nb = this.nb, n = this.n = nb + 5, nj = this.nj = nb - 1;
    this.q = new Float64Array(nj); this.v = new Float64Array(n); this.vPrev = new Float64Array(n);
    this.T = Array.from({ length: nb }, () => K3.I());
    for (const k of ['Sv', 'Vb', 'Ab', 'Fb']) this[k] = new Float64Array(6 * nb);
    this.Im = new Float64Array(nb); this.Ih = new Float64Array(3 * nb); this.Io = new Float64Array(9 * nb);
    this.Cm = new Float64Array(nb); this.Ch = new Float64Array(3 * nb); this.Co = new Float64Array(9 * nb);
    this.M = new Float64Array(n * n); this.L = new Float64Array(n * n); this.rhs = new Float64Array(n); this.bias = new Float64Array(n); this.ext = new Float64Array(n);
    const z = () => new Float64Array(nj);
    Object.assign(this, { mode: new Int8Array(nj), kp: z(), kd: z(), qt: z(), vt: z(), tff: z(), tmax: z().fill(1e9), wmax: z().fill(1e9), arm: z(), damp: z(), fric: z(), lo: z().fill(-1e9), hi: z().fill(1e9), tau: z(), kpE: z(), kdE: z(), tffE: z(), str: z().fill(1) });
    this.mass0 = this.mass.slice(); this.com0 = this.com.map(c => c.slice()); this.Ib0 = this.Ib.map(i => i.slice());
    this.cf = new Float64Array(3 * nb); this.cfz = new Float64Array(nb);
    this._M = new Float64Array(12); this._R = new Float64Array(12); this._a = new Float64Array(6); this._b = new Float64Array(6);
    this.totalMass = this.mass.reduce((a, b) => a + b, 0);
    this.fk();
    return this;
  }
  /* ---------- state ---------- */
  setPose(p, quat) { this.asleep = false; this.sleepT = 0; this.pos[0] = p[0]; this.pos[1] = p[1]; this.pos[2] = p[2]; if (quat) { for (let i = 0; i < 4; i++) this.quat[i] = quat[i]; } this.fk(); }
  setYaw(x, y, z, yaw) { this.setPose([x, y, z], [0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2)]); }
  zeroVel() { this.v.fill(0); this.vPrev.fill(0); }
  qOf(name) { const k = this.ji[name]; return k == null ? 0 : this.q[k]; }
  setQ(name, v) { const k = this.ji[name]; if (k != null) this.q[k] = v; }
  /* ---------- kinematics: body transforms and joint motion subspaces (world aligned, about the base origin) ---------- */
  fk() {
    const T = this.T, o = this.pos, Sv = this.Sv, M = this._M, R = this._R;
    phMat(this.quat, this.pos, T[0]);
    for (let b = 1; b < this.nb; b++) {
      K3.mul(T[this.par[b]], this.Xj[b], M);
      const a = this.ax[b], s = 6 * b, k = b - 1;
      const awx = M[0] * a[0] + M[1] * a[1] + M[2] * a[2], awy = M[4] * a[0] + M[5] * a[1] + M[6] * a[2], awz = M[8] * a[0] + M[9] * a[1] + M[10] * a[2];
      if (this.jt[b] === 1) {
        K3.axisAngle(a[0], a[1], a[2], this.q[k] + this.qoff[b], R); K3.mul(M, R, T[b]);
        const px = M[3] - o[0], py = M[7] - o[1], pz = M[11] - o[2];
        Sv[s] = awx; Sv[s + 1] = awy; Sv[s + 2] = awz; Sv[s + 3] = py * awz - pz * awy; Sv[s + 4] = pz * awx - px * awz; Sv[s + 5] = px * awy - py * awx;
      } else {
        const d = this.q[k], Tb = T[b]; Tb.set(M); Tb[3] += awx * d; Tb[7] += awy * d; Tb[11] += awz * d;
        Sv[s] = 0; Sv[s + 1] = 0; Sv[s + 2] = 0; Sv[s + 3] = awx; Sv[s + 4] = awy; Sv[s + 5] = awz;
      }
    }
  }
  linkT(name, out) { const b = this.linkBody[name]; if (b == null) return null; return K3.mul(this.T[b], this.linkOff[name], out); }
  // spatial inertia of every body about the base origin
  inert() {
    const o = this.pos;
    for (let b = 0; b < this.nb; b++) {
      const T = this.T[b], m = this.mass[b], c = this.com[b], I = this.Ib[b];
      const cx = T[0] * c[0] + T[1] * c[1] + T[2] * c[2] + T[3] - o[0], cy = T[4] * c[0] + T[5] * c[1] + T[6] * c[2] + T[7] - o[1], cz = T[8] * c[0] + T[9] * c[1] + T[10] * c[2] + T[11] - o[2];
      this.Im[b] = m; const h = 3 * b; this.Ih[h] = m * cx; this.Ih[h + 1] = m * cy; this.Ih[h + 2] = m * cz;
      // Iw = R I R^T
      const O = this.Io, i9 = 9 * b, cc = cx * cx + cy * cy + cz * cz;
      for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) {
        let s = 0; for (let a = 0; a < 3; a++) { const Ra = T[4 * r + a]; if (!Ra) continue; for (let e = 0; e < 3; e++) s += Ra * I[3 * a + e] * T[4 * q + e]; }
        const cr = r === 0 ? cx : r === 1 ? cy : cz, cq = q === 0 ? cx : q === 1 ? cy : cz;
        O[i9 + 3 * r + q] = s + m * ((r === q ? cc : 0) - cr * cq);
      }
    }
  }
  // composite-rigid-body algorithm: joint-space mass matrix
  crba() {
    const nb = this.nb, n = this.n, M = this.M, Sv = this.Sv, Cm = this.Cm, Ch = this.Ch, Co = this.Co, F = this._a;
    Cm.set(this.Im); Ch.set(this.Ih); Co.set(this.Io);
    for (let b = nb - 1; b > 0; b--) { const p = this.par[b]; Cm[p] += Cm[b]; for (let i = 0; i < 3; i++) Ch[3 * p + i] += Ch[3 * b + i]; for (let i = 0; i < 9; i++) Co[9 * p + i] += Co[9 * b + i]; }
    M.fill(0);
    const m0 = Cm[0], hx = Ch[0], hy = Ch[1], hz = Ch[2];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) M[r * n + c] = Co[3 * r + c];
    // [h]x top-right, its transpose bottom-left, m I bottom-right
    M[0 * n + 4] = -hz; M[0 * n + 5] = hy; M[1 * n + 3] = hz; M[1 * n + 5] = -hx; M[2 * n + 3] = -hy; M[2 * n + 4] = hx;
    M[4 * n + 0] = -hz; M[5 * n + 0] = hy; M[3 * n + 1] = hz; M[5 * n + 1] = -hx; M[3 * n + 2] = -hy; M[4 * n + 2] = hx;
    M[3 * n + 3] = m0; M[4 * n + 4] = m0; M[5 * n + 5] = m0;
    for (let b = 1; b < nb; b++) {
      const s = 6 * b, h = 3 * b, i9 = 9 * b, m = Cm[b];
      const wx = Sv[s], wy = Sv[s + 1], wz = Sv[s + 2], vx = Sv[s + 3], vy = Sv[s + 4], vz = Sv[s + 5], Hx = Ch[h], Hy = Ch[h + 1], Hz = Ch[h + 2];
      F[0] = Co[i9] * wx + Co[i9 + 1] * wy + Co[i9 + 2] * wz + (Hy * vz - Hz * vy);
      F[1] = Co[i9 + 3] * wx + Co[i9 + 4] * wy + Co[i9 + 5] * wz + (Hz * vx - Hx * vz);
      F[2] = Co[i9 + 6] * wx + Co[i9 + 7] * wy + Co[i9 + 8] * wz + (Hx * vy - Hy * vx);
      F[3] = m * vx + (wy * Hz - wz * Hy); F[4] = m * vy + (wz * Hx - wx * Hz); F[5] = m * vz + (wx * Hy - wy * Hx);
      const k = 5 + b;
      M[k * n + k] = wx * F[0] + wy * F[1] + wz * F[2] + vx * F[3] + vy * F[4] + vz * F[5];
      for (let r = 0; r < 6; r++) { M[k * n + r] = F[r]; M[r * n + k] = F[r]; }
      for (let j = this.par[b]; j > 0; j = this.par[j]) { const t = 6 * j, val = Sv[t] * F[0] + Sv[t + 1] * F[1] + Sv[t + 2] * F[2] + Sv[t + 3] * F[3] + Sv[t + 4] * F[4] + Sv[t + 5] * F[5]; M[k * n + 5 + j] = val; M[(5 + j) * n + k] = val; }
    }
  }
  // twists of every body from the generalized velocity
  twists(v) {
    const V = this.Vb, Sv = this.Sv; for (let i = 0; i < 6; i++) V[i] = v[i];
    for (let b = 1; b < this.nb; b++) { const s = 6 * b, p = 6 * this.par[b], qd = v[5 + b]; for (let i = 0; i < 6; i++) V[s + i] = V[p + i] + Sv[s + i] * qd; }
  }
  // recursive Newton-Euler: Coriolis, centrifugal and gravity forces
  rnea() {
    const nb = this.nb, V = this.Vb, A = this.Ab, F = this.Fb, Sv = this.Sv, v = this.v, m = this.Im, H = this.Ih, O = this.Io;
    this.twists(v);
    // base velocity is (omega, velocity of the base origin): its spatial acceleration carries -omega x v
    A[0] = 0; A[1] = 0; A[2] = 0; A[3] = -(v[1] * v[5] - v[2] * v[4]); A[4] = -(v[2] * v[3] - v[0] * v[5]); A[5] = PH_G - (v[0] * v[4] - v[1] * v[3]);
    for (let b = 1; b < nb; b++) {
      const s = 6 * b, p = 6 * this.par[b], qd = v[5 + b];
      const sw0 = Sv[s] * qd, sw1 = Sv[s + 1] * qd, sw2 = Sv[s + 2] * qd, sv0 = Sv[s + 3] * qd, sv1 = Sv[s + 4] * qd, sv2 = Sv[s + 5] * qd;
      const w0 = V[s], w1 = V[s + 1], w2 = V[s + 2], u0 = V[s + 3], u1 = V[s + 4], u2 = V[s + 5];
      A[s] = A[p] + (w1 * sw2 - w2 * sw1); A[s + 1] = A[p + 1] + (w2 * sw0 - w0 * sw2); A[s + 2] = A[p + 2] + (w0 * sw1 - w1 * sw0);
      A[s + 3] = A[p + 3] + (w1 * sv2 - w2 * sv1) + (u1 * sw2 - u2 * sw1); A[s + 4] = A[p + 4] + (w2 * sv0 - w0 * sv2) + (u2 * sw0 - u0 * sw2); A[s + 5] = A[p + 5] + (w0 * sv1 - w1 * sv0) + (u0 * sw1 - u1 * sw0);
    }
    for (let b = 0; b < nb; b++) {
      const s = 6 * b, h = 3 * b, i9 = 9 * b, M = m[b], Hx = H[h], Hy = H[h + 1], Hz = H[h + 2];
      const w0 = V[s], w1 = V[s + 1], w2 = V[s + 2], u0 = V[s + 3], u1 = V[s + 4], u2 = V[s + 5];
      const a0 = A[s], a1 = A[s + 1], a2 = A[s + 2], a3 = A[s + 3], a4 = A[s + 4], a5 = A[s + 5];
      const v0 = O[i9] * w0 + O[i9 + 1] * w1 + O[i9 + 2] * w2 + (Hy * u2 - Hz * u1), v1 = O[i9 + 3] * w0 + O[i9 + 4] * w1 + O[i9 + 5] * w2 + (Hz * u0 - Hx * u2), v2 = O[i9 + 6] * w0 + O[i9 + 7] * w1 + O[i9 + 8] * w2 + (Hx * u1 - Hy * u0);
      const v3 = M * u0 + (w1 * Hz - w2 * Hy), v4 = M * u1 + (w2 * Hx - w0 * Hz), v5 = M * u2 + (w0 * Hy - w1 * Hx);
      const i0 = O[i9] * a0 + O[i9 + 1] * a1 + O[i9 + 2] * a2 + (Hy * a5 - Hz * a4), i1 = O[i9 + 3] * a0 + O[i9 + 4] * a1 + O[i9 + 5] * a2 + (Hz * a3 - Hx * a5), i2 = O[i9 + 6] * a0 + O[i9 + 7] * a1 + O[i9 + 8] * a2 + (Hx * a4 - Hy * a3);
      const i3 = M * a3 + (a1 * Hz - a2 * Hy), i4 = M * a4 + (a2 * Hx - a0 * Hz), i5 = M * a5 + (a0 * Hy - a1 * Hx);
      F[s] = i0 + (w1 * v2 - w2 * v1) + (u1 * v5 - u2 * v4); F[s + 1] = i1 + (w2 * v0 - w0 * v2) + (u2 * v3 - u0 * v5); F[s + 2] = i2 + (w0 * v1 - w1 * v0) + (u0 * v4 - u1 * v3);
      F[s + 3] = i3 + (w1 * v5 - w2 * v4); F[s + 4] = i4 + (w2 * v3 - w0 * v5); F[s + 5] = i5 + (w0 * v4 - w1 * v3);
    }
    const bias = this.bias;
    for (let b = nb - 1; b > 0; b--) { const s = 6 * b, p = 6 * this.par[b]; bias[5 + b] = Sv[s] * F[s] + Sv[s + 1] * F[s + 1] + Sv[s + 2] * F[s + 2] + Sv[s + 3] * F[s + 3] + Sv[s + 4] * F[s + 4] + Sv[s + 5] * F[s + 5]; for (let i = 0; i < 6; i++) F[p + i] += F[s + i]; }
    for (let i = 0; i < 6; i++) bias[i] = F[i];
  }
  /* ---------- external forces (applied during the next step) ---------- */
  push(b, pLocal, F) { this.forces.push([b, pLocal, F]); }          // force F (world, N) at a body-local point
  pushTorque(b, tau) { this.forces.push([b, null, tau]); }          // pure torque (world, N·m) on body b
  pushWorld(b, pWorld, F) { const Ti = K3.inv(this.T[b]); this.forces.push([b, K3.apply(Ti, pWorld[0], pWorld[1], pWorld[2]), F]); }
  addForce(b, p, F) {
    const o = this.pos, ext = this.ext, Sv = this.Sv, rx = p[0] - o[0], ry = p[1] - o[1], rz = p[2] - o[2];
    const mx = ry * F[2] - rz * F[1], my = rz * F[0] - rx * F[2], mz = rx * F[1] - ry * F[0];
    ext[0] += mx; ext[1] += my; ext[2] += mz; ext[3] += F[0]; ext[4] += F[1]; ext[5] += F[2];
    for (let j = b; j > 0; j = this.par[j]) { const s = 6 * j; ext[5 + j] += Sv[s] * mx + Sv[s + 1] * my + Sv[s + 2] * mz + Sv[s + 3] * F[0] + Sv[s + 4] * F[1] + Sv[s + 5] * F[2]; }
  }
  /* ---------- dynamics: unconstrained velocity with implicit actuators ---------- */
  dyn(h) {
    this.inert(); this.crba(); this.rnea();
    for (const [b, pl, F] of this.forces) {
      if (pl) { const p = K3.apply(this.T[b], pl[0], pl[1], pl[2]); this.addForce(b, p, F); continue; }
      const ext = this.ext, Sv = this.Sv; ext[0] += F[0]; ext[1] += F[1]; ext[2] += F[2];
      for (let j = b; j > 0; j = this.par[j]) { const s = 6 * j; ext[5 + j] += Sv[s] * F[0] + Sv[s + 1] * F[1] + Sv[s + 2] * F[2]; }
    }
    this.forces.length = 0;
    const n = this.n, M = this.M, L = this.L, r = this.rhs, v = this.v, nj = this.nj;
    this.vPrev.set(v);
    for (let i = 0; i < n; i++) { let s = 0; const ii = i * n; for (let j = 0; j < n; j++) s += M[ii + j] * v[j]; r[i] = s + h * (this.ext[i] - this.bias[i]); }
    L.set(M);
    for (let k = 0; k < nj; k++) {
      const d = k + 6, dd = d * n + d, vk = v[d], qk = this.q[k], md = this.mode[k];
      let kp = 0, kd = 0, tf = 0;
      if (md === 1) { kp = this.kp[k]; kd = this.kd[k]; tf = this.tff[k]; } else if (md === 2) { kd = this.kd[k]; tf = this.tff[k]; } else if (md === 3) tf = this.tff[k];
      const est = kp * (this.qt[k] - qk) + kd * (this.vt[k] - vk) + tf;
      let lim = this.tmax[k] * this.str[k];
      if (est * vk > 0) lim *= Math.max(0.05, 1 - Math.abs(vk) / this.wmax[k]);   // torque-speed line while motoring
      if (Math.abs(est) > lim) { const sc = lim / Math.abs(est); kp *= sc; kd *= sc; tf *= sc; }
      this.kpE[k] = kp; this.kdE[k] = kd; this.tffE[k] = tf;
      const arm = this.arm[k], fr = this.fric[k]; let c = this.damp[k], fc = 0;
      if (fr > 0) { if (Math.abs(vk) < 0.04) c += fr / 0.04; else fc = -fr * Math.sign(vk); }
      L[dd] += arm + h * (kd + c) + h * h * kp;
      r[d] += arm * vk + h * (kp * (this.qt[k] - qk) + kd * this.vt[k] + tf + fc);
    }
    if (this.fixed) for (let i = 0; i < 6; i++) { L[i * n + i] += 1e12; r[i] = 0; }
    phChol(L, n);
    v.set(r); phSolve(L, n, v);
    this.ext.fill(0);
  }
  integrate(h) {
    const v = this.v;
    for (let k = 0; k < this.nj; k++) { const vk = v[k + 6]; this.tau[k] = this.kpE[k] * (this.qt[k] - this.q[k] - h * vk) + this.kdE[k] * (this.vt[k] - vk) + this.tffE[k]; this.q[k] += h * vk; }
    const wx = v[0], wy = v[1], wz = v[2], vx = v[3], vy = v[4], vz = v[5], p = this.pos;
    p[0] += h * vx; p[1] += h * vy; p[2] += h * vz;
    phQuatStep(this.quat, wx, wy, wz, h);
    this.fk();
  }
  /* ---------- Jacobians for constraints ---------- */
  chainIdx(b) {
    const C = this._ci || (this._ci = []); if (C[b]) return C[b];
    const out = [0, 1, 2, 3, 4, 5]; for (let j = b; j > 0; j = this.par[j]) out.push(5 + j);
    return (C[b] = Int32Array.from(out));
  }
  jacPt(b, p, ux, uy, uz, J, o, sg) {
    const P = this.pos, rx = p[0] - P[0], ry = p[1] - P[1], rz = p[2] - P[2], Sv = this.Sv, n = this.n;
    const cx = (ry * uz - rz * uy) * sg, cy = (rz * ux - rx * uz) * sg, cz = (rx * uy - ry * ux) * sg; ux *= sg; uy *= sg; uz *= sg;
    for (let i = 6; i < n; i++) J[o + i] = 0;
    J[o] = cx; J[o + 1] = cy; J[o + 2] = cz; J[o + 3] = ux; J[o + 4] = uy; J[o + 5] = uz;
    for (let j = b; j > 0; j = this.par[j]) { const s = 6 * j; J[o + 5 + j] = Sv[s] * cx + Sv[s + 1] * cy + Sv[s + 2] * cz + Sv[s + 3] * ux + Sv[s + 4] * uy + Sv[s + 5] * uz; }
  }
  jacRot(b, ux, uy, uz, J, o, sg) {
    const Sv = this.Sv, n = this.n; ux *= sg; uy *= sg; uz *= sg;
    for (let i = 6; i < n; i++) J[o + i] = 0;
    J[o] = ux; J[o + 1] = uy; J[o + 2] = uz; J[o + 3] = 0; J[o + 4] = 0; J[o + 5] = 0;
    for (let j = b; j > 0; j = this.par[j]) { const s = 6 * j; J[o + 5 + j] = Sv[s] * ux + Sv[s + 1] * uy + Sv[s + 2] * uz; }
  }
  /* ---------- sensors and helpers ---------- */
  // whole-body centre of mass and its velocity: [x, y, z, vx, vy, vz]
  comState(out) {
    out = out || new Float64Array(6); this.twists(this.v);
    const o = this.pos, V = this.Vb; let M = 0, x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;
    for (let b = 0; b < this.nb; b++) {
      const T = this.T[b], m = this.mass[b], c = this.com[b], s = 6 * b; if (!m) continue;
      const cx = T[0] * c[0] + T[1] * c[1] + T[2] * c[2] + T[3], cy = T[4] * c[0] + T[5] * c[1] + T[6] * c[2] + T[7], cz = T[8] * c[0] + T[9] * c[1] + T[10] * c[2] + T[11];
      const rx = cx - o[0], ry = cy - o[1], rz = cz - o[2];
      M += m; x += m * cx; y += m * cy; z += m * cz;
      vx += m * (V[s + 3] + V[s + 1] * rz - V[s + 2] * ry); vy += m * (V[s + 4] + V[s + 2] * rx - V[s] * rz); vz += m * (V[s + 5] + V[s] * ry - V[s + 1] * rx);
    }
    out[0] = x / M; out[1] = y / M; out[2] = z / M; out[3] = vx / M; out[4] = vy / M; out[5] = vz / M;
    return out;
  }
  // velocity of a world point fixed to body b
  ptVel(b, p, out) {
    this.twists(this.v); const o = this.pos, V = this.Vb, s = 6 * b, rx = p[0] - o[0], ry = p[1] - o[1], rz = p[2] - o[2];
    out = out || [0, 0, 0]; out[0] = V[s + 3] + V[s + 1] * rz - V[s + 2] * ry; out[1] = V[s + 4] + V[s + 2] * rx - V[s] * rz; out[2] = V[s + 5] + V[s] * ry - V[s + 1] * rx; return out;
  }
  bodyOmega(b) { this.twists(this.v); const s = 6 * b; return [this.Vb[s], this.Vb[s + 1], this.Vb[s + 2]]; }
  // extra mass attached to a body (held object): m at body-local com with inertia Ib (body frame)
  setPayload(b, m, c, Ib) {
    for (let i = 0; i < this.nb; i++) { this.mass[i] = this.mass0[i] * (this.massScale || 1); this.com[i] = this.com0[i].slice(); this.Ib[i] = this.Ib0[i].map(x => x * (this.massScale || 1)); }
    this.payload = m ? { b, m, c, Ib } : null;
    if (!m) return;
    const m0 = this.mass[b], c0 = this.com[b], M = m0 + m, cn = [0, 1, 2].map(i => (m0 * c0[i] + m * c[i]) / M), I = this.Ib[b].slice();
    const add = (mm, cc, II) => { const d = [cc[0] - cn[0], cc[1] - cn[1], cc[2] - cn[2]], dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]; for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) I[3 * r + q] += (II ? II[3 * r + q] : 0) + mm * ((r === q ? dd : 0) - d[r] * d[q]); };
    for (let i = 0; i < 9; i++) I[i] = 0; add(m0, c0, this.Ib[b]); add(m, c, Ib);
    this.mass[b] = M; this.com[b] = cn; this.Ib[b] = I;
  }
  scaleMass(s) { this.massScale = s; const p = this.payload; this.setPayload(p ? p.b : 0, p ? p.m : 0, p ? p.c : null, p ? p.Ib : null); this.totalMass = this.mass.reduce((a, b) => a + b, 0); }
}

/* ---------- geometry helpers ---------- */
const PH_BOX = 0, PH_SPH = 1, PH_CAP = 2, PH_WHEEL = 3, PH_VCYL = 4;
function phGeom(t, size, T, o = {}) {
  const g = Object.assign({ t, size: new Float64Array([size[0] || 0, size[1] || 0, size[2] || 0]), T: T ? new Float64Array(T) : K3.I(), W: K3.I(), mu: 0.9, rest: 0, roll: 0, aniso: null, conv: 0, id: 0 }, o);
  const s = g.size; g.rad = t === PH_BOX ? Math.hypot(s[0], s[1], s[2]) : t === PH_SPH ? s[0] : Math.hypot(s[0], s[1]) + (t === PH_CAP ? s[1] : 0);
  return g;
}
// inertia of one visual primitive (unit density), in the link frame
function phVisInertia(v) {
  const s = v.size; let V, D;
  if (v.s === 'box') { V = s[0] * s[1] * s[2]; D = [V * (s[1] * s[1] + s[2] * s[2]) / 12, V * (s[0] * s[0] + s[2] * s[2]) / 12, V * (s[0] * s[0] + s[1] * s[1]) / 12]; }
  else if (v.s === 'sph') { V = 4 / 3 * Math.PI * s[0] ** 3; D = [0.4 * V * s[0] * s[0], 0.4 * V * s[0] * s[0], 0.4 * V * s[0] * s[0]]; }
  else {
    const r = v.s === 'cone' ? (s[0] + s[1]) / 2 : s[0], L = v.s === 'cap' ? s[1] + 4 * s[0] / 3 : v.s === 'cone' ? s[2] : s[1];
    V = Math.PI * r * r * L; const Ia = 0.5 * V * r * r, Ip = V * (3 * r * r + L * L) / 12, ax = v.ax || 'z';
    D = ax === 'x' ? [Ia, Ip, Ip] : ax === 'y' ? [Ip, Ia, Ip] : [Ip, Ip, Ia];
  }
  const e = v.rpy || [0, 0, 0], R = K3.fromRPY(0, 0, 0, e[0], e[1], e[2]), I = new Array(9).fill(0);
  for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) for (let a = 0; a < 3; a++) I[3 * r + q] += R[4 * r + a] * D[a] * R[4 * q + a];
  return { V, c: (v.xyz || [0, 0, 0]).slice(), I };
}
function phVisCorners(v) {
  const s = v.size; let hx, hy, hz;
  if (v.s === 'box') [hx, hy, hz] = [s[0] / 2, s[1] / 2, s[2] / 2]; else if (v.s === 'sph') hx = hy = hz = s[0];
  else { const r = v.s === 'cone' ? Math.max(s[0], s[1]) : s[0], L = (v.s === 'cap' ? s[1] + 2 * s[0] : v.s === 'cone' ? s[2] : s[1]) / 2, ax = v.ax || 'z'; hx = ax === 'x' ? L : r; hy = ax === 'y' ? L : r; hz = ax === 'z' ? L : r; }
  const c = v.xyz || [0, 0, 0], e = v.rpy || [0, 0, 0], R = K3.fromRPY(0, 0, 0, e[0], e[1], e[2]), out = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) { const p = K3.rot(R, sx * hx, sy * hy, sz * hz); out.push([c[0] + p[0], c[1] + p[1], c[2] + p[2]]); }
  return out;
}
/* Build a physics system from a KinRobot. o: { name, mass (total kg), masses: {link: [m, com]}, density: {mat: rel},
   coll: {link: [{t, size, xyz, rpy, ...}] | false}, noColl: RegExp, act(jointName, joint) -> actuator params, mu } */
function physFromKin(kin, o = {}) {
  const S = new PhysSys(o.name || 'robot'), desc = kin.desc, bodyOf = {}, off = {};
  bodyOf[kin.root] = S.addBody(-1, 0); off[kin.root] = K3.I();
  for (const j of kin.order) {
    const pb = bodyOf[j.parent], Xp = K3.mul(off[j.parent], j.O);
    if (j.type === 'fixed') { bodyOf[j.child] = pb; off[j.child] = Xp; continue; }
    bodyOf[j.child] = S.addBody(pb, j.type === 'prismatic' ? 2 : 1, Xp, j.ax, j.name, j.offset || 0); off[j.child] = K3.I();
  }
  S.linkBody = bodyOf; S.linkOff = off;
  // mass properties per link from the visuals (relative densities), scaled to the total mass
  const dens = Object.assign({ servo: 3, alu: 1.4, shell: 0.75, white: 0.75, dark: 1.2, grey: 1.2, rubber: 1.3, metal: 2.5, accent: 0.8, glass: 1, glow: 0.4, navy: 1.2, copper: 2.5, wood: 0.7 }, o.density || {});
  const lm = {};
  for (const ln of kin.links) {
    let m = 0, cx = 0, cy = 0, cz = 0; const parts = [];
    (desc.visuals[ln] || []).forEach(v => { if (v.massless) return; const p = phVisInertia(v), rho = dens[typeof v.m === 'string' ? v.m : 'dark'] || 1, mm = p.V * rho; parts.push([mm, p]); m += mm; cx += mm * p.c[0]; cy += mm * p.c[1]; cz += mm * p.c[2]; });
    if (m <= 0) { lm[ln] = { m: 0, c: [0, 0, 0], I: new Array(9).fill(0) }; continue; }
    const c = [cx / m, cy / m, cz / m], I = new Array(9).fill(0);
    parts.forEach(([mm, p]) => { const d = [p.c[0] - c[0], p.c[1] - c[1], p.c[2] - c[2]], dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2], k = mm / p.V; for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) I[3 * r + q] += p.I[3 * r + q] * k + mm * ((r === q ? dd : 0) - d[r] * d[q]); });
    lm[ln] = { m, c, I };
  }
  if (o.masses) for (const [ln, [m, c]] of Object.entries(o.masses)) { const L = lm[ln]; if (!L) continue; const k = L.m > 0 ? m / L.m : 0; lm[ln] = { m, c: c.slice(), I: L.I.map(x => x * k) }; }
  const tot = Object.values(lm).reduce((a, b) => a + b.m, 0), sc = o.mass && !o.masses ? o.mass / tot : (o.mass && o.masses ? o.mass / tot : 1);
  for (const L of Object.values(lm)) { L.m *= sc; L.I = L.I.map(x => x * sc); }
  // aggregate links into bodies
  const acc = Array.from({ length: S.nb }, () => ({ m: 0, f: [0, 0, 0], I: new Array(9).fill(0) }));
  for (const ln of kin.links) {
    const L = lm[ln]; if (!L.m) continue; const X = off[ln], a = acc[bodyOf[ln]], c = K3.apply(X, L.c[0], L.c[1], L.c[2]), cc = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    a.m += L.m; for (let i = 0; i < 3; i++) a.f[i] += L.m * c[i];
    for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) { let s = 0; for (let x = 0; x < 3; x++) for (let y = 0; y < 3; y++) s += X[4 * r + x] * L.I[3 * x + y] * X[4 * q + y]; a.I[3 * r + q] += s + L.m * ((r === q ? cc : 0) - c[r] * c[q]); }
  }
  acc.forEach((a, b) => {
    const m = Math.max(a.m, 1e-4), c = a.m > 0 ? a.f.map(x => x / m) : [0, 0, 0], cc = c[0] * c[0] + c[1] * c[1] + c[2] * c[2], I = a.I.slice();
    for (let r = 0; r < 3; r++) for (let q = 0; q < 3; q++) I[3 * r + q] -= m * ((r === q ? cc : 0) - c[r] * c[q]);
    const eps = Math.max(1e-7, m * 1e-5); I[0] += eps; I[4] += eps; I[8] += eps;
    S.mass[b] = m; S.com[b] = c; S.Ib[b] = I;
  });
  // collision geometry: explicit shapes, or one box around each link's visuals
  for (const ln of kin.links) {
    const b = bodyOf[ln], X = off[ln], spec = o.coll && Object.prototype.hasOwnProperty.call(o.coll, ln) ? o.coll[ln] : undefined;
    if (spec === false || (spec === undefined && o.noColl && o.noColl.test(ln))) continue;
    const list = spec || (() => { const vs = (desc.visuals[ln] || []).filter(v => !v.noColl); if (!vs.length) return []; const P = vs.flatMap(phVisCorners), lo = [0, 1, 2].map(i => Math.min(...P.map(p => p[i]))), hi = [0, 1, 2].map(i => Math.max(...P.map(p => p[i]))); return [{ t: 'box', size: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]], xyz: [0, 1, 2].map(i => (lo[i] + hi[i]) / 2) }]; })();
    list.forEach(c => {
      const xyz = c.xyz || [0, 0, 0], e = c.rpy || [0, 0, 0], T = K3.mul(X, K3.fromRPY(xyz[0], xyz[1], xyz[2], e[0], e[1], e[2]));
      const t = { box: PH_BOX, sph: PH_SPH, cap: PH_CAP, wheel: PH_WHEEL }[c.t], size = c.t === 'box' ? c.size.map(v => v / 2) : c.size;
      const g = phGeom(t, size, T, { mu: c.mu != null ? c.mu : (o.mu || 0.9), rest: c.rest || 0, roll: c.roll || 0, link: ln, tag: c.tag || ln });
      if (c.aniso) { g.aniso = K3.rot(X, c.aniso[0], c.aniso[1], c.aniso[2]); g.mu2 = c.mu2 != null ? c.mu2 : 0.02; }
      g.s = S; g.b = b; S.geoms.push(g);
    });
  }
  S.init();
  // actuators
  kin.movable.forEach(j => {
    const k = S.ji[j.name]; if (k == null) return; const a = o.act ? o.act(j.name, j) || {} : {};
    S.mode[k] = a.mode != null ? a.mode : 1; S.kp[k] = a.kp || 0; S.kd[k] = a.kd || 0; S.tmax[k] = a.tmax || 1e9; S.wmax[k] = a.wmax || 1e9;
    S.arm[k] = a.arm || 0; S.damp[k] = a.damp || 0; S.fric[k] = a.fric || 0;
    if (j.type !== 'continuous' && j.lo > -1e8) { S.lo[k] = j.lo; S.hi[k] = j.hi; }
  });
  return S;
}
function physBox(name, size, mass, o = {}) {
  const S = new PhysSys(name); S.addBody(-1, 0); const [a, b, c] = size;
  S.mass[0] = mass; S.Ib[0] = [mass * (b * b + c * c) / 12, 0, 0, 0, mass * (a * a + c * c) / 12, 0, 0, 0, mass * (a * a + b * b) / 12];
  const g = phGeom(PH_BOX, [a / 2, b / 2, c / 2], null, Object.assign({ mu: 0.7 }, o)); g.s = S; g.b = 0; S.geoms.push(g); S.free = true; return S.init();
}
function physSphere(name, r, mass, o = {}) {
  const S = new PhysSys(name); S.addBody(-1, 0); const I = (o.hollow ? 2 / 3 : 0.4) * mass * r * r;
  S.mass[0] = mass; S.Ib[0] = [I, 0, 0, 0, I, 0, 0, 0, I];
  const g = phGeom(PH_SPH, [r], null, Object.assign({ mu: 0.6 }, o)); g.s = S; g.b = 0; S.geoms.push(g); S.free = true; return S.init();
}

/* ---------- the world: systems, static obstacles, contacts and the constraint solver ---------- */
class PhysWorld {
  constructor(o = {}) {
    Object.assign(this, { h: o.h || 0.002, t: 0, sys: [], statics: [], iters: o.iters || 14, erp: 0.2, slop: 0.0005, margin: 0.004, cfm: 0.002, gid: 0, muScale: 1, steps: 0 });
    this.ground = o.ground === false ? null : { id: 1, mu: o.mu != null ? o.mu : 0.9, ground: true };
    this.gid = 10; this.cache = new Map(); this.C = []; this.nc = 0; this.rows = 0; this._s = new Float64Array(8); this._p = new Float64Array(3);
  }
  add(s) { s.world = this; s.uid = ++this.gid; this.sys.push(s); s.geoms.forEach(g => { g.id = ++this.gid; }); s.pool = null; return s; }
  remove(s) { const i = this.sys.indexOf(s); if (i >= 0) this.sys.splice(i, 1); }
  // static box: centre, full size, yaw (extends a little below the floor so it never pulls things down)
  // a static box reaches down into the floor (no gap for things to slip under) unless o.float: true (a roof, a shelf)
  addBox(c, size, yaw = 0, o = {}) {
    const bot = c[2] - size[2] / 2, z0 = o.float ? bot : Math.min(bot, -0.05), top = c[2] + size[2] / 2, cz = (z0 + top) / 2, hz = (top - z0) / 2;
    const g = phGeom(PH_BOX, [size[0] / 2, size[1] / 2, hz], K3.fromRPY(c[0], c[1], cz, 0, 0, yaw), Object.assign({ mu: 0.6 }, o));
    g.W = g.T; g.id = ++this.gid; g.static = true; this.statics.push(g); return g;
  }
  addCyl(x, y, r, top, o = {}) { const g = phGeom(PH_VCYL, [r, top], K3.fromRPY(x, y, 0, 0, 0, 0), Object.assign({ mu: 0.6 }, o)); g.W = g.T; g.id = ++this.gid; g.static = true; this.statics.push(g); return g; }
  step() {
    const h = this.h;
    this.wake();
    for (const s of this.sys) if (!s.off && !s.asleep) s.dyn(h);
    this.collide(); this.solve(h);
    for (const s of this.sys) if (!s.off && !s.asleep) { s.integrate(h); if (s.free) this.sleepCheck(s, h); }
    this.t += h; this.steps++;
  }
  // loose objects at rest go to sleep (no dynamics, no contacts) until something comes close
  sleepCheck(s, h) {
    const v = s.v, slow = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] < 0.01 && v[3] * v[3] + v[4] * v[4] + v[5] * v[5] < 1e-4 && s.forces.length === 0;
    s.sleepT = slow ? (s.sleepT || 0) + h : 0;
    if (s.sleepT > 0.4) { s.asleep = true; v.fill(0); }
  }
  wake() {
    for (const a of this.sys) {
      if (!a.asleep) continue;
      if (a.forces.length) { a.asleep = false; a.sleepT = 0; continue; }
      outer: for (const b of this.sys) {
        if (b === a || b.asleep || b.off) continue;
        for (const ga of a.geoms) for (const gb of b.geoms) {
          const A = ga.W, B = gb.W, dx = A[3] - B[3], dy = A[7] - B[7], dz = A[11] - B[11], r = ga.rad + gb.rad + 0.03;
          if (dx * dx + dy * dy + dz * dz < r * r) { a.asleep = false; a.sleepT = 0; break outer; }
        }
      }
    }
  }
  /* ---------- collision detection ---------- */
  contact(ga, gb, px, py, pz, nx, ny, nz, d, f) {
    let c = this.C[this.nc]; if (!c) c = this.C[this.nc] = { p: new Float64Array(3), n: new Float64Array(3), t1: new Float64Array(3), t2: new Float64Array(3), r: [] };
    this.nc++;
    c.a = ga; c.b = gb; c.p[0] = px; c.p[1] = py; c.p[2] = pz; c.n[0] = nx; c.n[1] = ny; c.n[2] = nz; c.d = d;
    c.key = (ga.id * 2048 + gb.id) * 64 + f;
    const ma = ga.mu, mb = gb.mu; c.mu = Math.max(ma, mb) * this.muScale;
    c.rest = Math.max(ga.rest || 0, gb.rest || 0); c.roll = Math.max(ga.roll || 0, gb.roll || 0);
    c.aniso = ga.aniso ? ga : gb.aniso ? gb : null; c.conv = ga.conv || 0;
    return c;
  }
  upd() { for (const s of this.sys) for (const g of s.geoms) K3.mul(s.T[g.b], g.T, g.W); }
  collide() {
    this.nc = 0; this.upd();
    const G = this.ground, m = this.margin, S = this.sys;
    for (const s of S) {
      if (s.off || s.asleep) continue;
      for (const g of s.geoms) {
        if (G) this.vsGround(g, G);
        const W = g.W, r = g.rad + m;
        for (const st of this.statics) {
          if (st.t === PH_BOX) { const dx = W[3] - st.W[3], dy = W[7] - st.W[7], rr = r + st.rad; if (dx * dx + dy * dy > rr * rr) continue; }
          else { const dx = W[3] - st.W[3], dy = W[7] - st.W[7], rr = r + st.size[0]; if (dx * dx + dy * dy > rr * rr || W[11] - r > st.size[1]) continue; }
          this.pair(g, st);
        }
      }
    }
    for (let i = 0; i < S.length; i++) {
      if (S[i].off) continue;
      for (let j = i + 1; j < S.length; j++) {
        if (S[j].off || (S[i].asleep && S[j].asleep) || (S[i].noCollide && S[i].noCollide === S[j])) continue;
        for (const ga of S[i].geoms) for (const gb of S[j].geoms) {
          const A = ga.W, B = gb.W, dx = A[3] - B[3], dy = A[7] - B[7], dz = A[11] - B[11], rr = ga.rad + gb.rad + m;
          if (dx * dx + dy * dy + dz * dz < rr * rr) this.pair(ga, gb);
        }
      }
    }
  }
  vsGround(g, G) {
    const W = g.W, s = g.size, m = this.margin;
    if (g.t === PH_BOX) {
      if (W[11] - g.rad > m) return;
      for (let i = 0; i < 8; i++) {
        const x = i & 1 ? s[0] : -s[0], y = i & 2 ? s[1] : -s[1], z = i & 4 ? s[2] : -s[2];
        const pz = W[8] * x + W[9] * y + W[10] * z + W[11];
        if (pz < m) this.contact(g, G, W[0] * x + W[1] * y + W[2] * z + W[3], W[4] * x + W[5] * y + W[6] * z + W[7], pz, 0, 0, 1, -pz, i);
      }
    } else if (g.t === PH_SPH) { const z = W[11] - s[0]; if (z < m) this.contact(g, G, W[3], W[7], z, 0, 0, 1, -z, 0); }
    else if (g.t === PH_CAP) {
      for (let e = 0; e < 2; e++) { const L = e ? s[1] : -s[1], z = W[10] * L + W[11] - s[0]; if (z < m) this.contact(g, G, W[2] * L + W[3], W[6] * L + W[7], z, 0, 0, 1, -z, e); }
    } else if (g.t === PH_WHEEL) {
      const ax = W[2], ay = W[6], az = W[10], wl = Math.sqrt(Math.max(0, 1 - az * az)); if (wl < 0.25) return;
      const dx = az * ax / wl, dy = az * ay / wl, dz = (az * az - 1) / wl, r = s[0];
      const px = W[3] + r * dx, py = W[7] + r * dy, pz = W[11] + r * dz;
      if (pz < m) this.contact(g, G, px, py, pz, 0, 0, 1, -pz, 0);
    }
  }
  // sphere (centre c, radius r) against box geom B; writes n (from box to sphere), depth, closest box point into this._s
  sphBox(cx, cy, cz, r, B) {
    const W = B.W, h = B.size, dx = cx - W[3], dy = cy - W[7], dz = cz - W[11];
    const lx = W[0] * dx + W[4] * dy + W[8] * dz, ly = W[1] * dx + W[5] * dy + W[9] * dz, lz = W[2] * dx + W[6] * dy + W[10] * dz;
    const qx = clampN(lx, -h[0], h[0]), qy = clampN(ly, -h[1], h[1]), qz = clampN(lz, -h[2], h[2]);
    const ex = lx - qx, ey = ly - qy, ez = lz - qz, d2 = ex * ex + ey * ey + ez * ez, mr = r + this.margin;
    if (d2 > mr * mr) return false;
    let nx, ny, nz, dep, sx = qx, sy = qy, sz = qz;
    if (d2 > 1e-14) { const d = Math.sqrt(d2); nx = ex / d; ny = ey / d; nz = ez / d; dep = r - d; }
    else {
      const px = h[0] - Math.abs(lx), py = h[1] - Math.abs(ly), pz = h[2] - Math.abs(lz);
      if (px <= py && px <= pz) { nx = Math.sign(lx) || 1; ny = 0; nz = 0; dep = r + px; sx = nx * h[0]; }
      else if (py <= pz) { nx = 0; ny = Math.sign(ly) || 1; nz = 0; dep = r + py; sy = ny * h[1]; }
      else { nx = 0; ny = 0; nz = Math.sign(lz) || 1; dep = r + pz; sz = nz * h[2]; }
    }
    const o = this._s;
    o[0] = W[0] * nx + W[1] * ny + W[2] * nz; o[1] = W[4] * nx + W[5] * ny + W[6] * nz; o[2] = W[8] * nx + W[9] * ny + W[10] * nz; o[3] = dep;
    o[4] = W[0] * sx + W[1] * sy + W[2] * sz + W[3]; o[5] = W[4] * sx + W[5] * sy + W[6] * sz + W[7]; o[6] = W[8] * sx + W[9] * sy + W[10] * sz + W[11];
    return true;
  }
  // corners of box A that are inside box B -> contacts on A (normal out of B's face)
  cornersIn(A, B, flip, ga, gb, f0) {
    const WA = A.W, s = A.size, WB = B.W, h = B.size, m = this.margin;
    for (let i = 0; i < 8; i++) {
      const x = i & 1 ? s[0] : -s[0], y = i & 2 ? s[1] : -s[1], z = i & 4 ? s[2] : -s[2];
      const px = WA[0] * x + WA[1] * y + WA[2] * z + WA[3], py = WA[4] * x + WA[5] * y + WA[6] * z + WA[7], pz = WA[8] * x + WA[9] * y + WA[10] * z + WA[11];
      const dx = px - WB[3], dy = py - WB[7], dz = pz - WB[11];
      const lx = WB[0] * dx + WB[4] * dy + WB[8] * dz, ly = WB[1] * dx + WB[5] * dy + WB[9] * dz, lz = WB[2] * dx + WB[6] * dy + WB[10] * dz;
      const ex = h[0] - Math.abs(lx), ey = h[1] - Math.abs(ly), ez = h[2] - Math.abs(lz);
      if (ex < -m || ey < -m || ez < -m) continue;
      let k, d; if (ex <= ey && ex <= ez) { k = 0; d = ex; } else if (ey <= ez) { k = 1; d = ey; } else { k = 2; d = ez; }
      const sg = Math.sign(k === 0 ? lx : k === 1 ? ly : lz) || 1, nx = WB[k] * sg * flip, ny = WB[4 + k] * sg * flip, nz = WB[8 + k] * sg * flip;
      this.contact(ga, gb, px, py, pz, nx, ny, nz, d, f0 + i);
    }
  }
  // sphere attached to ga against geom gb (static or dynamic); normal points toward ga
  sphVs(ga, cx, cy, cz, r, gb, f) {
    if (gb.t === PH_BOX) { if (!this.sphBox(cx, cy, cz, r, gb)) return; const o = this._s; this.contact(ga, gb, cx - o[0] * r, cy - o[1] * r, cz - o[2] * r, o[0], o[1], o[2], o[3], f); return; }
    if (gb.t === PH_VCYL) {
      const W = gb.W, R = gb.size[0], top = gb.size[1], dx = cx - W[3], dy = cy - W[7], dl = Math.hypot(dx, dy);
      let qx, qy, qz = clampN(cz, 0, top);
      if (dl > R) { qx = W[3] + dx / dl * R; qy = W[7] + dy / dl * R; } else { qx = cx; qy = cy; }
      let ex = cx - qx, ey = cy - qy, ez = cz - qz, d = Math.hypot(ex, ey, ez), dep;
      if (d < 1e-9) { if (top - cz < R - dl) { ex = 0; ey = 0; ez = 1; dep = r + top - cz; } else { ex = dx / (dl || 1); ey = dy / (dl || 1); ez = 0; dep = r + R - dl; } }
      else { ex /= d; ey /= d; ez /= d; dep = r - d; }
      if (dep < -this.margin) return;
      this.contact(ga, gb, cx - ex * r, cy - ey * r, cz - ez * r, ex, ey, ez, dep, f); return;
    }
    const B = gb.W;
    if (gb.t === PH_SPH || gb.t === PH_WHEEL) { const rb = gb.size[0], dx = cx - B[3], dy = cy - B[7], dz = cz - B[11], d = Math.hypot(dx, dy, dz), dep = r + rb - d; if (dep < -this.margin || d < 1e-9) return; this.contact(ga, gb, cx - dx / d * r, cy - dy / d * r, cz - dz / d * r, dx / d, dy / d, dz / d, dep, f); return; }
    if (gb.t === PH_CAP) {
      const L = gb.size[1], ax = B[2], ay = B[6], az = B[10], t = clampN((cx - B[3]) * ax + (cy - B[7]) * ay + (cz - B[11]) * az, -L, L);
      const qx = B[3] + ax * t, qy = B[7] + ay * t, qz = B[11] + az * t, rb = gb.size[0], dx = cx - qx, dy = cy - qy, dz = cz - qz, d = Math.hypot(dx, dy, dz), dep = r + rb - d;
      if (dep < -this.margin || d < 1e-9) return; this.contact(ga, gb, cx - dx / d * r, cy - dy / d * r, cz - dz / d * r, dx / d, dy / d, dz / d, dep, f);
    }
  }
  // sample points along a geom for shapes without an exact routine
  samples(g, out) {
    const W = g.W, s = g.size; out.length = 0;
    if (g.t === PH_SPH || g.t === PH_WHEEL) out.push([W[3], W[7], W[11], s[0]]);
    else if (g.t === PH_CAP) for (const t of [-1, 0, 1]) out.push([W[3] + W[2] * s[1] * t, W[7] + W[6] * s[1] * t, W[11] + W[10] * s[1] * t, s[0]]);
    return out;
  }
  pair(ga, gb) {
    const A = ga.t, B = gb.t, sm = this._sm || (this._sm = []);
    if (A === PH_BOX && B === PH_BOX) { this.cornersIn(ga, gb, 1, ga, gb, 0); this.cornersIn(gb, ga, -1, ga, gb, 8); return; }
    if (A === PH_BOX && B === PH_VCYL) {
      // box corners inside the post, and the post poking into a box face
      const W = gb.W, R = gb.size[0], top = gb.size[1], WA = ga.W, s = ga.size;
      for (let i = 0; i < 8; i++) {
        const x = i & 1 ? s[0] : -s[0], y = i & 2 ? s[1] : -s[1], z = i & 4 ? s[2] : -s[2];
        const px = WA[0] * x + WA[1] * y + WA[2] * z + WA[3], py = WA[4] * x + WA[5] * y + WA[6] * z + WA[7], pz = WA[8] * x + WA[9] * y + WA[10] * z + WA[11];
        const dx = px - W[3], dy = py - W[7], dl = Math.hypot(dx, dy); if (dl > R || pz > top || dl < 1e-9) continue;
        this.contact(ga, gb, px, py, pz, dx / dl, dy / dl, 0, R - dl, i);
      }
      const cz = clampN(WA[11], 0.01, top - 0.01);
      if (this.sphBox(W[3], W[7], cz, R, ga)) { const o = this._s; this.contact(ga, gb, o[4], o[5], o[6], -o[0], -o[1], -o[2], o[3], 9); }
      return;
    }
    if (A === PH_BOX && (B === PH_SPH || B === PH_CAP || B === PH_WHEEL)) {
      this.samples(gb, sm).forEach(([x, y, z, r], i) => { if (this.sphBox(x, y, z, r, ga)) { const o = this._s; this.contact(ga, gb, o[4], o[5], o[6], -o[0], -o[1], -o[2], o[3], 20 + i); } });
      return;
    }
    this.samples(ga, sm).forEach(([x, y, z, r], i) => this.sphVs(ga, x, y, z, r, gb, 30 + i));
  }
  /* ---------- constraint solver ---------- */
  rowAlloc(s) {
    if (!s.pool || s.pool.length < (s.used + 1) * s.n) { const np = new Float64Array(Math.max(64, (s.used + 1) * 2) * s.n), ny = new Float64Array(np.length); if (s.pool) { np.set(s.pool); ny.set(s.ypool); } s.pool = np; s.ypool = ny; }
    return s.used++ * s.n;
  }
  mkRow(c, kind, ux, uy, uz) {
    // kind 0: point velocity along u at c.p; 1: angular velocity along u
    const A = c.a.s, B = c.b.s, R = c.r[c.nr] || (c.r[c.nr] = { lam: 0 }); c.nr++;
    R.A = A; R.oa = this.rowAlloc(A); R.B = B || null; R.lam = 0; R.ia = A.chainIdx(c.a.b);
    if (kind === 0) A.jacPt(c.a.b, c.p, ux, uy, uz, A.pool, R.oa, 1); else A.jacRot(c.a.b, ux, uy, uz, A.pool, R.oa, 1);
    let D = 0; const n = A.n; for (let i = 0; i < n; i++) A.ypool[R.oa + i] = A.pool[R.oa + i];
    phSolve(A.L, n, A.ypool, R.oa); for (let i = 0; i < n; i++) D += A.pool[R.oa + i] * A.ypool[R.oa + i];
    if (B) {
      R.ob = this.rowAlloc(B); R.ib = B.chainIdx(c.b.b);
      if (kind === 0) B.jacPt(c.b.b, c.p, ux, uy, uz, B.pool, R.ob, -1); else B.jacRot(c.b.b, ux, uy, uz, B.pool, R.ob, -1);
      const m = B.n; for (let i = 0; i < m; i++) B.ypool[R.ob + i] = B.pool[R.ob + i];
      phSolve(B.L, m, B.ypool, R.ob); for (let i = 0; i < m; i++) D += B.pool[R.ob + i] * B.ypool[R.ob + i];
    }
    R.D = D * (1 + this.cfm) + 1e-12; R.cf = D * this.cfm;
    return R;
  }
  vel(R) {
    const A = R.A, J = A.pool, v = A.v, o = R.oa, ia = R.ia; let u = 0;
    if (ia) for (let k = 0; k < ia.length; k++) { const i = ia[k]; u += J[o + i] * v[i]; } else for (let i = 0; i < A.n; i++) u += J[o + i] * v[i];
    if (R.B) { const B = R.B, J2 = B.pool, v2 = B.v, o2 = R.ob, ib = R.ib; if (ib) for (let k = 0; k < ib.length; k++) { const i = ib[k]; u += J2[o2 + i] * v2[i]; } else for (let i = 0; i < B.n; i++) u += J2[o2 + i] * v2[i]; }
    return u;
  }
  apply(R, dl) {
    if (!dl) return; const A = R.A, n = A.n, Y = A.ypool, v = A.v, o = R.oa;
    for (let i = 0; i < n; i++) v[i] += Y[o + i] * dl;
    if (R.B) { const B = R.B, m = B.n, Y2 = B.ypool, v2 = B.v, o2 = R.ob; for (let i = 0; i < m; i++) v2[i] += Y2[o2 + i] * dl; }
  }
  solve(h) {
    for (const s of this.sys) { s.used = 0; s.cf.fill(0); s.cfz.fill(0); }
    const C = this.C, nc = this.nc, cache = this.cache, next = new Map(), erp = this.erp, slop = this.slop;
    // joint limits as one-sided constraints
    const lims = this._lims || (this._lims = []); lims.length = 0;
    for (const s of this.sys) {
      if (s.off) continue;
      for (let k = 0; k < s.nj; k++) {
        const lo = s.lo[k], hi = s.hi[k], q = s.q[k], d = 6 + k; if (lo < -1e8) continue;
        for (let side = 0; side < 2; side++) {
          const gap = side ? hi - q : q - lo, sg = side ? -1 : 1;
          if (gap > 0.02) continue;
          const R = { A: s, oa: this.rowAlloc(s), B: null, lam: 0, ia: Int32Array.of(d) }, J = s.pool, n = s.n; for (let i = 0; i < n; i++) J[R.oa + i] = 0; J[R.oa + d] = sg;
          for (let i = 0; i < n; i++) s.ypool[R.oa + i] = J[R.oa + i]; phSolve(s.L, n, s.ypool, R.oa);
          R.D = s.ypool[R.oa + d] * sg * 1.002 + 1e-12; R.cf = R.D * 0.002;
          R.tgt = gap > 0 ? -gap / h : Math.min(-gap * erp / h, 2); R.key = -(s.uid * 4096 + k * 2 + side) - 1;
          const w = cache.get(R.key); if (w) { R.lam = w[0]; this.apply(R, R.lam); }
          lims.push(R);
        }
      }
    }
    // contact rows: normal, two friction directions, optional rolling friction
    for (let i = 0; i < nc; i++) {
      const c = C[i], n = c.n; c.nr = 0;
      let t1x, t1y, t1z;
      if (c.aniso) {
        const g = c.aniso, s = g.s, Tb = s.T[g.anisoB != null ? g.anisoB : g.b], a = g.aniso, wx = Tb[0] * a[0] + Tb[1] * a[1] + Tb[2] * a[2], wy = Tb[4] * a[0] + Tb[5] * a[1] + Tb[6] * a[2], wz = Tb[8] * a[0] + Tb[9] * a[1] + Tb[10] * a[2], dn = wx * n[0] + wy * n[1] + wz * n[2];
        t1x = wx - dn * n[0]; t1y = wy - dn * n[1]; t1z = wz - dn * n[2];
      } else if (Math.abs(n[2]) < 0.9) { t1x = -n[1]; t1y = n[0]; t1z = 0; } else { t1x = 0; t1y = -n[2]; t1z = n[1]; }
      let l = Math.hypot(t1x, t1y, t1z) || 1; t1x /= l; t1y /= l; t1z /= l;
      const t2x = n[1] * t1z - n[2] * t1y, t2y = n[2] * t1x - n[0] * t1z, t2z = n[0] * t1y - n[1] * t1x;
      c.t1[0] = t1x; c.t1[1] = t1y; c.t1[2] = t1z; c.t2[0] = t2x; c.t2[1] = t2y; c.t2[2] = t2z;
      const rn = this.mkRow(c, 0, n[0], n[1], n[2]), r1 = this.mkRow(c, 0, t1x, t1y, t1z), r2 = this.mkRow(c, 0, t2x, t2y, t2z);
      const u0 = this.vel(rn);
      let tgt = c.d > 0 ? Math.min(erp * Math.max(0, c.d - slop) / h, 1.0) : c.d / h;
      if (c.rest > 0 && u0 < -0.25) tgt = Math.max(tgt, -c.rest * u0);
      rn.tgt = tgt; r1.tgt = c.conv || 0; r2.tgt = 0;
      c.mu1 = c.mu; c.mu2 = c.aniso ? (c.aniso.mu2 || 0.02) : c.mu;
      if (c.roll > 0) { c.rr1 = this.mkRow(c, 1, t1x, t1y, t1z); c.rr2 = this.mkRow(c, 1, t2x, t2y, t2z); c.rr1.tgt = 0; c.rr2.tgt = 0; } else { c.rr1 = null; c.rr2 = null; }
      // warm start
      const w = cache.get(c.key);
      if (w) {
        rn.lam = w[0] * 0.95; r1.lam = (w[1] * t1x + w[2] * t1y + w[3] * t1z) * 0.95; r2.lam = (w[1] * t2x + w[2] * t2y + w[3] * t2z) * 0.95;
        this.apply(rn, rn.lam); this.apply(r1, r1.lam); this.apply(r2, r2.lam);
      }
    }
    // projected Gauss-Seidel
    for (let it = 0; it < this.iters; it++) {
      for (const R of lims) { const u = this.vel(R), nl = Math.max(0, R.lam + (R.tgt - u - R.cf * R.lam) / R.D); this.apply(R, nl - R.lam); R.lam = nl; }
      for (let i = 0; i < nc; i++) {
        const c = C[i], rn = c.r[0], r1 = c.r[1], r2 = c.r[2];
        let u = this.vel(rn); const nl = Math.max(0, rn.lam + (rn.tgt - u - rn.cf * rn.lam) / rn.D); this.apply(rn, nl - rn.lam); rn.lam = nl;
        const u1 = this.vel(r1), u2 = this.vel(r2);
        let l1 = r1.lam + (r1.tgt - u1 - r1.cf * r1.lam) / r1.D, l2 = r2.lam + (r2.tgt - u2 - r2.cf * r2.lam) / r2.D;
        const f1 = c.mu1 * nl, f2 = c.mu2 * nl;
        if (c.aniso) { l1 = clampN(l1, -f1, f1); l2 = clampN(l2, -f2, f2); }
        else { const L = Math.hypot(l1, l2); if (L > f1) { const k = f1 / L; l1 *= k; l2 *= k; } }
        this.apply(r1, l1 - r1.lam); r1.lam = l1; this.apply(r2, l2 - r2.lam); r2.lam = l2;
        if (c.rr1) {
          const fr = c.roll * nl;
          for (const R of [c.rr1, c.rr2]) { u = this.vel(R); const lr = clampN(R.lam + (0 - u - R.cf * R.lam) / R.D, -fr, fr); this.apply(R, lr - R.lam); R.lam = lr; }
        }
      }
    }
    // remember impulses, report forces
    for (const R of lims) next.set(R.key, [R.lam]);
    for (let i = 0; i < nc; i++) {
      const c = C[i], rn = c.r[0], r1 = c.r[1], r2 = c.r[2];
      const fx = c.n[0] * rn.lam + c.t1[0] * r1.lam + c.t2[0] * r2.lam, fy = c.n[1] * rn.lam + c.t1[1] * r1.lam + c.t2[1] * r2.lam, fz = c.n[2] * rn.lam + c.t1[2] * r1.lam + c.t2[2] * r2.lam;
      next.set(c.key, [rn.lam, fx - c.n[0] * rn.lam, fy - c.n[1] * rn.lam, fz - c.n[2] * rn.lam]);
      c.fn = rn.lam / h; c.f = [fx / h, fy / h, fz / h];
      const sa = c.a.s, ba = c.a.b; sa.cf[3 * ba] += fx / h; sa.cf[3 * ba + 1] += fy / h; sa.cf[3 * ba + 2] += fz / h; sa.cfz[ba] += rn.lam / h;
      if (c.b.s) { const sb = c.b.s, bb = c.b.b; sb.cf[3 * bb] -= fx / h; sb.cf[3 * bb + 1] -= fy / h; sb.cf[3 * bb + 2] -= fz / h; sb.cfz[bb] += rn.lam / h; }
    }
    this.cache = next;
  }
  // list of current contacts for drawing: [x, y, z, fx, fy, fz]
  contactList() { const out = []; for (let i = 0; i < this.nc; i++) { const c = this.C[i]; if (c.fn > 1e-4 || c.d > 0) out.push([c.p[0], c.p[1], c.p[2], c.f ? c.f[0] : 0, c.f ? c.f[1] : 0, c.f ? c.f[2] : 0]); } return out; }
}
