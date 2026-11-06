/* ============================== SnoBots: model-based biped walking controller ==============================
   How it walks (the classic model-based recipe, as used on many position-controlled humanoids):
   1. Footsteps come from the velocity command: each step moves a walking frame, and the leading foot opens
      sideways or turns first so the feet never cross.
   2. The centre of mass follows the linear inverted pendulum. Its divergent component of motion (DCM,
      ξ = c + ċ/ω) is planned backwards in time over a ZMP that sits on the stance foot during single
      support and slides to the next foot during double support.
   3. A stabilizer measures the real DCM. Small errors shift the commanded centre of mass (the servos then
      push the ZMP the right way); big errors move the next footstep toward the capture point.
   4. Leg inverse kinematics turns pelvis and foot poses into joint targets for the servos.
   Nothing here moves the robot directly: the physics engine decides what really happens. */

// allocation-free damped least-squares IK for one 6-joint leg (chain from the pelvis link to the sole)
class LegIK {
  constructor(chain) {
    this.ch = chain; this.mi = []; chain.forEach((j, i) => { if (j.type !== 'fixed') this.mi.push(i); });
    this.Tj = chain.map(() => new Float64Array(12)); this.Pt = chain.map(() => new Float64Array(12)); this.R = new Float64Array(12); this.E = new Float64Array(12);
    this.kOf = chain.map((j, i) => this.mi.indexOf(i));
    this.J = new Float64Array(36); this.N = new Float64Array(36); this.e = new Float64Array(6); this.y = new Float64Array(6);
  }
  fk(q) {
    const n = this.ch.length; let P = this.I || (this.I = K3.I());
    for (let i = 0; i < n; i++) {
      const j = this.ch[i]; K3.mul(P, j.O, this.Tj[i]);
      if (j.type === 'fixed') this.Pt[i].set(this.Tj[i]);
      else { K3.axisAngle(j.ax[0], j.ax[1], j.ax[2], q[this.kOf[i]] + j.offset, this.R); K3.mul(this.Tj[i], this.R, this.Pt[i]); }
      P = this.Pt[i];
    }
    this.E.set(P); return this.E;
  }
  jac(q) {
    const E = this.fk(q), J = this.J;
    for (let k = 0; k < 6; k++) {
      const i = this.mi[k], j = this.ch[i], M = this.Tj[i], zx = M[0] * j.ax[0] + M[1] * j.ax[1] + M[2] * j.ax[2], zy = M[4] * j.ax[0] + M[5] * j.ax[1] + M[6] * j.ax[2], zz = M[8] * j.ax[0] + M[9] * j.ax[1] + M[10] * j.ax[2];
      const px = E[3] - M[3], py = E[7] - M[7], pz = E[11] - M[11];
      J[k] = zy * pz - zz * py; J[6 + k] = zz * px - zx * pz; J[12 + k] = zx * py - zy * px; J[18 + k] = zx; J[24 + k] = zy; J[30 + k] = zz;
    }
    return J;
  }
  // q: Float64Array(6) warm start, modified in place. target: sole pose in the pelvis frame
  solve(q, target, iters = 3) {
    const J = this.J, N = this.N, e = this.e, y = this.y, lam = 0.004;
    let err = 0;
    for (let it = 0; it <= iters; it++) {
      const E = this.fk(q);
      e[0] = target[3] - E[3]; e[1] = target[7] - E[7]; e[2] = target[11] - E[11]; e[3] = 0; e[4] = 0; e[5] = 0;
      for (let c = 0; c < 3; c++) {
        const ex = E[c], ey = E[4 + c], ez = E[8 + c], tx = target[c], ty = target[4 + c], tz = target[8 + c];
        e[3] += 0.5 * (ey * tz - ez * ty); e[4] += 0.5 * (ez * tx - ex * tz); e[5] += 0.5 * (ex * ty - ey * tx);
      }
      err = Math.hypot(e[0], e[1], e[2]);
      if (it === iters || (err < 2e-5 && Math.hypot(e[3], e[4], e[5]) < 2e-4)) break;
      // Jacobian columns (pos, rot) for each movable joint
      for (let k = 0; k < 6; k++) {
        const i = this.mi[k], j = this.ch[i], M = this.Tj[i], zx = M[0] * j.ax[0] + M[1] * j.ax[1] + M[2] * j.ax[2], zy = M[4] * j.ax[0] + M[5] * j.ax[1] + M[6] * j.ax[2], zz = M[8] * j.ax[0] + M[9] * j.ax[1] + M[10] * j.ax[2];
        const px = E[3] - M[3], py = E[7] - M[7], pz = E[11] - M[11];
        J[k] = zy * pz - zz * py; J[6 + k] = zz * px - zx * pz; J[12 + k] = zx * py - zy * px; J[18 + k] = zx; J[24 + k] = zy; J[30 + k] = zz;
      }
      // (J J^T + lam^2 I) y = e ; dq = J^T y
      for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) { let s = r === c ? lam * lam : 0; for (let k = 0; k < 6; k++) s += J[6 * r + k] * J[6 * c + k]; N[6 * r + c] = s; }
      for (let r = 0; r < 6; r++) y[r] = e[r];
      for (let c = 0; c < 6; c++) {
        let p = c; for (let r = c + 1; r < 6; r++) if (Math.abs(N[6 * r + c]) > Math.abs(N[6 * p + c])) p = r;
        if (p !== c) { for (let k = 0; k < 6; k++) { const t = N[6 * p + k]; N[6 * p + k] = N[6 * c + k]; N[6 * c + k] = t; } const t = y[p]; y[p] = y[c]; y[c] = t; }
        const d = N[6 * c + c] || 1e-12;
        for (let r = c + 1; r < 6; r++) { const f = N[6 * r + c] / d; if (!f) continue; for (let k = c; k < 6; k++) N[6 * r + k] -= f * N[6 * c + k]; y[r] -= f * y[c]; }
      }
      for (let r = 5; r >= 0; r--) { let s = y[r]; for (let k = r + 1; k < 6; k++) s -= N[6 * r + k] * y[k]; y[r] = s / (N[6 * r + r] || 1e-12); }
      let big = 0; for (let k = 0; k < 6; k++) { let s = 0; for (let r = 0; r < 6; r++) s += J[6 * r + k] * y[r]; e[k] = s; big = Math.max(big, Math.abs(s)); }
      const sc = big > 0.4 ? 0.4 / big : 1;
      for (let k = 0; k < 6; k++) { const j = this.ch[this.mi[k]]; q[k] = clampN(q[k] + e[k] * sc, j.lo, j.hi); }
    }
    return err;
  }
}

const smooth5 = (t) => { t = clampN(t, 0, 1); return t * t * t * (10 + t * (-15 + 6 * t)); };

class BipedWalker {
  /* o: { kin, S (PhysSys), sole: {l, r}, footY, z (pelvis height), g: gait params } */
  constructor(o) {
    Object.assign(this, o);
    const kin = this.kin;
    this.ik = { l: new LegIK(kin.chain(kin.root, o.sole.l)), r: new LegIK(kin.chain(kin.root, o.sole.r)) };
    this.legJ = { l: this.ik.l.mi.map(i => this.ik.l.ch[i].name), r: this.ik.r.mi.map(i => this.ik.r.ch[i].name) };
    this.lq = { l: new Float64Array(6), r: new Float64Array(6) };
    this.cmd = [0, 0, 0]; this.v = [0, 0, 0];
    this.fb = { kc: 1.0, kstep: 1.0, dead: 0.012 };
    this._P = new Float64Array(12); this._F = new Float64Array(12); this._c = new Float64Array(6);
    // CoM model in the pelvis frame (uses the physics bodies' masses)
    const S = this.S; this.cT = Array.from({ length: S.nb }, () => K3.I()); this._M = new Float64Array(12); this._R = new Float64Array(12);
  }
  get om() { return Math.sqrt(PH_G / this.zc); }
  // centre of mass relative to the pelvis for joint values q (object by name)
  comLocal(q, out) {
    const S = this.S, T = this.cT; T[0].set(K3.I());
    let m = 0, x = 0, y = 0, z = 0;
    for (let b = 0; b < S.nb; b++) {
      if (b > 0) {
        K3.mul(T[S.par[b]], S.Xj[b], this._M); const a = S.ax[b], v = q[S.jn[b]] != null ? q[S.jn[b]] : S.q[b - 1];
        if (S.jt[b] === 1) { K3.axisAngle(a[0], a[1], a[2], v + S.qoff[b], this._R); K3.mul(this._M, this._R, T[b]); } else T[b].set(this._M);
      }
      const M = T[b], c = S.com[b], mb = S.mass[b];
      m += mb; x += mb * (M[0] * c[0] + M[1] * c[1] + M[2] * c[2] + M[3]); y += mb * (M[4] * c[0] + M[5] * c[1] + M[6] * c[2] + M[7]); z += mb * (M[8] * c[0] + M[9] * c[1] + M[10] * c[2] + M[11]);
    }
    out[0] = x / m; out[1] = y / m; out[2] = z / m; return out;
  }
  // static balance: move the pelvis so the model CoM sits over the middle of the feet, re-solving the legs
  balance(q, pelvis, feet) {
    const P = K3.I(), c = [0, 0, 0];
    for (let it = 0; it < 4; it++) {
      this.comLocal(q, c);
      const yaw = pelvis.yaw, cy = Math.cos(yaw), sy = Math.sin(yaw), mx = (feet.l.x + feet.r.x) / 2, my = (feet.l.y + feet.r.y) / 2;
      pelvis.x = mx - (cy * c[0] - sy * c[1]); pelvis.y = my - (sy * c[0] + cy * c[1]);
      const Pi = K3.inv(K3.fromRPY(pelvis.x, pelvis.y, pelvis.z, 0, 0, yaw));
      for (const s of ['l', 'r']) {
        const f = feet[s]; this.legJ[s].forEach((n, i) => { this.lq[s][i] = q[n] || 0; });
        this.ik[s].solve(this.lq[s], K3.mul(Pi, K3.fromRPY(f.x, f.y, 0, 0, 0, f.yaw)), 30);
        this.legJ[s].forEach((n, i) => { q[n] = this.lq[s][i]; });
      }
    }
    return pelvis;
  }
  footPose(s) { const T = this.S.linkT(this.sole[s], this._F); return { x: T[3], y: T[7], yaw: Math.atan2(T[4], T[0]) }; }
  // (re)start standing on the current feet
  reset(q) {
    this.q = q; this.feet = { l: this.footPose('l'), r: this.footPose('r') };
    for (const s of ['l', 'r']) this.legJ[s].forEach((n, i) => { this.lq[s][i] = q[n] || 0; });
    const cl = this.comLocal(q, [0, 0, 0]); this.cOff = cl; this.zc = this.z + cl[2];
    const c = this.S.comState(this._c), m = this.mid();
    this.c = [c[0], c[1]]; this.cd = [0, 0]; this.xi = [c[0], c[1]];
    this.mode = 'stand'; this.segs = null; this.steps = []; this.swing = null; this.sup = null; this.v = [0, 0, 0];
    this.yaw = Math.atan2(Math.sin(this.feet.l.yaw) + Math.sin(this.feet.r.yaw), Math.cos(this.feet.l.yaw) + Math.cos(this.feet.r.yaw));
    this.r = m; this.stepsDone = 0; this.recover = 0; this.err = [0, 0]; this.pitchCmd = 0;
  }
  // ZMP reference point on a foot: its centre, nudged toward the other foot
  zmpOn(s) { const f = this.feet[s], k = (s === 'l' ? -1 : 1) * this.g.zmpIn; return [f.x - Math.sin(f.yaw) * k + Math.cos(f.yaw) * this.g.zmpX, f.y + Math.cos(f.yaw) * k + Math.sin(f.yaw) * this.g.zmpX]; }
  mid() { const l = this.feet.l, r = this.feet.r, x = this.g.zmpX; return [(l.x + r.x) / 2 + Math.cos(this.yaw || 0) * x, (l.y + r.y) / 2 + Math.sin(this.yaw || 0) * x]; }
  setCmd(vx, vy, wz) { this.cmd = [vx, vy, wz]; }
  walking() { return this.mode === 'walk'; }
  /* ---------- footstep planning ---------- */
  // pose of foot s relative to foot o
  rel(p, o) { const c = Math.cos(o.yaw), s = Math.sin(o.yaw), dx = p.x - o.x, dy = p.y - o.y; return { x: c * dx + s * dy, y: -s * dx + c * dy, yaw: wrapPi(p.yaw - o.yaw) }; }
  abs(l, o) { const c = Math.cos(o.yaw), s = Math.sin(o.yaw); return { x: o.x + c * l.x - s * l.y, y: o.y + s * l.x + c * l.y, yaw: wrapPi(o.yaw + l.yaw) }; }
  clampStep(sw, st, t) {
    // keep the swing target reachable and away from the stance foot
    const g = this.g, sg = sw === 'l' ? 1 : -1, l = this.rel(t, st), fy = this.footY * 2;
    l.x = clampN(l.x, -g.maxBack, g.maxFwd);
    const ly = sg * l.y; l.y = sg * clampN(ly, fy * g.minSep, fy + g.maxSide);
    l.yaw = sg > 0 ? clampN(l.yaw, -g.maxYawIn, g.maxYaw) : clampN(l.yaw, -g.maxYaw, g.maxYawIn);
    return this.abs(l, st);
  }
  // next step for swing foot sw given the stance foot pose and the commanded velocity
  nextStep(sw, st, v) {
    const g = this.g, sg = sw === 'l' ? 1 : -1, T = g.Tss + g.Tds;
    const frame = this.abs({ x: 0, y: sg * this.footY, yaw: 0 }, st);   // walking frame implied by the stance foot
    const lead = (val) => (val > 0 && sg > 0) || (val < 0 && sg < 0);
    const dx = v[0] * T, dy = lead(v[1]) ? 2 * v[1] * T : 0, dyaw = lead(v[2]) ? 2 * v[2] * T : 0;
    const fr = this.abs({ x: dx, y: dy, yaw: dyaw }, frame);
    const t = this.abs({ x: 0, y: sg * this.footY, yaw: 0 }, fr);
    return Object.assign(this.clampStep(sw, st, t), { s: sw });
  }
  // build the ZMP segments and backward DCM from the current situation
  plan(first) {
    const g = this.g, segs = [], om = this.om;
    let st = first.stance, feet = { l: Object.assign({}, this.feet.l), r: Object.assign({}, this.feet.r) }, r0 = this.r.slice(), v = this.v.slice();
    const zmpAt = (s) => { const f = feet[s], k = (s === 'l' ? -1 : 1) * g.zmpIn; return [f.x - Math.sin(f.yaw) * k + Math.cos(f.yaw) * g.zmpX, f.y + Math.cos(f.yaw) * k + Math.sin(f.yaw) * g.zmpX]; };
    const steps = this.steps;
    for (let i = 0; i < steps.length; i++) {
      const p = steps[i], sw = p.s, stance = sw === 'l' ? 'r' : 'l', rz = zmpAt(stance);
      segs.push({ T: i === 0 ? first.Tds : g.Tds, ra: r0, rb: rz, kind: 'ds', step: i });
      segs.push({ T: p.Tss || g.Tss, ra: rz, rb: rz, kind: 'ss', step: i });
      feet[sw] = { x: p.x, y: p.y, yaw: p.yaw }; r0 = rz; st = sw;
    }
    void st; void v;
    const l = feet.l, rr = feet.r, mid = [(l.x + rr.x) / 2 + Math.cos(this.yaw) * g.zmpX, (l.y + rr.y) / 2 + Math.sin(this.yaw) * g.zmpX];
    const end = this.stopping ? mid : r0;
    if (this.stopping) segs.push({ T: g.Tds * 2.5, ra: r0, rb: mid, kind: 'ds', step: -1 });
    // backward recursion: xi at the start of every segment
    const back = () => {
      let xi = end.slice();
      for (let k = segs.length - 1; k >= 0; k--) {
        const s = segs[k], rd = [(s.rb[0] - s.ra[0]) / s.T / om, (s.rb[1] - s.ra[1]) / s.T / om], ex = Math.exp(-om * s.T);
        s.xe = xi; s.rd = rd;
        xi = [s.ra[0] + rd[0] + ex * (xi[0] - s.rb[0] - rd[0]), s.ra[1] + rd[1] + ex * (xi[1] - s.rb[1] - rd[1])];
        s.xs = xi;
      }
    };
    back();
    // starting from rest: choose where the first transfer begins so the plan starts at the current DCM
    // (the ZMP first swings away from the stance foot, which pushes the body toward it)
    if (first.xi0 && segs.length) {
      const s = segs[0], wT = om * s.T, ex = Math.exp(-wT), a = 1 - 1 / wT + ex / wT, b = 1 / wT - ex - ex / wT;
      const ra = [0, 1].map(i => (first.xi0[i] - b * s.rb[i] - ex * s.xe[i]) / a);
      const m = this.mid(), lim = this.footY * 1.6, dx = ra[0] - m[0], dy = ra[1] - m[1], c = Math.cos(this.yaw), sn = Math.sin(this.yaw);
      const lx = clampN(c * dx + sn * dy, -0.03, 0.03), ly = clampN(-sn * dx + c * dy, -lim, lim);
      s.ra = [m[0] + c * lx - sn * ly, m[1] + sn * lx + c * ly]; back();
    }
    this.segs = segs; this.si = 0; this.st = 0;
  }
  refAt(s, t, out) {
    const om = this.om, u = t / s.T, ex = Math.exp(om * (t - s.T));
    const rx = s.ra[0] + (s.rb[0] - s.ra[0]) * u, ry = s.ra[1] + (s.rb[1] - s.ra[1]) * u;
    out[0] = rx + s.rd[0] + ex * (s.xe[0] - s.rb[0] - s.rd[0]); out[1] = ry + s.rd[1] + ex * (s.xe[1] - s.rb[1] - s.rd[1]); out[2] = rx; out[3] = ry;
    return out;
  }
  start(push) {
    // first swing foot: the one on the side we move toward (or away from a push)
    const v = push ? [0, 0, 0] : this.v, e = push || [v[0], v[1]];
    const lat = -Math.sin(this.yaw) * e[0] + Math.cos(this.yaw) * e[1];
    const sw = Math.abs(lat) > 1e-3 ? (lat > 0 ? 'l' : 'r') : (this.cmd[2] > 1e-3 ? 'l' : this.cmd[2] < -1e-3 ? 'r' : (this.lastSw === 'l' ? 'r' : 'l'));
    this.mode = 'walk'; this.stopping = false; this.steps = [];
    this.fillSteps(sw);
    if (push) this.steps[0].Tss = this.g.Tss * 0.75;   // a quick recovery step
    this.plan({ stance: sw === 'l' ? 'r' : 'l', Tds: push ? this.g.Tds * 0.6 : this.g.T0, xi0: this.xm });
  }
  fillSteps(sw) {
    const want = this.g.horizon || 3; let feet = { l: this.feet.l, r: this.feet.r };
    this.steps.forEach(p => { feet = Object.assign({}, feet, { [p.s]: p }); sw = p.s === 'l' ? 'r' : 'l'; });
    // velocity expected at each future step (the command is ramped), so the plan already holds the speed-up
    const g = this.g, T = g.Tss + g.Tds;
    while (this.steps.length < want) {
      const k = this.steps.length + 1, v = [0, 1, 2].map(i => approach(this.v[i], this.cmd[i], g.acc[i] * T * k));
      const st = sw === 'l' ? 'r' : 'l', p = this.nextStep(sw, feet[st], v);
      this.steps.push(p); feet = Object.assign({}, feet, { [sw]: p }); sw = st;
    }
  }
  /* ---------- per control tick ---------- */
  update(dt, meas) {
    const g = this.g, om = this.om;
    // ramp the velocity command
    for (let i = 0; i < 3; i++) this.v[i] = approach(this.v[i], this.cmd[i], g.acc[i] * dt);
    const want = Math.abs(this.cmd[0]) + Math.abs(this.cmd[1]) + Math.abs(this.cmd[2]) > 1e-3;
    const ref = this._ref || (this._ref = [0, 0, 0, 0]);
    // measured DCM
    const c = meas.com, xm = [c[0] + c[3] / om, c[1] + c[4] / om]; this.xm = xm;
    if (this.mode === 'stand') {
      const m = this.mid(); this.r = m; ref[0] = m[0]; ref[1] = m[1]; ref[2] = m[0]; ref[3] = m[1];
      const e = [xm[0] - m[0], xm[1] - m[1]], en = Math.hypot(e[0], e[1]);
      if (want) this.start(null);
      else if (en > g.pushStep) { this.recover++; this.start(e); }
    }
    let swingS = 0;
    if (this.mode === 'walk') {
      this.st += dt;
      let s = this.segs[this.si];
      while (s && this.st >= s.T) {
        const over = this.st - s.T;
        if (s.kind === 'ss') this.touchdown(s.step); else this.si++;
        this.st = over; s = this.segs[this.si];
      }
      if (!s) { this.mode = 'stand'; this.segs = null; this.swing = null; }
      else {
        this.refAt(s, this.st, ref); this.r = [ref[2], ref[3]];
        if (s.kind === 'ss') {
          const p = this.steps[s.step], u = this.st / s.T; swingS = u;
          if (!this.swing || this.swing.step !== p) this.swing = { step: p, from: Object.assign({}, this.feet[p.s]) };
          // step adjustment toward the capture point while the foot is still in the air: compare the DCM we will
          // have at touchdown (ZMP held on the stance foot) with the plan, and move this and later footsteps
          if (u < 0.6) {
            if (!p.base) { this.steps.forEach(q => { q.base = { x: q.x, y: q.y }; }); p.xe0 = s.xe.slice(); }
            const st = p.s === 'l' ? 'r' : 'l', rz = this.zmpOn(st), ex = Math.exp(om * (s.T - this.st)), wT = om * this.g.Tds, al = (1 - Math.exp(-wT)) / wT;
            const d = [rz[0] + ex * (xm[0] - rz[0]) - p.xe0[0], rz[1] + ex * (xm[1] - rz[1]) - p.xe0[1]], dn = Math.hypot(d[0], d[1]);
            const dead = Math.max(this.fb.dead, (ex - 1) * this.g.zmpMargin);   // what the ankles can still fix
            const k = dn > dead ? this.fb.kstep * (1 - dead / dn) / al : 0;
            const t = this.clampStep(p.s, this.feet[st], { x: p.base.x + k * d[0], y: p.base.y + k * d[1], yaw: p.yaw }), mx = t.x - p.base.x, my = t.y - p.base.y;
            if (Math.hypot(mx - (p.x - p.base.x), my - (p.y - p.base.y)) > 2e-4) {
              this.steps.forEach(q => { if (q.base) { q.x = q.base.x + mx; q.y = q.base.y + my; } });
              this.adjust = Math.max(this.adjust || 0, Math.hypot(mx, my)); this.replan(); s = this.segs[this.si]; this.refAt(s, this.st, ref);
            }
          }
        }
      }
    }
    if (this.mode === 'stand') { const m = this.mid(); ref[0] = m[0]; ref[1] = m[1]; this.r = m; }
    // reference CoM follows the reference DCM
    const xr = [ref[0], ref[1]];
    this.c[0] += om * (xr[0] - this.c[0]) * dt; this.c[1] += om * (xr[1] - this.c[1]) * dt;
    // stabilizer: shift the commanded CoM against the DCM error
    const e = [xm[0] - xr[0], xm[1] - xr[1]]; this.err = e; this.xiRef = xr;
    const kc = this.fb.kc, cc = [this.c[0] - kc * e[0], this.c[1] - kc * e[1]];
    // feet targets
    const feetT = { l: this.feet.l, r: this.feet.r }, z = { l: 0, r: 0 };
    if (this.swing && this.mode === 'walk' && this.segs[this.si] && this.segs[this.si].kind === 'ss') {
      const p = this.swing.step, f0 = this.swing.from, u = swingS, eh = smooth5((u - 0.08) / 0.84);
      feetT[p.s] = { x: f0.x + (p.x - f0.x) * eh, y: f0.y + (p.y - f0.y) * eh, yaw: f0.yaw + wrapPi(p.yaw - f0.yaw) * eh };
      z[p.s] = g.stepH * 0.5 * (1 - Math.cos(2 * Math.PI * clampN(u, 0, 1))) - (u > 0.9 ? g.land * (u - 0.9) * 10 : 0);
    }
    // pelvis yaw: average of the feet
    const fl = feetT.l, fr = feetT.r, yaw = Math.atan2(Math.sin(fl.yaw) + Math.sin(fr.yaw), Math.cos(fl.yaw) + Math.cos(fr.yaw)); this.yaw = yaw;
    // pelvis position so that the model CoM lands on the commanded CoM
    const co = this.cOff, cy = Math.cos(yaw), sy = Math.sin(yaw);
    // torso attitude feedback: lean the commanded pelvis against the measured tilt and tilt rate
    let rollC = 0, pitchC = this.pitchCmd;
    if (meas.pelvis && g.kAtt) {
      const T = meas.pelvis, pr = Math.atan2(T[9], T[10]), pp = Math.asin(clampN(-T[8], -1, 1)), w = meas.omega || [0, 0, 0];
      const wr = Math.cos(yaw) * w[0] + Math.sin(yaw) * w[1], wp = -Math.sin(yaw) * w[0] + Math.cos(yaw) * w[1];
      rollC = clampN(-g.kAtt[0] * pr - g.kAtt[1] * wr, -0.2, 0.2); pitchC += clampN(-g.kAtt[0] * pp - g.kAtt[1] * wp, -0.2, 0.2);
    }
    const P = K3.fromRPY(cc[0] - (cy * co[0] - sy * co[1]), cc[1] - (sy * co[0] + cy * co[1]), this.z - (this.mode === 'walk' ? g.dz : 0), rollC, pitchC, yaw);
    const Pi = K3.inv(P), q = this.q, Pm = meas.pelvis ? K3.inv(meas.pelvis) : Pi;
    for (const s of ['l', 'r']) {
      // the stance leg drives the body; a swinging foot is aimed in the world using the measured pelvis pose
      const f = feetT[s], Ts = K3.mul(z[s] > 0.003 && this.g.worldSwing !== false ? Pm : Pi, K3.fromRPY(f.x, f.y, z[s], 0, 0, f.yaw));
      this.ik[s].solve(this.lq[s], Ts, 2);
      this.legJ[s].forEach((n, i) => { q[n] = this.lq[s][i]; });
    }
    this.comLocal(q, this.cOff);
    this.support(feetT, z, yaw);
    // joint velocity targets (finite differences of the IK solution) so the servos' damping follows the motion
    const qd = this.qd || (this.qd = {}), qp = this.qp || (this.qp = {}), qa = this.qa || (this.qa = {});
    for (const s of ['l', 'r']) this.legJ[s].forEach(n => { const v = qp[n] != null ? (q[n] - qp[n]) / dt : 0, v0 = qd[n] != null ? qd[n] : v, nv = v0 + (v - v0) * 0.5; qa[n] = qa[n] != null ? qa[n] + ((nv - v0) / dt - qa[n]) * 0.3 : 0; qd[n] = nv; qp[n] = q[n]; });
    if (this.g.invDyn) this.invDyn();
    return q;
  }
  // inverse-dynamics feed-forward for the legs: tau += M(q) qdd_ref + bias(q, qd) (gravity, Coriolis), using the robot model
  invDyn() {
    const S = this.S, n = S.n, M = S.M, b = S.bias, ff = this.ff, qa = this.qa, k = this.g.idGain != null ? this.g.idGain : 1;
    const ds = this._ds || (this._ds = ['l', 'r'].flatMap(s => this.legJ[s].map(nm => [nm, 6 + S.ji[nm]])));
    for (const [nm, d] of ds) { let t = b[d]; for (const [nm2, e] of ds) t += M[d * n + e] * (qa[nm2] || 0); ff[nm] += k * t; }
  }
  // feed-forward torques: each stance leg carries its share of the body weight at the ZMP (tau = -J^T F)
  support(feetT, z, yaw) {
    const ff = this.ff || (this.ff = {}), mg = this.S.totalMass * PH_G * (this.g.ffGain != null ? this.g.ffGain : 1), r = this.r, l = feetT.l, rr = feetT.r;
    let wl;
    if (z.l > 0.002) wl = 0; else if (z.r > 0.002) wl = 1;
    else { const dx = l.x - rr.x, dy = l.y - rr.y, L2 = dx * dx + dy * dy || 1; wl = clampN(((r[0] - rr.x) * dx + (r[1] - rr.y) * dy) / L2, 0, 1); }
    const c = Math.cos(yaw), sn = Math.sin(yaw);
    for (const s of ['l', 'r']) {
      const w = s === 'l' ? wl : 1 - wl, f = feetT[s], F = w * mg, J = this.ik[s].jac(this.lq[s]);
      // moment of the vertical force about the sole origin, in the pelvis frame
      const ox = clampN(r[0] - f.x, -0.2, 0.2), oy = clampN(r[1] - f.y, -0.2, 0.2), mxw = oy * F, myw = -ox * F, mx = c * mxw + sn * myw, my = -sn * mxw + c * myw;
      this.legJ[s].forEach((n, k) => { ff[n] = w > 0 ? -(J[12 + k] * F + J[18 + k] * mx + J[24 + k] * my) : 0; });
    }
    return ff;
  }
  replan() {
    const si = this.si, st = this.st, first = this.segs[0], keep = this.r; this.r = first.ra;
    this.plan({ stance: this.steps[0].s === 'l' ? 'r' : 'l', Tds: first.T }); this.si = si; this.st = st; this.r = keep;
  }
  touchdown(i) {
    const p = this.steps[i];
    // plan on from where the feet really are (they can slip or land early)
    const fp = this.footPose(p.s), fo = p.s === 'l' ? 'r' : 'l'; this.landErr = Math.hypot(fp.x - p.x, fp.y - p.y);
    this.feet[p.s] = fp; this.feet[fo] = this.footPose(fo);
    this.lastSw = p.s; this.stepsDone++; this.swing = null;
    const zp = this.zmpOn(p.s === 'l' ? 'r' : 'l'), nsw = p.s === 'l' ? 'r' : 'l';
    const want = Math.abs(this.cmd[0]) + Math.abs(this.cmd[1]) + Math.abs(this.cmd[2]) > 1e-3 || Math.hypot(this.v[0], this.v[1]) > 0.01 || Math.abs(this.v[2]) > 0.05;
    this.steps = []; this.stopping = false;
    if (!want) {
      // stop once the feet are side by side and the robot is calm
      const t = this.nextStep(nsw, this.feet[p.s], [0, 0, 0]), f = this.feet[nsw], close = Math.hypot(t.x - f.x, t.y - f.y) < 0.008 && Math.abs(wrapPi(t.yaw - f.yaw)) < 0.05;
      if (close && Math.hypot(this.err[0], this.err[1]) < this.g.pushStep * 0.5) this.stopping = true;
      else { this.fillSteps(nsw); this.steps.length = Math.min(this.steps.length, 2); }
    } else this.fillSteps(nsw);
    this.r = zp; this.plan({ stance: p.s, Tds: this.g.Tds });
  }
}
