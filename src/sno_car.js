/* ============================== SnoBots: AgileX LIMO and LIMO COBOT ==============================
   A four-mode mobile base (4WD differential, Ackermann, tracked, mecanum) in a small city table, with a 2D
   lidar, a depth camera, occupancy-grid SLAM, an A* + pure-pursuit navigator and a lane follower.
   The COBOT adds a 6-axis arm (myCobot-280-style DH chain, 280 mm reach) and a parallel gripper. */
const LIMO_DIM = { L: 0.322, W: 0.22, wb: 0.2, track: 0.175, wr: 0.045, ww: 0.045, minR: 0.4 };
const LIMO_MODES = [['diff', '4WD diff', 'drive', 0], ['ackermann', 'Ackermann', 'wheel', 1], ['tracked', 'Tracks', 'layers', 0], ['mecanum', 'Mecanum', 'axes', 2]];

/* ---------- world: walls, boxes and round obstacles; raycasts, collisions and a static occupancy map ---------- */
class CarWorld {
  constructor(sp) {
    Object.assign(this, sp); this.res = 0.05;
    const segs = [[0, 0, this.W, 0], [this.W, 0, this.W, this.H], [this.W, this.H, 0, this.H], [0, this.H, 0, 0]];
    this.boxes.forEach(b => { if (b.h > 0.05) segs.push([b.x0, b.y0, b.x1, b.y0], [b.x1, b.y0, b.x1, b.y1], [b.x1, b.y1, b.x0, b.y1], [b.x0, b.y1, b.x0, b.y0]); });
    this.segs = segs;
    const gw = this.gw = Math.round(this.W / this.res), gh = this.gh = Math.round(this.H / this.res), occ = this.occ = new Uint8Array(gw * gh);
    for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) { const x = (i + 0.5) * this.res, y = (j + 0.5) * this.res; if (this.blockedAt(x, y, 0)) occ[j * gw + i] = 1; }
    this.infl = this.inflate(occ, 0.21);
  }
  blockedAt(x, y, m) {
    if (x < m || y < m || x > this.W - m || y > this.H - m) return true;
    for (const b of this.boxes) if (b.h > 0.05 && x > b.x0 - m && x < b.x1 + m && y > b.y0 - m && y < b.y1 + m) return true;
    for (const c of this.circles) if (Math.hypot(x - c.x, y - c.y) < c.r + m) return true;
    return false;
  }
  inflate(occ, rad) {
    const gw = this.gw, gh = this.gh, out = new Uint8Array(gw * gh), k = Math.ceil(rad / this.res), offs = [];
    for (let dj = -k; dj <= k; dj++) for (let di = -k; di <= k; di++) if ((di * di + dj * dj) * this.res * this.res <= rad * rad) offs.push([di, dj]);
    for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) { if (!occ[j * gw + i]) continue; for (const [di, dj] of offs) { const a = i + di, b = j + dj; if (a >= 0 && b >= 0 && a < gw && b < gh) out[b * gw + a] = 1; } }
    for (let i = 0; i < gw; i++) { out[i] = 1; out[(gh - 1) * gw + i] = 1; } for (let j = 0; j < gh; j++) { out[j * gw] = 1; out[j * gw + gw - 1] = 1; }
    return out;
  }
  raycast(ox, oy, a, maxR) {
    const dx = Math.cos(a), dy = Math.sin(a); let best = maxR;
    for (const [x1, y1, x2, y2] of this.segs) {
      const ex = x2 - x1, ey = y2 - y1, den = dx * ey - dy * ex; if (Math.abs(den) < 1e-12) continue;
      const t = ((x1 - ox) * ey - (y1 - oy) * ex) / den, u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
      if (t > 0 && t < best && u >= 0 && u <= 1) best = t;
    }
    for (const c of this.circles) {
      if (c.h < 0.12) continue;
      const fx = ox - c.x, fy = oy - c.y, b = fx * dx + fy * dy, cc = fx * fx + fy * fy - c.r * c.r, disc = b * b - cc; if (disc < 0) continue;
      const t = -b - Math.sqrt(disc); if (t > 0 && t < best) best = t;
    }
    return best >= maxR ? Infinity : best;
  }
  hitsOBB(cx, cy, th, hl, hw) {
    const c = Math.cos(th), s = Math.sin(th), ac = Math.abs(c), as = Math.abs(s);
    const ex = hl * ac + hw * as, ey = hl * as + hw * ac;
    if (cx - ex < 0 || cy - ey < 0 || cx + ex > this.W || cy + ey > this.H) return 'wall';
    for (const b of this.boxes) {
      if (b.h <= 0.05) continue;
      const bx = (b.x0 + b.x1) / 2, by = (b.y0 + b.y1) / 2, hx = (b.x1 - b.x0) / 2, hy = (b.y1 - b.y0) / 2, dx = bx - cx, dy = by - cy;
      if (Math.abs(dx) > hx + ex || Math.abs(dy) > hy + ey) continue;
      if (Math.abs(dx * c + dy * s) > hl + hx * ac + hy * as || Math.abs(-dx * s + dy * c) > hw + hx * as + hy * ac) continue;
      return b.name || 'box';
    }
    for (const ci of this.circles) {
      const dx = ci.x - cx, dy = ci.y - cy, u = dx * c + dy * s, v = -dx * s + dy * c, qu = clampN(u, -hl, hl), qv = clampN(v, -hw, hw);
      if ((u - qu) * (u - qu) + (v - qv) * (v - qv) < ci.r * ci.r) return ci.name || 'obstacle';
    }
    return null;
  }
  // A* on the inflated grid (8-connected), then line-of-sight shortcuts
  plan(sx, sy, gx, gy) {
    const gw = this.gw, gh = this.gh, R = this.res, G = this.infl;
    const cell = (x, y) => [clampN(Math.floor(x / R), 0, gw - 1), clampN(Math.floor(y / R), 0, gh - 1)];
    let [si, sj] = cell(sx, sy); const [gi, gj] = cell(gx, gy);
    if (G[gj * gw + gi]) return null;
    if (G[sj * gw + si]) { let best = null; for (let r = 1; r < 6 && !best; r++) for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) { const a = si + di, b = sj + dj; if (a >= 0 && b >= 0 && a < gw && b < gh && !G[b * gw + a]) { best = [a, b]; } } if (!best) return null; [si, sj] = best; }
    const N = gw * gh, g = new Float32Array(N).fill(Infinity), from = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
    const heap = [], push = (k, f) => { heap.push([f, k]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
    const hfn = (i, j) => { const dx = Math.abs(i - gi), dy = Math.abs(j - gj); return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy); };
    const s0 = sj * gw + si, goal = gj * gw + gi; g[s0] = 0; push(s0, hfn(si, sj));
    const D = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
    let found = false, it = 0;
    while (heap.length && it++ < 60000) {
      const [, k] = pop(); if (closed[k]) continue; closed[k] = 1; if (k === goal) { found = true; break; }
      const i = k % gw, j = (k - i) / gw;
      for (const [di, dj, c] of D) { const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= gw || b >= gh) continue; const kk = b * gw + a; if (G[kk] || closed[kk]) continue; if (di && dj && (G[j * gw + a] || G[b * gw + i])) continue; const ng = g[k] + c; if (ng < g[kk]) { g[kk] = ng; from[kk] = k; push(kk, ng + hfn(a, b)); } }
    }
    if (!found) return null;
    const cells = []; for (let k = goal; k >= 0; k = from[k]) { cells.push([(k % gw + 0.5) * R, (Math.floor(k / gw) + 0.5) * R]); if (k === s0) break; }
    cells.reverse(); cells[0] = [sx, sy]; cells[cells.length - 1] = [gx, gy];
    const clear = (a, b) => { const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / (R * 0.5)); for (let t = 1; t < n; t++) { const x = a[0] + (b[0] - a[0]) * t / n, y = a[1] + (b[1] - a[1]) * t / n, i = Math.floor(x / R), j = Math.floor(y / R); if (G[j * gw + i]) return false; } return true; };
    const out = [cells[0]]; let i = 0;
    while (i < cells.length - 1) { let k = cells.length - 1; while (k > i + 1 && !clear(cells[i], cells[k])) k--; out.push(cells[k]); i = k; }
    return out;
  }
}
/* ---------- occupancy-grid SLAM (log-odds, ray tracing) ---------- */
class OccGrid {
  constructor(W, H, res) { this.res = res; this.w = Math.round(W / res); this.h = Math.round(H / res); this.lo = new Float32Array(this.w * this.h); this.known = 0; }
  clear() { this.lo.fill(0); this.known = 0; }
  update(ox, oy, th0, ranges, inc, a0, maxR) {
    const w = this.w, h = this.h, R = this.res, lo = this.lo;
    const i0 = Math.floor(ox / R), j0 = Math.floor(oy / R);
    for (let k = 0; k < ranges.length; k += 2) {
      const r = ranges[k], hit = Number.isFinite(r), d = hit ? r : maxR * 0.6, a = th0 + a0 + k * inc;
      let i1 = Math.floor((ox + d * Math.cos(a)) / R), j1 = Math.floor((oy + d * Math.sin(a)) / R);
      let x = i0, y = j0; const dx = Math.abs(i1 - x), dy = -Math.abs(j1 - y), sx = x < i1 ? 1 : -1, sy = y < j1 ? 1 : -1; let err = dx + dy;
      for (let n = 0; n < 400; n++) {
        if (x === i1 && y === j1) break;
        if (x >= 0 && y >= 0 && x < w && y < h) { const c = y * w + x; lo[c] = Math.max(-4, lo[c] - 0.4); }
        const e2 = 2 * err; if (e2 >= dy) { err += dy; x += sx; } if (e2 <= dx) { err += dx; y += sy; }
      }
      if (hit && i1 >= 0 && j1 >= 0 && i1 < w && j1 < h) { const c = j1 * w + i1; lo[c] = Math.min(4, lo[c] + 1.2); }
    }
    let kn = 0; for (let c = 0; c < lo.length; c++) if (Math.abs(lo[c]) > 0.6) kn++; this.known = kn;
  }
  cell(i, j) { const v = this.lo[j * this.w + i]; return v > 0.6 ? 100 : v < -0.6 ? 0 : -1; }
}

/* ---------- LIMO description (URDF-like) ---------- */
function limoDesc(o = {}) {
  const J = [], V = {}, D = LIMO_DIM, add = (j, vis) => { J.push(j); if (vis) V[j.child] = vis; };
  V.base_link = [
    { s: 'box', size: [0.3, 0.125, 0.06], xyz: [0, 0, 0.02], m: 'dark' }, { s: 'box', size: [0.322, 0.205, 0.06], xyz: [0, 0, 0.08], m: 'navy' },
    { s: 'box', size: [0.29, 0.18, 0.012], xyz: [0, 0, 0.116], m: 'grey' }, { s: 'box', size: [0.2, 0.209, 0.008], xyz: [-0.02, 0, 0.06], m: 'accent' },
    { s: 'box', size: [0.026, 0.11, 0.03], xyz: [0.155, 0, 0.085], m: 'glass' }, { s: 'cyl', size: [0.008, 0.004], ax: 'x', xyz: [0.169, 0.03, 0.085], m: 'glow' }, { s: 'cyl', size: [0.008, 0.004], ax: 'x', xyz: [0.169, -0.03, 0.085], m: 'glow' },
    { s: 'box', size: [0.004, 0.12, 0.012], xyz: [-0.162, 0, 0.085], m: 'red' }];
  if (!o.cobot) V.base_link.push({ s: 'box', size: [0.07, 0.07, 0.04], xyz: [0.09, 0, 0.142], m: 'dark' }, { s: 'cyl', size: [0.035, 0.035], xyz: [0.09, 0, 0.18], m: 'servo' }, { s: 'cyl', size: [0.036, 0.006], xyz: [0.09, 0, 0.17], m: 'accent' });
  else V.base_link.push({ s: 'box', size: [0.05, 0.08, 0.03], xyz: [0.135, 0, 0.137], m: 'dark' }, { s: 'cyl', size: [0.03, 0.03], xyz: [0.135, 0, 0.167], m: 'servo' }, { s: 'cyl', size: [0.031, 0.006], xyz: [0.135, 0, 0.158], m: 'accent' });
  add({ name: 'base_joint', parent: 'base_footprint', child: 'base_link', type: 'fixed', xyz: [0, 0, D.wr] });
  add({ name: 'laser_joint', parent: 'base_link', child: 'laser_link', type: 'fixed', xyz: o.cobot ? [0.135, 0, 0.167] : [0.09, 0, 0.18] });
  add({ name: 'camera_joint', parent: 'base_link', child: 'camera_link', type: 'fixed', xyz: [0.17, 0, 0.085] });
  add({ name: 'camera_color_optical_joint', parent: 'camera_link', child: 'camera_color_optical_frame', type: 'fixed', rpy: [-Math.PI / 2, 0, -Math.PI / 2] });
  add({ name: 'imu_joint', parent: 'base_link', child: 'imu_link', type: 'fixed', xyz: [0, 0, 0.02] });
  for (const [fx, n] of [[1, 'front'], [-1, 'rear']]) for (const [sy, side] of [[1, 'left'], [-1, 'right']]) {
    const st = `${n}_${side}_steering`, wh = `${n}_${side}_wheel`;
    add({ name: st + '_joint', parent: 'base_link', child: st + '_link', xyz: [fx * D.wb / 2, sy * D.track / 2, 0], axis: 'z', lo: -0.6, hi: 0.6 });
    add({ name: wh + '_joint', parent: st + '_link', child: wh + '_link', xyz: [0, 0, 0], axis: 'y', type: 'continuous', lo: -1e9, hi: 1e9 }, [{ s: 'cyl', size: [D.wr, D.ww], ax: 'y', m: 'rubber', name: 'tire' }, { s: 'cyl', size: [0.026, D.ww + 0.003], ax: 'y', m: 'metal', name: 'hub' }, { s: 'box', size: [0.05, D.ww + 0.004, 0.008], m: 'grey', name: 'spoke' }]);
  }
  return { root: 'base_footprint', joints: J, visuals: V };
}
/* myCobot-280-style arm from its modified DH table (alpha_{i-1}, a_{i-1}, d_i, offset) */
const COBOT_DH = [[0, 0, 0.13122, 0], [Math.PI / 2, 0, 0, -Math.PI / 2], [0, -0.1104, 0, 0], [0, -0.096, 0.0634, -Math.PI / 2], [Math.PI / 2, 0, 0.07505, Math.PI / 2], [-Math.PI / 2, 0, 0.0456, 0]];
const COBOT_LIM = [168, 135, 150, 145, 165, 175];
const COBOT_J = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
function cobotDesc() {
  const d = limoDesc({ cobot: true }), J = d.joints, V = d.visuals;
  J.push({ name: 'arm_mount', parent: 'base_link', child: 'arm_base', type: 'fixed', xyz: [0.02, 0, 0.122] });
  V.arm_base = [{ s: 'cyl', size: [0.045, 0.05], xyz: [0, 0, 0.025], m: 'white' }, { s: 'cyl', size: [0.047, 0.012], xyz: [0, 0, 0.004], m: 'dark' }, { s: 'cap', size: [0.024, 0.08], xyz: [0, 0, 0.075], m: 'white' }];
  const links = ['arm_base'];
  COBOT_DH.forEach(([al, a, dd, off], i) => {
    const xyz = [a, -dd * Math.sin(al), dd * Math.cos(al)].map(v => Math.abs(v) < 1e-12 ? 0 : v), ch = 'link' + (i + 1);
    J.push({ name: 'joint' + (i + 1), parent: links[i], child: ch, xyz, rpy: [al, 0, 0], axis: 'z', offset: off, lo: -COBOT_LIM[i] * D2R, hi: COBOT_LIM[i] * D2R }); links.push(ch);
  });
  // auto visuals: motor ring on each joint axis + a capsule to the next joint
  for (let i = 1; i <= 6; i++) {
    const vis = V['link' + i] = [{ s: 'cyl', size: [0.026, 0.05], ax: 'z', m: i % 2 ? 'dark' : 'grey' }];
    const nx = J.find(j => j.name === 'joint' + (i + 1));
    if (nx) { const [x, y, z] = nx.xyz, L = Math.hypot(x, y, z); if (L > 1e-6) vis.push({ s: 'cap', size: [0.022, L], xyz: [x / 2, y / 2, z / 2], rpy: [0, Math.atan2(Math.hypot(x, y), z), Math.atan2(y, x)], m: 'white' }); }
  }
  V.link6.push({ s: 'cyl', size: [0.02, 0.012], xyz: [0, 0, 0.006], m: 'dark' });
  J.push({ name: 'gripper_mount', parent: 'link6', child: 'gripper_base', type: 'fixed', xyz: [0, 0, 0.012] });
  V.gripper_base = [{ s: 'box', size: [0.032, 0.066, 0.034], xyz: [0, 0, 0.017], m: 'dark' }, { s: 'box', size: [0.006, 0.05, 0.02], xyz: [0.018, 0, 0.017], m: 'accent' }];
  J.push({ name: 'gripper_left', parent: 'gripper_base', child: 'finger_left', type: 'prismatic', axis: 'y', xyz: [0, 0.004, 0.034], lo: 0, hi: 0.02 });
  J.push({ name: 'gripper_right', parent: 'gripper_base', child: 'finger_right', type: 'prismatic', axis: '-y', xyz: [0, -0.004, 0.034], lo: 0, hi: 0.02 });
  V.finger_left = [{ s: 'box', size: [0.014, 0.006, 0.048], xyz: [0, 0.003, 0.024], m: 'grey' }]; V.finger_right = [{ s: 'box', size: [0.014, 0.006, 0.048], xyz: [0, -0.003, 0.024], m: 'grey' }];
  J.push({ name: 'tcp_joint', parent: 'gripper_base', child: 'tcp', type: 'fixed', xyz: [0, 0, 0.068] });
  J.push({ name: 'arm_camera_joint', parent: 'gripper_base', child: 'arm_camera', type: 'fixed', xyz: [0.03, 0, 0.012] });
  V.gripper_base.push({ s: 'box', size: [0.018, 0.03, 0.02], xyz: [0.03, 0, 0.008], m: 'glass' });
  return d;
}

/* ---------- physics: masses, collision shapes and motors (four hub motors, steering servos, arm servos, gripper) ---------- */
const LIMO_WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right'];
function limoPhys(cobot) {
  const D = LIMO_DIM, coll = { base_footprint: false, laser_link: false, camera_link: false, camera_color_optical_frame: false, imu_link: false, base_link: [{ t: 'box', size: [0.322, 0.21, 0.11], xyz: [0, 0, 0.055], tag: 'chassis', mu: 0.5 }] };
  const masses = { base_link: [cobot ? 4.1 : 3.6, [0, 0, 0.05]] };
  LIMO_WHEELS.forEach(w => { coll[w + '_steering_link'] = false; coll[w + '_wheel_link'] = [{ t: 'wheel', size: [D.wr, D.ww / 2], rpy: [-Math.PI / 2, 0, 0], tag: 'wheel', mu: 0.9 }]; masses[w + '_wheel_link'] = [0.16, [0, 0, 0]]; masses[w + '_steering_link'] = [0.05, [0, 0, 0]]; });
  if (cobot) {
    Object.assign(masses, { arm_base: [0.16, [0, 0, 0.04]], link1: [0.12, [0, 0, 0]], link2: [0.12, [-0.055, 0, 0]], link3: [0.1, [-0.048, 0, 0]], link4: [0.08, [0, 0, 0]], link5: [0.06, [0, 0, 0]], link6: [0.05, [0, 0, 0]], gripper_base: [0.08, [0, 0, 0.017]], finger_left: [0.012, [0, 0.003, 0.024]], finger_right: [0.012, [0, -0.003, 0.024]] });
    Object.assign(coll, { tcp: false, arm_camera: false, link1: false, link6: false, arm_base: false,
      finger_left: [{ t: 'box', size: [0.014, 0.006, 0.048], xyz: [0, 0.003, 0.024], tag: 'finger', mu: 1.0 }], finger_right: [{ t: 'box', size: [0.014, 0.006, 0.048], xyz: [0, -0.003, 0.024], tag: 'finger', mu: 1.0 }] });
  }
  const armT = [3, 3, 2.5, 1.2, 1.2, 0.8];
  return {
    name: cobot ? 'cobot' : 'limo', masses, coll, mu: 0.9,
    act: (n) => /wheel/.test(n) ? { mode: 2, kd: 0.5, tmax: 1.6, wmax: 30, arm: 0.002, damp: 0.004 }
      : /steering/.test(n) ? { kp: 10, kd: 0.25, tmax: 2.5, wmax: 8, arm: 0.002, damp: 0.02 }
      : /gripper/.test(n) ? { kp: 600, kd: 6, tmax: 6, wmax: 0.2, arm: 0.02, damp: 1 }
      : { kp: 40, kd: 0.6, tmax: armT[+n.slice(5) - 1] || 1, wmax: 3.2, arm: 0.008, damp: 0.05, fric: 0.01 },
  };
}

/* ---------- the front camera's colour stream (sensor_msgs/Image) ---------- */
const LIMO_CAM = { w: 320, h: 240, vfov: 56, near: 0.02, far: 20 };

/* ---------- arenas ---------- */
const LIMO_ARENA = {
  W: 6, H: 4,
  boxes: [{ x0: 1.5, y0: 1.45, x1: 2.4, y1: 2.55, h: 0.45, m: 'navy', name: 'building' }, { x0: 2.7, y0: 2.2, x1: 3.1, y1: 2.6, h: 0.3, m: 'grey', name: 'kiosk' }],
  circles: [{ x: 3.0, y: 1.55, r: 0.07, h: 0.5, m: 'green', kind: 'tree', name: 'tree' }, { x: 5.75, y: 1.0, r: 0.055, h: 0.25, kind: 'cone', name: 'cone' }, { x: 5.75, y: 3.0, r: 0.055, h: 0.25, kind: 'cone', name: 'cone' }, { x: 0.25, y: 2.0, r: 0.055, h: 0.25, kind: 'cone', name: 'cone' }, { x: 3.0, y: 3.8, r: 0.055, h: 0.25, kind: 'cone', name: 'cone' }, { x: 3.47, y: 0.27, r: 0.025, h: 0.6, kind: 'pole', name: 'traffic light' }],
  road: { cx0: 1.5, cx1: 4.5, cy0: 1.4, cy1: 2.6, r: 0.6, w: 0.7, lane: 0.175 },
  light: { x: 3.47, y: 0.27, stop: 3.27 }, slots: [{ id: 'P1', x: 3.95, y: 1.58, yaw: 0, L: 0.5, W: 0.36 }, { id: 'P2', x: 3.95, y: 2.02, yaw: 0, L: 0.5, W: 0.36 }],
  start: [1.6, 0.625, 0],
  // ArUco markers (DICT_4X4_50, 12 cm black square) at camera height; yaw is the way the marker faces
  markers: [{ id: 7, x: 5.997, y: 2.0, yaw: Math.PI }, { id: 3, x: 1.5, y: 3.997, yaw: -Math.PI / 2 }, { id: 12, x: 5.0, y: 0.003, yaw: Math.PI / 2 }, { id: 21, x: 0.003, y: 3.0, yaw: 0 }, { id: 5, x: 2.9, y: 2.197, yaw: -Math.PI / 2 }],
  balls: [{ color: 'red', x: 2.55, y: 0.27 }, { color: 'magenta', x: 0.4, y: 0.35 }],
};
// ArUco DICT_4X4_50 codes: the 4 x 4 inner cells row by row, 1 = white (checked against OpenCV's dictionary)
const ARUCO_4X4 = { 3: 39238, 5: 31181, 7: 50418, 12: 3767, 21: 45099 };
const ARUCO_SIZE = 0.12, ARUCO_Z = 0.13, BALL_R = 0.05;
const COBOT_ARENA = {
  W: 5, H: 4,
  boxes: [{ x0: 1.75, y0: 3.55, x1: 2.75, y1: 3.95, h: 0.12, m: 'wood', name: 'pick table' }, { x0: 0.35, y0: 0.9, x1: 1.15, y1: 1.3, h: 0.6, m: 'grey', name: 'shelf' }, { x0: 4.52, y0: 1.2, x1: 4.68, y1: 1.36, h: 0.08, m: 'blue', name: 'blue bin', bin: 'blue' }, { x0: 4.52, y0: 1.92, x1: 4.68, y1: 2.08, h: 0.08, m: 'green', name: 'green bin', bin: 'green' }, { x0: 4.52, y0: 2.64, x1: 4.68, y1: 2.8, h: 0.08, m: 'red', name: 'red bin', bin: 'red' }],
  circles: [{ x: 3.2, y: 2.0, r: 0.12, h: 0.9, m: 'grey', kind: 'pillar', name: 'pillar' }],
  cubes: [['red', 1.95], ['green', 2.25], ['blue', 2.55]], tableY: 3.55, tableTop: 0.12, cubeY: 3.61, start: [1.2, 2.2, 0],
};
function laneLoop(rd, R) {
  const pts = [], st = 0.02, add = (x, y) => pts.push([x, y]);
  const line = (x0, y0, x1, y1) => { const n = Math.ceil(Math.hypot(x1 - x0, y1 - y0) / st); for (let i = 0; i < n; i++) add(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n); };
  const arc = (cx, cy, a0, a1) => { const n = Math.ceil(Math.abs(a1 - a0) * R / st); for (let i = 0; i < n; i++) { const a = a0 + (a1 - a0) * i / n; add(cx + R * Math.cos(a), cy + R * Math.sin(a)); } };
  line(rd.cx0, rd.cy0 - R, rd.cx1, rd.cy0 - R); arc(rd.cx1, rd.cy0, -Math.PI / 2, 0); line(rd.cx1 + R, rd.cy0, rd.cx1 + R, rd.cy1); arc(rd.cx1, rd.cy1, 0, Math.PI / 2);
  line(rd.cx1, rd.cy1 + R, rd.cx0, rd.cy1 + R); arc(rd.cx0, rd.cy1, Math.PI / 2, Math.PI); line(rd.cx0 - R, rd.cy1, rd.cx0 - R, rd.cy0); arc(rd.cx0, rd.cy0, Math.PI, Math.PI * 1.5);
  return pts;
}

/* ---------- LIMO simulator ---------- */
const tfMat = (x, y, z, q) => { const qx = +q.x || 0, qy = +q.y || 0, qz = +q.z || 0, qw = q.w == null ? 1 : +q.w; return K3.fromRPY(x, y, z, Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)), Math.asin(Math.max(-1, Math.min(1, 2 * (qw * qy - qz * qx)))), Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))); };
class LimoSim extends SnoSim {
  constructor(def) {
    super(def);
    this.A = def.arena; this.cw = new CarWorld(this.A); this.desc = def.desc(); this.kin = new KinRobot(this.desc);
    this.q = {}; this.kin.names.forEach(n => { this.q[n] = 0; });
    this.slam = new OccGrid(this.A.W, this.A.H, 0.05);
    this.lane = this.A.road ? laneLoop(this.A.road, this.A.road.r + this.A.road.lane) : null;
    this.ranges = new Array(360).fill(Infinity); this.mode = 'diff';
    this.beh = { slam: false, lane: false, nav: false };
    // physics: the robot, the arena walls and obstacles
    this.world = new PhysWorld({ h: 1 / 480, mu: 0.9 });
    this.rb = physFromKin(this.kin, limoPhys(!!def.cobot)); this.world.add(this.rb);
    this.wheels = LIMO_WHEELS.map(w => ({ w, k: this.rb.ji[w + '_wheel_joint'], s: this.rb.ji[w + '_steering_joint'], g: this.rb.geoms.find(g => g.link === w + '_wheel_link'), sb: this.rb.linkBody[w + '_steering_link'] }));
    this.gains = { kp: this.rb.kp.slice(), kd: this.rb.kd.slice() };
    this.buildStatics(); this.vizF = 0.25; this.vizSize = 0.35;
    // balls for the camera to find: real rigid bodies that LIMO can push and you can grab (Ctrl+drag)
    this.balls = (this.A.balls || []).map(b => { const sys = physSphere('ball_' + b.color, BALL_R, 0.12, { mu: 0.7 }); this.world.add(sys); return Object.assign({ sys }, b); });
    this.reset();
  }
  buildStatics() {
    const A = this.A, W = this.world, t = 0.05;
    W.addBox([A.W / 2, -t / 2, 0.15], [A.W + 2 * t, t, 0.3]); W.addBox([A.W / 2, A.H + t / 2, 0.15], [A.W + 2 * t, t, 0.3]); W.addBox([-t / 2, A.H / 2, 0.15], [t, A.H, 0.3]); W.addBox([A.W + t / 2, A.H / 2, 0.15], [t, A.H, 0.3]);
    A.boxes.forEach(b => {
      const cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2, sx = b.x1 - b.x0, sy = b.y1 - b.y0;
      if (b.bin) { const w = 0.008; [[0, sy / 2 - w / 2, sx, w], [0, -sy / 2 + w / 2, sx, w], [sx / 2 - w / 2, 0, w, sy], [-sx / 2 + w / 2, 0, w, sy]].forEach(([x, y, a, d]) => W.addBox([cx + x, cy + y, b.h / 2], [a, d, b.h], 0, { mu: 0.6 })); return; }
      if (b.h > 0.02) W.addBox([cx, cy, b.h / 2], [sx, sy, b.h], 0, { mu: 0.6 });
    });
    A.circles.forEach(c => W.addCyl(c.x, c.y, c.kind === 'tree' ? 0.03 : c.r, c.h, { mu: 0.5 }));
  }
  reset() {
    const [x, y, th] = this.A.start, rb = this.rb;
    for (const n in rb.ji) { const k = rb.ji[n]; rb.q[k] = 0; rb.qt[k] = 0; rb.vt[k] = 0; rb.tff[k] = 0; rb.kp[k] = this.gains.kp[k]; rb.kd[k] = this.gains.kd[k]; }
    rb.setYaw(x, y, 0.0004, th); rb.zeroVel(); this.world.cache.clear(); this.grab = null; if (this.wheels) this.wheels.forEach(W => { W.I = 0; });
    (this.balls || []).forEach(b => { b.sys.setYaw(b.x, b.y, BALL_R, 0); b.sys.zeroVel(); });
    Object.assign(this, { x, y, th, vx: 0, vy: 0, w: 0, steer: 0, bumps: 0, bumpT: -9, bumping: null, trail: [], otrail: [], wheelA: 0, cmdV: [0, 0, 0] });
    this.odom = { x, y, th, v: 0, vy: 0, w: 0 };
    this.st = { dist: 0, modes: {}, strafe: 0, cliDist: 0, userDist: 0, lap: 0, laneIdx: -1, vLine: 0, vLineIdx: -1, ids: new Set(), still: 0, vHz: false, vInfo: false };
    this.light = { t: 0, s: 'green' }; this.goal = null; this.path = null; this.navState = 'idle';
    this.slam.clear(); this.imuPrev = null; this.tf = null;
    this.applyWheelModel();
    this.resetExtra();
    this.readState(0);
  }
  teleport(x, y, th) { this.rb.setYaw(x, y, 0.0004, th); this.rb.zeroVel(); Object.assign(this.odom, { x, y, th }); this.trail = []; this.otrail = []; this.readState(0); }
  // the wheels' contact model follows the motion mode: rubber tyres, rubber tracks, or mecanum rollers (anisotropic friction)
  applyWheelModel() {
    const r2 = Math.SQRT1_2, dirs = { front_left: [r2, -r2, 0], front_right: [r2, r2, 0], rear_left: [r2, r2, 0], rear_right: [r2, -r2, 0] };
    this.wheels.forEach(W => { const g = W.g; if (this.mode === 'mecanum') { g.aniso = dirs[W.w]; g.anisoB = W.sb; g.mu2 = 0.03; g.mu = 0.9; } else { g.aniso = null; g.mu = this.mode === 'tracked' ? 1.15 : 0.9; } });
  }
  resetExtra() { }
  P0(k) { return this.P('/limo_base', k); }
  /* ---------- ROS ---------- */
  setupRos() {
    this.tn = {}; this.tfStatic = new Map(); const B = this.bus;
    // the TF tree is what is actually being published: static frames from /tf_static, moving links from the robot model
    B.onPublish = (node, topic, type, msg) => {
      if (topic === '/tf_static' && msg && msg.transforms) this.noteStatic(node, msg.transforms);
      if (topic === '/marker_id' && type === 'std_msgs/msg/Int32' && msg) this.markerSeen(Math.trunc(+msg.data), node);
    };
    B.onRemove = (node) => { this.tfStatic.delete(node); };
    B.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true;
    this.restoreRobot({});
    if (this.lane) { this.node('/lane_detector', 'vision', [['lookahead', 0.3, 'double', [0.1, 0.8]]]); B.advertise('/lane_detector', '/lane_error', 'std_msgs/msg/Float64'); B.advertise('/lane_detector', '/traffic_light', 'std_msgs/msg/String'); }
    this.setupExtra();
  }
  staticJoints() { return ['base_joint', 'laser_joint', 'camera_joint', 'camera_color_optical_joint', 'imu_joint'].concat(this.def.cobot ? ['arm_mount'] : []); }
  pkgs() { return Object.assign(super.pkgs(), { robot_state_publisher: ['robot_state_publisher'], limo_base: ['limo_base_node'], ydlidar_ros2_driver: ['ydlidar_ros2_driver_node'], orbbec_camera: ['orbbec_camera_node'], slam_toolbox: ['async_slam_toolbox_node'] }); }
  // The robot's bringup, the way its launch file starts it: description, lidar transform, base driver, lidar driver.
  // cfg {} is the normal robot; a Fix it ticket passes its fault: cfg.<proc> = false leaves that node out, an object
  // starts it with { params, remaps }, cfg.died names a node that crashes at start-up, cfg.slam adds SLAM, and
  // cfg.joy starts the joystick's teleop node publishing 'twist' or 'stamped' (TwistStamped) on /cmd_vel.
  restoreRobot(cfg = {}) {
    const B = this.bus, D = this.procDefs();
    Object.keys(this.procs).forEach(k => this.stopProc(k));
    ['nav', 'lane'].forEach(k => { if (this.beh[k]) this.setBeh(k, false); });
    [...B.nodes.keys()].filter(n => n === '/teleop_panel' || n === '/goal_tool' || /^\/(static_transform_publisher_|relay)/.test(n)).forEach(n => B.removeNode(n));
    this._tele = null; this.keys = {}; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; this.warnT = {}; this.lastWarn = ''; B.clashes.clear();
    const d0 = new Date(), p2 = (v) => String(v).padStart(2, '0'), day = `${d0.getFullYear()}-${p2(d0.getMonth() + 1)}-${p2(d0.getDate())}-${p2(d0.getHours())}-${p2(d0.getMinutes())}-${p2(d0.getSeconds())}`;
    const keys = ['rsp', 'laser_tf', 'base', 'lidar', 'camera'].concat(cfg.slam ? ['slam'] : []).filter(k => cfg[k] !== false || cfg.died === k);
    this.launchLabel = {};
    this.log('launch', 'INFO', `All log files can be found below /home/learner/.ros/log/${day}-${this.id}-sim-${B.pid}`, true);
    this.log('launch', 'INFO', 'Default logging verbosity is set to INFO', true);
    const run = keys.map((k, i) => { const d = D[k], label = `${d.exe}-${i + 1}`, pid = ++B.pid; this.launchLabel[d.node] = label; this.log(label, 'INFO', `process started with pid [${pid}]`, true); return [k, d, label, pid]; });
    run.forEach(([k, d, label, pid]) => {
      if (cfg[k] !== false) { const c = cfg[k]; this.startProc(k, c && typeof c === 'object' ? c : {}); return; }
      if (d.crash) this.log(label, 'ERROR', d.crash, 'out');
      this.log(label, 'ERROR', `process has died [pid ${pid}, exit code 1, cmd '/opt/ros/jazzy/lib/${d.pkg}/${d.exe} --ros-args -r __node:=${d.node.slice(1)} --params-file /tmp/launch_params_${Math.random().toString(36).slice(2, 10)}'].`, true);
    });
    if (cfg.joy) { const st = cfg.joy === 'stamped'; this.node('/teleop_panel', 'teleop', [['publish_stamped_twist', st, 'bool']]); B.advertise('/teleop_panel', '/cmd_vel', st ? 'geometry_msgs/msg/TwistStamped' : 'geometry_msgs/msg/Twist'); }
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  /* ---------- the robot's own nodes are processes: they can crash, be restarted, remapped and given parameters ---------- */
  procDefs() {
    if (this._pd) return this._pd;
    const B = this.bus, TF = 'tf2_msgs/msg/TFMessage', LS = 'sensor_msgs/msg/LaserScan', latched = { dur: 'TRANSIENT_LOCAL', depth: 1 };
    const qosOk = (k, v) => k.startsWith('qos_overrides.') && !['reliable', 'best_effort', 'system_default'].includes(String(v)) ? `${k}: '${v}' is not a reliability (use reliable or best_effort)` : '';
    const statics = (node, joints) => { B.topic('/tf_static', TF).latched = true; B.advertise(node, '/tf_static', TF, latched); this.pub(node, '/tf_static', TF, this.tfMsg(joints.map(n => { const j = this.kin.J[n]; return [j.parent, j.child, j.O]; }))); };
    return (this._pd = {
      rsp: { pkg: 'robot_state_publisher', exe: 'robot_state_publisher', node: '/robot_state_publisher', kind: 'description', params: () => [['publish_frequency', 20.0, 'double', [1, 100]]],
        start: () => { const n = '/robot_state_publisher'; B.advertise(n, '/tf', TF); B.subscribe(n, '/joint_states', 'sensor_msgs/msg/JointState', () => {}); statics(n, this.staticJoints().filter(j => j !== 'laser_joint')); this.kin.links.slice(0, 4).forEach(l => this.log(n, 'INFO', `got segment ${l}`)); } },
      laser_tf: { pkg: null, exe: 'static_transform_publisher', node: '/base_link_to_laser_link', kind: 'tf', params: () => [],
        start: () => statics('/base_link_to_laser_link', ['laser_joint']) },
      base: { pkg: 'limo_base', exe: 'limo_base_node', node: '/limo_base', kind: 'sim',
        params: () => [['max_linear_speed', 1.0, 'double', [0.05, 1.0]], ['max_angular_speed', 2.5, 'double', [0.1, 4]], ['cmd_timeout', 0.6, 'double', [0.1, 5]], ['motion_mode', this.mode || 'diff', 'string']],
        check: (k, v) => k === 'motion_mode' && !LIMO_MODES.some(x => x[0] === String(v).toLowerCase()) ? 'motion_mode must be diff, ackermann, tracked or mecanum' : ['max_linear_speed', 'max_angular_speed', 'cmd_timeout'].includes(k) && (v <= 0 || v > 5) ? `${k} is out of range` : '',
        start: (rec) => {
          const c = '/limo_base', R = rec.R; this.tn.cmd = R('cmd_vel'); this.tn.odom = R('odom'); this.tn.imu = R('imu'); this.tn.status = R('limo_status');
          rec.un.push(this.cmdVelSub(c, this.tn.cmd));
          [[this.tn.odom, 'nav_msgs/msg/Odometry'], [this.tn.imu, 'sensor_msgs/msg/Imu'], [this.tn.status, 'limo_msgs/msg/LimoStatus'], ['/tf', TF]].forEach(([t, ty]) => B.advertise(c, t, ty));
          B.service('/reset_world', 'std_srvs/srv/Empty', c, () => { this.reset(); this.event('service', '/reset_world'); return {}; });
          const mm = String(this.P(c, 'motion_mode')).toLowerCase(); if (mm !== this.mode && LIMO_MODES.some(x => x[0] === mm)) this.setMode(mm, true);
          this.log(c, 'INFO', `LIMO base ready, listening on ${this.tn.cmd} (motion mode: ${this.mode})`);
        },
        stop: () => { this.tn.cmd = null; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; } },
      lidar: { pkg: 'ydlidar_ros2_driver', exe: 'ydlidar_ros2_driver_node', node: '/ydlidar_node', kind: 'driver',
        // frame_id and the QoS override are read once at start-up, so they are read-only here: restart the driver to change them
        params: () => [['range_max', 8.0, 'double', [1, 12]], ['frequency', 10.0, 'double', [5, 12], true], ['frame_id', 'laser_link', 'string', null, true], ['qos_overrides./scan.publisher.reliability', 'reliable', 'string', null, true]],
        check: qosOk, crash: '[YDLIDAR]:Error, cannot bind to the specified serial port[/dev/ydlidar] and baudrate[115200]',
        start: (rec) => {
          const n = '/ydlidar_node', be = this.P(n, 'qos_overrides./scan.publisher.reliability') === 'best_effort'; this.tn.scan = rec.R('scan');
          B.advertise(n, this.tn.scan, LS, { rel: be ? 'BEST_EFFORT' : 'RELIABLE', depth: be ? 5 : 10 });
          this.log(n, 'INFO', '[YDLIDAR] Lidar running correctly! The health status: good'); this.log(n, 'INFO', `Now YDLIDAR is scanning: ${this.tn.scan} in frame '${this.P(n, 'frame_id')}', 10 Hz`);
        },
        stop: () => { this.tn.scan = null; this.ranges.fill(Infinity); } },
      // the front RGB-D camera (colour stream). Images are rendered only while something subscribes, like a lazy driver
      camera: { pkg: 'orbbec_camera', exe: 'orbbec_camera_node', node: '/camera/camera', kind: 'driver',
        params: () => [['camera_name', 'camera', 'string', null, true], ['color_width', LIMO_CAM.w, 'integer', null, true], ['color_height', LIMO_CAM.h, 'integer', null, true], ['color_fps', 10, 'integer', [1, 30]], ['enable_color', true, 'bool']],
        start: (rec) => {
          const n = '/camera/camera'; this.tn.img = rec.R('/camera/color/image_raw'); this.tn.info = rec.R('/camera/color/camera_info'); this.camLast = null; this.camDue = null;
          B.advertise(n, this.tn.img, 'sensor_msgs/msg/Image'); B.advertise(n, this.tn.info, 'sensor_msgs/msg/CameraInfo');
          this.log(n, 'INFO', `color: ${LIMO_CAM.w}x${LIMO_CAM.h} rgb8 @ ${this.P(n, 'color_fps')} fps on ${this.tn.img}`);
        },
        stop: () => { this.tn.img = null; this.tn.info = null; this.camDue = null; } },
      slam: { pkg: 'slam_toolbox', exe: 'async_slam_toolbox_node', node: '/slam_toolbox', kind: 'slam',
        params: () => [['resolution', 0.05, 'double', [0.05, 0.05], true], ['map_update_interval', 1.0, 'double', [0.2, 5]], ['qos_overrides./scan.subscription.reliability', 'reliable', 'string', null, true]],
        check: qosOk,
        start: (rec) => {
          const n = '/slam_toolbox', R = rec.R, be = this.P(n, 'qos_overrides./scan.subscription.reliability') === 'best_effort'; this.tn.map = R('map');
          B.advertise(n, this.tn.map, 'nav_msgs/msg/OccupancyGrid', latched); B.advertise(n, '/tf', TF);
          rec.un.push(B.subscribe(n, R('scan'), LS, (m) => this.onSlamScan(m), { rel: be ? 'BEST_EFFORT' : 'RELIABLE', depth: 5 }), B.subscribe(n, R('odom'), 'nav_msgs/msg/Odometry', () => {}), B.subscribe(n, '/tf', TF, () => {}), B.subscribe(n, '/tf_static', TF, () => {}, latched));
          this.beh.slam = true; this.log(n, 'INFO', 'Using solver plugin solver_plugins::CeresSolver'); this.log(n, 'INFO', 'CeresSolver: Using SCHUR_JACOBI preconditioner.');
        },
        stop: () => { this.beh.slam = false; } },
    });
  }
  startProc(key, o = {}) {
    const d = this.procDefs()[key]; if (!d) return;
    if (this.procs[key]) this.stopProc(key);
    const rec = this.procs[key] = { key, R: this.remapper(o.remaps), remaps: o.remaps || [], un: [], T: o.T || null };
    if (o.T) this.procTerm[d.node] = o.T; else delete this.procTerm[d.node];
    const given = o.params || {};
    this.node(d.node, d.kind, d.params().map(([k, v, t, range, ro]) => [k, k in given ? given[k] : v, t, range, ro]));
    d.start(rec);
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  stopProc(key) {
    const d = this.procDefs()[key], rec = this.procs[key]; if (!d || !rec) return;
    rec.un.forEach(f => { try { f(); } catch (e) { /* already gone */ } });
    delete this.procs[key]; delete this.procTerm[d.node];
    if (d.stop) d.stop(rec);
    this.bus.removeNode(d.node);
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  /* ---------- TF: frames exist only while someone publishes them ---------- */
  noteStatic(node, list) {
    let m = this.tfStatic.get(node); if (!m) this.tfStatic.set(node, m = new Map());
    list.forEach(t => { const tr = t.transform || {}, p = tr.translation || {}, q = tr.rotation || {}; m.set(t.child_frame_id, { parent: (t.header || {}).frame_id, T: tfMat(+p.x || 0, +p.y || 0, +p.z || 0, q) }); });
  }
  staticOf(f) { let hit = null; this.tfStatic.forEach(m => { if (m.has(f)) hit = m.get(f); }); return hit; }
  // pose of a frame in odom, following the published tree; null when the chain is broken
  poseOf(f, d = 0) {
    if (!f || d > 16) return null;
    const base = !!this.procs.base, root = this.kin.root;
    if (f === 'odom') return base ? K3.I() : null;
    if (f === 'map') return base && (this.beh.slam || this.beh.nav) ? K3.inv(this.mapToOdom()) : null;
    if (f === root) { if (!base) return null; const O = this.odom; return K3.fromRPY(O.x, O.y, 0, 0, 0, O.th); }
    const st = this.staticOf(f); if (st) { const P = this.poseOf(st.parent, d + 1); return P ? K3.mul(P, st.T) : null; }
    this._sk = this._sk || new Set(this.staticJoints().map(n => this.kin.J[n].child));
    if (this.kin.T[f] && !this._sk.has(f) && this.procs.rsp) { const R0 = this.poseOf(root, d + 1); return R0 ? K3.mul(R0, K3.mul(K3.inv(this.kin.T[root]), this.kin.T[f])) : null; }
    return null;
  }
  warnEvery(key, sec, node, text) { const t = (this.warnT = this.warnT || {}); if (t[key] != null && this.t - t[key] < sec) return; t[key] = this.t; this.log(node, 'WARN', text); }
  setupExtra() { }
  onParam(node, k, v) {
    if (node === '/limo_base' && k === 'motion_mode') { const m = String(v).toLowerCase(), ok = LIMO_MODES.some(x => x[0] === m) || m === 'mcnamu'; if (!ok) return { ok: false, reason: 'motion_mode must be diff, ackermann, tracked or mecanum' }; this.setMode(m === 'mcnamu' ? 'mecanum' : m, true); this.event('param', { node, k, v: m }); return { ok: true }; }
    this.event('param', { node, k, v }); return { ok: true };
  }
  setMode(m, fromParam) {
    if (!LIMO_MODES.some(x => x[0] === m)) return;
    this.mode = m; this.steer = 0; this.applyWheelModel(); if (!fromParam) this.setP('/limo_base', 'motion_mode', m);
    else this.setP('/limo_base', 'motion_mode', m);
    this.event('mode', m); SnoUI.refreshTab('control'); if (this.view3) this.applyModeVisuals();
  }
  behaviors() {
    const b = {
      'slam.launch.py': { label: 'slam_toolbox', nodes: ['/slam_toolbox'], on: () => this.beh.slam, set: (on) => this.setBeh('slam', on), hello: '[slam_toolbox-1] [INFO] Using solver plugin solver_plugins::CeresSolver (sim: log-odds grid, 5 cm cells)' },
      'navigation.launch.py': { label: 'navigation', nodes: ['/navigator'], on: () => this.beh.nav, set: (on) => this.setBeh('nav', on), hello: '[navigator-1] [INFO] map loaded (static map of the arena). Send a goal: click the floor or the map, or publish /goal_pose' },
    };
    if (this.lane) b['lane_follow.launch.py'] = { label: 'lane follower', nodes: ['/lane_follower'], on: () => this.beh.lane, set: (on) => this.setBeh('lane', on), hello: '[lane_follower-1] [INFO] following /lane_error, stopping for red on /traffic_light' };
    return b;
  }
  setBeh(k, on) {
    if (this.beh[k] === on) return; const B = this.bus;
    if (k === 'slam') { if (on) this.startProc('slam'); else this.stopProc('slam'); this.event('behavior', { k, on }); SnoUI.refreshTab('control'); return; }
    this.beh[k] = on;
    if (k === 'lane') {
      if (on) { this.node('/lane_follower', 'behavior', [['speed', 0.3, 'double', [0.05, 0.8]], ['kp', 6.0, 'double', [0, 20]]]); B.advertise('/lane_follower', '/cmd_vel', 'geometry_msgs/msg/Twist'); this.laneIn = { err: null, t: -9, light: 'none' }; this.unLane = [B.subscribe('/lane_follower', '/lane_error', 'std_msgs/msg/Float64', (m) => { this.laneIn.err = +m.data; this.laneIn.t = this.t; }), B.subscribe('/lane_follower', '/traffic_light', 'std_msgs/msg/String', (m) => { this.laneIn.light = String(m.data); })]; if (this.beh.nav) this.setBeh('nav', false); }
      else { (this.unLane || []).forEach(u => u()); B.removeNode('/lane_follower'); this.teleop(0, 0, 0); }
    }
    if (k === 'nav') {
      if (on) { this.node('/navigator', 'behavior', [['max_vel', 0.4, 'double', [0.05, 1.0]], ['lookahead', 0.3, 'double', [0.1, 1]], ['goal_tolerance', 0.05, 'double', [0.01, 0.5]]]); B.advertise('/navigator', '/cmd_vel', 'geometry_msgs/msg/Twist'); B.advertise('/navigator', '/plan', 'nav_msgs/msg/Path'); this.unNav = [B.subscribe('/navigator', '/goal_pose', 'geometry_msgs/msg/PoseStamped', (m) => this.onGoal(m)), B.subscribe('/navigator', '/odom', 'nav_msgs/msg/Odometry', () => {}), B.subscribe('/navigator', '/scan', 'sensor_msgs/msg/LaserScan', () => {})]; if (this.beh.lane) this.setBeh('lane', false); }
      else { (this.unNav || []).forEach(u => u()); B.removeNode('/navigator'); this.goal = null; this.path = null; this.navState = 'idle'; }
    }
    this.event('behavior', { k, on }); SnoUI.refreshTab('control');
  }
  sendGoal(x, y, yaw) {
    if (!this.beh.nav) this.setBeh('nav', true);
    if (!this.bus.nodes.has('/goal_tool')) { this.bus.addNode('/goal_tool', 'tool'); this.bus.advertise('/goal_tool', '/goal_pose', 'geometry_msgs/msg/PoseStamped'); }
    const yw = yaw == null ? Math.atan2(y - this.y, x - this.x) : yaw;
    this.pub('/goal_tool', '/goal_pose', 'geometry_msgs/msg/PoseStamped', { header: { stamp: stampNow(), frame_id: 'map' }, pose: { position: { x: +x.toFixed(3), y: +y.toFixed(3), z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yw / 2), w: Math.cos(yw / 2) } } });
  }
  onGoal(m) {
    const p = m.pose.position, o = m.pose.orientation, yaw = 2 * Math.atan2(o.z || 0, o.w == null ? 1 : o.w);
    this.goal = { x: +p.x, y: +p.y, yaw, exact: !!this.exactGoal }; this.exactGoal = false; this.docking = false; this.dockPh = null; this.escape = null;
    if (this.goal.exact) { this.goal.px = this.goal.x - Math.cos(yaw) * 0.3; this.goal.py = this.goal.y - Math.sin(yaw) * 0.3; }
    this.replan(true);
  }
  tooClose() { const D = LIMO_DIM; return [0.35, -0.35, 0.8, -0.8].some(a => this.cw.hitsOBB(this.x, this.y, this.th + a, D.L / 2 + 0.01, D.W / 2 + 0.01)); }
  replan(announce) {
    const g = this.goal; if (!g) return;
    if (this.tooClose() && !this.escape) { this.escape = { t: 0, dir: this.cw.hitsOBB(this.x - Math.cos(this.th) * 0.08, this.y - Math.sin(this.th) * 0.08, this.th, LIMO_DIM.L / 2, LIMO_DIM.W / 2) ? 1 : -1 }; this.navState = 'backing_up'; this.lastPlanT = this.t; return; }
    const path = this.cw.plan(this.x, this.y, g.exact ? g.px : g.x, g.exact ? g.py : g.y);
    this.lastPlanT = this.t;
    if (!path) { this.navState = 'no_path'; this.path = null; if (announce) this.lastWarn = 'No path: the goal is inside or too close to an obstacle.'; this.event('nav_fail'); return; }
    this.path = path; this.pi = 0; this.navState = 'driving'; this.lastWarn = '';
    this.pub('/navigator', '/plan', 'nav_msgs/msg/Path', { header: { stamp: stampNow(), frame_id: 'map' }, poses: path.map(([x, y]) => ({ header: { stamp: stampNow(), frame_id: 'map' }, pose: { position: { x: +x.toFixed(3), y: +y.toFixed(3), z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } })) });
  }
  /* ---------- per-step: the base driver turns /cmd_vel into wheel speeds; physics does the rest ---------- */
  step(dt) {
    this.t += dt; this.teleopTick();
    this.behTick(dt);
    const n = 4, h = dt / n, t0 = typeof performance !== 'undefined' ? performance.now() : 0;
    this.world.h = h / 2;
    for (let i = 0; i < n; i++) { this.baseCtl(h); this.stepExtra(h); this.ballCtl(); if (this.grab) this.grabCtl(); this.world.step(); this.world.step(); }
    this.phys = this.phys || { ms: 0 }; if (t0) this.phys.ms = this.phys.ms * 0.9 + (performance.now() - t0) * 0.1;
    this.readState(dt);
    this.lightStep(dt);
    this.publish(dt);
    this.camTick();
    this.missionTick(dt);
    if (this.fix) SnoFix.tick(this, dt);
  }
  /* ---------- the camera: camera_info on the sim clock, images rendered by the 3D view when one is due ---------- */
  camInfo(hd) {
    const C = LIMO_CAM, f = +((C.h / 2) / Math.tan(C.vfov * Math.PI / 360)).toFixed(4), cx = (C.w - 1) / 2, cy = (C.h - 1) / 2;
    return { header: hd, height: C.h, width: C.w, distortion_model: 'plumb_bob', d: [0, 0, 0, 0, 0], k: [f, 0, cx, 0, f, cy, 0, 0, 1], r: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0], binning_x: 0, binning_y: 0, roi: { x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false } };
  }
  camTick() {
    if (!this.procs.camera || !this.tn.info || this.P('/camera/camera', 'enable_color') === false) return;
    const fps = clampN(+this.P('/camera/camera', 'color_fps') || 10, 1, 30);
    if (this.camLast != null && this.t >= this.camLast && this.t - this.camLast < 1 / fps - 1e-6) return;
    this.camLast = this.t;
    const hd = { stamp: stampNow(), frame_id: 'camera_color_optical_frame' };
    this.pub('/camera/camera', this.tn.info, 'sensor_msgs/msg/CameraInfo', this.camInfo(hd));
    this.camDue = hd;
  }
  cameraFrame(view) {
    const hd = this.camDue; if (!hd) return; this.camDue = null;
    const T = this.tn.img && this.bus.topics.get(this.tn.img); if (!T || !T.subs.size || !this.camCap) return;
    const data = view.capture(this.camCap); if (!data) return;
    this.pub('/camera/camera', this.tn.img, 'sensor_msgs/msg/Image', { header: hd, height: LIMO_CAM.h, width: LIMO_CAM.w, encoding: 'rgb8', is_bigendian: 0, step: LIMO_CAM.w * 3, data });
  }
  baseCtl(h) {
    const D = LIMO_DIM, up = !!this.procs.base, live = up && this.cmdLive(this.P0('cmd_timeout')), vmax = up ? this.P0('max_linear_speed') : 0, wmax0 = up ? this.P0('max_angular_speed') : 0, m = this.mode;
    const tvx = live ? clampN(this.cmd.vx, -vmax, vmax) : 0, tvy = live && m === 'mecanum' ? clampN(this.cmd.vy, -vmax * 0.7, vmax * 0.7) : 0, wmax = m === 'tracked' ? Math.min(wmax0, 1.8) : wmax0, twz = live ? clampN(this.cmd.wz, -wmax, wmax) : 0;
    const c = this.cmdV; c[0] = approach(c[0], tvx, 1.6 * h); c[1] = approach(c[1], tvy, 1.6 * h); c[2] = approach(c[2], twz, 7 * h);
    const rb = this.rb, r = D.wr, a = D.wb / 2, b = D.track / 2, pts = [[a, b], [a, -b], [-a, b], [-a, -b]];
    let wv, st = [0, 0, 0, 0];
    if (m === 'ackermann') {
      const dmax = Math.atan(D.wb / D.minR), want = Math.abs(tvx) > 0.02 ? Math.atan(twz * D.wb / tvx) : (Math.abs(twz) > 0.05 ? Math.sign(twz) * dmax * Math.sign(tvx || 1) : this.steer);
      this.steer = approach(this.steer, clampN(want, -dmax, dmax), 2.5 * h);
      const v = c[0], om = v * Math.tan(this.steer) / D.wb;
      // every wheel rolls about the instantaneous centre of rotation (on the rear axle line)
      wv = pts.map(([x, y]) => { const ux = v - om * y, uy = om * (x + a); return (x > 0 ? Math.sign(ux || 1) * Math.hypot(ux, uy) : ux) / r; });
      st = pts.map(([x, y]) => { if (x < 0) return 0; const ux = v - om * y, uy = om * (x + a); if (Math.abs(ux) < 1e-6 && Math.abs(uy) < 1e-6) return this.steer; let d = Math.atan2(uy, ux); if (d > Math.PI / 2) d -= Math.PI; if (d < -Math.PI / 2) d += Math.PI; return d; });
    } else if (m === 'mecanum') {
      this.steer = 0; const k = a + b; wv = [c[0] - c[1] - c[2] * k, c[0] + c[1] + c[2] * k, c[0] + c[1] - c[2] * k, c[0] - c[1] + c[2] * k].map(x => x / r);
    } else {
      this.steer = 0; const chi = this.skidK(), vl = c[0] - c[2] * b * chi, vr = c[0] + c[2] * b * chi; wv = [vl, vr, vl, vr].map(x => x / r);
    }
    // motor drivers run a PI speed loop: the integral term pushes through static friction at low speed
    this.wheels.forEach((W, i) => { rb.mode[W.k] = 2; rb.vt[W.k] = wv[i]; rb.qt[W.s] = st[i]; rb.mode[W.s] = 1; W.I = clampN((W.I || 0) + (wv[i] - rb.v[6 + W.k]) * h, -0.4, 0.4); if (Math.abs(wv[i]) < 1e-3 && Math.abs(rb.v[6 + W.k]) < 0.05) W.I *= 0.9; rb.tff[W.k] = 3 * W.I; });
  }
  // skid steering slips: the driver scales the wheel speed difference (effective track width factor)
  skidK() { return this.mode === 'tracked' ? 2.6 : 2.3; }
  stepExtra() { }
  // rolling resistance and a little drag, so a pushed ball slows down and stops like a real one
  ballCtl() {
    for (const b of this.balls || []) {
      const S = b.sys; if (S.asleep) continue;
      const T = S.T[0], v = S.ptVel(0, [T[3], T[7], T[11]]), sp = Math.hypot(v[0], v[1]); if (sp < 1e-4) continue;
      const m = S.totalMass, k = (0.04 * m * 9.81) / sp + 0.4 * m;
      S.push(0, [0, 0, 0], [-k * v[0], -k * v[1], 0]);
    }
  }
  ballPos(color) { const b = (this.balls || []).find(x => x.color === color); if (!b) return null; const T = b.sys.T[0]; return [T[3], T[7], T[11]]; }
  fk() { this.kin.fk(this.q, this.Tb); }
  readState(dt) {
    const rb = this.rb, T = rb.T[0], D = LIMO_DIM;
    for (const n in rb.ji) this.q[n] = rb.q[rb.ji[n]];
    this.Tb = new Float64Array(T); this.kin.fk(this.q, this.Tb);
    const ox = this.x, oy = this.y, oth = this.th;
    this.x = T[3]; this.y = T[7]; this.th = Math.atan2(T[4], T[0]);
    const c = Math.cos(this.th), s = Math.sin(this.th);
    this.vx = c * rb.v[3] + s * rb.v[4]; this.vy = -s * rb.v[3] + c * rb.v[4]; this.w = rb.v[2];
    // wheel odometry from the motor encoders (it drifts when the wheels slip)
    const om = this.wheels.map(W => rb.v[6 + W.k]), r = D.wr, a = D.wb / 2, b = D.track / 2, nz = this.noise ? this.noise.odom : 0, O = this.odom;
    let v, vy = 0, w;
    if (this.mode === 'mecanum') { v = r * (om[0] + om[1] + om[2] + om[3]) / 4; vy = r * (-om[0] + om[1] + om[2] - om[3]) / 4; w = r * (-om[0] + om[1] - om[2] + om[3]) / (4 * (a + b)); }
    else if (this.mode === 'ackermann') { v = r * (om[2] + om[3]) / 2; w = v * Math.tan(this.steer) / D.wb; vy = w * a; }   // bicycle model about the rear axle; base_footprint sits a = wb/2 ahead of it, so it also slides sideways
    else { const vl = r * (om[0] + om[2]) / 2, vr = r * (om[1] + om[3]) / 2; v = (vl + vr) / 2; w = rb.v[2] + (this.gyroBias || 0); }   // skid steering: heading from the IMU gyro, as drivers usually fuse it
    if (nz) { v *= 1 + (Math.random() - 0.5) * 0.04 * nz; w += (Math.random() - 0.5) * 0.05 * nz + 0.004 * nz; }
    if (dt > 0) { const mth = O.th + w * dt / 2; O.x += (Math.cos(mth) * v - Math.sin(mth) * vy) * dt; O.y += (Math.sin(mth) * v + Math.cos(mth) * vy) * dt; O.th = wrapPi(O.th + w * dt); }
    O.v = v; O.vy = vy; O.w = w;
    this.wheelA = (this.q.rear_left_wheel_joint + this.q.rear_right_wheel_joint) / 2 || 0;
    // bumps: the chassis touching a wall or an obstacle
    let hit = null; for (let i = 0; i < this.world.nc; i++) { const k = this.world.C[i]; if (k.a.s === rb && k.a.tag === 'chassis' && k.b.static && k.fn > 0.3) { hit = 'obstacle'; break; } }
    this.bumping = hit; if (hit && this.t - this.bumpT > 0.7) { this.bumps++; this.bumpT = this.t; Sound.play('bump'); this.event('bump', hit); }
    if (dt > 0) {
      const d = Math.hypot(this.x - ox, this.y - oy), live = this.cmdLive(this.P0('cmd_timeout'));
      if (d > 0 && d < 0.5) {
        this.st.dist += d; this.st.modes[this.mode] = (this.st.modes[this.mode] || 0) + d;
        if (this.mode === 'mecanum') this.st.strafe += Math.abs(-Math.sin(oth) * (this.x - ox) + Math.cos(oth) * (this.y - oy));
        if (live && this.cmd.kind === 'cli') this.st.cliDist += d; if (live && this.cmd.kind === 'user') this.st.userDist += d;
        const tl = this.trail[this.trail.length - 1]; if (!tl || Math.hypot(this.x - tl[0], this.y - tl[1]) > 0.04) { this.trail.push([this.x, this.y]); if (this.trail.length > 600) this.trail.shift(); this.otrail.push([O.x, O.y]); if (this.otrail.length > 600) this.otrail.shift(); }
      }
      if (this.every('pbuf', dt, 0.05)) { const P = this.pbuf || (this.pbuf = []); P.push([this.t, this.wheels.map(W => rb.tau[W.k])]); if (P.length > 130) P.shift(); }
    }
  }
  odomDrift() { return Math.hypot(this.odom.x - this.x, this.odom.y - this.y); }
  plotSpec() { return { title: 'wheel motor torques', labels: ['front left', 'front right', 'rear left', 'rear right'], unit: 'N·m' }; }
  physDoc() {
    return `LIMO is a ${this.rb.totalMass.toFixed(1)} kg rigid body on four torque-limited hub motors. The base driver turns <code>/cmd_vel</code> into wheel speeds; friction at each tyre decides what really happens, so skid turns slip, mecanum rollers only grip across their axis, and Ackermann wheels roll about one turning centre. `
      + '<code>/odom</code> is integrated from the wheel encoders (heading from the IMU gyro in the skid modes), so it drifts from the true pose (compare the grey and blue trails on the map); SLAM or localization publishes the <code>map → odom</code> correction. Hold <kbd>Ctrl</kbd> and drag to push the robot or an object.';
  }
  behTick(dt) {
    // lidar driver
    if (this.every('scan', dt, 0.1)) this.scan();
    // lane + light detector (camera)
    if (this.lane && this.every('lane', dt, 1 / 15)) this.laneDetect();
    if (this.beh.slam && this.every('map', dt, Math.max(0.2, this.P('/slam_toolbox', 'map_update_interval') || 1))) this.pubMap();
    if (this.beh.lane) this.laneFollow();
    if (this.beh.nav) this.navStep(dt);
    this.behExtra(dt);
  }
  behExtra() { }
  scan() {
    if (!this.procs.lidar) { this.ranges.fill(Infinity); return; }
    const maxR = this.P('/ydlidar_node', 'range_max'), L = this.kin.T.laser_link || this.Tb, ox = L[3], oy = L[7], n = 360, inc = 2 * Math.PI / n;
    for (let i = 0; i < n; i++) { const r = this.cw.raycast(ox, oy, this.th - Math.PI + i * inc, maxR); this.ranges[i] = r < 0.1 ? 0.1 : r; }
    this.extraScan(ox, oy, inc, maxR);
    this.pub('/ydlidar_node', this.tn.scan, 'sensor_msgs/msg/LaserScan', { header: { stamp: stampNow(), frame_id: String(this.P('/ydlidar_node', 'frame_id') || '') }, angle_min: -3.141593, angle_max: +(Math.PI - inc).toFixed(6), angle_increment: +inc.toFixed(6), time_increment: 0, scan_time: 0.1, range_min: 0.1, range_max: maxR, ranges: this.ranges.map(r => Number.isFinite(r) ? +r.toFixed(3) : Infinity), intensities: [] });
  }
  extraScan() { }
  onSlamScan(m) {
    const fr = (m.header && m.header.frame_id) || '', Tl = this.lookup(this.kin.root, fr);
    if (!Tl) { this.warnEvery('slam_tf', 3, '/slam_toolbox', `Failed to compute laser pose, aborting initialization ("${fr || '(empty)'}" passed to lookupTransform argument source_frame does not exist. )`); return; }
    const Tw = K3.mul(K3.fromRPY(this.x, this.y, 0, 0, 0, this.th), Tl);
    this.slam.update(Tw[3], Tw[7], Math.atan2(Tw[4], Tw[0]), m.ranges.map(r => r == null ? Infinity : r), m.angle_increment, m.angle_min, m.range_max);
    if (this.fix) this.fix.stats.slam++;
    this.coverage = Math.min(1, this.slam.known / (this.cw.gw * this.cw.gh * 0.85));
    if (this.def.mSlam && this.coverage >= 0.7) this.complete(this.def.mSlam);
  }
  pubMap() {
    const g = this.slam, data = new Array(g.w * g.h);
    for (let j = 0; j < g.h; j++) for (let i = 0; i < g.w; i++) data[j * g.w + i] = g.cell(i, j);
    this.pub('/slam_toolbox', this.tn.map || '/map', 'nav_msgs/msg/OccupancyGrid', { header: { stamp: stampNow(), frame_id: 'map' }, info: { map_load_time: stampNow(), resolution: g.res, width: g.w, height: g.h, origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }, data });
  }
  laneDetect() {
    const la = this.P('/lane_detector', 'lookahead') || 0.3, px = this.x + Math.cos(this.th) * la, py = this.y + Math.sin(this.th) * la, P = this.lane;
    let bi = this.st.laneIdx, bd = Infinity;
    const scan = (i0, n) => { for (let k = 0; k < n; k++) { const i = ((i0 + k) % P.length + P.length) % P.length, d = (P[i][0] - px) ** 2 + (P[i][1] - py) ** 2; if (d < bd) { bd = d; bi = i; } } };
    if (bi < 0) scan(0, P.length); else scan(bi - 40, 81);
    if (Math.sqrt(bd) > 0.3) scan(0, P.length);
    const dist = Math.sqrt(bd), nx = P[(bi + 1) % P.length], dirx = nx[0] - P[bi][0], diry = nx[1] - P[bi][1], dl = Math.hypot(dirx, diry) || 1, align = (Math.cos(this.th) * dirx + Math.sin(this.th) * diry) / dl;
    const found = dist < 0.28 && align > 0.35;
    if (found) {
      const err = -Math.sin(this.th) * (P[bi][0] - px) + Math.cos(this.th) * (P[bi][1] - py);
      this.laneErr = err; this.pub('/lane_detector', '/lane_error', 'std_msgs/msg/Float64', { data: +err.toFixed(4) });
      if (this.st.laneIdx >= 0) { let di = bi - this.st.laneIdx; if (di < -P.length / 2) di += P.length; if (di > P.length / 2) di -= P.length; if (di > 0 && di < 30 && dist < 0.15) { this.st.lap += di * 0.02; if (this.st.lap >= P.length * 0.02 * 0.98) this.complete('m7'); } }
    } else this.laneErr = null;
    this.st.laneIdx = bi;
    // traffic light: visible when ahead and close
    const Lg = this.A.light, dx = Lg.x - this.x, dy = Lg.y - this.y, dd = Math.hypot(dx, dy), ang = Math.abs(wrapPi(Math.atan2(dy, dx) - this.th));
    const seen = dd < 1.5 && ang < 1.15 && Math.cos(this.th) > 0.5;
    this.lightSeen = seen ? this.light.s : 'none';
    if (this.every('tl', 1 / 15, 0.2)) this.pub('/lane_detector', '/traffic_light', 'std_msgs/msg/String', { data: this.lightSeen });
  }
  laneFollow() {
    const L = this.laneIn, sp = this.P('/lane_follower', 'speed'), kp = this.P('/lane_follower', 'kp');
    let v = 0, w = 0;
    if (L && L.err != null && this.t - L.t < 0.4) {
      v = sp; w = clampN(kp * L.err, -2.5, 2.5);
      const stopD = this.A.light.stop - this.x;
      if ((L.light === 'red' || L.light === 'yellow') && stopD > 0.02 && stopD < 0.45) v = 0, w = 0;
      else if ((L.light === 'red') && stopD > 0.45 && stopD < 0.8) v = sp * clampN((stopD - 0.45) / 0.35, 0.25, 1);
    }
    this.pub('/lane_follower', '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: +v.toFixed(3), y: 0, z: 0 }, angular: { x: 0, y: 0, z: +w.toFixed(3) } });
  }
  lightStep(dt) {
    if (!this.A.light) return;
    const L = this.light; L.t += dt; const cyc = { green: [6, 'yellow'], yellow: [1.5, 'red'], red: [5, 'green'] }[L.s];
    if (L.t >= cyc[0]) { L.t = 0; L.s = cyc[1]; }
    if (L.s === 'red' && this.x > this.A.light.stop - 0.6 && this.x < this.A.light.stop - 0.02 && Math.abs(this.y - 0.625) < 0.2 && Math.cos(this.th) > 0.85 && Math.abs(this.vx) < 0.03 && this.st.dist > 0.5) { this.redHold = (this.redHold || 0) + dt; if (this.redHold > 0.8) this.complete('m8'); } else this.redHold = 0;
  }
  navStep(dt) {
    if (!this.goal || (!this.path && !this.docking && !this.escape) || this.navState === 'arrived') return;
    const g = this.goal, tol = this.P('/navigator', 'goal_tolerance') || 0.05, vmax = this.P('/navigator', 'max_vel') || 0.4, la = this.P('/navigator', 'lookahead') || 0.3;
    if (this.docking) return this.dock(g);
    if (this.escape) {
      const e = this.escape; e.t += dt;
      if ((!this.tooClose() && e.t > 0.3) || e.t > 2.5) { this.escape = null; this.replan(false); return; }
      return this.cmdNav(0.15 * e.dir, 0, 0);
    }
    const gx = g.exact ? g.px : g.x, gy = g.exact ? g.py : g.y, dg = Math.hypot(gx - this.x, gy - this.y);
    let vx = 0, vy = 0, w = 0;
    if (this.t - this.lastPlanT > 2.5 || this.bumping) { this.replan(false); if (!this.path) return; }
    if (dg < (g.exact ? 0.06 : tol)) {
      if (g.exact) { this.docking = true; this.navState = 'docking'; return this.dock(g); }
      const ey = wrapPi(g.yaw - this.th);
      if (this.mode !== 'ackermann' && Math.abs(ey) > 0.08) { this.navState = 'aligning'; return this.cmdNav(0, 0, clampN(2.2 * ey, -1.0, 1.0)); }
      this.navState = 'arrived'; this.path = null; this.event('nav_done', g); this.cmdNav(0, 0, 0); return;
    }
    // lookahead point on the path
    const P = this.path; let tx = gx, ty = gy;
    for (let i = this.pi; i < P.length - 1; i++) { const [ax, ay] = P[i], [bx, by] = P[i + 1]; if (Math.hypot(bx - this.x, by - this.y) > la) { const L2 = (bx - ax) ** 2 + (by - ay) ** 2 || 1; const t = clampN(((this.x - ax) * (bx - ax) + (this.y - ay) * (by - ay)) / L2, 0, 1); const cx = ax + t * (bx - ax), cy = ay + t * (by - ay); const rem = Math.max(0, la - Math.hypot(cx - this.x, cy - this.y)); const sl = Math.sqrt(L2); tx = cx + (bx - ax) / sl * rem; ty = cy + (by - ay) / sl * rem; if (Math.hypot(tx - bx, ty - by) > Math.hypot(cx - bx, cy - by)) { tx = bx; ty = by; } this.pi = i; break; } }
    if (dg < la) { tx = gx; ty = gy; }
    const dx = tx - this.x, dy = ty - this.y, lx = Math.cos(this.th) * dx + Math.sin(this.th) * dy, ly = -Math.sin(this.th) * dx + Math.cos(this.th) * dy, ang = Math.atan2(ly, lx), Ld = Math.hypot(lx, ly) || 1e-3;
    const slow = clampN(dg / 0.4, 0.15, 1);
    if (this.mode === 'mecanum') { vx = vmax * slow * lx / Ld; vy = vmax * slow * ly / Ld; w = clampN(1.5 * wrapPi((dg < 0.25 ? g.yaw : Math.atan2(dy, dx)) - this.th), -1.2, 1.2); }
    else if (this.mode === 'ackermann') { vx = vmax * slow * (Math.abs(ang) > 1.6 ? -0.6 : 1); const k = 2 * ly / (Ld * Ld); w = vx * clampN(k, -1 / LIMO_DIM.minR, 1 / LIMO_DIM.minR) * (vx < 0 ? -1 : 1); }
    else { if (Math.abs(ang) > 0.9) { w = Math.sign(ang) * 1.0; vx = 0; } else { vx = vmax * slow * Math.cos(ang); w = clampN(2 * vx * ly / (Ld * Ld) + 0.8 * ang * (dg < 0.15 ? 1 : 0), -1.6, 1.6); } }
    this.cmdNav(vx, vy, w);
  }
  // final straight-in approach for precise docking (about 1 cm): face the goal heading first, then drive straight in,
  // so the robot never spins its corners into the table it is docking at
  dock(g) {
    const dx = g.x - this.x, dy = g.y - this.y, c = Math.cos(g.yaw), s = Math.sin(g.yaw), along = c * dx + s * dy, lat = -s * dx + c * dy, ey = wrapPi(g.yaw - this.th);
    const done = () => { this.navState = 'arrived'; this.docking = false; this.path = null; this.event('nav_done', g); return this.cmdNav(0, 0, 0); };
    if (this.mode === 'ackermann') { if (Math.abs(along) < 0.03 || along < 0) return done(); return this.cmdNav(Math.min(0.12, 1.2 * along), 0, clampN(2 * ey + 4 * lat, -0.6, 0.6) * Math.min(0.12, 1.2 * along)); }
    if (!this.dockPh) this.dockPh = 'align';
    if (this.dockPh === 'align') { if (Math.abs(ey) > 0.03) return this.cmdNav(0, this.mode === 'mecanum' ? clampN(2 * lat, -0.1, 0.1) : 0, clampN(2.2 * ey, -0.8, 0.8)); this.dockPh = 'in'; }
    if (along < 0.008 || (this.bumping && along < 0.04)) { this.dockPh = null; return done(); }
    const v = clampN(1.5 * along, 0.03, 0.15), w = clampN(2.5 * ey + (this.mode === 'mecanum' ? 0 : 6 * lat), -0.5, 0.5);
    return this.cmdNav(v, this.mode === 'mecanum' ? clampN(2 * lat, -0.1, 0.1) : 0, w);
  }
  cmdNav(vx, vy, w) { this.pub('/navigator', '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: +vx.toFixed(3), y: +vy.toFixed(3), z: 0 }, angular: { x: 0, y: 0, z: +w.toFixed(3) } }); }
  publish(dt) {
    const c = '/limo_base', tn = this.tn;
    if (!this.procs.base) { this.publishExtra(dt); return; }
    if (this.every('odom', dt, 0.05)) {
      // /odom and odom -> base_footprint come from the wheel encoders; localization (SLAM or the nav stack) adds map -> odom
      const st = stampNow(), O = this.odom, qz = Math.sin(O.th / 2), qw = Math.cos(O.th / 2), To = K3.fromRPY(O.x, O.y, 0, 0, 0, O.th);
      this.pub(c, tn.odom, 'nav_msgs/msg/Odometry', { header: { stamp: st, frame_id: 'odom' }, child_frame_id: 'base_footprint', pose: { pose: { position: { x: +O.x.toFixed(4), y: +O.y.toFixed(4), z: 0 }, orientation: { x: 0, y: 0, z: qz, w: qw } }, covariance: zeros36() }, twist: { twist: { linear: { x: +O.v.toFixed(4), y: +O.vy.toFixed(4), z: 0 }, angular: { x: 0, y: 0, z: +O.w.toFixed(4) } }, covariance: zeros36() } });
      this.pub(c, '/tf', 'tf2_msgs/msg/TFMessage', this.tfMsg([['odom', 'base_footprint', To]]));
      const loc = this.beh.slam ? '/slam_toolbox' : this.beh.nav ? '/navigator' : null; if (loc) this.pub(loc, '/tf', 'tf2_msgs/msg/TFMessage', this.tfMsg([['map', 'odom', this.mapToOdom()]]));
    }
    if (this.every('imu', dt, 0.05)) {
      const rb = this.rb, T = rb.T[0], v = [rb.v[3], rb.v[4], rb.v[5]], p = this.imuPrev, ddt = p ? Math.max(1e-3, this.t - p.t) : 1, a = p ? [(v[0] - p.v[0]) / ddt, (v[1] - p.v[1]) / ddt, (v[2] - p.v[2]) / ddt + 9.81] : [0, 0, 9.81]; this.imuPrev = { t: this.t, v };
      const Ti = K3.inv(T), wb = K3.rot(Ti, rb.v[0], rb.v[1], rb.v[2]), ab = K3.rot(Ti, a[0], a[1], a[2]), q = K3.quat(T), nz = this.noise ? this.noise.imu : 0, rn = () => nz * (Math.random() * 2 - 1);
      this.pub(c, tn.imu, 'sensor_msgs/msg/Imu', { header: { stamp: stampNow(), frame_id: 'imu_link' }, orientation: q, orientation_covariance: new Array(9).fill(0), angular_velocity: { x: +(wb[0] + rn() * 0.05).toFixed(4), y: +(wb[1] + rn() * 0.05).toFixed(4), z: +(wb[2] + rn() * 0.05).toFixed(4) }, angular_velocity_covariance: new Array(9).fill(0), linear_acceleration: { x: +(ab[0] + rn()).toFixed(3), y: +(ab[1] + rn()).toFixed(3), z: +(ab[2] + rn()).toFixed(3) }, linear_acceleration_covariance: new Array(9).fill(0) });
    }
    if (this.every('status', dt, 0.1)) this.pub(c, tn.status, 'limo_msgs/msg/LimoStatus', { header: { stamp: stampNow(), frame_id: 'base_link' }, vehicle_state: this.bumping ? 1 : 0, control_mode: 1, battery_voltage: +(12.2 - this.t * 0.0004).toFixed(2), error_code: 0, motion_mode: LIMO_MODES.find(m => m[0] === this.mode)[3] });
    this.publishExtra(dt);
  }
  publishExtra() { }
  // localization: the map -> odom correction that makes odom + correction = the robot's true pose
  mapToOdom() { const O = this.odom; return K3.mul(K3.fromRPY(this.x, this.y, 0, 0, 0, this.th), K3.inv(K3.fromRPY(O.x, O.y, 0, 0, 0, O.th))); }
  frames() {
    const out = new Set(['map', 'odom', this.kin.root, ...this.kin.links]); this.tfStatic.forEach(m => m.forEach((v, c) => { out.add(c); out.add(v.parent); }));
    return [...out].filter(f => this.poseOf(f));
  }
  lookup(a, b) { const A = this.poseOf(a), B = this.poseOf(b); return A && B ? K3.mul(K3.inv(A), B) : null; }
  focusPos() { return [this.x, this.y, 0.12]; }
  focusYaw() { return this.th; }
  /* ---------- missions ---------- */
  missionTick() {
    const s = this.st;
    if (this.A.markers) this.visionTick();
    if (s.dist >= 2) this.complete('m1');
    if (Object.keys(s.modes).filter(k => s.modes[k] >= 0.3).length >= 4) this.complete('m2');
    if (s.strafe >= 0.5) this.complete('m3');
    if (s.cliDist >= 1) this.complete('m9');
    if (s.userDist >= 1) this.complete('m10');
    const P1 = this.A.slots && this.A.slots[0];
    if (P1 && Math.hypot(this.x - P1.x, this.y - P1.y) < 0.1 && Math.abs(Math.cos(wrapPi(this.th - P1.yaw))) > 0.9 && Math.abs(this.vx) < 0.02) { this.parkT = (this.parkT || 0) + 1 / 60; if (this.parkT > 0.8) this.complete('m6'); } else this.parkT = 0;
  }
  event(type, data) {
    if (type === 'cmd' && data === 'echo:/limo_status') this.complete('m4');
    if (type === 'cmd' && this.A.markers) {
      if (data === 'hz:/camera/color/image_raw' || data === 'bw:/camera/color/image_raw') this.st.vHz = true;
      if (data === 'echo:/camera/color/camera_info') this.st.vInfo = true;
      if (this.st.vHz && this.st.vInfo) this.complete('v1');
    }
  }
  /* ---------- vision missions: checked from the true state of the world ---------- */
  userDriving() { return this.cmd.kind === 'user' && this.t - this.cmd.t < 1.0; }
  camPose() { const C = this.kin.T.camera_link; return C ? [C[3], C[7], C[11], Math.atan2(C[4], C[0])] : [this.x, this.y, 0.13, this.th]; }
  visionTick() {
    const s = this.st, dt = 1 / 60, P = this.ctrPts || (this.ctrPts = laneLoop(this.A.road, this.A.road.r));
    // v2: driven by your node along the yellow centre line (either lane, either direction)
    if (this.userDriving() && Math.abs(this.vx) > 0.02) {
      let bi = -1, bd = 1e9; const look = (i0, n) => { for (let k = 0; k < n; k++) { const i = ((i0 + k) % P.length + P.length) % P.length, d = (P[i][0] - this.x) ** 2 + (P[i][1] - this.y) ** 2; if (d < bd) { bd = d; bi = i; } } };
      if (s.vLineIdx >= 0) look(s.vLineIdx - 30, 61); if (bi < 0 || Math.sqrt(bd) > 0.3) look(0, P.length);
      if (Math.sqrt(bd) < 0.3 && s.vLineIdx >= 0) { let di = bi - s.vLineIdx; if (di < -P.length / 2) di += P.length; if (di > P.length / 2) di -= P.length; if (Math.abs(di) < 30) s.vLine += Math.abs(di) * 0.02; if (s.vLine >= 4) this.complete('v2'); }
      s.vLineIdx = Math.sqrt(bd) < 0.3 ? bi : -1;
    }
    const [cx, cy, , cth] = this.camPose(), still = Math.abs(this.vx) < 0.03 && Math.abs(this.w) < 0.12;
    s.still = still && this.cmd.kind === 'user' && this.t - this.cmd.t < 3 ? s.still + dt : 0;
    // v3: stopped in front of the magenta ball, facing it
    const B = this.ballPos('magenta');
    if (B) { const d = Math.hypot(B[0] - cx, B[1] - cy), brg = Math.abs(wrapPi(Math.atan2(B[1] - cy, B[0] - cx) - cth)); this.vBall = { d, brg }; if (d > 0.18 && d < 0.5 && brg < 0.18 && s.still > 1) this.complete('v3'); }
    // v5: docked 0.5 m in front of marker 7, square to it and centred
    const M = this.A.markers.find(m => m.id === 7);
    if (M) { const nx = Math.cos(M.yaw), ny = Math.sin(M.yaw), dn = (cx - M.x) * nx + (cy - M.y) * ny, lat = -(cx - M.x) * ny + (cy - M.y) * nx, he = Math.abs(wrapPi(cth - (M.yaw + Math.PI))); this.vDock = { dn, lat, he }; if (Math.abs(dn - 0.5) < 0.05 && Math.abs(lat) < 0.05 && he < 0.105 && s.still > 1) this.complete('v5'); }
  }
  // v4: marker IDs your node publishes on /marker_id
  markerSeen(id, from) {
    const n = this.bus.nodes.get(from); if (!n || !n.user || !this.A.markers) return;
    if (this.A.markers.some(m => m.id === id)) { this.st.ids.add(id); if (this.st.ids.size >= 4) this.complete('v4'); }
  }
  /* ---------- 3D ---------- */
  buildScene(view, T) {
    this.view3 = view; const mats = view.mats;
    this.model = snoBuildModel(T, this.kin, this.desc.visuals, mats); view.world.add(this.model.root);
    // wheel variants: mecanum rollers and track belts
    this.wheelParts = { tire: [], mec: [], tracks: [] };
    this.model.root.traverse(o => { if (o.name === 'tire' || o.name === 'spoke') this.wheelParts.tire.push(o); });
    ['front_left', 'front_right', 'rear_left', 'rear_right'].forEach(w => {
      const link = this.model.links[w + '_wheel_link'], g = new T.Group(), sgn = (w.includes('front') ? 1 : -1) * (w.includes('left') ? 1 : -1);
      for (let k = 0; k < 9; k++) { const a = k / 9 * Math.PI * 2, r = snoMesh(T, { s: 'cap', size: [0.009, 0.034], ax: 'y', xyz: [Math.cos(a) * 0.036, 0, Math.sin(a) * 0.036], rpy: [0, -a, 0], m: 'rubber' }, mats); r.children[0].rotation.z = sgn * 0.75; g.add(r); }
      g.add(snoMesh(T, { s: 'cyl', size: [0.03, LIMO_DIM.ww + 0.002], ax: 'y', m: 'metal' }, mats)); link.add(g); this.wheelParts.mec.push(g);
    });
    const tex = snoTex(T, 64, 256, (g, w, hh) => { g.fillStyle = '#1b1f22'; g.fillRect(0, 0, w, hh); g.fillStyle = '#3a4248'; for (let y = 0; y < hh; y += 32) g.fillRect(0, y, w, 12); });
    tex.wrapS = tex.wrapT = T.RepeatWrapping; tex.repeat.set(1, 3); this.trackTex = tex;
    const tm = new T.MeshStandardMaterial({ map: tex, roughness: 0.9 });
    for (const sy of [1, -1]) { const b = new T.Group(); const box = new T.Mesh(new T.BoxGeometry(0.31, 0.05, 0.09), tm); box.castShadow = true; b.add(box); b.position.set(0, sy * LIMO_DIM.track / 2, 0.0); this.model.links.base_link.add(b); this.wheelParts.tracks.push(b); }
    this.applyModeVisuals();
    // arena
    const A = this.A, flo = snoTex(T, 1536, Math.round(1536 * A.H / A.W), (g, w, hh) => this.drawFloor(g, w, hh));
    const floor = new T.Mesh(new T.PlaneGeometry(A.W, A.H), new T.MeshStandardMaterial({ map: flo, roughness: 0.92 })); floor.position.set(A.W / 2, A.H / 2, 0.001); floor.receiveShadow = true; view.world.add(floor);
    const wallM = mats.white;
    [[A.W / 2, -0.02, A.W + 0.08, 0.04], [A.W / 2, A.H + 0.02, A.W + 0.08, 0.04], [-0.02, A.H / 2, 0.04, A.H], [A.W + 0.02, A.H / 2, 0.04, A.H]].forEach(([x, y, w, d]) => view.world.add(snoMesh(T, { s: 'box', size: [w, d, 0.3], xyz: [x, y, 0.15], m: wallM }, mats)));
    A.boxes.forEach(b => {
      if (b.bin) { const g = new T.Group(), cx = (b.x0 + b.x1) / 2, cy = (b.y0 + b.y1) / 2, sx = b.x1 - b.x0, sy2 = b.y1 - b.y0; [[0, sy2 / 2, sx, 0.008], [0, -sy2 / 2, sx, 0.008], [sx / 2, 0, 0.008, sy2], [-sx / 2, 0, 0.008, sy2]].forEach(([x, y, w, d]) => g.add(snoMesh(T, { s: 'box', size: [w, d, b.h], xyz: [x, y, b.h / 2], m: b.m }, mats))); g.add(snoMesh(T, { s: 'box', size: [sx, sy2, 0.006], xyz: [0, 0, 0.003], m: 'dark' }, mats)); g.position.set(cx, cy, 0); view.world.add(g); return; }
      view.world.add(snoMesh(T, { s: 'box', size: [b.x1 - b.x0, b.y1 - b.y0, b.h], xyz: [(b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, b.h / 2], m: b.m || 'grey' }, mats));
      if (b.name === 'building') for (let i = 0; i < 3; i++) for (const s of [-1, 1]) view.world.add(snoMesh(T, { s: 'box', size: [0.12, 0.004, 0.08], xyz: [b.x0 + 0.2 + i * 0.25, s > 0 ? b.y1 + 0.002 : b.y0 - 0.002, 0.28], m: 'glass' }, mats));
    });
    A.circles.forEach(c => {
      if (c.kind === 'cone') { view.world.add(snoMesh(T, { s: 'cone', size: [0.008, c.r, c.h], xyz: [c.x, c.y, c.h / 2], m: 'orange' }, mats)); view.world.add(snoMesh(T, { s: 'cyl', size: [c.r * 0.62, 0.035], xyz: [c.x, c.y, c.h * 0.55], m: 'white' }, mats)); }
      else if (c.kind === 'tree') { view.world.add(snoMesh(T, { s: 'cyl', size: [0.025, 0.25], xyz: [c.x, c.y, 0.125], m: 'wood' }, mats)); view.world.add(snoMesh(T, { s: 'sph', size: [0.13], xyz: [c.x, c.y, 0.34], m: 'green' }, mats)); }
      else if (c.kind === 'pole') { view.world.add(snoMesh(T, { s: 'cyl', size: [c.r, c.h], xyz: [c.x, c.y, c.h / 2], m: 'dark' }, mats)); }
      else view.world.add(snoMesh(T, { s: 'cyl', size: [c.r, c.h], xyz: [c.x, c.y, c.h / 2], m: c.m || 'grey' }, mats));
    });
    if (A.light) {
      const L = A.light, box = snoMesh(T, { s: 'box', size: [0.05, 0.06, 0.15], xyz: [L.x, L.y, 0.55], m: 'dark' }, mats); view.world.add(box);
      this.lamps = ['red', 'yellow', 'green'].map((c, i) => { const m = new T.MeshStandardMaterial({ color: new T.Color({ red: '#ff3b30', yellow: '#ffc400', green: '#22c55e' }[c]).convertSRGBToLinear(), emissive: new T.Color({ red: '#ff3b30', yellow: '#ffc400', green: '#22c55e' }[c]).convertSRGBToLinear(), emissiveIntensity: 0.1, roughness: 0.4 }); const s = new T.Mesh(new T.SphereGeometry(0.016, 16, 12), m); s.position.set(L.x - 0.027, L.y, 0.595 - i * 0.045); view.world.add(s); return [c, m]; });
    }
    // vision props: ArUco markers on the walls (printed on matte paper) and two balls
    (A.markers || []).forEach(M => {
      const code = ARUCO_4X4[M.id], cell = 16, tex = snoTex(T, 8 * cell, 8 * cell, (g) => { g.fillStyle = '#fff'; g.fillRect(0, 0, 8 * cell, 8 * cell); g.fillStyle = '#000'; g.fillRect(cell, cell, 6 * cell, 6 * cell); g.fillStyle = '#fff'; for (let i = 0; i < 16; i++) if ((code >> (15 - i)) & 1) g.fillRect((2 + i % 4) * cell, (2 + (i >> 2)) * cell, cell, cell); });
      tex.magFilter = T.NearestFilter;
      const pl = new T.Mesh(new T.PlaneGeometry(ARUCO_SIZE * 8 / 6, ARUCO_SIZE * 8 / 6), new T.MeshStandardMaterial({ map: tex, roughness: 0.92, metalness: 0 }));
      const c = Math.cos(M.yaw), sn = Math.sin(M.yaw);
      pl.quaternion.setFromRotationMatrix(new T.Matrix4().makeBasis(new T.Vector3(-sn, c, 0), new T.Vector3(0, 0, 1), new T.Vector3(c, sn, 0)));
      pl.position.set(M.x + c * 0.001, M.y + sn * 0.001, ARUCO_Z); pl.receiveShadow = true; pl.userData.marker = M.id; view.world.add(pl);
    });
    this.ballMesh = (this.balls || []).map(b => { const mat = b.color === 'magenta' ? new T.MeshStandardMaterial({ color: new T.Color('#D946EF').convertSRGBToLinear(), roughness: 0.45, metalness: 0.02 }) : mats[b.color]; const m = snoMesh(T, { s: 'sph', size: [BALL_R], m: mat }, mats); m.userData.obj = b; view.world.add(m); return m; });
    // helpers: lidar points, trail, path, goal
    const pg = new T.BufferGeometry(); pg.setAttribute('position', new T.Float32BufferAttribute(new Float32Array(360 * 3), 3));
    this.lidarPts = new T.Points(pg, new T.PointsMaterial({ color: 0xff3355, size: 0.035 })); view.helpers.add(this.lidarPts);
    this.pathLine = snoLine(T, 0x2f6fd0); view.helpers.add(this.pathLine);
    this.goalMark = new T.Group(); this.goalMark.add(snoMesh(T, { s: 'cyl', size: [0.08, 0.006], xyz: [0, 0, 0.003], m: 'blue' }, mats)); this.goalMark.add(snoMesh(T, { s: 'box', size: [0.12, 0.02, 0.008], xyz: [0.06, 0, 0.01], m: 'blue' }, mats)); view.helpers.add(this.goalMark);
    this.showRays = true;
    view.addPip(this.model.links.camera_link, { label: 'front depth camera', vfov: 58, near: 0.03, far: 5, depth: false });
    this.camCap = view.addCamera(this.model.links.camera_link, LIMO_CAM);
    view.onClick = (p) => { if (p[0] < 0 || p[1] < 0 || p[0] > A.W || p[1] > A.H) return; this.sendGoal(p[0], p[1]); Sound.play('tap'); this.event('click_goal'); };
    this.buildExtra(view, T);
  }
  buildExtra() { }
  applyModeVisuals() {
    if (!this.wheelParts || !this.model) return;
    const m = this.mode;
    ['front_left', 'front_right', 'rear_left', 'rear_right'].forEach(w => { const l = this.model.links[w + '_wheel_link']; if (l) l.children.forEach(ch => { ch.visible = this.wheelParts.mec.includes(ch) ? m === 'mecanum' : (m === 'diff' || m === 'ackermann'); }); });
    this.wheelParts.tracks.forEach(o => { o.visible = m === 'tracked'; });
  }
  drawFloor(g, w, hh) {
    const A = this.A, s = w / A.W, X = (x) => x * s, Y = (y) => (A.H - y) * s;
    g.fillStyle = '#C9CFC9'; g.fillRect(0, 0, w, hh);
    g.strokeStyle = 'rgba(0,0,0,.05)'; g.lineWidth = 1; for (let x = 0; x <= A.W; x += 0.5) { g.beginPath(); g.moveTo(X(x), 0); g.lineTo(X(x), hh); g.stroke(); } for (let y = 0; y <= A.H; y += 0.5) { g.beginPath(); g.moveTo(0, Y(y)); g.lineTo(w, Y(y)); g.stroke(); }
    const rd = A.road; if (!rd) return this.drawFloorExtra(g, X, Y, s);
    const rr = (R, fill, stroke, lw, dash) => { g.beginPath(); g.moveTo(X(rd.cx0), Y(rd.cy0 - R)); g.lineTo(X(rd.cx1), Y(rd.cy0 - R)); g.arc(X(rd.cx1), Y(rd.cy0), R * s, Math.PI / 2, 0, true); g.lineTo(X(rd.cx1 + R), Y(rd.cy1)); g.arc(X(rd.cx1), Y(rd.cy1), R * s, 0, -Math.PI / 2, true); g.lineTo(X(rd.cx0), Y(rd.cy1 + R)); g.arc(X(rd.cx0), Y(rd.cy1), R * s, -Math.PI / 2, -Math.PI, true); g.lineTo(X(rd.cx0 - R), Y(rd.cy0)); g.arc(X(rd.cx0), Y(rd.cy0), R * s, Math.PI, Math.PI / 2, true); g.closePath(); if (fill) { g.fillStyle = fill; g.fill(); } if (stroke) { g.strokeStyle = stroke; g.lineWidth = lw * s; g.setLineDash(dash ? dash.map(d => d * s) : []); g.stroke(); g.setLineDash([]); } };
    rr(rd.r + rd.w / 2, '#3B4146'); rr(rd.r - rd.w / 2, '#B9C7B5');
    rr(rd.r + rd.w / 2 - 0.02, null, '#F4F4F0', 0.022); rr(rd.r - rd.w / 2 + 0.02, null, '#F4F4F0', 0.022); rr(rd.r, null, '#F5B700', 0.024);
    const L = A.light; g.fillStyle = '#F4F4F0'; g.fillRect(X(L.stop) - 0.025 * s, Y(0.8), 0.035 * s, 0.36 * s);
    for (let i = 0; i < 6; i++) { g.fillStyle = '#F4F4F0'; g.fillRect(X(2.6) + i * 0.065 * s, Y(0.8 + 0.34), 0.035 * s, 0.68 * s); }
    A.slots.forEach(sl => { g.strokeStyle = '#F4F4F0'; g.lineWidth = 0.02 * s; g.strokeRect(X(sl.x - sl.L / 2), Y(sl.y + sl.W / 2), sl.L * s, sl.W * s); g.fillStyle = '#F4F4F0'; g.font = `700 ${0.11 * s}px Lexend, sans-serif`; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(sl.id, X(sl.x), Y(sl.y)); });
    g.fillStyle = 'rgba(48,164,108,.35)'; g.beginPath(); g.arc(X(A.start[0]), Y(A.start[1]), 0.18 * s, 0, 7); g.fill();
  }
  drawFloorExtra() { }
  sync(view) {
    if (!this.model) return;
    this.model.pose(this.q, this.Tb);
    if (this.ballMesh) this.ballMesh.forEach((m, i) => { const M = this.balls[i].sys.T[0], q = K3.quat(M); m.position.set(M[3], M[7], M[11]); m.quaternion.set(q.x, q.y, q.z, q.w); });
    if (this.trackTex) this.trackTex.offset.y = -this.wheelA * LIMO_DIM.wr * 3 / 0.3;
    if (this.lamps) this.lamps.forEach(([c, m]) => { m.emissiveIntensity = this.light.s === c ? 1.6 : 0.05; });
    const pa = this.lidarPts.geometry.attributes.position, L = this.kin.T.laser_link, inc = 2 * Math.PI / 360; let n = 0;
    this.lidarPts.visible = this.showRays;
    if (this.showRays && L) { for (let i = 0; i < 360; i++) { const r = this.ranges[i]; if (!Number.isFinite(r)) continue; const a = this.th - Math.PI + i * inc; pa.setXYZ(n++, L[3] + r * Math.cos(a), L[7] + r * Math.sin(a), L[11]); } }
    pa.needsUpdate = true; this.lidarPts.geometry.setDrawRange(0, n);
    if (this.path && this.navState !== 'arrived') { snoSetLine(this.pathLine, [[this.x, this.y], ...this.path.slice(this.pi + 1)], 0.012); this.pathLine.visible = true; } else this.pathLine.visible = false;
    if (this.goal && this.navState !== 'arrived') { this.goalMark.visible = true; this.goalMark.position.set(this.goal.x, this.goal.y, 0); this.goalMark.rotation.z = this.goal.yaw; } else this.goalMark.visible = false;
    this.syncExtra(view);
  }
  syncExtra() { }
  tools() { return [['Lidar', 'radio', this.showRays, () => { this.showRays = !this.showRays; }, 'Show lidar points'], ['Depth', 'layers', this.view3 && this.view3.pips[0] ? this.view3.pips[0].depth : false, () => { const p = this.view3 && this.view3.pips.find(x => x.label.includes('front')); if (p) { p.depth = !p.depth; this.view3.pipOn = true; this.event('depth', p.depth); } }, 'Switch the front camera between colour and depth']]; }
  onKey(e, up, release) {
    if (release) { this.keys = {}; if (this._tele && (this._tele.vx || this._tele.vy || this._tele.wz)) this.teleop(0, 0, 0); return false; }
    const map = { ArrowUp: 'f', w: 'f', W: 'f', ArrowDown: 'b', s: 'b', S: 'b', ArrowLeft: 'l', a: 'l', A: 'l', ArrowRight: 'r', d: 'r', D: 'r', q: 'sl', Q: 'sl', e: 'sr', E: 'sr' };
    const k = map[e.key]; if (!k) return false;
    this.keys = this.keys || {}; this.keys[k] = !up; const K = this.keys, sp = this.speed || 0.5;
    this.teleop(((K.f ? 1 : 0) - (K.b ? 1 : 0)) * sp, ((K.sl ? 1 : 0) - (K.sr ? 1 : 0)) * sp * 0.7, ((K.l ? 1 : 0) - (K.r ? 1 : 0)) * (this.mode === 'ackermann' ? 1.4 : 1.6));
    return true;
  }
  hud() {
    const d = (v) => (v * 180 / Math.PI).toFixed(0);
    return `<span>${esc(LIMO_MODES.find(m => m[0] === this.mode)[1])}</span><span>x ${this.x.toFixed(2)}  y ${this.y.toFixed(2)}  θ ${d(this.th)}°</span><span>v ${this.vx.toFixed(2)} m/s${this.mode === 'mecanum' ? `  vy ${this.vy.toFixed(2)}` : ''}  ω ${this.w.toFixed(2)}</span>${this.mode === 'ackermann' ? `<span>steer ${d(this.steer)}°</span>` : ''}${this.lane ? `<span>light ${this.light.s}</span>` : ''}<span class="${this.bumping ? 'bump' : ''}">bumps ${this.bumps}</span><span>odom drift ${(this.odomDrift() * 100).toFixed(1)} cm</span>${this.hudExtra()}`;
  }
  hudExtra() { return ''; }
  /* ---------- panels ---------- */
  renderTab(tab, body) { if (tab === 'control') this.tabControl(body); else if (tab === 'arm') this.tabArm(body); }
  tabControl(body) {
    this.speed = this.speed || 0.5;
    const w = h('div', { class: 'sno-ctl' });
    w.append(h('h4', { html: 'Motion mode <span>param <code>/limo_base motion_mode</code></span>' }), snoSeg(LIMO_MODES.map(m => [m[0], m[1], m[2]]), this.mode, (v) => { this.setMode(v); Sound.play('tap'); }, 'Motion mode'));
    w.append(h('p', { class: 'sidecap', html: { diff: 'Skid steering: the left and right wheels spin at different speeds, so LIMO can turn on the spot.', ackermann: 'Car-like: the front wheels steer (inner wheel turns more). It cannot turn on the spot; the tightest circle is 0.4 m radius.', tracked: 'Rubber tracks: skid steering with more slip. Great for rough ground, a bit slower to turn.', mecanum: 'Mecanum rollers let LIMO slide sideways: use <code>linear.y</code>, the side buttons or Q/E.' }[this.mode] }));
    const jw = h('div', { class: 'joy-wrap', html: `<div class="joy" role="application" aria-label="Joystick: up/down drives, left/right turns"><div class="knob"></div></div>
      <div><div class="dpad"><button data-d="sl" aria-label="Strafe left"${this.mode === 'mecanum' ? '' : ' disabled'}>${icon('arrowL')}<small>slide</small></button><button data-d="f" aria-label="Forward">${icon('arrowU')}</button><button data-d="sr" aria-label="Strafe right"${this.mode === 'mecanum' ? '' : ' disabled'}>${icon('arrowR')}<small>slide</small></button><button data-d="l" aria-label="Turn left">${icon('replay')}</button><button data-d="s" aria-label="Stop">${icon('stop')}</button><button data-d="r" aria-label="Turn right" class="mirror">${icon('replay')}</button><span></span><button data-d="b" aria-label="Backward">${icon('arrowD')}</button><span></span></div>
      <div class="sliders" style="margin-top:12px"><div class="slider"><label for="ls${this.id}">speed</label><input id="ls${this.id}" type="range" min="0.1" max="1" step="0.05" value="${this.speed}"><output>${this.speed.toFixed(2)}</output></div></div></div>` });
    w.append(jw);
    const sp = $('input', jw); sp.addEventListener('input', () => { this.speed = +sp.value; $('output', jw).textContent = this.speed.toFixed(2); });
    snoJoystick($('.joy', jw), (fy, fx) => this.teleop(+(fy * this.speed).toFixed(3), 0, +(fx * 1.8).toFixed(3)), () => this.teleop(0, 0, 0));
    $$('.dpad button', jw).forEach(b => { const v = { f: [1, 0, 0], b: [-1, 0, 0], l: [0.0, 0, 1.4], r: [0, 0, -1.4], sl: [0, 0.7, 0], sr: [0, -0.7, 0], s: [0, 0, 0] }[b.dataset.d]; snoHold(b, () => { const ack = this.mode === 'ackermann' && (b.dataset.d === 'l' || b.dataset.d === 'r'); this.teleop((ack ? 0.35 : v[0]) * this.speed, v[1] * this.speed, v[2]); }, () => this.teleop(0, 0, 0)); });
    w.append(h('div', { class: 'twist-readout sno-read' }));
    w.append(h('h4', { html: 'Autonomy <span>each switch starts real nodes on the graph</span>' }));
    w.append(snoSwitch('SLAM mapping', this.beh.slam, (on) => this.setBeh('slam', on), 'Node <code>/slam_toolbox</code> builds <code>/map</code> from <code>/scan</code>.'));
    w.append(snoSwitch('Navigation', this.beh.nav, (on) => this.setBeh('nav', on), 'Node <code>/navigator</code>: A* on the map + pure pursuit. Click the floor or the map to send <code>/goal_pose</code>.'));
    if (this.lane) w.append(snoSwitch('Lane following', this.beh.lane, (on) => this.setBeh('lane', on), 'Node <code>/lane_follower</code>: steers from <code>/lane_error</code>, stops on red.'));
    this.controlExtra(w);
    const mm = h('div', { class: 'minimap-wrap', html: `<div class="mm-head"><b>Map</b><span>${this.beh.slam ? 'live SLAM grid' : 'static map'} · click to send a goal</span></div><canvas class="minimap" width="480" height="${Math.round(480 * this.A.H / this.A.W)}" aria-label="Map of the arena. Click to send a navigation goal."></canvas>` });
    $('canvas', mm).addEventListener('click', (e) => { const r = e.target.getBoundingClientRect(), x = (e.clientX - r.left) / r.width * this.A.W, y = (1 - (e.clientY - r.top) / r.height) * this.A.H; this.sendGoal(x, y); Sound.play('tap'); this.event('click_goal'); });
    w.append(mm);
    body.append(h('div', { class: 'lpad' }, w));
    this.tickTab('control');
  }
  controlExtra() { }
  tickTab(tab) {
    const root = SnoUI.root; if (!root || tab !== 'control') { if (root && tab === 'arm' && this.tickArm) this.tickArm(root); return; }
    const el = $('.sno-read', root); if (el) { const T = this.bus.topics.get('/cmd_vel'), act = this.bus.active('/cmd_vel'), m = T && T.last && (T.last.twist || T.last); el.innerHTML = `/cmd_vel ${act ? `<span style="color:var(--term-green)">● from ${esc(T.lastFrom || '')}</span>` : '<span style="color:var(--term-dim)">○ idle</span>'}<br>linear.x <b>${m ? pyFloat(+(+m.linear.x).toFixed(2)) : '0.0'}</b>  linear.y <b>${m ? pyFloat(+(+m.linear.y).toFixed(2)) : '0.0'}</b>  angular.z <b>${m ? pyFloat(+(+m.angular.z).toFixed(2)) : '0.0'}</b><br>/limo_status motion_mode <b>${LIMO_MODES.find(x => x[0] === this.mode)[3]}</b>${this.beh.nav ? `  ·  nav <b>${esc(this.navState)}</b>` : ''}${this.beh.slam ? `  ·  mapped <b>${Math.round((this.coverage || 0) * 100)}%</b>` : ''}${this.lane ? `  ·  lane_error <b>${this.laneErr == null ? 'lost' : this.laneErr.toFixed(3)}</b>` : ''}${this.lastWarn ? `<br><span style="color:var(--term-yellow)">${esc(this.lastWarn)}</span>` : ''}`; }
    const cv = $('.minimap', root); if (cv) this.drawMap(cv);
  }
  drawMap(cv) {
    const g = cv.getContext('2d'), W = cv.width, Hh = cv.height, A = this.A, s = W / A.W, X = (x) => x * s, Y = (y) => Hh - y * s;
    const dark = SnoUI.view && SnoUI.view.dark;
    if (this.beh.slam || this.slam.known) {
      const G = this.slam, img = g.createImageData(G.w, G.h);
      for (let j = 0; j < G.h; j++) for (let i = 0; i < G.w; i++) { const v = G.cell(i, j), o = ((G.h - 1 - j) * G.w + i) * 4, c = v < 0 ? (dark ? 60 : 196) : v === 0 ? (dark ? 30 : 250) : (dark ? 230 : 25); img.data[o] = c; img.data[o + 1] = v < 0 ? c + 4 : c; img.data[o + 2] = c; img.data[o + 3] = 255; }
      const tmp = this._tmpc || (this._tmpc = document.createElement('canvas')); tmp.width = G.w; tmp.height = G.h; tmp.getContext('2d').putImageData(img, 0, 0);
      g.imageSmoothingEnabled = false; g.drawImage(tmp, 0, 0, W, Hh);
    } else {
      const Wd = this.cw; g.fillStyle = dark ? '#1A2420' : '#F4F7F5'; g.fillRect(0, 0, W, Hh);
      g.fillStyle = dark ? '#5B6A64' : '#3A4642';
      for (let j = 0; j < Wd.gh; j++) for (let i = 0; i < Wd.gw; i++) if (Wd.occ[j * Wd.gw + i]) g.fillRect(i * Wd.res * s, Hh - (j + 1) * Wd.res * s, Wd.res * s + 0.5, Wd.res * s + 0.5);
      if (A.road) { g.strokeStyle = dark ? 'rgba(245,183,0,.35)' : 'rgba(245,183,0,.6)'; g.lineWidth = 1.5; g.beginPath(); this.lane.forEach(([x, y], i) => { i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); }); g.closePath(); g.stroke(); }
    }
    if (this.otrail && this.otrail.length > 1) { g.strokeStyle = 'rgba(130,130,130,.7)'; g.lineWidth = 2; g.setLineDash([4, 3]); g.beginPath(); this.otrail.forEach(([x, y], i) => { i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); }); g.stroke(); g.setLineDash([]); }
    if (this.trail.length > 1) { g.strokeStyle = 'rgba(47,111,208,.55)'; g.lineWidth = 2; g.beginPath(); this.trail.forEach(([x, y], i) => { i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); }); g.stroke(); }
    if (this.path) { g.strokeStyle = '#2F6FD0'; g.lineWidth = 2.5; g.setLineDash([6, 4]); g.beginPath(); g.moveTo(X(this.x), Y(this.y)); this.path.slice(this.pi + 1).forEach(([x, y]) => g.lineTo(X(x), Y(y))); g.stroke(); g.setLineDash([]); }
    if (this.goal) { g.fillStyle = '#2F6FD0'; g.beginPath(); g.arc(X(this.goal.x), Y(this.goal.y), 5, 0, 7); g.fill(); }
    g.fillStyle = 'rgba(255,51,85,.85)'; const inc = 2 * Math.PI / 360;
    for (let i = 0; i < 360; i += 2) { const r = this.ranges[i]; if (!Number.isFinite(r)) continue; const a = this.th - Math.PI + i * inc; g.fillRect(X(this.x + r * Math.cos(a)) - 1, Y(this.y + r * Math.sin(a)) - 1, 2, 2); }
    g.save(); g.translate(X(this.x), Y(this.y)); g.rotate(-this.th); g.fillStyle = '#F26419'; g.strokeStyle = '#fff'; g.lineWidth = 1.5; g.beginPath(); g.rect(-LIMO_DIM.L / 2 * s, -LIMO_DIM.W / 2 * s, LIMO_DIM.L * s, LIMO_DIM.W * s); g.fill(); g.stroke(); g.fillStyle = '#fff'; g.beginPath(); g.moveTo(LIMO_DIM.L / 2 * s, 0); g.lineTo(LIMO_DIM.L / 2 * s - 6, -4); g.lineTo(LIMO_DIM.L / 2 * s - 6, 4); g.fill(); g.restore();
  }
}

/* ---------- LIMO COBOT: LIMO base + 6-axis arm + gripper ---------- */
class CobotSim extends LimoSim {
  constructor(def) {
    super(def);
    this.armChain = this.kin.chain('arm_base', 'tcp');
  }
  resetExtra() {
    this.arm = { q: [0, 0, 0, 0, 0, 0], from: null, to: null, t: 0, dur: 0, g: 100, gt: 100, speed: 50 };
    const home = this.armHomeQ();
    this.arm.q = home.slice();
    if (!this.cubes) this.cubes = this.A.cubes.map(([c, x]) => { const sys = physBox('cube', [0.03, 0.03, 0.03], 0.02, { mu: 0.8 }); this.world.add(sys); return { color: c, x0: x, sys }; });
    this.cubes.forEach(c => { c.sys.off = false; c.sys.setYaw(c.x0, this.A.cubeY, this.A.tableTop + 0.0153, 0); c.sys.zeroVel(); Object.assign(c, { x: c.x0, y: this.A.cubeY, z: this.A.tableTop + 0.015, yaw: 0, held: false, placed: null, rest: 0 }); });
    COBOT_J.forEach((n, i) => { const k = this.rb.ji[n]; this.rb.q[k] = home[i]; this.rb.qt[k] = home[i]; }); this.rb.fk();
    ['gripper_left', 'gripper_right'].forEach(n => { const k = this.rb.ji[n]; this.rb.q[k] = 0.02; this.rb.qt[k] = 0.02; });
    if (this.rb.payload) this.rb.setPayload(0, 0);
    this.held = null; this.demo = null; this.gripLog = { open: false, closed: false };
  }
  armQ() { return COBOT_J.map(n => this.rb.q[this.rb.ji[n]]); }
  armHomeQ() { return [0, 30, -130, 10, 0, 0].map(v => v * D2R); }
  setupExtra() {
    const n = '/arm_driver';
    this.node(n, 'driver', [['speed', 50, 'integer', [1, 100]]]);
    this.bus.advertise(n, '/joint_states', 'sensor_msgs/msg/JointState'); this.bus.advertise(n, '/arm/angles', 'snobots_msgs/msg/ArmAngles'); this.bus.advertise(n, '/arm/coords', 'snobots_msgs/msg/ArmCoords');
    this.bus.subscribe(n, '/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', (m, from) => { const r = this.sendAngles((m.angles || []).map(Number), +m.speed || this.P(n, 'speed')); if (!r.ok) this.lastWarn = r.reason; this.armEvent('angles', from); });
    this.bus.subscribe(n, '/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', (m, from) => { const r = this.sendCoords([+m.x, +m.y, +m.z, +m.rx, +m.ry, +m.rz], +m.speed || this.P(n, 'speed')); if (!r.ok) this.lastWarn = r.reason; this.armEvent('coords', from); });
    this.bus.subscribe(n, '/arm/gripper', 'std_msgs/msg/Int32', (m, from) => { this.setGripper(+m.data); this.armEvent('gripper', from); });
    this.bus.service('/arm/go_home', 'std_srvs/srv/Trigger', n, () => { this.sendAngles(this.armHomeQ().map(v => v / D2R), 60); this.event('service', '/arm/go_home'); return { success: true, message: 'moving to home pose' }; });
  }
  runExe(T, pkg, exe) {
    if (exe === 'go_home') { this.sendAngles(this.armHomeQ().map(v => v / D2R), 60); this.event('service', '/arm/go_home'); return T.print('[INFO] [go_home]: folding the arm'); }
    if (exe === 'open_gripper' || exe === 'close_gripper') { this.panelPub('/arm/gripper', 'std_msgs/msg/Int32', { data: exe === 'open_gripper' ? 100 : 0 }); return T.print(`[INFO] [${exe}]: done`); }
    T.print('This executable is not simulated here.', 'w');
  }
  armEvent(kind, from) { const nd = this.bus.nodes.get(from), k = nd ? (nd.user ? 'user' : nd.kind) : ''; if (k === 'cli' && kind === 'angles') this.complete('m4'); if (k === 'user') { this.userArm = (this.userArm || 0) + 1; if (this.userArm >= 3) this.complete('m8'); } }
  behaviors() { const b = super.behaviors(); b['pick_place.launch.py'] = { label: 'pick & place demo', nodes: ['/pick_place_demo'], on: () => !!this.demo, set: (on) => this.setDemo(on), hello: '[pick_place_demo-1] [INFO] sorting cubes: drive to the table, pick, drive to the matching bin, place' }; return b; }
  /* ---- arm motion: joint-space interpolation, like send_angles(angles, speed) ---- */
  sendAngles(deg, speed) {
    if (!Array.isArray(deg) || deg.length !== 6 || deg.some(v => !Number.isFinite(v))) return { ok: false, reason: 'angles must be a list of 6 numbers (degrees)' };
    const tgt = deg.map((d, i) => clampN(d * D2R, -COBOT_LIM[i] * D2R, COBOT_LIM[i] * D2R));
    const sp = clampN(speed || 50, 1, 100) / 100 * 2.6, dmax = Math.max(...tgt.map((v, i) => Math.abs(v - this.arm.q[i])));
    Object.assign(this.arm, { from: this.arm.q.slice(), to: tgt, t: 0, dur: Math.max(0.15, dmax / sp) });
    return { ok: true };
  }
  armBaseT() { return this.kin.T.arm_base || K3.mul(this.Tb, K3.mul(this.kin.J.base_joint.O, this.kin.J.arm_mount.O)); }
  ikArm(Tt) {
    const seeds = [this.qObj(this.arm.q), this.qObj([0, -60 * D2R, -50 * D2R, 20 * D2R, 0, 0]), this.qObj([0, -30 * D2R, -90 * D2R, 30 * D2R, 0, 0]), this.qObj([0, -10 * D2R, -110 * D2R, 30 * D2R, 0, 0])];
    let best = null;
    for (const q of seeds) {
      const yaw = Math.atan2(Tt[7], Tt[3]); if (q !== seeds[0]) q.joint1 = yaw;
      const r = this.kin.ik(this.armChain, q, K3.I(), Tt, { iters: 80, wr: 0.5, lambda: 0.01 });
      const T = this.kin.fk(Object.assign({}, this.q, q), K3.I()), elbow = T.link3 ? T.link3[11] : 0;
      const qa = COBOT_J.map(n => q[n]), dist = qa.reduce((s, v, i) => s + Math.abs(v - this.arm.q[i]), 0);
      const score = r.ep * 100 + r.er * 5 + dist * 0.05 - elbow * 0.5;
      if (r.ep < 0.004 && r.er < 0.05 && (!best || score < best.score)) best = { q: qa, score };
    }
    this.fk();
    return best ? best.q : null;
  }
  qObj(a) { const o = {}; COBOT_J.forEach((n, i) => { o[n] = a[i]; }); return o; }
  sendCoords(c, speed) {
    if (c.some(v => !Number.isFinite(v))) return { ok: false, reason: 'coords must be numbers: x y z (mm), rx ry rz (deg)' };
    const Tt = K3.fromRPY(c[0] / 1000, c[1] / 1000, c[2] / 1000, c[3] * D2R, c[4] * D2R, c[5] * D2R);
    const q = this.ikArm(Tt); if (!q) return { ok: false, reason: `no IK solution for [${c.map(v => v.toFixed(0)).join(', ')}]: out of reach or blocked by joint limits` };
    return this.sendAngles(q.map(v => v / D2R), speed);
  }
  setGripper(v) { this.arm.gt = clampN(Math.round(v), 0, 100); if (this.arm.gt >= 80) this.gripLog.open = true; if (this.arm.gt <= 10 && this.gripLog.open) { this.gripLog.closed = true; this.complete('m2'); } }
  stepExtra(dt) {
    const A = this.arm, rb = this.rb;
    if (A.to) { A.t += dt; const u = ease01(A.t / A.dur); A.q = A.from.map((v, i) => lerpN(v, A.to[i], u)); if (A.t >= A.dur) { A.q = A.to.slice(); A.to = null; this.event('arm_done'); } }
    // joint servos follow the trajectory; the arm controller adds gravity compensation from its model
    COBOT_J.forEach((n, i) => { const k = rb.ji[n]; rb.mode[k] = 1; rb.qt[k] = A.q[i]; rb.tff[k] = rb.bias[6 + k]; });
    let gmin = 0; if (this.held) gmin = (0.03 - 0.008) / 0.04 * 100 - 1;
    A.g = approach(A.g, Math.max(A.gt, gmin), 160 * dt);
    const o = A.g / 100 * 0.02; ['gripper_left', 'gripper_right'].forEach(n => { const k = rb.ji[n]; rb.mode[k] = 1; rb.qt[k] = o; });
    if (Math.abs(this.armQ()[0]) >= 1.5) this.complete('m1');
    this.demoStep(dt);
  }
  // grasp: both fingers squeeze a cube -> it is held (welded to the gripper); opening the gripper lets go
  readState(dt) {
    super.readState(dt);
    if (!this.arm || !this.cubes) return;
    const rb = this.rb, A = this.arm, gb = rb.linkBody.gripper_base, Tg = rb.linkT('gripper_base'), open = 0.008 + 2 * rb.q[rb.ji.gripper_left];
    if (!this.held && A.gt < 50) {
      for (const c of this.cubes) {
        if (c.held) continue;
        const l = K3.apply(K3.inv(Tg), c.x, c.y, c.z); if (Math.abs(l[0]) > 0.02 || Math.abs(l[1]) > 0.012 || l[2] < 0.03 || l[2] > 0.075) continue;
        let fl = false, fr = false; for (let i = 0; i < this.world.nc; i++) { const k = this.world.C[i], o = k.a.s === c.sys ? k.b : k.b.s === c.sys ? k.a : null; if (!o || k.fn < 0.2) continue; if (o.link === 'finger_left') fl = true; if (o.link === 'finger_right') fr = true; }
        if ((fl && fr) || open < 0.031) {
          this.held = c; c.held = true; c.placed = null; c.rel = K3.mul(K3.inv(Tg), c.sys.T[0]); c.sys.off = true;
          const lc = [c.rel[3], c.rel[7], c.rel[11]], m = c.sys.totalMass, I = m * 0.03 * 0.03 / 6;
          rb.setPayload(gb, m, K3.apply(rb.linkOff.gripper_base, ...lc), [I, 0, 0, 0, I, 0, 0, 0, I]); Sound.play('tap'); this.event('grasp', c); this.complete('m5'); break;
        }
      }
    }
    if (this.held) {
      const c = this.held, M = K3.mul(Tg, c.rel);
      if (A.gt > 60 && open > 0.034) { c.held = false; this.held = null; rb.setPayload(gb, 0); c.sys.off = false; c.sys.setPose([M[3], M[7], M[11]], phQuat(M)); const v = rb.ptVel(gb, [M[3], M[7], M[11]]); c.sys.v.fill(0); c.sys.v[3] = v[0]; c.sys.v[4] = v[1]; c.sys.v[5] = v[2]; this.event('release', c); }
      else { c.x = M[3]; c.y = M[7]; c.z = M[11]; c.yaw = Math.atan2(M[4], M[0]); c.M = M; }
    }
    for (const c of this.cubes) {
      if (c.held) continue;
      const S2 = c.sys; c.x = S2.pos[0]; c.y = S2.pos[1]; c.z = S2.pos[2]; c.M = S2.T[0];
      // landed in a bin?
      const slow = Math.hypot(S2.v[3], S2.v[4], S2.v[5]) < 0.02;
      if (slow) { c.rest += dt; if (c.rest > 0.25 && !c.restDone) { c.restDone = true; const b = this.A.boxes.find(x => x.bin && c.x > x.x0 && c.x < x.x1 && c.y > x.y0 && c.y < x.y1); c.placed = b ? b.bin : null; if (b) { Sound.play(b.bin === c.color ? 'coin' : 'bump'); this.event('placed', { cube: c, bin: b.bin }); } } }
      else { c.rest = 0; c.restDone = false; }
    }
  }
  publishExtra(dt) {
    const n = '/arm_driver';
    if (this.every('ajs', dt, 0.05)) { const names = [...COBOT_J, 'gripper'], rb = this.rb, q = Object.assign({}, this.qObj(this.armQ()), { gripper: +(0.008 + 2 * rb.q[rb.ji.gripper_left]).toFixed(4) }), m = this.jointStateMsg(names, q); m.effort = names.map(nm => +(rb.tau[rb.ji[nm === 'gripper' ? 'gripper_left' : nm]] || 0).toFixed(3)); this.pub(n, '/joint_states', 'sensor_msgs/msg/JointState', m); }
    if (this.every('aang', dt, 0.1)) {
      this.pub(n, '/arm/angles', 'snobots_msgs/msg/ArmAngles', { angles: this.armQ().map(v => +(v / D2R).toFixed(2)), speed: this.arm.to ? this.arm.speed : 0 });
      const c = this.coords(); this.pub(n, '/arm/coords', 'snobots_msgs/msg/ArmCoords', { x: c[0], y: c[1], z: c[2], rx: c[3], ry: c[4], rz: c[5], speed: 0 });
    }
  }
  coords() { const T = this.kin.T, M = K3.mul(K3.inv(T.arm_base), T.tcp), e = K3.rpy(M); return [M[3] * 1000, M[7] * 1000, M[11] * 1000, e[0] / D2R, e[1] / D2R, e[2] / D2R].map(v => +v.toFixed(1)); }
  /* ---- scripted pick & place (a node on the graph) ---- */
  setDemo(on) {
    if (!!this.demo === on) return;
    if (on) { this.demo = { st: 'next', t: 0 }; this.bus.addNode('/pick_place_demo', 'behavior'); ['/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', '/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', '/arm/gripper', 'std_msgs/msg/Int32', '/goal_pose', 'geometry_msgs/msg/PoseStamped'].forEach((x, i, a) => { if (i % 2 === 0) this.bus.advertise('/pick_place_demo', x, a[i + 1]); }); if (!this.beh.nav) this.setBeh('nav', true); }
    else { this.demo = null; this.bus.removeNode('/pick_place_demo'); }
    SnoUI.refreshTab('control'); this.event('behavior', { k: 'demo', on });
  }
  demoPub(topic, type, msg) { this.pub('/pick_place_demo', topic, type, msg); }
  demoArm(mm, speed = 60) { this.demoPub('/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', { x: mm[0], y: mm[1], z: mm[2], rx: 180, ry: 0, rz: mm[3] || 0, speed }); }
  demoGoal(x, y, yaw) { this.exactGoal = true; this.pub('/pick_place_demo', '/goal_pose', 'geometry_msgs/msg/PoseStamped', { header: { stamp: stampNow(), frame_id: 'map' }, pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } } }); }
  armLocal(wx, wy, wz) { const p = K3.apply(K3.inv(this.armBaseT()), wx, wy, wz); return p.map(v => +(v * 1000).toFixed(1)); }
  demoStep(dt) {
    const d = this.demo; if (!d) return; d.t += dt;
    const aq = this.armQ(), busy = !!this.arm.to || aq.some((v, i) => Math.abs(v - this.arm.q[i]) > 0.03), A = this.A, mount = this.kin.J.arm_mount.O[3];
    const go = (st) => { d.st = st; d.t = 0; };
    switch (d.st) {
      case 'next': { const c = this.cubes.find(k => !k.held && k.placed !== k.color); if (!c) { this.demoLog('all cubes sorted'); this.setDemo(false); return; } d.c = c; this.demoPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: this.armHomeQ().map(v => +(v / D2R).toFixed(1)), speed: 60 }); this.demoPub('/arm/gripper', 'std_msgs/msg/Int32', { data: 100 });
        const onTable = c.y > A.tableY - 0.02, yaw = onTable ? Math.PI / 2 : Math.atan2(c.y - this.y, c.x - this.x);
        const reach = 0.21 + mount; const gx = onTable ? c.x - mount * 0 : c.x - Math.cos(yaw) * reach, gy = onTable ? A.tableY - (LIMO_DIM.L / 2 + 0.01) : c.y - Math.sin(yaw) * reach; d.gx = gx; d.gy = gy; d.gyaw = yaw;
        this.demoGoal(+gx.toFixed(3), +gy.toFixed(3), yaw); go('drive_pick'); break; }
      case 'drive_pick': if (this.navState === 'arrived' && Math.abs(this.vx) < 0.01) go('above'); else if (this.navState === 'no_path' && d.t > 1) { this.demoLog('no path to the cube'); this.setDemo(false); } break;
      case 'above': if (busy || d.t < 0.3) break; { const p = this.armLocal(d.c.x, d.c.y, d.c.z + 0.06); this.demoArm([p[0], p[1], p[2], Math.atan2(p[1], p[0]) / D2R]); go('descend'); } break;
      case 'descend': if (busy || d.t < 0.2) break; { const p = this.armLocal(d.c.x, d.c.y, d.c.z); this.demoArm([p[0], p[1], p[2], Math.atan2(p[1], p[0]) / D2R], 30); go('grip'); } break;
      case 'grip': if (busy || d.t < 0.2) break; this.demoPub('/arm/gripper', 'std_msgs/msg/Int32', { data: 0 }); go('lift'); break;
      case 'lift': if (d.t < 0.8) break; if (this.held !== d.c) { this.demoLog('missed the cube, retrying'); go('next'); break; } this.demoPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: [0, -20, -90, -40, 0, 0], speed: 50 }); go('to_bin'); break;
      case 'to_bin': if (busy) break; { const b = A.boxes.find(x => x.bin === d.c.color), by = (b.y0 + b.y1) / 2, gx = b.x0 - (LIMO_DIM.L / 2 + 0.01); this.demoGoal(+gx.toFixed(3), +by.toFixed(3), 0); d.bin = b; go('drive_bin'); } break;
      case 'drive_bin': if (this.navState === 'arrived' && Math.abs(this.vx) < 0.01) go('over'); break;
      case 'over': if (busy || d.t < 0.3) break; { const b = d.bin, p = this.armLocal((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, 0.17); this.demoArm([p[0], p[1], p[2], Math.atan2(p[1], p[0]) / D2R], 50); go('drop'); } break;
      case 'drop': if (busy || d.t < 0.2) break; this.demoPub('/arm/gripper', 'std_msgs/msg/Int32', { data: 100 }); go('home'); break;
      case 'home': if (d.t < 0.7) break; this.demoPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: this.armHomeQ().map(v => +(v / D2R).toFixed(1)), speed: 60 }); go('next'); break;
    }
  }
  demoLog(t) { this.lastWarn = 'pick_place_demo: ' + t; }
  missionTick() {
    const s = this.st;
    if (s.userDist >= 1) this.complete('m8');
  }
  event(type, data) {
    if (type === 'service' && data === '/arm/go_home') this.complete('m3');
    if (type === 'placed') { if (data.bin === data.cube.color) this.complete('m6'); if (this.cubes.every(c => c.placed === c.color)) this.complete('m7'); }
  }
  /* ---- 3D extras ---- */
  buildExtra(view, T) {
    const mats = view.mats;
    this.cubeMesh = this.cubes.map(c => { const m = snoMesh(T, { s: 'box', size: [0.03, 0.03, 0.03], m: c.color, round: 0.002 }, mats); m.userData.obj = c; view.world.add(m); return m; });
    view.addPip(this.model.links.arm_camera, { label: 'arm camera', vfov: 62, near: 0.01, far: 3, look: 'z' });
    view.pipIdx = 1;
  }
  syncExtra() { if (!this.cubeMesh) return; this.cubeMesh.forEach((m, i) => { const c = this.cubes[i], M = c.M || c.sys.T[0], q = K3.quat(M); m.position.set(M[3], M[7], M[11]); m.quaternion.set(q.x, q.y, q.z, q.w); }); }
  drawFloorExtra(g, X, Y, s) {
    const A = this.A;
    g.strokeStyle = '#F5B700'; g.lineWidth = 0.025 * s; g.setLineDash([0.1 * s, 0.07 * s]);
    g.strokeRect(X(A.boxes[0].x0 - 0.05), Y(A.boxes[0].y0), (A.boxes[0].x1 - A.boxes[0].x0 + 0.1) * s, 0.4 * s); g.strokeRect(X(4.0), Y(2.95), 0.5 * s, 1.9 * s); g.setLineDash([]);
    g.fillStyle = '#5B6670'; g.font = `700 ${0.1 * s}px Lexend, sans-serif`; g.textAlign = 'center'; g.fillText('PICK', X(2.25), Y(3.27)); g.save(); g.translate(X(4.25), Y(2.0)); g.rotate(-Math.PI / 2); g.fillText('DROP', 0, 0); g.restore();
    g.fillStyle = 'rgba(48,164,108,.35)'; g.beginPath(); g.arc(X(A.start[0]), Y(A.start[1]), 0.18 * s, 0, 7); g.fill();
  }
  hudExtra() { const c = this.coords(); return `<span>TCP ${c[0].toFixed(0)}, ${c[1].toFixed(0)}, ${c[2].toFixed(0)} mm</span><span>gripper ${Math.round((this.rb.q[this.rb.ji.gripper_left] / 0.02) * 100)}${this.held ? ' · holding ' + this.held.color : ''}</span><span>sorted ${this.cubes.filter(c => c.placed === c.color).length}/3</span>`; }
  controlExtra(w) {
    w.append(snoSwitch('Pick & place demo', !!this.demo, (on) => this.setDemo(on), 'Node <code>/pick_place_demo</code> sorts every cube into the bin of the same colour.'));
  }
  tabArm(body) {
    const w = h('div', { class: 'lpad sno-arm' });
    w.append(h('p', { class: 'sidecap', style: 'margin-top:0', html: 'Like <code>send_angles</code>: each slider sets a joint target in degrees, published on <code>/arm/angles_goal</code> by <code>/arm_panel</code>. Zero on every joint points the arm straight up.' }));
    const g = h('div', { class: 'jgroup' }); g.append(h('h4', {}, 'Joints (degrees)'));
    const send = () => { const a = COBOT_J.map((_, i) => +$(`#aj${i}`, w).value); this.panelPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: a, speed: 70 }); this.event('arm_ui'); };
    COBOT_J.forEach((n, i) => { const row = h('div', { class: 'slider jrow', html: `<label for="aj${i}">${n} (J${i + 1})</label><input id="aj${i}" type="range" min="${-COBOT_LIM[i]}" max="${COBOT_LIM[i]}" step="1" value="${Math.round(this.arm.q[i] / D2R)}" data-ai="${i}"><output>${Math.round(this.arm.q[i] / D2R)}°</output>` }); $('input', row).addEventListener('input', send); g.append(row); });
    const gr = h('div', { class: 'slider jrow', html: `<label for="agr">gripper (0 closed, 100 open)</label><input id="agr" type="range" min="0" max="100" step="1" value="${this.arm.gt}"><output>${Math.round(this.arm.g)}</output>` });
    $('input', gr).addEventListener('input', (e) => this.panelPub('/arm/gripper', 'std_msgs/msg/Int32', { data: +e.target.value }));
    g.append(gr); w.append(g);
    const jog = h('div', { class: 'jog' });
    jog.innerHTML = `<h4>Cartesian jog <span>gripper pointing down · like <code>send_coords</code></span></h4><div class="jog-grid">${[['x', '+'], ['x', '-'], ['y', '+'], ['y', '-'], ['z', '+'], ['z', '-']].map(([a, s]) => `<button class="btn btn-sm" data-ax="${a}" data-s="${s}">${a.toUpperCase()}${s === '+' ? '+' : '−'} 2 cm</button>`).join('')}</div>
      <div class="row-btns"><button class="btn btn-sm" data-p="home">${icon('replay')}Home</button><button class="btn btn-sm" data-p="ready">Ready to pick</button><button class="btn btn-sm" data-p="zero">All zero (up)</button><button class="btn btn-sm btn-good" data-p="pick">Pick nearest cube</button></div>`;
    $$('[data-ax]', jog).forEach(b => b.addEventListener('click', () => {
      const c = this.coords(), d = b.dataset.s === '+' ? 20 : -20, ax = { x: 0, y: 1, z: 2 }[b.dataset.ax]; const t = [c[0], c[1], c[2]]; t[ax] += d;
      const yaw = Math.atan2(t[1], t[0]) / D2R; this.panelPub('/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', { x: t[0], y: t[1], z: t[2], rx: 180, ry: 0, rz: yaw, speed: 60 }); this.event('arm_ui');
    }));
    $$('[data-p]', jog).forEach(b => b.addEventListener('click', () => {
      const p = b.dataset.p;
      if (p === 'home') this.panelPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: this.armHomeQ().map(v => +(v / D2R).toFixed(1)), speed: 60 });
      if (p === 'zero') this.panelPub('/arm/angles_goal', 'snobots_msgs/msg/ArmAngles', { angles: [0, 0, 0, 0, 0, 0], speed: 50 });
      if (p === 'ready') this.panelPub('/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', { x: 200, y: 0, z: 60, rx: 180, ry: 0, rz: 0, speed: 60 });
      if (p === 'pick') { const c = this.cubes.filter(k => !k.held).map(k => ({ k, p: this.armLocal(k.x, k.y, k.z) })).sort((a, b2) => Math.hypot(...a.p) - Math.hypot(...b2.p))[0]; if (!c) return; const r = Math.hypot(c.p[0], c.p[1]); if (r > 300) { toast(`The nearest cube is ${Math.round(r)} mm away. Drive closer (the arm reaches about 280 mm).`, { mood: 'think' }); return; } this.panelPub('/arm/gripper', 'std_msgs/msg/Int32', { data: 100 }); this.panelPub('/arm/coords_goal', 'snobots_msgs/msg/ArmCoords', { x: c.p[0], y: c.p[1], z: c.p[2] + 15, rx: 180, ry: 0, rz: Math.atan2(c.p[1], c.p[0]) / D2R, speed: 40 }); if (this.lastWarn && this.lastWarn.startsWith('no IK')) toast(esc(this.lastWarn), { mood: 'think' }); else toast('Arm moving above the cube. Now close the gripper (slider) to grab it.', { mood: 'happy', ms: 3000 }); }
    }));
    w.append(jog, h('div', { class: 'twist-readout arm-read' }));
    body.append(w); this.tickArm(SnoUI.root);
  }
  panelPub(topic, type, msg) { if (!this.bus.nodes.has('/arm_panel')) this.bus.addNode('/arm_panel', 'teleop'); this.pub('/arm_panel', topic, type, msg); }
  tickArm(root) {
    if (!root) return;
    const aq = this.armQ(); $$('[data-ai]', root).forEach(inp => { const i = +inp.dataset.ai; if (document.activeElement !== inp && !this.arm.to) inp.value = Math.round(this.arm.q[i] / D2R); inp.nextElementSibling.textContent = Math.round(aq[i] / D2R) + '°'; });
    const gi = $('#agr', root); if (gi) { if (document.activeElement !== gi) gi.value = this.arm.gt; gi.nextElementSibling.textContent = Math.round(this.arm.g); }
    const el = $('.arm-read', root); if (el) { const c = this.coords(); el.innerHTML = `/arm/coords (mm, deg): <b>[${c.join(', ')}]</b><br>/arm/angles: <b>[${this.armQ().map(v => (v / D2R).toFixed(1)).join(', ')}]</b><br>${this.held ? `holding the <b>${this.held.color}</b> cube` : 'gripper empty'} · ${this.arm.to ? '<span style="color:var(--term-green)">moving</span>' : 'idle'}${this.lastWarn ? `<br><span style="color:var(--term-yellow)">${esc(this.lastWarn)}</span>` : ''}`; }
  }
}
