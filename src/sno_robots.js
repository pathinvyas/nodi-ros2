/* ============================== SnoBots: AiNex and R1 specifics ============================== */
const D2R = Math.PI / 180;
const AINEX_FIELD = { x0: -0.9, x1: 1.5, y0: -0.9, y1: 0.9, goalY: 0.25, goalD: 0.18, goalH: 0.2 };

class AinexSim extends HumanoidSim {
  setupExtra() {
    this.node('/color_detector', 'vision', [['target_color', 'red', 'string'], ['min_area', 0.0001, 'double', [0, 0.05]]]);
    this.bus.advertise('/color_detector', '/color_detection', 'snobots_msgs/msg/ColorDetection');
    this.trk = { on: false, held: 0 }; this.rx = { on: false, cool: 0 }; this.det = null; this.goals = 0;
  }
  onParam(node, k, v) {
    if (node === '/color_detector' && k === 'target_color' && !['red', 'green', 'blue', 'any'].includes(String(v))) return { ok: false, reason: "target_color must be red, green, blue or any" };
    this.event('param', { node, k, v }); return { ok: true };
  }
  behaviors() {
    return {
      'color_tracking.launch.py': { label: 'head tracking', nodes: ['/head_tracker'], on: () => this.trk.on, set: (on) => this.setTracker(on), hello: '[head_tracker-1] [INFO] following the target_color blob with a P controller on head_pan / head_tilt' },
      'color_reaction.launch.py': { label: 'nod / shake', nodes: ['/color_reactor'], on: () => this.rx.on, set: (on) => this.setReactor(on), hello: '[color_reactor-1] [INFO] red -> nod, green or blue -> shake. Tip: ros2 param set /color_detector target_color any' },
    };
  }
  setTracker(on) {
    if (on === this.trk.on) return; this.trk.on = on;
    if (on) {
      this.node('/head_tracker', 'behavior', [['kp', 0.45, 'double', [0, 1.5]]]);
      this.bus.advertise('/head_tracker', '/joint_commands', 'sensor_msgs/msg/JointState');
      this.trk.un = this.bus.subscribe('/head_tracker', '/color_detection', 'snobots_msgs/msg/ColorDetection', (m) => this.onTrack(m));
    } else { if (this.trk.un) this.trk.un(); this.bus.removeNode('/head_tracker'); this.trk.held = 0; }
    SnoUI.refreshTab('control');
  }
  onTrack(m) {
    if (!m.found) return;
    const hf = Math.tan(this.H.hfov / 2), vf = Math.tan(this.H.camVfov * D2R / 2), k = this.P('/head_tracker', 'kp');
    const J = this.kin.J, pan = clampN(this.q.head_pan - k * Math.atan(m.x * hf), J.head_pan.lo, J.head_pan.hi), tilt = clampN(this.q.head_tilt + k * Math.atan(m.y * vf), J.head_tilt.lo, J.head_tilt.hi);
    this.pub('/head_tracker', '/joint_commands', 'sensor_msgs/msg/JointState', { header: { stamp: stampNow(), frame_id: '' }, name: ['head_pan', 'head_tilt'], position: [+pan.toFixed(4), +tilt.toFixed(4)], velocity: [], effort: [] });
  }
  setReactor(on) {
    if (on === this.rx.on) return; this.rx.on = on;
    if (on) { this.bus.addNode('/color_reactor', 'behavior'); this.bus.advertise('/color_reactor', '/action_group', 'std_msgs/msg/String'); this.rx.un = this.bus.subscribe('/color_reactor', '/color_detection', 'snobots_msgs/msg/ColorDetection', (m) => this.onReact(m)); }
    else { if (this.rx.un) this.rx.un(); this.bus.removeNode('/color_reactor'); }
    SnoUI.refreshTab('control');
  }
  onReact(m) {
    if (!m.found || this.t < this.rx.cool || Math.abs(m.x) > 0.3 || Math.abs(m.y) > 0.4) return;
    this.rx.cool = this.t + 3.2;
    this.pub('/color_reactor', '/action_group', 'std_msgs/msg/String', { data: m.color === 'red' ? 'nod' : 'shake' });
  }
  behaviorsTick(dt) {
    if (!this.every('det', dt, 1 / 15)) return;
    const Tc = this.kin.T.camera_link; if (!Tc) return;
    const Ti = K3.inv(Tc), hf = Math.tan(this.H.hfov / 2), vf = Math.tan(this.H.camVfov * D2R / 2);
    const want = String(this.P('/color_detector', 'target_color') || 'red').toLowerCase(), minA = this.P('/color_detector', 'min_area') || 0;
    let best = null;
    for (const o of this.objs) {
      if (o.kind !== 'block' || (want !== 'any' && o.color !== want)) continue;
      const p = K3.apply(Ti, o.x, o.y, o.z); if (p[0] < 0.04) continue;
      const u = -p[1] / p[0] / hf, v = -p[2] / p[0] / vf; if (Math.abs(u) > 1 || Math.abs(v) > 1 || p[0] > 3) continue;
      const area = Math.pow(o.size / p[0], 2) / (4 * hf * vf); if (area < minA) continue;
      const sticky = want === 'any' && this.det && this.det.color === o.color ? 1.8 : 1;   // keep following the same blob
      if (!best || area * sticky > best.score) best = { color: o.color, x: u, y: v, area, score: area * sticky };
    }
    this.det = best;
    this.pub('/color_detector', '/color_detection', 'snobots_msgs/msg/ColorDetection', { header: { stamp: stampNow(), frame_id: 'camera_link' }, color: best ? best.color : '', found: !!best, x: best ? +best.x.toFixed(3) : 0, y: best ? +best.y.toFixed(3) : 0, area: best ? +best.area.toFixed(5) : 0 });
    if (this.trk.on && best && best.color === 'red' && Math.abs(best.x) < 0.15 && Math.abs(best.y) < 0.2) { this.trk.held += 1 / 15; if (this.trk.held >= 3) this.complete('m6'); } else this.trk.held = 0;
  }
  buildWorld() {
    const F = AINEX_FIELD, W = this.world, gy = F.goalY, gd = F.goalD, gh = F.goalH, cx = (F.x0 + F.x1) / 2;
    // boards around the pitch
    W.addBox([cx, F.y1 + 0.02, 0.025], [F.x1 - F.x0 + 0.08, 0.02, 0.05]); W.addBox([cx, F.y0 - 0.02, 0.025], [F.x1 - F.x0 + 0.08, 0.02, 0.05]); W.addBox([F.x0 - 0.02, 0, 0.025], [0.02, F.y1 - F.y0, 0.05]);
    W.addBox([F.x1 + 0.02, (gy + F.y1) / 2 + 0.01, 0.025], [0.02, F.y1 - gy, 0.05]); W.addBox([F.x1 + 0.02, -(gy + F.y1) / 2 - 0.01, 0.025], [0.02, F.y1 - gy, 0.05]);
    // goal: posts, crossbar and net
    [[F.x1, gy], [F.x1, -gy], [F.x1 + gd, gy], [F.x1 + gd, -gy]].forEach(([x, y]) => W.addCyl(x, y, 0.011, gh, { mu: 0.4, rest: 0.3 }));
    W.addBox([F.x1 + gd + 0.006, 0, gh / 2], [0.012, 2 * gy, gh], 0, { mu: 0.9 }); W.addBox([F.x1 + gd / 2, gy + 0.006, gh / 2], [gd, 0.012, gh], 0, { mu: 0.9 }); W.addBox([F.x1 + gd / 2, -gy - 0.006, gh / 2], [gd, 0.012, gh], 0, { mu: 0.9 });
    W.addBox([F.x1 + gd / 2, 0, gh + 0.006], [gd + 0.02, 2 * gy + 0.02, 0.012]);
  }
  // the hoist sets AiNex down inside the pitch, clear of the boards and the goal
  hoistSpot(x, y) { const F = AINEX_FIELD; return [clampN(x, F.x0 + 0.2, F.x1 - 0.25), clampN(y, F.y0 + 0.2, F.y1 - 0.2)]; }
  objTick(o) {
    const F = AINEX_FIELD;
    if (o.kind === 'ball' && o.x > F.x1 + o.r && Math.abs(o.y) < F.goalY && !this.scored) {
      this.scored = true; this.goals++; Sound.play('coin'); confetti(60); this.event('goal');
      toast('<b>Goal!</b> AiNex scores.', { mood: 'wow', ms: 2200 });
      setTimeout(() => { this.placeObj(o, 0.45, 0); this.scored = false; }, 2200);
    }
    if (o.z < -0.5 || Math.abs(o.x) > 5 || Math.abs(o.y) > 5) this.placeObj(o, 0.45, 0);
  }
  hudExtra() { const d = this.det; return `<span>camera: ${d ? `${esc(d.color)} at x ${d.x.toFixed(2)}` : 'no target'}</span><span>goals ${this.goals}</span>`; }
  extraControls(w) {
    w.append(h('h4', { html: 'Vision <span>head camera → <code>/color_detection</code></span>' }));
    w.append(snoSwitch('Head tracking', this.trk.on, (on) => { this.setTracker(on); this.event('behavior', { k: 'track', on }); }, 'Node <code>/head_tracker</code>: turns the head to keep the blob centred.'));
    w.append(snoSwitch('Nod / shake reaction', this.rx.on, (on) => { this.setReactor(on); this.event('behavior', { k: 'react', on }); }, 'Node <code>/color_reactor</code>: nods at red, shakes at green or blue.'));
    const cur = String(this.P('/color_detector', 'target_color'));
    w.append(h('div', { class: 'seg-label', html: 'target_color' }), snoSeg([['red', 'Red'], ['green', 'Green'], ['blue', 'Blue'], ['any', 'Any']], cur, (v) => { this.setP('/color_detector', 'target_color', v); this.event('param', { node: '/color_detector', k: 'target_color', v }); SnoUI.refreshTab('control'); }, 'Target colour'));
    const rb = h('button', { class: 'btn btn-sm', html: icon('replay') + 'Reset ball' }); rb.addEventListener('click', () => { const b = this.objs.find(o => o.kind === 'ball'); if (b) this.placeObj(b, 0.45, 0); });
    w.append(h('div', { class: 'row-btns', style: 'margin-top:10px' }, rb));
  }
  missionTick() {
    const s = this.stats;
    if (s.dist >= 1.0) this.complete('m1');
    if (s.turn >= Math.PI) this.complete('m2');
    if (s.side >= 0.3) this.complete('m3');
    if (s.cliDist >= 0.5) this.complete('m8');
    if (s.userDist >= 0.3 || s.userJoint >= 15) this.complete('m10');
  }
  event(type, data) {
    if (type === 'action' && data === 'wave') this.complete('m4');
    else if (type === 'rec_play' && data >= 2) this.complete('m5');
    else if (type === 'goal') this.complete('m7');
    else if (type === 'fall') this.fellOnce = true;
    else if (type === 'stood_up' && this.fellOnce) this.complete('m9');
    else if (type === 'joint_cmd' && data.kind === 'user') this.stats.userJoint++;
  }
  buildEnv(view, T) {
    const F = AINEX_FIELD, m = 0.18, W = F.x1 - F.x0 + 2 * m, Hh = F.y1 - F.y0 + 2 * m, mats = view.mats;
    const tex = snoTex(T, 1400, Math.round(1400 * Hh / W), (g, cw, ch) => {
      const s = cw / W, X = (x) => (x - F.x0 + m) * s, Y = (y) => (F.y1 + m - y) * s;
      g.fillStyle = '#2F8F5B'; g.fillRect(0, 0, cw, ch);
      for (let i = 0; i < 12; i++) { g.fillStyle = i % 2 ? '#2C8756' : '#33975F'; g.fillRect(X(F.x0 + i * 0.2), Y(F.y1), 0.2 * s, (F.y1 - F.y0) * s); }
      g.strokeStyle = '#F4F7F2'; g.lineWidth = 0.018 * s; g.lineJoin = 'round';
      g.strokeRect(X(F.x0), Y(F.y1), (F.x1 - F.x0) * s, (F.y1 - F.y0) * s);
      const cx = (F.x0 + F.x1) / 2; g.beginPath(); g.moveTo(X(cx), Y(F.y1)); g.lineTo(X(cx), Y(F.y0)); g.stroke();
      g.beginPath(); g.arc(X(cx), Y(0), 0.25 * s, 0, 7); g.stroke();
      g.strokeRect(X(F.x1 - 0.32), Y(0.45), 0.32 * s, 0.9 * s); g.strokeRect(X(F.x0), Y(0.45), 0.32 * s, 0.9 * s);
      g.fillStyle = '#F4F7F2'; g.beginPath(); g.arc(X(F.x1 - 0.55), Y(0), 0.016 * s, 0, 7); g.fill(); g.beginPath(); g.arc(X(cx), Y(0), 0.016 * s, 0, 7); g.fill();
    });
    const field = new T.Mesh(new T.PlaneGeometry(W, Hh), new T.MeshStandardMaterial({ map: tex, roughness: 0.96 })); field.position.set((F.x0 + F.x1) / 2, 0, 0.001); field.receiveShadow = true; view.world.add(field);
    const board = (x, y, w, d) => { const b = snoMesh(T, { s: 'box', size: [w, d, 0.05], xyz: [x, y, 0.025], m: 'white' }, mats); view.world.add(b); };
    board((F.x0 + F.x1) / 2, F.y1 + 0.02, F.x1 - F.x0 + 0.08, 0.02); board((F.x0 + F.x1) / 2, F.y0 - 0.02, F.x1 - F.x0 + 0.08, 0.02); board(F.x0 - 0.02, 0, 0.02, F.y1 - F.y0);
    board(F.x1 + 0.02, (F.goalY + F.y1) / 2 + 0.01, 0.02, F.y1 - F.goalY); board(F.x1 + 0.02, -(F.goalY + F.y1) / 2 - 0.01, 0.02, F.y1 - F.goalY);
    const gx = F.x1, gd = F.goalD, gh = F.goalH, gyy = F.goalY;
    [[gx, gyy], [gx, -gyy], [gx + gd, gyy], [gx + gd, -gyy]].forEach(([x, y]) => view.world.add(snoMesh(T, { s: 'cyl', size: [0.011, gh], xyz: [x, y, gh / 2], m: 'white' }, mats)));
    view.world.add(snoMesh(T, { s: 'cyl', size: [0.011, 2 * gyy], ax: 'y', xyz: [gx, 0, gh], m: 'white' }, mats));
    const net = new T.MeshStandardMaterial({ color: 0xffffff, transparent: true, opacity: 0.32, side: T.DoubleSide, roughness: 1 });
    const pl = (w, hh2, x, y, z, rx, ry) => { const p = new T.Mesh(new T.PlaneGeometry(w, hh2), net); p.position.set(x, y, z); p.rotation.set(rx, ry, 0); view.world.add(p); };
    pl(2 * gyy, gh, gx + gd, 0, gh / 2, Math.PI / 2, Math.PI / 2); pl(gd, gh, gx + gd / 2, gyy, gh / 2, Math.PI / 2, 0); pl(gd, gh, gx + gd / 2, -gyy, gh / 2, Math.PI / 2, 0); pl(gd, 2 * gyy, gx + gd / 2, 0, gh, 0, 0);
    this.objMesh = this.objs.map(o => {
      const g = o.kind === 'ball' ? snoMesh(T, { s: 'sph', size: [o.r], m: 'orange' }, mats) : snoMesh(T, { s: 'box', size: [o.size, o.size, o.size], m: o.color }, mats);
      if (o.kind === 'ball') { const st = new T.Mesh(new T.TorusGeometry(o.r * 0.999, o.r * 0.12, 8, 28), mats.white); g.children[0].add(st); const st2 = st.clone(); st2.rotation.y = Math.PI / 2; g.children[0].add(st2); }
      view.world.add(g); return g;
    });
  }
  syncEnv() {
    if (!this.objMesh) return;
    this.objMesh.forEach((m, i) => { const S = this.objs[i].sys; m.position.set(S.pos[0], S.pos[1], S.pos[2]); m.quaternion.set(S.quat[0], S.quat[1], S.quat[2], S.quat[3]); });
  }
}

class R1Sim extends HumanoidSim {
  constructor(def) { super(def); this.loco = 'walk'; this.pdReset(); this.pdSel = 'left_shoulder_pitch_joint'; this.hold = 0; }
  gait() { return this.loco === 'run' ? this.H.run : this.H.gait; }
  reset() { this.loco = 'walk'; super.reset(); }
  setupExtra() {
    const c = this.ctl;
    this.bus.subscribe(c, '/lowcmd', 'snobots_msgs/msg/LowCmd', (m, from) => this.onLowCmd(m, from));
    this.bus.advertise(c, '/lowstate', 'snobots_msgs/msg/LowState');
    this.bus.service('/loco/set_mode', 'snobots_msgs/srv/SetMode', c, (rq) => { const r = this.setMode(String(rq.mode || '').trim().toLowerCase()); this.event('service', '/loco/set_mode'); return { success: !!r.ok, message: r.ok ? `mode is now ${this.locoStateRaw()}` : r.reason }; });
  }
  stateString() { return this.mode === 'act' && this.act ? 'action:' + this.act.name : this.locoStateRaw(); }
  locoStateRaw() { return this.mode === 'low' ? 'lowlevel' : this.mode === 'damp' ? 'damp' : this.mode === 'fallen' || this.mode === 'fall' ? 'fallen' : this.mode === 'getup' ? 'standing_up' : this.mode === 'manual' ? 'servo' : this.loco === 'run' ? (this.mode === 'walk' ? 'running' : 'run_ready') : (this.mode === 'walk' ? 'walking' : 'standing'); }
  setMode(m) {
    const alias = { stand_up: 'stand', standup: 'stand', lowlevel: 'low', low_level: 'low', debug: 'low', locomotion: 'walk', zero_torque: 'damp', passive: 'damp' };
    m = alias[m] || m;
    if (!['damp', 'stand', 'walk', 'run', 'low'].includes(m)) return { ok: false, reason: `unknown mode '${m}': use damp, stand, walk, run or lowlevel` };
    const standing = ['stand', 'walk', 'act', 'manual'].includes(this.mode);
    if (m === 'damp') {
      if (this.mode === 'damp') return { ok: true };
      if (this.mode === 'getup') return { ok: false, reason: 'wait until the robot stops moving' };
      this.gantry = null; this.act = null; this.ovl = null; this.queue = null; this.loco = 'walk'; this.mode = 'damp'; this.event('damp'); return { ok: true };
    }
    if (m === 'low') { this.enterLow(); return { ok: true }; }
    if (m === 'stand' || m === 'walk') {
      if (this.mode === 'low') { this.loco = 'walk'; this.startHoist(true); return { ok: true }; }
      if (this.loco === 'run') { this.loco = 'walk'; this.armsHome(); }
      if (['damp', 'fallen', 'fall', 'manual'].includes(this.mode)) return this.standUp();
      return { ok: true };
    }
    if (m === 'run') {
      if (!standing) return { ok: false, reason: 'stand up first (mode stand)' };
      if (this.mode === 'manual') this.goStand();
      this.loco = 'run'; ['L.ep', 'R.ep'].forEach(k => { this.qt[this.jn(k)] = -1.35; }); ['L.sr', 'R.sr'].forEach(k => { this.qt[this.jn(k)] = k[0] === 'L' ? 0.2 : -0.2; });
      return { ok: true };
    }
    return { ok: false, reason: 'unsupported' };
  }
  armsHome() { Object.entries(HU_ARMS_HOME).forEach(([k, v]) => { const n = this.jn(k); if (n) this.qt[n] = v; }); }
  // low-level mode: R1 hangs from the gantry rope and every motor runs the user's PD loop
  enterLow() {
    if (this.mode === 'low') return;
    this.act = null; this.ovl = null; this.queue = null; this.loco = 'walk';
    const p = this.hookPos(), yaw = this.nav.yaw;
    this.gantry = { ax: p[0], ay: p[1], az: this.hookZ + this.H.hangLift + 0.05, yaw, L: 0.05 };
    this.names.forEach(n => { this.pd.qd[n] = this.q[n]; this.pd.dqd[n] = 0; this.pd.tff[n] = 0; });
    if (['fallen', 'damp', 'fall'].includes(this.mode)) this.names.forEach(n => { this.pd.qd[n] = this.home[n]; });
    this.mode = 'low'; this.event('low');
  }
  goStand() { if (this.mode === 'low') { this.startHoist(true); return { ok: true }; } return super.goStand(); }
  pdReset() {
    const pd = this.pd = { kp: {}, kd: {}, qd: {}, dqd: {}, tff: {}, tau: {}, tg: {} };
    this.names.forEach(n => { const leg = this.legSet.has(n), small = /wrist|head|ankle_roll/.test(n); pd.kp[n] = leg ? 80 : small ? 20 : 40; pd.kd[n] = leg ? 2.5 : small ? 0.6 : 1.2; pd.qd[n] = this.q[n] || this.home[n] || 0; pd.dqd[n] = 0; pd.tff[n] = 0; pd.tau[n] = 0; pd.tg[n] = 0; });
  }
  lowCtl(h) {
    const rb = this.rb, pd = this.pd, g = this.gantry, m = rb.totalMass;
    for (const n of this.names) { const k = rb.ji[n]; rb.mode[k] = 1; rb.kp[k] = pd.kp[n]; rb.kd[k] = pd.kd[n]; rb.qt[k] = pd.qd[n]; rb.vt[k] = pd.dqd[n]; rb.tff[k] = pd.tff[n]; }
    if (!g) return;
    // the rope: lifts the robot to hanging height, then holds it (pulls only)
    g.L = Math.max(0.02, g.L - 0.25 * h);
    const p = this.hookPos(), v = rb.ptVel(this.torsoB, p), dx = g.ax - p[0], dy = g.ay - p[1], dz = g.az - p[2], d = Math.hypot(dx, dy, dz), K = 300 * m, C = 2 * Math.sqrt(K * m) * 0.6;
    if (d > g.L) { const ux = dx / d, uy = dy / d, uz = dz / d, vn = v[0] * ux + v[1] * uy + v[2] * uz, f = Math.max(0, K * (d - g.L) - C * vn); rb.push(this.torsoB, this.hookLocal(), [f * ux, f * uy, f * uz]); }
    // harness damping keeps the hanging robot from spinning
    rb.pushTorque(0, [-2 * m * rb.v[0], -2 * m * rb.v[1], -4 * m * rb.v[2]]);
  }
  readState(dt) {
    super.readState(dt);
    if (this.mode !== 'low' || !this.pd) return;
    const pd = this.pd, rb = this.rb;
    this.names.forEach(n => { const k = rb.ji[n]; pd.tau[n] = rb.tau[k]; pd.tg[n] = -rb.bias[6 + k]; });
    if (this.every('plot', dt, 1 / 30)) { const n = this.pdSel; this.plot.push([this.t, this.q[n], pd.qd[n]]); if (this.plot.length > 200) this.plot.shift(); }
    const n = 'left_shoulder_pitch_joint';
    if (Math.abs(this.q[n] + 1.57) < 0.06 && Math.abs(this.dq[n]) < 0.25) { this.hold += dt; if (this.hold >= 2) this.complete('m7'); } else this.hold = 0;
  }
  onLowCmd(m, from) {
    if (this.mode !== 'low') { this.lastWarn = '/lowcmd ignored: switch to Low-level mode first'; return; }
    const arr = m.motor_cmd || [], pd = this.pd;
    this.names.forEach((n, i) => { const c = arr[i]; if (!c || !(+c.mode >= 1)) return; pd.qd[n] = +c.q || 0; pd.dqd[n] = +c.dq || 0; pd.kp[n] = clampN(+c.kp || 0, 0, 500); pd.kd[n] = clampN(+c.kd || 0, 0, 50); pd.tff[n] = +c.tau || 0; });
    const nd = this.bus.nodes.get(from); if (nd && nd.user) { this.userLow = (this.userLow || 0) + 1; if (this.userLow >= 20) this.complete('m9'); }
    this.event('lowcmd', from);
  }
  publishExtra(dt) {
    if (this.every('low', dt, 0.04)) { this.lowTick = (this.lowTick || 0) + 1; this.pub(this.ctl, '/lowstate', 'snobots_msgs/msg/LowState', { tick: this.lowTick, motor_state: this.names.map(n => ({ q: +this.q[n].toFixed(4), dq: +(this.dq[n] || 0).toFixed(3), tau_est: +(this.tau[n] || 0).toFixed(2), temperature: 32 + Math.min(40, Math.round(Math.abs(this.tau[n] || 0) / 3)) })) }); }
  }
  /* ---- UI ---- */
  modesUI() {
    const wrap = h('div', { class: 'r1-modes' });
    wrap.append(h('h4', { html: `Mode <span>like the robot's state machine · <code>/loco/set_mode</code></span>` }));
    const cur = this.mode === 'low' ? 'low' : this.mode === 'damp' ? 'damp' : (this.mode === 'stand' || this.mode === 'walk') ? this.loco : '';
    wrap.append(snoSeg([['damp', 'Damp', 'stop'], ['stand', 'Stand', 'arrowU'], ['walk', 'Walk', 'drive'], ['run', 'Run', 'bolt'], ['low', 'Low-level', 'sliders']], cur, (v) => { const r = this.setMode(v); if (!r.ok) toast(esc(r.reason), { mood: 'think' }); else { Sound.play('tap'); this.event('ui_mode', v); } setTimeout(() => SnoUI.refreshTab('control'), 30); SnoUI.renderTools(); }, 'R1 mode'));
    wrap.append(h('p', { class: 'sidecap', html: { low: 'R1 hangs from the gantry and every motor runs a PD loop. Open the <b>Joints</b> tab to tune kp and kd, or publish <code>/lowcmd</code>.', damp: 'Damping mode: the motors only resist motion, so R1 slumps to the floor under gravity. Press <b>Stand</b> and the gantry hoist sets it back on its feet.' }[cur] || 'Drive with the joystick, keys or <code>/cmd_vel</code>. <b>Run</b> switches to a quicker step rhythm (in this sim up to about 0.55 m/s).' }));
    return wrap;
  }
  pdPanel() {
    const n = this.pdSel, j = this.kin.J[n], pd = this.pd, id = ++UID;
    const w = h('div', { class: 'pd-panel' });
    w.innerHTML = `<h4>PD motor control <span>τ = kp·(q_des − q) + kd·(dq_des − dq) + τ_ff</span></h4>
      <label class="mini">joint <select class="sel" id="pdj${id}">${this.names.map((x, i) => `<option value="${x}"${x === n ? ' selected' : ''}>${i} · ${x}</option>`).join('')}</select></label>
      <div class="sliders">
        <div class="slider"><label for="pk${id}">kp</label><input id="pk${id}" data-k="kp" type="range" min="0" max="200" step="1" value="${pd.kp[n]}"><output></output></div>
        <div class="slider"><label for="pdd${id}">kd</label><input id="pdd${id}" data-k="kd" type="range" min="0" max="8" step="0.1" value="${pd.kd[n]}"><output></output></div>
        <div class="slider"><label for="pq${id}">q_des</label><input id="pq${id}" data-k="qd" type="range" min="${j.lo}" max="${j.hi}" step="0.01" value="${pd.qd[n]}"><output></output></div>
        <div class="slider"><label for="pt${id}">τ_ff</label><input id="pt${id}" data-k="tff" type="range" min="-15" max="15" step="0.1" value="${pd.tff[n]}"><output></output></div>
      </div>
      <canvas class="pd-plot" width="560" height="150" aria-label="Plot of q and q_des over time"></canvas><div class="pd-read twist-readout"></div>`;
    $('select', w).addEventListener('change', (e) => { this.pdSel = e.target.value; this.plot = []; SnoUI.renderTab(); });
    $$('input[data-k]', w).forEach(inp => { inp.addEventListener('input', () => { pd[inp.dataset.k][this.pdSel] = +inp.value; this.event('pd_ui', inp.dataset.k); }); });
    return w;
  }
  tickPd(root) {
    const w = $('.pd-panel', root); if (!w || this.mode !== 'low') return;
    const n = this.pdSel, pd = this.pd;
    $$('input[data-k]', w).forEach(inp => { const v = pd[inp.dataset.k][n]; if (document.activeElement !== inp) inp.value = v; inp.nextElementSibling.textContent = inp.dataset.k === 'qd' ? `${(v * 57.3).toFixed(0)}°` : (+v).toFixed(inp.dataset.k === 'kp' ? 0 : 1); });
    const err = pd.qd[n] - this.q[n];
    $('.pd-read', w).innerHTML = `q <b>${this.q[n].toFixed(3)}</b> rad (${(this.q[n] * 57.3).toFixed(1)}°) · error <b>${(err * 57.3).toFixed(1)}°</b><br>motor τ <b>${pd.tau[n].toFixed(2)}</b> N·m · gravity τ <b>${pd.tg[n].toFixed(2)}</b> N·m${Math.abs(pd.tau[n]) >= this.rb.tmax[this.rb.ji[n]] - 0.05 ? ' · <span style="color:var(--term-yellow)">torque limit!</span>' : ''}`;
    const cv = $('.pd-plot', w), g = cv.getContext('2d'), W = cv.width, Hh = cv.height, P = this.plot;
    g.clearRect(0, 0, W, Hh); g.fillStyle = cssVar('--term-bg') || '#10201A'; g.fillRect(0, 0, W, Hh);
    if (P.length < 2) return;
    const j = this.kin.J[n], lo = Math.min(j.lo, ...P.map(p => Math.min(p[1], p[2]))), hi = Math.max(j.hi, ...P.map(p => Math.max(p[1], p[2])));
    const t0 = P[P.length - 1][0] - 6, X = (t) => (t - t0) / 6 * W, Y = (v) => Hh - 10 - (v - lo) / (hi - lo || 1) * (Hh - 20);
    g.strokeStyle = 'rgba(255,255,255,.1)'; g.lineWidth = 1; [0.25, 0.5, 0.75].forEach(f => { g.beginPath(); g.moveTo(0, Hh * f); g.lineTo(W, Hh * f); g.stroke(); });
    const line = (k, col, dash) => { g.strokeStyle = col; g.lineWidth = 2.2; g.setLineDash(dash || []); g.beginPath(); P.forEach((p, i) => { const x = X(p[0]), y = Y(p[k]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); g.setLineDash([]); };
    line(2, '#F5B700', [6, 5]); line(1, '#5EF2D6');
    g.font = '12px JetBrains Mono, monospace'; g.fillStyle = '#5EF2D6'; g.fillText('q', 8, 16); g.fillStyle = '#F5B700'; g.fillText('q_des', 26, 16); g.fillStyle = 'rgba(255,255,255,.55)'; g.fillText('last 6 s', W - 70, 16);
  }
  hudExtra() { return `<span>mode ${esc(this.locoStateRaw())}</span>`; }
  missionTick(dt) {
    const s = this.stats;
    if (s.dist >= 2) this.complete('m1');
    if (this.loco === 'run' && this.v[0] >= 0.45) this.complete('m2');
    if (s.cliDist >= 1) this.complete('m8');
    if (s.userDist >= 1) this.complete('m9');
    if (this.pushWatch && this.t > this.pushWatch) { this.pushWatch = 0; if (['stand', 'walk', 'act', 'manual'].includes(this.mode)) this.complete('m4'); }
    void dt;
  }
  event(type, data) {
    if (type === 'action_done' && ['dance', 'kungfu', 'wave', 'handshake', 'bow'].includes(data)) this.complete('m3');
    else if (type === 'push' && this.P(this.ctl, 'balance') !== false && data < 2) this.pushWatch = this.t + 2.5;
    else if (type === 'fall') this.pushWatch = 0;
    else if (type === 'damp') this.damped = true;
    else if (type === 'stood_up' && this.damped) this.complete('m5');
    else if (type === 'service' && data === '/loco/set_mode') this.complete('m6');
  }
  buildEnv(view, T) {
    const mats = view.mats;
    for (let i = 1; i <= 6; i++) for (const y of [-1.3, 1.3]) { const c = new T.Group(); c.add(snoMesh(T, { s: 'cone', size: [0.012, 0.075, 0.32], xyz: [0, 0, 0.16], m: 'orange' }, mats)); c.add(snoMesh(T, { s: 'box', size: [0.2, 0.2, 0.025], xyz: [0, 0, 0.0125], m: 'dark' }, mats)); c.add(snoMesh(T, { s: 'cyl', size: [0.052, 0.04], xyz: [0, 0, 0.17], m: 'white' }, mats)); c.position.set(i * 1.5, y, 0); view.world.add(c); }
    const line = (x, col) => view.world.add(snoMesh(T, { s: 'box', size: [0.07, 2.4, 0.004], xyz: [x, 0, 0.002], m: col }, mats));
    line(0, 'white'); line(3, 'yellow'); line(6, 'white');
    const pad = new T.Mesh(new T.CircleGeometry(0.6, 48), new T.MeshStandardMaterial({ color: new T.Color('#30A46C').convertSRGBToLinear(), transparent: true, opacity: 0.55, roughness: 1 })); pad.position.set(4.5, 3, 0.003); view.world.add(pad);
  }
  syncEnv() { }
}
