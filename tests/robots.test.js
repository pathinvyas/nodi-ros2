// Drives the four SnoBots simulators headless (the same classes the app runs) and checks
// what a robotics teacher would check: they stand, walk, recover, drive, navigate and pick.
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(); sim.step(DT); } };
const deg = (r) => r * 180 / Math.PI;
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/* ---------- AiNex: 24-joint small humanoid ---------- */
test('AiNex stands on its own under gravity', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 2);
  assert(sim.mode === 'stand', `mode is ${sim.mode}`);
  assert(deg(sim.tilt()) < 3, `leaning ${deg(sim.tilt()).toFixed(1)} deg`);
  return `tilt ${deg(sim.tilt()).toFixed(1)} deg`;
});

test('AiNex walks forward at 0.15 m/s and stays upright', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 1);
  const x0 = sim.nav.x, y0 = sim.nav.y; let maxTilt = 0;
  steps(sim, 4, () => { sim.teleop(0.15, 0, 0); maxTilt = Math.max(maxTilt, deg(sim.tilt())); });
  const d = Math.hypot(sim.nav.x - x0, sim.nav.y - y0);
  assert(d > 0.4, `only walked ${d.toFixed(2)} m in 4 s`);
  assert(maxTilt < 15, `leaned up to ${maxTilt.toFixed(1)} deg`);
  assert(sim.walker.stepsDone >= 6, `took ${sim.walker.stepsDone} steps`);
  return `${d.toFixed(2)} m, ${sim.walker.stepsDone} steps, max tilt ${maxTilt.toFixed(1)} deg`;
});

test('AiNex turns on the spot', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 1);
  const y0 = sim.nav.yaw; steps(sim, 3, () => sim.teleop(0, 0, 0.6)); steps(sim, 1, () => sim.teleop(0, 0, 0));
  const dy = deg(wrap(sim.nav.yaw - y0));
  assert(dy > 60, `turned only ${dy.toFixed(0)} deg`);
  return `${dy.toFixed(0)} deg`;
});

test('AiNex plays wave, bow and both kicks without falling', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 1);
  for (const a of ['wave', 'bow', 'left_kick', 'right_kick']) {
    const r = sim.playAction(a); assert(r.ok, `${a} refused: ${r.reason}`);
    steps(sim, 4); assert(sim.mode === 'stand', `after ${a} the robot is ${sim.mode}`);
  }
});

test('AiNex steps to catch a small push', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 1);
  sim.push(1); steps(sim, 3);
  assert(sim.mode !== 'fallen', 'fell over');
  return `${sim.walker.stepsDone} recovery steps`;
});

// a hard shove in a chosen direction (relative to the heading): 0 tips it onto its front, pi onto its back
const knock = (sim, rel, s = 3) => { const a = sim.nav.yaw + rel, F = sim.H.pushDv * s * sim.rb.totalMass / 0.05; sim.pushF = [F * Math.cos(a), F * Math.sin(a), 0]; sim.pushLeft = 0.05; };
const until = (sim, sec, cond) => { for (let t = 0; t < sec && !cond(); t += 0.25) steps(sim, 0.25); return cond(); };
test('A hard push knocks AiNex over and it stands back up by itself, from its back and from its front', () => {
  const out = [];
  for (const [rel, side] of [[Math.PI, 'back'], [0, 'front']]) {
    const sim = makeSim(L, 'ainex'), seen = []; steps(sim, 0.5);
    const ev = sim.event.bind(sim); sim.event = (t, d) => { seen.push(t === 'getup_from' ? t + ':' + d : t); return ev(t, d); };
    knock(sim, rel); assert(until(sim, 4, () => sim.mode === 'fallen'), `still ${sim.mode} after a hard push`);
    assert(sim.lyingOn() === side, `lies on its ${sim.lyingOn()}, expected its ${side}`);
    assert(until(sim, 14, () => sim.mode === 'stand' && seen.includes('stood_up')), `not up again: ${sim.mode}`);
    steps(sim, 1.5); assert(sim.mode === 'stand' && deg(sim.tilt()) < 5, `after standing up: ${sim.mode}, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
    assert(!sim.hoist && seen.includes(side === 'front' ? 'getup_roll' : 'getup_from:back'), `it should use its own moves (${seen.filter(x => /getup/.test(x))})`);
    out.push(`${side} ${sim.t.toFixed(1)} s`);
  }
  return out.join(', ');
});
test('AiNex stands up from its back at every heading (12 falls), at the first try', () => {
  const firsts = [];
  for (let k = 0; k < 12; k++) {
    const sim = makeSim(L, 'ainex'); sim.teleport(-0.2, 0, k * Math.PI / 6); steps(sim, 0.25 + 0.04 * (k % 3));
    let tries = 0; const ev = sim.event.bind(sim); sim.event = (t, d) => { if (t === 'act_start' && d === 'stand_up') tries++; return ev(t, d); };
    knock(sim, Math.PI, 2.7 + 0.3 * (k % 3));
    assert(until(sim, 16, () => sim.mode === 'stand' && tries > 0), `heading ${k * 30} deg: ${sim.mode} after ${tries} tries`);
    firsts.push(tries);
  }
  assert(firsts.every(t => t === 1), `tries per fall: ${firsts}`);
});
test('With auto_stand_up off AiNex stays down until /stand_up is called; then it gets up on its own', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 0.5); sim.setP(sim.ctl, 'auto_stand_up', false);
  knock(sim, Math.PI); until(sim, 4, () => sim.mode === 'fallen'); steps(sim, 3);
  assert(sim.mode === 'fallen', `mode ${sim.mode}`);
  const r = sim.bus.services.get('/stand_up').handler({}); assert(r.success, JSON.stringify(r));
  assert(until(sim, 10, () => sim.mode === 'stand'), `mode ${sim.mode}`);
  assert(sim.bus.services.get('/stand_up').handler({}).success === false, 'a standing robot refuses to stand up again');
});
test('Walking into the pitch boards, AiNex stops and steps back instead of tripping (bump_reflex)', () => {
  const run = (on) => {
    const sim = makeSim(L, 'ainex'); sim.setP(sim.ctl, 'bump_reflex', on); sim.setP(sim.ctl, 'auto_stand_up', false);
    sim.teleport(0.3, 0.6, Math.PI / 2); steps(sim, 0.3); let fell = false;
    steps(sim, 6, () => { sim.teleop(0.18, 0, 0); if (sim.mode === 'fall') fell = true; });
    return { sim, fell };
  };
  const off = run(false); assert(off.fell, 'without the reflex it should trip over the board');
  const on = run(true), s = on.sim;
  assert(!on.fell && s.bumps === 1 && s.nav.y < 0.8 && s.nav.y > 0.6, `with the reflex: fell ${on.fell}, bumps ${s.bumps}, y ${s.nav.y.toFixed(2)}`);
  assert(s.logs.some(l => /hit something solid: stopping and stepping back/.test(l[1])), 'the controller says why it stopped');
  const gap = 0.9 - s.nav.y; steps(s, 4, () => s.teleop(0, 0, 0.6)); steps(s, 4, () => s.teleop(0.15, 0, 0));
  assert(s.mode !== 'fall' && s.mode !== 'fallen' && s.nav.y < 0.75, `turned away and walked on: y ${s.nav.y.toFixed(2)}, ${s.mode}`);
  return `stopped ${(gap * 100).toFixed(0)} cm from the board`;
});

/* ---------- Unitree R1: 25 kg humanoid ---------- */
test('R1 walks at 0.4 m/s', () => {
  const sim = makeSim(L, 'r1'); steps(sim, 1);
  const x0 = sim.nav.x, y0 = sim.nav.y; let maxTilt = 0;
  steps(sim, 5, () => { sim.teleop(0.4, 0, 0); maxTilt = Math.max(maxTilt, deg(sim.tilt())); });
  const d = Math.hypot(sim.nav.x - x0, sim.nav.y - y0);
  assert(d > 1.4, `only walked ${d.toFixed(2)} m in 5 s`);
  assert(maxTilt < 15, `leaned up to ${maxTilt.toFixed(1)} deg`);
  return `${d.toFixed(2)} m in 5 s`;
});

test('R1 walks at the speed it is told, forwards, backwards and sideways', () => {
  const out = [];
  for (const [vx, vy] of [[0.2, 0], [0.45, 0], [-0.15, 0], [0, 0.15]]) {
    const sim = makeSim(L, 'r1'); steps(sim, 0.3); steps(sim, 2, () => sim.teleop(vx, vy, 0));
    const x0 = sim.nav.x, y0 = sim.nav.y; steps(sim, 5, () => sim.teleop(vx, vy, 0));
    const v = vx ? (sim.nav.x - x0) / 5 : (sim.nav.y - y0) / 5, want = vx || vy;
    assert(Math.abs(v - want) < 0.05 * Math.abs(want) + 0.005, `asked for ${want} m/s, walked ${v.toFixed(3)} m/s`);
    out.push(`${want}: ${v.toFixed(3)}`);
  }
  return out.join(', ');
});

test('R1 runs at full speed, and turns while running, without falling', () => {
  const out = [];
  for (const [vx, wz] of [[0.55, 0], [0.55, 0.8], [0.5, 1.0]]) {
    const sim = makeSim(L, 'r1'); steps(sim, 0.3); assert(sim.setMode('run').ok, 'run refused');
    steps(sim, 1.5, () => sim.teleop(vx, 0, wz)); const x0 = sim.nav.x;
    let fell = false; steps(sim, 6, () => { sim.teleop(vx, 0, wz); if (sim.mode === 'fall') fell = true; });
    assert(!fell && sim.mode === 'walk', `running ${vx} m/s, turning ${wz} rad/s: ${sim.mode}`);
    if (!wz) { const v = (sim.nav.x - x0) / 6; assert(Math.abs(v - vx) < 0.04, `ran ${v.toFixed(3)} m/s, asked for ${vx}`); out.push(`${v.toFixed(2)} m/s`); }
  }
  return out.join(', ');
});

test('R1 collapses in damping mode and stands back up by itself (no gantry)', () => {
  const sim = makeSim(L, 'r1'), seen = []; steps(sim, 1);
  const ev = sim.event.bind(sim); sim.event = (t, d) => { seen.push(t === 'getup_from' ? t + ':' + d : t); return ev(t, d); };
  assert(sim.setMode('damp').ok, 'damp refused'); steps(sim, 3);
  assert(sim.Tr[11] < 0.4, `pelvis still at ${sim.Tr[11].toFixed(2)} m`);
  steps(sim, 2); assert(sim.mode === 'damp', `damp should wait for Stand, not get up by itself: ${sim.mode}`);
  assert(sim.setMode('stand').ok, 'stand refused'); let hoisted = false;
  assert(until(sim, 12, () => { hoisted = hoisted || !!sim.hoist; return sim.mode === 'stand'; }), `not up again: ${sim.mode}`);
  steps(sim, 1.5);
  assert(sim.Tr[11] > 0.6 && deg(sim.tilt()) < 10, `pelvis at ${sim.Tr[11].toFixed(2)} m, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
  assert(!hoisted && seen.includes('getup_from:back') && sim.done('m5'), `own moves ${!hoisted}, ${seen.filter(x => /getup/.test(x))}, m5 ${sim.done('m5')}`);
  return `stood up ${(sim.t - 7.5).toFixed(1)} s after Stand`;
});

test('R1 knocked over in 12 directions, standing, walking or running, stands up by itself at the first try', () => {
  const out = { back: 0, front: 0, side: 0 };
  for (let k = 0; k < 12; k++) {
    const sim = makeSim(L, 'r1'), mode = ['stand', 'walk', 'run'][k % 3]; sim.teleport(0, 0, k * 0.7); steps(sim, 0.4);
    if (mode !== 'stand') { if (mode === 'run') sim.setMode('run'); steps(sim, 1.4, () => sim.teleop(mode === 'run' ? 0.5 : 0.35, 0, k % 2 ? 0.3 : 0)); }
    let tries = 0, hoisted = false; const ev = sim.event.bind(sim); sim.event = (t, d) => { if (t === 'act_start' && d === 'stand_up') tries++; return ev(t, d); };
    knock(sim, k * Math.PI / 6 + 0.13, 2.2 + 0.25 * (k % 6)); sim.teleop(0, 0, 0);
    assert(until(sim, 6, () => sim.mode === 'fallen'), `direction ${k * 30} deg (${mode}): ${sim.mode} after the push`);
    out[sim.lyingOn()]++;
    assert(until(sim, 20, () => { hoisted = hoisted || !!sim.hoist; return sim.mode === 'stand' && tries > 0; }), `direction ${k * 30} deg (${mode}): ${sim.mode} after ${tries} tries`);
    steps(sim, 1.5); assert(sim.mode === 'stand' && deg(sim.tilt()) < 8 && tries === 1 && !hoisted, `direction ${k * 30} deg: ${sim.mode}, tilt ${deg(sim.tilt()).toFixed(1)}, ${tries} tries, hoist ${hoisted}`);
  }
  return `lay on its back ${out.back}x, front ${out.front}x, side ${out.side}x`;
});

test('Pushed onto its front, R1 pushes up, brings its knees under and stands (no roll over its head)', () => {
  const out = [];
  for (const s of [3.2, 3.3, 3.4]) {
    const sim = makeSim(L, 'r1'), seen = []; steps(sim, 0.5);
    const ev = sim.event.bind(sim); sim.event = (t, d) => { seen.push(t === 'getup_from' ? t + ':' + d : t); return ev(t, d); };
    knock(sim, 0, s); assert(until(sim, 6, () => sim.mode === 'fallen') && sim.lyingOn() === 'front', `push ${s}: lies on its ${sim.lyingOn()}`);
    const t0 = sim.t; let maxTilt = 0;
    assert(until(sim, 15, () => { if (sim.gu && sim.gu.phase === 'front') maxTilt = Math.max(maxTilt, deg(sim.tilt())); return sim.mode === 'stand'; }), `not up: ${sim.mode}`);
    assert(seen.includes('getup_from:front') && !seen.includes('getup_roll') && maxTilt < 135, `events ${seen.filter(x => /getup/.test(x))}, tilted ${maxTilt.toFixed(0)} deg`);
    out.push(`${(sim.t - t0).toFixed(1)} s`);
  }
  return out.join(', ');
});

test('R1 lying on its side (left or right) tips over first, then stands up', () => {
  for (const sg of [1, -1]) {
    const sim = makeSim(L, 'r1'), seen = []; steps(sim, 0.2);
    const ev = sim.event.bind(sim); sim.event = (t, d) => { seen.push(t === 'getup_from' ? t + ':' + d : t); return ev(t, d); };
    const a = -sg * Math.PI / 4; sim.rb.setPose([0, 0, 0.26], [Math.sin(a), 0, 0, Math.cos(a)]); sim.rb.zeroVel(); sim.startFall();
    assert(until(sim, 6, () => sim.mode === 'fallen') && sim.lyingOn() === 'side', `lies on its ${sim.lyingOn()} (${sim.mode})`);
    assert(until(sim, 25, () => sim.mode === 'stand'), `not up: ${sim.mode}`);
    steps(sim, 1.5); assert(sim.mode === 'stand' && deg(sim.tilt()) < 5, `${sim.mode}, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
    assert(seen.includes('getup_tip') && seen.filter(x => x === 'fall').length === 1, `events: ${seen.filter(x => /getup|fall/.test(x))}`);
    assert(sim.logs.some(l => /lying on its side: tipping over first/.test(l[1])), 'the controller says what it does');
  }
});

test('With auto_stand_up off R1 stays down until /loco/stand_up is called', () => {
  const sim = makeSim(L, 'r1'); steps(sim, 0.5); sim.setP(sim.ctl, 'auto_stand_up', false);
  knock(sim, Math.PI); until(sim, 5, () => sim.mode === 'fallen'); steps(sim, 3);
  assert(sim.mode === 'fallen' && sim.locoStateRaw() === 'fallen', `mode ${sim.mode}`);
  assert(sim.logs.some(l => /fall detected .* Call \/loco\/stand_up/.test(l[1])), 'the controller says how to get up');
  const r = sim.bus.services.get('/loco/stand_up').handler({}); assert(r.success, JSON.stringify(r));
  assert(until(sim, 12, () => sim.mode === 'stand'), `mode ${sim.mode}`);
});

test('R1 low-level mode: a PD command holds a shoulder joint on the gantry', () => {
  const sim = makeSim(L, 'r1'); steps(sim, 1);
  assert(sim.setMode('low').ok, 'low-level refused'); steps(sim, 2);
  const j = 'left_shoulder_pitch_joint'; sim.pd.kp[j] = 60; sim.pd.kd[j] = 2; sim.pd.qd[j] = -1.2; steps(sim, 3);
  const err = Math.abs(sim.q[j] + 1.2);
  assert(err < 0.12, `joint at ${sim.q[j].toFixed(3)} rad, target -1.2`);
  return `error ${deg(err).toFixed(1)} deg`;
});

test('Out of low-level mode the gantry waits until R1 hangs still, then sets it down standing (even after a long hang)', () => {
  const out = [];
  for (const hang of [1, 3, 9]) {
    const sim = makeSim(L, 'r1'); steps(sim, 1); sim.setMode('low'); steps(sim, hang);
    assert(sim.setMode('walk').ok, 'walk refused'); let fell = false;
    steps(sim, 6, () => { if (sim.mode === 'fall') fell = true; });
    assert(!fell && sim.mode === 'stand' && deg(sim.tilt()) < 3, `after hanging ${hang} s: ${sim.mode}, fell ${fell}, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
    out.push(`${hang} s`);
  }
  return `set down standing after hanging ${out.join(', ')}`;
});

/* ---------- AgileX LIMO: four wheel modes ---------- */
for (const mode of ['diff', 'tracked', 'ackermann', 'mecanum']) test(`LIMO drives 1 m straight in ${mode} mode`, () => {
  const sim = makeSim(L, 'limo'); sim.setMode(mode); steps(sim, 0.5);
  const x0 = sim.x, y0 = sim.y;
  steps(sim, 2, () => sim.teleop(0.5, 0, 0)); steps(sim, 0.6, () => sim.teleop(0, 0, 0));
  const d = Math.hypot(sim.x - x0, sim.y - y0);
  assert(Math.abs(d - 1) < 0.1, `moved ${d.toFixed(3)} m`);
  return `${d.toFixed(3)} m`;
});

test('LIMO Ackermann steering respects the 0.4 m minimum turning radius', () => {
  const sim = makeSim(L, 'limo'); sim.setMode('ackermann'); steps(sim, 0.5);
  const th0 = sim.th; steps(sim, 2, () => sim.teleop(0.3, 0, 1.2)); steps(sim, 0.6, () => sim.teleop(0, 0, 0));
  const dth = deg(wrap(sim.th - th0)), max = deg(0.3 / 0.4 * 2.6);
  assert(dth > 50 && dth < max, `turned ${dth.toFixed(0)} deg (geometry allows at most ${max.toFixed(0)})`);
  return `${dth.toFixed(0)} deg`;
});

test('LIMO mecanum wheels strafe sideways', () => {
  const sim = makeSim(L, 'limo'); sim.setMode('mecanum'); steps(sim, 0.5);
  const x0 = sim.x, y0 = sim.y, th0 = sim.th;
  steps(sim, 2, () => sim.teleop(0, 0.3, 0)); steps(sim, 0.5, () => sim.teleop(0, 0, 0));
  const dx = sim.x - x0, dy = sim.y - y0, side = -Math.sin(th0) * dx + Math.cos(th0) * dy, fwd = Math.cos(th0) * dx + Math.sin(th0) * dy;
  assert(side > 0.45 && Math.abs(fwd) < 0.1, `sideways ${side.toFixed(2)} m, forward ${fwd.toFixed(2)} m`);
  return `${side.toFixed(2)} m left`;
});

test('LIMO /odom tracks the true pose while the tyres grip', () => {
  const sim = makeSim(L, 'limo'); sim.setMode('ackermann'); steps(sim, 0.5);
  steps(sim, 1.6, () => sim.teleop(0.4, 0, 1.0)); steps(sim, 0.6, () => sim.teleop(0, 0, 0));
  const drift = sim.odomDrift();
  assert(sim.bumps === 0, 'hit something during the turn');
  assert(drift < 0.03, `odometry is ${(drift * 100).toFixed(1)} cm off after a clean turn`);
  assert(sim.bus.topics.get('/odom').count > 40, '/odom is not being published');
  return `${(drift * 100).toFixed(1)} cm off after a 90 degree arc`;
});

test('LIMO /odom runs away when the wheels spin against a wall (it comes from the encoders)', () => {
  const sim = makeSim(L, 'limo'); sim.setMode('diff');
  sim.teleport(1.6, 0.45, -Math.PI / 2); steps(sim, 0.3);
  const x0 = sim.x, y0 = sim.y, ox = sim.odom.x, oy = sim.odom.y;
  steps(sim, 3, () => sim.teleop(0.3, 0, 0)); steps(sim, 0.5, () => sim.teleop(0, 0, 0));
  const moved = Math.hypot(sim.x - x0, sim.y - y0), odom = Math.hypot(sim.odom.x - ox, sim.odom.y - oy);
  assert(moved < 0.35, `the robot moved ${moved.toFixed(2)} m, so it never reached the wall`);
  assert(odom > moved + 0.3, `odometry says ${odom.toFixed(2)} m, the robot moved ${moved.toFixed(2)} m`);
  assert(sim.bumps > 0, 'no bump was counted');
  return `moved ${moved.toFixed(2)} m, odometry says ${odom.toFixed(2)} m`;
});

test('LIMO navigates to a goal around the furniture', () => {
  const sim = makeSim(L, 'limo'); sim.setMode('diff'); sim.setBeh('nav', true);
  sim.sendGoal(4.6, 3.2);
  for (let t = 0; t < 40 && sim.navState !== 'arrived' && sim.navState !== 'no_path'; t += 0.5) steps(sim, 0.5);
  assert(sim.navState === 'arrived', `navigation state ${sim.navState}`);
  assert(Math.hypot(sim.x - 4.6, sim.y - 3.2) < 0.25, `stopped ${Math.hypot(sim.x - 4.6, sim.y - 3.2).toFixed(2)} m from the goal`);
  return `bumps ${sim.bumps}`;
});

/* ---------- LIMO Cobot: mobile base + 6-axis arm ---------- */
test('LIMO Cobot picks all three cubes and sorts them into the matching bins', () => {
  const sim = makeSim(L, 'cobot'); steps(sim, 1);
  sim.setDemo(true); let t = 0;
  while (sim.demo && t < 160) { steps(sim, 0.5); t += 0.5; }
  const wrong = sim.cubes.filter(c => c.placed !== c.color).map(c => `${c.color} in ${c.placed || 'no bin'}`);
  assert(!wrong.length, wrong.join(', '));
  return `${t.toFixed(0)} s, ${sim.bumps} bumps`;
});

run();
