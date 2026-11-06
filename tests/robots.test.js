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

test('A hard push knocks AiNex over and the hoist stands it back up', () => {
  const sim = makeSim(L, 'ainex'); steps(sim, 1);
  sim.push(3); steps(sim, 3);
  assert(sim.mode === 'fallen', `still ${sim.mode} after a hard push`);
  const r = sim.standUp(); assert(r.ok, 'stand-up refused');
  steps(sim, 5);
  assert(sim.mode === 'stand' && deg(sim.tilt()) < 5, `after the hoist: ${sim.mode}, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
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

test('R1 collapses in damping mode and the gantry hoists it back up', () => {
  const sim = makeSim(L, 'r1'); steps(sim, 1);
  assert(sim.setMode('damp').ok, 'damp refused'); steps(sim, 3);
  assert(sim.Tr[11] < 0.4, `pelvis still at ${sim.Tr[11].toFixed(2)} m`);
  assert(sim.setMode('stand').ok, 'stand refused'); steps(sim, 5);
  assert(sim.Tr[11] > 0.6 && deg(sim.tilt()) < 10, `pelvis at ${sim.Tr[11].toFixed(2)} m, tilt ${deg(sim.tilt()).toFixed(1)} deg`);
});

test('R1 low-level mode: a PD command holds a shoulder joint on the gantry', () => {
  const sim = makeSim(L, 'r1'); steps(sim, 1);
  assert(sim.setMode('low').ok, 'low-level refused'); steps(sim, 2);
  const j = 'left_shoulder_pitch_joint'; sim.pd.kp[j] = 60; sim.pd.kd[j] = 2; sim.pd.qd[j] = -1.2; steps(sim, 3);
  const err = Math.abs(sim.q[j] + 1.2);
  assert(err < 0.12, `joint at ${sim.q[j].toFixed(3)} rad, target -1.2`);
  return `error ${deg(err).toFixed(1)} deg`;
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
