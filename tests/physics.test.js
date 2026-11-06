// Checks the rigid-body engine against textbook answers.
const { loadModules, test, assert, run } = require('./harness');
const M = loadModules(['sno_kin.js', 'sno_phys.js'], ['K3', 'KinRobot', 'PhysWorld', 'physFromKin', 'physBox', 'physSphere']);
const { PhysWorld, physBox, physSphere } = M;

test('a dropped box comes to rest on the floor and stays put', () => {
  const W = new PhysWorld({ mu: 0.8 }), b = W.add(physBox('b', [0.1, 0.1, 0.1], 1));
  const q = [0.1, 0.05, 0.02, 1], n = Math.hypot(...q); b.setPose([0, 0, 0.5], q.map(v => v / n));
  for (let i = 0; i < 1500; i++) W.step();
  assert(Math.abs(b.pos[2] - 0.05) < 0.002, `rest height ${b.pos[2].toFixed(4)} m, expected 0.050`);
  assert(Math.hypot(...b.v) < 0.01, `still moving: |v| = ${Math.hypot(...b.v).toFixed(4)}`);
  const p0 = b.pos.slice(); for (let i = 0; i < 1000; i++) W.step();
  const drift = Math.hypot(b.pos[0] - p0[0], b.pos[1] - p0[1]);
  assert(drift < 1e-4, `creeps ${(drift * 1000).toFixed(3)} mm while resting`);
  return `z = ${b.pos[2].toFixed(4)} m`;
});

test('a sliding box stops where Coulomb friction says (mu = 0.5)', () => {
  const W = new PhysWorld({ mu: 0.5 }), b = W.add(physBox('b', [0.1, 0.1, 0.1], 1, { mu: 0.5 })); b.setPose([0, 0, 0.0505]);
  for (let i = 0; i < 50; i++) W.step();
  b.v[3] = 2; let t = 0; const x0 = b.pos[0];
  while (b.v[3] > 1e-3 && t < 2) { W.step(); t += W.h; }
  const d = b.pos[0] - x0, tExp = 2 / (0.5 * 9.81), dExp = 4 / (2 * 0.5 * 9.81);
  assert(Math.abs(t - tExp) < 0.03, `stopped after ${t.toFixed(3)} s, expected ${tExp.toFixed(3)}`);
  assert(Math.abs(d - dExp) < 0.03, `slid ${d.toFixed(3)} m, expected ${dExp.toFixed(3)}`);
  return `${d.toFixed(3)} m in ${t.toFixed(3)} s`;
});

test('a ball rolls without slipping and slows from rolling resistance', () => {
  const W = new PhysWorld({ mu: 0.8 }), s = W.add(physSphere('s', 0.035, 0.045, { roll: 0.0012, hollow: true, mu: 0.8 })); s.setPose([0, 0, 0.035]);
  for (let i = 0; i < 50; i++) W.step();
  s.v[3] = 1.0; s.v[1] = 1.0 / 0.035;
  for (let t = 0; t < 1; t += W.h) W.step();
  const vx = s.v[3], wr = s.v[1] * 0.035;
  assert(Math.abs(vx - wr) < 0.01, `slipping: v = ${vx.toFixed(3)}, w*r = ${wr.toFixed(3)}`);
  assert(vx > 0.6 && vx < 0.95, `speed after 1 s is ${vx.toFixed(3)} m/s`);
  return `v = ${vx.toFixed(3)} m/s after 1 s`;
});

test('an articulated body in free fall: centre of mass falls at g, momentum is kept', () => {
  const desc = { root: 'a', joints: [{ name: 'j1', parent: 'a', child: 'b', xyz: [0.2, 0, 0], axis: 'y' }, { name: 'j2', parent: 'b', child: 'c', xyz: [0.2, 0, 0], axis: 'z' }],
    visuals: { a: [{ s: 'box', size: [0.1, 0.1, 0.1] }], b: [{ s: 'box', size: [0.2, 0.05, 0.05], xyz: [0.1, 0, 0] }], c: [{ s: 'box', size: [0.2, 0.05, 0.05], xyz: [0.1, 0, 0] }] } };
  const S = M.physFromKin(new M.KinRobot(desc), { mass: 3, act: () => ({ mode: 0 }) });
  const W = new PhysWorld({ ground: false }); W.add(S); S.setPose([0, 0, 1]);
  S.v[0] = 0.3; S.v[2] = 0.5; S.v[6] = 2; S.v[7] = -3;
  const c0 = S.comState().slice();
  for (let i = 0; i < 250; i++) W.step();
  const c1 = S.comState(), az = (c1[5] - c0[5]) / (250 * W.h);
  assert(Math.abs(az + 9.81) < 1e-3, `vertical acceleration ${az.toFixed(5)}, expected -9.81`);
  assert(Math.abs(c1[3] - c0[3]) < 1e-3, `horizontal momentum drifted by ${(c1[3] - c0[3]).toExponential(2)}`);
  return `a = ${az.toFixed(4)} m/s²`;
});

test('a pendulum swings with the period 2*pi*sqrt(L/g)', () => {
  const desc = { root: 'base', joints: [{ name: 'hinge', parent: 'base', child: 'arm', xyz: [0, 0, 0], axis: 'y' }],
    visuals: { base: [{ s: 'box', size: [0.05, 0.05, 0.05] }], arm: [{ s: 'sph', size: [0.01], xyz: [0, 0, -0.5] }] } };
  const S = M.physFromKin(new M.KinRobot(desc), { masses: { arm: [1, [0, 0, -0.5]], base: [1, [0, 0, 0]] }, act: () => ({ mode: 0 }) });
  S.fixed = true; const W = new PhysWorld({ ground: false }); W.add(S); S.setPose([0, 0, 2]); S.q[0] = 0.1;
  let t = 0, last = S.q[0]; const cross = [];
  while (t < 6) { W.step(); t += W.h; if (last < 0 && S.q[0] >= 0) cross.push(t); last = S.q[0]; }
  const T = (cross[cross.length - 1] - cross[0]) / (cross.length - 1), Ti = 2 * Math.PI * Math.sqrt(0.5 / 9.81);
  assert(cross.length > 2 && Math.abs(T - Ti) / Ti < 0.01, `period ${T.toFixed(4)} s, expected ${Ti.toFixed(4)}`);
  return `T = ${T.toFixed(4)} s`;
});

run();
