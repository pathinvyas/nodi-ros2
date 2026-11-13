// R1 as a ROS graph of real processes: its bringup, a TF tree that only holds what is published, and a lazy RGB-D head
// camera that still feeds the robot's own person locator. Then its seven Fix it tickets (each fault really present,
// then fixed the way a ROS developer would) and its five vision missions, checked from the true state of the world.
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(i); sim.step(DT); } };
const fresh = () => { L.S.sno.m = {}; L.S.sno.fix = {}; return makeSim(L, 'r1'); };
const open = (id) => { const sim = fresh(); L.SnoFix.start(sim, id); return sim; };
// a terminal for `ros2 run`: the sim prints into it and parks the running process in T.proc
const term = () => { const T = { lines: [], proc: null, print(s) { T.lines.push(String(s)); }, stop() { if (T.proc) { const p = T.proc; T.proc = null; p.stop(); } }, ctx: { ui: { termState() {} } } }; return T; };
const ros2run = (sim, line) => { const a = line.trim().split(/\s+/), T = term(); assert(a[0] === 'ros2' && a[1] === 'run', 'not a ros2 run line'); sim.run(T, a[2], a.slice(3)); return T; };
// what `ros2 param set` does for a simulated node
const paramSet = (sim, node, k, v) => { const n = sim.bus.nodes.get(node), e = n && n.params.get(k); assert(e, `${node} has no parameter ${k}`); assert(!e.ro, `${k} is read-only`); e.v = v; const r = n.onParam ? n.onParam(k, v) : { ok: true }; assert(!r || r.ok !== false, 'param rejected'); };
const failing = (sim) => { const tk = L.SnoFix.ticket(sim); return tk.check(sim, sim.fix.stats).items.filter(x => !x[1]).map(x => x[0]); };
const logHas = (sim, re) => sim.logs.some(l => re.test(l[1]));
const solved = (sim, sec, every) => { for (let t = 0; t < sec && !sim.fix.solved; t += 0.5) steps(sim, 0.5, every); return sim.fix.solved; };
// stands in for the 3D view: counts renders and returns blank colour frames and a flat 2 m depth image
const fakeView = () => { const v = { calls: 0, depth: 0, capture() { v.calls++; return new Uint8Array(320 * 240 * 3); }, captureDepth() { v.depth++; return new Uint16Array(320 * 240).fill(2000); } }; return v; };
const withView = (sim) => { const V = fakeView(); sim.camCap = {}; return { V, step: (sec, every) => steps(sim, sec, (i) => { if (every) every(i); sim.cameraFrame(V); }) }; };
const TW = 'geometry_msgs/msg/Twist', JS = 'sensor_msgs/msg/JointState', PS = 'geometry_msgs/msg/PointStamped', IMG = 'sensor_msgs/msg/Image';
const twist = (v = 0, w = 0) => ({ linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } });
const user = (sim, name = '/my_node') => { if (!sim.bus.nodes.has(name)) sim.bus.addNode(name, 'user', { user: true }); return name; };
const near = (sim) => Math.hypot(sim.guest.x - sim.nav.x, sim.guest.y - sim.nav.y);
const NORMAL = ['/head_camera', '/person_locator', '/r1_controller', '/robot_state_publisher'];

/* ---------- the robot's software ---------- */
test('Bringup: four processes with their launch output; colour and depth camera_info at 15 Hz with the same intrinsics', () => {
  const sim = fresh(), B = sim.bus, ci = [], di = [];
  B.addNode('/t'); B.subscribe('/t', '/head_camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', (m) => ci.push(m)); B.subscribe('/t', '/head_camera/depth/camera_info', 'sensor_msgs/msg/CameraInfo', (m) => di.push(m));
  steps(sim, 2);
  const nodes = [...B.nodes.keys()].filter(n => n !== '/t').sort();
  assert(JSON.stringify(nodes) === JSON.stringify(NORMAL), `nodes ${nodes}`);
  assert(sim.logs.filter(l => /process started with pid/.test(l[1])).length === 4, 'four processes in the launch output');
  assert(logHas(sim, /depth 320x240 16UC1 \(millimetres, aligned to colour\)/), 'the camera says what its depth is');
  assert(ci.length >= 29 && ci.length <= 31 && di.length === ci.length, `${ci.length} colour and ${di.length} depth camera_info messages in 2 s`);
  const m = ci[ci.length - 1], d = di[di.length - 1];
  assert(m.k[0] === 160 && m.k[4] === 160 && m.k[2] === 159.5 && m.k[5] === 119.5 && JSON.stringify(d.k) === JSON.stringify(m.k), `K ${m.k}`);
  assert(m.header.frame_id === 'camera_optical_frame' && d.header.frame_id === 'camera_optical_frame' && m.width === 320 && m.height === 240, `${m.header.frame_id} ${m.width}x${m.height}`);
  const C = B.topics.get('/head_camera/color/image_raw'), D = B.topics.get('/head_camera/depth/image_raw');
  assert(C.pubs.get('/head_camera') === IMG && D.pubs.get('/head_camera') === IMG && C.subs.has('/person_locator') && D.subs.has('/person_locator'), 'the locator subscribes to colour and depth');
  assert(Math.abs(L.R1_CAM.hfov - Math.PI / 2) < 1e-9 && Math.abs(L.R1_CAM.vfov - 2 * Math.atan(0.75) * 180 / Math.PI) < 1e-9, 'a 90° horizontal view');
  return `${ci.length} frames in 2 s`;
});
test('TF holds only what is published: the optical frame from /tf_static, the head while robot_state_publisher runs', async () => {
  const sim = fresh(), B = sim.bus; steps(sim, 0.5);
  const T = sim.lookup('base_link', 'camera_optical_frame'), K = sim.kin.T, A = L.K3.mul(L.K3.inv(K.base_link), K.camera_optical_frame);
  assert(T && [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11].every(i => Math.abs(T[i] - A[i]) < 1e-9), 'the published optical frame is where the camera is');
  assert(Math.abs(K.camera_optical_frame[2] - 1) < 1e-6 && Math.abs(K.camera_optical_frame[11] - 1.144) < 0.01, 'its z axis looks forward, 1.14 m up');
  let tf = null; B.addNode('/late'); B.subscribe('/late', '/tf_static', 'tf2_msgs/msg/TFMessage', (m) => m.transforms.forEach(t => { if (t.child_frame_id === 'camera_optical_frame') tf = t; }));
  await new Promise(r => setTimeout(r, 5));
  const q = tf && tf.transform.rotation;
  assert(tf && tf.header.frame_id === 'camera_link' && [q.x, q.y, q.z, q.w].every((v, i) => Math.abs(v - [-0.5, 0.5, -0.5, 0.5][i]) < 1e-6), 'camera_link -> camera_optical_frame on /tf_static, z forward');
  sim.stopProc('rsp');
  assert(!sim.frames().includes('head_link') && !sim.lookup('base_link', 'camera_optical_frame'), 'without robot_state_publisher there are no joint frames');
  sim.startProc('rsp'); sim.stopProc('ctl');
  assert(!sim.frames().includes('odom') && !sim.lookup('odom', 'base_link'), 'without the controller there is no odometry');
  const old = fresh(); old.logs = []; old.restoreRobot({ oldUrdf: true }); steps(old, 0.2);
  assert(old.frames().includes('camera_link') && !old.frames().includes('camera_optical_frame') && !logHas(old, /got segment camera_optical_frame/), "last year's description has no optical frame");
  return `${sim.frames().length} frames`;
});
test('The camera renders only for subscribers that want pixels; the person locator places the visitor from every frame anyway', () => {
  const sim = fresh(), B = sim.bus, { V, step } = withView(sim), pts = [];
  B.addNode('/t'); B.subscribe('/t', '/person_position', PS, (m) => pts.push(m));
  step(1);
  assert(V.calls === 0 && V.depth === 0, `rendered ${V.calls} colour and ${V.depth} depth frames for the robot's own node`);
  assert(pts.length >= 14 && pts.length <= 16, `${pts.length} positions in 1 s`);
  const p = pts[pts.length - 1], S = sim.guestSurface(), w = L.K3.apply(sim.kin.T.base_link, p.point.x, p.point.y, p.point.z);
  assert(p.header.frame_id === 'base_link' && Math.hypot(w[0] - S[0], w[1] - S[1], w[2] - S[2]) < 0.01, `person at ${JSON.stringify(p.point)}`);
  B.addNode('/viewer'); const dimg = []; B.subscribe('/viewer', '/head_camera/depth/image_raw', IMG, (m) => dimg.push(m));
  step(1);
  assert(V.depth >= 14 && V.calls === 0 && dimg.length === V.depth, `${V.depth} depth renders (and ${V.calls} colour) for a depth subscriber`);
  const m = dimg[0], v = new DataView(m.data.buffer, m.data.byteOffset, m.data.byteLength).getUint16(2 * (120 * 320 + 160), true);
  assert(m.encoding === '16UC1' && m.step === 640 && m.data.length === 320 * 240 * 2 && m.is_bigendian === 0 && v === 2000, `${m.encoding}, step ${m.step}, centre ${v}`);
  sim.stopProc('cam'); pts.length = 0; step(1);
  assert(pts.length === 0, 'no images, no positions');
  return `placed within ${(Math.hypot(w[0] - S[0], w[1] - S[1], w[2] - S[2]) * 100).toFixed(1)} cm`;
});
test('The visitor strolls a loop, faces where they walk, and steps back when R1 comes close', () => {
  const sim = fresh(); steps(sim, 0.5);
  sim.guestMode = 'stroll'; const seen = new Set(); let moved = 0, px = sim.guest.x, py = sim.guest.y;
  steps(sim, 45, () => { seen.add(sim.guest.wp); moved += Math.hypot(sim.guest.x - px, sim.guest.y - py); px = sim.guest.x; py = sim.guest.y; });
  assert(seen.size === 4 && moved > 8, `waypoints ${[...seen]}, walked ${moved.toFixed(1)} m`);
  sim.guestMode = 'stand'; sim.guestReset(); sim.teleport(1.9, 0, 0); steps(sim, 1.5);
  assert(near(sim) >= 0.59, `R1 0.4 m away: the visitor should back off (now ${near(sim).toFixed(2)} m)`);
  return `${moved.toFixed(1)} m in 45 s`;
});
test('Follow me: R1 looks at the visitor and walks up to about a metre away, then stops', () => {
  const sim = fresh(); steps(sim, 0.5); sim.setFollow(true); steps(sim, 8);
  const uv = sim.guestUV(), d = near(sim);
  assert(sim.mode === 'stand' && d > 1.0 && d < 1.3 && uv && Math.abs(uv[0]) < 0.1 && Math.abs(uv[1]) < 0.15, `distance ${d.toFixed(2)}, image ${uv}`);
  sim.setFollow(false); assert(!sim.bus.nodes.has('/follow_me'), 'switching it off removes the node');
  return `${d.toFixed(2)} m from the visitor's centre`;
});

/* ---------- Fix it: every fault is real, and the usual fixes work ---------- */
test('Ticket "R1 doesn\'t see anyone": the camera driver died at start-up; restarting it brings the pictures back', () => {
  const sim = open('cam-down'); steps(sim, 1.5);
  assert(!sim.bus.nodes.has('/head_camera') && logHas(sim, /no stereo device on USB/) && logHas(sim, /process has died/), 'the driver should be dead, with its last words in the log');
  assert(failing(sim).length === 2 && sim.fix.stats.locC === 0 && near(sim) > 2.2, `failing: ${failing(sim)}`);
  const T = ros2run(sim, 'ros2 run r1_camera head_camera_node');
  assert(T.proc && sim.procs.cam, 'the driver should run in that terminal');
  assert(solved(sim, 4), `still failing: ${failing(sim)}`);
  steps(sim, 6); assert(near(sim) < 1.4, `follow me should walk to the visitor once R1 can see (${near(sim).toFixed(2)} m)`);
});
test('Ticket "R1 ignores the joystick": still in low-level mode; the controller says so, and /loco/set_mode walk fixes it', () => {
  const sim = open('mode'); steps(sim, 1);
  steps(sim, 3, (i) => { if (i % 6 === 0) sim.teleop(0.4, 0, 0); }); sim.teleop(0, 0, 0);
  assert(sim.locoStateRaw() === 'lowlevel' && sim.fix.stats.joy === 0 && logHas(sim, /Ignoring \/cmd_vel: R1 is in 'lowlevel' mode/), `state ${sim.locoStateRaw()}, joystick distance ${sim.fix.stats.joy}`);
  const r = sim.bus.services.get('/loco/set_mode').handler({ mode: 'walk' });
  assert(r.success && /standing_up/.test(r.message), JSON.stringify(r));
  steps(sim, 4); assert(sim.locoStateRaw() === 'standing', `after the hoist: ${sim.locoStateRaw()}`);
  assert(solved(sim, 5, (i) => { if (i % 6 === 0) sim.teleop(0.4, 0, 0); }), `still failing: ${failing(sim)}`);
});
test('Ticket "Head yes, feet no": follow me walks a robot that isn\'t there; a relay or a clean restart fixes it', () => {
  let sim = open('fm-remap'); steps(sim, 4);
  const uv = sim.guestUV();
  assert(uv && Math.abs(uv[0]) < 0.1 && near(sim) > 2.2 && sim.bus.topics.has('/r1_b/cmd_vel') && failing(sim).length === 1, `the head should look, the feet stay (${near(sim).toFixed(2)} m), failing ${failing(sim)}`);
  ros2run(sim, 'ros2 run topic_tools relay /r1_b/cmd_vel /cmd_vel');
  assert(solved(sim, 8), `relay: still failing: ${failing(sim)} (walked ${sim.fix.stats.fm.toFixed(2)} m)`);
  sim = open('fm-remap'); steps(sim, 1); sim.setFollow(false); sim.setFollow(true);
  assert(solved(sim, 8), `restart: still failing: ${failing(sim)}`);
});
test('Ticket "R1 looks away from people": head_kp has the wrong sign; flipping it keeps the visitor in view', () => {
  const sim = open('look-away'); steps(sim, 4);
  assert(sim.fix.stats.loc >= 5 && sim.fix.stats.head < 1.5 && !sim.fix.solved, `centred for ${sim.fix.stats.head.toFixed(2)} s in a row`);
  paramSet(sim, '/follow_me', 'head_kp', 0.6);
  assert(solved(sim, 8), `still failing: ${failing(sim)}`);
});
test('Ticket "Depth at 15 Hz, but nobody is found": BEST_EFFORT depth, RELIABLE locator; either side can give way', () => {
  let sim = open('depth-qos'); steps(sim, 4.5);
  assert(sim.fix.stats.locC > 30 && sim.fix.stats.loc === 0 && logHas(sim, /offering incompatible QoS/) && logHas(sim, /requesting incompatible QoS/) && logHas(sim, /No depth image on '\/head_camera\/depth\/image_raw'/), 'colour arrives, depth does not, and the log says why');
  ros2run(sim, 'ros2 run r1_camera head_camera_node --ros-args -p qos_overrides./head_camera/depth/image_raw.publisher.reliability:=reliable');
  assert(solved(sim, 4), `camera restart: still failing: ${failing(sim)}`);
  sim = open('depth-qos'); steps(sim, 1);
  ros2run(sim, 'ros2 run r1_vision person_locator --ros-args -p qos_overrides./head_camera/depth/image_raw.subscription.reliability:=best_effort');
  assert(solved(sim, 4), `locator restart: still failing: ${failing(sim)}`);
});
test('Ticket "The visitor is a kilometre away": depth_scale 1.0 reads millimetres as metres; 0.001 places the visitor', () => {
  const sim = open('depth-scale'); steps(sim, 2);
  assert(sim.fix.stats.loc === 0 && logHas(sim, /person seen 2\d{3} m away, beyond max_range \(5 m\)/), 'the locator should drop a visitor 2000 m away');
  paramSet(sim, '/person_locator', 'depth_scale', 0.001);
  assert(solved(sim, 4), `still failing: ${failing(sim)}`);
});
test('Ticket "A frame went missing": no optical frame in the old URDF; a bridge without the rotation is caught, the right one works', () => {
  const sim = open('optical'); steps(sim, 3.5);
  assert(logHas(sim, /Could not transform the person from 'camera_optical_frame' to 'base_link'/) && sim.fix.stats.loc === 0, 'the locator should complain and place nothing');
  const wrong = ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --frame-id camera_link --child-frame-id camera_optical_frame'); steps(sim, 2);
  const res = L.SnoFix.ticket(sim).check(sim, sim.fix.stats);
  assert(res.items[0][1] && !res.items[1][1] && /120° away from the real mount/.test(res.note) && sim.fix.stats.locOk === 0, `wrong bridge: ${JSON.stringify(res)}`);
  wrong.stop(); ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --roll -1.5708 --yaw -1.5708 --frame-id camera_link --child-frame-id camera_optical_frame');
  assert(solved(sim, 4), `static bridge: still failing: ${failing(sim)}`);
});
test('Closing a ticket restarts R1 normally', () => {
  const sim = open('fm-remap'); steps(sim, 0.5); L.SnoFix.quit(sim); steps(sim, 1);
  const nodes = [...sim.bus.nodes.keys()].filter(n => !n.startsWith('/_')).sort();
  assert(JSON.stringify(nodes) === JSON.stringify(NORMAL), `nodes ${nodes}`);
  assert(!sim.fm.on && !sim.fix && sim.frames().includes('camera_optical_frame'), 'follow me off, the full description back');
});
test('Every R1 ticket is self-consistent: an id, a level, hints, a fix and a debrief, and its setup really breaks something', () => {
  const T = L.FIX_TICKETS.r1; assert(T.length === 7 && new Set(T.map(t => t.id)).size === 7, `${T.length} tickets`);
  T.forEach(tk => {
    assert(tk.id && tk.level >= 1 && tk.level <= 3 && tk.hints.length === 3 && /^ros2 /.test(tk.fix) && tk.debrief.length > 120, `${tk.id} is incomplete`);
    const sim = open(tk.id); steps(sim, 1);
    assert(failing(sim).length > 0 && !sim.fix.solved, `${tk.id} starts solved`);
  });
});

/* ---------- vision missions ---------- */
test('v1 Camera check: ros2 topic hz on a camera image and an echo of a camera_info', () => {
  const sim = fresh();
  sim.event('cmd', 'echo:/head_camera/depth/camera_info'); assert(!sim.done('v1'), 'echo alone');
  sim.event('cmd', 'hz:/head_camera/color/image_raw'); assert(sim.done('v1'), 'hz + echo should count');
});
test('v2 How far away?: 10 distances within 10 cm from your node; wrong numbers or another node do not count', () => {
  const sim = fresh(), B = sim.bus, me = user(sim), F = 'std_msgs/msg/Float32'; B.addNode('/other'); B.advertise(me, '/visitor_distance', F); B.advertise('/other', '/visitor_distance', F); steps(sim, 0.5);
  const z = sim.guestUV()[2];
  for (let i = 0; i < 12; i++) { B.publish('/other', '/visitor_distance', F, { data: z }); B.publish(me, '/visitor_distance', F, { data: z * 1000 }); }
  assert(!sim.done('v2') && sim.vst.dist === 0, 'millimetres, or another node, should not count');
  for (let i = 0; i < 10; i++) B.publish(me, '/visitor_distance', F, { data: z + 0.06 });
  assert(sim.done('v2'), `${sim.vst.dist} good distances (true depth ${z.toFixed(2)} m)`);
  return `the vest is ${z.toFixed(2)} m deep in the picture`;
});
test('v3 Eyes on the visitor: your node keeps the strolling visitor centred by turning the head; no head work, no credit', () => {
  const run3 = (mine) => {
    const sim = fresh(), B = sim.bus, me = user(sim); B.advertise(me, '/joint_commands', JS); steps(sim, 0.5); sim.guestMode = 'stroll';
    steps(sim, 9, (i) => { const uv = sim.guestUV(); if (!mine || !uv || i % 4) return; B.publish(me, '/joint_commands', JS, { name: ['head_yaw_joint', 'head_pitch_joint'], position: [sim.q.head_yaw_joint - 0.4 * uv[0], sim.q.head_pitch_joint + 0.4 * uv[1]] }); });
    return sim;
  };
  const a = run3(true); assert(a.done('v3'), `only ${a.vst.watch.toFixed(1)} s of watching`);
  const b = run3(false); assert(!b.done('v3'), `counted ${b.vst.watch.toFixed(1)} s without head commands`);
  return `${a.vst.watch.toFixed(1)} s`;
});
test('v4 Follow me: a node like the template keeps R1 0.7 to 1.4 m from the strolling visitor for 10 s', () => {
  const sim = fresh(), B = sim.bus, me = user(sim); B.advertise(me, '/cmd_vel', TW); steps(sim, 0.5); sim.guestMode = 'stroll';
  // the template's logic with the true picture: turn toward the vest, walk until its depth is 0.8 m
  steps(sim, 40, (i) => { if (i % 4 || sim.done('v4')) return; const uv = sim.guestUV(); const t = uv ? twist(Math.abs(uv[0]) < 0.4 ? Math.max(-0.1, Math.min(0.35, 0.6 * (uv[2] - 0.8))) : 0, -1.0 * uv[0]) : twist(); B.publish(me, '/cmd_vel', TW, t); });
  assert(sim.done('v4'), `only ${sim.vst.near.toFixed(1)} s in the band`);
  const lazy = fresh(); lazy.guestMode = 'stroll'; steps(lazy, 20, () => lazy.teleop(0, 0, 0));
  assert(!lazy.done('v4') && lazy.vst.near === 0, 'standing still with the joystick should not count');
  return `in the band for ${sim.vst.near.toFixed(1)} s`;
});
test('v5 A point in 3D: back-projected points in the optical frame count; a point at the camera does not', () => {
  const sim = fresh(), B = sim.bus, me = user(sim); B.advertise(me, '/visitor_point', PS); steps(sim, 0.5);
  const S = sim.guestSurface(), p = L.K3.apply(L.K3.inv(sim.kin.T.camera_optical_frame), S[0], S[1], S[2]), hd = { stamp: { sec: 0, nanosec: 0 }, frame_id: 'camera_optical_frame' };
  for (let i = 0; i < 12; i++) B.publish(me, '/visitor_point', PS, { header: hd, point: { x: 0, y: 0, z: 0.1 } });
  for (let i = 0; i < 12; i++) B.publish(me, '/visitor_point', PS, { header: { stamp: hd.stamp, frame_id: 'camera_link' }, point: { x: p[0], y: p[1], z: p[2] } });
  assert(!sim.done('v5') && sim.vst.pts === 0, 'a point at the lens, or optical numbers in camera_link, should not count');
  for (let i = 0; i < 10; i++) B.publish(me, '/visitor_point', PS, { header: hd, point: { x: p[0] + 0.03, y: p[1] - 0.05, z: p[2] + 0.04 } });
  assert(sim.done('v5'), `${sim.vst.pts} good points`);
});
test('Starter code: the violet thresholds catch the vest and nothing else on the course; templates read R1\'s topics', () => {
  const sim = fresh(), py = sim.def.py, hex = (h) => L.SnoVision.hsv(parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16));
  const inR = (c, lo, hi) => c.every((v, i) => v >= lo[i] && v <= hi[i]);
  ['distance', 'head', 'follow', 'point'].forEach(k => {
    const code = py[k].code, lo = JSON.parse(code.match(/VEST_LO = np\.array\((\[[^\]]+\])\)/)[1]), hi = JSON.parse(code.match(/VEST_HI = np\.array\((\[[^\]]+\])\)/)[1]);
    assert(inR(hex(L.R1_VISITOR.color), lo, hi), `${k}: the vest ${hex(L.R1_VISITOR.color)} is outside ${lo}..${hi}`);
    ['#FF8A3D', '#F26419', '#30A46C', '#F5B700', '#F6F8F9', '#E9EEF0', '#2F6FD0', '#24324A', '#8E989E', '#2B3236', '#4d80b3', '#2a4560', '#1a3349', '#334d66'].forEach(c => assert(!inR(hex(c), lo, hi), `${k}: ${c} ${hex(c)} would look like the vest`));
    assert(/'\/head_camera\/color\/image_raw'/.test(code), `${k} should read the colour image`);
  });
  ['cam', 'distance', 'follow', 'point'].forEach(k => assert(/'\/head_camera\/depth\/image_raw'/.test(py[k].code), `${k} should read the depth image`));
  assert(/'\/visitor_distance'/.test(py.distance.code) && /'\/visitor_point'/.test(py.point.code) && /head_yaw_joint/.test(py.head.code), 'templates publish where the missions look');
  const ms = L.SnoVision.missions(sim); assert(ms.map(m => m.id).join() === 'v1,v2,v3,v4,v5' && ms.every(m => !m.tpl || py[m.tpl]), 'five vision missions with their templates');
});

run();
