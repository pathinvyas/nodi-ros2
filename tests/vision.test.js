// Vision lab: the simulated camera's ROS interface, its pinhole model and TF frame, the arena's markers and
// balls, and the five vision missions. Images need the 3D view, so here a stand-in view hands back a blank frame;
// the browser suite (OpenCV in Pyodide) checks real pixels.
const fs = require('fs'), path = require('path');
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const IMG = '/camera/color/image_raw', INFO = '/camera/color/camera_info', TW = 'geometry_msgs/msg/Twist', INT = 'std_msgs/msg/Int32';
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(); sim.step(DT); } };
// mission progress is kept per robot, so every test starts from a clean slate
const fresh = () => { L.S.sno.m = {}; return makeSim(L, 'limo'); };
const twist = (v = 0, w = 0) => ({ linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } });
// a node from your Python file (the Code tab marks it as yours); returns its /cmd_vel publisher
const userNode = (sim, name = '/my_node') => {
  const B = sim.bus; if (!B.nodes.has(name)) B.addNode(name, 'user', { user: true });
  B.advertise(name, '/cmd_vel', TW); return (v, w) => B.publish(name, '/cmd_vel', TW, twist(v, w));
};
// put the camera (not base_link) at x, y facing th: it sits 17 cm ahead of base_link
const placeCam = (sim, x, y, th) => sim.teleport(x - 0.17 * Math.cos(th), y - 0.17 * Math.sin(th), th);
// stands in for the 3D view: counts renders and returns a blank 320x240 rgb8 frame
const fakeView = () => { const v = { calls: 0, capture() { v.calls++; return new Uint8Array(320 * 240 * 3); } }; return v; };
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/* ---------- the camera driver ---------- */
test('camera_info: a 320x240 pinhole camera whose K matches the 56 degree vertical field of view, at 10 Hz', () => {
  const sim = fresh(), B = sim.bus, got = [];
  B.addNode('/t'); B.subscribe('/t', INFO, 'sensor_msgs/msg/CameraInfo', (m) => got.push(m));
  steps(sim, 2);
  const m = got[got.length - 1], f = 120 / Math.tan(28 * Math.PI / 180);
  assert(got.length >= 19 && got.length <= 21, `${got.length} messages in 2 s of sim time`);
  assert(m.width === 320 && m.height === 240, `size ${m.width}x${m.height}`);
  assert(Math.abs(m.k[0] - f) < 1e-3 && m.k[4] === m.k[0] && m.k[1] === 0 && m.k[3] === 0 && m.k[8] === 1, `K ${m.k}`);
  assert(m.k[2] === 159.5 && m.k[5] === 119.5, `principal point (${m.k[2]}, ${m.k[5]}): OpenCV puts pixel centres on integers`);
  assert(m.p[0] === m.k[0] && m.p[2] === m.k[2] && m.p[5] === m.k[4] && m.p[6] === m.k[5] && m.p[3] === 0 && m.p[10] === 1, `P ${m.p}`);
  assert(m.r.join() === '1,0,0,0,1,0,0,0,1' && m.distortion_model === 'plumb_bob' && m.d.length === 5 && m.d.every(x => x === 0), 'an ideal, undistorted camera');
  assert(m.header.frame_id === 'camera_color_optical_frame', `frame ${m.header.frame_id}`);
  return `fx = fy = ${m.k[0]} px`;
});
test('The optical frame: z forward, x right, y down, 17 cm ahead of base_link and at the markers\' height', async () => {
  const sim = fresh(), B = sim.bus; steps(sim, 0.5);
  const T = sim.lookup('base_link', 'camera_color_optical_frame'), R = [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
  [0, 0, 1, -1, 0, 0, 0, -1, 0].forEach((v, i) => assert(Math.abs(R[i] - v) < 1e-9, `rotation ${R.map(x => +x.toFixed(3))}`));
  assert(Math.abs(T[3] - 0.17) < 1e-9 && Math.abs(T[7]) < 1e-9 && Math.abs(T[11] - 0.085) < 1e-9, `offset ${T[3]}, ${T[7]}, ${T[11]}`);
  assert(Math.abs(sim.camPose()[2] - 0.13) < 0.005, `camera height ${sim.camPose()[2]}`);
  // robot_state_publisher latches it on /tf_static like any fixed joint
  let tf = null; B.addNode('/late'); B.subscribe('/late', '/tf_static', 'tf2_msgs/msg/TFMessage', (m) => m.transforms.forEach(t => { if (t.child_frame_id === 'camera_color_optical_frame') tf = t; }));
  await new Promise(r => setTimeout(r, 5));
  assert(tf && tf.header.frame_id === 'camera_link', 'camera_link -> camera_color_optical_frame missing from /tf_static');
  const q = tf.transform.rotation; [q.x, q.y, q.z, q.w].forEach((v, i) => assert(Math.abs(v - [-0.5, 0.5, -0.5, 0.5][i]) < 1e-6, `quaternion ${[q.x, q.y, q.z, q.w]}`));
});
test('Camera parameters: name and size are read-only, color_fps sets the rate, enable_color stops it', () => {
  const sim = fresh(), n = sim.bus.nodes.get('/camera/camera'); let got = 0;
  ['camera_name', 'color_width', 'color_height'].forEach(k => assert(n.params.get(k).ro, `${k} should be read-only`));
  [['camera_name', 'string'], ['color_width', 'integer'], ['color_height', 'integer'], ['color_fps', 'integer'], ['enable_color', 'bool']].forEach(([k, t]) => assert(n.params.get(k).t === t, `${k} is a ${n.params.get(k).t}, want ${t}`));
  assert(JSON.stringify(n.params.get('color_fps').range) === '[1,30]', `color_fps range ${n.params.get('color_fps').range}`);
  sim.bus.addNode('/t'); sim.bus.subscribe('/t', INFO, 'sensor_msgs/msg/CameraInfo', () => got++);
  n.params.get('color_fps').v = 5; steps(sim, 2); const at5 = got;
  n.params.get('enable_color').v = false; got = 0; steps(sim, 1);
  assert(Math.abs(at5 - 10) <= 1, `${at5} messages in 2 s at 5 fps`); assert(got === 0, `${got} messages with the colour stream off`);
});
test('Images: rendered only while something subscribes, one per camera_info, as rgb8 in the optical frame', () => {
  const sim = fresh(), B = sim.bus, V = fakeView(), imgs = []; sim.camCap = {};
  const frame = () => { steps(sim, 0.1); sim.cameraFrame(V); sim.cameraFrame(V); };
  for (let i = 0; i < 5; i++) frame();
  assert(V.calls === 0, `rendered ${V.calls} frames with nobody listening`);
  B.addNode('/viewer'); const un = B.subscribe('/viewer', IMG, 'sensor_msgs/msg/Image', (m) => imgs.push(m));
  for (let i = 0; i < 10; i++) frame();
  assert(imgs.length === 10 && V.calls === 10, `${imgs.length} images from ${V.calls} renders in 1 s`);
  const m = imgs[0];
  assert(m.encoding === 'rgb8' && m.width === 320 && m.height === 240 && m.step === 960 && m.is_bigendian === 0, `${m.encoding} ${m.width}x${m.height} step ${m.step}`);
  assert(m.data instanceof Uint8Array && m.data.length === 230400, `data ${m.data && m.data.length}`);
  assert(m.header.frame_id === 'camera_color_optical_frame', `frame ${m.header.frame_id}`);
  un(); const n0 = V.calls; for (let i = 0; i < 5; i++) frame();
  assert(V.calls === n0, 'kept rendering after the last subscriber left');
});
test('ros2 topic bw sizes: an image is about 230 kB, a Twist 48 bytes', () => {
  const lab = fs.readFileSync(path.join(__dirname, '..', 'src', 'lab.js'), 'utf8'), i0 = lab.indexOf('function msgBytes');
  const msgBytes = new Function(lab.slice(i0, lab.indexOf('\n}\n', i0) + 3) + 'return msgBytes;')();
  const img = { header: { stamp: { sec: 1, nanosec: 2 }, frame_id: 'camera_color_optical_frame' }, height: 240, width: 320, encoding: 'rgb8', is_bigendian: 0, step: 960, data: new Uint8Array(230400) };
  const b = msgBytes(img); assert(b > 230400 && b < 230500, `${b} bytes`);
  assert(msgBytes(twist(0.1, 0.2)) === 48, `Twist ${msgBytes(twist())} bytes`);
  return `${(b * 10 / 1e6).toFixed(2)} MB/s at 10 Hz`;
});

/* ---------- the arena ---------- */
test('ArUco markers: the DICT_4X4_50 codes OpenCV draws, each facing open floor', () => {
  // cv2.aruco.generateImageMarker(DICT_4X4_50, id, 6): the inner 4x4 cells, row by row, white = 1
  const OPENCV = { 0: 46386, 1: 3994, 2: 13101, 3: 39238, 4: 21662, 5: 31181, 7: 50418, 8: 65242, 12: 3767, 21: 45099 };
  assert(JSON.stringify(L.ARUCO_4X4) === JSON.stringify(OPENCV), `codes ${JSON.stringify(L.ARUCO_4X4)}`);
  const sim = fresh(), A = sim.A, ids = A.markers.map(M => M.id);
  assert(new Set(ids).size === ids.length, `duplicate ids ${ids}`);
  A.markers.forEach(M => {
    assert(OPENCV[M.id] != null, `no code for marker ${M.id}`);
    const fx = M.x + 0.6 * Math.cos(M.yaw), fy = M.y + 0.6 * Math.sin(M.yaw);
    assert(fx > 0.1 && fx < A.W - 0.1 && fy > 0.1 && fy < A.H - 0.1, `marker ${M.id} faces out of the arena`);
    assert(!A.boxes.some(b => fx > b.x0 && fx < b.x1 && fy > b.y0 && fy < b.y1), `marker ${M.id} faces into a box`);
  });
  return `ids ${ids.join(', ')}`;
});
test('The balls are real bodies: they rest on the floor, roll when pushed, slow down and stop, and reset', () => {
  const sim = fresh(); steps(sim, 0.5);
  const S = sim.balls.find(b => b.color === 'magenta').sys, p0 = sim.ballPos('magenta');
  assert(Math.abs(p0[2] - 0.05) < 0.003, `resting at z = ${p0[2].toFixed(4)}`);
  S.asleep = false; S.v[1] = 0.8 / 0.05; S.v[3] = 0.8;   // rolling east at 0.8 m/s (angular velocity first, then linear)
  steps(sim, 4); const p1 = sim.ballPos('magenta'); steps(sim, 1); const p2 = sim.ballPos('magenta');
  const rolled = p1[0] - p0[0];
  assert(rolled > 0.3 && rolled < 2, `rolled ${rolled.toFixed(2)} m`);
  assert(Math.abs(p1[1] - p0[1]) < 0.05, `drifted ${(p1[1] - p0[1]).toFixed(3)} m sideways`);
  assert(Math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 0.005, 'still rolling after 4 s');
  sim.reset(); steps(sim, 0.1); const p3 = sim.ballPos('magenta');
  assert(Math.hypot(p3[0] - 0.4, p3[1] - 0.35) < 0.01, `after reset at ${p3[0].toFixed(3)}, ${p3[1].toFixed(3)}`);
  return `rolled ${rolled.toFixed(2)} m from 0.8 m/s`;
});

/* ---------- the Vision tab ---------- */
test('The pixel picker reports OpenCV\'s 8-bit HSV (H 0-179)', () => {
  // reference values from cv2.cvtColor(..., cv2.COLOR_RGB2HSV)
  const cases = [[[255, 0, 0], '0,255,255'], [[255, 255, 0], '30,255,255'], [[0, 0, 255], '120,255,255'], [[128, 128, 128], '0,0,128'], [[217, 70, 239], '146,180,239'], [[245, 183, 0], '22,255,245'], [[10, 200, 30], '63,242,200'], [[250, 250, 250], '0,0,250']];
  cases.forEach(([c, want]) => { const got = L.SnoVision.hsv(...c).join(); assert(got === want, `rgb ${c}: ${got}, OpenCV says ${want}`); });
});
test('The viewer lists image topics and is a real subscriber that goes away with the tab', () => {
  const sim = fresh(), B = sim.bus, V = fakeView(); sim.camCap = {};
  assert(L.SnoVision.topics(sim).join() === IMG, `topics ${L.SnoVision.topics(sim)}`);
  B.addNode('/my_node', 'user', { user: true }); B.advertise('/my_node', '/debug/mask', 'sensor_msgs/msg/Image');
  assert(L.SnoVision.topics(sim).includes('/debug/mask'), 'your own image topics should be listed too');
  L.SnoVision.view(sim, IMG);
  assert(B.nodes.has('/image_view') && B.topics.get(IMG).subs.size === 1, 'the viewer should subscribe as /image_view');
  steps(sim, 0.1); sim.cameraFrame(V);
  assert(V.calls === 1 && sim.vis.n === 1, `viewer got ${sim.vis.n} images`);
  sim.tabOff();
  assert(!B.nodes.has('/image_view') && B.topics.get(IMG).subs.size === 0, 'leaving the tab should remove the viewer');
});
test('Vision missions: v1 in the terminal, v2-v5 on the Vision tab, and each starter template exists and reads the camera', () => {
  const sim = fresh(), ms = L.SnoVision.missions(sim);
  assert(ms.map(m => m.id).join() === 'v1,v2,v3,v4,v5', `missions ${ms.map(m => m.id)}`);
  assert(sim.def.tabs.some(t => t[0] === 'vision'), 'no Vision tab');
  ms.forEach(m => {
    assert(m.tab === (m.id === 'v1' ? 'term' : 'vision') && m.xp > 0 && m.text, `${m.id} is incomplete or on the ${m.tab} tab`);
    if (m.tpl) { const t = sim.def.py[m.tpl]; assert(t && t.vision && t.code.includes(IMG), `${m.id}: template ${m.tpl} missing or not reading ${IMG}`); }
  });
  assert(/'\/marker_id'/.test(sim.def.py.aruco.code) && /DICT_4X4_50/.test(sim.def.py.aruco.code), 'the ArUco template should publish /marker_id from DICT_4X4_50');
  assert(L.SnoVision.left(sim) === 5, `${L.SnoVision.left(sim)} left`);
});
test('Starter thresholds: the line template catches the yellow paint, the colour template the red ball and not the magenta one', () => {
  const sim = fresh(), H = (hex) => L.SnoVision.hsv(parseInt(hex.slice(1, 3), 16), parseInt(hex.slice(3, 5), 16), parseInt(hex.slice(5, 7), 16));
  const inR = (c, lo, hi) => c.every((v, i) => v >= lo[i] && v <= hi[i]);
  const line = sim.def.py.line.code, lo = JSON.parse(line.match(/YELLOW_LO = np\.array\((\[[^\]]+\])\)/)[1]), hi = JSON.parse(line.match(/YELLOW_HI = np\.array\((\[[^\]]+\])\)/)[1]);
  assert(inR(H('#F5B700'), lo, hi), `yellow ${H('#F5B700')} outside ${lo}..${hi}`);
  const rs = sim.def.py.color.code.match(/RANGES = (\[.*\])/)[1].replace(/\(/g, '[').replace(/\)/g, ']'), R = JSON.parse(rs);
  assert(R.some(([a, b]) => inR(H('#E5484D'), a, b)), `red ${H('#E5484D')} outside ${rs}`);
  assert(!R.some(([a, b]) => inR(H('#D946EF'), a, b)), `magenta ${H('#D946EF')} would be tracked as red`);
  assert(!R.some(([a, b]) => inR(H('#F5B700'), a, b)), 'the yellow line would be tracked as red');
});

/* ---------- missions, checked from the true state of the world ---------- */
test('v1 Camera check: needs ros2 topic hz (or bw) on the image and an echo of camera_info', () => {
  let sim = fresh();
  sim.event('cmd', 'echo:/camera/color/camera_info'); sim.event('cmd', 'hz:/scan');
  assert(!sim.done('v1'), 'counted without measuring the image rate');
  sim.event('cmd', 'bw:/camera/color/image_raw'); assert(sim.done('v1'), 'bw + echo should count');
  sim = fresh(); sim.event('cmd', 'hz:/camera/color/image_raw'); assert(!sim.done('v1'), 'counted without the echo');
  sim.event('cmd', 'echo:/camera/color/camera_info'); assert(sim.done('v1'), 'hz + echo should count');
});
test('v2 Follow the yellow line: 4 m along the centre line by your node; teleop does not count', () => {
  const drive = (useNode) => {
    const sim = fresh(); steps(sim, 0.1);
    const P = sim.ctrPts, n = P.length, pub = userNode(sim, '/line_follower');
    sim.teleport(P[0][0], P[0][1], Math.atan2(P[1][1] - P[0][1], P[1][0] - P[0][0]));
    let i = 0, worst = 0, t = 0;
    const follow = () => {   // pure pursuit along the true centre line, 24 cm ahead
      let bi = i, bd = 1e9; for (let k = 0; k < 40; k++) { const j = (i + k) % n, d = (P[j][0] - sim.x) ** 2 + (P[j][1] - sim.y) ** 2; if (d < bd) { bd = d; bi = j; } }
      i = bi; worst = Math.max(worst, Math.sqrt(bd));
      const g = P[(i + 12) % n], e = wrap(Math.atan2(g[1] - sim.y, g[0] - sim.x) - sim.th);
      if (useNode) pub(0.3, 2.5 * e); else sim.teleop(0.3, 0, 2.5 * e);
    };
    while (t < 20 && !sim.done('v2')) { steps(sim, 0.5, follow); t += 0.5; }
    return { sim, t, worst };
  };
  const a = drive(true);
  assert(a.sim.done('v2'), `only ${a.sim.st.vLine.toFixed(2)} m counted in ${a.t} s (worst ${a.worst.toFixed(2)} m off the line)`);
  const b = drive(false);
  assert(!b.sim.done('v2') && b.sim.st.vLine === 0, `teleop counted ${b.sim.st.vLine.toFixed(2)} m`);
  return `done after ${a.t} s, at most ${(a.worst * 100).toFixed(0)} cm off the line`;
});
test('v3 The magenta ball: stopped 18-50 cm from it and facing it, under your node', () => {
  const sim = fresh(), B = sim.ballPos('magenta'), pub = userNode(sim);
  const tryAt = (d, off, hold = () => pub(0, 0)) => { sim.st.still = 0; placeCam(sim, B[0] + d, B[1], Math.PI + off); steps(sim, 1.6, hold); return sim.done('v3'); };
  assert(!tryAt(0.15, 0), 'counted 15 cm away');
  assert(!tryAt(0.3, 0.35), 'counted while looking 20 degrees past it');
  assert(!tryAt(0.3, 0, () => sim.teleop(0, 0, 0)), 'counted while teleop held it, not your node');
  assert(tryAt(0.3, 0.05), `not counted: ${JSON.stringify(sim.vBall)}`);
});
test('v4 Marker reader: four different arena marker IDs on /marker_id, published by your node', () => {
  const sim = fresh(), B = sim.bus;
  B.addNode('/marker_reader', 'user', { user: true }); B.addNode('/someone_else');
  ['/marker_reader', '/someone_else'].forEach(n => B.advertise(n, '/marker_id', INT));
  const say = (node, id) => B.publish(node, '/marker_id', INT, { data: id });
  [7, 7, 3, 4, 99].forEach(id => say('/marker_reader', id));   // repeats, and ids that are not in the arena
  [12, 21, 5].forEach(id => say('/someone_else', id));          // not your node
  assert(!sim.done('v4') && [...sim.st.ids].sort().join() === '3,7', `ids ${[...sim.st.ids]}`);
  say('/marker_reader', 12); assert(!sim.done('v4'), 'counted with three ids');
  say('/marker_reader', 21); assert(sim.done('v4'), 'four ids should count');
});
test('v5 Dock at marker 7: camera 0.5 m in front, within 5 cm and 6 degrees, held still by your node', () => {
  const sim = fresh(), M = sim.A.markers.find(m => m.id === 7), pub = userNode(sim), nx = Math.cos(M.yaw), ny = Math.sin(M.yaw);
  const tryAt = (dn, lat, he, hold = () => pub(0, 0)) => {
    sim.st.still = 0; placeCam(sim, M.x + dn * nx - lat * ny, M.y + dn * ny + lat * nx, M.yaw + Math.PI + he);
    steps(sim, 1.6, hold); return sim.done('v5');
  };
  assert(!tryAt(0.58, 0, 0), 'counted 8 cm too far');
  assert(!tryAt(0.5, 0.07, 0), 'counted 7 cm off-centre');
  assert(!tryAt(0.5, 0, 0.14), 'counted 8 degrees askew');
  assert(!tryAt(0.5, 0, 0, () => pub(0, 0.5)), 'counted while turning');
  assert(!tryAt(0.5, 0, 0, () => sim.teleop(0, 0, 0)), 'counted while teleop held it, not your node');
  assert(tryAt(0.53, -0.03, -0.07), `not counted: ${JSON.stringify(sim.vDock)}`);
  const d = sim.vDock; return `docked at ${d.dn.toFixed(3)} m, ${(d.lat * 100).toFixed(1)} cm, ${(d.he * 180 / Math.PI).toFixed(1)} deg`;
});

run();
