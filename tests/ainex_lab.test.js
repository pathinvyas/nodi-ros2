// AiNex as a ROS graph of real processes: its bringup, a TF tree that only holds what is published, and a lazy head
// camera that still feeds the robot's own vision nodes. Then its seven Fix it tickets (each fault really present, then
// fixed the way a ROS developer would) and its five vision missions, checked from the true state of the world.
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(); sim.step(DT); } };
const fresh = () => { L.S.sno.m = {}; L.S.sno.fix = {}; return makeSim(L, 'ainex'); };
const open = (id) => { const sim = fresh(); L.SnoFix.start(sim, id); return sim; };
// a terminal for `ros2 run`: the sim prints into it and parks the running process in T.proc
const term = () => { const T = { lines: [], proc: null, print(s) { T.lines.push(String(s)); }, stop() { if (T.proc) { const p = T.proc; T.proc = null; p.stop(); } }, ctx: { ui: { termState() {} } } }; return T; };
const ros2run = (sim, line) => { const a = line.trim().split(/\s+/), T = term(); assert(a[0] === 'ros2' && a[1] === 'run', 'not a ros2 run line'); sim.run(T, a[2], a.slice(3)); return T; };
// what `ros2 param set` does for a simulated node
const paramSet = (sim, node, k, v) => { const n = sim.bus.nodes.get(node), e = n && n.params.get(k); assert(e, `${node} has no parameter ${k}`); assert(!e.ro, `${k} is read-only`); e.v = v; const r = n.onParam ? n.onParam(k, v) : { ok: true }; assert(!r || r.ok !== false, 'param rejected'); };
const failing = (sim) => { const tk = L.SnoFix.ticket(sim); return tk.check(sim, sim.fix.stats).items.filter(x => !x[1]).map(x => x[0]); };
const logHas = (sim, re) => sim.logs.some(l => re.test(l[1]));
const solved = (sim, sec) => { for (let t = 0; t < sec && !sim.fix.solved; t += 0.5) steps(sim, 0.5); return sim.fix.solved; };
// stands in for the 3D view: counts renders and returns a blank frame
const fakeView = () => { const v = { calls: 0, capture() { v.calls++; return new Uint8Array(320 * 240 * 3); } }; return v; };
const withView = (sim) => { const V = fakeView(); sim.camCap = {}; return { V, step: (sec) => steps(sim, sec, () => sim.cameraFrame(V)) }; };
const TW = 'geometry_msgs/msg/Twist', JS = 'sensor_msgs/msg/JointState';
const twist = (v = 0, w = 0, vy = 0) => ({ linear: { x: v, y: vy, z: 0 }, angular: { x: 0, y: 0, z: w } });
const user = (sim, name = '/my_node') => { if (!sim.bus.nodes.has(name)) sim.bus.addNode(name, 'user', { user: true }); return name; };
const NORMAL = ['/ainex_controller', '/ball_locator', '/color_detector', '/robot_state_publisher', '/usb_cam'];

/* ---------- the robot's software ---------- */
test('Bringup: five processes with their launch output, and the head camera on /image_raw and /camera_info', () => {
  const sim = fresh(), B = sim.bus, infos = [];
  B.addNode('/t'); B.subscribe('/t', '/camera_info', 'sensor_msgs/msg/CameraInfo', (m) => infos.push(m));
  steps(sim, 2);
  assert(NORMAL.every(n => B.nodes.has(n)), `nodes ${[...B.nodes.keys()]}`);
  assert(sim.logs.filter(l => /process started with pid/.test(l[1])).length === 5, 'five processes in the launch output');
  assert(logHas(sim, /Starting 'ainex_head' \(\/dev\/video0\) at 320x240 via mmap \(yuyv\) at 15 FPS/), 'usb_cam start-up line');
  const m = infos[infos.length - 1], f = 200 / Math.tan(Math.PI / 3);
  assert(infos.length >= 29 && infos.length <= 31, `${infos.length} camera_info messages in 2 s`);
  assert(Math.abs(m.k[0] - f) < 1e-3 && m.k[4] === m.k[0] && m.k[2] === 159.5 && m.k[5] === 119.5, `K ${m.k}`);
  assert(m.header.frame_id === 'camera_optical_frame' && m.width === 320 && m.height === 240, `${m.header.frame_id} ${m.width}x${m.height}`);
  const T = B.topics.get('/image_raw');
  assert(T.pubs.get('/usb_cam') === 'sensor_msgs/msg/Image' && T.subs.has('/color_detector') && T.subs.has('/ball_locator'), 'the detector and the locator subscribe to /image_raw');
  return `f = ${m.k[0]} px: a 120° diagonal lens`;
});
test('TF holds only what is published: fixed parts from /tf_static, joints while robot_state_publisher runs, odom from the controller', async () => {
  const sim = fresh(), B = sim.bus; steps(sim, 0.5);
  const T = sim.lookup('base_link', 'camera_optical_frame'), K = sim.kin.T, A = L.K3.mul(L.K3.inv(K.base_link), K.camera_optical_frame);
  assert(T && [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11].every(i => Math.abs(T[i] - A[i]) < 1e-9), 'the published optical frame is where the camera is');
  let tf = null; B.addNode('/late'); B.subscribe('/late', '/tf_static', 'tf2_msgs/msg/TFMessage', (m) => m.transforms.forEach(t => { if (t.child_frame_id === 'camera_optical_frame') tf = t; }));
  await new Promise(r => setTimeout(r, 5));
  const q = tf && tf.transform.rotation;
  assert(tf && tf.header.frame_id === 'camera_link' && [q.x, q.y, q.z, q.w].every((v, i) => Math.abs(v - [-0.5, 0.5, -0.5, 0.5][i]) < 1e-6), 'camera_link -> camera_optical_frame on /tf_static, z forward');
  sim.stopProc('rsp');
  assert(!sim.frames().includes('head_link') && !sim.lookup('base_link', 'camera_optical_frame'), 'without robot_state_publisher there are no joint frames');
  assert(sim.frames().includes('odom') && sim.frames().includes('base_link'), 'odom -> base_link still comes from the controller');
  sim.startProc('rsp'); sim.stopProc('ctl');
  assert(!sim.frames().includes('odom') && !sim.lookup('odom', 'base_link'), 'without the controller there is no odometry');
  sim.startProc('ctl'); ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --z 0.1 --frame-id head_link --child-frame-id antenna');
  const M = sim.lookup('head_link', 'antenna'); assert(M && Math.abs(M[11] - 0.1) < 1e-9, 'a frame you publish joins the tree');
  return `${sim.frames().length} frames`;
});
test('The camera renders only for subscribers that want pixels; the detector and the ball locator get every frame anyway', () => {
  const sim = fresh(), B = sim.bus, { V, step } = withView(sim), det = [], ball = [];
  B.addNode('/t'); B.subscribe('/t', '/color_detection', 'snobots_msgs/msg/ColorDetection', (m) => det.push(m)); B.subscribe('/t', '/ball_position', 'geometry_msgs/msg/PointStamped', (m) => ball.push(m));
  step(1);
  assert(V.calls === 0, `rendered ${V.calls} frames for the robot's own nodes`);
  assert(det.length >= 14 && det.length <= 16 && det.some(m => m.found && m.color === 'red'), `${det.length} detections, red found: ${det.some(m => m.found)}`);
  const b = sim.objs.find(o => o.kind === 'ball'), p = ball[ball.length - 1].point, n = sim.nav, c = Math.cos(n.yaw), s = Math.sin(n.yaw);
  const tx = c * (b.x - n.x) + s * (b.y - n.y), ty = -s * (b.x - n.x) + c * (b.y - n.y);
  assert(ball[ball.length - 1].header.frame_id === 'base_link' && Math.hypot(p.x - tx, p.y - ty) < 0.02, `ball at ${p.x}, ${p.y}; really ${tx.toFixed(3)}, ${ty.toFixed(3)}`);
  B.addNode('/viewer'); const imgs = []; B.subscribe('/viewer', '/image_raw', 'sensor_msgs/msg/Image', (m) => imgs.push(m));
  det.length = 0; step(1);
  assert(V.calls >= 14 && imgs.length === V.calls && imgs[0].header.frame_id === 'camera_optical_frame', `${V.calls} renders for a real subscriber`);
  assert(det.length >= 14 && det.length <= 16, `${det.length} detections: each frame once, not twice`);
  sim.stopProc('cam'); det.length = 0; step(1);
  assert(det.length === 0, 'no images, no detections');
  return `ball placed within ${(Math.hypot(p.x - tx, p.y - ty) * 100).toFixed(1)} cm`;
});

/* ---------- Fix it: every fault is real, and the usual fixes work ---------- */
test('Ticket "AiNex is blind": the camera driver died at start-up; restarting it brings the pictures back', () => {
  const sim = open('cam-down'); steps(sim, 1.5);
  assert(!sim.bus.nodes.has('/usb_cam') && logHas(sim, /Cannot identify '\/dev\/video0'/) && logHas(sim, /process has died/), 'the driver should be dead, with its last words in the log');
  assert(failing(sim).length === 2 && sim.fix.stats.det === 0, `failing: ${failing(sim)}`);
  const pan0 = sim.q.head_pan, T = ros2run(sim, 'ros2 run usb_cam usb_cam_node_exe');
  assert(T.proc && sim.procs.cam, 'the driver should run in that terminal');
  assert(solved(sim, 4), `still failing: ${failing(sim)}`);
  steps(sim, 1); assert(Math.abs(sim.q.head_pan - pan0) > 0.05, 'head tracking should turn toward the red block once it can see');
});
test('Ticket "Head tracking ignores the red block": min_area is far too big; lowering it fixes tracking', () => {
  const sim = open('min-area'); steps(sim, 2);
  assert(sim.fix.stats.det > 20 && sim.fix.stats.red === 0 && sim.trk.on, `frames ${sim.fix.stats.det}, red ${sim.fix.stats.red}`);
  paramSet(sim, '/color_detector', 'min_area', 0.0001);
  assert(solved(sim, 6), `still failing: ${failing(sim)}`);
});
test('Ticket "AiNex trips over its own feet": an 8 mm step height trips it; the default walks', () => {
  const sim = open('trip'); steps(sim, 0.5);
  steps(sim, 5, () => sim.teleop(0.15, 0, 0)); sim.teleop(0, 0, 0); steps(sim, 1);
  assert(['fall', 'fallen'].includes(sim.mode) && sim.fix.stats.falls >= 1 && logHas(sim, /fall detected/), `mode ${sim.mode}, falls ${sim.fix.stats.falls}`);
  sim.standUp(); steps(sim, 4); assert(sim.mode === 'stand', `after the hoist: ${sim.mode}`);
  paramSet(sim, '/ainex_controller', 'step_height', 0.025);
  steps(sim, 5, () => sim.teleop(0.15, 0, 0)); sim.teleop(0, 0, 0);
  assert(solved(sim, 1), `walked ${sim.fix.stats.walk.toFixed(2)} m; failing: ${failing(sim)}`);
});
test('Ticket "Pictures, but nobody looks": a leftover remap; restarting the camera or relaying the topic fixes it', () => {
  let sim = open('cam-remap'); steps(sim, 1.5);
  assert(sim.bus.topics.has('/ainex/image_raw') && sim.fix.stats.det === 0, 'the camera should publish on /ainex/image_raw while the detector waits');
  ros2run(sim, 'ros2 run usb_cam usb_cam_node_exe');
  assert(solved(sim, 4) && sim.tn.img === '/image_raw', `restart: still failing: ${failing(sim)}`);
  // the relay moves real images, so it needs pixels: a stand-in 3D view renders them
  sim = open('cam-remap'); const { step } = withView(sim); step(1);
  assert(sim.fix.stats.det === 0, 'not fixed yet');
  ros2run(sim, 'ros2 run topic_tools relay /ainex/image_raw /image_raw');
  for (let t = 0; t < 4 && !sim.fix.solved; t += 0.5) step(0.5);
  assert(sim.fix.solved, `relay: still failing: ${failing(sim)} (frames ${sim.fix.stats.det})`);
});
test('Ticket "Echo sees pictures, the detector does not": BEST_EFFORT camera, RELIABLE detector; either side can give way', () => {
  let sim = open('cam-qos'); steps(sim, 1.5);
  assert(sim.fix.stats.det === 0 && logHas(sim, /offering incompatible QoS/) && logHas(sim, /requesting incompatible QoS/), 'both sides should warn once');
  ros2run(sim, 'ros2 run usb_cam usb_cam_node_exe --ros-args -p qos_overrides./image_raw.publisher.reliability:=reliable');
  assert(solved(sim, 4), `camera restart: still failing: ${failing(sim)}`);
  sim = open('cam-qos'); steps(sim, 1);
  ros2run(sim, 'ros2 run ainex_vision color_detector --ros-args -p qos_overrides./image_raw.subscription.reliability:=best_effort');
  assert(solved(sim, 4), `detector restart: still failing: ${failing(sim)}`);
});
test('Ticket "Where is the ball?": images stamped with a frame the tree lacks; a wrong bridge is caught, the right one works', () => {
  let sim = open('frame-id'); steps(sim, 3.5);
  assert(logHas(sim, /Could not transform the ball from 'camera' to 'base_link'/) && sim.fix.stats.ballOk === 0, 'the locator should complain and place nothing');
  // a zero transform from camera_link has the wrong axes (x forward instead of z forward)
  const wrong = ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --frame-id camera_link --child-frame-id camera'); steps(sim, 2);
  const res = L.SnoFix.ticket(sim).check(sim, sim.fix.stats);
  assert(res.items[0][1] && !res.items[1][1] && /away from the real mount/.test(res.note) && sim.fix.stats.ballOk === 0, `wrong bridge: ${JSON.stringify(res)}`);
  wrong.stop(); ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --frame-id camera_optical_frame --child-frame-id camera');
  assert(solved(sim, 4), `static bridge: still failing: ${failing(sim)}`);
  sim = open('frame-id'); steps(sim, 1);
  ros2run(sim, 'ros2 run usb_cam usb_cam_node_exe --ros-args -p frame_id:=camera_optical_frame');
  assert(solved(sim, 4), `camera restart: still failing: ${failing(sim)}`);
});
test('Ticket "AiNex won\'t walk from the joystick": TwistStamped vs Twist; switching the joystick back walks', () => {
  const sim = open('stamped'); steps(sim, 0.5); const x0 = sim.nav.x;
  steps(sim, 3, () => sim.teleop(0.15, 0, 0)); sim.teleop(0, 0, 0); steps(sim, 0.5);
  assert(Math.abs(sim.nav.x - x0) < 0.03 && sim.fix.stats.joy === 0, `moved ${(sim.nav.x - x0).toFixed(3)} m`);
  paramSet(sim, '/teleop_panel', 'publish_stamped_twist', false);
  steps(sim, 5, () => sim.teleop(0.15, 0, 0)); sim.teleop(0, 0, 0);
  assert(solved(sim, 1), `joystick distance ${sim.fix.stats.joy.toFixed(2)} m; failing: ${failing(sim)}`);
});
test('Closing a ticket restarts AiNex normally', () => {
  const sim = open('cam-remap'); steps(sim, 0.5); L.SnoFix.quit(sim); steps(sim, 1);
  const nodes = [...sim.bus.nodes.keys()].filter(n => !n.startsWith('/_')).sort();
  assert(JSON.stringify(nodes) === JSON.stringify(NORMAL), `nodes ${nodes}`);
  assert(sim.tn.img === '/image_raw' && !sim.trk.on && !sim.fix, 'the camera back on /image_raw, head tracking off');
});
test('Every AiNex ticket is self-consistent: an id, a level, hints, a fix and a debrief, and its setup really breaks something', () => {
  const T = L.FIX_TICKETS.ainex; assert(T.length === 7, `${T.length} tickets`);
  T.forEach(tk => {
    assert(tk.id && tk.level >= 1 && tk.level <= 3 && tk.hints.length === 3 && /^ros2 /.test(tk.fix) && tk.debrief.length > 120, `${tk.id} is incomplete`);
    const sim = open(tk.id); steps(sim, 1);
    assert(failing(sim).length > 0 && !sim.fix.solved, `${tk.id} starts solved`);
  });
});

test('The goal mouth is open: a ball rolled at the goal goes in and counts (the roof net used to fill it)', () => {
  const sim = fresh(), b = sim.objs.find(o => o.kind === 'ball'); steps(sim, 0.5);
  sim.placeObj(b, 1.25, 0); const S = b.sys; S.asleep = false; S.v[3] = 0.6; S.v[1] = 0.6 / b.r;
  steps(sim, 2);
  assert(sim.goals === 1 && sim.done('m7'), `ball stopped at x = ${b.x.toFixed(3)}, goals ${sim.goals}`);
  const top = sim.world.statics.find(g => Math.abs(g.W[3] - 1.59) < 1e-6 && Math.abs(g.W[7]) < 1e-6 && g.W[11] > 0.15);
  assert(top, 'the roof net is still a static body above the goal');
});

/* ---------- vision missions ---------- */
test('v1 Camera check: ros2 topic hz on /image_raw and an echo of /camera_info', () => {
  const sim = fresh();
  sim.event('cmd', 'echo:/camera_info'); assert(!sim.done('v1'), 'echo alone');
  sim.event('cmd', 'hz:/image_raw'); assert(sim.done('v1'), 'hz + echo should count');
});
test('v2 Watch the ball: your node keeps the ball centred by turning the head while AiNex walks; no head work, no credit', () => {
  const run2 = (mine) => {
    const sim = fresh(), B = sim.bus, me = user(sim); B.advertise(me, '/joint_commands', JS); steps(sim, 0.5);
    const hf = Math.tan(sim.H.hfov / 2), vf = Math.tan(sim.H.camVfov * Math.PI / 360);
    const look = () => { const uv = sim.ballInImage(); if (!uv || !mine) return; B.publish(me, '/joint_commands', JS, { name: ['head_pan', 'head_tilt'], position: [sim.q.head_pan - 0.5 * Math.atan(uv[0] * hf), sim.q.head_tilt + 0.5 * Math.atan(uv[1] * vf)] }); };
    steps(sim, 3, () => { sim.teleop(0, 0, 0.3); look(); }); steps(sim, 3.5, () => { sim.teleop(0, 0, -0.3); look(); }); sim.teleop(0, 0, 0);
    return sim;
  };
  const a = run2(true); assert(a.done('v2'), `only ${a.vst.watch.toFixed(1)} s of watching`);
  const b = run2(false); assert(!b.done('v2'), `counted ${b.vst.watch.toFixed(1)} s without head commands`);
  return `${a.vst.watch.toFixed(1)} s`;
});
test('v3 Walk to the ball: standing still with the ball 10 to 25 cm ahead while your node holds /cmd_vel', () => {
  const tryAt = (dx, dy, mine = true) => {
    const sim = fresh(), B = sim.bus, me = user(sim), b = sim.objs.find(o => o.kind === 'ball'); B.advertise(me, '/cmd_vel', TW); steps(sim, 0.5);
    const n = sim.nav, c = Math.cos(n.yaw), s = Math.sin(n.yaw); sim.placeObj(b, n.x + c * dx - s * dy, n.y + s * dx + c * dy);
    steps(sim, 1.6, () => { if (mine) B.publish(me, '/cmd_vel', TW, twist()); else sim.teleop(0, 0, 0); }); return sim.done('v3');
  };
  assert(!tryAt(0.35, 0), 'counted 35 cm away');
  assert(!tryAt(0.17, 0.12), 'counted 12 cm to the side');
  assert(!tryAt(0.17, 0, false), 'counted while the joystick held it, not your node');
  assert(tryAt(0.17, 0.02), 'not counted at 17 cm');
});
test('v4 Tag reader: three different pitch tags on /marker_id, from your node', () => {
  const sim = fresh(), B = sim.bus, me = user(sim), INT = 'std_msgs/msg/Int32'; B.addNode('/other'); B.advertise(me, '/marker_id', INT); B.advertise('/other', '/marker_id', INT);
  [1, 1, 3, 7].forEach(i => B.publish(me, '/marker_id', INT, { data: i }));   // 3 and 7 are LIMO's markers, not on this pitch
  [2, 4].forEach(i => B.publish('/other', '/marker_id', INT, { data: i }));
  assert(!sim.done('v4') && sim.vst.ids.size === 1, `ids ${[...sim.vst.ids]}`);
  [2, 0].forEach(i => B.publish(me, '/marker_id', INT, { data: i }));
  assert(sim.done('v4'), 'three tags should count');
});
test('v5 Vision striker: a goal soon after a kick your node asked for; the Control tab\'s kick button does not count', () => {
  const goal = (sim) => { const b = sim.objs.find(o => o.kind === 'ball'); sim.placeObj(b, 1.62, 0); steps(sim, 0.2); };
  let sim = fresh(); const me = user(sim); steps(sim, 0.5);
  sim.playAction('left_kick'); steps(sim, 1); goal(sim);
  assert(!sim.done('v5') && sim.done('m7'), 'a kick from the button is not your node');
  sim = fresh(); user(sim); steps(sim, 0.5);
  sim.bus.advertise(me, '/action_group', 'std_msgs/msg/String'); sim.bus.publish(me, '/action_group', 'std_msgs/msg/String', { data: 'right_kick' }); steps(sim, 7.5); goal(sim);
  assert(!sim.done('v5'), 'counted a goal 7 s after the kick');
  sim = fresh(); user(sim); steps(sim, 0.5);
  sim.bus.advertise(me, '/action_group', 'std_msgs/msg/String'); sim.bus.publish(me, '/action_group', 'std_msgs/msg/String', { data: 'left_kick' }); steps(sim, 2); goal(sim);
  assert(sim.done('v5'), 'a goal 2 s after your kick should count');
});
test('Starter code: orange thresholds catch the ball, not the red block or the pitch; the tag reader knows the tag sizes', () => {
  const sim = fresh(), py = sim.def.py, hex = (h) => L.SnoVision.hsv(parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16));
  const inR = (c, lo, hi) => c.every((v, i) => v >= lo[i] && v <= hi[i]);
  ['ball_head', 'ball_walk'].forEach(k => {
    const code = py[k].code, lo = JSON.parse(code.match(/ORANGE_LO = np\.array\((\[[^\]]+\])\)/)[1]), hi = JSON.parse(code.match(/ORANGE_HI = np\.array\((\[[^\]]+\])\)/)[1]);
    assert(inR(hex('#FF8A3D'), lo, hi), `${k}: the ball ${hex('#FF8A3D')} is outside ${lo}..${hi}`);
    ['#E5484D', '#2F8F5B', '#33975F', '#F4F7F2'].forEach(c => assert(!inR(hex(c), lo, hi), `${k}: ${c} ${hex(c)} would look like the ball`));
  });
  assert(/TAG_SIZE = 0\.15/.test(py.aruco.code) && /0\.20 if i == 0/.test(py.aruco.code), 'tag sizes in the tag reader');
  const sizes = new Set(L.AINEX_TAGS.filter(t => t.id !== 0).map(t => t.size)); assert(sizes.size === 1 && sizes.has(0.15) && L.AINEX_TAGS.find(t => t.id === 0).size === 0.2, 'the pitch tags are 15 cm, tag 0 is 20 cm');
  Object.entries(py).filter(([, t]) => t.vision).forEach(([k, t]) => assert(/'\/image_raw'/.test(t.code), `${k} should read /image_raw`));
  const ms = L.SnoVision.missions(sim); assert(ms.map(m => m.id).join() === 'v1,v2,v3,v4,v5' && ms.every(m => !m.tpl || py[m.tpl]), 'five vision missions with their templates');
});
test('Pitch tags: OpenCV\'s DICT_4X4_50 codes, facing into the pitch, above the boards', () => {
  const OPENCV = { 0: 46386, 1: 3994, 2: 13101, 4: 21662, 8: 65242 };   // cv2.aruco.generateImageMarker(DICT_4X4_50, id, 6), inner 4x4, white = 1
  L.AINEX_TAGS.forEach(t => {
    assert(L.ARUCO_4X4[t.id] === OPENCV[t.id], `tag ${t.id} code ${L.ARUCO_4X4[t.id]}`);
    const fx = t.x + 0.5 * Math.cos(t.yaw), fy = t.y + 0.5 * Math.sin(t.yaw);
    assert(fx > -0.9 && fx < 1.5 && Math.abs(fy) < 0.9, `tag ${t.id} faces away from the pitch`);
    assert(t.z - t.size * 4 / 6 > 0.05, `tag ${t.id}'s white border dips behind the 5 cm boards`);
  });
});

run();
