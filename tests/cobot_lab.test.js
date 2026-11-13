// The LIMO Cobot's arm as a ROS graph: its driver, the camera on its gripper and a cube detector run as processes;
// the arm's TF frames exist only while its joint positions reach robot_state_publisher; the pick & place demo finds
// its cube with the camera. Then the Cobot's seven Fix it tickets (each fault really present, then fixed the way a ROS
// developer would) and its five vision missions, checked from the true state of the world.
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(i); sim.step(DT); } };
const fresh = () => { L.S.sno.m = {}; L.S.sno.fix = {}; return makeSim(L, 'cobot'); };
const open = (id) => { const sim = fresh(); L.SnoFix.start(sim, id); return sim; };
const term = () => { const T = { lines: [], proc: null, print(s) { T.lines.push(String(s)); }, stop() { if (T.proc) { const p = T.proc; T.proc = null; p.stop(); } }, ctx: { ui: { termState() {} } } }; return T; };
const ros2run = (sim, line) => { const a = line.trim().split(/\s+/), T = term(); assert(a[0] === 'ros2' && a[1] === 'run', 'not a ros2 run line'); sim.run(T, a[2], a.slice(3)); return T; };
const paramSet = (sim, node, k, v) => { const n = sim.bus.nodes.get(node), e = n && n.params.get(k); assert(e, `${node} has no parameter ${k}`); assert(!e.ro, `${k} is read-only`); e.v = v; const r = n.onParam ? n.onParam(k, v) : { ok: true }; assert(!r || r.ok !== false, 'param rejected'); };
const failing = (sim) => L.SnoFix.ticket(sim).check(sim, sim.fix.stats).items.filter(x => !x[1]).map(x => x[0]);
const logHas = (sim, re) => sim.logs.some(l => re.test(l[1]));
const solved = (sim, sec) => { for (let t = 0; t < sec && !sim.fix.solved; t += 0.5) steps(sim, 0.5); return sim.fix.solved; };
const fakeView = () => { const v = { calls: 0, capture() { v.calls++; return new Uint8Array(320 * 240 * 3).fill(200); } }; return v; };
const AC = 'snobots_msgs/msg/ArmCoords', PS = 'geometry_msgs/msg/PointStamped', IMG = 'sensor_msgs/msg/Image';
const user = (sim, name = '/my_node') => { if (!sim.bus.nodes.has(name)) sim.bus.addNode(name, 'user', { user: true }); return name; };
const goal = (x, y, z, speed = 60) => ({ x, y, z, rx: 180, ry: 0, rz: 0, speed });
const parked = (color = 'red') => { const sim = fresh(), g = sim.parkPose(color); sim.teleport(g[0], g[1], g[2]); steps(sim, 0.3); return sim; };
const lookDown = (sim) => { const me = user(sim, '/look'); sim.bus.advertise(me, '/arm/coords_goal', AC); sim.bus.publish(me, '/arm/coords_goal', AC, goal(150, 0, 120)); steps(sim, 3); };
const NORMAL = ['/arm_camera', '/arm_driver', '/base_link_to_laser_link', '/camera/camera', '/cube_detector', '/limo_base', '/robot_state_publisher', '/ydlidar_node'];
const red = (sim) => sim.cubes.find(c => c.color === 'red');

/* ---------- the arm's software ---------- */
test('Bringup: eight processes, the arm driver, the camera on the gripper and the cube detector among them', () => {
  const sim = fresh(), B = sim.bus, ci = [];
  B.addNode('/t'); B.subscribe('/t', '/arm_camera/camera_info', 'sensor_msgs/msg/CameraInfo', (m) => ci.push(m));
  steps(sim, 2);
  const nodes = [...B.nodes.keys()].filter(n => n !== '/t').sort();
  assert(JSON.stringify(nodes) === JSON.stringify(NORMAL), `nodes ${nodes}`);
  assert(sim.logs.filter(l => /process started with pid/.test(l[1])).length === 8, 'eight processes in the launch output');
  assert(logHas(sim, /connected to the arm on \/dev\/ttyACM0/) && logHas(sim, /auto white balance on/) && logHas(sim, /cubes are 0\.03 m wide/), 'each says what it does');
  assert(!logHas(sim, /Could not transform/), 'no TF warnings during a normal start');
  assert(ci.length >= 29 && ci.length <= 31, `${ci.length} camera_info messages in 2 s`);
  const m = ci[ci.length - 1];
  assert(m.k[0] === 200 && m.k[4] === 200 && m.k[2] === 159.5 && m.k[5] === 119.5 && m.header.frame_id === 'arm_camera_optical_frame' && m.width === 320, `K ${m.k}, ${m.header.frame_id}`);
  return `${ci.length} frames in 2 s`;
});
test('TF: the arm\'s frames need its joint positions on /joint_states; the camera looks down the gripper, picture top toward +x', () => {
  const sim = fresh(); steps(sim, 0.5);
  const T = sim.lookup('arm_base', 'arm_camera_optical_frame'), K = sim.kin.T, A = L.K3.mul(L.K3.inv(K.arm_base), K.arm_camera_optical_frame);
  assert(T && [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11].every(i => Math.abs(T[i] - A[i]) < 1e-9), 'the published camera frame is where the camera is');
  const G = K.gripper_base, O = K.arm_camera_optical_frame;
  assert(Math.abs(O[2] * G[2] + O[6] * G[6] + O[10] * G[10] - 1) < 1e-9, 'optical z is the gripper\'s z (it looks along the gripper)');
  assert(Math.abs(-(O[1] * G[0] + O[5] * G[4] + O[9] * G[8]) - 1) < 1e-9, 'image up (-y) is the gripper\'s +x');
  sim.stopProc('arm'); steps(sim, 1);
  assert(!sim.lookup('arm_base', 'link6') && !sim.lookup('arm_base', 'arm_camera_optical_frame') && sim.lookup('odom', 'arm_base'), 'without the arm driver the arm leaves TF, the base stays');
  sim.startProc('arm'); steps(sim, 0.2); sim.stopProc('rsp');
  assert(!sim.lookup('arm_base', 'link6'), 'without robot_state_publisher there are no arm frames');
});
test('The gripper camera renders only for subscribers that want pixels; the detector places the red cube from every frame', () => {
  const sim = parked(), B = sim.bus, V = fakeView(), dets = []; sim.armCamCfg().cap = {};
  B.addNode('/t'); B.subscribe('/t', '/cube_detections', 'snobots_msgs/msg/CubeDetection', (m) => dets.push(m));
  lookDown(sim); dets.length = 0;
  steps(sim, 1, () => sim.cameraFrame(V));
  assert(V.calls === 0, `rendered ${V.calls} frames for the robot's own detector`);
  const r = dets.filter(d => d.color === 'red'); assert(r.length >= 14 && dets.length === r.length, `${r.length} red detections (of ${dets.length}) in 1 s`);
  const c = red(sim), Ta = sim.kin.T.arm_base, w = L.K3.apply(Ta, r[0].position.x, r[0].position.y, r[0].position.z);
  assert(r[0].header.frame_id === 'arm_base' && Math.hypot(w[0] - c.x, w[1] - c.y, w[2] - c.z) < 0.002 && Math.abs(r[0].yaw) < 0.01, `red at ${JSON.stringify(r[0].position)}`);
  const imgs = []; B.addNode('/viewer'); B.subscribe('/viewer', '/arm_camera/image_raw', IMG, (m) => imgs.push(m));
  steps(sim, 1, () => sim.cameraFrame(V));
  assert(V.calls >= 14 && imgs.length === V.calls && imgs[0].encoding === 'rgb8' && imgs[0].header.frame_id === 'arm_camera_optical_frame', `${V.calls} renders for a real subscriber`);
  assert(imgs[0].data[0] === 200 && imgs[0].data[2] === 200, 'auto white balance leaves the colours alone');
  return `placed within ${(Math.hypot(w[0] - c.x, w[1] - c.y, w[2] - c.z) * 1000).toFixed(1)} mm`;
});
test('A fixed white balance casts the picture blue, and red and green drop out of the detector', () => {
  const sim = fresh(); sim.restoreRobot({ armcam: { params: { auto_white_balance: false, white_balance_temperature: 2800 } }, park: 'red' });
  const px = new Uint8Array([229, 72, 77, 48, 164, 108]); sim.camTint(px);
  const hr = L.SnoVision.hsv(px[0], px[1], px[2]), hg = L.SnoVision.hsv(px[3], px[4], px[5]);
  assert(hr[0] > 150 && hr[0] < 168 && hg[0] > 90, `red turns purple (H ${hr[0]}), green teal (H ${hg[0]})`);
  const dets = []; sim.bus.addNode('/t'); sim.bus.subscribe('/t', '/cube_detections', 'snobots_msgs/msg/CubeDetection', (m) => dets.push(m));
  lookDown(sim); assert(dets.length === 0, `${dets.length} detections under the cast`);
});
test('Released servos let the arm fall; goals are refused with a reason; powered again, it holds and obeys', () => {
  const sim = fresh(), B = sim.bus, me = user(sim); B.advertise(me, '/arm/coords_goal', AC); steps(sim, 0.5);
  const z0 = sim.kin.T.tcp[11]; sim.bus.services.get('/arm/servos').handler({ data: false }); steps(sim, 2);
  assert(sim.arm.free && sim.kin.T.tcp[11] < z0 - 0.08, `the gripper fell from ${z0.toFixed(2)} to ${sim.kin.T.tcp[11].toFixed(2)} m`);
  B.publish(me, '/arm/coords_goal', AC, goal(200, 0, 60)); steps(sim, 0.5);
  assert(logHas(sim, /The servos are released/) && sim.bus.services.get('/arm/go_home').handler({}).success === false, 'goals and go_home are refused, and the log says why');
  const q = sim.armQ(); sim.bus.services.get('/arm/servos').handler({ data: true }); steps(sim, 1);
  assert(!sim.arm.free && sim.armQ().every((v, i) => Math.abs(v - q[i]) < 0.05), 'powered, it holds where it is');
  B.publish(me, '/arm/coords_goal', AC, goal(200, 0, 60)); steps(sim, 3);
  const c = sim.coords(); assert(Math.hypot(c[0] - 200, c[1], c[2] - 60) < 5, `then reaches the goal (${c.slice(0, 3)})`);
});
test('The pick & place demo finds the red cube with the camera and picks it up', () => {
  const sim = parked(); sim.setDemo(true); let t = 0;
  for (; t < 20 && !sim.held; t += 0.5) steps(sim, 0.5);
  assert(sim.held && sim.held.color === 'red', `nothing held after ${t} s (demo at ${sim.demo && sim.demo.st})`);
  return `picked in ${t} s`;
});

/* ---------- Fix it: every fault is real, and the usual fixes work ---------- */
test('Ticket "The arm won\'t move": the arm driver died at start-up; restarting it lets the demo pick', () => {
  const sim = open('arm-down'); steps(sim, 6);
  assert(!sim.bus.nodes.has('/arm_driver') && logHas(sim, /could not open \/dev\/ttyACM0/) && logHas(sim, /process has died/), 'the driver should be dead, with its reason in the log');
  assert(failing(sim).length === 2 && !sim.lookup('arm_base', 'link6'), `failing: ${failing(sim)}`);
  const T = ros2run(sim, 'ros2 run cobot_driver arm_driver_node');
  assert(T.proc && sim.procs.arm, 'the driver should run in that terminal');
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Ticket "The arm is floppy": the servos are released; the driver says so, and /arm/servos true fixes it', () => {
  const sim = open('limp'); steps(sim, 6);
  assert(sim.arm.free && sim.fix.stats.grasp === 0 && logHas(sim, /The servos are released/), 'goals should be refused with a reason');
  const r = sim.bus.services.get('/arm/servos').handler({ data: true });
  assert(r.success && /holding the current pose/.test(r.message), JSON.stringify(r));
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Ticket "Butterfingers": the gripper is reversed; the demo will not lower a closed gripper; the parameter fixes it', () => {
  const sim = open('gripper'); steps(sim, 12);
  assert(sim.fix.stats.grasp === 0 && logHas(sim, /the gripper did not open \(0 %\)/) && red(sim).z < 0.14, 'the demo should stop above the cube, which stays put');
  paramSet(sim, '/arm_driver', 'gripper_reversed', false);
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Ticket "Where did red go?": a fixed white balance hides red; restarting the camera with auto white balance fixes it', () => {
  const sim = open('white-balance'); steps(sim, 8);
  assert(sim.fix.stats.detRed === 0 && logHas(sim, /no red cube in view on \/cube_detections/) && logHas(sim, /auto white balance off: white_balance_temperature 2800 K/), 'the detector should find no red');
  const T = ros2run(sim, 'ros2 run usb_cam usb_cam_node_exe --ros-args -p auto_white_balance:=true');
  assert(T.lines.some(l => /Stopped the robot's own \/arm_camera/.test(l)) && sim.P('/arm_camera', 'auto_white_balance') === true, `restart: ${T.lines.join(' | ')}`);
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Ticket "The arm vanished from TF": joint states remapped away from robot_state_publisher; a relay or a clean restart fixes it', () => {
  let sim = open('joint-states'); steps(sim, 6);
  assert(!sim.lookup('arm_base', 'link6') && sim.bus.topics.has('/arm/joint_states') && logHas(sim, /Could not transform from 'arm_camera_optical_frame' to 'arm_base'/), 'the arm should be missing from TF');
  ros2run(sim, 'ros2 run topic_tools relay /arm/joint_states /joint_states');
  assert(solved(sim, 10), `relay: still failing: ${failing(sim)}`);
  sim = open('joint-states'); steps(sim, 1); ros2run(sim, 'ros2 run cobot_driver arm_driver_node');
  assert(solved(sim, 10), `restart: still failing: ${failing(sim)}`);
});
test('Ticket "Out of reach?": goals read in base_link land 122 mm low; reference_frame arm_base fixes it', () => {
  const sim = open('ref-frame'); steps(sim, 6);
  assert(sim.fix.stats.grasp === 0 && logHas(sim, /no IK solution for \[130, 0, -2/), 'the look-down pose should land out of reach');
  paramSet(sim, '/arm_driver', 'reference_frame', 'arm_base');
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Ticket "Two metres under the table": cube_size 0.3 puts the cube ten times too far; 0.03 places it', () => {
  const sim = open('cube-size'); steps(sim, 6);
  assert(sim.fix.stats.det > 10 && sim.fix.stats.detOk === 0 && logHas(sim, /no IK solution for \[\d+, -?\d+, -1[89]\d\d/), 'the cube should land far below the table');
  paramSet(sim, '/cube_detector', 'cube_size', 0.03);
  assert(solved(sim, 15), `still failing: ${failing(sim)}`);
});
test('Closing a ticket restarts the Cobot normally', () => {
  const sim = open('limp'); steps(sim, 0.5); L.SnoFix.quit(sim); steps(sim, 1);
  const nodes = [...sim.bus.nodes.keys()].filter(n => !n.startsWith('/_')).sort();
  assert(JSON.stringify(nodes) === JSON.stringify(NORMAL), `nodes ${nodes}`);
  assert(!sim.demo && !sim.fix, 'the demo is off');
});
test('Every Cobot ticket is self-consistent: an id, a level, hints, a fix and a debrief, and its setup really breaks something', () => {
  const T = L.FIX_TICKETS.cobot; assert(T.length === 7 && new Set(T.map(t => t.id)).size === 7, `${T.length} tickets`);
  T.forEach(tk => {
    assert(tk.id && tk.level >= 1 && tk.level <= 3 && tk.hints.length === 3 && /^ros2 /.test(tk.fix) && tk.debrief.length > 120, `${tk.id} is incomplete`);
    const sim = open(tk.id); steps(sim, 1);
    assert(failing(sim).length > 0 && !sim.fix.solved, `${tk.id} starts solved`);
  });
});

/* ---------- vision missions ---------- */
test('v1 Camera check: ros2 topic hz on the gripper camera and an echo of its camera_info', () => {
  const sim = fresh();
  sim.event('cmd', 'hz:/arm_camera/image_raw'); assert(!sim.done('v1'), 'hz alone');
  sim.event('cmd', 'echo:/arm_camera/camera_info'); assert(sim.done('v1'), 'hz + echo should count');
});
test('v2 Which colours?: 10 right answers from your node; a wrong colour or another node does not count', () => {
  const sim = parked(), B = sim.bus, me = user(sim), S = 'std_msgs/msg/String'; B.addNode('/other'); B.advertise(me, '/cubes_seen', S); B.advertise('/other', '/cubes_seen', S); lookDown(sim);
  for (let i = 0; i < 12; i++) { B.publish('/other', '/cubes_seen', S, { data: 'red' }); B.publish(me, '/cubes_seen', S, { data: 'red,blue' }); }
  assert(!sim.done('v2') && sim.vst.seen === 0, 'a colour that is not there, or another node, should not count');
  for (let i = 0; i < 10; i++) B.publish(me, '/cubes_seen', S, { data: 'red' });
  assert(sim.done('v2'), `${sim.vst.seen} right answers`);
});
test('v3 Centre it: visual servoing like the template centres the red cube; arm moves from elsewhere do not count', () => {
  const sim = parked(), B = sim.bus, me = user(sim); B.advertise(me, '/arm/coords_goal', AC); lookDown(sim);
  const v0 = sim.cubeView(red(sim)); assert(v0 && Math.abs(v0[1]) > 0.15, `the cube should start off-centre (${v0 && v0.slice(0, 2)})`);
  // the template's loop, with the true picture: move by 0.6 of the error, scaled by the cube's known width, every 0.5 s
  const servo = (who, sec) => steps(sim, sec, (i) => { if (i % 30 || sim.done('v3')) return; const v = sim.cubeView(red(sim)); if (!v) return; const c = sim.coords(), mm = 30 / v[3], u = v[0] * 160, w = v[1] * 120; B.publish(who, '/arm/coords_goal', AC, goal(c[0] - 0.6 * w * mm, c[1] - 0.6 * u * mm, c[2], 40)); });
  B.addNode('/teleop'); B.advertise('/teleop', '/arm/coords_goal', AC); servo('/teleop', 6);
  const v1 = sim.cubeView(red(sim)); assert(!sim.done('v3') && Math.abs(v1[0]) < 0.1 && Math.abs(v1[1]) < 0.1, 'centred by a node that is not yours should not count');
  servo(me, 4);
  assert(sim.done('v3'), `centred for ${sim.vst.centre.toFixed(1)} s`);
  return `centred for ${sim.vst.centre.toFixed(1)} s`;
});
test('v4 How far down?: the top face from its pixel width counts; a point in the wrong frame does not', () => {
  const sim = parked(), B = sim.bus, me = user(sim); B.advertise(me, '/cube_point', PS); lookDown(sim);
  const v = sim.cubeView(red(sim)), z = 200 * 0.03 / v[3], p = v[4], hd = { stamp: { sec: 0, nanosec: 0 }, frame_id: 'arm_camera_optical_frame' };
  for (let i = 0; i < 12; i++) B.publish(me, '/cube_point', PS, { header: { stamp: hd.stamp, frame_id: 'arm_base' }, point: { x: p[0], y: p[1], z: p[2] } });
  assert(!sim.done('v4') && sim.vst.pts === 0, 'optical numbers in arm_base should not count');
  for (let i = 0; i < 10; i++) B.publish(me, '/cube_point', PS, { header: hd, point: { x: p[0] / p[2] * z, y: p[1] / p[2] * z, z: z + 0.004 } });
  assert(sim.done('v4'), `${sim.vst.pts} good points`);
});
test('v4 is judged from the camera: odometry drift does not spoil a right answer, in the camera\'s frame or in odom', () => {
  const sim = parked(), B = sim.bus, me = user(sim); B.advertise(me, '/cube_point', PS); lookDown(sim);
  sim.odom.x += 0.05; sim.odom.y -= 0.04; steps(sim, 0.2);   // wheels slipped: odom is now 6 cm from the truth
  assert(sim.odomDrift() > 0.05, `drift ${sim.odomDrift()}`);
  const v = sim.cubeView(red(sim)), p = v[4], Mo = sim.lookup('odom', 'arm_camera_optical_frame'), w = L.K3.apply(Mo, p[0], p[1], p[2]), st = { sec: 0, nanosec: 0 };
  for (let i = 0; i < 5; i++) B.publish(me, '/cube_point', PS, { header: { stamp: st, frame_id: 'arm_camera_optical_frame' }, point: { x: p[0], y: p[1], z: p[2] } });
  for (let i = 0; i < 5; i++) B.publish(me, '/cube_point', PS, { header: { stamp: st, frame_id: 'odom' }, point: { x: w[0], y: w[1], z: w[2] } });
  assert(sim.done('v4'), `${sim.vst.pts} points counted`);
});
test('v5 Pick by sight: a node like the template looks, centres, moves over and grabs the red cube', () => {
  const sim = parked(), B = sim.bus, me = user(sim), G = 'std_msgs/msg/Int32'; B.advertise(me, '/arm/coords_goal', AC); B.advertise(me, '/arm/gripper', G);
  const move = (x, y, z, s = 40) => B.publish(me, '/arm/coords_goal', AC, goal(x, y, z, s));
  let st = 'look', t0 = 0, n = 0, depth = 0;
  const go = (s) => { st = s; t0 = sim.t; };
  steps(sim, 25, (i) => {
    if (i % 6 || sim.done('v5')) return;
    const c = sim.coords(), t = sim.t - t0, v = sim.cubeView(red(sim));
    if (st === 'look') { B.publish(me, '/arm/gripper', G, { data: 100 }); move(150, 0, 120, 60); go('find'); }
    else if (st === 'find' && t > 2.5) go(v ? 'centre' : 'look');
    else if (st === 'centre' && t > 0.5 && v) {
      const u = v[0] * 160, w = v[1] * 120, mm = 30 / v[3];
      if (Math.abs(u) < 4 && Math.abs(w) < 4) { if (++n >= 3) { depth = 200 * 30 / v[3]; move(c[0] + 30, c[1], c[2]); go('over'); } }
      else { n = 0; move(c[0] - 0.6 * w * mm, c[1] - 0.6 * u * mm, c[2]); t0 = sim.t; }
    }
    else if (st === 'over' && t > 1.5) { move(c[0], c[1], c[2] + 56 - depth - 15, 30); go('down'); }
    else if (st === 'down' && t > 2.5) { B.publish(me, '/arm/gripper', G, { data: 0 }); go('grip'); }
  });
  assert(sim.done('v5') && sim.held && sim.held.color === 'red', `state ${st}, held ${sim.held && sim.held.color}`);
});
// What the gripper camera really renders from the look-down pose, measured in the browser in all three scene looks
// (OpenCV HSV): each cube's lit top and sides, and the robot's own parts in view. Lit paint is paler than the palette.
const RENDERED = {
  red: { h: [0, 1, 179], s: [121, 207], v: [154, 255] },
  green: { h: [73, 79], s: [93, 255], v: [116, 209] },
  blue: { h: [106, 111], s: [130, 255], v: [144, 244] },
};
const OWN = { 'lidar ring': { h: [6, 13], s: [121, 234], v: [166, 255] }, 'status lights': { h: [84, 90], s: [50, 173], v: [215, 255] }, 'dark-blue deck': { h: [101, 111], s: [90, 200], v: [12, 119] } };
const rangesOf = (code) => { const out = {}; for (const m of code.matchAll(/'(red|green|blue)': \[(.*)\],?\n/g)) out[m[1]] = [...m[2].matchAll(/\(\((\d+), (\d+), (\d+)\), \((\d+), (\d+), (\d+)\)\)/g)].map(r => [r.slice(1, 4).map(Number), r.slice(4, 7).map(Number)]); return out; };
test('Starter code: the colour ranges catch each cube as rendered and nothing else in view; templates read the gripper camera', () => {
  const sim = fresh(), py = sim.def.py, hex = (x) => L.SnoVision.hsv(parseInt(x.slice(1, 3), 16), parseInt(x.slice(3, 5), 16), parseInt(x.slice(5, 7), 16));
  const R = rangesOf(py.cam.code), hit = (c, name) => R[name].some(([lo, hi]) => c.every((v, i) => v >= lo[i] && v <= hi[i]));
  assert(Object.keys(R).join() === 'red,green,blue', `ranges for ${Object.keys(R)}`);
  // a measured box is caught whole when both of its corners are in one range (red: each hue it showed)
  Object.entries(RENDERED).forEach(([k, b]) => (k === 'red' ? b.h.map(x => [x, x]) : [b.h]).forEach(hh => [[hh[0], b.s[0], b.v[0]], [hh[1], b.s[1], b.v[1]]].forEach(c => assert(hit(c, k), `${k} as rendered (${c}) is outside its range`))));
  // the robot's own parts: each measured box misses every range in at least one of H, S, V
  Object.entries(OWN).forEach(([part, b]) => Object.keys(R).forEach(k => R[k].forEach(([lo, hi]) => assert([b.h, b.s, b.v].some((r, i) => r[1] < lo[i] || r[0] > hi[i]), `${part} ${JSON.stringify(b)} overlaps ${k}`))));
  // and the paint itself, plus everything else that can be under the camera when it looks down
  Object.entries({ red: '#E5484D', green: '#30A46C', blue: '#2F6FD0' }).forEach(([k, x]) => assert(hit(hex(x), k), `${k} paint ${hex(x)} is outside its range`));
  ['#C9A27A', '#F26419', '#FF8A3D', '#C9773B', '#F5B700', '#F6F8F9', '#E9EEF0', '#8E989E', '#2B3236', '#B9C2C8', '#CDD4D8', '#24324A', '#5EF2D6', '#1a3349', '#334d66'].forEach(x => Object.keys(R).forEach(k => assert(!hit(hex(x), k), `${x} ${hex(x)} would look like ${k}`)));
  const redSrc = py.cam.code.match(/'red': (\[.*\]),\n/)[1];
  ['colours', 'centre', 'size', 'vpick'].forEach(k => assert(py[k].code.includes(redSrc), `${k} should use the viewer's red`));
  ['cam', 'colours', 'centre', 'size', 'vpick'].forEach(k => assert(/'\/arm_camera\/image_raw'/.test(py[k].code), `${k} should read the gripper camera`));
  assert(/'\/cubes_seen'/.test(py.colours.code) && /'\/cube_point'/.test(py.size.code) && /'\/arm\/coords_goal'/.test(py.centre.code) && /'\/arm\/gripper'/.test(py.vpick.code), 'templates publish where the missions look');
  const ms = L.SnoVision.missions(sim); assert(ms.map(m => m.id).join() === 'v1,v2,v3,v4,v5' && ms.every(m => !m.tpl || py[m.tpl]), 'five vision missions with their templates');
});

run();
