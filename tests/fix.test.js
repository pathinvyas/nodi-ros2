// Broken-robot missions: every ticket must start with its fault really present in the ROS graph,
// and the fixes a ROS developer would try (restart, param set, relay, static transform…) must really fix it.
// Also checks the QoS and latching rules of the simulated bus that the tickets rely on.
const { loadSims, makeSim, test, assert, run } = require('./harness');
const L = loadSims(), DT = 1 / 60;
const steps = (sim, sec, every) => { for (let i = 0; i < Math.round(sec / DT); i++) { if (every) every(); sim.step(DT); } };
const drive = (sim, sec, v = 0.5) => { steps(sim, sec, () => sim.teleop(v, 0, 0)); sim.teleop(0, 0, 0); steps(sim, 0.3); };
// a terminal for `ros2 run`: the sim prints into it and parks the running process in T.proc
const term = () => { const T = { lines: [], proc: null, print(s) { T.lines.push(String(s)); }, stop() { if (T.proc) { const p = T.proc; T.proc = null; p.stop(); } }, ctx: { ui: { termState() {} } } }; return T; };
const ros2run = (sim, line) => { const a = line.trim().split(/\s+/), T = term(); assert(a[0] === 'ros2' && a[1] === 'run', 'not a ros2 run line'); sim.run(T, a[2], a.slice(3)); return T; };
// what `ros2 param set` does for a simulated node
const paramSet = (sim, node, k, v) => { const n = sim.bus.nodes.get(node), e = n && n.params.get(k); assert(e, `${node} has no parameter ${k}`); assert(!e.ro, `${k} is read-only`); e.v = v; const r = n.onParam ? n.onParam(k, v) : { ok: true }; assert(!r || r.ok !== false, 'param rejected'); };
const ticket = (id) => L.FIX_TICKETS.limo.find(t => t.id === id);
const failing = (sim) => { const tk = L.SnoFix.ticket(sim); return tk.check(sim, sim.fix.stats).items.filter(x => !x[1]).map(x => x[0]); };
const logHas = (sim, re) => sim.logs.some(l => re.test(l[1]));
const open = (id) => { const sim = makeSim(L, 'limo'); L.SnoFix.start(sim, id); return sim; };
const needsDrive = { slow: 1, remap: 1, stamped: 1 };
const NORMAL = ['/robot_state_publisher', '/base_link_to_laser_link', '/limo_base', '/ydlidar_node', '/lane_detector'];

/* ---------- the bus rules the tickets are built on ---------- */
test('QoS: a RELIABLE subscription gets nothing from a BEST_EFFORT publisher, and both sides hear about it once', () => {
  const sim = makeSim(L, 'limo'), B = sim.bus; let got = 0; const warn = [];
  B.onClash = (p, s, t, pol) => warn.push(pol);
  B.addNode('/a'); B.addNode('/b'); B.advertise('/a', '/x', 'std_msgs/msg/String', { rel: 'BEST_EFFORT' });
  B.subscribe('/b', '/x', 'std_msgs/msg/String', () => got++);
  for (let i = 0; i < 5; i++) B.publish('/a', '/x', 'std_msgs/msg/String', { data: 'hi' });
  assert(got === 0, `delivered ${got}`); assert(warn.length === 1 && warn[0] === 'RELIABILITY', `warnings ${warn}`);
  B.subscribe('/b', '/y', 'std_msgs/msg/String', () => got++, { rel: 'BEST_EFFORT' }); B.advertise('/a', '/y', 'std_msgs/msg/String');
  B.publish('/a', '/y', 'std_msgs/msg/String', { data: 'hi' });
  assert(got === 1, 'a RELIABLE publisher must reach a BEST_EFFORT subscription');
});
test('QoS: a TRANSIENT_LOCAL subscription refuses a VOLATILE publisher; different types never connect', () => {
  const B = makeSim(L, 'limo').bus; let got = 0;
  B.addNode('/a'); B.addNode('/b'); B.advertise('/a', '/x', 'std_msgs/msg/String');
  B.subscribe('/b', '/x', 'std_msgs/msg/String', () => got++, { dur: 'TRANSIENT_LOCAL' });
  B.publish('/a', '/x', 'std_msgs/msg/String', { data: 'hi' });
  B.subscribe('/b', '/t', 'geometry_msgs/msg/Twist', () => got++); B.publish('/a', '/t', 'geometry_msgs/msg/TwistStamped', { header: {}, twist: {} });
  assert(got === 0, `delivered ${got}`);
  assert(B.types('/t').length === 2, 'the topic should list both types');
});
test('Latched /tf_static: a late subscriber gets every running publisher\'s transforms, not a stopped one\'s', async () => {
  const sim = makeSim(L, 'limo'), B = sim.bus, T = ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --x 1 --frame-id base_link --child-frame-id extra');
  const kids = []; B.addNode('/late'); B.subscribe('/late', '/tf_static', 'tf2_msgs/msg/TFMessage', (m) => m.transforms.forEach(t => kids.push(t.child_frame_id)));
  await new Promise(r => setTimeout(r, 5));
  ['camera_link', 'laser_link', 'extra'].forEach(f => assert(kids.includes(f), `missing ${f}: got ${kids}`));
  T.stop(); const kids2 = []; B.addNode('/later'); B.subscribe('/later', '/tf_static', 'tf2_msgs/msg/TFMessage', (m) => m.transforms.forEach(t => kids2.push(t.child_frame_id)));
  await new Promise(r => setTimeout(r, 5));
  assert(!kids2.includes('extra') && kids2.includes('laser_link'), `after Ctrl+C got ${kids2}`);
  assert(!sim.frames().includes('extra'), 'the frame should leave the TF tree with its publisher');
});

/* ---------- every ticket starts broken ---------- */
for (const tk of L.FIX_TICKETS.limo) test(`Ticket "${tk.title}" starts with its fault`, () => {
  const sim = open(tk.id);
  if (needsDrive[tk.id]) drive(sim, 2.5); else steps(sim, 3);
  assert(!sim.fix.solved, 'solved itself');
  const bad = failing(sim); assert(bad.length, 'every check already passes');
  return `still failing: ${bad.join(' · ').replace(/`/g, '')}`;
});
test('The faults show up the way they would on a real robot', () => {
  let sim = open('lidar-down'); steps(sim, 1);
  assert(logHas(sim, /\[ERROR\] \[ydlidar_ros2_driver_node-4\]: process has died \[pid \d+, exit code 1/), 'no "process has died" line');
  assert(!sim.bus.nodes.has('/ydlidar_node'), 'the crashed driver is still in the graph');
  assert(sim.bus.topics.get('/scan') && sim.bus.topics.get('/scan').subs.has('/slam_toolbox') && !sim.bus.topics.get('/scan').pubs.size, '/scan should have SLAM subscribed and no publisher');
  sim = open('qos'); steps(sim, 1);
  assert(logHas(sim, /\[slam_toolbox\]: New publisher discovered on topic '\/scan', offering incompatible QoS.*RELIABILITY/), 'no incompatible-QoS warning from SLAM');
  assert(logHas(sim, /\[ydlidar_node\]: New subscription discovered on topic '\/scan', requesting incompatible QoS/), 'no incompatible-QoS warning from the driver');
  sim = open('tf-missing'); steps(sim, 1);
  assert(!sim.frames().includes('laser_link'), 'laser_link should be missing from the tree');
  assert(logHas(sim, /Failed to compute laser pose/), 'SLAM should complain about the laser pose');
  sim = open('remap'); steps(sim, 0.5);
  assert(sim.bus.topics.get('/limo/cmd_vel').subs.has('/limo_base') && !(sim.bus.topics.get('/cmd_vel') || { subs: new Map() }).subs.has('/limo_base'), 'the base should listen on /limo/cmd_vel only');
  assert(logHas(sim, /\[limo_base_node-3\] \[INFO\] \[[\d.]+\] \[limo_base\]: LIMO base ready, listening on \/limo\/cmd_vel/), 'launch output should carry the process name');
  sim = open('stamped'); drive(sim, 0.5);
  assert(sim.bus.types('/cmd_vel').length === 2, `cmd_vel types ${sim.bus.types('/cmd_vel')}`);
  sim = open('frame-id'); steps(sim, 0.5);
  assert(sim.bus.topics.get('/scan').last.header.frame_id === 'laser', 'scans should say frame laser');
});

/* ---------- …and the fixes a ROS developer would try really fix it ---------- */
const fixes = [
  ['lidar-down', 'restart the driver', (sim) => ros2run(sim, ticket('lidar-down').fix)],
  ['slow', 'ros2 param set', (sim) => paramSet(sim, '/limo_base', 'max_linear_speed', 1.0)],
  ['slow', 'restart the base with its defaults', (sim) => ros2run(sim, 'ros2 run limo_base limo_base_node')],
  ['remap', 'restart the base without the remap', (sim) => ros2run(sim, ticket('remap').fix)],
  ['remap', 'topic_tools relay', (sim) => { sim.teleop(0, 0, 0); return ros2run(sim, 'ros2 run topic_tools relay /cmd_vel /limo/cmd_vel'); }],
  ['qos', 'restart the lidar as reliable', (sim) => ros2run(sim, ticket('qos').fix)],
  ['qos', 'restart SLAM as best effort', (sim) => ros2run(sim, 'ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p qos_overrides./scan.subscription.reliability:=best_effort')],
  ['tf-missing', 'static_transform_publisher (Jazzy style)', (sim) => ros2run(sim, ticket('tf-missing').fix)],
  ['tf-missing', 'static_transform_publisher (old positional style)', (sim) => ros2run(sim, 'ros2 run tf2_ros static_transform_publisher 0.09 0 0.18 0 0 0 base_link laser_link')],
  ['frame-id', 'restart the driver with frame_id:=laser_link', (sim) => ros2run(sim, ticket('frame-id').fix)],
  ['frame-id', 'a zero static transform laser_link → laser', (sim) => ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --frame-id laser_link --child-frame-id laser')],
  ['stamped', 'ros2 param set', (sim) => paramSet(sim, '/teleop_panel', 'publish_stamped_twist', false)],
];
for (const [id, how, apply] of fixes) test(`Ticket "${ticket(id).title}" is fixed by: ${how}`, () => {
  const sim = open(id); steps(sim, 1);
  const T = apply(sim); if (T && T.lines.some(l => /error|Aborted|not found|usage/i.test(l))) throw new Error(T.lines.join('\n'));
  if (needsDrive[id]) drive(sim, 2.2); else steps(sim, 3);
  assert(sim.fix.solved, `not fixed: ${failing(sim).join(' · ')}`);
  assert(L.S.sno.fix['limo:' + id], 'progress not saved');
  if (T && T.proc) T.stop();
  return `${sim.fix.secs} s`;
});

/* ---------- near misses that must not count ---------- */
test('A static transform with the wrong offset does not fix "Where is the lidar?" and says why', () => {
  const sim = open('tf-missing'); steps(sim, 1);
  ros2run(sim, 'ros2 run tf2_ros static_transform_publisher --z 0.18 --frame-id base_link --child-frame-id laser_link'); steps(sim, 3);
  assert(!sim.fix.solved, 'counted a 9 cm error as fixed');
  const r = ticket('tf-missing').check(sim, sim.fix.stats); assert(/9\.0 cm/.test(r.note), `note: ${r.note}`);
  return r.note.split(',')[0];
});
test('Driving with ros2 topic pub does not fix a joystick ticket, and an integer for a double crashes the node like rclcpp', () => {
  const sim = open('remap'), B = sim.bus; steps(sim, 0.5);
  B.addNode('/_ros2cli_42', 'cli'); steps(sim, 2, () => B.publish('/_ros2cli_42', '/limo/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: 0.4, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }));
  assert(Math.hypot(sim.x - sim.A.start[0], sim.y - sim.A.start[1]) > 0.3 && !sim.fix.solved, 'topic pub should move LIMO but not solve the ticket');
  const T = ros2run(sim, 'ros2 run limo_base limo_base_node --ros-args -p max_linear_speed:=1');
  assert(T.lines.join('\n').includes('InvalidParameterTypeException') && !T.proc, 'expected the parameter type crash');
});
test('Read-only parameters must be set at start-up, and hints and "show the fix" cost XP', () => {
  const sim = open('frame-id'), e = sim.bus.nodes.get('/ydlidar_node').params.get('frame_id');
  assert(e.ro, 'frame_id should be read-only at runtime');
  const tk = ticket('slow'), F = { hints: 2, shown: false };
  assert(L.SnoFix.xp(tk, F) === 20 && L.SnoFix.xp(tk, { hints: 3, shown: true }) === 10 && L.SnoFix.xp(ticket('qos'), { hints: 1 }) === 60, 'xp rule');
});
test('Fixing a ticket again gives no more XP; closing the ticket restores the normal robot', () => {
  const x0 = L.getXP(), sim = open('slow'); steps(sim, 0.5);
  paramSet(sim, '/limo_base', 'max_linear_speed', 1.0); drive(sim, 2);
  assert(sim.fix.solved && L.getXP() === x0, `XP went from ${x0} to ${L.getXP()}`);
  L.SnoFix.quit(sim); steps(sim, 1);
  const nodes = [...sim.bus.nodes.values()].filter(n => !n.hidden && !n.name.startsWith('/_')).map(n => n.name);
  assert(NORMAL.every(n => nodes.includes(n)) && !nodes.includes('/slam_toolbox'), `nodes ${nodes}`);
  assert(sim.P('/limo_base', 'max_linear_speed') === 1.0 && sim.P('/ydlidar_node', 'frame_id') === 'laser_link' && sim.frames().includes('laser_link'), 'not the normal robot');
  assert(!sim.fix, 'ticket still open');
});
run();
