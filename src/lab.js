/* ============================== Robot Lab: world + physics ============================== */
const LAB = { W: 10, H: 7, R: 0.2 };
const OBST = { boxes: [[3.0, 3.6, 3.4, 7.0], [5.5, 0.9, 6.5, 1.9], [8.2, 2.2, 9.0, 3.0]], circles: [[6.2, 5.6, 0.45], [1.8, 5.2, 0.4], [4.6, 4.4, 0.25]] };
const PAD = { x: 8.6, y: 5.6, r: 0.45 }, HOME = { x: 1.0, y: 1.0, th: 0 };
const BEACONS = [[4.7, 0.6], [7.3, 3.6], [1.6, 6.2]];
const SEGS = (() => { const s = [[0, 0, 10, 0], [10, 0, 10, 7], [10, 7, 0, 7], [0, 7, 0, 0]]; OBST.boxes.forEach(([a, b, c, d]) => s.push([a, b, c, b], [c, b, c, d], [c, d, a, d], [a, d, a, b])); return s; })();
const NRAYS = 360;
function raycast(ox, oy, a, maxR) {
  const dx = Math.cos(a), dy = Math.sin(a); let best = Infinity;
  for (const [x1, y1, x2, y2] of SEGS) {
    const ex = x2 - x1, ey = y2 - y1, den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-12) continue;
    const t = ((x1 - ox) * ey - (y1 - oy) * ex) / den, u = ((x1 - ox) * dy - (y1 - oy) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  for (const [cx, cy, r] of OBST.circles) {
    const fx = ox - cx, fy = oy - cy, b = fx * dx + fy * dy, c = fx * fx + fy * fy - r * r, disc = b * b - c;
    if (disc < 0) continue; const t = -b - Math.sqrt(disc); if (t > 0 && t < best) best = t;
  }
  return best <= maxR ? best : Infinity;
}
function collide(p) {
  let hit = false;
  for (const [x1, y1, x2, y2] of SEGS) {
    const ex = x2 - x1, ey = y2 - y1, L2 = ex * ex + ey * ey; const u = clamp(((p.x - x1) * ex + (p.y - y1) * ey) / L2, 0, 1);
    const cx = x1 + u * ex, cy = y1 + u * ey, dx = p.x - cx, dy = p.y - cy, d = Math.hypot(dx, dy);
    if (d < LAB.R) { const nx = d > 1e-9 ? dx / d : 0, ny = d > 1e-9 ? dy / d : 1; p.x = cx + nx * LAB.R; p.y = cy + ny * LAB.R; hit = true; }
  }
  for (const [cx, cy, r] of OBST.circles) {
    const dx = p.x - cx, dy = p.y - cy, d = Math.hypot(dx, dy), m = r + LAB.R;
    if (d < m) { p.x = cx + dx / (d || 1) * m; p.y = cy + dy / (d || 1) * m; hit = true; }
  }
  return hit;
}
// header stamps: wall-clock time, or simulated seconds while a simulator owns the clock (ROS use_sim_time)
let STAMP_CLOCK = null;
const stampNow = () => { if (STAMP_CLOCK) { const t = Math.max(0, +STAMP_CLOCK() || 0), sec = Math.floor(t); return { sec, nanosec: Math.min(999999999, Math.floor((t - sec) * 1e9)) }; } const t = Date.now(); return { sec: Math.floor(t / 1000), nanosec: (t % 1000) * 1e6 }; };
const zeros36 = () => new Array(36).fill(0);

/* ============================== interfaces ============================== */
const IFACES = {
  'builtin_interfaces/msg/Time': { f: [['sec', 'int32'], ['nanosec', 'uint32']] },
  'std_msgs/msg/Header': { f: [['stamp', 'builtin_interfaces/msg/Time'], ['frame_id', 'string']] },
  'std_msgs/msg/String': { f: [['data', 'string']] },
  'std_msgs/msg/Bool': { f: [['data', 'bool']] },
  'std_msgs/msg/Int32': { f: [['data', 'int32']] },
  'std_msgs/msg/Float32': { f: [['data', 'float32']] },
  'std_msgs/msg/Float64': { f: [['data', 'float64']] },
  'geometry_msgs/msg/Vector3': { f: [['x', 'float64'], ['y', 'float64'], ['z', 'float64']] },
  'geometry_msgs/msg/Point': { f: [['x', 'float64'], ['y', 'float64'], ['z', 'float64']] },
  'geometry_msgs/msg/Quaternion': { f: [['x', 'float64', 0], ['y', 'float64', 0], ['z', 'float64', 0], ['w', 'float64', 1]] },
  'geometry_msgs/msg/Pose': { f: [['position', 'geometry_msgs/msg/Point'], ['orientation', 'geometry_msgs/msg/Quaternion']] },
  'geometry_msgs/msg/PoseStamped': { f: [['header', 'std_msgs/msg/Header'], ['pose', 'geometry_msgs/msg/Pose']] },
  'geometry_msgs/msg/PointStamped': { f: [['header', 'std_msgs/msg/Header'], ['point', 'geometry_msgs/msg/Point']], head: '# This represents a Point with reference coordinate frame and timestamp' },
  'geometry_msgs/msg/Twist': { f: [['linear', 'geometry_msgs/msg/Vector3'], ['angular', 'geometry_msgs/msg/Vector3']], head: '# This expresses velocity in free space broken into its linear and angular parts.\n' },
  'geometry_msgs/msg/TwistStamped': { f: [['header', 'std_msgs/msg/Header'], ['twist', 'geometry_msgs/msg/Twist']], head: '# A twist with reference coordinate frame and timestamp' },
  'rcl_interfaces/msg/Log': { f: [['stamp', 'builtin_interfaces/msg/Time'], ['level', 'uint8'], ['name', 'string'], ['msg', 'string'], ['file', 'string'], ['function', 'string'], ['line', 'uint32']], head: '# Every node publishes its log messages here (DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50)' },
  'geometry_msgs/msg/PoseWithCovariance': { f: [['pose', 'geometry_msgs/msg/Pose'], ['covariance', 'float64[36]']] },
  'geometry_msgs/msg/TwistWithCovariance': { f: [['twist', 'geometry_msgs/msg/Twist'], ['covariance', 'float64[36]']] },
  'geometry_msgs/msg/Transform': { f: [['translation', 'geometry_msgs/msg/Vector3'], ['rotation', 'geometry_msgs/msg/Quaternion']] },
  'geometry_msgs/msg/TransformStamped': { f: [['header', 'std_msgs/msg/Header'], ['child_frame_id', 'string'], ['transform', 'geometry_msgs/msg/Transform']] },
  'tf2_msgs/msg/TFMessage': { f: [['transforms', 'geometry_msgs/msg/TransformStamped[]']] },
  'nav_msgs/msg/Odometry': { f: [['header', 'std_msgs/msg/Header'], ['child_frame_id', 'string'], ['pose', 'geometry_msgs/msg/PoseWithCovariance'], ['twist', 'geometry_msgs/msg/TwistWithCovariance']], head: '# This represents an estimate of a position and velocity in free space.\n' },
  'sensor_msgs/msg/LaserScan': { f: [['header', 'std_msgs/msg/Header'], ['angle_min', 'float32'], ['angle_max', 'float32'], ['angle_increment', 'float32'], ['time_increment', 'float32'], ['scan_time', 'float32'], ['range_min', 'float32'], ['range_max', 'float32'], ['ranges', 'float32[]'], ['intensities', 'float32[]']], head: '# Single scan from a planar laser range-finder\n' },
  'std_msgs/msg/Int32MultiArray': { f: [['data', 'int32[]']] },
  'sensor_msgs/msg/JointState': { f: [['header', 'std_msgs/msg/Header'], ['name', 'string[]'], ['position', 'float64[]'], ['velocity', 'float64[]'], ['effort', 'float64[]']], head: '# This is a message that holds data to describe the state of a set of torque controlled joints.\n' },
  'sensor_msgs/msg/RegionOfInterest': { f: [['x_offset', 'uint32'], ['y_offset', 'uint32'], ['height', 'uint32'], ['width', 'uint32'], ['do_rectify', 'bool']], head: '# This message is used to specify a region of interest within an image.\n' },
  'sensor_msgs/msg/Image': { f: [['header', 'std_msgs/msg/Header'], ['height', 'uint32'], ['width', 'uint32'], ['encoding', 'string'], ['is_bigendian', 'uint8'], ['step', 'uint32'], ['data', 'uint8[]']], head: '# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n' },
  'sensor_msgs/msg/CameraInfo': { f: [['header', 'std_msgs/msg/Header'], ['height', 'uint32'], ['width', 'uint32'], ['distortion_model', 'string'], ['d', 'float64[]'], ['k', 'float64[9]'], ['r', 'float64[9]'], ['p', 'float64[12]'], ['binning_x', 'uint32'], ['binning_y', 'uint32'], ['roi', 'sensor_msgs/msg/RegionOfInterest']], head: '# This message defines meta information for a camera. It should be in a\n# camera namespace on topic "camera_info" and accompanied by up to five\n# image topics named: image_raw, image, image_color, image_rect, image_rect_color\n' },
  'sensor_msgs/msg/Imu': { f: [['header', 'std_msgs/msg/Header'], ['orientation', 'geometry_msgs/msg/Quaternion'], ['orientation_covariance', 'float64[9]'], ['angular_velocity', 'geometry_msgs/msg/Vector3'], ['angular_velocity_covariance', 'float64[9]'], ['linear_acceleration', 'geometry_msgs/msg/Vector3'], ['linear_acceleration_covariance', 'float64[9]']], head: '# This is a message to hold data from an IMU (Inertial Measurement Unit)\n' },
  'nav_msgs/msg/Path': { f: [['header', 'std_msgs/msg/Header'], ['poses', 'geometry_msgs/msg/PoseStamped[]']], head: '# An array of poses that represents a Path for a robot to follow.\n' },
  'nav_msgs/msg/MapMetaData': { f: [['map_load_time', 'builtin_interfaces/msg/Time'], ['resolution', 'float32'], ['width', 'uint32'], ['height', 'uint32'], ['origin', 'geometry_msgs/msg/Pose']] },
  'nav_msgs/msg/OccupancyGrid': { f: [['header', 'std_msgs/msg/Header'], ['info', 'nav_msgs/msg/MapMetaData'], ['data', 'int8[]']], head: '# This represents a 2-D grid map\n' },
  'limo_msgs/msg/LimoStatus': { f: [['header', 'std_msgs/msg/Header'], ['vehicle_state', 'uint8'], ['control_mode', 'uint8'], ['battery_voltage', 'float64'], ['error_code', 'uint16'], ['motion_mode', 'uint8']] },
  'snobots_msgs/msg/CubeDetection': { f: [['header', 'std_msgs/msg/Header'], ['color', 'string'], ['position', 'geometry_msgs/msg/Point'], ['yaw', 'float64'], ['size_px', 'float64']], head: '# A coloured cube found in a camera image: its centre in header.frame_id, its yaw (radians, -pi/4 to pi/4) and its width in pixels' },
  'snobots_msgs/msg/ColorDetection': { f: [['header', 'std_msgs/msg/Header'], ['color', 'string'], ['found', 'bool'], ['x', 'float32'], ['y', 'float32'], ['area', 'float32']], head: '# SnoBots sim: largest colour blob in the head camera. x, y in [-1, 1] (right, down); area = image fraction\n' },
  'snobots_msgs/msg/MotorCmd': { f: [['mode', 'uint8'], ['q', 'float32'], ['dq', 'float32'], ['tau', 'float32'], ['kp', 'float32'], ['kd', 'float32']] },
  'snobots_msgs/msg/LowCmd': { f: [['motor_cmd', 'snobots_msgs/msg/MotorCmd[26]']], head: '# SnoBots sim: one PD command per motor (index order in the Info tab). mode 1 = apply, 0 = keep current\n' },
  'snobots_msgs/msg/MotorState': { f: [['q', 'float32'], ['dq', 'float32'], ['tau_est', 'float32'], ['temperature', 'int16']] },
  'snobots_msgs/msg/LowState': { f: [['tick', 'uint32'], ['motor_state', 'snobots_msgs/msg/MotorState[]']] },
  'snobots_msgs/msg/ArmAngles': { f: [['angles', 'float32[]'], ['speed', 'int32']], head: '# SnoBots sim: 6 joint angles in degrees, speed 1-100 (like send_angles)\n' },
  'snobots_msgs/msg/ArmCoords': { f: [['x', 'float32'], ['y', 'float32'], ['z', 'float32'], ['rx', 'float32'], ['ry', 'float32'], ['rz', 'float32'], ['speed', 'int32']], head: '# SnoBots sim: gripper pose in the arm base frame. x y z in mm, rx ry rz in degrees (like send_coords)\n' },
  'snobots_msgs/srv/SetMode': { srv: [[['mode', 'string']], [['success', 'bool'], ['message', 'string']]] },
  'std_srvs/srv/Empty': { srv: [[], []] },
  'std_srvs/srv/Trigger': { srv: [[], [['success', 'bool'], ['message', 'string']]] },
  'std_srvs/srv/SetBool': { srv: [[['data', 'bool']], [['success', 'bool'], ['message', 'string']]] },
  'example_interfaces/srv/AddTwoInts': { srv: [[['a', 'int64'], ['b', 'int64']], [['sum', 'int64']]] },
  'turtlesim/srv/Spawn': { srv: [[['x', 'float32'], ['y', 'float32'], ['theta', 'float32'], ['name', 'string']], [['name', 'string']]] },
};
const isMsg = (t) => !!(IFACES[t] && IFACES[t].f);
const baseT = (t) => t.replace(/\[\d*\]$/, '');
const isArr = (t) => /\]$/.test(t);
const isFloatT = (t) => /^float/.test(t), isIntT = (t) => /^u?int/.test(t);
const pyType = (t) => t.replace('/msg/', '.msg.').replace('/srv/', '.srv.');
function shortRef(t, home) { const [p, , n] = t.split('/'); return p === home ? n : `${p}/${n}`; }
function showIface(type) {
  const it = IFACES[type]; if (!it) return null;
  const pkg = type.split('/')[0];
  const lines = [];
  const walk = (fields, ind, home) => fields.forEach(([n, t, d]) => {
    const bt = baseT(t);
    if (isMsg(bt)) { lines.push(`${ind}${shortRef(bt, home)}${isArr(t) ? t.slice(bt.length) : ''} ${n}`); walk(IFACES[bt].f, ind + '\t', bt.split('/')[0]); }
    else lines.push(`${ind}${t} ${n}${d != null ? ' ' + d : ''}`);
  });
  if (it.srv) { walk(it.srv[0], '', pkg); lines.push('---'); walk(it.srv[1], '', pkg); }
  else { walk(it.f, '', pkg); }
  return (it.head ? it.head + '\n' : '') + lines.join('\n');
}
function defVal(t) { if (isArr(t)) { if (!/\[\d+\]$/.test(t)) return []; const n = +t.match(/\[(\d+)\]$/)[1], bt = baseT(t); return Array.from({ length: n }, () => defVal(bt)); } if (isMsg(t)) return defMsg(t); if (t === 'string') return ''; if (t === 'bool') return false; return 0; }
function defMsg(t, fields) { const o = {}; (fields || IFACES[t].f).forEach(([n, ft, d]) => { o[n] = d != null ? d : defVal(ft); }); return o; }
function fillMsg(type, obj, fields) {
  const fl = fields || IFACES[type].f, out = defMsg(type, fl), nm = type.split('/').pop();
  if (obj == null) return out;
  if (typeof obj !== 'object' || Array.isArray(obj)) throw new Error(`Failed to populate message: expected a mapping for '${nm}'`);
  for (const k of Object.keys(obj)) {
    const f = fl.find(x => x[0] === k);
    if (!f) throw new Error(`Failed to populate field: '${nm}' object has no attribute '${k}'`);
    const ft = f[1], v = obj[k];
    if (isMsg(ft)) out[k] = fillMsg(ft, v);
    else if (isArr(ft)) { if (!Array.isArray(v)) throw new Error(`Failed to populate field '${k}': expected a list`); const bt = baseT(ft); out[k] = isMsg(bt) ? v.map(x => fillMsg(bt, x)) : bt === 'string' ? v.map(String) : bt === 'bool' ? v.map(Boolean) : v.map(Number); }
    else if (ft === 'bool') { if (typeof v !== 'boolean') throw new Error(`Failed to populate field '${k}': expected true or false`); out[k] = v; }
    else if (ft === 'string') out[k] = String(v);
    else { if (typeof v !== 'number') throw new Error(`Failed to populate field '${k}': expected a number`); out[k] = isIntT(ft) ? Math.trunc(v) : v; }
  }
  return out;
}
const pyFloat = (v) => !Number.isFinite(v) ? (Number.isNaN(v) ? 'nan' : (v > 0 ? 'inf' : '-inf')) : Number.isInteger(v) ? v.toFixed(1) : String(+v.toPrecision(12));
function pyRepr(type, obj, fields, name) {
  const fl = fields || IFACES[type].f;
  const parts = fl.map(([n, ft]) => {
    const v = obj[n];
    if (isMsg(ft)) return `${n}=${pyRepr(ft, v)}`;
    if (isArr(ft)) { const bt = baseT(ft), a = v || []; return `${n}=[${a.slice(0, 6).map(x => isMsg(bt) ? pyRepr(bt, x) : bt === 'string' ? `'${x}'` : isFloatT(bt) ? pyFloat(+x) : String(Math.trunc(+x))).join(', ')}${a.length > 6 ? ', ...' : ''}]`; }
    if (ft === 'bool') return `${n}=${v ? 'True' : 'False'}`;
    if (ft === 'string') return `${n}='${String(v).replace(/'/g, "\\'")}'`;
    return `${n}=${isFloatT(ft) ? pyFloat(+v) : Math.trunc(+v)}`;
  });
  return `${name || pyType(type)}(${parts.join(', ')})`;
}
const yScalar = (ft, v) => {
  if (ft === 'bool') return v ? 'true' : 'false';
  if (ft === 'string') { const s = String(v ?? ''); return (!s || /[:#{}[\],&*!|>'"%@`]|^\s|\s$|^(true|false|null|~|-?\d)/i.test(s)) ? `'${s.replace(/'/g, "''")}'` : s; }
  if (v === null || v === undefined) return '.nan';
  if (isFloatT(ft)) return v === Infinity ? '.inf' : v === -Infinity ? '-.inf' : Number.isNaN(v) ? '.nan' : pyFloat(v);
  return String(Math.trunc(v));
};
// rough serialized size of a message (CDR-like), for ros2 topic bw
function msgBytes(m) {
  if (m == null) return 0;
  if (ArrayBuffer.isView(m)) return 4 + m.byteLength;
  if (typeof m === 'string') return 5 + m.length;
  if (typeof m === 'number') return 8;
  if (typeof m === 'boolean') return 1;
  if (Array.isArray(m)) return 4 + m.reduce((a, x) => a + msgBytes(x), 0);
  if (typeof m === 'object') return Object.values(m).reduce((a, x) => a + msgBytes(x), 0);
  return 0;
}
function yamlMsg(type, obj, ind = '', out = [], fields) {
  (fields || IFACES[type].f).forEach(([n, ft]) => {
    const v = obj ? obj[n] : undefined;
    if (isMsg(ft)) { out.push(`${ind}${n}:`); yamlMsg(ft, v, ind + '  ', out); }
    else if (isArr(ft)) {
      const bt = baseT(ft), arr = v || [];
      if (!arr.length) { out.push(`${ind}${n}: []`); return; }
      out.push(`${ind}${n}:`);
      if (isMsg(bt)) { arr.slice(0, 8).forEach(el => { const sub = []; yamlMsg(bt, el, ind + '  ', sub); sub[0] = ind + '- ' + sub[0].trimStart(); out.push(...sub); }); if (arr.length > 8) out.push(`${ind}- '... ${arr.length - 8} more'`); }
      else { arr.slice(0, 10).forEach(x => out.push(`${ind}- ${yScalar(bt, x)}`)); if (arr.length > 10) out.push(`${ind}- '...'`); }
    } else out.push(`${ind}${n}: ${yScalar(ft, v)}`);
  });
  return out;
}
function parseYaml(src) {
  let s = String(src).trim(); if (!s) return {};
  if (s[0] !== '{' && s[0] !== '[') s = '{' + s + '}';
  let i = 0;
  const ws = () => { while (i < s.length && /\s/.test(s[i])) i++; };
  const fail = (m) => { throw new Error(`could not parse values: ${m}`); };
  const scalar = () => {
    ws();
    if (s[i] === '"' || s[i] === "'") { const q = s[i++]; let o = ''; while (i < s.length && s[i] !== q) o += s[i++]; if (s[i] !== q) fail('unclosed quote'); i++; return o; }
    const m = /^[^,}\]]+/.exec(s.slice(i)); if (!m) fail(`unexpected '${s[i] || 'end'}'`); i += m[0].length; const t = m[0].trim();
    if (/^[-+]?(\d+\.?\d*|\.\d+)(e[-+]?\d+)?$/i.test(t)) return Number(t);
    if (/^(true|True)$/.test(t)) return true; if (/^(false|False)$/.test(t)) return false;
    if (/^[-+]?\.inf$/i.test(t)) return t[0] === '-' ? -Infinity : Infinity;
    return t;
  };
  const val = () => { ws(); if (s[i] === '{') return obj(); if (s[i] === '[') return arr(); return scalar(); };
  const key = () => { ws(); const m = /^['"]?([A-Za-z_]\w*)['"]?/.exec(s.slice(i)); if (!m) fail(`expected a field name near '${s.slice(i, i + 8)}'`); i += m[0].length; return m[1]; };
  const obj = () => { const o = {}; i++; ws(); if (s[i] === '}') { i++; return o; } for (;;) { const k = key(); ws(); if (s[i] !== ':') fail(`expected ':' after '${k}'`); i++; o[k] = val(); ws(); if (s[i] === ',') { i++; continue; } if (s[i] === '}') { i++; return o; } fail(`expected ',' or '}'`); } };
  const arr = () => { const a = []; i++; ws(); if (s[i] === ']') { i++; return a; } for (;;) { a.push(val()); ws(); if (s[i] === ',') { i++; continue; } if (s[i] === ']') { i++; return a; } fail(`expected ',' or ']'`); } };
  const r = val(); ws(); if (i < s.length) fail(`unexpected '${s.slice(i, i + 10)}'`);
  return r;
}

/* ============================== simulated ROS graph ============================== */
const HIDE_TOPICS = new Set(['/parameter_events', '/rosout']);
// Quality of service: every publisher and subscription has a profile. A subscription only receives from publishers whose
// offer is at least as strong as its request (a RELIABLE reader ignores a BEST_EFFORT writer; TRANSIENT_LOCAL needs TRANSIENT_LOCAL).
const QOS_DEF = { rel: 'RELIABLE', dur: 'VOLATILE', depth: 10 };
const qosOf = (q) => Object.assign({}, QOS_DEF, q || {});
function qosClash(pq, sq) { pq = pq || QOS_DEF; sq = sq || QOS_DEF; if (pq.rel === 'BEST_EFFORT' && sq.rel === 'RELIABLE') return 'RELIABILITY'; if (pq.dur === 'VOLATILE' && sq.dur === 'TRANSIENT_LOCAL') return 'DURABILITY'; return ''; }
function makeBus() { return {
  nodes: new Map(), topics: new Map(), listeners: new Map(), services: new Map(), pid: 4100, clashes: new Set(), onClash: null, onPublish: null, onRemove: null,
  addNode(name, kind = 'other', opts = {}) {
    if (this.nodes.has(name)) return this.nodes.get(name);
    const n = { name, kind, pubs: new Set(), subs: new Set(), params: new Map(), hidden: !!opts.hidden, onParam: opts.onParam || null, user: !!opts.user };
    if (!opts.noParams) { n.params.set('use_sim_time', { v: false, t: 'bool' }); n.params.set('start_type_description_service', { v: true, t: 'bool', ro: true }); }
    this.nodes.set(name, n); return n;
  },
  removeNode(name) {
    const n = this.nodes.get(name); if (!n) return;
    n.pubs.forEach(t => { const T = this.topics.get(t); if (T) T.pubs.delete(name); });
    n.subs.forEach(t => { const T = this.topics.get(t); if (T) T.subs.delete(name); const L = this.listeners.get(t); if (L) [...L].forEach(x => { if (x.node === name) L.delete(x); }); });
    [...this.services].forEach(([k, s]) => { if (s.node === name) this.services.delete(k); });
    this.topics.forEach(T => { T.pubQos.delete(name); T.subQos.delete(name); if (T.held) T.held.delete(name); });
    [...this.clashes].forEach(k => { if (k.split('|').includes(name)) this.clashes.delete(k); });
    this.nodes.delete(name);
    if (this.onRemove) this.onRemove(name);
    [...this.topics].forEach(([k, T]) => { if (!T.pubs.size && !T.subs.size && !T.keep) this.topics.delete(k); });
  },
  topic(name, type) { let T = this.topics.get(name); if (!T) { T = { name, type, pubs: new Map(), subs: new Map(), pubQos: new Map(), subQos: new Map(), times: [], last: null, lastType: type, count: 0, pubAt: new Map() }; this.topics.set(name, T); } if (!T.type) T.type = type; return T; },
  advertise(node, topic, type, qos) { const T = this.topic(topic, type); T.pubs.set(node, type); if (qos || !T.pubQos.has(node)) T.pubQos.set(node, qosOf(qos)); const n = this.nodes.get(node); if (n) n.pubs.add(topic); return T; },
  // every type seen on a topic name (publishers and subscriptions can disagree)
  types(topic) { const T = this.topics.get(topic); if (!T) return []; const s = new Set([...T.pubs.values(), ...T.subs.values()]); if (!s.size && T.type) s.add(T.type); return [...s]; },
  subscribe(node, topic, type, fn, qos) {
    const T = this.topic(topic, type); T.subs.set(node, type); const q = qosOf(qos); T.subQos.set(node, q);
    const n = this.nodes.get(node); if (n) n.subs.add(topic);
    let L = this.listeners.get(topic); if (!L) { L = new Set(); this.listeners.set(topic, L); }
    const rec = { node, type, fn, qos: q }; L.add(rec);
    // a latched topic (transient local) hands a late subscriber the last message of every publisher still running
    if (T.latched) {
      const held = T.held ? [...T.held] : T.last ? [[T.lastFrom || '/robot_sim', { type: T.lastType, msg: T.last }]] : [];
      if (held.length) setTimeout(() => held.forEach(([from, x]) => {
        if (!L.has(rec) || x.type !== type || from === node) return;
        const bad = qosClash(T.pubQos.get(from), q); if (bad) { this.clash(from, node, topic, bad); return; }
        try { fn(x.msg, from); } catch (e) { console.error(e); }
      }), 0);
    }
    return () => { L.delete(rec); const TT = this.topics.get(topic); if (TT && ![...L].some(x => x.node === node)) { TT.subs.delete(node); TT.subQos.delete(node); } };
  },
  publish(node, topic, type, msg) {
    const T = this.topic(topic, type); const now = performance.now();
    if (!T.pubs.has(node)) this.advertise(node, topic, type);
    T.last = msg; T.lastType = type; T.lastFrom = node; T.count++; T.times.push(now); if (T.times.length > 80) T.times.shift(); T.pubAt.set(node, now);
    if (T.latched) (T.held || (T.held = new Map())).set(node, { type, msg });
    const L = this.listeners.get(topic), pq = T.pubQos.get(node) || QOS_DEF;
    if (L) [...L].forEach(x => {
      if (x.type !== type || x.node === node) return;
      const bad = qosClash(pq, x.qos); if (bad) { this.clash(node, x.node, topic, bad); return; }
      try { x.fn(msg, node); } catch (e) { console.error(e); }
    });
    if (this.onPublish) this.onPublish(node, topic, type, msg);
  },
  // reported once per publisher/subscription pair, like the warning DDS discovery gives
  clash(pub, sub, topic, policy) { const k = `${pub}|${sub}|${topic}`; if (this.clashes.has(k)) return; this.clashes.add(k); if (this.onClash) this.onClash(pub, sub, topic, policy); },
  hz(topic) { const T = this.topics.get(topic); if (!T) return 0; const now = performance.now(); const ts = T.times.filter(t => now - t < 2000); if (ts.length < 2) return ts.length && now - ts[0] < 1100 ? 1 : 0; return (ts.length - 1) / ((ts[ts.length - 1] - ts[0]) / 1000); },
  active(topic, node) { const T = this.topics.get(topic); if (!T) return false; const now = performance.now(); if (node) { const t = T.pubAt.get(node); return t != null && now - t < 1600; } return T.times.length && now - T.times[T.times.length - 1] < 1600; },
  service(name, type, node, handler) { this.services.set(name, { type, node, handler }); },
}; }
const Bus = makeBus();

/* ============================== robot simulation ============================== */
const Sim = {
  x: HOME.x, y: HOME.y, th: HOME.th, v: 0, w: 0, cmd: { v: 0, w: 0, t: -1e9, src: '', kind: '' }, bumps: 0, bumpT: 0, trail: [], got: [false, false, false], lidar: true, ranges: new Array(NRAYS).fill(Infinity), goal: null,
  acc: 0, tOdom: 0, tScan: 0, tBump: 0, lastBump: false, dist: {},
  params: null,
  init() {
    const n = Bus.addNode('/robot_sim', 'sim', { onParam: (name, v) => this.setParam(name, v) });
    const P = [['max_linear_speed', 0.6, 'double', [0, 1.5]], ['max_angular_speed', 2.0, 'double', [0, 4]], ['lidar_range', 3.5, 'double', [0.5, 10]], ['cmd_timeout', 1.2, 'double', [0.1, 5]], ['show_rays', true, 'bool']];
    P.forEach(([k, v, t, r]) => n.params.set(k, { v, t, range: r }));
    this.params = n.params;
    Bus.subscribe('/robot_sim', '/cmd_vel', 'geometry_msgs/msg/Twist', (m, from) => { const n = Bus.nodes.get(from); this.cmd = { v: +m.linear.x || 0, w: +m.angular.z || 0, t: performance.now() / 1000, src: from, kind: n ? (n.user ? 'user' : n.kind) : '' }; });
    ['/odom', 'nav_msgs/msg/Odometry', '/scan', 'sensor_msgs/msg/LaserScan', '/tf', 'tf2_msgs/msg/TFMessage', '/tf_static', 'tf2_msgs/msg/TFMessage', '/bumper', 'std_msgs/msg/Bool'].forEach((x, i, a) => { if (i % 2 === 0) Bus.advertise('/robot_sim', x, a[i + 1]); });
    Bus.topic('/parameter_events', 'rcl_interfaces/msg/ParameterEvent').keep = true; Bus.topic('/rosout', 'rcl_interfaces/msg/Log').keep = true;
    Bus.service('/reset_world', 'std_srvs/srv/Empty', '/robot_sim', () => { this.reset(); Missions.event('service', '/reset_world'); return {}; });
    Bus.service('/spawn_beacons', 'std_srvs/srv/Trigger', '/robot_sim', () => { this.got = [false, false, false]; return { success: true, message: '3 beacons spawned' }; });
    Bus.service('/set_lidar', 'std_srvs/srv/SetBool', '/robot_sim', (rq) => { this.lidar = !!rq.data; if (!this.lidar) this.ranges.fill(Infinity); return { success: true, message: this.lidar ? 'lidar on' : 'lidar off' }; });
    Bus.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true;
    Bus.publish('/robot_sim', '/tf_static', 'tf2_msgs/msg/TFMessage', { transforms: [{ header: { stamp: stampNow(), frame_id: 'base_link' }, child_frame_id: 'base_scan', transform: { translation: { x: 0, y: 0, z: 0.12 }, rotation: { x: 0, y: 0, z: 0, w: 1 } } }] });
  },
  P(k) { return this.params.get(k).v; },
  setParam(name, v) { if (name === 'max_linear_speed') Missions.event('param', { node: '/robot_sim', name, v }); return { ok: true }; },
  reset() { Object.assign(this, { x: HOME.x, y: HOME.y, th: HOME.th, v: 0, w: 0, cmd: { v: 0, w: 0, t: -1e9, src: '', kind: '' }, bumps: 0, trail: [], got: [false, false, false] }); Missions.st.run = { dist: 0, bumps: 0 }; },
  step(dt) {
    const now = performance.now() / 1000;
    const live = now - this.cmd.t < this.P('cmd_timeout');
    const vmax = this.P('max_linear_speed'), wmax = this.P('max_angular_speed');
    const tv = live ? clamp(this.cmd.v, -vmax, vmax) : 0, tw = live ? clamp(this.cmd.w, -wmax, wmax) : 0;
    this.v += clamp(tv - this.v, -1.6 * dt, 1.6 * dt); this.w += clamp(tw - this.w, -7 * dt, 7 * dt);
    const ox = this.x, oy = this.y;
    this.th = wrapAngle(this.th + this.w * dt); this.x += this.v * Math.cos(this.th) * dt; this.y += this.v * Math.sin(this.th) * dt;
    const p = { x: this.x, y: this.y }; const hit = collide(p); this.x = p.x; this.y = p.y;
    const moved = Math.hypot(this.x - ox, this.y - oy);
    if (moved > 0 && live) Missions.move(this.cmd.kind, moved);
    if (hit) { this.v *= 0.2; if (now - this.bumpT > 0.6) { this.bumps++; this.bumpT = now; Missions.event('bump'); Sound.play('bump'); } }
    this.bumping = hit;
    if (!this.trail.length || Math.hypot(this.x - this.trail[this.trail.length - 1][0], this.y - this.trail[this.trail.length - 1][1]) > 0.04) { this.trail.push([this.x, this.y]); if (this.trail.length > 700) this.trail.shift(); }
    BEACONS.forEach(([bx, by], i) => { if (!this.got[i] && Math.hypot(this.x - bx, this.y - by) < 0.32) { this.got[i] = true; Sound.play('coin'); Missions.event('beacon', this.got.filter(Boolean).length); } });
    if (Math.hypot(this.x - PAD.x, this.y - PAD.y) < 0.35) Missions.event('pad');
    this.tOdom += dt; this.tScan += dt; this.tBump += dt;
    if (this.tOdom >= 0.05) { this.tOdom = 0; this.pubOdom(); }
    if (this.tScan >= 0.1) { this.tScan = 0; this.scan(); }
    if (hit !== this.lastBump || this.tBump >= 1) { this.tBump = 0; this.lastBump = hit; Bus.publish('/robot_sim', '/bumper', 'std_msgs/msg/Bool', { data: hit }); }
  },
  pubOdom() {
    const st = stampNow(), qz = Math.sin(this.th / 2), qw = Math.cos(this.th / 2);
    Bus.publish('/robot_sim', '/odom', 'nav_msgs/msg/Odometry', { header: { stamp: st, frame_id: 'odom' }, child_frame_id: 'base_link', pose: { pose: { position: { x: this.x, y: this.y, z: 0 }, orientation: { x: 0, y: 0, z: qz, w: qw } }, covariance: zeros36() }, twist: { twist: { linear: { x: this.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: this.w } }, covariance: zeros36() } });
    Bus.publish('/robot_sim', '/tf', 'tf2_msgs/msg/TFMessage', { transforms: [{ header: { stamp: st, frame_id: 'odom' }, child_frame_id: 'base_link', transform: { translation: { x: this.x, y: this.y, z: 0 }, rotation: { x: 0, y: 0, z: qz, w: qw } } }] });
  },
  scan() {
    if (!this.lidar) return;
    const maxR = this.P('lidar_range'), inc = 2 * Math.PI / NRAYS;
    for (let i = 0; i < NRAYS; i++) { const r = raycast(this.x, this.y, this.th + i * inc, maxR); this.ranges[i] = r < 0.12 ? 0.12 : r; }
    Bus.publish('/robot_sim', '/scan', 'sensor_msgs/msg/LaserScan', { header: { stamp: stampNow(), frame_id: 'base_scan' }, angle_min: 0, angle_max: +(2 * Math.PI - inc).toFixed(6), angle_increment: +inc.toFixed(6), time_increment: 0, scan_time: 0.1, range_min: 0.12, range_max: maxR, ranges: this.ranges.map(r => Number.isFinite(r) ? +r.toFixed(3) : Infinity), intensities: [] });
  },
};

/* ============================== built-in nodes ============================== */
const Builtin = {
  teleopV: 0, teleopW: 0, teleopOn: false, teleopTimer: null, teleopLast: 0,
  teleop(v, w, src = '/teleop_panel') {
    this.teleopV = v; this.teleopW = w; this.teleopLast = performance.now();
    if (!Bus.nodes.has(src)) { Bus.addNode(src, 'teleop'); Bus.advertise(src, '/cmd_vel', 'geometry_msgs/msg/Twist'); }
    const send = () => Bus.publish(src, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: this.teleopV, y: 0, z: 0 }, angular: { x: 0, y: 0, z: this.teleopW } });
    send();
    if (!this.teleopTimer) this.teleopTimer = setInterval(() => { if (this.teleopV || this.teleopW) send(); else if (performance.now() - this.teleopLast > 400) { send(); clearInterval(this.teleopTimer); this.teleopTimer = null; } }, 100);
  },
  wander(opts = {}) {
    const name = '/wander'; if (Bus.nodes.has(name)) return null;
    const n = Bus.addNode(name, 'wander', { onParam: (k, v) => { if (k === 'speed') Missions.event('wanderParam'); return { ok: true }; } });
    n.params.set('speed', { v: opts.speed ?? 0.3, t: 'double', range: [0, 1] }); n.params.set('safe_distance', { v: 0.55, t: 'double', range: [0.2, 2] }); n.params.set('turn_speed', { v: 1.2, t: 'double', range: [0, 3] });
    Bus.advertise(name, '/cmd_vel', 'geometry_msgs/msg/Twist');
    let cmd = { v: 0, w: 0 };
    const un = Bus.subscribe(name, '/scan', 'sensor_msgs/msg/LaserScan', (m) => {
      const r = m.ranges.map(d => Math.min(Number.isFinite(d) ? d : 9, 3.5));
      const front = Math.min(...r.slice(0, 25), ...r.slice(-25)), left = Math.min(...r.slice(25, 90)), right = Math.min(...r.slice(270, 335));
      const sp = n.params.get('speed').v, sd = n.params.get('safe_distance').v, ts = n.params.get('turn_speed').v;
      cmd = front < sd ? { v: 0, w: left > right ? ts : -ts } : { v: sp, w: clamp(0.8 * (left - right), -ts, ts) };
    });
    const iv = setInterval(() => Bus.publish(name, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: cmd.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: cmd.w } }), 100);
    return () => { clearInterval(iv); un(); Bus.removeNode(name); };
  },
  talker(log) {
    const name = '/talker'; if (Bus.nodes.has(name)) { log('[WARN] A node named /talker is already running. Duplicate names cause confusion in ROS 2.', 'w'); }
    Bus.addNode(name, 'demo'); Bus.advertise(name, '/chatter', 'std_msgs/msg/String');
    let k = 1; const iv = setInterval(() => { const d = `Hello World: ${k++}`; Bus.publish(name, '/chatter', 'std_msgs/msg/String', { data: d }); log(`[INFO] [${(Date.now() / 1000).toFixed(3)}] [talker]: Publishing: '${d}'`); }, 1000);
    return () => { clearInterval(iv); Bus.removeNode(name); };
  },
  listener(log) {
    const name = '/listener'; Bus.addNode(name, 'demo');
    const un = Bus.subscribe(name, '/chatter', 'std_msgs/msg/String', (m) => log(`[INFO] [${(Date.now() / 1000).toFixed(3)}] [listener]: I heard: [${m.data}]`));
    return () => { un(); Bus.removeNode(name); };
  },
};

/* ============================== missions ============================== */
const MISSIONS = [
  { id: 'm1', title: 'First drive', text: 'Drive onto the green pad using the joystick, the arrow buttons or your arrow keys.', tab: 'drive', xp: 15 },
  { id: 'm2', title: 'Look around', text: 'In the Terminal, run `ros2 topic list`, then `ros2 topic echo /scan --once`.', tab: 'term', xp: 15 },
  { id: 'm3', title: 'Command-line pilot', text: 'Move the robot 1 m using only `ros2 topic pub` on /cmd_vel. Tip: add `--rate 10`.', tab: 'term', xp: 15 },
  { id: 'm4', title: 'Speed limit', text: 'Use `ros2 param set` to lower /robot_sim `max_linear_speed` to 0.3.', tab: 'term', xp: 15 },
  { id: 'm5', title: 'Beacon hunter', text: 'Collect all three glowing beacons (any way you like).', tab: 'drive', xp: 20 },
  { id: 'm6', title: 'Fresh start', text: 'Call the `/reset_world` service from the Terminal.', tab: 'term', xp: 15 },
  { id: 'm7', title: 'Your first node', text: 'In the Code tab, run a Python node that publishes /cmd_vel and drives the robot 2 m.', tab: 'code', xp: 25 },
  { id: 'm8', title: 'No-crash zone', text: 'Run a Python node that drives 6 m without a single bump. The wall-avoider template is a good start.', tab: 'code', xp: 30 },
  { id: 'm9', title: 'Launch party', text: 'Start `ros2 launch lab_bringup wander.launch.py`, then change `/wander` `speed` with `ros2 param set`.', tab: 'term', xp: 20 },
  { id: 'm10', title: 'TF detective', text: 'Print the robot\'s pose with `ros2 run tf2_ros tf2_echo odom base_link`.', tab: 'term', xp: 15 },
];
const Missions = {
  st: { cmds: new Set(), dist: { cli: 0, code: 0 }, run: { dist: 0, bumps: 0 }, launched: false },
  done(id) { return !!S.lab.missions[id]; },
  current() { return MISSIONS.find(m => !this.done(m.id)); },
  complete(id) {
    if (this.done(id)) return;
    const m = MISSIONS.find(x => x.id === id); S.lab.missions[id] = Date.now();
    addXP(m.xp); S.gears += 5; const nd = bumpStreak(); save(); checkBadges(); save();
    Sound.play('done'); confetti(90);
    toast(`<b>Mission complete: ${esc(m.title)}</b><br>+${m.xp} XP · +5 gears${nd ? ` · ${S.streak}-day streak` : ''}`, { mood: 'happy', ms: 3400 });
    Lab.renderMissionBar(); Lab.refreshTab('missions'); App.renderStats();
  },
  event(type, data) {
    const s = this.st;
    if (type === 'pad') this.complete('m1');
    else if (type === 'cmd') { s.cmds.add(data); if (s.cmds.has('topic list') && s.cmds.has('echo scan once')) this.complete('m2'); if (data === 'tf2_echo') this.complete('m10'); if (data === 'launch wander') s.launched = true; }
    else if (type === 'param' && data.node === '/robot_sim' && data.name === 'max_linear_speed' && Math.abs(data.v - 0.3) < 1e-9) this.complete('m4');
    else if (type === 'beacon' && data >= 3) this.complete('m5');
    else if (type === 'service' && data === '/reset_world') this.complete('m6');
    else if (type === 'wanderParam' && s.launched) this.complete('m9');
    else if (type === 'bump') s.run.bumps++;
    else if (type === 'codeStart') { s.run = { dist: 0, bumps: 0 }; }
  },
  move(kind, d) {
    const s = this.st;
    if (kind === 'cli') { s.dist.cli += d; if (s.dist.cli >= 1) this.complete('m3'); }
    if (kind === 'user') { s.dist.code += d; s.run.dist += d; if (s.dist.code >= 2) this.complete('m7'); if (s.run.dist >= 6 && s.run.bumps === 0) this.complete('m8'); }
  },
};

/* ============================== terminal ============================== */
const PROMPT = 'learner@ros2:~/ros2_ws$';
const ROS2_CMDS = ['node', 'topic', 'service', 'param', 'interface', 'run', 'launch', 'pkg'];
const VERBS = { node: ['list', 'info'], topic: ['list', 'echo', 'hz', 'bw', 'info', 'type', 'pub'], service: ['list', 'type', 'call'], param: ['list', 'get', 'set', 'dump'], interface: ['list', 'show'], pkg: ['list', 'executables'] };
const PKGS = { demo_nodes_cpp: ['talker', 'listener'], demo_nodes_py: ['talker', 'listener'], tf2_ros: ['tf2_echo', 'static_transform_publisher'], teleop_twist_keyboard: ['teleop_twist_keyboard'], lab_bringup: ['wander.launch.py', 'demo.launch.py'], turtlesim: ['turtlesim_node', 'turtle_teleop_key'] };
// a stable fake DDS GID per endpoint, printed by `ros2 topic info -v`
function gidOf(key) { let h1 = 2166136261; for (const c of key) h1 = Math.imul(h1 ^ c.charCodeAt(0), 16777619); const b = []; for (let i = 0; i < 16; i++) { h1 = Math.imul(h1 ^ i, 16777619); b.push(((h1 >>> 24) & 255).toString(16).padStart(2, '0')); } return b.join('.'); }
const stampSec = () => { const st = stampNow(); return (st.sec + st.nanosec / 1e9).toFixed(9); };
function tokenize(line) {
  const out = []; let cur = '', q = null, has = false;
  for (const ch of line) {
    if (q) { if (ch === q) q = null; else cur += ch; continue; }
    if (ch === '"' || ch === "'") { q = ch; has = true; continue; }
    if (/\s/.test(ch)) { if (cur || has) { out.push(cur); cur = ''; has = false; } continue; }
    cur += ch;
  }
  if (q) throw new Error('unexpected EOF while looking for matching quote');
  if (cur || has) out.push(cur);
  return out;
}
/* A terminal talks to one simulated ROS graph. ctx = { bus, missions, ui, prompt, help, pkgs, chips, hist(), saveHist(h), run?, launch? } */
function labCtx() { return { bus: Bus, missions: Missions, ui: Lab, prompt: PROMPT, help: HELP_TEXT, pkgs: PKGS, frames: ['odom', 'base_link', 'base_scan'], hist: () => S.lab.hist, saveHist: (hs) => { S.lab.hist = hs.slice(-50); save({ local: true }); } }; }
class Term {
  constructor(n, ctx) { this.n = n; this.ctx = ctx || labCtx(); this.lines = []; this.hist = (this.ctx.hist() || []).slice(); this.hi = this.hist.length; this.proc = null; this.out = null; this.draft = ''; }
  print(text, cls = 'o') {
    String(text).split('\n').forEach(l => this.lines.push([l, cls]));
    if (this.lines.length > 700) this.lines.splice(0, this.lines.length - 600);
    if (this.out && this.out.isConnected) {
      String(text).split('\n').forEach(l => this.out.append(h('div', { class: cls }, l || ' ')));
      while (this.out.childElementCount > 600) this.out.firstChild.remove();
      this.out.scrollTop = this.out.scrollHeight;
    }
  }
  renderInto(out) { this.out = out; out.innerHTML = ''; const frag = document.createDocumentFragment(); this.lines.forEach(([l, c]) => frag.append(h('div', { class: c }, l || ' '))); out.append(frag); out.scrollTop = out.scrollHeight; }
  stop(silent) { if (this.proc) { const p = this.proc; this.proc = null; try { p.stop(); } catch (e) { /* ignore */ } if (!silent) this.print('^C', 'd'); if (p.after) this.print(p.after, 'd'); } this.ctx.ui.termState(); }
  run(line) {
    this.print(`${this.ctx.prompt} ${line}`, 'p');
    const t = line.trim(); if (!t) return;
    if (this.hist[this.hist.length - 1] !== t) { this.hist.push(t); this.ctx.saveHist(this.hist); }
    this.hi = this.hist.length;
    let argv; try { argv = tokenize(t); } catch (e) { this.print('bash: ' + e.message, 'e'); return; }
    try { CLI.exec(this, argv, t); } catch (e) { this.print(String(e.message || e), 'e'); }
    this.ctx.ui.termState();
  }
}
const CLI = {
  exec(T, a, raw) {
    const { ui: Lab } = T.ctx;
    const p = (s, c) => T.print(s, c);
    const c0 = a[0];
    if (c0 === 'clear') { T.lines = []; if (T.out) T.out.innerHTML = ''; return; }
    if (c0 === 'help') return p(T.ctx.help, 'i');
    if (c0 === 'history') return p(T.hist.map((x, i) => `  ${String(i + 1).padStart(3)}  ${x}`).join('\n'));
    if (c0 === 'pwd') return p('/home/learner/ros2_ws');
    if (c0 === 'whoami') return p('learner');
    if (c0 === 'ls') return p('build  install  log  src');
    if (c0 === 'cd') return;
    if (c0 === 'source') { if (!a[1]) p('bash: source: filename argument required', 'e'); return; }
    if (c0 === 'echo') return p(a.slice(1).join(' ').replace(/\$ROS_DISTRO/g, 'jazzy').replace(/\$USER/g, 'learner'));
    if (c0 === 'rqt_graph' || c0 === 'rqt') { Lab.focusTab('graph'); return p(`Opening the live graph (${T.ctx.graphTab || 'Graph'} tab)…`, 'd'); }
    if (c0 === 'rviz2') return p('rviz2 is not available here. The arena on the left is your live view.', 'w');
    if (c0 === 'colcon') return p('There is no workspace to build in the lab. Use the Code tab to run your own Python node.', 'w');
    if (c0 === 'sudo' || c0 === 'python3' || c0 === 'pip') return p(`${c0}: not available in the lab terminal`, 'w');
    if (c0 !== 'ros2') { p(`bash: ${c0}: command not found`, 'e'); if (/^ros/.test(c0)) p("Did you mean 'ros2'?", 'd'); return; }
    const cmd = a[1];
    if (!cmd || cmd === '-h' || cmd === '--help') return p(ROS2_USAGE, 'o');
    if (!ROS2_CMDS.includes(cmd)) { p(`ros2: error: invalid choice: '${cmd}' (choose from ${ROS2_CMDS.map(x => `'${x}'`).join(', ')})`, 'e'); const near = ROS2_CMDS.find(x => cmd.startsWith(x) || x.startsWith(cmd.slice(0, 3))); if (near) p(`Did you mean 'ros2 ${near}'?`, 'd'); return; }
    const verb = a[2], rest = a.slice(3);
    if (VERBS[cmd] && (!verb || !VERBS[cmd].includes(verb))) { p(`usage: ros2 ${cmd} <command>\nros2 ${cmd}: error: ${verb ? `invalid choice: '${verb}'` : 'a command is required'} (choose from ${VERBS[cmd].map(x => `'${x}'`).join(', ')})`, 'e'); return; }
    return this[cmd](T, verb, rest, raw);
  },
  node(T, verb, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    const nodes = [...Bus.nodes.values()].filter(n => !n.hidden);
    if (verb === 'list') { T.print(nodes.map(n => n.name).sort().join('\n') || ''); Missions.event('cmd', 'node list'); return; }
    const n = Bus.nodes.get(r[0]); if (!r[0]) return T.print('ros2 node info: error: the following arguments are required: node_name', 'e');
    if (!n) return T.print(`Unable to find node '${r[0]}'`, 'e');
    const tp = (set, which) => [...set].sort().map(t => { const T2 = Bus.topics.get(t); return `    ${t}: ${T2 ? (T2[which].get(n.name) || T2.type) : '?'}`; }).join('\n');
    const pubs = new Set([...n.pubs, '/parameter_events', '/rosout']);
    const srv = [...Bus.services].filter(([, s]) => s.node === n.name).map(([k, s]) => `    ${k}: ${s.type}`).sort().join('\n');
    T.print(`${n.name}\n  Subscribers:\n${tp(n.subs, 'subs')}${n.subs.size ? '\n' : ''}  Publishers:\n${tp(pubs, 'pubs')}\n  Service Servers:\n${srv}${srv ? '\n' : ''}  Service Clients:\n\n  Action Servers:\n\n  Action Clients:\n`);
  },
  topic(T, verb, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    const p = (s, c) => T.print(s, c);
    const flags = r.filter(x => x.startsWith('-')), pos = r.filter(x => !x.startsWith('-'));
    if (verb === 'list') { const ts = [...Bus.topics.values()].sort((x, y) => x.name.localeCompare(y.name)); p(ts.map(t => flags.includes('-t') ? `${t.name} [${Bus.types(t.name).join(', ')}]` : t.name).join('\n')); Missions.event('cmd', 'topic list'); return; }
    if (verb === 'pub') return this.pub(T, r);
    const name = pos[0];
    if (!name) return p(`ros2 topic ${verb}: error: the following arguments are required: topic_name`, 'e');
    const Tp = Bus.topics.get(name);
    if (verb === 'type') return Tp ? p(Bus.types(name).join('\n')) : p(`Unknown topic '${name}'`, 'e');
    if (verb === 'info') {
      if (!Tp) return p(`Unknown topic '${name}'`, 'e');
      const tys = Bus.types(name).join(', ');
      let s = `Type: ${tys}\nPublisher count: ${Tp.pubs.size}\nSubscription count: ${Tp.subs.size}`;
      if (flags.includes('-v') || flags.includes('--verbose')) {
        const ep = (n, ty, q, kind) => `Node name: ${n.split('/').pop()}\nNode namespace: ${n.slice(0, n.lastIndexOf('/')) || '/'}\nTopic type: ${ty}\nEndpoint type: ${kind}\nGID: ${gidOf(n + kind + name)}\nQoS profile:\n  Reliability: ${q.rel}\n  History (Depth): KEEP_LAST (${q.depth})\n  Durability: ${q.dur}\n  Lifespan: Infinite\n  Deadline: Infinite\n  Liveliness: AUTOMATIC\n  Liveliness lease duration: Infinite\n`;
        s = `Type: ${tys}\n\nPublisher count: ${Tp.pubs.size}\n\n` + [...Tp.pubs].map(([n, ty]) => ep(n, ty, Tp.pubQos.get(n) || QOS_DEF, 'PUBLISHER')).join('\n') + `\nSubscription count: ${Tp.subs.size}\n\n` + [...Tp.subs].map(([n, ty]) => ep(n, ty, Tp.subQos.get(n) || QOS_DEF, 'SUBSCRIPTION')).join('\n');
        const types = new Set([...Tp.pubs.values(), ...Tp.subs.values()]); if (types.size > 1) s += `\nWARNING: endpoints on ${name} use different types: ${[...types].join(', ')}`;
      }
      return p(s);
    }
    if (verb === 'echo') {
      if (!Tp || !Tp.pubs.size) { p(`WARNING: topic [${name}] does not appear to be published yet\nCould not determine the type for the passed topic`, 'w'); return; }
      const tys = Bus.types(name); if (tys.length > 1) return p(`Cannot echo topic '${name}', as it contains more than one type: [${tys.join(', ')}]`, 'e');
      const once = flags.includes('--once'), type = Tp.pubs.values().next().value || Tp.type;
      // like ros2 topic echo: without --qos-* flags it picks a profile every publisher can serve
      const qa = (k) => { const i = r.indexOf(k); return i >= 0 ? String(r[i + 1] || '').toLowerCase() : ''; }, prof = qa('--qos-profile');
      const pqs = [...Tp.pubQos.values()], relF = qa('--qos-reliability') || (prof === 'sensor_data' ? 'best_effort' : ''), durF = qa('--qos-durability');
      const myQ = { rel: relF ? (relF === 'best_effort' ? 'BEST_EFFORT' : 'RELIABLE') : pqs.every(q => q.rel === 'RELIABLE') ? 'RELIABLE' : 'BEST_EFFORT', dur: durF ? (durF === 'transient_local' ? 'TRANSIENT_LOCAL' : 'VOLATILE') : pqs.length && pqs.every(q => q.dur === 'TRANSIENT_LOCAL') ? 'TRANSIENT_LOCAL' : 'VOLATILE', depth: 10 };
      if (!IFACES[type]) return p(`The message type '${type}' is invalid`, 'e');
      const node = `/_ros2cli_${++Bus.pid}`; Bus.addNode(node, 'cli', { hidden: true, noParams: true });
      let last = 0, un = null;
      const show = (m) => {
        const now = performance.now(); if (!once && now - last < 250) return; last = now;
        p(yamlMsg(type, m).join('\n') + '\n---');
        if (once) { T.proc = null; un(); Bus.removeNode(node); Lab.termState(); if (name === '/scan') Missions.event('cmd', 'echo scan once'); }
      };
      Missions.event('cmd', 'echo:' + name);
      un = Bus.subscribe(node, name, type, show, myQ);
      [...Tp.pubQos].forEach(([pn, pq]) => { const bad = qosClash(pq, myQ); if (bad) p(`[WARN] [${stampSec()}] [${node.slice(1)}]: New publisher discovered on topic '${name}', offering incompatible QoS. No messages will be received from it. Last incompatible policy: ${bad}`, 'w'); void pn; });
      T.proc = { name: 'echo', stop: () => { un(); Bus.removeNode(node); } };
      if (!once && Bus.hz(name) > 4) p('(the lab terminal shows at most 4 messages per second)', 'd');
      return;
    }
    if (verb === 'hz' || verb === 'bw') {
      if (!Tp || !Tp.pubs.size) { p(`WARNING: topic [${name}] does not appear to be published yet`, 'w'); }
      // like the real tools, hz and bw subscribe to the topic (so a camera that only renders for subscribers starts sending)
      const node = `/_ros2cli_${++Bus.pid}`, ty = Tp ? (Tp.pubs.values().next().value || Tp.type) : null, sizes = [];
      Bus.addNode(node, 'cli', { hidden: true, noParams: true });
      const un = ty ? Bus.subscribe(node, name, ty, (m) => { if (verb === 'bw') { sizes.push(msgBytes(m)); if (sizes.length > 100) sizes.shift(); } }, { rel: Tp && [...Tp.pubQos.values()].some(q => q.rel === 'BEST_EFFORT') ? 'BEST_EFFORT' : 'RELIABLE', dur: 'VOLATILE', depth: 10 }) : () => {};
      if (verb === 'bw') p(`Subscribed to [${name}]`);
      Missions.event('cmd', verb + ':' + name);
      let win = 0;
      const mb = (b) => b >= 1e6 ? (b / 1e6).toFixed(2) + ' MB' : b >= 1e3 ? (b / 1e3).toFixed(2) + ' KB' : b.toFixed(0) + ' B';
      const iv = setInterval(() => {
        const hz = Bus.hz(name); if (!(hz > 0)) return;
        if (verb === 'hz') { win += Math.round(hz); p(`average rate: ${hz.toFixed(3)}\n\tmin: ${(1 / hz * 0.98).toFixed(3)}s max: ${(1 / hz * 1.02).toFixed(3)}s std dev: ${(0.0004 + Math.random() * 0.0006).toFixed(5)}s window: ${win}`); return; }
        if (!sizes.length) return; const avg = sizes.reduce((a, x) => a + x, 0) / sizes.length;
        p(`${mb(avg * hz)}/s from ${sizes.length} messages\n\tMessage size mean: ${mb(avg)} min: ${mb(Math.min(...sizes))} max: ${mb(Math.max(...sizes))}`);
      }, 1000);
      T.proc = { name: verb, stop: () => { clearInterval(iv); un(); Bus.removeNode(node); } };
    }
  },
  pub(T, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    const p = (s, c) => T.print(s, c);
    let rate = 1, once = false, times = 0; const pos = [], q = {};
    for (let i = 0; i < r.length; i++) {
      const x = r[i];
      if (x === '-1' || x === '--once') once = true;
      else if (x === '-r' || x === '--rate') { rate = parseFloat(r[++i]); if (!(rate > 0)) return p('ros2 topic pub: error: argument -r/--rate: expected a positive number', 'e'); }
      else if (x === '-t' || x === '--times') times = parseInt(r[++i], 10) || 0;
      else if (x === '--qos-reliability') { const v = String(r[++i] || '').toLowerCase(); q.rel = v === 'best_effort' ? 'BEST_EFFORT' : 'RELIABLE'; }
      else if (x === '--qos-durability') { const v = String(r[++i] || '').toLowerCase(); q.dur = v === 'transient_local' ? 'TRANSIENT_LOCAL' : 'VOLATILE'; }
      else if (x === '--qos-profile') { if (String(r[++i] || '').toLowerCase() === 'sensor_data') q.rel = 'BEST_EFFORT'; }
      else if (x === '-w' || x === '-p' || x === '--qos-depth' || x === '-n' || x === '--node-name') i++;
      else if (x.startsWith('-') && !/^-?\d/.test(x) && !x.startsWith('{')) return p(`ros2 topic pub: error: unrecognized arguments: ${x}`, 'e');
      else pos.push(x);
    }
    const [name, type, vals] = pos;
    if (!name || !type) return p('usage: ros2 topic pub [-h] [-r N] [-1] [-t TIMES] topic_name message_type [values]\nros2 topic pub: error: the following arguments are required: topic_name, message_type', 'e');
    if (!name.startsWith('/') && !/^[A-Za-z]/.test(name)) return p(`Invalid topic name '${name}'`, 'e');
    const topic = name.startsWith('/') ? name : '/' + name;
    if (!isMsg(type)) return p(`The passed message type '${type}' is invalid${IFACES[type] ? ' (that is a service type)' : ''}`, 'e');
    let msg; try { msg = fillMsg(type, vals ? parseYaml(vals) : {}); } catch (e) { return p(e.message, 'e'); }
    const Tp = Bus.topics.get(topic);
    if (Tp && Tp.type && Tp.type !== type) p(`WARNING: ${topic} already uses ${Tp.type}. Subscribers expecting that type will ignore ${type}.`, 'w');
    const node = `/_ros2cli_${++Bus.pid}`; Bus.addNode(node, 'cli', { noParams: false }); Bus.advertise(node, topic, type, q);
    if (Tp) [...Tp.subQos].forEach(([sn, sq]) => { const bad = qosClash(qosOf(q), sq); if (bad) p(`[WARN] [${stampSec()}] [${node.slice(1)}]: New subscription discovered on topic '${topic}', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: ${bad}`, 'w'); void sn; });
    let k = 0;
    p('publisher: beginning loop');
    const fire = () => { k++; Bus.publish(node, topic, type, JSON.parse(JSON.stringify(msg))); p(`publishing #${k}: ${pyRepr(type, msg)}\n`); if ((once && k >= 1) || (times && k >= times)) finish(); };
    let iv = null;
    const finish = () => { clearInterval(iv); setTimeout(() => Bus.removeNode(node), 300); T.proc = null; Lab.termState(); };
    T.proc = { name: 'pub', stop: () => { clearInterval(iv); Bus.removeNode(node); } };
    setTimeout(() => { if (T.proc && T.proc.name === 'pub') { fire(); if (T.proc) iv = setInterval(fire, 1000 / rate); } }, 150);
  },
  service(T, verb, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    const p = (s, c) => T.print(s, c);
    if (verb === 'list') { const ss = [...Bus.services.keys()].sort(); p(ss.map(s => r.includes('-t') ? `${s} [${Bus.services.get(s).type}]` : s).join('\n')); return; }
    const name = r.find(x => !x.startsWith('-'));
    if (!name) return p(`ros2 service ${verb}: error: the following arguments are required: service_name`, 'e');
    const sv = Bus.services.get(name);
    if (verb === 'type') return sv ? p(sv.type) : p(`Unknown service '${name}'`, 'e');
    const pos = r.filter(x => !x.startsWith('-') || x.startsWith('{'));
    const type = pos[1], vals = pos[2];
    if (!type) return p('ros2 service call: error: the following arguments are required: service_type', 'e');
    const it = IFACES[type];
    if (!it || !it.srv) return p(`The passed service type is invalid: '${type}'`, 'e');
    if (!sv) { p('waiting for service to become available...', 'd'); T.proc = { name: 'call', stop: () => {} }; return; }
    if (sv.type !== type) return p(`The passed service type '${type}' does not match ${name}'s type '${sv.type}'`, 'e');
    let req; try { req = fillMsg(type, vals ? parseYaml(vals) : {}, it.srv[0]); } catch (e) { return p(e.message, 'e'); }
    const short = type.split('/').pop();
    p(`requester: making request: ${pyRepr(type, req, it.srv[0], pyType(type) + '_Request')}\n`);
    const res = fillMsg(type, sv.handler(req) || {}, it.srv[1]);
    p(`response:\n${pyRepr(type, res, it.srv[1], pyType(type) + '_Response')}\n`);
    void short;
    Missions.event('service', name);
  },
  param(T, verb, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    const p = (s, c) => T.print(s, c);
    const fmt = (e) => e.t === 'double' ? `Double value is: ${pyFloat(e.v)}` : e.t === 'bool' ? `Boolean value is: ${e.v ? 'True' : 'False'}` : e.t === 'integer' ? `Integer value is: ${e.v}` : `String value is: ${e.v}`;
    if (verb === 'list') {
      const nodes = [...Bus.nodes.values()].filter(n => !n.hidden && n.params.size && (!r[0] || n.name === r[0]));
      if (r[0] && !nodes.length) return p(`Node not found`, 'e');
      p(nodes.sort((a, b) => a.name.localeCompare(b.name)).map(n => `${n.name}:\n` + [...n.params.keys()].sort().map(k => `  ${k}`).join('\n')).join('\n'));
      return;
    }
    const n = Bus.nodes.get(r[0]);
    if (!r[0]) return p(`ros2 param ${verb}: error: the following arguments are required: node_name`, 'e');
    if (!n) return p('Node not found', 'e');
    if (verb === 'dump') { p(`${n.name}:\n  ros__parameters:\n` + [...n.params].sort().map(([k, e]) => `    ${k}: ${e.t === 'double' ? pyFloat(e.v) : e.t === 'string' ? yScalar('string', e.v) : String(e.v)}`).join('\n')); return; }
    const pn = r[1]; if (!pn) return p(`ros2 param ${verb}: error: the following arguments are required: parameter_name`, 'e');
    const e = n.params.get(pn);
    if (verb === 'get') return e ? p(fmt(e)) : p('Parameter not set.', 'e');
    if (r[2] == null) return p('ros2 param set: error: the following arguments are required: value', 'e');
    const raw = r.slice(2).join(' ');
    let v; try { v = parseYaml('[' + raw + ']')[0]; } catch (err) { v = raw; }
    const vt = typeof v === 'boolean' ? 'bool' : typeof v === 'number' ? (/^[-+]?\d+$/.test(raw.trim()) ? 'integer' : 'double') : 'string';
    if (!e) return p(`Setting parameter failed: parameter '${pn}' is not declared on ${n.name}`, 'e');
    if (e.ro) return p(`Setting parameter failed: Trying to set a read-only parameter: ${pn}.`, 'e');
    if (e.t !== vt) return p(`Setting parameter failed: Wrong parameter type, expected 'Type.${e.t.toUpperCase()}' got 'Type.${vt.toUpperCase()}'${e.t === 'double' && vt === 'integer' ? `\n(tip: write ${raw.trim()}.0 for a double)` : ''}`, 'e');
    if (e.range && (v < e.range[0] || v > e.range[1])) { const f = e.t === 'integer' ? String : pyFloat; return p(`Setting parameter failed: ${pn} must be between ${f(e.range[0])} and ${f(e.range[1])}`, 'e'); }
    if (n.user) {
      T.proc = { name: 'param', stop: () => {} };
      Py.setParam(n.name, pn, v, e.t).then(res => { if (T.proc && T.proc.name === 'param') T.proc = null; if (res.ok) { p('Set parameter successful'); } else p(`Setting parameter failed${res.reason ? ': ' + res.reason : ''}`, 'e'); Lab.termState(); });
      return;
    }
    e.v = v;
    const res = n.onParam ? n.onParam(pn, v) : { ok: true };
    p(res && res.ok === false ? `Setting parameter failed: ${res.reason}` : 'Set parameter successful', res && res.ok === false ? 'e' : 'o');
  },
  interface(T, verb, r) {
    if (verb === 'list') { const ks = Object.keys(IFACES).filter(k => !k.startsWith('builtin')); T.print('Messages:\n' + ks.filter(k => IFACES[k].f).map(k => '    ' + k).join('\n') + '\nServices:\n' + ks.filter(k => IFACES[k].srv).map(k => '    ' + k).join('\n')); return; }
    if (!r[0]) return T.print('ros2 interface show: error: the following arguments are required: type', 'e');
    const s = showIface(r[0]); if (!s) return T.print(`Could not find the interface '${r[0]}'`, 'e');
    T.print(s);
  },
  pkg(T, verb, r) {
    const { bus: Bus, missions: Missions, ui: Lab, pkgs: PKGS } = T.ctx; void Lab; void PKGS; void Missions;
    if (verb === 'list') return T.print(Object.keys(PKGS).concat(['geometry_msgs', 'nav_msgs', 'rclcpp', 'rclpy', 'sensor_msgs', 'std_msgs', 'std_srvs', 'tf2_msgs']).sort().join('\n'));
    const k = r[0]; if (!k) return T.print(Object.entries(PKGS).flatMap(([p, xs]) => xs.filter(x => !x.endsWith('.py')).map(x => `${p} ${x}`)).join('\n'));
    if (!PKGS[k]) return T.print(`Package '${k}' not found`, 'e');
    T.print(PKGS[k].filter(x => !x.endsWith('.py')).map(x => `${k} ${x}`).join('\n'));
  },
  run(T, pkg, r) {
    if (T.ctx.run) return T.ctx.run(T, pkg, r);
    const p = (s, c) => T.print(s, c);
    if (!pkg) return p('usage: ros2 run <package> <executable>', 'e');
    if (pkg === 'turtlesim') return p('turtlesim lives in the lessons. This lab simulates a lidar robot instead: try `ros2 node list`.', 'w');
    if (!PKGS[pkg]) return p(`Package '${pkg}' not found`, 'e');
    const exe = r[0];
    if (!exe || !PKGS[pkg].includes(exe)) return p(exe ? `No executable found` : 'usage: ros2 run <package> <executable>', 'e');
    const log = (s, c) => p(s, c);
    if ((pkg === 'demo_nodes_cpp' || pkg === 'demo_nodes_py') && exe === 'talker') { const stop = Builtin.talker(log); T.proc = { name: 'talker', stop }; return; }
    if ((pkg === 'demo_nodes_cpp' || pkg === 'demo_nodes_py') && exe === 'listener') { const stop = Builtin.listener(log); T.proc = { name: 'listener', stop }; return; }
    if (exe === 'static_transform_publisher') return p('[INFO] [static_transform_publisher]: Spinning until stopped (lab: static frames are odom → base_link → base_scan).', 'i');
    if (exe === 'tf2_echo') {
      const [src, dst] = r.slice(1);
      if (!src || !dst) return p('Usage: tf2_echo source_frame target_frame [echo_rate]', 'e');
      const node = '/tf2_echo'; Bus.addNode(node, 'cli');
      const un = Bus.subscribe(node, '/tf', 'tf2_msgs/msg/TFMessage', () => {});
      Missions.event('cmd', 'tf2_echo');
      const frames = ['odom', 'base_link', 'base_scan'];
      const iv = setInterval(() => {
        if (!frames.includes(src) || !frames.includes(dst)) { p(`[INFO] [${(Date.now() / 1000).toFixed(3)}] [tf2_echo]: Waiting for transform ${src} ->  ${dst}: Invalid frame ID "${frames.includes(src) ? dst : src}" passed to canTransform argument - frame does not exist`, 'w'); return; }
        const pose = (f) => f === 'odom' ? [0, 0, 0, 0] : f === 'base_link' ? [Sim.x, Sim.y, 0, Sim.th] : [Sim.x, Sim.y, 0.12, Sim.th];
        const A = pose(src), B = pose(dst), c = Math.cos(A[3]), s = Math.sin(A[3]);
        const dx = B[0] - A[0], dy = B[1] - A[1], x = c * dx + s * dy, y = -s * dx + c * dy, z = B[2] - A[2], yaw = wrapAngle(B[3] - A[3]);
        const f3 = (v) => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(3);
        p(`At time ${(Date.now() / 1000).toFixed(9)}\n- Translation: [${f3(x)}, ${f3(y)}, ${f3(z)}]\n- Rotation: in Quaternion (xyzw) [0.000, 0.000, ${f3(Math.sin(yaw / 2))}, ${f3(Math.cos(yaw / 2))}]\n- Rotation: in RPY (radian) [0.000, -0.000, ${f3(yaw)}]\n- Rotation: in RPY (degree) [0.000, -0.000, ${f3(yaw * 180 / Math.PI)}]`);
      }, 1000);
      T.proc = { name: 'tf2_echo', stop: () => { clearInterval(iv); un(); Bus.removeNode(node); } };
      return;
    }
    if (exe === 'teleop_twist_keyboard') {
      p(`This node takes keypresses from the keyboard and publishes them\nas Twist messages. (lab: click the input line first)\n---------------------------\nMoving around:\n   u    i    o\n   j    k    l\n   m    ,    .\n\nanything else : stop\nq/z : increase/decrease max speeds by 10%\nCTRL-C to quit\n\ncurrently:\tspeed 0.3\tturn 1.0`, 'o');
      const node = '/teleop_twist_keyboard'; Bus.addNode(node, 'teleop'); Bus.advertise(node, '/cmd_vel', 'geometry_msgs/msg/Twist');
      const st = { sp: 0.3, tn: 1.0, v: 0, w: 0 };
      const iv = setInterval(() => { if (st.v || st.w) Bus.publish(node, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: st.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: st.w } }); }, 100);
      const B = { i: [1, 0], o: [1, -1], j: [0, 1], l: [0, -1], u: [1, 1], ',': [-1, 0], '.': [-1, 1], m: [-1, -1] };
      T.proc = {
        name: 'teleop', stop: () => { clearInterval(iv); Bus.publish(node, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }); Bus.removeNode(node); },
        key: (k, up) => {
          if (up) { if (B[k]) { st.v = 0; st.w = 0; Bus.publish(node, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }); } return; }
          if (k === 'q' || k === 'z') { const f = k === 'q' ? 1.1 : 0.9; st.sp *= f; st.tn *= f; p(`currently:\tspeed ${st.sp.toFixed(2)}\tturn ${st.tn.toFixed(2)}`); return; }
          if (B[k]) { st.v = B[k][0] * st.sp; st.w = B[k][1] * st.tn; } else { st.v = 0; st.w = 0; }
          Bus.publish(node, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: st.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: st.w } });
        },
      };
    }
  },
  launch(T, pkg, r) {
    if (T.ctx.launch) return T.ctx.launch(T, pkg, r);
    const p = (s, c) => T.print(s, c);
    if (!pkg || !r[0]) return p('usage: ros2 launch <package> <launch_file> [args...]', 'e');
    if (!PKGS[pkg]) return p(`Package '${pkg}' not found: "package '${pkg}' not found, searching: ['/opt/ros/jazzy']"`, 'e');
    const file = r[0], args = Object.fromEntries(r.slice(1).filter(x => x.includes(':=')).map(x => x.split(':=')));
    if (!PKGS[pkg].includes(file)) return p(`file '${file}' was not found in the share directory of package '${pkg}'`, 'e');
    const pid = ++Bus.pid;
    p(`[INFO] [launch]: All log files can be found below /home/learner/.ros/log/2026-11-04-lab\n[INFO] [launch]: Default logging verbosity is set to INFO`, 'd');
    if (file === 'wander.launch.py') {
      const sp = args.speed != null ? parseFloat(args.speed) : 0.3;
      if (!(sp >= 0 && sp <= 1)) return p(`[ERROR] [launch]: speed must be between 0.0 and 1.0`, 'e');
      const stop = Builtin.wander({ speed: sp }); if (!stop) return p('[ERROR] [launch]: /wander is already running', 'e');
      Missions.event('cmd', 'launch wander');
      p(`[INFO] [wander-1]: process started with pid [${pid}]\n[wander-1] [INFO] [${(Date.now() / 1000).toFixed(3)}] [wander]: Wandering at ${sp.toFixed(2)} m/s. Try: ros2 param set /wander speed 0.5`, 'o');
      T.proc = { name: 'launch', stop, after: `[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n[INFO] [wander-1]: process has finished cleanly [pid ${pid}]` };
      return;
    }
    const s1 = Builtin.talker((s) => p('[talker-1] ' + s)), s2 = Builtin.listener((s) => p('[listener-2] ' + s));
    p(`[INFO] [talker-1]: process started with pid [${pid}]\n[INFO] [listener-2]: process started with pid [${pid + 1}]`);
    T.proc = { name: 'launch', stop: () => { s1(); s2(); }, after: '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n[INFO] [talker-1]: process has finished cleanly\n[INFO] [listener-2]: process has finished cleanly' };
  },
};
const ROS2_USAGE = `usage: ros2 [-h] Call \`ros2 <command> -h\` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

Commands available in the lab:
  interface  Show information about ROS interfaces
  launch     Run a launch file
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  service    Various service related sub-commands
  topic      Various topic related sub-commands

  Call \`ros2 <command> -h\` for more detailed usage.`;
const HELP_TEXT = `Lab terminal: a simulated ROS 2 Jazzy shell connected to the robot.
  ros2 node list | info /robot_sim
  ros2 topic list [-t] | echo /odom [--once] | hz /scan | info /cmd_vel [-v]
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
  ros2 interface show sensor_msgs/msg/LaserScan
  ros2 service list | call /reset_world std_srvs/srv/Empty
  ros2 service call /set_lidar std_srvs/srv/SetBool "{data: false}"
  ros2 param list | get /robot_sim max_linear_speed | set /robot_sim max_linear_speed 0.3 | dump /robot_sim
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ros2 run demo_nodes_cpp talker        (open a second terminal tab for a listener)
  ros2 launch lab_bringup wander.launch.py [speed:=0.4]
Keys: Up/Down history · Tab completes · Ctrl+C stops · Ctrl+L clears`;

/* ============================== terminal panel (shared by the Robot Lab and SnoBots) ============================== */
class TermPanel {
  constructor(ctxFn, opts) { this.ctxFn = ctxFn; this.opts = opts; this.terms = [new Term(1, ctxFn())]; this.ti = 0; this.terms[0].print(opts.welcome, 'i'); this.termIn = null; this.termStop = null; this.box = null; }
  term() { return this.terms[this.ti] || this.terms[0]; }
  mount(body) {
    const wrap = h('div', { class: 'lterm' }); this.box = wrap;
    const tabs = h('div', { class: 'lterm-tabs', role: 'tablist', 'aria-label': 'Terminals' });
    this.terms.forEach((T, i) => { const b = h('button', { class: 'lterm-tab', role: 'tab', 'aria-selected': String(i === this.ti), html: `${T.proc ? '<span class="run"></span>' : ''}Terminal ${T.n}` }); b.addEventListener('click', () => { this.ti = i; this.opts.rerender(); }); tabs.append(b); });
    if (this.terms.length < 3) { const add = h('button', { class: 'lterm-tab', 'aria-label': 'New terminal', html: icon('plus') }); add.addEventListener('click', () => { const n = Math.max(...this.terms.map(t => t.n)) + 1; const T = new Term(n, this.ctxFn()); T.print('New terminal. ROS 2 Jazzy is sourced.', 'd'); this.terms.push(T); this.ti = this.terms.length - 1; this.opts.rerender(); }); tabs.append(add); }
    const T = this.term(), prompt = T.ctx.prompt;
    const out = h('div', { class: 'lterm-out', role: 'log', 'aria-live': 'polite', 'aria-label': 'Terminal output' });
    const inl = h('div', { class: 'lterm-in' });
    const inp = h('input', { class: 'lterm-input', type: 'text', autocomplete: 'off', autocapitalize: 'off', spellcheck: 'false', 'aria-label': 'Command input' });
    const stop = h('button', { class: 'btn btn-sm btn-bad', html: icon('stop') + 'Ctrl+C' });
    inl.append(h('span', { class: 'ps1' }, prompt), inp, stop);
    const chips = h('div', { class: 'tchips', 'aria-label': 'Suggested commands' });
    (this.opts.chips || []).forEach(cmd => { const c = h('button', {}, cmd.length > 46 ? cmd.slice(0, 44) + '…' : cmd); c.title = cmd; c.addEventListener('click', () => { inp.value = cmd; inp.focus(); }); chips.append(c); });
    wrap.append(tabs, out, inl, chips); body.append(wrap);
    T.renderInto(out); T.draft = T.draft || '';
    inp.value = T.draft;
    stop.addEventListener('click', () => { T.stop(); inp.focus(); });
    inp.addEventListener('input', () => { T.draft = inp.value; });
    inp.addEventListener('keydown', (e) => {
      if (e.ctrlKey && (e.key === 'c' || e.key === 'C')) { e.preventDefault(); if (T.proc) T.stop(); else { T.print(`${prompt} ${inp.value}^C`, 'p'); inp.value = ''; T.draft = ''; } return; }
      if (e.ctrlKey && (e.key === 'l' || e.key === 'L')) { e.preventDefault(); T.lines = []; out.innerHTML = ''; return; }
      if (T.proc && T.proc.key) return;
      if (e.key === 'Enter') { e.preventDefault(); if (T.proc) { T.print('(a command is still running: press Ctrl+C first, or open another terminal tab)', 'd'); return; } const line = inp.value; inp.value = ''; T.draft = ''; T.run(line); return; }
      if (e.key === 'ArrowUp') { e.preventDefault(); if (T.hi > 0) { T.hi--; inp.value = T.hist[T.hi] || ''; } return; }
      if (e.key === 'ArrowDown') { e.preventDefault(); if (T.hi < T.hist.length) { T.hi++; inp.value = T.hist[T.hi] || ''; } return; }
      if (e.key === 'Tab') { e.preventDefault(); this.complete(inp, T); }
    });
    this.termIn = inp; this.termStop = stop; this.state();
    setTimeout(() => inp.focus({ preventScroll: true }), 30);
  }
  state() {
    const T = this.term(); if (!this.termIn || !this.termIn.isConnected) return;
    this.termStop.disabled = !T.proc;
    this.termIn.placeholder = T.proc ? (T.proc.key ? 'teleop running: press i j k l , (Ctrl+C to quit)' : 'running… press Ctrl+C to stop') : 'type a command, e.g. ros2 topic list';
    $$('.lterm-tab', this.box).forEach((b, i) => { const t = this.terms[i]; if (!t) return; const run = $('.run', b); if (t.proc && !run) b.prepend(h('span', { class: 'run' })); if (!t.proc && run) run.remove(); });
  }
  complete(inp, T) {
    const B = T.ctx.bus, PK = T.ctx.pkgs, prompt = T.ctx.prompt;
    const v = inp.value, parts = v.split(/\s+/), cur = parts[parts.length - 1], idx = parts.length - 1;
    let opts = [];
    if (idx === 0) opts = ['ros2', 'clear', 'help', 'history', 'rqt_graph'];
    else if (parts[0] === 'ros2' && idx === 1) opts = ROS2_CMDS;
    else if (parts[0] === 'ros2' && idx === 2) opts = VERBS[parts[1]] || (parts[1] === 'run' || parts[1] === 'launch' ? Object.keys(PK) : []);
    else if (parts[0] === 'ros2' && idx >= 3) {
      const c = parts[1], vb = parts[2];
      if (c === 'topic') opts = [...B.topics.keys()].concat(vb === 'pub' && idx >= 4 ? Object.keys(IFACES).filter(isMsg) : []);
      else if (c === 'node' || c === 'param') opts = idx === 3 ? [...B.nodes.values()].filter(n => !n.hidden).map(n => n.name) : (B.nodes.get(parts[3]) ? [...B.nodes.get(parts[3]).params.keys()] : []);
      else if (c === 'service') opts = idx === 3 ? [...B.services.keys()] : Object.keys(IFACES).filter(k => IFACES[k].srv);
      else if (c === 'interface') opts = Object.keys(IFACES);
      else if (c === 'run' || c === 'launch') opts = idx === 3 ? (PK[vb] || []).filter(x => c === 'launch' ? x.endsWith('.py') : !x.endsWith('.py')) : (T.ctx.frames || []);
    }
    const m = opts.filter(o => o.startsWith(cur));
    if (m.length === 1) { parts[idx] = m[0]; inp.value = parts.join(' ') + ' '; }
    else if (m.length > 1) { let pre = m[0]; m.forEach(x => { while (!x.startsWith(pre)) pre = pre.slice(0, -1); }); if (pre.length > cur.length) { parts[idx] = pre; inp.value = parts.join(' '); } else { T.print(`${prompt} ${v}`, 'p'); T.print(m.join('   '), 'd'); } }
    T.draft = inp.value;
  }
}

/* ============================== Python runner ============================== */
const Py = {
  worker: null, host: null, phase: 'off', disp: '', text: 'Python not started', pending: null, lastBeat: 0, reqs: new Map(), rid: 0, unsubs: new Map(), nodes: new Set(), watch: 0, loadT: 0, _warned: false,
  /* The runner serves one simulator at a time: the Robot Lab (host null) or a SnoBots robot. */
  H() { return this.host || { bus: Bus, ui: Lab, missions: Missions, saveCode: (c) => { S.lab.code = c; save({ local: true }); } }; },
  bind(host) { host = host || null; if (this.host === host) return; if (this.phase === 'running' || this.phase === 'hang') this.stop(); this.cleanup(); this.pending = null; this.host = host; if (this.phase === 'ready' || this.phase === 'off') this.show('', this.phase === 'ready' ? 'Python ready' : 'Python not started'); },
  base() { return window.LAB_PYODIDE_BASE || 'https://cdn.jsdelivr.net/pyodide/v0.27.7/full/'; },
  show(disp, text) { this.disp = disp; this.text = text; this.H().ui.pyStatus(); },
  ensure() {
    if (this.worker) return true;
    try { const url = URL.createObjectURL(new Blob([$('#py-worker').textContent], { type: 'text/javascript' })); this.worker = new Worker(url); }
    catch (e) { this.show('err', 'Python blocked here'); this.H().ui.pyLog('This viewer does not allow background workers, so Python code cannot run here. Everything else in the lab still works.', 'err'); return false; }
    this.worker.onmessage = (ev) => this.onMsg(ev.data);
    this.worker.onerror = (e) => { if (e && e.preventDefault) e.preventDefault(); this.fail('Python could not start' + (e && e.message ? ': ' + e.message : '') + '.'); };
    this.phase = 'loading'; this.loadT = performance.now(); this.show('load', 'Loading Python (about 10 MB, first run only)…');
    this.worker.postMessage({ type: 'init', base: this.base(), shim: $('#py-shim').textContent });
    this.lastBeat = performance.now(); clearInterval(this.watch); this.watch = setInterval(() => this.watchdog(), 1000);
    return true;
  },
  fail(msg) { this.kill(); this.show('err', 'Python unavailable'); this.H().ui.pyLog(msg + ' The rest of the lab still works.', 'err'); },
  run(code) {
    this.H().saveCode(code); this.H().ui.clearErr(); this._warned = false;
    if (this.phase === 'hang') this.kill();
    if (this.phase === 'off' && !this.ensure()) return;
    if (this.phase === 'loading') { this.pending = code; this.H().ui.pyLog('Python is still loading. Your node will start as soon as it is ready.', 'sys'); return; }
    this.cleanup();
    this.H().ui.pyLog('▶ python3 my_node.py', 'sys');
    const H = this.H(); this.worker.postMessage({ type: 'run', code, simT: H.simTime ? H.simTime() : null });
    this.phase = 'running'; this.lastBeat = performance.now(); this.show('run', 'Starting…');
    this.H().missions.event('codeStart');
  },
  /* a simulator that drives the clock sends its time after each batch of physics steps (ROS use_sim_time) */
  clock(host, t) { if (this.host === host && this.worker && this.phase === 'running') this.worker.postMessage({ type: 'clock', t }); },
  stop() {
    if (this.phase === 'hang') { this.kill(); this.H().ui.pyLog('■ Python was restarted because it was stuck. Press Run to start again.', 'warn'); return; }
    if (!this.worker || this.phase !== 'running') return;
    this.worker.postMessage({ type: 'stop' }); this.cleanup(); this.phase = 'ready'; this.show('', 'Stopped'); this.H().ui.pyLog('■ stopped (Ctrl+C)', 'sys');
  },
  kill() { if (this.worker) { try { this.worker.terminate(); } catch (e) { /* ignore */ } } this.worker = null; clearInterval(this.watch); this.cleanup(); this.phase = 'off'; this.pending = null; this.show('', 'Python not started'); },
  cleanup() { this.unsubs.forEach(u => u()); this.unsubs.clear(); this.nodes.forEach(n => this.H().bus.removeNode(n)); this.nodes.clear(); const ui = this.H().ui; if (ui.imshow) ui.imshow({ type: 'imclose', name: '' }); },
  watchdog() {
    const now = performance.now();
    if (this.phase === 'running' && now - this.lastBeat > 4000 && !(this.busyUntil > now)) { this.phase = 'hang'; this.show('err', 'Not responding'); this.H().ui.pyLog('Your code has not responded for 4 s. Is there an endless loop (while True) or a time.sleep()? In ROS, use timers and callbacks instead. Press Stop to restart Python.', 'err'); }
    if (this.phase === 'loading' && now - this.loadT > 45000 && !this._slow) { this._slow = true; this.H().ui.pyLog('Python is taking a while to download. A slow connection can take a minute.', 'warn'); }
  },
  setParam(node, name, value, ptype) {
    return new Promise((res) => {
      if (!this.worker || this.phase !== 'running') return res({ ok: false, reason: 'node is not running' });
      const id = ++this.rid; this.reqs.set(id, res);
      this.worker.postMessage({ type: 'param', id, node, name, json: JSON.stringify(value), ptype });
      setTimeout(() => { if (this.reqs.has(id)) { this.reqs.delete(id); res({ ok: false, reason: 'node did not answer' }); } }, 3000);
    });
  },
  onMsg(m) {
    if (m.type === 'beat') { this.lastBeat = performance.now(); if (this.phase === 'hang') { this.phase = 'running'; this.show('run', 'Running'); } return; }
    if (m.type === 'status') return this.show('load', m.text);
    // loading packages (numpy, OpenCV) can keep the worker busy for a while: that is not a hang
    if (m.type === 'busy') { this.busyUntil = performance.now() + 120000; this.show('load', m.text); if (m.text !== this._busyT) { this._busyT = m.text; this.H().ui.pyLog(m.text, 'sys'); } return; }
    if (m.type === 'idle') { this.busyUntil = 0; this._busyT = ''; this.lastBeat = performance.now(); if (this.phase === 'running' && this.disp === 'load') this.show('run', 'Starting…'); return; }
    if (m.type === 'ready') { this.phase = 'ready'; this.show('', `Python ${m.version} ready`); this.H().ui.pyLog(`Python ${m.version} ready · rclpy (lab edition) loaded`, 'sys'); if (this.pending) { const c = this.pending; this.pending = null; this.run(c); } return; }
    if (m.type === 'fatal') { if (this.phase === 'loading') this.fail(`Python could not load in this viewer (${m.text}).`); else this.H().ui.pyLog(m.text, 'err'); return; }
    if (m.type === 'paramResult') { const r = this.reqs.get(m.id); if (r) { this.reqs.delete(m.id); try { r(JSON.parse(m.json)); } catch (e) { r({ ok: false }); } } return; }
    if (m.type === 'out') return this.H().ui.pyLog(m.text, m.err ? 'err' : 'out');
    if (m.type !== 'bridge') return;
    let d; try { d = JSON.parse(m.json); } catch (e) { return; }
    if (m.bin && d.type === 'pub' && d.data) d.data.data = m.bin;
    if (d.type === 'imshow' || d.type === 'imclose') { const ui = this.H().ui; if (ui.imshow) ui.imshow(d, m.bin); return; }
    const nm = (x) => x.startsWith('/') ? x : '/' + x;
    switch (d.type) {
      case 'node': { const n = this.H().bus.addNode(nm(d.name), 'user', { user: true }); n.user = true; this.nodes.add(n.name); break; }
      case 'pub_create': this.H().bus.advertise(nm(d.node), d.topic, d.msgType); break;
      case 'sub_create': {
        const key = nm(d.node) + ' ' + d.topic;
        if (!IFACES[d.msgType]) this.H().ui.pyLog(`[WARN] nothing in the lab publishes ${d.msgType}`, 'warn');
        const T0 = this.H().bus.topics.get(d.topic);
        if (T0 && T0.type && T0.type !== d.msgType) this.H().ui.pyLog(`[WARN] ${d.topic} carries ${T0.type}, but your subscription expects ${d.msgType}. Types must match, so nothing will arrive.`, 'warn');
        const un = this.H().bus.subscribe(nm(d.node), d.topic, d.msgType, (msg) => {
          if (!this.worker || this.phase !== 'running') return;
          // an image's bytes travel as a typed array next to the JSON (a copy, since other subscribers share the message)
          if (msg && ArrayBuffer.isView(msg.data)) { const bin = new Uint8Array(msg.data.buffer.slice(msg.data.byteOffset, msg.data.byteOffset + msg.data.byteLength)); this.worker.postMessage({ type: 'msg', topic: d.topic, msgType: d.msgType, json: JSON.stringify(Object.assign({}, msg, { data: null })), bin }, [bin.buffer]); return; }
          this.worker.postMessage({ type: 'msg', topic: d.topic, msgType: d.msgType, json: JSON.stringify(msg, (k, v) => (typeof v === 'number' && !Number.isFinite(v)) ? null : v) });
        });
        this.unsubs.set(key, un); break;
      }
      case 'pub': {
        const T = this.H().bus.topics.get(d.topic);
        if (d.msgType === 'geometry_msgs/msg/Twist') { const f = (o) => ({ x: +(o && o.x) || 0, y: +(o && o.y) || 0, z: +(o && o.z) || 0 }); d.data = { linear: f(d.data.linear), angular: f(d.data.angular) }; }
        if (T && T.type && T.type !== d.msgType && !this._warned) { this._warned = true; this.H().ui.pyLog(`[WARN] ${d.topic} carries ${T.type}, but your node publishes ${d.msgType}. The robot ignores it.`, 'warn'); }
        this.H().bus.publish(nm(d.node), d.topic, d.msgType, d.data); break;
      }
      case 'log': this.H().ui.pyLog(`[${d.level}] [${(d.t || Date.now() / 1000).toFixed(3)}] [${d.node}]: ${d.text}`, d.level === 'INFO' ? 'info' : d.level === 'WARN' ? 'warn' : 'err'); break;
      case 'param_declare': case 'param_changed': { const n = this.H().bus.nodes.get(nm(d.node)); if (n) n.params.set(d.name, { v: d.value, t: d.ptype || 'other' }); break; }
      case 'node_gone': this.nodes.delete(nm(d.name)); this.H().bus.removeNode(nm(d.name)); break;
      case 'spinning': this.show('run', 'Running ' + d.nodes.map(nm).join(', ')); S.stats.ranCode++; save(); checkBadges(); break;
      case 'hint': this.H().ui.pyLog(d.text, 'warn'); break;
      case 'crash': this.H().ui.pyLog(d.text, 'err'); if (d.line) this.H().ui.markErr(d.line); this.cleanup(); this.phase = 'ready'; this.show('err', 'Crashed: see the console'); break;
      case 'stopped': this.cleanup(); if (this.phase === 'running') { this.phase = 'ready'; this.show('', 'Finished'); } break;
    }
  },
};
const PY_TEMPLATES = {
  circle: { label: 'Drive in a circle (timer)', code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Circler(Node):
    def __init__(self):
        super().__init__('circler')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

    def tick(self):
        cmd = Twist()
        cmd.linear.x = 0.3     # m/s forward
        cmd.angular.z = 0.6    # rad/s, positive turns left
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Circler()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
` },
  square: { label: 'Drive a square (open loop)', code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Square(Node):
    """Drive about 1 m, turn about 90 degrees, repeat.
    Open loop: no sensors, so small errors add up. Watch it drift!"""

    def __init__(self):
        super().__init__('square')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.t = 0.0

    def tick(self):
        self.t += 0.1
        phase = self.t % 4.5
        cmd = Twist()
        if phase < 3.0:
            cmd.linear.x = 0.33      # about 1 m in 3 s
        elif phase < 4.2:
            cmd.angular.z = 1.31     # about 90 degrees in 1.2 s
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Square())


if __name__ == '__main__':
    main()
` },
  cautious: { label: 'Stop before walls (/scan)', code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Cautious(Node):
    def __init__(self):
        super().__init__('cautious')
        self.declare_parameter('safe_distance', 0.6)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, scan):
        # ranges[0] is straight ahead, counterclockwise; look at a 30 degree cone
        front = min(scan.ranges[:15] + scan.ranges[-15:])
        safe = self.get_parameter('safe_distance').value
        cmd = Twist()
        if front > safe:
            cmd.linear.x = 0.3
        self.pub.publish(cmd)
        self.get_logger().info(f'front: {front:.2f} m', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    rclpy.spin(Cautious())


if __name__ == '__main__':
    main()
` },
  avoid: { label: 'Wall avoider (/scan)', code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Avoider(Node):
    def __init__(self):
        super().__init__('avoider')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, scan):
        r = [min(d, 3.5) for d in scan.ranges]   # inf means "nothing seen"
        front = min(r[:25] + r[-25:])            # +-25 degrees ahead
        left = min(r[25:100])
        right = min(r[260:335])
        cmd = Twist()
        if front < 0.6:
            # blocked: spin toward the side with more room
            cmd.angular.z = 1.5 if left > right else -1.5
        else:
            cmd.linear.x = 0.35
            cmd.angular.z = 0.9 * (left - right)  # drift toward open space
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(Avoider())


if __name__ == '__main__':
    main()
` },
  goal: { label: 'Go to goal (/odom + click)', code: `import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.declare_parameter('goal_x', 8.6)
        self.declare_parameter('goal_y', 5.6)
        self.goal = (self.get_parameter('goal_x').value,
                     self.get_parameter('goal_y').value)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # click anywhere in the arena to publish a new /goal_pose
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'new goal: x={self.goal[0]:.1f} y={self.goal[1]:.1f}')

    def on_odom(self, odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)          # quaternion -> heading
        dx, dy = self.goal[0] - p.x, self.goal[1] - p.y
        dist = math.hypot(dx, dy)
        err = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))   # wrap to [-pi, pi]
        cmd = Twist()
        if dist > 0.15:
            cmd.angular.z = 2.0 * err
            if abs(err) < 0.5:
                cmd.linear.x = min(0.5, 0.8 * dist)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(GoToGoal())


if __name__ == '__main__':
    main()
` },
};

/* ============================== Lab UI ============================== */
const Lab = {
  inited: false, root: null, tab: 'drive', terms: [], ti: 0, raf: 0, last: 0, acc: 0, colors: null, showTF: true, keys: {}, goalPin: null,
  init() {
    if (this.inited) return; this.inited = true;
    Sim.init();
    this.tp = new TermPanel(labCtx, { welcome: 'Welcome to the Robot Lab terminal. ROS 2 Jazzy is already sourced.\nType `help` for ideas, or try: ros2 node list', chips: ['ros2 node list', 'ros2 topic list', 'ros2 topic echo /scan --once', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.4}}"', 'ros2 param set /robot_sim max_linear_speed 0.3', 'ros2 service call /reset_world std_srvs/srv/Empty', 'ros2 run tf2_ros tf2_echo odom base_link', 'ros2 launch lab_bringup wander.launch.py'], rerender: () => this.renderTab() });
    window.addEventListener('keydown', (e) => this.onKey(e, false)); window.addEventListener('keyup', (e) => this.onKey(e, true));
    try { const mo = new MutationObserver(() => { this.colors = null; }); mo.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] }); matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { this.colors = null; }); } catch (e) { /* ignore */ }
  },
  mount(root) {
    this.init(); this.root = root;
    if (!S.lab.code) S.lab.code = PY_TEMPLATES[S.lab.tpl || 'avoid'].code;
    root.innerHTML = '';
    const v = h('div', { class: 'lab' });
    v.innerHTML = `<div class="lab-head"><div><h1>Robot Lab</h1><p class="sub" style="color:var(--muted)">A simulated ROS 2 robot with a 360° lidar. Drive it, inspect its topics, or program it in Python.</p></div><div class="topstats" data-stats></div></div>
      <div class="mission-bar" aria-live="polite"></div>
      <div class="lab-grid">
        <div><div class="arena-wrap" tabindex="0" aria-label="Robot arena. Use arrow keys or WASD to drive. Click to send a goal pose."><canvas></canvas><div class="hud"></div></div>
          <div class="arena-tools"><button class="btn btn-sm" data-act="reset">${icon('replay')}Reset world</button><button class="btn btn-sm" data-act="tf" aria-pressed="true">${icon('axes')}TF frames</button><button class="btn btn-sm" data-act="rays" aria-pressed="true">${icon('radio')}Lidar rays</button><span class="grow"></span><span class="kbd-hint">Drive: <kbd>↑</kbd><kbd>↓</kbd><kbd>←</kbd><kbd>→</kbd> or <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> · click = <code>/goal_pose</code></span></div></div>
        <div class="lab-panel"><div class="ltabs" role="tablist" aria-label="Lab tools"></div><div class="lbody"></div></div>
      </div>`;
    root.append(v);
    this.cv = $('canvas', v); this.hud = $('.hud', v); this.arena = $('.arena-wrap', v);
    $('[data-act="reset"]', v).addEventListener('click', () => { Sim.reset(); Sound.play('tap'); });
    const tfb = $('[data-act="tf"]', v); tfb.addEventListener('click', () => { this.showTF = !this.showTF; tfb.setAttribute('aria-pressed', String(this.showTF)); });
    const rb = $('[data-act="rays"]', v); rb.addEventListener('click', () => { const e = Sim.params.get('show_rays'); e.v = !e.v; rb.setAttribute('aria-pressed', String(e.v)); });
    this.cv.addEventListener('click', (e) => this.clickGoal(e));
    this.renderMissionBar(); this.renderTabs(); this.renderTab();
    this.resize(); this.ro = new ResizeObserver(() => this.resize()); this.ro.observe(this.arena);
    this.last = performance.now(); cancelAnimationFrame(this.raf); this.raf = requestAnimationFrame((t) => this.frame(t));
    this.uiTimer = setInterval(() => this.tickUI(), 500);
  },
  unmount() { cancelAnimationFrame(this.raf); this.raf = 0; clearInterval(this.uiTimer); if (this.ro) this.ro.disconnect(); this.root = null; this.keys = {}; },
  focusTab(t) { this.tab = t; if (this.root) { this.renderTabs(); this.renderTab(); } },
  refreshTab(t) { if (this.root && this.tab === t) this.renderTab(); },
  resize() { const r = this.arena.getBoundingClientRect(), dpr = Math.min(2, devicePixelRatio || 1); this.cv.width = Math.round(r.width * dpr); this.cv.height = Math.round(r.width * 0.7 * dpr); this.scale = this.cv.width / LAB.W; },
  frame(t) {
    if (!this.root) return;
    const dt = Math.max(0, Math.min(0.1, (t - this.last) / 1000)); this.last = Math.max(this.last, t); this.acc += dt;
    while (this.acc >= 1 / 60) { Sim.step(1 / 60); this.acc -= 1 / 60; }
    this.draw(); this.raf = requestAnimationFrame((tt) => this.frame(tt));
  },
  pal() {
    if (this.colors) return this.colors;
    const c = (n) => cssVar(n);
    return (this.colors = { floor: c('--surface'), grid: c('--line'), wall: c('--ink-2'), obst: c('--surface-3'), obstEdge: c('--muted'), pad: c('--good'), beacon: c('--c-msg'), ray: c('--bad'), trail: c('--c-topic'), ink: c('--ink'), accent: c('--accent'), x: c('--c-x'), y: c('--c-y'), muted: c('--muted'), srv: c('--c-srv') });
  },
  draw() {
    const g = this.cv.getContext('2d'), k = this.scale, H = this.cv.height, P = this.pal();
    const X = (x) => x * k, Y = (y) => H - y * k;
    g.fillStyle = P.floor; g.fillRect(0, 0, this.cv.width, H);
    g.strokeStyle = P.grid; g.lineWidth = 1; g.globalAlpha = .7;
    for (let x = 1; x < LAB.W; x++) { g.beginPath(); g.moveTo(X(x), 0); g.lineTo(X(x), H); g.stroke(); }
    for (let y = 1; y < LAB.H; y++) { g.beginPath(); g.moveTo(0, Y(y)); g.lineTo(this.cv.width, Y(y)); g.stroke(); }
    g.globalAlpha = 1;
    const tt = performance.now() / 1000;
    g.fillStyle = P.pad; g.globalAlpha = .22 + .08 * Math.sin(tt * 3); g.beginPath(); g.arc(X(PAD.x), Y(PAD.y), PAD.r * k, 0, 7); g.fill(); g.globalAlpha = 1;
    g.strokeStyle = P.pad; g.lineWidth = Math.max(2, k * 0.03); g.setLineDash([k * .08, k * .06]); g.beginPath(); g.arc(X(PAD.x), Y(PAD.y), PAD.r * k, 0, 7); g.stroke(); g.setLineDash([]);
    g.fillStyle = P.pad; g.font = `600 ${Math.max(10, k * 0.14)}px Lexend, system-ui`; g.textAlign = 'center'; g.fillText('GOAL', X(PAD.x), Y(PAD.y) + k * 0.05);
    g.strokeStyle = P.muted; g.globalAlpha = .5; g.lineWidth = 1.5; g.beginPath(); g.arc(X(HOME.x), Y(HOME.y), 0.3 * k, 0, 7); g.stroke(); g.globalAlpha = 1;
    OBST.boxes.forEach(([a, b, c, d]) => { g.fillStyle = P.obst; g.fillRect(X(a), Y(d), (c - a) * k, (d - b) * k); g.strokeStyle = P.obstEdge; g.lineWidth = 2; g.strokeRect(X(a), Y(d), (c - a) * k, (d - b) * k); });
    OBST.circles.forEach(([cx, cy, r]) => { g.fillStyle = P.obst; g.beginPath(); g.arc(X(cx), Y(cy), r * k, 0, 7); g.fill(); g.strokeStyle = P.obstEdge; g.lineWidth = 2; g.stroke(); });
    g.strokeStyle = P.wall; g.lineWidth = Math.max(4, k * 0.06); g.strokeRect(g.lineWidth / 2, g.lineWidth / 2, this.cv.width - g.lineWidth, H - g.lineWidth);
    BEACONS.forEach(([bx, by], i) => { if (Sim.got[i]) return; const pr = 0.13 + 0.03 * Math.sin(tt * 4 + i); g.fillStyle = P.beacon; g.globalAlpha = .3; g.beginPath(); g.arc(X(bx), Y(by), (pr + .1) * k, 0, 7); g.fill(); g.globalAlpha = 1; g.beginPath(); g.arc(X(bx), Y(by), pr * k, 0, 7); g.fill(); });
    if (this.goalPin) { const [gx, gy] = this.goalPin; g.strokeStyle = P.srv; g.lineWidth = 3; g.beginPath(); g.moveTo(X(gx), Y(gy)); g.lineTo(X(gx), Y(gy) - k * .45); g.stroke(); g.fillStyle = P.srv; g.beginPath(); g.moveTo(X(gx), Y(gy) - k * .45); g.lineTo(X(gx) + k * .28, Y(gy) - k * .36); g.lineTo(X(gx), Y(gy) - k * .27); g.fill(); }
    if (Sim.trail.length > 1) { g.strokeStyle = P.trail; g.globalAlpha = .55; g.lineWidth = Math.max(2, k * .03); g.beginPath(); Sim.trail.forEach(([x, y], i) => { i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); }); g.stroke(); g.globalAlpha = 1; }
    if (Sim.lidar && Sim.P('show_rays')) {
      g.strokeStyle = P.ray; g.globalAlpha = .18; g.lineWidth = 1; const inc = 2 * Math.PI / NRAYS;
      for (let i = 0; i < NRAYS; i += 4) { const r = Sim.ranges[i]; const L = Number.isFinite(r) ? r : Sim.P('lidar_range'); const a = Sim.th + i * inc; g.beginPath(); g.moveTo(X(Sim.x), Y(Sim.y)); g.lineTo(X(Sim.x + L * Math.cos(a)), Y(Sim.y + L * Math.sin(a))); g.stroke(); }
      g.globalAlpha = .9; g.fillStyle = P.ray;
      for (let i = 0; i < NRAYS; i += 2) { const r = Sim.ranges[i]; if (!Number.isFinite(r)) continue; const a = Sim.th + i * inc; g.fillRect(X(Sim.x + r * Math.cos(a)) - 1.5, Y(Sim.y + r * Math.sin(a)) - 1.5, 3, 3); }
      g.globalAlpha = 1;
    }
    if (this.showTF) { this.axes(g, X(0) + 12, Y(0) - 12, 0, k * .6, P, 'odom'); }
    const R = LAB.R * k;
    g.save(); g.translate(X(Sim.x), Y(Sim.y)); g.rotate(-Sim.th);
    g.fillStyle = '#1C2B25'; g.fillRect(-R * .6, -R * 1.12, R * 1.2, R * .32); g.fillRect(-R * .6, R * .8, R * 1.2, R * .32);
    g.fillStyle = Sim.bumping ? cssVar('--bad') : '#F4F8F6'; g.strokeStyle = '#1C2B25'; g.lineWidth = Math.max(2, R * .1); g.beginPath(); g.arc(0, 0, R, 0, 7); g.fill(); g.stroke();
    g.fillStyle = '#3A4A43'; g.beginPath(); g.arc(0, 0, R * .38, 0, 7); g.fill();
    g.fillStyle = '#F26419'; g.beginPath(); g.moveTo(R * .55, -R * .3); g.lineTo(R * .95, 0); g.lineTo(R * .55, R * .3); g.fill();
    g.restore();
    if (this.showTF) this.axes(g, X(Sim.x), Y(Sim.y), Sim.th, k * .45, P, 'base_link');
  },
  axes(g, x, y, th, L, P, label) {
    const line = (dx, dy, col) => { g.strokeStyle = col; g.lineWidth = 3; g.lineCap = 'round'; g.beginPath(); g.moveTo(x, y); g.lineTo(x + dx, y + dy); g.stroke(); };
    line(L * Math.cos(th), -L * Math.sin(th), P.x); line(-L * Math.sin(th), -L * Math.cos(th), P.y);
    if (label) { g.fillStyle = P.ink; g.font = '600 11px JetBrains Mono, monospace'; g.textAlign = 'left'; g.fillText(label, x + 6, y + 14); }
  },
  clickGoal(e) {
    const r = this.cv.getBoundingClientRect(), x = (e.clientX - r.left) / r.width * LAB.W, y = (1 - (e.clientY - r.top) / r.height) * LAB.H;
    this.goalPin = [x, y];
    if (!Bus.nodes.has('/goal_tool')) { Bus.addNode('/goal_tool', 'tool'); Bus.advertise('/goal_tool', '/goal_pose', 'geometry_msgs/msg/PoseStamped'); }
    Bus.publish('/goal_tool', '/goal_pose', 'geometry_msgs/msg/PoseStamped', { header: { stamp: stampNow(), frame_id: 'odom' }, pose: { position: { x: +x.toFixed(2), y: +y.toFixed(2), z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } });
    Sound.play('tap');
  },
  tickUI() {
    if (!this.root) return;
    const src = performance.now() / 1000 - Sim.cmd.t < Sim.P('cmd_timeout') ? Sim.cmd.src : '';
    this.hud.innerHTML = `<span>x ${Sim.x.toFixed(2)}  y ${Sim.y.toFixed(2)}  θ ${(Sim.th * 180 / Math.PI).toFixed(0)}°</span><span>v ${Sim.v.toFixed(2)} m/s  w ${Sim.w.toFixed(2)} rad/s</span>${src ? `<span>/cmd_vel ← ${esc(src)}</span>` : ''}<span class="${Sim.bumping ? 'bump' : ''}">bumps ${Sim.bumps}</span><span>beacons ${Sim.got.filter(Boolean).length}/3</span>`;
    if (this.tab === 'graph') this.renderGraph();
    if (this.tab === 'topics') this.renderTopicsLive();
    if (this.tab === 'drive') this.renderTwist();
  },
  renderMissionBar() {
    if (!this.root) return;
    const bar = $('.mission-bar', this.root), m = Missions.current(), n = Object.keys(S.lab.missions).length;
    if (!m) { bar.innerHTML = `${icon('trophy')}<div class="grow"><div class="mt">All ${MISSIONS.length} missions complete!</div><div class="mh">You are a Robot Lab legend. Keep experimenting: try writing your own wall follower.</div></div>`; return; }
    bar.innerHTML = `${icon('flag')}<div class="grow"><div class="mt">Mission ${MISSIONS.indexOf(m) + 1} of ${MISSIONS.length}: ${esc(m.title)} <span class="chip" style="margin-left:6px">+${m.xp} XP</span></div><div class="mh">${md(m.text)}</div></div><span class="sub tabnum" style="color:var(--muted);font-size:13px">${n} done</span>`;
    const b = h('button', { class: 'btn btn-sm btn-primary' }, 'Show me'); b.addEventListener('click', () => { this.focusTab(m.tab); });
    bar.append(b);
  },
  renderTabs() {
    const T = [['drive', 'drive', 'Drive'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['graph', 'graph', 'Graph'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions']];
    const bar = $('.ltabs', this.root); bar.innerHTML = '';
    T.forEach(([id, ic, label]) => {
      const b = h('button', { class: 'ltab', role: 'tab', id: 'lt-' + id, 'aria-selected': String(this.tab === id), 'aria-controls': 'lab-panel', html: icon(ic) + label + (id === 'missions' ? `<span class="cnt">${MISSIONS.length - Object.keys(S.lab.missions).length}</span>` : '') });
      b.addEventListener('click', () => { this.tab = id; this.renderTabs(); this.renderTab(); });
      bar.append(b);
    });
  },
  renderTab() {
    const body = $('.lbody', this.root); body.innerHTML = ''; body.id = 'lab-panel'; body.setAttribute('role', 'tabpanel'); body.setAttribute('aria-labelledby', 'lt-' + this.tab);
    this.graphSig = '';
    ({ drive: () => this.tabDrive(body), term: () => this.tabTerm(body), code: () => this.tabCode(body), graph: () => this.tabGraph(body), topics: () => this.tabTopics(body), missions: () => this.tabMissions(body) })[this.tab]();
  },
  tabDrive(body) {
    const id = ++UID;
    body.innerHTML = `<div class="joy-wrap"><div class="joy" role="application" aria-label="Joystick: drag to drive"><div class="knob"></div></div>
      <div><div class="dpad" aria-label="Arrow buttons"><span></span><button data-d="f" aria-label="Forward">${icon('arrowU')}</button><span></span><button data-d="l" aria-label="Turn left">${icon('arrowL')}</button><button data-d="s" aria-label="Stop">${icon('stop')}</button><button data-d="r" aria-label="Turn right">${icon('arrowR')}</button><span></span><button data-d="b" aria-label="Backward">${icon('arrowD')}</button><span></span></div>
      <div class="sliders" style="margin-top:14px"><div class="slider"><label for="sp${id}">max speed</label><input id="sp${id}" type="range" min="0.1" max="1" step="0.05" value="${this.speed || 0.5}"><output>${(this.speed || 0.5).toFixed(2)}</output></div></div></div></div>
      <div class="lpad"><div class="twist-readout"></div><p class="sidecap">The pad publishes <code>geometry_msgs/msg/Twist</code> on <code>/cmd_vel</code> from a node called <code>/teleop_panel</code>, just like a real teleop node. The robot still obeys its own <code>max_linear_speed</code> parameter.</p></div>`;
    const sp = $(`#sp${id}`, body); sp.addEventListener('input', () => { this.speed = +sp.value; $('output', body).textContent = this.speed.toFixed(2); });
    this.speed = this.speed || 0.5;
    const joy = $('.joy', body), knob = $('.knob', joy);
    let dragging = false;
    const move = (e) => { const r = joy.getBoundingClientRect(), R = r.width / 2 - 20; let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2); const L = Math.hypot(dx, dy); if (L > R) { dx *= R / L; dy *= R / L; } knob.style.transform = `translate(${dx}px, ${dy}px)`; Builtin.teleop(+(-dy / R * this.speed).toFixed(2), +(-dx / R * 1.8).toFixed(2)); };
    joy.addEventListener('pointerdown', (e) => { dragging = true; joy.setPointerCapture(e.pointerId); move(e); });
    joy.addEventListener('pointermove', (e) => { if (dragging) move(e); });
    const end = () => { if (!dragging) return; dragging = false; knob.style.transform = ''; Builtin.teleop(0, 0); };
    joy.addEventListener('pointerup', end); joy.addEventListener('pointercancel', end);
    $$('.dpad button', body).forEach(b => {
      const dir = b.dataset.d; const vw = { f: [1, 0], b: [-1, 0], l: [0, 1], r: [0, -1], s: [0, 0] }[dir];
      const down = (e) => { e.preventDefault(); b.classList.add('on'); Builtin.teleop(vw[0] * this.speed, vw[1] * 1.5); };
      const up = () => { b.classList.remove('on'); Builtin.teleop(0, 0); };
      b.addEventListener('pointerdown', down); b.addEventListener('pointerup', up); b.addEventListener('pointerleave', () => { if (b.classList.contains('on')) up(); });
    });
    this.renderTwist();
  },
  renderTwist() {
    const el = this.root && $('.twist-readout', this.root); if (!el) return;
    const T = Bus.topics.get('/cmd_vel'), m = T && T.last, act = Bus.active('/cmd_vel');
    el.innerHTML = `/cmd_vel ${act ? '<span style="color:var(--term-green)">● publishing</span>' : '<span style="color:var(--term-dim)">○ idle</span>'}<br>linear.x: <b>${m ? pyFloat(+(+m.linear.x).toFixed(2)) : '0.0'}</b>   angular.z: <b>${m ? pyFloat(+(+m.angular.z).toFixed(2)) : '0.0'}</b>${act && T.pubAt ? `<br><span style="color:var(--term-dim)">last from ${esc(Sim.cmd.src || '')}</span>` : ''}`;
  },
  onKey(e, up) {
    if (!this.root || App.view !== 'lab' || $('.player') || $('.modal-back') || $('.drawer')) return;
    const t = this.term(); const tag = (e.target.tagName || '').toLowerCase();
    if (t && t.proc && t.proc.key && e.target.classList && e.target.classList.contains('lterm-input')) { if (e.key.length === 1) { e.preventDefault(); t.proc.key(e.key.toLowerCase(), up); } return; }
    if (tag === 'input' || tag === 'textarea' || tag === 'select' || e.metaKey || e.ctrlKey || e.altKey) return;
    const map = { ArrowUp: 'f', w: 'f', W: 'f', ArrowDown: 'b', s: 'b', S: 'b', ArrowLeft: 'l', a: 'l', A: 'l', ArrowRight: 'r', d: 'r', D: 'r' };
    const d = map[e.key]; if (!d) return;
    const inLab = this.arena.contains(document.activeElement) || document.activeElement === document.body || this.root.contains(document.activeElement);
    if (!inLab) return;
    e.preventDefault();
    this.keys[d] = !up;
    const K = this.keys, v = (K.f ? 1 : 0) - (K.b ? 1 : 0), w = (K.l ? 1 : 0) - (K.r ? 1 : 0);
    Builtin.teleop(v * (this.speed || 0.5), w * 1.5);
  },
  term() { return this.tp.term(); },
  get terms() { return this.tp ? this.tp.terms : []; },
  get ti() { return this.tp ? this.tp.ti : 0; },
  set ti(v) { if (this.tp) this.tp.ti = v; },
  tabTerm(body) { this.tp.mount(body); },
  termState() { if (this.tp) this.tp.state(); },
  tabCode(body) {
    const tpl = S.lab.tpl || 'avoid';
    body.innerHTML = `<div class="code-tools"><label class="sr" for="tplsel">Template</label><select class="sel" id="tplsel">${Object.entries(PY_TEMPLATES).map(([k, t]) => `<option value="${k}"${k === tpl ? ' selected' : ''}>${esc(t.label)}</option>`).join('')}</select>
      <button class="btn btn-sm btn-ghost" data-a="load">Load template</button><span class="grow"></span><span class="py-status" aria-live="polite"></span>
      <button class="btn btn-sm btn-good" data-a="run">${icon('play')}Run</button><button class="btn btn-sm" data-a="stop">${icon('stop')}Stop</button></div>
      <div class="editor"><div class="gutter" aria-hidden="true"><div class="gi"></div></div><div class="ed"><pre aria-hidden="true"></pre><textarea spellcheck="false" autocapitalize="off" autocomplete="off" wrap="off" aria-label="Python code editor"></textarea></div></div>
      <div class="pyconsole" role="log" aria-live="polite" aria-label="Python console"></div>`;
    const ta = $('textarea', body), pre = $('pre', body), gi = $('.gi', body), sel = $('#tplsel', body);
    ta.value = S.lab.code;
    const paint = () => {
      const lines = ta.value.split('\n');
      pre.innerHTML = lines.map(l => hlLine(l, 'py')).join('\n') + '\n ';
      gi.innerHTML = lines.map((_, i) => `<div class="${this.errLine === i + 1 ? 'err' : ''}">${i + 1}</div>`).join('');
      sync();
    };
    const sync = () => { pre.style.transform = `translate(${-ta.scrollLeft}px, ${-ta.scrollTop}px)`; gi.style.transform = `translateY(${-ta.scrollTop}px)`; };
    let saveT = 0;
    ta.addEventListener('input', () => { this.errLine = 0; paint(); clearTimeout(saveT); saveT = setTimeout(() => { S.lab.code = ta.value; save({ local: true }); }, 600); });
    ta.addEventListener('scroll', sync);
    ta.addEventListener('keydown', (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); Py.bind(null); Py.run(ta.value); return; }
      if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); document.execCommand('insertText', false, '    ') || ins('    '); }
      if (e.key === 'Enter') { const s = ta.selectionStart, before = ta.value.slice(0, s), line = before.slice(before.lastIndexOf('\n') + 1); const ind = (line.match(/^\s*/) || [''])[0] + (/:\s*$/.test(line) ? '    ' : ''); e.preventDefault(); document.execCommand('insertText', false, '\n' + ind) || ins('\n' + ind); }
    });
    const ins = (txt) => { const s = ta.selectionStart, en = ta.selectionEnd; ta.value = ta.value.slice(0, s) + txt + ta.value.slice(en); ta.selectionStart = ta.selectionEnd = s + txt.length; ta.dispatchEvent(new Event('input')); return true; };
    $('[data-a="load"]', body).addEventListener('click', () => { const load = () => { S.lab.tpl = sel.value; ta.value = PY_TEMPLATES[sel.value].code; S.lab.code = ta.value; save({ local: true }); paint(); }; if (ta.value !== PY_TEMPLATES[S.lab.tpl || 'avoid'].code) modal({ mood: 'think', title: 'Replace your code?', body: 'Loading a template replaces what is in the editor.', actions: [{ label: 'Load template', cls: 'btn-primary', onClick: load }, { label: 'Cancel', cls: 'btn-ghost' }] }); else load(); });
    sel.addEventListener('change', () => { if (ta.value === PY_TEMPLATES[S.lab.tpl || 'avoid'].code) { S.lab.tpl = sel.value; ta.value = PY_TEMPLATES[sel.value].code; S.lab.code = ta.value; save({ local: true }); paint(); } });
    $('[data-a="run"]', body).addEventListener('click', () => { Py.bind(null); Py.run(ta.value); });
    $('[data-a="stop"]', body).addEventListener('click', () => { if (!Py.host) Py.stop(); });
    this.pycon = $('.pyconsole', body); this.paintCode = paint;
    (this.pyLines || []).forEach(([t, c]) => this.pycon.append(h('div', { class: c }, t)));
    if (!(this.pyLines || []).length) this.pyLog('Pick a template, press Run (or Ctrl+Enter), and watch the robot. Your node really publishes /cmd_vel: check the Graph and Topics tabs while it runs.', 'sys');
    this.pycon.scrollTop = this.pycon.scrollHeight;
    paint(); this.pyStatus();
  },
  pyLog(text, cls = 'info') {
    this.pyLines = this.pyLines || [];
    String(text).replace(/\n$/, '').split('\n').forEach(l => this.pyLines.push([l, cls]));
    if (this.pyLines.length > 400) this.pyLines.splice(0, 100);
    if (this.pycon && this.pycon.isConnected) { String(text).replace(/\n$/, '').split('\n').forEach(l => this.pycon.append(h('div', { class: cls }, l || ' '))); while (this.pycon.childElementCount > 400) this.pycon.firstChild.remove(); this.pycon.scrollTop = this.pycon.scrollHeight; }
  },
  pyStatus() {
    const el = this.root && $('.py-status', this.root); if (!el) return;
    el.className = 'py-status ' + (Py.disp || ''); el.textContent = Py.text;
  },
  markErr(line) { this.errLine = line; if (this.paintCode && this.root && this.tab === 'code') this.paintCode(); },
  clearErr() { this.errLine = 0; if (this.paintCode && this.root && this.tab === 'code') this.paintCode(); },
  tabGraph(body) {
    body.innerHTML = `<div class="graph-box" aria-live="off"></div><div class="graph-legend"><span><i style="background:var(--c-node-soft);border:2px solid var(--c-node)"></i>node</span><span><i style="background:var(--c-topic-soft);border:2px solid var(--c-topic)"></i>topic</span><span><i style="background:var(--c-msg)"></i>message flowing</span><span>like <code>rqt_graph</code>; debug topics hidden</span></div>`;
    this.renderGraph();
  },
  renderGraph() {
    const box = this.root && $('.graph-box', this.root); if (!box) return;
    const nodes = [...Bus.nodes.values()].filter(n => !n.hidden && (n.pubs.size || n.subs.size));
    const topics = [...Bus.topics.values()].filter(t => !HIDE_TOPICS.has(t.name) && (t.pubs.size || t.subs.size)).sort((a, b) => a.name.localeCompare(b.name));
    const left = nodes.filter(n => n.name !== '/robot_sim').sort((a, b) => a.name.localeCompare(b.name)), right = nodes.filter(n => n.name === '/robot_sim');
    const rows = Math.max(left.length, topics.length, 2), H = Math.max(240, rows * 54 + 50);
    const G = []; const E = [];
    const ly = (i, n) => H / 2 + (i - (n - 1) / 2) * 54;
    left.forEach((n, i) => G.push(GN(n.name, n.name, 88, ly(i, left.length), 'node', { w: 168 })));
    right.forEach((n) => G.push(GN(n.name, n.name, 444, H / 2, 'node', { w: 138 })));
    topics.forEach((t, i) => G.push(GN('T' + t.name, t.name, 262, ly(i, topics.length), 'topic', { w: 130 })));
    const has = (id) => G.some(g => g.id === id);
    topics.forEach(t => {
      t.pubs.forEach((_, n) => { if (has(n)) E.push({ a: n, b: 'T' + t.name, flow: Bus.active(t.name, n) ? 'msg' : null, dur: 1.4, n: 2 }); });
      t.subs.forEach((_, n) => { if (has(n)) E.push({ a: 'T' + t.name, b: n, flow: Bus.active(t.name) && [...t.pubs.keys()].some(p => p !== n) ? 'msg' : null, dur: 1.4, n: 2 }); });
    });
    const sig = JSON.stringify([G.map(g => g.id), E.map(e => e.a + e.b + (e.flow ? 1 : 0))]);
    if (sig === this.graphSig) return; this.graphSig = sig;
    box.innerHTML = graphSVG({ vw: 520, vh: H, nodes: G, edges: E, reveal: false, alt: 'Live ROS graph of the lab' });
  },
  tabTopics(body) {
    body.innerHTML = `<div style="overflow-x:auto"><table class="ttable"><thead><tr><th scope="col">Topic</th><th scope="col">Type</th><th scope="col" class="hz">Hz</th><th scope="col" class="hz">Pub/Sub</th></tr></thead><tbody></tbody></table></div><div class="echo-box" aria-live="off">Select a topic to see its latest message, like <code>ros2 topic echo</code>.</div>`;
    this.renderTopicsLive();
  },
  renderTopicsLive() {
    const tb = this.root && $('.ttable tbody', this.root); if (!tb) return;
    const ts = [...Bus.topics.values()].sort((a, b) => a.name.localeCompare(b.name));
    tb.innerHTML = '';
    ts.forEach(t => { const tr = h('tr', { class: this.selTopic === t.name ? 'sel' : '', tabindex: '0', html: `<td>${esc(t.name)}</td><td>${esc(t.type || '?')}</td><td class="hz">${Bus.hz(t.name).toFixed(1)}</td><td class="hz">${t.pubs.size}/${t.subs.size}</td>` }); const pick = () => { this.selTopic = t.name; this.renderTopicsLive(); }; tr.addEventListener('click', pick); tr.addEventListener('keydown', (e) => { if (e.key === 'Enter') pick(); }); tb.append(tr); });
    const eb = $('.echo-box', this.root), T = Bus.topics.get(this.selTopic);
    if (T) eb.textContent = T.last && IFACES[T.lastType] ? `$ ros2 topic echo ${T.name}\n` + yamlMsg(T.lastType, T.last).join('\n') + '\n---' : `${T.name}: no messages yet`;
  },
  tabMissions(body) {
    const list = h('div', { class: 'missions' });
    const cur = Missions.current();
    MISSIONS.forEach((m, i) => {
      const done = Missions.done(m.id);
      const b = h('button', { class: 'mission' + (done ? ' done' : '') + (cur === m ? ' active' : ''), html: `<span class="num">${done ? icon('check') : i + 1}</span><span><b>${esc(m.title)}</b><span>${md(m.text)}</span></span><span class="xp">${done ? 'Done' : '+' + m.xp + ' XP'}</span>` });
      b.addEventListener('click', () => this.focusTab(m.tab));
      list.append(b);
    });
    body.append(list);
  },
};
