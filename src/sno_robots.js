/* ============================== SnoBots: AiNex and R1 specifics ============================== */
const D2R = Math.PI / 180;
const AINEX_FIELD = { x0: -0.9, x1: 1.5, y0: -0.9, y1: 0.9, goalY: 0.25, goalD: 0.18, goalH: 0.2 };
// the head camera: 320x240 through a 120-degree (diagonal) wide-angle lens, simulated as an ideal pinhole (no distortion)
const AINEX_CAM = (() => { const w = 320, h = 240, f = +(Math.hypot(w, h) / 2 / Math.tan(60 * D2R)).toFixed(4); return { w, h, f, vfov: 2 * Math.atan(h / 2 / f) / D2R, hfov: 2 * Math.atan(w / 2 / f), near: 0.01, far: 20 }; })();
// ArUco tags (DICT_4X4_50) on posts around the pitch and one above the goal; size is the black square, z its centre
const AINEX_TAGS = [
  { id: 1, x: -1.0, y: -0.45, z: 0.165, yaw: 0, size: 0.15 }, { id: 2, x: -1.0, y: 0.45, z: 0.165, yaw: 0, size: 0.15 },
  { id: 4, x: -0.2, y: 1.0, z: 0.165, yaw: -Math.PI / 2, size: 0.15 }, { id: 8, x: 0.8, y: -1.0, z: 0.165, yaw: Math.PI / 2, size: 0.15 },
  { id: 0, x: 1.74, y: 0, z: 0.36, yaw: Math.PI, size: 0.2 },
];
// fixed joints that robot_state_publisher sends on /tf_static
const AINEX_STATIC = ['torso_joint', 'camera_joint', 'camera_optical_joint', 'imu_joint', 'l_sole_joint', 'r_sole_joint'];
// R1's head camera: the binocular depth camera's rectified colour view plus the depth it computes, registered to one
// 320x240 pinhole image with a 90° horizontal view (the real lenses see wider, up to about 146°), 15 Hz. Depth is
// 16UC1 in millimetres (REP 118), 0 outside 0.2 to 6 m, with stereo-like noise that grows with distance squared.
const R1_CAM = (() => { const w = 320, h = 240, f = 160; return { w, h, f, vfov: 2 * Math.atan(h / 2 / f) / D2R, hfov: 2 * Math.atan(w / 2 / f), near: 0.01, far: 30, dmin: 0.2, dmax: 6.0, noise: 0.005 }; })();
// fixed joints that robot_state_publisher sends on /tf_static
const R1_STATIC = ['camera_joint', 'camera_optical_joint', 'imu_joint', 'left_sole_joint', 'right_sole_joint'];
// the visitor: a mannequin in a violet vest (what R1's person locator looks for) who stands, or strolls a loop
// in front of R1; vest: the bottom and top of the vest (z), r: the torso's radius
const R1_VISITOR = { start: [2.3, 0, Math.PI], path: [[2.0, -0.85], [4.3, -0.85], [4.3, 0.85], [2.0, 0.85]], speed: 0.3, pause: 1.6, r: 0.155, vest: [0.98, 1.36], color: '#8E2BC9' };


class AinexSim extends HumanoidSim {
  reset() { super.reset(); this.vst = { watch: 0, headT: -9, still: 0, ids: new Set(), kickT: -99, hz: false, info: false }; }
  /* ---------- ROS: the robot's software runs as processes that its bringup launch file starts ---------- */
  setupRos() {
    const B = this.bus, cam = '/usb_cam';
    this.tn = {}; this.tfStatic = new Map();
    // the TF tree is what is actually being published; /marker_id is watched for the tag mission
    B.onPublish = (node, topic, type, msg) => {
      if (topic === '/tf_static' && msg && msg.transforms) this.noteStatic(node, msg.transforms);
      if (topic === '/marker_id' && type === 'std_msgs/msg/Int32' && msg) this.tagSeen(Math.trunc(+msg.data), node);
    };
    B.onRemove = (node) => { this.tfStatic.delete(node); };
    B.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true;
    // the detector and the ball locator subscribe like any node but never need the pixels, so they don't make the camera render
    this.camCfg = { key: 'cam', node: cam, w: AINEX_CAM.w, h: AINEX_CAM.h, f: AINEX_CAM.f, fps: () => this.P(cam, 'framerate'), on: () => true, frame: () => String(this.P(cam, 'frame_id') || ''), internal: new Set(['/color_detector', '/ball_locator']) };
    this.trk = { on: false, held: 0 }; this.rx = { on: false, cool: 0 }; this.det = null; this.goals = this.goals || 0;
    this.restoreRobot({});
  }
  pkgs() { return Object.assign(super.pkgs(), { robot_state_publisher: ['robot_state_publisher'], ainex_controller: ['ainex_controller'], usb_cam: ['usb_cam_node_exe'], ainex_vision: ['color_detector', 'ball_locator'] }); }
  onParam(node, k, v) {
    if (node === '/color_detector' && k === 'target_color' && !['red', 'green', 'blue', 'any'].includes(String(v))) return { ok: false, reason: 'target_color must be red, green, blue or any' };
    this.event('param', { node, k, v }); return { ok: true };
  }
  // cfg works like LimoSim.restoreRobot's: <proc>: false leaves it out, { params, remaps } changes it, died: crashes at
  // start-up. joy starts the joystick's teleop node ('twist' or 'stamped'), tracking switches head tracking on.
  restoreRobot(cfg = {}) {
    const B = this.bus;
    Object.keys(this.procs).forEach(k => this.stopProc(k));
    if (this.trk.on) this.setTracker(false); if (this.rx.on) this.setReactor(false);
    [...B.nodes.keys()].filter(n => n === '/teleop_panel' || /^\/(static_transform_publisher_|relay)/.test(n)).forEach(n => B.removeNode(n));
    this._tele = null; this.keys = {}; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; this.warnT = {}; this.lastWarn = ''; B.clashes.clear();
    this.launchProcs(['rsp', 'ctl', 'cam', 'det', 'ball'], cfg);
    if (cfg.joy) { const st = cfg.joy === 'stamped'; this.node('/teleop_panel', 'teleop', [['publish_stamped_twist', st, 'bool']]); B.advertise('/teleop_panel', '/cmd_vel', st ? 'geometry_msgs/msg/TwistStamped' : 'geometry_msgs/msg/Twist'); }
    if (cfg.tracking) this.setTracker(true);
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  procDefs() {
    if (this._pd) return this._pd;
    const B = this.bus, H = this.H, TF = 'tf2_msgs/msg/TFMessage', IMG = 'sensor_msgs/msg/Image', INFO = 'sensor_msgs/msg/CameraInfo', latched = { dur: 'TRANSIENT_LOCAL', depth: 1 };
    const qosOk = (k, v) => k.startsWith('qos_overrides.') && !['reliable', 'best_effort', 'system_default'].includes(String(v)) ? `${k}: '${v}' is not a reliability (use reliable or best_effort)` : '';
    const qos = (node, k) => this.P(node, k) === 'best_effort' ? { rel: 'BEST_EFFORT', depth: 5 } : { rel: 'RELIABLE', depth: 10 };
    return (this._pd = {
      rsp: { pkg: 'robot_state_publisher', exe: 'robot_state_publisher', node: '/robot_state_publisher', kind: 'description', params: () => [['publish_frequency', 20.0, 'double', [1, 100]]],
        start: () => {
          const n = '/robot_state_publisher'; B.advertise(n, '/tf', TF); B.subscribe(n, '/joint_states', 'sensor_msgs/msg/JointState', () => {});
          B.advertise(n, '/tf_static', TF, latched); this.pub(n, '/tf_static', TF, this.tfMsg(AINEX_STATIC.map(j => { const J = this.kin.J[j]; return [J.parent, J.child, J.O]; })));
          ['base_link', 'l_hip_yaw_link', 'r_hip_yaw_link', 'torso_link'].forEach(l => this.log(n, 'INFO', `got segment ${l}`));
        } },
      ctl: { pkg: 'ainex_controller', exe: 'ainex_controller', node: H.ctl, kind: 'sim', params: () => H.params, start: (rec) => this.ctlStart(rec), stop: () => this.ctlStop() },
      // usb_cam reads its settings once at start-up, so they are read-only here: restart the driver to change them
      cam: { pkg: 'usb_cam', exe: 'usb_cam_node_exe', node: '/usb_cam', kind: 'driver',
        params: () => [['video_device', '/dev/video0', 'string', null, true], ['image_width', AINEX_CAM.w, 'integer', null, true], ['image_height', AINEX_CAM.h, 'integer', null, true], ['framerate', 15.0, 'double', [1, 30], true], ['pixel_format', 'yuyv', 'string', null, true], ['io_method', 'mmap', 'string', null, true], ['camera_name', 'ainex_head', 'string', null, true], ['frame_id', 'camera_optical_frame', 'string', null, true], ['qos_overrides./image_raw.publisher.reliability', 'reliable', 'string', null, true]],
        check: (k, v) => qosOk(k, v) || ((k === 'image_width' && v !== AINEX_CAM.w) || (k === 'image_height' && v !== AINEX_CAM.h) ? `${k}: the simulated camera streams ${AINEX_CAM.w}x${AINEX_CAM.h} only` : ''),
        crash: "terminate called after throwing an instance of 'std::runtime_error'  what():  Cannot identify '/dev/video0': 2, No such file or directory",
        start: (rec) => {
          const n = '/usb_cam', R = rec.R; this.tn.img = R('/image_raw'); this.tn.info = R('/camera_info'); this.camLast = null; this.camDue = null;
          B.advertise(n, this.tn.img, IMG, qos(n, 'qos_overrides./image_raw.publisher.reliability')); B.advertise(n, this.tn.info, INFO);
          const nm = this.P(n, 'camera_name'), fps = +this.P(n, 'framerate');
          this.log(n, 'INFO', `camera_name value: ${nm}`);
          this.log(n, 'INFO', `Starting '${nm}' (${this.P(n, 'video_device')}) at ${AINEX_CAM.w}x${AINEX_CAM.h} via ${this.P(n, 'io_method')} (${this.P(n, 'pixel_format')}) at ${Math.round(fps)} FPS`);
          this.log(n, 'INFO', `Timer triggering every ${Math.round(1000 / fps)} ms`);
        },
        stop: () => { this.tn.img = null; this.tn.info = null; this.camDue = null; } },
      det: { pkg: 'ainex_vision', exe: 'color_detector', node: '/color_detector', kind: 'vision',
        params: () => [['target_color', 'red', 'string'], ['min_area', 0.0001, 'double', [0, 0.05]], ['qos_overrides./image_raw.subscription.reliability', 'reliable', 'string', null, true]],
        check: (k, v) => qosOk(k, v) || (k === 'target_color' && !['red', 'green', 'blue', 'any'].includes(String(v)) ? 'target_color must be red, green, blue or any' : ''),
        start: (rec) => {
          const n = '/color_detector', R = rec.R; this.det = null; this.tn.det = R('/color_detection');
          rec.un.push(B.subscribe(n, R('/image_raw'), IMG, (m) => this.visionIn('det', m.header), qos(n, 'qos_overrides./image_raw.subscription.reliability')), B.subscribe(n, R('/camera_info'), INFO, () => {}));
          B.advertise(n, this.tn.det, 'snobots_msgs/msg/ColorDetection');
          this.log(n, 'INFO', `looking for ${this.P(n, 'target_color')} blobs in ${R('/image_raw')}`);
        },
        stop: () => { this.det = null; this.tn.det = null; } },
      ball: { pkg: 'ainex_vision', exe: 'ball_locator', node: '/ball_locator', kind: 'vision',
        params: () => [['ball_radius', 0.035, 'double', [0.01, 0.2]], ['target_frame', 'base_link', 'string']],
        start: (rec) => {
          const n = '/ball_locator', R = rec.R; this.tn.ball = R('/ball_position');
          rec.un.push(B.subscribe(n, R('/image_raw'), IMG, (m) => this.visionIn('ball', m.header)), B.subscribe(n, R('/camera_info'), INFO, () => {}), B.subscribe(n, '/tf', TF, () => {}), B.subscribe(n, '/tf_static', TF, () => {}, latched));
          B.advertise(n, this.tn.ball, 'geometry_msgs/msg/PointStamped');
          this.log(n, 'INFO', `placing the orange ball on the floor: ${this.tn.ball} in ${this.P(n, 'target_frame')}`);
        },
        stop: () => { this.tn.ball = null; } },
    });
  }
  // the walking controller: commands in, joint states, IMU, odometry and its state out
  ctlStart(rec) {
    const c = this.ctl, B = this.bus, H = this.H;
    this.tn.cmd = rec.R('/cmd_vel');
    rec.un.push(this.cmdVelSub(c, this.tn.cmd));
    rec.un.push(B.subscribe(c, '/joint_commands', 'sensor_msgs/msg/JointState', (m, from) => this.onJointCmd(m, from)));
    rec.un.push(B.subscribe(c, H.actionTopic, 'std_msgs/msg/String', (m, from) => { const r = this.playAction(String(m.data || '').trim()); if (!r.ok) this.logWarn(r.reason); else this.event('action', { name: String(m.data).trim(), from }); }));
    ['/joint_states', 'sensor_msgs/msg/JointState', '/imu', 'sensor_msgs/msg/Imu', '/odom', 'nav_msgs/msg/Odometry', '/tf', 'tf2_msgs/msg/TFMessage', H.stateTopic, 'std_msgs/msg/String'].forEach((x, i, a) => { if (i % 2 === 0) B.advertise(c, x, a[i + 1]); });
    B.service('/reset_world', 'std_srvs/srv/Empty', c, () => { this.reset(); this.event('service', '/reset_world'); return {}; });
    B.service('/sim/push', 'std_srvs/srv/Trigger', c, () => { this.push(1); this.event('service', '/sim/push'); return { success: true, message: 'pushed' }; });
    B.service(H.standSrv, 'std_srvs/srv/Trigger', c, () => { const r = this.standUp(); this.event('service', H.standSrv); return { success: r.ok, message: r.ok ? 'standing up' : r.reason }; });
    this.log(c, 'INFO', 'servo bus open on /dev/ttyAMA0: 24 servos answered');
    this.log(c, 'INFO', `walking engine ready, listening on ${this.tn.cmd}`);
  }
  ctlStop() { this.tn.cmd = null; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; }
  publish(dt) { if (this.procs.ctl) super.publish(dt); }
  logWarn(t) { this.lastWarn = t; if (this.procs.ctl) this.log(this.ctl, 'WARN', t); }
  onJointCmd(m, from) {
    super.onJointCmd(m, from);
    const nd = this.bus.nodes.get(from); if (nd && nd.user && (m.name || []).some(x => x === 'head_pan' || x === 'head_tilt')) this.vst.headT = this.t;
  }
  /* ---------- vision nodes, fed frame by frame by the camera (TF as published: see HumanoidSim.poseOf) ---------- */
  onCamFrame(hd) {
    if (this.procs.det && this.camFeeds('/color_detector')) this.visionIn('det', hd);
    if (this.procs.ball && this.camFeeds('/ball_locator')) this.visionIn('ball', hd);
  }
  // one frame reaching a vision node, straight from the camera or as a real image (through a relay, say): once per frame
  visionIn(which, hd) {
    const seen = this._seen || (this._seen = {}); if (!hd || seen[which] === hd) return; seen[which] = hd;
    if (which === 'det') { this.detectTick(hd); if (this.fix) this.fix.stats.det++; } else this.locateTick(hd);
  }
  // /color_detector: the largest blob of target_color, as this frame shows it
  detectTick(hd) {
    const Tc = this.kin.T.camera_link; if (!Tc || !this.tn.det) return;
    const n = '/color_detector', Ti = K3.inv(Tc), hf = Math.tan(this.H.hfov / 2), vf = Math.tan(this.H.camVfov * D2R / 2);
    const want = String(this.P(n, 'target_color') || 'red').toLowerCase(), minA = this.P(n, 'min_area') || 0;
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
    this.pub(n, this.tn.det, 'snobots_msgs/msg/ColorDetection', { header: { stamp: hd.stamp, frame_id: hd.frame_id }, color: best ? best.color : '', found: !!best, x: best ? +best.x.toFixed(3) : 0, y: best ? +best.y.toFixed(3) : 0, area: best ? +best.area.toFixed(5) : 0 });
    const dtF = 1 / clampN(+this.P('/usb_cam', 'framerate') || 15, 1, 30), red = !!best && best.color === 'red', mid = red && Math.abs(best.x) < 0.15 && Math.abs(best.y) < 0.2;
    if (this.trk.on && mid) { this.trk.held += dtF; if (this.trk.held >= 3) this.complete('m6'); } else this.trk.held = 0;
    if (this.fix) { const st = this.fix.stats; if (red) st.red++; if (this.trk.on && mid) st.trk += dtF; }
  }
  // /ball_locator: the ball's pixel becomes a ray, TF places the ray in the world, and the floor stops it
  locateTick(hd) {
    const n = '/ball_locator', b = this.objs.find(o => o.kind === 'ball'), To = this.kin.T.camera_optical_frame, C = AINEX_CAM; if (!b || !To || !this.tn.ball) return;
    const p = K3.apply(K3.inv(To), b.x, b.y, b.z);
    if (p[2] < 0.05 || p[2] > 3 || Math.abs(p[0] / p[2] * C.f) > C.w / 2 || Math.abs(p[1] / p[2] * C.f) > C.h / 2) return;   // not in this picture
    const fr = hd.frame_id, tgt = String(this.P(n, 'target_frame') || 'base_link'), Pc = this.lookup('odom', fr), Pt = this.lookup('odom', tgt);
    if (!Pc || !Pt) { const miss = !Pc ? fr : tgt; this.warnEvery('ball_tf', 3, n, `Could not transform the ball from '${fr || '(empty)'}' to '${tgt}': "${miss || '(empty)'}" passed to lookupTransform argument ${!Pc ? 'source_frame' : 'target_frame'} does not exist. `); return; }
    const d = K3.rot(Pc, p[0] / p[2], p[1] / p[2], 1), r = +this.P(n, 'ball_radius') || 0.035;
    if (d[2] > -1e-3) return;   // this ray never comes down to the floor
    const s = (r - Pc[11]) / d[2], q = [Pc[3] + s * d[0], Pc[7] + s * d[1], r], pt = K3.apply(K3.inv(Pt), q[0], q[1], q[2]);
    this.pub(n, this.tn.ball, 'geometry_msgs/msg/PointStamped', { header: { stamp: hd.stamp, frame_id: tgt }, point: { x: +pt[0].toFixed(4), y: +pt[1].toFixed(4), z: +pt[2].toFixed(4) } });
    if (this.fix) { const st = this.fix.stats; st.ball++; if (Math.hypot(q[0] - b.x, q[1] - b.y) < 0.05) st.ballOk++; }
  }
  /* ---------- Fix it: what the tickets measure ---------- */
  fixStats0() { return { det: 0, red: 0, trk: 0, ball: 0, ballOk: 0, walk: 0, falls: 0 }; }
  fixPos() { return [this.nav.x, this.nav.y]; }
  fixVx() { return this.v[0]; }
  fixTick(st, dt, d) { if (this.mode === 'walk' && d < 0.2) st.walk += d; }
  /* ---------- the vision missions, checked from the true state of the world ---------- */
  tagSeen(id, node) {
    const nd = this.bus.nodes.get(node); if (!nd || !nd.user || !AINEX_TAGS.some(t => t.id === id)) return;
    this.vst.ids.add(id); if (this.vst.ids.size >= 3) this.complete('v4');
  }
  ballInImage() {
    const b = this.objs.find(o => o.kind === 'ball'), Tc = this.kin.T.camera_link; if (!b || !Tc) return null;
    const p = K3.apply(K3.inv(Tc), b.x, b.y, b.z); if (p[0] < 0.05) return null;
    return [-p[1] / p[0] / Math.tan(this.H.hfov / 2), -p[2] / p[0] / Math.tan(this.H.camVfov * D2R / 2)];
  }
  visionTick(dt) {
    const V = this.vst, b = this.objs.find(o => o.kind === 'ball'); if (!b) return;
    // v2: the ball stays in the middle third of the picture while AiNex walks and your node steers the head
    const uv = this.ballInImage();
    if (uv && Math.abs(uv[0]) < 1 / 3 && Math.abs(uv[1]) < 1 / 3 && this.mode === 'walk' && this.t - V.headT < 1 && this.camRunning()) { V.watch += dt; if (V.watch >= 5) this.complete('v2'); }
    // v3: standing still with the ball 10 to 25 cm in front, your node in charge of /cmd_vel
    const n = this.nav, c = Math.cos(n.yaw), sn = Math.sin(n.yaw), dx = c * (b.x - n.x) + sn * (b.y - n.y), dy = -sn * (b.x - n.x) + c * (b.y - n.y);
    const mine = this.cmd.kind === 'user' && this.t - this.cmd.t < 3;
    V.ballRel = [dx, dy];
    V.still = this.mode === 'stand' && mine && dx >= 0.1 && dx <= 0.25 && Math.abs(dy) <= 0.08 ? V.still + dt : 0;
    if (V.still > 1) this.complete('v3');
  }
  /* ---------- the original vision demos: head tracking and nod/shake reactions on /color_detection ---------- */
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
  buildWorld() {
    const F = AINEX_FIELD, W = this.world, gy = F.goalY, gd = F.goalD, gh = F.goalH, cx = (F.x0 + F.x1) / 2;
    // boards around the pitch
    W.addBox([cx, F.y1 + 0.02, 0.025], [F.x1 - F.x0 + 0.08, 0.02, 0.05]); W.addBox([cx, F.y0 - 0.02, 0.025], [F.x1 - F.x0 + 0.08, 0.02, 0.05]); W.addBox([F.x0 - 0.02, 0, 0.025], [0.02, F.y1 - F.y0, 0.05]);
    W.addBox([F.x1 + 0.02, (gy + F.y1) / 2 + 0.01, 0.025], [0.02, F.y1 - gy, 0.05]); W.addBox([F.x1 + 0.02, -(gy + F.y1) / 2 - 0.01, 0.025], [0.02, F.y1 - gy, 0.05]);
    // goal: posts, crossbar and net
    [[F.x1, gy], [F.x1, -gy], [F.x1 + gd, gy], [F.x1 + gd, -gy]].forEach(([x, y]) => W.addCyl(x, y, 0.011, gh, { mu: 0.4, rest: 0.3 }));
    W.addBox([F.x1 + gd + 0.006, 0, gh / 2], [0.012, 2 * gy, gh], 0, { mu: 0.9 }); W.addBox([F.x1 + gd / 2, gy + 0.006, gh / 2], [gd, 0.012, gh], 0, { mu: 0.9 }); W.addBox([F.x1 + gd / 2, -gy - 0.006, gh / 2], [gd, 0.012, gh], 0, { mu: 0.9 });
    W.addBox([F.x1 + gd / 2, 0, gh + 0.006], [gd + 0.02, 2 * gy + 0.02, 0.012], 0, { float: true });   // the roof net floats: the goal mouth stays open
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
  missionTick(dt) {
    const s = this.stats;
    if (s.dist >= 1.0) this.complete('m1');
    if (s.turn >= Math.PI) this.complete('m2');
    if (s.side >= 0.3) this.complete('m3');
    if (s.cliDist >= 0.5) this.complete('m8');
    if (s.userDist >= 0.3 || s.userJoint >= 15) this.complete('m10');
    this.visionTick(dt);
    if (this.fix) SnoFix.tick(this, dt);
  }
  event(type, data) {
    const V = this.vst;
    if (type === 'action' && data === 'wave') this.complete('m4');
    else if (type === 'action' && data && typeof data === 'object' && /_kick$/.test(data.name)) { const nd = this.bus.nodes.get(data.from); if (nd && nd.user) V.kickT = this.t; }
    else if (type === 'rec_play' && data >= 2) this.complete('m5');
    else if (type === 'goal') { this.complete('m7'); if (this.t - V.kickT < 6) this.complete('v5'); }
    else if (type === 'fall') {
      this.fellOnce = true; if (this.fix) { this.fix.stats.walk = 0; this.fix.stats.falls++; }
      if (this.procs.ctl) this.log(this.ctl, 'WARN', `fall detected (body tilted ${Math.round(this.tilt() / D2R)}°): servos softened. Call ${this.H.standSrv} to get up`);
    }
    else if (type === 'stood_up' && this.fellOnce) this.complete('m9');
    else if (type === 'joint_cmd' && data.kind === 'user') this.stats.userJoint++;
    else if (type === 'cmd' && typeof data === 'string') {
      const k = data.indexOf(':'), verb = data.slice(0, k), name = data.slice(k + 1);
      if ((verb === 'hz' || verb === 'bw') && name === (this.tn.img || '/image_raw')) V.hz = true;
      if (verb === 'echo' && name === (this.tn.info || '/camera_info')) V.info = true;
      if (V.hz && V.info) this.complete('v1');
    }
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
    // ArUco tags printed on matte boards, each on a post
    AINEX_TAGS.forEach(M => {
      const code = ARUCO_4X4[M.id], cell = 16, side = M.size * 8 / 6, c = Math.cos(M.yaw), sn = Math.sin(M.yaw);
      const tx = snoTex(T, 8 * cell, 8 * cell, (g) => { g.fillStyle = '#fff'; g.fillRect(0, 0, 8 * cell, 8 * cell); g.fillStyle = '#000'; g.fillRect(cell, cell, 6 * cell, 6 * cell); g.fillStyle = '#fff'; for (let i = 0; i < 16; i++) if ((code >> (15 - i)) & 1) g.fillRect((2 + i % 4) * cell, (2 + (i >> 2)) * cell, cell, cell); });
      tx.magFilter = T.NearestFilter;
      const sign = new T.Mesh(new T.PlaneGeometry(side, side), new T.MeshStandardMaterial({ map: tx, roughness: 0.92, metalness: 0 }));
      sign.quaternion.setFromRotationMatrix(new T.Matrix4().makeBasis(new T.Vector3(-sn, c, 0), new T.Vector3(0, 0, 1), new T.Vector3(c, sn, 0)));
      sign.position.set(M.x + c * 0.004, M.y + sn * 0.004, M.z); sign.receiveShadow = true; sign.userData.marker = M.id; view.world.add(sign);
      const back = snoMesh(T, { s: 'box', size: [0.006, side + 0.01, side + 0.01], xyz: [M.x - c * 0.0005, M.y - sn * 0.0005, M.z], m: 'dark' }, mats); back.rotation.z = M.yaw; view.world.add(back);
      const post = M.z - side / 2; view.world.add(snoMesh(T, { s: 'box', size: [0.012, 0.012, post], xyz: [M.x - c * 0.01, M.y - sn * 0.01, post / 2], m: 'dark' }, mats));
    });
    this.objMesh = this.objs.map(o => {
      const g = o.kind === 'ball' ? snoMesh(T, { s: 'sph', size: [o.r], m: 'orange' }, mats) : snoMesh(T, { s: 'box', size: [o.size, o.size, o.size], m: o.color }, mats);
      if (o.kind === 'ball') { const st = new T.Mesh(new T.TorusGeometry(o.r * 0.999, o.r * 0.12, 8, 28), mats.white); g.children[0].add(st); const st2 = st.clone(); st2.rotation.y = Math.PI / 2; g.children[0].add(st2); }
      view.world.add(g); return g;
    });
    this.camCap = view.addCamera(this.model.links.camera_link, AINEX_CAM);
  }
  syncEnv() {
    if (!this.objMesh) return;
    this.objMesh.forEach((m, i) => { const S = this.objs[i].sys; m.position.set(S.pos[0], S.pos[1], S.pos[2]); m.quaternion.set(S.quat[0], S.quat[1], S.quat[2], S.quat[3]); });
  }
}

class R1Sim extends HumanoidSim {
  constructor(def) { super(def); this.loco = 'walk'; this.pdReset(); this.pdSel = 'left_shoulder_pitch_joint'; this.hold = 0; }
  gait() { return this.loco === 'run' ? this.H.run : this.H.gait; }
  reset() { this.loco = 'walk'; super.reset(); this.guestReset(); this.vst = { watch: 0, headT: -9, near: 0, dist: 0, pts: 0, hz: false, info: false }; }
  /* ---------- ROS: R1's software runs as processes that its bringup launch file starts ---------- */
  setupRos() {
    const B = this.bus, cam = '/head_camera', self = this;
    this.tn = {}; this.tfStatic = new Map();
    // the TF tree is what is actually being published; two topics are watched for the vision missions
    B.onPublish = (node, topic, type, msg) => {
      if (topic === '/tf_static' && msg && msg.transforms) this.noteStatic(node, msg.transforms);
      if (topic === '/visitor_distance' && /^std_msgs\/msg\/Float(32|64)$/.test(type) && msg) this.distSeen(+msg.data, node);
      if (topic === '/visitor_point' && type === 'geometry_msgs/msg/PointStamped' && msg) this.pointSeen(msg, node);
    };
    B.onRemove = (node) => { this.tfStatic.delete(node); };
    B.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true;
    // the person locator subscribes like any node but needs no pixels, so it doesn't make the camera render
    this.camCfg = { key: 'cam', node: cam, w: R1_CAM.w, h: R1_CAM.h, f: R1_CAM.f, fps: () => this.P(cam, 'fps'), on: () => true, frame: () => String(this.P(cam, 'frame_id') || ''), internal: new Set(['/person_locator']),
      depth: { on: () => this.P(cam, 'enable_depth') !== false, get dmin() { return +self.P(cam, 'depth_min') || R1_CAM.dmin; }, get dmax() { return +self.P(cam, 'depth_max') || R1_CAM.dmax; }, noise: R1_CAM.noise } };
    this.fm = { on: false }; this.urdfOld = false;
    this.restoreRobot({});
  }
  pkgs() { return Object.assign(super.pkgs(), { robot_state_publisher: ['robot_state_publisher'], r1_controller: ['r1_controller'], r1_camera: ['head_camera_node'], r1_vision: ['person_locator'] }); }
  // cfg works like AinexSim.restoreRobot's: <proc>: false leaves it out, { params, remaps } changes it, died: crashes at
  // start-up. joy starts the joystick's teleop node, follow starts the follow-me demo (an object: its parameters),
  // lowlevel boots R1 hanging in the gantry, oldUrdf loads a description without the camera's optical frame,
  // guest sets the visitor to 'stand' or 'stroll'.
  restoreRobot(cfg = {}) {
    const B = this.bus;
    Object.keys(this.procs).forEach(k => this.stopProc(k));
    if (this.fm.on) this.setFollow(false);
    [...B.nodes.keys()].filter(n => n === '/teleop_panel' || /^\/(static_transform_publisher_|relay)/.test(n)).forEach(n => B.removeNode(n));
    this._tele = null; this.keys = {}; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; this.warnT = {}; this.lastWarn = ''; B.clashes.clear();
    this.urdfOld = !!cfg.oldUrdf; if (cfg.guest) this.guestMode = cfg.guest;
    this.launchProcs(['rsp', 'ctl', 'cam', 'loc'], cfg);
    if (cfg.lowlevel) this.enterLow();
    if (cfg.joy) { const st = cfg.joy === 'stamped'; this.node('/teleop_panel', 'teleop', [['publish_stamped_twist', st, 'bool']]); B.advertise('/teleop_panel', '/cmd_vel', st ? 'geometry_msgs/msg/TwistStamped' : 'geometry_msgs/msg/Twist'); }
    if (cfg.follow) this.setFollow(true, typeof cfg.follow === 'object' ? cfg.follow : {});
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  procDefs() {
    if (this._pd) return this._pd;
    const B = this.bus, H = this.H, TF = 'tf2_msgs/msg/TFMessage', IMG = 'sensor_msgs/msg/Image', INFO = 'sensor_msgs/msg/CameraInfo', latched = { dur: 'TRANSIENT_LOCAL', depth: 1 }, cam = '/head_camera';
    const CQ = 'qos_overrides./head_camera/color/image_raw.publisher.reliability', DQ = 'qos_overrides./head_camera/depth/image_raw.publisher.reliability', LQ = 'qos_overrides./head_camera/depth/image_raw.subscription.reliability';
    const qosOk = (k, v) => k.startsWith('qos_overrides.') && !['reliable', 'best_effort', 'system_default'].includes(String(v)) ? `${k}: '${v}' is not a reliability (use reliable or best_effort)` : '';
    const qos = (node, k) => this.P(node, k) === 'best_effort' ? { rel: 'BEST_EFFORT', depth: 5 } : { rel: 'RELIABLE', depth: 10 };
    return (this._pd = {
      rsp: { pkg: 'robot_state_publisher', exe: 'robot_state_publisher', node: '/robot_state_publisher', kind: 'description', params: () => [['publish_frequency', 20.0, 'double', [1, 100]]],
        start: () => {
          const n = '/robot_state_publisher', js = R1_STATIC.filter(j => !(this.urdfOld && j === 'camera_optical_joint'));
          B.advertise(n, '/tf', TF); B.subscribe(n, '/joint_states', 'sensor_msgs/msg/JointState', () => {});
          B.advertise(n, '/tf_static', TF, latched); this.pub(n, '/tf_static', TF, this.tfMsg(js.map(j => { const J = this.kin.J[j]; return [J.parent, J.child, J.O]; })));
          ['base_link', 'left_hip_pitch_link', 'right_hip_pitch_link', 'waist_yaw_link', 'torso_link', 'head_link', 'camera_link'].concat(this.urdfOld ? [] : ['camera_optical_frame']).forEach(l => this.log(n, 'INFO', `got segment ${l}`));
        } },
      ctl: { pkg: 'r1_controller', exe: 'r1_controller', node: H.ctl, kind: 'sim', params: () => H.params, start: (rec) => this.ctlStart(rec), stop: () => this.ctlStop() },
      // the camera driver reads its settings once at start-up, so they are read-only here: restart it to change them
      cam: { pkg: 'r1_camera', exe: 'head_camera_node', node: cam, kind: 'driver',
        params: () => [['camera_name', 'head_camera', 'string', null, true], ['color_width', R1_CAM.w, 'integer', null, true], ['color_height', R1_CAM.h, 'integer', null, true], ['fps', 15, 'integer', [5, 30], true], ['enable_depth', true, 'bool', null, true], ['depth_min', R1_CAM.dmin, 'double', [0.1, 1], true], ['depth_max', R1_CAM.dmax, 'double', [1, 10], true], ['frame_id', 'camera_optical_frame', 'string', null, true], [CQ, 'reliable', 'string', null, true], [DQ, 'reliable', 'string', null, true]],
        check: (k, v) => qosOk(k, v) || ((k === 'color_width' && v !== R1_CAM.w) || (k === 'color_height' && v !== R1_CAM.h) ? `${k}: the simulated camera streams ${R1_CAM.w}x${R1_CAM.h} only` : ''),
        crash: "terminate called after throwing an instance of 'std::runtime_error'  what():  head camera: no stereo device on USB (expected 2 video streams, found 0)",
        start: (rec) => {
          const n = cam, R = rec.R, dep = this.P(n, 'enable_depth') !== false; this.camLast = null; this.camDue = null;
          this.tn.img = R('/head_camera/color/image_raw'); this.tn.info = R('/head_camera/color/camera_info'); this.tn.dimg = R('/head_camera/depth/image_raw'); this.tn.dinfo = R('/head_camera/depth/camera_info');
          B.advertise(n, this.tn.img, IMG, qos(n, CQ)); B.advertise(n, this.tn.info, INFO);
          if (dep) { B.advertise(n, this.tn.dimg, IMG, qos(n, DQ)); B.advertise(n, this.tn.dinfo, INFO); }
          this.log(n, 'INFO', `opened the binocular camera: colour ${R1_CAM.w}x${R1_CAM.h} rgb8 on ${this.tn.img}`);
          this.log(n, 'INFO', dep ? `depth ${R1_CAM.w}x${R1_CAM.h} 16UC1 (millimetres, aligned to colour) on ${this.tn.dimg}, range ${this.P(n, 'depth_min')} to ${this.P(n, 'depth_max')} m` : 'depth is disabled (enable_depth: false)');
          this.log(n, 'INFO', `streaming at ${this.P(n, 'fps')} fps, frame_id ${this.P(n, 'frame_id')}`);
        },
        stop: () => { this.tn.img = this.tn.info = this.tn.dimg = this.tn.dinfo = null; this.camDue = null; } },
      // finds people in the colour image, reads their distance from the depth image and places them with TF
      loc: { pkg: 'r1_vision', exe: 'person_locator', node: '/person_locator', kind: 'vision',
        params: () => [['depth_scale', 0.001, 'double', [0, 10]], ['target_frame', 'base_link', 'string'], ['max_range', 5.0, 'double', [0.5, 10]], [LQ, 'reliable', 'string', null, true]],
        check: (k, v) => qosOk(k, v),
        start: (rec) => {
          const n = '/person_locator', R = rec.R; this.tn.person = R('/person_position'); this._seen = {}; this.locDT = this.t;
          rec.un.push(B.subscribe(n, R('/head_camera/color/image_raw'), IMG, (m) => this.locIn('c', m.header)), B.subscribe(n, R('/head_camera/depth/image_raw'), IMG, (m) => this.locIn('d', m.header), qos(n, LQ)),
            B.subscribe(n, R('/head_camera/color/camera_info'), INFO, () => {}), B.subscribe(n, '/tf', TF, () => {}), B.subscribe(n, '/tf_static', TF, () => {}, latched));
          B.advertise(n, this.tn.person, 'geometry_msgs/msg/PointStamped');
          this.log(n, 'INFO', `looking for people in ${R('/head_camera/color/image_raw')} with depth from ${R('/head_camera/depth/image_raw')}`);
          this.log(n, 'INFO', `publishing ${this.tn.person} in ${this.P(n, 'target_frame')} (depth_scale ${this.P(n, 'depth_scale')})`);
        },
        stop: () => { this.tn.person = null; } },
    });
  }
  // the motion controller bridge: commands in; joint states, IMU, odometry, the state machine and /lowstate out
  ctlStart(rec) {
    const c = this.ctl, B = this.bus, H = this.H;
    this.tn.cmd = rec.R('/cmd_vel');
    rec.un.push(this.cmdVelSub(c, this.tn.cmd));
    rec.un.push(B.subscribe(c, '/joint_commands', 'sensor_msgs/msg/JointState', (m, from) => this.onJointCmd(m, from)));
    rec.un.push(B.subscribe(c, H.actionTopic, 'std_msgs/msg/String', (m, from) => { const r = this.playAction(String(m.data || '').trim()); if (!r.ok) this.logWarn(r.reason); else this.event('action', { name: String(m.data).trim(), from }); }));
    rec.un.push(B.subscribe(c, '/lowcmd', 'snobots_msgs/msg/LowCmd', (m, from) => this.onLowCmd(m, from)));
    ['/joint_states', 'sensor_msgs/msg/JointState', '/imu', 'sensor_msgs/msg/Imu', '/odom', 'nav_msgs/msg/Odometry', '/tf', 'tf2_msgs/msg/TFMessage', H.stateTopic, 'std_msgs/msg/String', '/lowstate', 'snobots_msgs/msg/LowState'].forEach((x, i, a) => { if (i % 2 === 0) B.advertise(c, x, a[i + 1]); });
    B.service('/reset_world', 'std_srvs/srv/Empty', c, () => { this.reset(); this.event('service', '/reset_world'); return {}; });
    B.service('/sim/push', 'std_srvs/srv/Trigger', c, () => { this.push(1); this.event('service', '/sim/push'); return { success: true, message: 'pushed' }; });
    B.service(H.standSrv, 'std_srvs/srv/Trigger', c, () => { const r = this.standUp(); this.event('service', H.standSrv); return { success: r.ok, message: r.ok ? 'standing up' : r.reason }; });
    B.service('/loco/set_mode', 'snobots_msgs/srv/SetMode', c, (rq) => { const r = this.setMode(String(rq.mode || '').trim().toLowerCase()); this.event('service', '/loco/set_mode'); return { success: !!r.ok, message: r.ok ? `mode is now ${this.locoStateRaw()}` : r.reason }; });
    this.log(c, 'INFO', 'connected to the motion controller: 26 joint motors answered');
    this.log(c, 'INFO', `locomotion state ${this.locoStateRaw()}, listening on ${this.tn.cmd}`);
  }
  ctlStop() { this.tn.cmd = null; this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' }; }
  publish(dt) { if (this.procs.ctl) super.publish(dt); }
  logWarn(t) { this.lastWarn = t; if (this.procs.ctl) this.log(this.ctl, 'WARN', t); }
  onJointCmd(m, from) {
    super.onJointCmd(m, from);
    const nd = this.bus.nodes.get(from); if (nd && nd.user && (m.name || []).some(x => x === 'head_yaw_joint' || x === 'head_pitch_joint')) this.vst.headT = this.t;
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
  /* ---------- the visitor ---------- */
  guestReset() { const [x, y, yaw] = R1_VISITOR.start; this.guest = { x, y, yaw, wp: 0, wait: 0, ph: 0, v: 0 }; if (!this.guestMode) this.guestMode = 'stand'; }
  guestTick(dt) {
    const g = this.guest, V = R1_VISITOR; if (!g) return;
    const rx = this.nav.x - g.x, ry = this.nav.y - g.y, rd = Math.hypot(rx, ry), close = rd < 0.6 && this.mode !== 'low';
    let vx = 0, vy = 0;
    if (close) { const k = 0.35 / (rd || 1); vx = -rx * k; vy = -ry * k; }   // a robot this close: step back
    else if (this.guestMode === 'stroll') {
      if (g.wait > 0) g.wait -= dt;
      else { const [tx, ty] = V.path[g.wp], dx = tx - g.x, dy = ty - g.y, d = Math.hypot(dx, dy); if (d < 0.05) { g.wp = (g.wp + 1) % V.path.length; g.wait = V.pause; } else { const s = Math.min(V.speed, d / dt); vx = dx / d * s; vy = dy / d * s; } }
    }
    g.x += vx * dt; g.y += vy * dt; g.v = Math.hypot(vx, vy); g.ph += g.v * dt / 0.32 * Math.PI;
    // face the way you walk; standing still (or backing off), look at the robot
    const want = g.v > 0.02 && !close ? Math.atan2(vy, vx) : Math.atan2(ry, rx);
    g.yaw += clampN(wrapPi(want - g.yaw), -2.5 * dt, 2.5 * dt);
  }
  // the point of the vest that faces the camera (on the torso's surface, at mid-vest height)
  guestSurface() {
    const g = this.guest, V = R1_VISITOR, Tc = this.kin.T.camera_link, dx = Tc[3] - g.x, dy = Tc[7] - g.y, d = Math.hypot(dx, dy) || 1;
    return [g.x + dx / d * V.r, g.y + dy / d * V.r, (V.vest[0] + V.vest[1]) / 2];
  }
  // where the vest's centre lands in the picture, in [-1, 1] (x right, y down); null when it is not in view
  guestUV() {
    const To = this.kin.T.camera_optical_frame, C = R1_CAM, S = this.guestSurface(); if (!To) return null;
    const p = K3.apply(K3.inv(To), S[0], S[1], S[2]); if (p[2] < 0.1) return null;
    const u = p[0] / p[2] * C.f / (C.w / 2), v = p[1] / p[2] * C.f / (C.h / 2);
    return Math.abs(u) <= 1 && Math.abs(v) <= 1 ? [u, v, p[2]] : null;
  }
  /* ---------- /person_locator, fed frame by frame by the camera ---------- */
  onCamFrame(hd) {
    if (!this.procs.loc) return;
    if (this.camFeeds('/person_locator', 'img')) this.locIn('c', hd);
    if (this.camFeeds('/person_locator', 'dimg')) this.locIn('d', hd);
  }
  // half of a frame (colour or depth) reaching the locator, straight from the camera or as a real image through a relay;
  // like a message_filters synchronizer it works once both halves of the same frame have arrived
  locIn(which, hd) {
    const s = this._seen || (this._seen = {}); if (!hd || s[which] === hd) return; s[which] = hd;
    if (which === 'd') this.locDT = this.t;
    else {
      if (this.fix) this.fix.stats.locC++;
      if (this.t - this.locDT > 3) this.warnEvery('loc_depth', 5, '/person_locator', `No depth image on '${this.procs.loc.R('/head_camera/depth/image_raw')}' for ${Math.round(this.t - this.locDT)} s: colour frames arrive, but nobody can be placed without depth`);
    }
    if (s.c === s.d) this.locateTick(hd);
  }
  // the person's pixel and its depth become a 3D point in the image's frame, and TF moves it to target_frame
  locateTick(hd) {
    const n = '/person_locator'; if (!this.tn.person) return;
    const uv = this.guestUV(); if (!uv) return;
    const S = this.guestSurface(), p = K3.apply(K3.inv(this.kin.T.camera_optical_frame), S[0], S[1], S[2]), D = this.camCfg.depth;
    if (p[2] < D.dmin || p[2] > D.dmax) return;   // no depth there
    const z = Math.round(p[2] * 1000) * (+this.P(n, 'depth_scale')), maxR = +this.P(n, 'max_range') || 5;
    if (!(z > 0) || z > maxR) { this.warnEvery('loc_range', 4, n, `person seen ${z >= 100 ? Math.round(z) : z.toFixed(2)} m away, beyond max_range (${maxR} m): ignored`); return; }
    const q = [p[0] / p[2] * z, p[1] / p[2] * z, z];   // back-projected along the pixel's ray: x right, y down, z forward
    const fr = hd.frame_id, tgt = String(this.P(n, 'target_frame') || 'base_link'), M = this.lookup(tgt, fr);
    if (!M) { const miss = !this.poseOf(fr) ? fr : tgt; this.warnEvery('loc_tf', 3, n, `Could not transform the person from '${fr || '(empty)'}' to '${tgt}': "${miss || '(empty)'}" passed to lookupTransform argument ${miss === fr ? 'source_frame' : 'target_frame'} does not exist. `); return; }
    const pt = K3.apply(M, q[0], q[1], q[2]);
    this.pub(n, this.tn.person, 'geometry_msgs/msg/PointStamped', { header: { stamp: hd.stamp, frame_id: tgt }, point: { x: +pt[0].toFixed(4), y: +pt[1].toFixed(4), z: +pt[2].toFixed(4) } });
    if (this.fix) { const st = this.fix.stats, Tt = tgt === 'odom' ? K3.I() : this.kin.T[tgt]; st.loc++; if (Tt) { const w = K3.apply(Tt, pt[0], pt[1], pt[2]); if (Math.hypot(w[0] - S[0], w[1] - S[1], w[2] - S[2]) < 0.1) st.locOk++; } }
  }
  /* ---------- the follow-me demo: /person_position in, head and walking commands out ---------- */
  behaviors() {
    return { 'follow_me.launch.py': { label: 'follow me', nodes: ['/follow_me'], on: () => this.fm.on, set: (on, args) => this.setFollow(on, args), hello: '[follow_me-1] [INFO] following the person on /person_position: the head looks, the feet keep follow_distance' } };
  }
  setFollow(on, o = {}) {
    if (on === this.fm.on) return;
    const B = this.bus, n = '/follow_me';
    if (on) {
      const p = (k, v) => o[k] != null && Number.isFinite(+o[k]) ? +o[k] : v;
      this.node(n, 'behavior', [['follow_distance', p('follow_distance', 1.0), 'double', [0.5, 3]], ['head_kp', p('head_kp', 0.6), 'double', [-2, 2]], ['max_speed', p('max_speed', 0.3), 'double', [0.05, 0.45]]]);
      const cv = o.remap ? String(o.remap) : '/cmd_vel';   // remap: where its walking commands go (cmd_vel:=…)
      B.advertise(n, cv, 'geometry_msgs/msg/Twist'); B.advertise(n, '/joint_commands', 'sensor_msgs/msg/JointState');
      this.fm = { on: true, cv, last: -9, lost: true, scan: 0, un: B.subscribe(n, '/person_position', 'geometry_msgs/msg/PointStamped', (m) => this.onPerson(m)) };
    } else { if (this.fm.un) this.fm.un(); B.removeNode(n); this.fm = { on: false }; }
    SnoUI.refreshTab('control');
  }
  onPerson(m) {
    const F = this.fm, n = '/follow_me'; if (!F.on || !m || !m.point) return;
    const M = this.lookup('base_link', (m.header && m.header.frame_id) || 'base_link'); if (!M) return;
    const p = K3.apply(M, +m.point.x || 0, +m.point.y || 0, +m.point.z || 0); F.last = this.t; F.lost = false;
    // the head turns toward the person, from where the camera looks now
    const Mc = this.lookup('camera_link', 'base_link');
    if (Mc) {
      const c = K3.apply(Mc, p[0], p[1], p[2]), k = +this.P(n, 'head_kp'), J = this.kin.J;
      const yaw = clampN(this.q.head_yaw_joint + k * Math.atan2(c[1], c[0]), J.head_yaw_joint.lo, J.head_yaw_joint.hi), pitch = clampN(this.q.head_pitch_joint - k * Math.atan2(c[2], c[0]), J.head_pitch_joint.lo, J.head_pitch_joint.hi);
      this.pub(n, '/joint_commands', 'sensor_msgs/msg/JointState', { header: { stamp: stampNow(), frame_id: '' }, name: ['head_yaw_joint', 'head_pitch_joint'], position: [+yaw.toFixed(4), +pitch.toFixed(4)], velocity: [], effort: [] });
    }
    // the feet: turn to face the person, then walk until follow_distance is left
    const d = Math.hypot(p[0], p[1]), b = Math.atan2(p[1], p[0]), D = +this.P(n, 'follow_distance'), vmax = +this.P(n, 'max_speed');
    let vx = Math.abs(b) < 0.35 ? clampN(0.8 * (d - D), -0.1, vmax) : 0, wz = clampN(1.2 * b, -0.8, 0.8);
    if (Math.abs(d - D) < 0.08) vx = 0; if (Math.abs(b) < 0.08) wz = 0;
    this.pub(n, F.cv, 'geometry_msgs/msg/Twist', { linear: { x: +vx.toFixed(3), y: 0, z: 0 }, angular: { x: 0, y: 0, z: +wz.toFixed(3) } });
  }
  // nobody seen for a while: stop walking, then bring the head back to the middle bit by bit
  behaviorsTick(dt) {
    const F = this.fm; if (!F || !F.on) return;
    if (!F.lost && this.t - F.last > 0.6) { F.lost = true; this.pub('/follow_me', F.cv, 'geometry_msgs/msg/Twist', { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }); }
    if (F.lost && this.t - F.last > 1.2 && (F.scan -= dt) <= 0) {
      F.scan = 0.2; const y = this.q.head_yaw_joint, pt = this.q.head_pitch_joint;
      if (Math.abs(y) > 0.02 || Math.abs(pt) > 0.02) this.pub('/follow_me', '/joint_commands', 'sensor_msgs/msg/JointState', { header: { stamp: stampNow(), frame_id: '' }, name: ['head_yaw_joint', 'head_pitch_joint'], position: [+(y - clampN(y, -0.1, 0.1)).toFixed(4), +(pt - clampN(pt, -0.06, 0.06)).toFixed(4)], velocity: [], effort: [] });
    }
  }
  /* ---------- Fix it: what the tickets measure ---------- */
  fixStats0() { return { locC: 0, loc: 0, locOk: 0, head: 0, headRun: 0, fm: 0, falls: 0 }; }
  fixPos() { return [this.nav.x, this.nav.y]; }
  fixVx() { return this.v[0]; }
  fixWalking() { return this.mode === 'stand' || this.mode === 'walk'; }
  fixTick(st, dt, d) {
    const src = this.cmd.src || '';
    if (this.fm.on && this.fixWalking() && this.cmdLive(0.5) && (src === '/follow_me' || src.startsWith('/relay')) && d < 0.5) st.fm += d;
    const uv = this.fm.on ? this.guestUV() : null;
    if (uv && Math.abs(uv[0]) < 1 / 3 && Math.abs(uv[1]) < 1 / 3) { st.headRun += dt; st.head = Math.max(st.head, st.headRun); } else st.headRun = 0;
  }
  /* ---------- the vision missions, checked from the true state of the world ---------- */
  distSeen(z, node) {
    const nd = this.bus.nodes.get(node), uv = this.guestUV(); if (!nd || !nd.user || !uv || !Number.isFinite(z)) return;
    if (Math.abs(z - uv[2]) <= Math.max(0.1, 0.05 * uv[2])) { this.vst.dist++; if (this.vst.dist >= 10) this.complete('v2'); }
  }
  pointSeen(m, node) {
    const nd = this.bus.nodes.get(node); if (!nd || !nd.user || !m.point) return;
    const M = this.lookup('odom', (m.header && m.header.frame_id) || ''); if (!M) return;
    const w = K3.apply(M, +m.point.x || 0, +m.point.y || 0, +m.point.z || 0), g = this.guest, V = R1_VISITOR;
    if (Math.hypot(w[0] - g.x, w[1] - g.y) <= V.r + 0.12 && Math.abs(w[2] - (V.vest[0] + V.vest[1]) / 2) <= 0.25) { this.vst.pts++; if (this.vst.pts >= 10) this.complete('v5'); }
  }
  visionTick(dt) {
    const V = this.vst, uv = this.guestUV(), g = this.guest;
    // v3: the visitor strolls and stays in the middle third of the picture because your node steers the head
    if (uv && Math.abs(uv[0]) < 1 / 3 && Math.abs(uv[1]) < 1 / 3 && this.guestMode === 'stroll' && this.t - V.headT < 1 && this.camRunning()) { V.watch += dt; if (V.watch >= 5) this.complete('v3'); }
    // v4: 0.7 to 1.4 m from the strolling visitor, with your node in charge of /cmd_vel
    const d = Math.hypot(g.x - this.nav.x, g.y - this.nav.y), mine = this.cmd.kind === 'user' && this.t - this.cmd.t < 1;
    if (this.guestMode === 'stroll' && mine && d >= 0.7 && d <= 1.4 && this.canMove()) { V.near += dt; if (V.near >= 10) this.complete('v4'); }
  }
  objStep(dt) { super.objStep(dt); if (dt > 0) this.guestTick(dt); }
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
  hudExtra() { const g = this.guest, d = g ? Math.hypot(g.x - this.nav.x, g.y - this.nav.y) : 0; return `<span>mode ${esc(this.locoStateRaw())}</span><span>visitor ${d.toFixed(1)} m</span>`; }
  extraControls(w) {
    w.append(h('h4', { html: 'Vision <span>head camera → <code>/person_position</code></span>' }));
    w.append(snoSwitch('Follow me', this.fm.on, (on) => { this.setFollow(on); this.event('behavior', { k: 'follow', on }); }, 'Node <code>/follow_me</code>: R1 looks at the visitor and keeps about 1 m away.'));
    w.append(h('div', { class: 'seg-label', html: 'The visitor' }), snoSeg([['stand', 'Stand'], ['stroll', 'Stroll']], this.guestMode, (v) => { this.guestMode = v; this.event('guest', v); SnoUI.refreshTab('control'); }, 'The visitor'));
    const rb = h('button', { class: 'btn btn-sm', html: icon('replay') + 'Reset visitor' }); rb.addEventListener('click', () => this.guestReset());
    w.append(h('div', { class: 'row-btns', style: 'margin-top:10px' }, rb));
  }
  missionTick(dt) {
    const s = this.stats;
    if (s.dist >= 2) this.complete('m1');
    if (this.loco === 'run' && this.v[0] >= 0.45) this.complete('m2');
    if (s.cliDist >= 1) this.complete('m8');
    if (s.userDist >= 1) this.complete('m9');
    if (this.pushWatch && this.t > this.pushWatch) { this.pushWatch = 0; if (['stand', 'walk', 'act', 'manual'].includes(this.mode)) this.complete('m4'); }
    this.visionTick(dt);
    if (this.fix) SnoFix.tick(this, dt);
  }
  event(type, data) {
    const V = this.vst;
    if (type === 'action_done' && ['dance', 'kungfu', 'wave', 'handshake', 'bow'].includes(data)) this.complete('m3');
    else if (type === 'push' && this.P(this.ctl, 'balance') !== false && data < 2) this.pushWatch = this.t + 2.5;
    else if (type === 'fall') { this.pushWatch = 0; if (this.fix) this.fix.stats.falls++; }
    else if (type === 'damp') this.damped = true;
    else if (type === 'stood_up' && this.damped) this.complete('m5');
    else if (type === 'service' && data === '/loco/set_mode') this.complete('m6');
    else if (type === 'cmd_vel' && data && (data.vx || data.vy || data.wz) && ['low', 'damp', 'fall', 'fallen', 'getup', 'manual'].includes(this.mode) && this.procs.ctl) this.warnEvery('mode', 4, this.ctl, `Ignoring ${this.tn.cmd || '/cmd_vel'}: R1 is in '${this.locoStateRaw()}' mode, and only walks in stand, walk or run mode. Change it with /loco/set_mode.`);
    else if (type === 'cmd' && typeof data === 'string') {
      const k = data.indexOf(':'), verb = data.slice(0, k), name = data.slice(k + 1);
      if ((verb === 'hz' || verb === 'bw') && (name === this.tn.img || name === this.tn.dimg)) V.hz = true;
      if (verb === 'echo' && (name === this.tn.info || name === this.tn.dinfo)) V.info = true;
      if (V.hz && V.info) this.complete('v1');
    }
  }
  buildEnv(view, T) {
    const mats = view.mats;
    for (let i = 1; i <= 6; i++) for (const y of [-1.3, 1.3]) { const c = new T.Group(); c.add(snoMesh(T, { s: 'cone', size: [0.012, 0.075, 0.32], xyz: [0, 0, 0.16], m: 'orange' }, mats)); c.add(snoMesh(T, { s: 'box', size: [0.2, 0.2, 0.025], xyz: [0, 0, 0.0125], m: 'dark' }, mats)); c.add(snoMesh(T, { s: 'cyl', size: [0.052, 0.04], xyz: [0, 0, 0.17], m: 'white' }, mats)); c.position.set(i * 1.5, y, 0); view.world.add(c); }
    const line = (x, col) => view.world.add(snoMesh(T, { s: 'box', size: [0.07, 2.4, 0.004], xyz: [x, 0, 0.002], m: col }, mats));
    line(0, 'white'); line(3, 'yellow'); line(6, 'white');
    const pad = new T.Mesh(new T.CircleGeometry(0.6, 48), new T.MeshStandardMaterial({ color: new T.Color('#30A46C').convertSRGBToLinear(), transparent: true, opacity: 0.55, roughness: 1 })); pad.position.set(4.5, 3, 0.003); view.world.add(pad);
    // the visitor, a mannequin: legs and arms swing from hips and shoulders as it walks
    const vest = new T.MeshStandardMaterial({ color: new T.Color(R1_VISITOR.color).convertSRGBToLinear(), roughness: 0.8, metalness: 0 }), V = R1_VISITOR, g = new T.Group(), m = mats;
    const limb = (x, y, z, parts) => { const p = new T.Group(); p.position.set(x, y, z); parts.forEach(v => p.add(snoMesh(T, v, m))); g.add(p); return p; };
    this.guestLegs = [0.09, -0.09].map(y => limb(0, y, 0.88, [{ s: 'cap', size: [0.065, 0.66], ax: 'z', xyz: [0, 0, -0.4], m: 'navy' }, { s: 'box', size: [0.24, 0.1, 0.09], xyz: [0.05, 0, -0.835], m: 'dark' }]));
    g.add(snoMesh(T, { s: 'box', size: [0.24, 0.32, 0.16], xyz: [0, 0, 0.92], m: 'navy' }, m));
    g.add(snoMesh(T, { s: 'cyl', size: [V.r, V.vest[1] - V.vest[0]], ax: 'z', xyz: [0, 0, (V.vest[0] + V.vest[1]) / 2], m: vest }, m));
    g.add(snoMesh(T, { s: 'cap', size: [0.075, 0.3], ax: 'y', xyz: [0, 0, 1.39], m: vest }, m));
    this.guestArms = [0.215, -0.215].map(y => limb(0, y, 1.39, [{ s: 'cap', size: [0.045, 0.5], ax: 'z', xyz: [0, 0, -0.3], m: 'grey' }]));
    g.add(snoMesh(T, { s: 'cyl', size: [0.045, 0.1], ax: 'z', xyz: [0, 0, 1.48], m: 'grey' }, m));
    g.add(snoMesh(T, { s: 'sph', size: [0.105], xyz: [0, 0, 1.6], m: 'grey' }, m));
    g.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    view.world.add(g); this.guestMesh = g;
    this.camCap = view.addCamera(this.model.links.camera_link, R1_CAM);
  }
  syncEnv() {
    const g = this.guest, M = this.guestMesh; if (!g || !M) return;
    M.position.set(g.x, g.y, 0); M.rotation.z = g.yaw;
    const a = Math.sin(g.ph) * Math.min(1, g.v / 0.15) * 0.42;
    this.guestLegs[0].rotation.y = a; this.guestLegs[1].rotation.y = -a; this.guestArms[0].rotation.y = -a * 0.8; this.guestArms[1].rotation.y = a * 0.8;
  }
}
