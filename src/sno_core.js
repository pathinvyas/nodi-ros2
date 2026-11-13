/* ============================== SnoBots: 3D view, page shell and shared ROS plumbing ============================== */
const SNO_IDS = ['ainex', 'r1', 'limo', 'cobot'];
const SNO_THREE = 'https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js';
const SnoThree = {
  p: null,
  load() {
    if (window.THREE) return Promise.resolve(window.THREE);
    if (this.p) return this.p;
    this.p = new Promise((res, rej) => {
      const s = document.createElement('script'); s.src = window.SNO_THREE_URL || SNO_THREE; s.async = true;
      const to = setTimeout(() => { this.p = null; rej(new Error('timeout')); }, 25000);
      s.onload = () => { clearTimeout(to); if (window.THREE) res(window.THREE); else { this.p = null; rej(new Error('missing')); } };
      s.onerror = () => { clearTimeout(to); this.p = null; s.remove(); rej(new Error('blocked')); };
      document.head.append(s);
    });
    return this.p;
  },
};

/* materials: [sRGB colour, roughness, metalness, emissive?] */
const SNO_MAT = {
  shell: ['#E9EEF0', 0.5, 0.05], white: ['#F6F8F9', 0.45, 0.02], grey: ['#8E989E', 0.55, 0.2], dark: ['#2B3236', 0.55, 0.25], servo: ['#1E2326', 0.45, 0.15],
  metal: ['#B9C2C8', 0.32, 0.7], alu: ['#CDD4D8', 0.38, 0.55], accent: ['#F26419', 0.5, 0.05], rubber: ['#16191B', 0.95, 0], glass: ['#0B1820', 0.12, 0.5],
  glow: ['#5EF2D6', 0.4, 0, '#2BBFA6'], blue: ['#2F6FD0', 0.45, 0.1], red: ['#E5484D', 0.5, 0.05], green: ['#30A46C', 0.5, 0.05], yellow: ['#F5B700', 0.5, 0.05],
  orange: ['#FF8A3D', 0.5, 0.05], wood: ['#C9A27A', 0.8, 0], navy: ['#24324A', 0.55, 0.2], copper: ['#C9773B', 0.4, 0.6],
};
function snoMats(T) {
  const out = {};
  for (const [k, [c, r, m, e]] of Object.entries(SNO_MAT)) {
    const mat = new T.MeshStandardMaterial({ color: new T.Color(c).convertSRGBToLinear(), roughness: r, metalness: m });
    if (e) { mat.emissive = new T.Color(e).convertSRGBToLinear(); mat.emissiveIntensity = 0.9; }
    out[k] = mat;
  }
  return out;
}
const SNO_SOFT = new Set(['shell', 'white', 'accent', 'navy', 'grey', 'dark', 'blue', 'red', 'green', 'yellow', 'orange']);
// box with rounded edges: an odd-segmented unit box whose vertices are pushed out from an inner box by the radius
function snoRoundBox(T, w, h, d, r, seg = 3) {
  const n = seg * 2 + 1, g = new T.BoxGeometry(1, 1, 1, n, n, n), P = g.attributes.position, N = g.attributes.normal, half = 0.5 / n;
  const bx = w / 2 - r, by = h / 2 - r, bz = d / 2 - r;
  for (let i = 0; i < P.count; i++) {
    const x = P.getX(i), y = P.getY(i), z = P.getZ(i);
    let nx = x - Math.sign(x) * half, ny = y - Math.sign(y) * half, nz = z - Math.sign(z) * half; const L = Math.hypot(nx, ny, nz) || 1; nx /= L; ny /= L; nz /= L;
    P.setXYZ(i, bx * Math.sign(x) + nx * r, by * Math.sign(y) + ny * r, bz * Math.sign(z) + nz * r); N.setXYZ(i, nx, ny, nz);
  }
  g.computeBoundingSphere(); return g;
}
const _snoGeo = new Map();
function snoGeo(T, key, make) { let g = _snoGeo.get(key); if (!g || g._T !== T) { g = make(); g._T = T; _snoGeo.set(key, g); } return g; }
/* One visual: { s: 'box'|'cyl'|'sph'|'cap'|'cone', size, xyz, rpy, m, ax: 'x'|'y'|'z' (cylinder axis) } */
function snoMesh(T, v, mats) {
  const m = typeof v.m === 'object' ? v.m : (mats[v.m || 'shell'] || mats.shell);
  const r = v.size, k = v.s + ':' + r.map(x => x.toFixed(4)).join(',');
  let obj;
  if (v.s === 'box') {
    const rr = v.round != null ? v.round : (SNO_SOFT.has(v.m) ? Math.min(0.012, Math.min(r[0], r[1], r[2]) * 0.22) : 0);
    obj = new T.Mesh(snoGeo(T, k + ':' + rr.toFixed(4), () => rr > 0.0015 ? snoRoundBox(T, r[0], r[1], r[2], rr) : new T.BoxGeometry(r[0], r[1], r[2])), m);
  }
  else if (v.s === 'sph') obj = new T.Mesh(snoGeo(T, k, () => new T.SphereGeometry(r[0], 22, 16)), m);
  else if (v.s === 'cyl' || v.s === 'cone') obj = new T.Mesh(snoGeo(T, k, () => v.s === 'cone' ? new T.CylinderGeometry(r[0], r[1], r[2], 28) : new T.CylinderGeometry(r[0], r[0], r[1], 28)), m);
  else if (v.s === 'cap') {
    obj = new T.Group();
    const c = new T.Mesh(snoGeo(T, 'cyl:' + r[0].toFixed(4) + ',' + r[1].toFixed(4), () => new T.CylinderGeometry(r[0], r[0], r[1], 24)), m);
    const sg = snoGeo(T, 'sph:' + r[0].toFixed(4), () => new T.SphereGeometry(r[0], 22, 14));
    const a = new T.Mesh(sg, m), b = new T.Mesh(sg, m); a.position.y = r[1] / 2; b.position.y = -r[1] / 2;
    [c, a, b].forEach(x => { x.castShadow = true; x.receiveShadow = true; obj.add(x); });
  }
  if (v.s === 'cyl' || v.s === 'cone' || v.s === 'cap') {
    const ax = v.ax || 'z';
    const inner = obj; obj = new T.Group(); obj.add(inner);
    if (ax === 'z') inner.rotation.x = Math.PI / 2; else if (ax === 'x') inner.rotation.z = -Math.PI / 2;
  }
  obj.castShadow = true; obj.receiveShadow = true;
  const p = v.xyz || [0, 0, 0], e = v.rpy || [0, 0, 0];
  const wrap = new T.Group(); wrap.position.set(p[0], p[1], p[2]); wrap.rotation.set(e[0], e[1], e[2], 'ZYX'); wrap.add(obj);
  if (v.name) wrap.name = v.name;
  wrap.traverse(o => { if (o.isMesh) { o.castShadow = v.shadow !== false; o.receiveShadow = true; } });
  return wrap;
}
/* Build a three.js hierarchy that mirrors a KinRobot. visuals: { linkName: [visual, ...] } */
function snoBuildModel(T, kin, visuals, mats) {
  const links = {}, joints = [];
  const root = new T.Group(); root.matrixAutoUpdate = false; root.userData.link = kin.root; links[kin.root] = root;
  (visuals[kin.root] || []).forEach(v => root.add(snoMesh(T, v, mats)));
  for (const j of kin.order) {
    const jg = new T.Group(), xyz = j.xyz || [0, 0, 0], rpy = j.rpy || [0, 0, 0];
    jg.position.set(xyz[0], xyz[1], xyz[2]); jg.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
    const rg = new T.Group(); rg.userData.link = j.child; jg.add(rg); links[j.parent].add(jg); links[j.child] = rg;
    (visuals[j.child] || []).forEach(v => rg.add(snoMesh(T, v, mats)));
    if (j.type !== 'fixed') joints.push({ g: rg, ax: new T.Vector3(j.ax[0], j.ax[1], j.ax[2]), name: j.name, offset: j.offset || 0, type: j.type });
  }
  const setMat = (g, B) => { g.matrix.set(B[0], B[1], B[2], B[3], B[4], B[5], B[6], B[7], B[8], B[9], B[10], B[11], 0, 0, 0, 1); g.matrixWorldNeedsUpdate = true; };
  return {
    root, links, joints,
    pose(q, B) { if (B) setMat(root, B); for (const j of joints) { const v = q[j.name] || 0; if (j.type === 'prismatic') j.g.position.copy(j.ax).multiplyScalar(v); else j.g.quaternion.setFromAxisAngle(j.ax, v + j.offset); } },
  };
}

/* ------------------------------ 3D viewport with orbit camera and optional robot camera (picture in picture) ------------------------------ */
class SnoView {
  constructor(wrap, o) {
    this.wrap = wrap; this.o = Object.assign({ dist: 1.4, az: -2.4, el: 0.42, target: [0, 0, 0.2], minDist: 0.25, maxDist: 14, shadow: 1.6, grid: [6, 60], fog: [6, 24], ground: true }, o);
    this.tgt = this.o.target.slice(); this.dist = this.o.dist; this.az = this.o.az; this.el = this.o.el; this.follow = true; this.ready = false;
    this.pips = []; this.pipIdx = 0; this.pipOn = this.o.pipOn !== false; this.ptrs = new Map(); this.onClick = null; this.followPos = null; this.cams = [];
  }
  init(T) {
    this.T = T;
    let r;
    try { r = new T.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' }); } catch (e) { return false; }
    if (!r || !r.getContext()) return false;
    this.r = r; r.setPixelRatio(Math.min(2, devicePixelRatio || 1)); r.outputEncoding = T.sRGBEncoding;
    r.shadowMap.enabled = true; r.shadowMap.type = T.PCFSoftShadowMap;
    this.cv = r.domElement; this.cv.className = 'sno-canvas'; this.cv.setAttribute('aria-hidden', 'true');
    this.wrap.prepend(this.cv);
    const S3 = this.scene = new T.Scene();
    this.cam = new T.PerspectiveCamera(48, 1.4, 0.01, 120);
    this.world = new T.Group(); this.world.rotation.x = -Math.PI / 2; S3.add(this.world);
    this.mats = snoMats(T);
    this.hemi = new T.HemisphereLight(0xffffff, 0x8d9894, 0.62); S3.add(this.hemi);
    const sun = this.sun = new T.DirectionalLight(0xffffff, 0.95); sun.castShadow = true;
    const sb = this.o.shadow; sun.shadow.mapSize.set(2048, 2048); Object.assign(sun.shadow.camera, { left: -sb, right: sb, top: sb, bottom: -sb, near: 0.1, far: 40 });
    sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.015 * sb; S3.add(sun); S3.add(sun.target);
    if (this.o.ground) {
      this.groundMat = new T.MeshStandardMaterial({ color: 0xffffff, roughness: 0.95, metalness: 0 });
      // tiled, not one huge quad: two 240 m triangles lose depth precision near a low camera and poke through the arena floor
      const g = this.groundMesh = new T.Mesh(new T.PlaneGeometry(240, 240, 24, 24), this.groundMat); g.receiveShadow = true; g.position.z = -0.0005; this.world.add(g);
      if (this.o.grid) { const [gs, gd] = this.o.grid; this.grid = new T.GridHelper(gs, gd, 0x888888, 0x888888); this.grid.material.transparent = true; this.grid.material.opacity = 0.55; this.grid.position.y = 0.0006;
      if (this.o.gridAt) this.grid.position.set(this.o.gridAt[0], 0.0006, -this.o.gridAt[1]);
      S3.add(this.grid); }
    }
    this.helpers = new T.Group(); this.world.add(this.helpers); // lidar points, paths: hidden from robot cameras
    // image-based lighting from a small procedural studio (soft reflections on metal and plastic)
    try {
      const env = new T.Scene(), room = new T.Mesh(new T.BoxGeometry(10, 10, 10), new T.MeshBasicMaterial({ color: 0x4a4f56, side: T.BackSide })); env.add(room);
      const lamp = (x, y, z, w, hh, c) => { const m = new T.Mesh(new T.PlaneGeometry(w, hh), new T.MeshBasicMaterial({ color: c, side: T.DoubleSide })); m.position.set(x, y, z); m.lookAt(0, 0, 0); env.add(m); };
      lamp(0, 4.9, 0, 5, 5, 0xb8b8b8); lamp(4.9, 1.5, 1, 2, 3, 0x9aa8c0); lamp(-4.9, 1, -1.5, 2, 2, 0xc0b49c); lamp(0, -4.9, 0, 10, 10, 0x202326);
      const pm = new T.PMREMGenerator(r); this.envTex = pm.fromScene(env, 0.03).texture; pm.dispose();
    } catch (e) { this.envTex = null; }
    if (this.groundMat) this.mirrorSetup();
    this.applyTheme();
    this.bindPointer();
    this.resize(); this.ro = new ResizeObserver(() => this.resize()); this.ro.observe(this.wrap);
    try { this.mo = new MutationObserver(() => this.applyTheme()); this.mo.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] }); this.mq = matchMedia('(prefers-color-scheme: dark)'); this.mqf = () => this.applyTheme(); this.mq.addEventListener('change', this.mqf); } catch (e) { /* ignore */ }
    this.ready = true;
    return true;
  }
  lin(c) { return new this.T.Color(c).convertSRGBToLinear(); }
  applyTheme() {
    if (!this.scene) return;
    const T = this.T, bg = cssVar('--surface-2') || '#EEF3F1', fl = cssVar('--surface') || '#ffffff', ln = cssVar('--muted') || '#5D6D66', P = this.preset = SnoView.preset();
    this.dark = document.documentElement.dataset.theme === 'dark' || (!document.documentElement.dataset.theme && matchMedia('(prefers-color-scheme: dark)').matches);
    const r = this.r, gm = this.groundMat;
    r.toneMapping = P === 'classic' ? T.NoToneMapping : T.ACESFilmicToneMapping; r.toneMappingExposure = P === 'isaac' ? 0.9 : 0.85;
    this.scene.environment = P === 'classic' ? null : this.envTex;
    if (P === 'mujoco') {
      // MuJoCo's default look: blue gradient sky and a dark blue checker floor with light edge marks
      this.scene.background = this.skyTex || (this.skyTex = snoTex(T, 4, 256, (g, w, hh) => { const gr = g.createLinearGradient(0, 0, 0, hh); gr.addColorStop(0, '#4d80b3'); gr.addColorStop(0.55, '#2a4560'); gr.addColorStop(1, '#05080c'); g.fillStyle = gr; g.fillRect(0, 0, w, hh); }));
      this.scene.fog = null;
      if (gm) { gm.map = this.checkerTex || (this.checkerTex = snoTex(T, 512, 512, (g) => { for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) { g.fillStyle = (i + j) % 2 ? '#1a3349' : '#334d66'; g.fillRect(i * 256, j * 256, 256, 256); } g.strokeStyle = 'rgba(204,204,204,0.55)'; g.lineWidth = 3; for (let k = 0; k <= 2; k++) { g.beginPath(); g.moveTo(k * 256, 0); g.lineTo(k * 256, 512); g.stroke(); g.beginPath(); g.moveTo(0, k * 256); g.lineTo(512, k * 256); g.stroke(); } })); gm.map.wrapS = gm.map.wrapT = T.RepeatWrapping; gm.map.repeat.set(240, 240); gm.color.set(0xffffff); gm.roughness = 0.5; gm.metalness = 0; gm.needsUpdate = true; }
      if (this.grid) this.grid.visible = false;
      this.hemi.color.set(0xdfe9ff); this.hemi.groundColor.set(0x1d2630); this.hemi.intensity = 0.35; this.sun.intensity = 1.15;
    } else if (P === 'isaac') {
      // Isaac Sim's look: neutral grey studio, grey ground with a fine grid
      this.scene.background = this.isaacTex || (this.isaacTex = snoTex(T, 4, 256, (g, w, hh) => { const gr = g.createLinearGradient(0, 0, 0, hh); gr.addColorStop(0, '#22262b'); gr.addColorStop(0.6, '#3b4046'); gr.addColorStop(1, '#4b5056'); g.fillStyle = gr; g.fillRect(0, 0, w, hh); }));
      this.scene.fog = new T.Fog(this.lin('#3b4046'), this.o.fog[0] * 1.6, this.o.fog[1] * 2);
      if (gm) { gm.map = this.gridTex || (this.gridTex = snoTex(T, 512, 512, (g) => { g.fillStyle = '#62676d'; g.fillRect(0, 0, 512, 512); g.strokeStyle = 'rgba(255,255,255,0.10)'; g.lineWidth = 1; for (let k = 0; k <= 10; k++) { g.beginPath(); g.moveTo(k * 51.2, 0); g.lineTo(k * 51.2, 512); g.stroke(); g.beginPath(); g.moveTo(0, k * 51.2); g.lineTo(512, k * 51.2); g.stroke(); } g.strokeStyle = 'rgba(255,255,255,0.28)'; g.lineWidth = 3; g.strokeRect(0, 0, 512, 512); })); gm.map.wrapS = gm.map.wrapT = T.RepeatWrapping; gm.map.repeat.set(240, 240); gm.color.set(0xffffff); gm.roughness = 0.82; gm.needsUpdate = true; }
      if (this.grid) this.grid.visible = false;
      this.hemi.color.set(0xffffff); this.hemi.groundColor.set(0x50555b); this.hemi.intensity = 0.45; this.sun.intensity = 1.0;
    } else {
      this.scene.background = new T.Color(bg);
      const [f0, f1] = this.o.fog; this.scene.fog = new T.Fog(this.lin(bg), f0, f1);
      if (gm) { gm.map = null; gm.color = this.lin(this.o.groundColor || fl); gm.roughness = 0.95; gm.needsUpdate = true; }
      if (this.grid) { this.grid.visible = true; const c = this.lin(ln); this.grid.material.color.set(0xffffff); if (this.grid.material.vertexColors) { const cols = this.grid.geometry.attributes.color; for (let i = 0; i < cols.count; i++) cols.setXYZ(i, c.r, c.g, c.b); cols.needsUpdate = true; } this.grid.material.opacity = this.dark ? 0.35 : 0.3; }
      this.hemi.color.set(0xffffff); this.hemi.groundColor.set(0x8d9894); this.hemi.intensity = this.dark ? 0.5 : 0.62; this.sun.intensity = this.dark ? 0.85 : 0.95;
    }
    if (this.mirror) this.mirror.k = P === 'mujoco' && SnoView.reflect() && !(this.ql >= 2) ? 0.22 : 0;
    if (this.onTheme) this.onTheme(this.dark, P);
  }
  // adaptive quality for slow devices: 0 full, 1 one pixel per CSS pixel, 2 also no mirror pass and smaller shadow maps
  setQuality(l) {
    if (!this.r || l === (this.ql || 0)) return; this.ql = l;
    this.r.setPixelRatio(l >= 1 ? 1 : Math.min(2, devicePixelRatio || 1));
    const ms = l >= 2 ? 1024 : 2048; if (this.sun.shadow.mapSize.x !== ms) { this.sun.shadow.mapSize.set(ms, ms); if (this.sun.shadow.map) { this.sun.shadow.map.dispose(); this.sun.shadow.map = null; } }
    this.resize(); this.applyTheme();
  }
  static preset() { return (S.sno && S.sno.scene) || 'mujoco'; }
  static reflect() { return !(S.sno && S.sno.noReflect); }
  // planar reflection of the floor (rendered from a mirrored camera into a texture)
  mirrorSetup() {
    const T = this.T, rt = new T.WebGLRenderTarget(512, 512), cam = new T.PerspectiveCamera(), tm = new T.Matrix4(), M = this.mirror = { rt, cam, tm, k: 0 }, gm = this.groundMat;
    gm.onBeforeCompile = (sh) => {
      sh.uniforms.tRefl = { value: rt.texture }; sh.uniforms.texMat = { value: tm }; sh.uniforms.reflK = { get value() { return M.k; } };
      sh.vertexShader = 'uniform mat4 texMat;\nvarying vec4 vRefl;\n' + sh.vertexShader.replace('#include <project_vertex>', '#include <project_vertex>\n  vRefl = texMat * modelMatrix * vec4(position, 1.0);');
      sh.fragmentShader = 'uniform sampler2D tRefl;\nuniform float reflK;\nvarying vec4 vRefl;\n' + sh.fragmentShader.replace('#include <dithering_fragment>', '#include <dithering_fragment>\n  if (reflK > 0.0) { vec4 rc = texture2DProj(tRefl, vRefl); gl_FragColor.rgb = mix(gl_FragColor.rgb, rc.rgb, reflK); }');
    };
  }
  renderMirror() {
    const M = this.mirror; if (!M || !M.k || !this.groundMesh) return;
    const T = this.T, c = this.cam, mc = M.cam, r = this.r;
    mc.copy(c); mc.position.set(c.position.x, -c.position.y, c.position.z);
    const t3 = this.toThree(this.tgt); mc.up.set(0, -1, 0); mc.lookAt(t3.x, -t3.y, t3.z); mc.updateMatrixWorld(); mc.updateProjectionMatrix();
    M.tm.set(0.5, 0, 0, 0.5, 0, 0.5, 0, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 1); M.tm.multiply(mc.projectionMatrix); M.tm.multiply(mc.matrixWorldInverse);
    this.groundMesh.visible = false; this.helpers.visible = false; const sh = r.shadowMap.autoUpdate; r.shadowMap.autoUpdate = false;
    r.setRenderTarget(M.rt); r.clear(); r.render(this.scene, mc); r.setRenderTarget(null);
    r.shadowMap.autoUpdate = sh; this.groundMesh.visible = true; this.helpers.visible = true;
  }
  toThree(p) { return new this.T.Vector3(p[0], p[2], -p[1]); }
  resize() {
    if (!this.r) return;
    const w = Math.max(10, this.wrap.clientWidth), hgt = Math.max(10, Math.round(w * (this.o.aspect || 0.7)));
    this.w = w; this.h = hgt; this.r.setSize(w, hgt, false); this.cv.style.width = '100%'; this.cv.style.height = hgt + 'px';
    this.cam.aspect = w / hgt; this.cam.updateProjectionMatrix();
  }
  addPip(link, o) {
    const T = this.T, cam = new T.PerspectiveCamera(o.vfov || 60, o.aspect || 4 / 3, o.near || 0.02, o.far || 30);
    const M = o.look === 'z' ? new T.Matrix4().makeBasis(new T.Vector3(0, 1, 0), new T.Vector3(1, 0, 0), new T.Vector3(0, 0, -1)) : new T.Matrix4().makeBasis(new T.Vector3(0, -1, 0), new T.Vector3(0, 0, 1), new T.Vector3(-1, 0, 0)); cam.quaternion.setFromRotationMatrix(M);
    if (o.xyz) cam.position.set(o.xyz[0], o.xyz[1], o.xyz[2]);
    cam.layers.set(0); link.add(cam);
    const p = Object.assign({ cam, label: 'camera', depth: false }, o); this.pips.push(p); return p;
  }
  // an onboard camera for sensor_msgs/Image: renders off screen at its own resolution, the way the robot sees the scene
  addCamera(link, o) {
    const T = this.T, cam = new T.PerspectiveCamera(o.vfov || 56, o.w / o.h, o.near || 0.02, o.far || 30);
    cam.quaternion.setFromRotationMatrix(new T.Matrix4().makeBasis(new T.Vector3(0, -1, 0), new T.Vector3(0, 0, 1), new T.Vector3(-1, 0, 0))); cam.layers.set(0); link.add(cam);
    const rt = new T.WebGLRenderTarget(o.w, o.h, { minFilter: T.LinearFilter, magFilter: T.LinearFilter, format: T.RGBAFormat, encoding: T.sRGBEncoding });
    const C = { cam, rt, w: o.w, h: o.h, px: new Uint8Array(o.w * o.h * 4) }; this.cams.push(C); return C;
  }
  // render one frame and return it as rgb8, top row first (sensor_msgs/Image order); helpers such as lidar points stay hidden
  capture(C) {
    const r = this.r; if (!r || !this.ready) return null;
    const prev = r.getRenderTarget(), sh = r.shadowMap.autoUpdate, gv = this.grid ? this.grid.visible : false;
    this.helpers.visible = false; if (this.grid) this.grid.visible = false; r.shadowMap.autoUpdate = false;
    try { r.setRenderTarget(C.rt); r.setScissorTest(false); r.clear(); r.render(this.scene, C.cam); r.readRenderTargetPixels(C.rt, 0, 0, C.w, C.h, C.px); }
    finally { r.setRenderTarget(prev); r.shadowMap.autoUpdate = sh; this.helpers.visible = true; if (this.grid) this.grid.visible = gv; }
    const W = C.w, H = C.h, src = C.px, out = new Uint8Array(W * H * 3);
    for (let y = 0; y < H; y++) { let i = (H - 1 - y) * W * 4, o = y * W * 3; for (let x = 0; x < W; x++, i += 4, o += 3) { out[o] = src[i]; out[o + 1] = src[i + 1]; out[o + 2] = src[i + 2]; } }
    return out;
  }
  // the same camera as a depth sensor (REP 118): z along the optical axis in millimetres, 0 where nothing is in range.
  // o: { dmin, dmax (metres), noise (relative error per metre of depth: stereo error grows with z²) }
  captureDepth(C, o) {
    const r = this.r; if (!r || !this.ready) return null;
    const T = this.T, dm = this.depthMmMat(), sc = this.scene;
    if (!C.drt) { C.drt = new T.WebGLRenderTarget(C.w, C.h, { minFilter: T.NearestFilter, magFilter: T.NearestFilter, format: T.RGBAFormat }); C.dpx = new Uint8Array(C.w * C.h * 4); }
    dm.uniforms.uNear.value = o.dmin; dm.uniforms.uFar.value = o.dmax; dm.uniforms.uNoise.value = o.noise || 0; dm.uniforms.uSeed.value = (this._seed = ((this._seed || 0) + 7.31) % 97);
    const prev = r.getRenderTarget(), sh = r.shadowMap.autoUpdate, gv = this.grid ? this.grid.visible : false, bg = sc.background, fog = sc.fog, cc = r.getClearColor(new T.Color()), ca = r.getClearAlpha();
    this.helpers.visible = false; if (this.grid) this.grid.visible = false; r.shadowMap.autoUpdate = false; sc.background = null; sc.fog = null; sc.overrideMaterial = dm; r.setClearColor(0x000000, 0);
    try { r.setRenderTarget(C.drt); r.setScissorTest(false); r.clear(); r.render(sc, C.cam); r.readRenderTargetPixels(C.drt, 0, 0, C.w, C.h, C.dpx); }
    finally { r.setRenderTarget(prev); r.shadowMap.autoUpdate = sh; this.helpers.visible = true; if (this.grid) this.grid.visible = gv; sc.background = bg; sc.fog = fog; sc.overrideMaterial = null; r.setClearColor(cc, ca); }
    const W = C.w, H = C.h, src = C.dpx, out = new Uint16Array(W * H);
    for (let y = 0; y < H; y++) { let i = (H - 1 - y) * W * 4, k = y * W; for (let x = 0; x < W; x++, i += 4, k++) out[k] = (src[i] << 8) | src[i + 1]; }
    return out;
  }
  // depth in mm, split over two 8-bit channels (high byte in red, low byte in green); no tone mapping or colour space
  depthMmMat() {
    if (this._dmm) return this._dmm;
    const T = this.T;
    this._dmm = new T.ShaderMaterial({
      uniforms: { uNear: { value: 0.2 }, uFar: { value: 6.0 }, uNoise: { value: 0 }, uSeed: { value: 0 } }, side: T.DoubleSide,
      vertexShader: 'varying float vD; void main(){ vec4 mv = modelViewMatrix * vec4(position, 1.0); vD = -mv.z; gl_Position = projectionMatrix * mv; }',
      fragmentShader: `uniform float uNear; uniform float uFar; uniform float uNoise; uniform float uSeed; varying float vD;
        float rnd(vec2 p){ return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
        void main(){
          if (vD < uNear || vD > uFar) { gl_FragColor = vec4(0.0); return; }
          float n = rnd(gl_FragCoord.xy + uSeed) + rnd(gl_FragCoord.yx * 1.37 + uSeed * 1.71) - 1.0;
          float mm = clamp(floor(vD * 1000.0 * (1.0 + uNoise * vD * n) + 0.5), ceil(uNear * 1000.0), floor(uFar * 1000.0)), hi = floor(mm / 256.0);
          gl_FragColor = vec4(hi / 255.0, (mm - hi * 256.0) / 255.0, 0.0, 1.0);
        }`,
    });
    return this._dmm;
  }
  depthMat() {
    if (this._dm) return this._dm;
    const T = this.T;
    this._dm = new T.ShaderMaterial({
      uniforms: { uFar: { value: 4.0 } },
      vertexShader: 'varying float vD; void main(){ vec4 mv = modelViewMatrix * vec4(position, 1.0); vD = -mv.z; gl_Position = projectionMatrix * mv; }',
      fragmentShader: 'uniform float uFar; varying float vD; void main(){ float d = clamp(vD / uFar, 0.0, 1.0); vec3 near = vec3(1.0, 0.92, 0.25), mid = vec3(0.95, 0.25, 0.35), far = vec3(0.12, 0.18, 0.62); vec3 c = d < 0.5 ? mix(near, mid, d * 2.0) : mix(mid, far, d * 2.0 - 1.0); if (vD > uFar) c = vec3(0.03, 0.04, 0.08); gl_FragColor = vec4(pow(c, vec3(2.2)), 1.0); }',
    });
    return this._dm;
  }
  pipRect() { const pw = Math.round(Math.min(this.w * 0.36, 300)), ph = Math.round(pw * 0.75); return { x: this.w - pw - 10, y: 10, w: pw, h: ph }; }
  setTarget(p, yaw) { this.followPos = p; this.followYaw = yaw; }
  bindPointer() {
    const cv = this.cv; cv.style.touchAction = 'none';
    let down = null, pinch0 = 0;
    const pts = () => [...this.ptrs.values()];
    cv.addEventListener('pointerdown', (e) => {
      cv.setPointerCapture(e.pointerId);
      // MuJoCo-style perturbation: Ctrl+drag (or the Grab tool) pulls the body under the cursor with a spring
      if (e.button === 0 && (e.ctrlKey || e.metaKey || this.grabMode) && this.onGrab && this.ptrs.size === 0) {
        const hit = this.pick(e.clientX, e.clientY);
        if (hit && this.onGrab(hit)) { const T = this.T, n = new T.Vector3(); this.cam.getWorldDirection(n); this.dragPlane = new T.Plane().setFromNormalAndCoplanarPoint(n, hit.p3); this.ptrs.set(e.pointerId, { x: e.clientX, y: e.clientY, grab: true }); down = null; return; }
      }
      this.ptrs.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.ptrs.size === 1) down = { x: e.clientX, y: e.clientY, t: performance.now(), btn: e.button, shift: e.shiftKey, moved: 0 };
      if (this.ptrs.size === 2) { const [a, b] = pts(); pinch0 = Math.hypot(a.x - b.x, a.y - b.y); if (down) down.moved = 99; }
      this.wrap.focus({ preventScroll: true });
    });
    cv.addEventListener('pointermove', (e) => {
      const p = this.ptrs.get(e.pointerId); if (!p) return;
      if (p.grab) { const t = this.rayPlane(e.clientX, e.clientY, this.dragPlane); if (t && this.onDrag) this.onDrag(t); return; }
      const dx = e.clientX - p.x, dy = e.clientY - p.y; p.x = e.clientX; p.y = e.clientY;
      if (this.ptrs.size === 2) {
        const [a, b] = pts(), d = Math.hypot(a.x - b.x, a.y - b.y);
        if (pinch0 > 0) { this.dist = clampN(this.dist * pinch0 / d, this.o.minDist, this.o.maxDist); pinch0 = d; }
        this.pan(dx / 2, dy / 2); return;
      }
      if (!down) return; down.moved += Math.abs(dx) + Math.abs(dy);
      if (down.btn === 2 || down.btn === 1 || down.shift) this.pan(dx, dy);
      else { this.az -= dx * 0.008; this.el = clampN(this.el + dy * 0.006, 0.04, 1.5); }
    });
    const up = (e) => {
      const pp = this.ptrs.get(e.pointerId);
      if (pp && pp.grab) { this.ptrs.delete(e.pointerId); this.dragPlane = null; if (this.onRelease) this.onRelease(); return; }
      if (down && this.ptrs.size === 1 && down.moved < 6 && performance.now() - down.t < 450 && down.btn === 0 && this.onClick) { const g = this.groundAt(e.clientX, e.clientY); if (g) this.onClick(g, e); }
      this.ptrs.delete(e.pointerId); if (!this.ptrs.size) down = null; pinch0 = 0;
    };
    cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
    cv.addEventListener('contextmenu', (e) => e.preventDefault());
    cv.addEventListener('wheel', (e) => { e.preventDefault(); this.dist = clampN(this.dist * Math.exp(e.deltaY * 0.0012), this.o.minDist, this.o.maxDist); }, { passive: false });
    cv.addEventListener('dblclick', () => { this.follow = true; this.dist = this.o.dist; this.el = this.o.el; if (this.onFollow) this.onFollow(); });
  }
  ray(cx, cy) { const T = this.T, rc = this.cv.getBoundingClientRect(), r = new T.Raycaster(); r.setFromCamera(new T.Vector2(((cx - rc.left) / rc.width) * 2 - 1, -((cy - rc.top) / rc.height) * 2 + 1), this.cam); return r; }
  // the body under the cursor: { obj (three object), p3 (three point), p (ROS point) }
  pick(cx, cy) {
    const r = this.ray(cx, cy), hits = r.intersectObjects(this.world.children.filter(o => o !== this.groundMesh && o !== this.helpers), true);
    for (const h of hits) { let o = h.object; while (o && !o.userData.link && !o.userData.obj) o = o.parent; if (o) return { obj: o, p3: h.point.clone(), p: [h.point.x, -h.point.z, h.point.y] }; }
    return null;
  }
  rayPlane(cx, cy, plane) { if (!plane) return null; const v = new this.T.Vector3(); return this.ray(cx, cy).ray.intersectPlane(plane, v) ? [v.x, -v.z, v.y] : null; }
  pan(dx, dy) {
    this.follow = false; if (this.onFollow) this.onFollow();
    const s = this.dist * 0.0016, ca = Math.cos(this.az), sa = Math.sin(this.az);
    // screen right in ROS frame = (sin az, -cos az); screen up (projected forward) = (cos az, sin az)
    this.tgt[0] += (-dx * sa + dy * ca) * s; this.tgt[1] += (dx * ca + dy * sa) * s;
  }
  groundAt(cx, cy) {
    const T = this.T, rc = this.cv.getBoundingClientRect();
    const v = new T.Vector2(((cx - rc.left) / rc.width) * 2 - 1, -((cy - rc.top) / rc.height) * 2 + 1);
    const ray = new T.Raycaster(); ray.setFromCamera(v, this.cam);
    const o = ray.ray.origin, d = ray.ray.direction; if (Math.abs(d.y) < 1e-6) return null;
    const t = -o.y / d.y; if (t < 0) return null;
    return [o.x + d.x * t, -(o.z + d.z * t)];
  }
  render() {
    if (!this.ready) return;
    const T = this.T;
    if (this.chase && this.follow && this.followYaw != null) this.az += wrapPi(this.followYaw - this.az) * 0.06;
    if (this.follow && this.followPos) { const f = this.followPos, far = Math.hypot(f[0] - this.tgt[0], f[1] - this.tgt[1]) > 2.5, k = far ? 1 : 0.14; this.tgt[0] += (f[0] - this.tgt[0]) * k; this.tgt[1] += (f[1] - this.tgt[1]) * k; this.tgt[2] += (f[2] - this.tgt[2]) * k; }
    const ce = Math.cos(this.el), t3 = this.toThree(this.tgt);
    const off = this.toThree([-ce * Math.cos(this.az) * this.dist, -ce * Math.sin(this.az) * this.dist, Math.sin(this.el) * this.dist]);
    this.cam.position.copy(t3).add(off); this.cam.lookAt(t3);
    this.sun.position.set(t3.x + 3.0, t3.y + 6.5, t3.z + 2.2); this.sun.target.position.copy(t3);
    if (this.grid && this.o.gridFollow) { const gs = this.o.grid[0] / this.o.grid[1]; this.grid.position.x = Math.round(t3.x / gs) * gs; this.grid.position.z = Math.round(t3.z / gs) * gs; }
    const r = this.r; this.cam.layers.enableAll();
    this.cam.updateMatrixWorld(); this.renderMirror();
    r.setViewport(0, 0, this.w, this.h); r.setScissorTest(false);
    r.render(this.scene, this.cam);
    const p = this.pipOn && this.pips[this.pipIdx];
    if (p) {
      const R = this.pipRect(), y = this.h - R.y - R.h;
      r.setScissorTest(true); r.setViewport(R.x, y, R.w, R.h); r.setScissor(R.x, y, R.w, R.h);
      p.cam.aspect = R.w / R.h; p.cam.updateProjectionMatrix();
      this.helpers.visible = false; const gv = this.grid ? this.grid.visible : false; if (this.grid && p.depth) this.grid.visible = false;
      if (p.hide) p.hide.forEach(o => { o.visible = false; });
      if (p.depth) { const dm = this.depthMat(); dm.uniforms.uFar.value = p.far || 4; this.scene.overrideMaterial = dm; }
      r.render(this.scene, p.cam);
      this.scene.overrideMaterial = null; this.helpers.visible = true; if (this.grid) this.grid.visible = gv;
      if (p.hide) p.hide.forEach(o => { o.visible = true; });
      r.setScissorTest(false); r.setViewport(0, 0, this.w, this.h);
    }
  }
  dispose() {
    this.ready = false;
    try { if (this.ro) this.ro.disconnect(); if (this.mo) this.mo.disconnect(); if (this.mq) this.mq.removeEventListener('change', this.mqf); } catch (e) { /* ignore */ }
    if (this.r) { try { this.scene.traverse(o => { if (o.material && o.material.map) o.material.map.dispose(); }); this.r.dispose(); this.r.forceContextLoss(); } catch (e) { /* ignore */ } if (this.cv) this.cv.remove(); }
    this.r = null; this.scene = null;
  }
}
/* canvas texture helper (floor mats, signs) */
function snoTex(T, w, hgt, draw) {
  const c = document.createElement('canvas'); c.width = w; c.height = hgt; const g = c.getContext('2d'); draw(g, w, hgt);
  const t = new T.CanvasTexture(c); t.encoding = T.sRGBEncoding; t.anisotropy = 4; return t;
}
function snoLine(T, color, opacity = 1) { const g = new T.BufferGeometry(); g.setAttribute('position', new T.Float32BufferAttribute(new Float32Array(3 * 2), 3)); return new T.Line(g, new T.LineBasicMaterial({ color, transparent: opacity < 1, opacity })); }
function snoSetLine(line, pts, z = 0.01) {
  const T = window.THREE, g = line.geometry; let a = g.attributes.position;
  if (a.count < pts.length) { g.setAttribute('position', new T.Float32BufferAttribute(new Float32Array(Math.max(pts.length, 8) * 3 * 2), 3)); a = g.attributes.position; }
  pts.forEach((p, i) => a.setXYZ(i, p[0], p[1], p.length > 2 ? p[2] : z)); a.needsUpdate = true; g.setDrawRange(0, pts.length); g.computeBoundingSphere();
}

/* ------------------------------ physics overlays: contacts and forces, centre of mass, support polygon, collision shapes ------------------------------ */
function snoPhysViz(sim, view, T) {
  if (!sim.world) return;
  const G = new T.Group(); view.helpers.add(G);
  const MAX = 128, dot = new T.InstancedMesh(new T.SphereGeometry(1, 10, 8), new T.MeshBasicMaterial({ color: 0xff3b30 }), MAX); dot.frustumCulled = false; G.add(dot);
  const fl = new T.LineSegments(new T.BufferGeometry(), new T.LineBasicMaterial({ color: 0xffd60a })); fl.geometry.setAttribute('position', new T.Float32BufferAttribute(new Float32Array(MAX * 6), 3)); fl.frustumCulled = false; G.add(fl);
  const com = new T.Mesh(new T.SphereGeometry(1, 16, 12), new T.MeshBasicMaterial({ color: 0xffd60a, depthTest: false, transparent: true, opacity: 0.95 })); com.renderOrder = 10; G.add(com);
  const comLine = snoLine(T, 0xffd60a, 0.8), comDisc = new T.Mesh(new T.RingGeometry(0.6, 1, 24), new T.MeshBasicMaterial({ color: 0xffd60a, transparent: true, opacity: 0.8, side: T.DoubleSide })); G.add(comLine, comDisc);
  const poly = new T.LineLoop(new T.BufferGeometry(), new T.LineBasicMaterial({ color: 0x34c759 })); poly.geometry.setAttribute('position', new T.Float32BufferAttribute(new Float32Array(MAX * 3), 3)); poly.frustumCulled = false; G.add(poly);
  const grab = snoLine(T, 0xff375f), grabDot = new T.Mesh(new T.SphereGeometry(1, 12, 8), new T.MeshBasicMaterial({ color: 0xff375f })); G.add(grab, grabDot);
  const shapes = new T.Group(); G.add(shapes);
  const wmat = new T.MeshBasicMaterial({ color: 0x30d158, wireframe: true, transparent: true, opacity: 0.55 }), gm = [];
  const geomOf = (g) => {
    const s = g.size;
    if (g.t === PH_BOX) return new T.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
    if (g.t === PH_SPH) return new T.SphereGeometry(s[0], 12, 8);
    const c = new T.CylinderGeometry(s[0], s[0], g.t === PH_CAP ? 2 * s[1] : 2 * s[1], 14, 1, g.t !== PH_CAP); c.rotateX(Math.PI / 2); return c;
  };
  const size = sim.vizSize || 0.25, m = new T.Matrix4(), tmp = new T.Object3D(), R = (sim.rb ? sim.rb.totalMass : 1) * 9.81, fk = (sim.vizF || 0.3) / R;
  const setM = (o, W) => { o.matrix.set(W[0], W[1], W[2], W[3], W[4], W[5], W[6], W[7], W[8], W[9], W[10], W[11], 0, 0, 0, 1); o.matrixAutoUpdate = false; o.matrixWorldNeedsUpdate = true; };
  sim.physViz = {
    sync() {
      const V = S.sno.viz || {}, W = sim.world;
      dot.visible = fl.visible = !!V.contacts; com.visible = comLine.visible = comDisc.visible = poly.visible = !!V.com; shapes.visible = !!V.shapes;
      if (V.contacts) {
        const P = fl.geometry.attributes.position; let n = 0;
        for (let i = 0; i < W.nc && n < MAX; i++) {
          const c = W.C[i]; if (!(c.fn > 1e-3)) continue;
          tmp.position.set(c.p[0], c.p[1], c.p[2]); tmp.scale.setScalar(size * 0.02); tmp.updateMatrix(); dot.setMatrixAt(n, tmp.matrix);
          const f = c.f || [0, 0, 0]; P.setXYZ(2 * n, c.p[0], c.p[1], c.p[2]); P.setXYZ(2 * n + 1, c.p[0] + f[0] * fk, c.p[1] + f[1] * fk, c.p[2] + f[2] * fk); n++;
        }
        dot.count = n; dot.instanceMatrix.needsUpdate = true; P.needsUpdate = true; fl.geometry.setDrawRange(0, 2 * n);
      }
      if (V.com && sim.rb) {
        const c = sim.rb.comState(sim._vc || (sim._vc = new Float64Array(6))), r = size * 0.05;
        com.position.set(c[0], c[1], c[2]); com.scale.setScalar(r); snoSetLine(comLine, [[c[0], c[1], c[2]], [c[0], c[1], 0.002]]);
        comDisc.position.set(c[0], c[1], 0.003); comDisc.scale.setScalar(r * 1.4);
        // support polygon: convex hull of the robot's contact points
        const pts = []; for (let i = 0; i < W.nc; i++) { const k = W.C[i]; if (k.fn > 1e-3 && k.a.s === sim.rb && !k.b.s) pts.push([k.p[0], k.p[1]]); }
        const hull = snoHull(pts), P = poly.geometry.attributes.position; hull.forEach((p, i) => P.setXYZ(i, p[0], p[1], 0.004)); P.needsUpdate = true; poly.geometry.setDrawRange(0, hull.length); poly.visible = hull.length > 1;
      }
      const g = sim.grab; grab.visible = grabDot.visible = !!g;
      if (g) { const p = K3.apply(g.sys.T[g.b], ...g.local); snoSetLine(grab, [p, g.target]); grabDot.position.set(g.target[0], g.target[1], g.target[2]); grabDot.scale.setScalar(size * 0.025); }
      if (V.shapes) {
        let i = 0;
        for (const S2 of W.sys) for (const ge of S2.geoms) { let o = gm[i]; if (!o) { o = gm[i] = new T.Mesh(geomOf(ge), wmat); o.userData.g = ge; shapes.add(o); } if (o.userData.g !== ge) { o.geometry.dispose(); o.geometry = geomOf(ge); o.userData.g = ge; } o.visible = !S2.off; setM(o, ge.W); i++; }
        for (const st of W.statics) { let o = gm[i]; if (!o) { o = gm[i] = new T.Mesh(geomOf(st.t === PH_VCYL ? { t: PH_WHEEL, size: [st.size[0], st.size[1] / 2] } : st), wmat); o.userData.g = st; shapes.add(o); } const Wm = st.t === PH_VCYL ? K3.fromRPY(st.W[3], st.W[7], st.size[1] / 2, 0, 0, 0) : st.W; setM(o, Wm); i++; }
        for (let k = i; k < gm.length; k++) gm[k].visible = false;
      }
      void m;
    },
  };
}
function snoHull(pts) {
  if (pts.length < 3) return pts.slice();
  const P = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]), cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]), lo = [], up = [];
  for (const p of P) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
  for (let i = P.length - 1; i >= 0; i--) { const p = P[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

/* ------------------------------ simulator base class ------------------------------ */
class SnoSim {
  constructor(def) {
    this.def = def; this.id = def.id; this.bus = makeBus(); this.started = false; this.t = 0; this.tab = def.tabs[0][0];
    this.pyLines = []; this.errLine = 0; this.rates = {}; this.selTopic = null; this.topicView = 'list';
    this.cmd = { vx: 0, vy: 0, wz: 0, t: -1e9, src: '', kind: '' };
    this.procs = {}; this.procTerm = {}; this.logs = []; this.fix = null; this.tn = {}; this.tfStatic = new Map();
    // DDS discovery warns once on each side when a publisher and a subscription cannot agree on QoS
    this.bus.onClash = (pub, sub, topic, policy) => {
      const vis = (n) => { const nd = this.bus.nodes.get(n); return nd && !nd.hidden; };
      if (vis(sub)) this.log(sub, 'WARN', `New publisher discovered on topic '${topic}', offering incompatible QoS. No messages will be received from it. Last incompatible policy: ${policy}`);
      if (vis(pub)) this.log(pub, 'WARN', `New subscription discovered on topic '${topic}', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: ${policy}`);
    };
    const self = this;
    this.ui = { termState() { if (self.tp) self.tp.state(); }, focusTab(t) { SnoUI.focusTab(t === 'graph' ? 'topics' : t, t === 'graph'); } };
    this.missionApi = { event: (t, d) => this.event(t, d) };
    this.pyHost = { bus: this.bus, missions: this.missionApi, simTime: () => this.t, saveCode: (c) => { S.sno.code[this.id] = c; save({ local: true }); }, ui: { pyLog: (t, c) => this.pyLog(t, c), pyStatus: () => this.pyStatus(), markErr: (l) => this.markErr(l), clearErr: () => this.clearErr(), imshow: (d, bin) => SnoVision.imshow(this, d, bin) } };
  }
  start() {
    if (this.started) return; this.started = true;
    this.bus.topic('/parameter_events', 'rcl_interfaces/msg/ParameterEvent').keep = true; this.bus.topic('/rosout', 'rcl_interfaces/msg/Log').keep = true;
    this.setupRos();
    this.tp = new TermPanel(() => this.termCtx(), { welcome: `Connected to the ${this.def.name} simulator. ROS 2 Jazzy is sourced.\nType \`help\` for this robot's topics, or try: ros2 topic list`, chips: this.def.chips || [], rerender: () => SnoUI.renderTab() });
  }
  termCtx() {
    return { bus: this.bus, missions: this.missionApi, ui: this.ui, prompt: `learner@${this.id}-sim:~$`, help: this.def.help, pkgs: this.pkgs(), frames: this.frames(), graphTab: 'Topics', hist: () => S.sno.hist, saveHist: (hs) => { S.sno.hist = hs.slice(-50); save({ local: true }); }, run: (T, pkg, r) => this.run(T, pkg, r), launch: (T, pkg, r) => this.launch(T, pkg, r) };
  }
  /* ---------- ROS helpers ---------- */
  // a node's log line: printed in the terminal that started it, published on /rosout, kept for the Fix it log.
  // raw: true is a line from the launch system itself ([INFO] [launch]: …), 'out' is a process's own console output
  log(node, level, text, raw) {
    const st = stampNow(), name = node.replace(/^\//, ''), line = raw === 'out' ? `[${name}] ${text}` : raw ? `[${level}] [${name}]: ${text}` : `[${level}] [${(st.sec + st.nanosec / 1e9).toFixed(9)}] [${name}]: ${text}`;
    // in the robot's launch output a node's lines carry the process name, e.g. [limo_base_node-3]
    const lab = !raw && this.launchLabel && !(this.procTerm && this.procTerm[node]) ? this.launchLabel[node] : '';
    (this.logs = this.logs || []).push([level, lab ? `[${lab}] ${line}` : line]); if (this.logs.length > 240) this.logs.splice(0, 60);
    if (!raw && this.bus.nodes.has(node)) this.bus.publish(node, '/rosout', 'rcl_interfaces/msg/Log', { stamp: st, level: { DEBUG: 10, INFO: 20, WARN: 30, ERROR: 40, FATAL: 50 }[level] || 20, name, msg: text, file: '', function: '', line: 0 });
    const T = this.procTerm && this.procTerm[node]; if (T) T.print(line, level === 'ERROR' || level === 'FATAL' ? 'e' : level === 'WARN' ? 'w' : 'o');
    if (this.onLog) this.onLog(level, line);
  }
  // `--ros-args -r from:=to -p name:=value`
  static rosArgs(list) {
    const out = { pos: list, remaps: [], params: {}, bad: [] }, i0 = list.indexOf('--ros-args'); if (i0 < 0) return out;
    out.pos = list.slice(0, i0);
    for (let i = i0 + 1; i < list.length; i++) {
      const a = list[i], kv = (v) => { const k = String(v || '').indexOf(':='); return k > 0 ? [v.slice(0, k), v.slice(k + 2)] : null; };
      if (a === '-r' || a === '--remap') { const x = kv(list[++i]); if (x) out.remaps.push(x); else out.bad.push(list[i] || a); }
      else if (a === '-p' || a === '--param') { const x = kv(list[++i]); if (x) out.params[x[0]] = x[1]; else out.bad.push(list[i] || a); }
      else if (a !== '--') out.bad.push(a);
    }
    return out;
  }
  // topic names after remapping (this sim runs every node in the root namespace)
  remapper(rules) {
    const full = (n) => (String(n).startsWith('/') ? n : '/' + n).replace(/\/+/g, '/');
    return (name) => { const f = full(name); for (const [a, b] of rules || []) if (!a.startsWith('__') && full(a) === f) return full(b); return f; };
  }
  node(name, kind, params, opts = {}) {
    const n = this.bus.addNode(name, kind, Object.assign({ onParam: (k, v) => this.onParam(name, k, v) }, opts));
    (params || []).forEach(([k, v, t, range, ro]) => n.params.set(k, { v, t, range, ro }));
    return n;
  }
  P(node, k) { const n = this.bus.nodes.get(node); const e = n && n.params.get(k); return e ? e.v : undefined; }
  setP(node, k, v) { const n = this.bus.nodes.get(node); const e = n && n.params.get(k); if (e) e.v = v; }
  onParam() { return { ok: true }; }
  pub(node, topic, type, msg) { this.bus.publish(node, topic, type, msg); }
  every(key, dt, period) { this.rates[key] = (this.rates[key] || 0) + dt; if (this.rates[key] >= period) { this.rates[key] -= period; if (this.rates[key] > period) this.rates[key] = 0; return true; } return false; }
  cmdVelSub(node, topic = '/cmd_vel') {
    return this.bus.subscribe(node, topic, 'geometry_msgs/msg/Twist', (m, from) => {
      const n = this.bus.nodes.get(from);
      this.cmd = { vx: +m.linear.x || 0, vy: +m.linear.y || 0, wz: +m.angular.z || 0, t: this.t, src: from, kind: n ? (n.user ? 'user' : n.kind) : '' };
      this.event('cmd_vel', this.cmd);
    });
  }
  cmdLive(timeout) { return this.t - this.cmd.t < timeout; }
  teleop(vx, vy, wz, src = '/teleop_panel') {
    if (!this.bus.nodes.has(src)) this.node(src, 'teleop', [['publish_stamped_twist', false, 'bool']]);
    this._tele = { vx, vy, wz, src, last: this.t };
    this.teleSend(src, vx, vy, wz);
  }
  // like teleop_twist_joy: publish_stamped_twist chooses geometry_msgs/msg/Twist or TwistStamped on /cmd_vel
  teleSend(src, vx, vy, wz) {
    const st = !!this.P(src, 'publish_stamped_twist'), type = st ? 'geometry_msgs/msg/TwistStamped' : 'geometry_msgs/msg/Twist';
    const T = this.bus.topics.get('/cmd_vel'); if (!T || T.pubs.get(src) !== type) this.bus.advertise(src, '/cmd_vel', type);
    const tw = { linear: { x: vx, y: vy, z: 0 }, angular: { x: 0, y: 0, z: wz } };
    this.pub(src, '/cmd_vel', type, st ? { header: { stamp: stampNow(), frame_id: 'base_link' }, twist: tw } : tw);
  }
  teleopTick() { const t = this._tele; if (t && (t.vx || t.vy || t.wz) && this.t - t.last > 0.1) { t.last = this.t; this.teleSend(t.src, t.vx, t.vy, t.wz); } }
  tfMsg(list) { const st = stampNow(); return { transforms: list.map(([parent, child, T]) => { const q = K3.quat(T); return { header: { stamp: st, frame_id: parent }, child_frame_id: child, transform: { translation: { x: T[3], y: T[7], z: T[11] }, rotation: q } }; }) }; }
  jointStateMsg(names, q, dq) { return { header: { stamp: stampNow(), frame_id: '' }, name: names.slice(), position: names.map(n => +(q[n] || 0).toFixed(5)), velocity: names.map(n => +((dq && dq[n]) || 0).toFixed(4)), effort: [] }; }
  /* ---------- terminal: ros2 run / launch ---------- */
  pkgs() { return Object.assign({ tf2_ros: ['tf2_echo', 'static_transform_publisher'], teleop_twist_keyboard: ['teleop_twist_keyboard'], topic_tools: ['relay'] }, this.def.pkgs || {}); }
  frames() { return []; }
  lookup() { return null; }
  run(T, pkg, r) {
    const p = (s, c) => T.print(s, c), B = this.bus, PK = this.pkgs();
    if (!pkg) return p('usage: ros2 run <package> <executable>', 'e');
    if (!PK[pkg]) return p(`Package '${pkg}' not found`, 'e');
    const exe = r[0]; if (!exe || !PK[pkg].includes(exe)) return p(exe ? 'No executable found' : 'usage: ros2 run <package> <executable>', 'e');
    const ra = SnoSim.rosArgs(r.slice(1));
    const key = Object.keys(this.procDefs()).find(k => { const d = this.procDefs()[k]; return d.pkg === pkg && d.exe === exe; });
    if (key) return this.runProc(T, key, ra);
    if (exe === 'static_transform_publisher') return this.runStaticTf(T, ra);
    if (pkg === 'topic_tools' && exe === 'relay') return this.runRelay(T, ra);
    if (exe === 'tf2_echo') {
      const [src, dst] = r.slice(1); if (!src || !dst) return p('Usage: tf2_echo source_frame target_frame [echo_rate]', 'e');
      const node = '/tf2_echo'; B.addNode(node, 'cli'); const un = B.subscribe(node, '/tf', 'tf2_msgs/msg/TFMessage', () => {});
      this.event('cmd', 'tf2_echo');
      const iv = setInterval(() => {
        const fr = this.frames();
        if (!fr.includes(src) || !fr.includes(dst)) { p(`[INFO] [${(Date.now() / 1000).toFixed(3)}] [tf2_echo]: Waiting for transform ${src} ->  ${dst}: Invalid frame ID "${fr.includes(src) ? dst : src}" passed to canTransform argument - frame does not exist`, 'w'); return; }
        const M = this.lookup(src, dst); if (!M) return;
        const f3 = (v) => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(3), q = K3.quat(M), e = K3.rpy(M);
        p(`At time ${(() => { const st = stampNow(); return (st.sec + st.nanosec / 1e9).toFixed(9); })()}\n- Translation: [${f3(M[3])}, ${f3(M[7])}, ${f3(M[11])}]\n- Rotation: in Quaternion (xyzw) [${f3(q.x)}, ${f3(q.y)}, ${f3(q.z)}, ${f3(q.w)}]\n- Rotation: in RPY (radian) [${f3(e[0])}, ${f3(e[1])}, ${f3(e[2])}]\n- Rotation: in RPY (degree) [${f3(e[0] * 57.2958)}, ${f3(e[1] * 57.2958)}, ${f3(e[2] * 57.2958)}]`);
      }, 1000);
      T.proc = { name: 'tf2_echo', stop: () => { clearInterval(iv); un(); B.removeNode(node); } };
      return;
    }
    if (exe === 'teleop_twist_keyboard') {
      p(`This node takes keypresses from the keyboard and publishes them\nas Twist messages. (sim: keep the cursor in the input line)\n---------------------------\nMoving around:\n   u    i    o\n   j    k    l\n   m    ,    .\n\nFor Holonomic mode (strafing), hold down the shift key:\n   U    I    O\n   J    K    L\n   M    <    >\n\nanything else : stop\nq/z : increase/decrease max speeds by 10%\nCTRL-C to quit\n\ncurrently:\tspeed ${this.def.teleop[0]}\tturn ${this.def.teleop[1]}`, 'o');
      const node = '/teleop_twist_keyboard'; B.addNode(node, 'teleop'); B.advertise(node, '/cmd_vel', 'geometry_msgs/msg/Twist');
      const st = { sp: this.def.teleop[0], tn: this.def.teleop[1], x: 0, y: 0, w: 0 };
      const send = () => B.publish(node, '/cmd_vel', 'geometry_msgs/msg/Twist', { linear: { x: st.x, y: st.y, z: 0 }, angular: { x: 0, y: 0, z: st.w } });
      const iv = setInterval(() => { if (st.x || st.y || st.w) send(); }, 100);
      const M = { i: [1, 0, 0], o: [1, 0, -1], j: [0, 0, 1], l: [0, 0, -1], u: [1, 0, 1], ',': [-1, 0, 0], '.': [-1, 0, 1], m: [-1, 0, -1], I: [1, 0, 0], O: [1, -1, 0], J: [0, 1, 0], L: [0, -1, 0], U: [1, 1, 0], '<': [-1, 0, 0], '>': [-1, -1, 0], M: [-1, 1, 0] };
      T.proc = {
        name: 'teleop', stop: () => { clearInterval(iv); st.x = st.y = st.w = 0; send(); B.removeNode(node); },
        key: (k, upd) => {
          if (upd) { if (M[k] || M[k.toLowerCase()]) { st.x = st.y = st.w = 0; send(); } return; }
          if (k === 'q' || k === 'z') { const f = k === 'q' ? 1.1 : 0.9; st.sp *= f; st.tn *= f; p(`currently:\tspeed ${st.sp.toFixed(2)}\tturn ${st.tn.toFixed(2)}`); return; }
          const v = M[k]; if (v) { st.x = v[0] * st.sp; st.y = v[1] * st.sp; st.w = v[2] * st.tn; } else { st.x = st.y = st.w = 0; }
          send();
        },
      };
      return;
    }
    return this.runExe(T, pkg, exe, r.slice(1));
  }
  runExe(T) { T.print('This executable is not simulated here.', 'w'); }
  /* ---------- the robot's own nodes, as processes you can stop and start again (ros2 run … --ros-args …) ---------- */
  // procDefs: key -> { pkg, exe, node, kind, params() (the node's own defaults), launch() (what the robot's launch file
  // sets on top), check(k, v), crash (its last words when it dies at start-up), start(rec), stop(rec) }
  procDefs() { return {}; }
  startProc(key, o = {}) {
    const d = this.procDefs()[key]; if (!d) return;
    if (this.procs[key]) this.stopProc(key);
    const rec = this.procs[key] = { key, R: this.remapper(o.remaps), remaps: o.remaps || [], un: [], T: o.T || null };
    if (o.T) this.procTerm[d.node] = o.T; else delete this.procTerm[d.node];
    const given = o.params || {};
    this.node(d.node, d.kind, d.params().map(([k, v, t, range, ro]) => [k, k in given ? given[k] : v, t, range, ro]));
    d.start(rec);
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  stopProc(key) {
    const d = this.procDefs()[key], rec = this.procs[key]; if (!d || !rec) return;
    rec.un.forEach(f => { try { f(); } catch (e) { /* already gone */ } });
    delete this.procs[key]; delete this.procTerm[d.node];
    if (d.stop) d.stop(rec);
    this.bus.removeNode(d.node);
    if (SnoUI.cur === this) SnoUI.refreshTab('control');
  }
  // the robot's launch file: its log header, then each process in order. cfg.<key> = false leaves a node out,
  // an object starts it with { params, remaps } on top of the launch file's values, cfg.died names one that crashes
  launchProcs(keys, cfg = {}) {
    const B = this.bus, D = this.procDefs(), d0 = new Date(), p2 = (v) => String(v).padStart(2, '0');
    const day = `${d0.getFullYear()}-${p2(d0.getMonth() + 1)}-${p2(d0.getDate())}-${p2(d0.getHours())}-${p2(d0.getMinutes())}-${p2(d0.getSeconds())}`;
    keys = keys.filter(k => cfg[k] !== false || cfg.died === k);
    this.launchLabel = {};
    this.log('launch', 'INFO', `All log files can be found below /home/learner/.ros/log/${day}-${this.id}-sim-${B.pid}`, true);
    this.log('launch', 'INFO', 'Default logging verbosity is set to INFO', true);
    const run = keys.map((k, i) => { const d = D[k], label = `${d.exe}-${i + 1}`, pid = ++B.pid; this.launchLabel[d.node] = label; this.log(label, 'INFO', `process started with pid [${pid}]`, true); return [k, d, label, pid]; });
    run.forEach(([k, d, label, pid]) => {
      if (cfg[k] !== false) { const c = cfg[k] && typeof cfg[k] === 'object' ? cfg[k] : {}; this.startProc(k, { remaps: c.remaps, params: Object.assign({}, d.launch ? d.launch() : {}, c.params || {}) }); return; }
      if (d.crash) this.log(label, 'ERROR', d.crash, 'out');
      this.log(label, 'ERROR', `process has died [pid ${pid}, exit code 1, cmd '/opt/ros/jazzy/lib/${d.pkg}/${d.exe} --ros-args -r __node:=${d.node.slice(1)} --params-file /tmp/launch_params_${Math.random().toString(36).slice(2, 10)}'].`, true);
    });
  }
  /* ---------- TF: frames exist only while someone publishes them ---------- */
  noteStatic(node, list) {
    let m = this.tfStatic.get(node); if (!m) this.tfStatic.set(node, m = new Map());
    list.forEach(t => { const tr = t.transform || {}, p = tr.translation || {}, q = tr.rotation || {}; m.set(t.child_frame_id, { parent: (t.header || {}).frame_id, T: tfMat(+p.x || 0, +p.y || 0, +p.z || 0, q) }); });
  }
  staticOf(f) { let hit = null; this.tfStatic.forEach(m => { if (m.has(f)) hit = m.get(f); }); return hit; }
  warnEvery(key, sec, node, text) { const t = (this.warnT = this.warnT || {}); if (t[key] != null && this.t - t[key] < sec) return; t[key] = this.t; this.log(node, 'WARN', text); }
  /* ---------- a camera driver: camera_info on the frame clock, images rendered only when someone wants pixels ---------- */
  // camCfg: { key (its proc), node, w, h, f (focal length, px), fps(), on(), frame(), internal (the robot's own
  // vision nodes: they subscribe like any node but need no pixels, so they don't make the camera render),
  // depth (optional, an RGB-D camera: { on(), dmin, dmax, noise }, depth aligned to the colour image) }.
  // The driver's start() sets this.tn.img and this.tn.info (after remapping), and this.tn.dimg / dinfo for depth.
  camInfo(hd) {
    const C = this.camCfg, f = C.f, cx = (C.w - 1) / 2, cy = (C.h - 1) / 2;
    return { header: hd, height: C.h, width: C.w, distortion_model: 'plumb_bob', d: [0, 0, 0, 0, 0], k: [f, 0, cx, 0, f, cy, 0, 0, 1], r: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0], binning_x: 0, binning_y: 0, roi: { x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false } };
  }
  camTick() {
    const C = this.camCfg; if (!C || !this.procs[C.key] || !this.tn || !this.tn.info || !C.on()) return;
    const fps = clampN(+C.fps() || 10, 1, 30);
    if (this.camLast != null && this.t >= this.camLast && this.t - this.camLast < 1 / fps - 1e-6) return;
    this.camLast = this.t;
    const hd = { stamp: stampNow(), frame_id: C.frame() };
    this.pub(C.node, this.tn.info, 'sensor_msgs/msg/CameraInfo', this.camInfo(hd));
    if (this.camDepthOn()) this.pub(C.node, this.tn.dinfo, 'sensor_msgs/msg/CameraInfo', this.camInfo(hd));
    this.camDue = hd;
    this.onCamFrame(hd);
  }
  onCamFrame() { }
  camRunning() { return !!(this.camCfg && this.procs[this.camCfg.key]); }
  camDepthOn() { const C = this.camCfg; return !!(C && C.depth && this.tn && this.tn.dimg && this.tn.dinfo && C.depth.on()); }
  cameraFrame(view) {
    const hd = this.camDue; if (!hd) return; this.camDue = null;
    const C = this.camCfg; if (!this.camCap) return;
    const wanted = (topic) => { const T = topic && this.bus.topics.get(topic); return !!T && [...T.subs.keys()].some(n => !(C.internal && C.internal.has(n))); };
    if (wanted(this.tn.img)) { const data = view.capture(this.camCap); if (data) this.pub(C.node, this.tn.img, 'sensor_msgs/msg/Image', { header: hd, height: C.h, width: C.w, encoding: 'rgb8', is_bigendian: 0, step: C.w * 3, data }); }
    if (this.camDepthOn() && wanted(this.tn.dimg) && view.captureDepth) {
      const d = view.captureDepth(this.camCap, C.depth); if (!d) return;
      this.pub(C.node, this.tn.dimg, 'sensor_msgs/msg/Image', { header: hd, height: C.h, width: C.w, encoding: '16UC1', is_bigendian: 0, step: C.w * 2, data: new Uint8Array(d.buffer, d.byteOffset, d.byteLength) });
    }
  }
  // would this frame reach that subscriber? Same topic name and type, and QoS they can agree on (else DDS warns once).
  // which: 'img' for the colour image, 'dimg' for the depth image
  camFeeds(node, which = 'img') {
    const C = this.camCfg, B = this.bus, topic = this.tn && this.tn[which], IMG = 'sensor_msgs/msg/Image';
    if (!C || !topic || !this.procs[C.key] || (which === 'dimg' && !this.camDepthOn())) return false;
    const T = B.topics.get(topic); if (!T || T.pubs.get(C.node) !== IMG || T.subs.get(node) !== IMG) return false;
    const bad = qosClash(T.pubQos.get(C.node), T.subQos.get(node)); if (bad) { B.clash(C.node, node, topic, bad); return false; }
    return true;
  }
  // values from -p name:=value are typed like YAML; a double parameter refuses an integer, as rclcpp does
  procParams(key, given) {
    const d = this.procDefs()[key], out = {}, errs = [];
    for (const [k, raw] of Object.entries(given || {})) {
      const decl = d.params().find(x => x[0] === k);
      if (!decl) { out[k] = raw; continue; }
      let v; try { v = parseYaml('[' + raw + ']')[0]; } catch (e) { v = raw; }
      const vt = typeof v === 'boolean' ? 'bool' : typeof v === 'number' ? (/^[-+]?\d+$/.test(String(raw).trim()) ? 'integer' : 'double') : 'string';
      if (decl[2] !== vt) { errs.push(`parameter '${k}' has invalid type: Wrong parameter type, parameter {${k}} is of type {${decl[2]}}, setting it to {${vt}} is not allowed.`); continue; }
      if (d.check) { const why = d.check(k, v); if (why) { errs.push(why); continue; } }
      out[k] = v;
    }
    return { out, errs };
  }
  runProc(T, key, ra) {
    const p = (s, c) => T.print(s, c), d = this.procDefs()[key];
    if (ra.bad.length) return p(`[ERROR] [rcl]: Couldn't parse remap rule or parameter: '${ra.bad[0]}' (write -r from:=to or -p name:=value after --ros-args)`, 'e');
    const pp = this.procParams(key, ra.params);
    if (pp.errs.length) { p(`terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'\n  what():  ${pp.errs[0]}\n[ros2run]: Aborted`, 'e'); return; }
    ra.remaps.filter(([a]) => a.startsWith('__')).forEach(([a]) => p(`[sim] ${a} is not supported here: every node runs in the root namespace with its usual name.`, 'd'));
    const was = this.procs[key];
    if (was) {
      const old = was.T;
      if (old && old !== T && old.proc && old.proc.pk === key) { old.proc = null; old.print(`[sim] ${d.node} was started again from another terminal.`, 'd'); old.ctx.ui.termState(); }
      else if (!old) p(`[sim] Stopped the robot's own ${d.node} so this one can take over. On a real robot you would first stop the launch file that started it (Ctrl+C).`, 'd');
    }
    this.stopProc(key);
    this.startProc(key, { remaps: ra.remaps, params: pp.out, T });
    // (pk, not key: a proc's key() is the teleop keyboard handler)
    T.proc = { name: d.exe, pk: key, stop: () => { if (this.procs[key] && this.procs[key].T === T) { this.stopProc(key); T.print(`[INFO] [${stampSec()}] [rclcpp]: signal_handler(signum=2)`, 'd'); } } };
    this.event('run', { key, remaps: ra.remaps, params: pp.out });
  }
  // tf2_ros static_transform_publisher: Jazzy's --x … --frame-id … style, or the old positional style
  runStaticTf(T, ra) {
    const p = (s, c) => T.print(s, c), a = ra.pos, o = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 }; let q = null, parent = '', child = '', old = false;
    const num = (v, k) => { const n = parseFloat(v); if (!Number.isFinite(n)) throw new Error(`argument --${k}: expected a number, got '${v}'`); return n; };
    try {
      if (a.length && !a[0].startsWith('--')) {
        old = true; const n = a.length;
        if (n === 8) { [o.x, o.y, o.z, o.yaw, o.pitch, o.roll] = a.slice(0, 6).map((v, i) => num(v, ['x', 'y', 'z', 'yaw', 'pitch', 'roll'][i])); [parent, child] = a.slice(6); }
        else if (n === 9) { [o.x, o.y, o.z] = a.slice(0, 3).map(v => num(v, 'x')); q = a.slice(3, 7).map(v => num(v, 'q')); [parent, child] = a.slice(7); }
        else throw new Error('expected x y z yaw pitch roll frame_id child_frame_id');
      } else {
        for (let i = 0; i < a.length; i++) {
          const k = a[i].replace(/^--/, ''), v = a[++i];
          if (['x', 'y', 'z', 'roll', 'pitch', 'yaw'].includes(k)) o[k] = num(v, k);
          else if (['qx', 'qy', 'qz', 'qw'].includes(k)) { q = q || [0, 0, 0, 1]; q[['qx', 'qy', 'qz', 'qw'].indexOf(k)] = num(v, k); }
          else if (k === 'frame-id') parent = v || ''; else if (k === 'child-frame-id') child = v || '';
          else throw new Error(`unrecognized arguments: ${a[i - 1]}`);
        }
      }
      if (!parent || !child) throw new Error('the following arguments are required: --frame-id, --child-frame-id');
      if (parent === child) throw new Error(`frame-id and child-frame-id cannot be the same ('${parent}')`);
    } catch (e) { return p(`usage: static_transform_publisher [--x X] [--y Y] [--z Z] [--qx QX] [--qy QY] [--qz QZ] [--qw QW] [--roll ROLL] [--pitch PITCH] [--yaw YAW] --frame-id FRAME_ID --child-frame-id CHILD_FRAME_ID\nstatic_transform_publisher: error: ${e.message}`, 'e'); }
    if (!q) { const cr = Math.cos(o.roll / 2), sr = Math.sin(o.roll / 2), cp = Math.cos(o.pitch / 2), sp = Math.sin(o.pitch / 2), cy = Math.cos(o.yaw / 2), sy = Math.sin(o.yaw / 2); q = [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]; }
    const qn = Math.hypot(...q) || 1; q = q.map(v => v / qn);
    const id = Array.from({ length: 16 }, () => 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'[Math.floor(Math.random() * 62)]).join('');
    const node = '/static_transform_publisher_' + id, B = this.bus;
    B.addNode(node, 'tf'); B.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true; B.advertise(node, '/tf_static', 'tf2_msgs/msg/TFMessage', { dur: 'TRANSIENT_LOCAL', depth: 1 });
    if (old) p(`[WARN] [${stampSec()}] [${node.slice(1)}]: Old-style arguments are deprecated; see --help for new-style arguments`, 'w');
    const f6 = (v) => `'${v.toFixed(6)}'`;
    p(`[INFO] [${stampSec()}] [${node.slice(1)}]: Spinning until stopped - publishing transform\ntranslation: (${[o.x, o.y, o.z].map(f6).join(', ')})\nrotation: (${q.map(f6).join(', ')})\nfrom '${parent}' to '${child}'`);
    B.publish(node, '/tf_static', 'tf2_msgs/msg/TFMessage', { transforms: [{ header: { stamp: stampNow(), frame_id: parent }, child_frame_id: child, transform: { translation: { x: o.x, y: o.y, z: o.z }, rotation: { x: q[0], y: q[1], z: q[2], w: q[3] } } }] });
    T.proc = { name: 'static_transform_publisher', stop: () => B.removeNode(node) };
    this.event('static_tf', { parent, child, xyz: [o.x, o.y, o.z], q });
  }
  // topic_tools relay: subscribe to one topic and republish every message on another
  runRelay(T, ra) {
    const p = (s, c) => T.print(s, c), B = this.bus, [inT, outT] = ra.pos;
    if (!inT || !outT) return p('usage: ros2 run topic_tools relay <intopic> <outtopic>', 'e');
    const full = (n) => n.startsWith('/') ? n : '/' + n, src = full(inT), dst = full(outT);
    if (src === dst) return p('[ERROR] [relay]: input and output topics must differ', 'e');
    let node = '/relay', k = 1; while (B.nodes.has(node)) node = '/relay_' + (++k);
    B.addNode(node, 'relay');
    let un = null, iv = null;
    const hook = () => {
      const ty = B.types(src).find(t => [...(B.topics.get(src) || { pubs: new Map() }).pubs.values()].includes(t)); if (!ty) return false;
      B.advertise(node, dst, ty); un = B.subscribe(node, src, ty, (m) => B.publish(node, dst, ty, m));
      p(`[INFO] [${stampSec()}] [${node.slice(1)}]: Relaying ${src} -> ${dst} (${ty})`); return true;
    };
    if (!hook()) { p(`[INFO] [${stampSec()}] [${node.slice(1)}]: Waiting for a publisher on ${src} to learn its type…`, 'd'); iv = setInterval(() => { if (hook()) { clearInterval(iv); iv = null; } }, 400); }
    T.proc = { name: 'relay', stop: () => { if (iv) clearInterval(iv); if (un) un(); B.removeNode(node); } };
    this.event('relay', { src, dst, node });
  }
  launch(T, pkg, r) {
    const p = (s, c) => T.print(s, c), PK = this.pkgs();
    if (!pkg || !r[0]) return p('usage: ros2 launch <package> <launch_file> [args...]', 'e');
    if (!PK[pkg]) return p(`Package '${pkg}' not found: "package '${pkg}' not found, searching: ['/opt/ros/jazzy']"`, 'e');
    const file = r[0]; if (!PK[pkg].includes(file)) return p(`file '${file}' was not found in the share directory of package '${pkg}'`, 'e');
    const args = Object.fromEntries(r.slice(1).filter(x => x.includes(':=')).map(x => x.split(':=')));
    const b = this.behaviors()[file]; if (!b) return p(`[ERROR] [launch]: ${file} is not simulated`, 'e');
    if (b.on()) return p(`[ERROR] [launch]: ${b.label} is already running (switch it off in the Control tab or press Ctrl+C where you launched it)`, 'e');
    const pid = ++this.bus.pid;
    p(`[INFO] [launch]: All log files can be found below /home/learner/.ros/log/2026-11-04-${this.id}\n[INFO] [launch]: Default logging verbosity is set to INFO`, 'd');
    const err = b.set(true, args); if (err) return p(`[ERROR] [launch]: ${err}`, 'e');
    b.nodes.forEach((n, i) => p(`[INFO] [${n.slice(1)}-${i + 1}]: process started with pid [${pid + i}]`));
    if (b.hello) p(b.hello, 'o');
    this.event('launch', file);
    T.proc = { name: 'launch', stop: () => b.set(false), after: `[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)\n` + b.nodes.map((n, i) => `[INFO] [${n.slice(1)}-${i + 1}]: process has finished cleanly [pid ${pid + i}]`).join('\n') };
    SnoUI.refreshTab('control');
  }
  behaviors() { return {}; }
  /* ---------- missions ---------- */
  get mstore() { return (S.sno.m[this.id] = S.sno.m[this.id] || {}); }
  done(id) { return !!this.mstore[id]; }
  current() { return this.def.missions.find(m => !this.done(m.id)); }
  complete(id) {
    if (this.done(id)) return;
    const m = this.def.missions.find(x => x.id === id); if (!m) return;
    this.mstore[id] = Date.now(); addXP(m.xp); S.gears += 5; const nd = bumpStreak(); save(); checkBadges(); save();
    Sound.play('done'); confetti(80);
    toast(`<b>${esc(this.def.name)}: ${esc(m.title)}</b><br>+${m.xp} XP · +5 gears${nd ? ` · ${S.streak}-day streak` : ''}`, { mood: 'happy', ms: 3400 });
    SnoUI.renderMissionBar(); SnoUI.refreshTab('missions'); App.renderStats();
  }
  event() { }
  /* ---------- Python console ---------- */
  pyLog(text, cls = 'info') {
    String(text).replace(/\n$/, '').split('\n').forEach(l => this.pyLines.push([l, cls]));
    if (this.pyLines.length > 400) this.pyLines.splice(0, 100);
    const pc = SnoUI.cur === this && SnoUI.root ? $('.pyconsole', SnoUI.root) : null;
    if (pc) { String(text).replace(/\n$/, '').split('\n').forEach(l => pc.append(h('div', { class: cls }, l || ' '))); while (pc.childElementCount > 400) pc.firstChild.remove(); pc.scrollTop = pc.scrollHeight; }
  }
  pyStatus() { const el = SnoUI.cur === this && SnoUI.root ? $('.py-status', SnoUI.root) : null; if (!el) return; const mine = Py.host === this.pyHost; el.className = 'py-status ' + (mine ? Py.disp || '' : ''); el.textContent = mine ? Py.text : (Py.phase === 'running' ? 'Python is busy in another simulator' : 'Python not started'); }
  markErr(l) { this.errLine = l; if (this.paintCode && SnoUI.cur === this && this.tab === 'code') this.paintCode(); }
  clearErr() { this.errLine = 0; if (this.paintCode && SnoUI.cur === this && this.tab === 'code') this.paintCode(); }
  /* ---------- mouse perturbation (spring between the cursor and the picked body) ---------- */
  grabStart(hit) {
    const ud = hit.obj.userData; let sys = null, b = 0;
    if (ud.link && this.rb) { sys = this.rb; b = this.rb.linkBody[ud.link]; if (b == null) return false; }
    else if (ud.obj && ud.obj.sys) sys = ud.obj.sys;
    if (!sys) return false;
    this.grab = { sys, b, local: K3.apply(K3.inv(sys.T[b]), hit.p[0], hit.p[1], hit.p[2]), target: hit.p.slice() }; sys.asleep = false;
    this.event('grab'); return true;
  }
  grabMove(t) { if (this.grab) this.grab.target = t; }
  grabEnd() { this.grab = null; }
  grabCtl() {
    const g = this.grab; if (!g) return;
    const Sy = g.sys, p = K3.apply(Sy.T[g.b], g.local[0], g.local[1], g.local[2]), v = Sy.ptVel(g.b, p), m = Math.max(0.03, Sy.totalMass), k = 120 * m, c = 2 * Math.sqrt(k * m) * 0.8;
    const F = [k * (g.target[0] - p[0]) - c * v[0], k * (g.target[1] - p[1]) - c * v[1], k * (g.target[2] - p[2]) - c * v[2]];
    Sy.push(g.b, g.local, F); g.F = Math.hypot(F[0], F[1], F[2]); Sy.asleep = false;
  }
  /* ---------- domain randomization (Physics tab) ---------- */
  drDefaults() { return { mu: 1, mass: 0, motor: 1, noise: 0 }; }
  applyDR() {
    const d = this.dr || (this.dr = this.drDefaults()); if (!this.world) return;
    this.world.muScale = d.mu; this.noise = { imu: d.noise * 0.4, odom: d.noise };
    if (this.rb) { this.rb.str.fill(d.motor); this.applyPayload(d.mass); }
  }
  applyPayload() { }
  /* ---------- default hooks ---------- */
  setupRos() { }
  step() { }
  reset() { }
  hud() { return ''; }
  focusPos() { return [0, 0, 0.2]; }
  buildScene() { }
  sync() { }
  onKey() { return false; }
  tickTab() { }
  renderTab() { }
  onHide() { }
}

/* ------------------------------ generic panels shared by every robot ------------------------------ */
const SnoPanels = {
  term(sim, body) { if (sim.fix) SnoFix.strip(sim, body); sim.tp.mount(body); },
  fix(sim, body) { SnoFix.panel(sim, body); },
  vision(sim, body) { SnoVision.panel(sim, body); },
  code(sim, body) {
    const tpls = sim.def.py, key = sim.id; let tpl = S.sno.tpl[key] || Object.keys(tpls)[0];
    if (!tpls[tpl]) tpl = Object.keys(tpls)[0];
    if (!S.sno.code[key]) S.sno.code[key] = tpls[tpl].code;
    body.innerHTML = `<div class="code-tools"><label class="sr" for="sno-tpl">Template</label><select class="sel" id="sno-tpl">${Object.entries(tpls).map(([k, t]) => `<option value="${k}"${k === tpl ? ' selected' : ''}>${esc(t.label)}</option>`).join('')}</select>
      <button class="btn btn-sm btn-ghost" data-a="load">Load template</button><span class="grow"></span><span class="py-status" aria-live="polite"></span>
      <button class="btn btn-sm btn-good" data-a="run">${icon('play')}Run</button><button class="btn btn-sm" data-a="stop">${icon('stop')}Stop</button></div>
      <div class="editor"><div class="gutter" aria-hidden="true"><div class="gi"></div></div><div class="ed"><pre aria-hidden="true"></pre><textarea spellcheck="false" autocapitalize="off" autocomplete="off" wrap="off" aria-label="Python code editor"></textarea></div></div>
      <div class="pyconsole" role="log" aria-live="polite" aria-label="Python console"></div>`;
    const ta = $('textarea', body), pre = $('pre', body), gi = $('.gi', body), sel = $('#sno-tpl', body);
    ta.value = S.sno.code[key];
    const sync = () => { pre.style.transform = `translate(${-ta.scrollLeft}px, ${-ta.scrollTop}px)`; gi.style.transform = `translateY(${-ta.scrollTop}px)`; };
    const paint = () => { const lines = ta.value.split('\n'); pre.innerHTML = lines.map(l => hlLine(l, 'py')).join('\n') + '\n '; gi.innerHTML = lines.map((_, i) => `<div class="${sim.errLine === i + 1 ? 'err' : ''}">${i + 1}</div>`).join(''); sync(); };
    let saveT = 0;
    const ins = (txt) => { const s = ta.selectionStart, en = ta.selectionEnd; ta.value = ta.value.slice(0, s) + txt + ta.value.slice(en); ta.selectionStart = ta.selectionEnd = s + txt.length; ta.dispatchEvent(new Event('input')); return true; };
    ta.addEventListener('input', () => { sim.errLine = 0; paint(); clearTimeout(saveT); saveT = setTimeout(() => { S.sno.code[key] = ta.value; save({ local: true }); }, 600); });
    ta.addEventListener('scroll', sync);
    const run = () => { S.sno.code[key] = ta.value; Py.bind(sim.pyHost); Py.run(ta.value); sim.pyStatus(); };
    ta.addEventListener('keydown', (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); run(); return; }
      if (e.key === 'Tab' && !e.shiftKey) { e.preventDefault(); document.execCommand('insertText', false, '    ') || ins('    '); }
      if (e.key === 'Enter') { const s = ta.selectionStart, before = ta.value.slice(0, s), line = before.slice(before.lastIndexOf('\n') + 1); const ind = (line.match(/^\s*/) || [''])[0] + (/:\s*$/.test(line) ? '    ' : ''); e.preventDefault(); document.execCommand('insertText', false, '\n' + ind) || ins('\n' + ind); }
    });
    const load = () => { S.sno.tpl[key] = sel.value; ta.value = tpls[sel.value].code; S.sno.code[key] = ta.value; save({ local: true }); sim.errLine = 0; paint(); };
    $('[data-a="load"]', body).addEventListener('click', () => { if (ta.value !== tpls[S.sno.tpl[key] || tpl].code) modal({ mood: 'think', title: 'Replace your code?', body: 'Loading a template replaces what is in the editor.', actions: [{ label: 'Load template', cls: 'btn-primary', onClick: load }, { label: 'Cancel', cls: 'btn-ghost' }] }); else load(); });
    sel.addEventListener('change', () => { if (ta.value === tpls[S.sno.tpl[key] || tpl].code) load(); });
    $('[data-a="run"]', body).addEventListener('click', run);
    $('[data-a="stop"]', body).addEventListener('click', () => { if (Py.host === sim.pyHost) Py.stop(); });
    const pc = $('.pyconsole', body); sim.paintCode = paint;
    sim.pyLines.forEach(([t, c]) => pc.append(h('div', { class: c }, t)));
    if (!sim.pyLines.length) sim.pyLog(sim.def.pyHint || 'Pick a template, press Run (or Ctrl+Enter) and watch the robot.', 'sys');
    pc.scrollTop = pc.scrollHeight; paint(); sim.pyStatus();
  },
  topics(sim, body) {
    body.innerHTML = `<div class="sno-seg-row"><div class="seg" role="group" aria-label="View"><button data-v="list" aria-pressed="${sim.topicView === 'list'}">${icon('topics')}Topics</button><button data-v="graph" aria-pressed="${sim.topicView === 'graph'}">${icon('graph')}Graph</button></div><span class="sub" style="color:var(--muted);font-size:12.5px">live from the simulated ROS graph</span></div><div class="tv"></div>`;
    $$('.seg button', body).forEach(b => b.addEventListener('click', () => { sim.topicView = b.dataset.v; SnoUI.renderTab(); }));
    const tv = $('.tv', body);
    if (sim.topicView === 'graph') { tv.innerHTML = `<div class="graph-box" aria-live="off"></div><div class="graph-legend"><span><i style="background:var(--c-node-soft);border:2px solid var(--c-node)"></i>node</span><span><i style="background:var(--c-topic-soft);border:2px solid var(--c-topic)"></i>topic</span><span><i style="background:var(--c-msg)"></i>message flowing</span></div>`; sim.graphSig = ''; SnoPanels.graph(sim); return; }
    tv.innerHTML = `<div style="overflow-x:auto"><table class="ttable"><thead><tr><th scope="col">Topic</th><th scope="col">Type</th><th scope="col" class="hz">Hz</th><th scope="col" class="hz">Pub/Sub</th></tr></thead><tbody></tbody></table></div><div class="echo-box" aria-live="off">Select a topic to see its latest message, like <code>ros2 topic echo</code>.</div>`;
    SnoPanels.topicsLive(sim);
  },
  topicsLive(sim) {
    const root = SnoUI.root; if (!root) return;
    if (sim.topicView === 'graph') return SnoPanels.graph(sim);
    const tb = $('.ttable tbody', root); if (!tb) return;
    const B = sim.bus, ts = [...B.topics.values()].sort((a, b) => a.name.localeCompare(b.name));
    tb.innerHTML = '';
    ts.forEach(t => { const tr = h('tr', { class: sim.selTopic === t.name ? 'sel' : '', tabindex: '0', html: `<td>${esc(t.name)}</td><td>${esc(t.type || '?')}</td><td class="hz">${B.hz(t.name).toFixed(1)}</td><td class="hz">${t.pubs.size}/${t.subs.size}</td>` }); const pick = () => { sim.selTopic = t.name; SnoPanels.topicsLive(sim); sim.event('cmd', 'topic select'); }; tr.addEventListener('click', pick); tr.addEventListener('keydown', (e) => { if (e.key === 'Enter') pick(); }); tb.append(tr); });
    const eb = $('.echo-box', root), T = B.topics.get(sim.selTopic);
    if (T && eb) eb.textContent = T.last && IFACES[T.lastType] ? `$ ros2 topic echo ${T.name}\n` + yamlMsg(T.lastType, T.last).slice(0, 80).join('\n') + '\n---' : `${T.name}: no messages yet`;
  },
  graph(sim) {
    const box = SnoUI.root && $('.graph-box', SnoUI.root); if (!box) return;
    const B = sim.bus, main = new Set(sim.def.mainNodes);
    const nodes = [...B.nodes.values()].filter(n => !n.hidden && (n.pubs.size || n.subs.size));
    const topics = [...B.topics.values()].filter(t => !HIDE_TOPICS.has(t.name) && (t.pubs.size || t.subs.size)).sort((a, b) => a.name.localeCompare(b.name));
    const left = nodes.filter(n => !main.has(n.name)).sort((a, b) => a.name.localeCompare(b.name)), right = nodes.filter(n => main.has(n.name));
    const rows = Math.max(left.length, topics.length, right.length, 2), H = Math.max(260, rows * 46 + 50);
    const G = [], E = [], ly = (i, n) => H / 2 + (i - (n - 1) / 2) * 46;
    left.forEach((n, i) => G.push(GN(n.name, n.name, 92, ly(i, left.length), 'node', { w: 176 })));
    right.forEach((n, i) => G.push(GN(n.name, n.name, 448, ly(i, right.length), 'node', { w: 150 })));
    topics.forEach((t, i) => G.push(GN('T' + t.name, t.name, 268, ly(i, topics.length), 'topic', { w: 150 })));
    const has = (id) => G.some(g => g.id === id);
    topics.forEach(t => {
      t.pubs.forEach((_, n) => { if (has(n)) E.push({ a: n, b: 'T' + t.name, flow: B.active(t.name, n) ? 'msg' : null, dur: 1.4, n: 2 }); });
      t.subs.forEach((_, n) => { if (has(n)) E.push({ a: 'T' + t.name, b: n, flow: B.active(t.name) && [...t.pubs.keys()].some(p => p !== n) ? 'msg' : null, dur: 1.4, n: 2 }); });
    });
    const sig = JSON.stringify([G.map(g => g.id), E.map(e => e.a + e.b + (e.flow ? 1 : 0))]);
    if (sig === sim.graphSig) return; sim.graphSig = sig;
    box.innerHTML = graphSVG({ vw: 530, vh: H, nodes: G, edges: E, reveal: false, alt: `Live ROS graph of the ${sim.def.name} simulator` });
  },
  // Physics tab: engine status, overlays, scene style, domain randomization and a live plot
  phys(sim, body) {
    if (!sim.world) { body.append(h('p', { class: 'lpad sidecap' }, 'No physics in this simulator.')); return; }
    const V = S.sno.viz = S.sno.viz || {}, d = sim.dr || (sim.dr = sim.drDefaults()), w = h('div', { class: 'lpad sno-phys' });
    w.append(h('div', { class: 'phys-stats', html: '<div><span>timestep</span><b data-p="h"></b></div><div><span>real-time</span><b data-p="rtf"></b></div><div><span>contacts</span><b data-p="nc"></b></div><div><span>physics cost</span><b data-p="cpu"></b></div><div><span>render quality</span><b data-p="q"></b></div>' }));
    w.append(h('h4', { html: 'Show <span>like MuJoCo\'s visualization flags</span>' }));
    [['contacts', 'Contact points and forces', 'Red dots where bodies touch; yellow lines show the contact force.'], ['com', 'Centre of mass and support polygon', 'Yellow ball: whole-body centre of mass. Green outline: hull of the ground contacts. Outside it, the robot tips.'], ['shapes', 'Collision shapes', 'The simplified boxes, capsules and spheres the physics actually uses.']].forEach(([k, l, sub]) => w.append(snoSwitch(l, !!V[k], (on) => { V[k] = on; save({ local: true }); sim.event('viz', k); }, sub)));
    w.append(h('h4', { html: 'Scene <span>look only, same physics</span>' }));
    w.append(snoSeg([['mujoco', 'MuJoCo'], ['isaac', 'Isaac'], ['classic', 'Classic']], SnoView.preset(), (v) => { S.sno.scene = v; save({ local: true }); if (SnoUI.view) SnoUI.view.applyTheme(); SnoUI.renderTab(); sim.event('scene', v); }, 'Scene style'));
    w.append(snoSwitch('Floor reflections', SnoView.reflect(), (on) => { S.sno.noReflect = !on; save({ local: true }); if (SnoUI.view) SnoUI.view.applyTheme(); }, 'MuJoCo-style mirror floor. Turn off for speed on slower devices.'));
    w.append(snoSwitch('Always full quality', !!S.sno.fullQ, (on) => { S.sno.fullQ = on; save({ local: true }); if (on && SnoUI.view) SnoUI.view.setQuality(0); }, 'Off: the view renders fewer pixels on its own when frames are slow, so the physics keeps real time.'));
    w.append(h('h4', { html: 'Domain randomization <span>vary the world, like Isaac Lab events</span>' }));
    const sl = h('div', { class: 'sliders' }), rows = sim.drRows ? sim.drRows() : [['mu', 'friction ×', 0.3, 1.5, 0.05], ['mass', 'payload kg', 0, sim.rb ? +(sim.rb.totalMass * 0.3).toFixed(1) : 1, sim.rb && sim.rb.totalMass > 5 ? 0.5 : 0.05], ['motor', 'motor strength ×', 0.4, 1.3, 0.05], ['noise', 'sensor noise', 0, 1, 0.05]];
    rows.forEach(([k, l, lo, hi, st]) => { const id = 'dr' + (++UID), r = h('div', { class: 'slider', html: `<label for="${id}">${l}</label><input id="${id}" type="range" min="${lo}" max="${hi}" step="${st}" value="${d[k]}"><output>${(+d[k]).toFixed(2)}</output>` }); const inp = $('input', r); inp.addEventListener('input', () => { d[k] = +inp.value; $('output', r).textContent = (+inp.value).toFixed(2); sim.applyDR(); sim.event('dr', k); }); sl.append(r); });
    w.append(sl);
    const bb = h('div', { class: 'row-btns' }), rb = h('button', { class: 'btn btn-sm btn-primary', html: icon('sparkle') + 'Randomize' }), db = h('button', { class: 'btn btn-sm btn-ghost' }, 'Defaults');
    rb.addEventListener('click', () => { rows.forEach(([k, , lo, hi, st]) => { d[k] = +(lo + Math.round(Math.random() * (hi - lo) / st) * st).toFixed(3); }); if (d.mu < 0.5) d.mu = 0.5; sim.applyDR(); sim.event('dr', 'random'); SnoUI.renderTab(); toast('New world: friction ×' + d.mu.toFixed(2) + ', payload ' + d.mass.toFixed(1) + ' kg, motors ×' + d.motor.toFixed(2) + '. Does the controller still cope?', { mood: 'think', ms: 3200 }); });
    db.addEventListener('click', () => { sim.dr = sim.drDefaults(); sim.applyDR(); SnoUI.renderTab(); });
    bb.append(rb, db); w.append(bb);
    w.append(h('h4', { html: `Live plot <span>${esc((sim.plotSpec ? sim.plotSpec() : { title: '' }).title)}</span>` }), h('canvas', { class: 'pd-plot phys-plot', width: 560, height: 150, 'aria-label': 'Live physics plot' }), h('div', { class: 'phys-legend' }));
    w.append(h('p', { class: 'sidecap', html: sim.physDoc ? sim.physDoc() : '' }));
    body.append(w); SnoPanels.physLive(sim);
  },
  physLive(sim) {
    const root = SnoUI.root, w = root && $('.sno-phys', root); if (!w || !sim.world) return;
    const set = (k, v) => { const e = $(`[data-p="${k}"]`, w); if (e) e.textContent = v; };
    set('h', (sim.world.h * 1000).toFixed(2) + ' ms'); set('rtf', sim.paused ? 'paused' : '×' + (sim.rtf || 1).toFixed(2)); set('nc', String(sim.world.nc)); set('cpu', ((sim.cpu || 0) * 100).toFixed(0) + '% of a core'); set('q', !SnoUI.view ? 'no 3D' : S.sno.fullQ ? 'full (locked)' : ['full', 'reduced (auto)', 'low (auto)'][SnoUI.view.ql || 0]);
    const spec = sim.plotSpec ? sim.plotSpec() : null, cv = $('.phys-plot', w); if (!spec || !cv) return;
    const P = sim.pbuf || [], g = cv.getContext('2d'), W = cv.width, H = cv.height, cols = ['#5EF2D6', '#F5B700', '#FF6B6B', '#8AB4FF'];
    g.fillStyle = cssVar('--term-bg') || '#10201A'; g.fillRect(0, 0, W, H);
    $('.phys-legend', w).innerHTML = spec.labels.map((l, i) => `<span><i style="background:${cols[i]}"></i>${esc(l)}</span>`).join('');
    if (P.length < 2) return;
    let lo = spec.lo != null ? spec.lo : Infinity, hi = spec.hi != null ? spec.hi : -Infinity; P.forEach(p => p[1].forEach(v => { if (spec.lo == null) lo = Math.min(lo, v); if (spec.hi == null) hi = Math.max(hi, v); }));
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const t0 = P[P.length - 1][0] - 6, X = (t) => (t - t0) / 6 * W, Y = (v) => H - 10 - (v - lo) / (hi - lo) * (H - 22);
    g.strokeStyle = 'rgba(255,255,255,.1)'; g.lineWidth = 1; [0.25, 0.5, 0.75].forEach(f => { g.beginPath(); g.moveTo(0, H * f); g.lineTo(W, H * f); g.stroke(); });
    spec.labels.forEach((_, k) => { g.strokeStyle = cols[k]; g.lineWidth = 2; g.beginPath(); P.forEach((p, i) => { const x = X(p[0]), y = Y(p[1][k]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); });
    g.font = '12px JetBrains Mono, monospace'; g.fillStyle = 'rgba(255,255,255,.6)'; g.fillText(hi.toFixed(1) + ' ' + (spec.unit || ''), 6, 14); g.fillText(lo.toFixed(1), 6, H - 4);
  },
  missions(sim, body) {
    const list = h('div', { class: 'missions' }), cur = sim.current();
    sim.def.missions.forEach((m, i) => {
      const done = sim.done(m.id);
      const b = h('button', { class: 'mission' + (done ? ' done' : '') + (cur === m ? ' active' : ''), html: `<span class="num">${done ? icon('check') : i + 1}</span><span><b>${esc(m.title)}</b><span>${md(m.text)}</span></span><span class="xp">${done ? 'Done' : '+' + m.xp + ' XP'}</span>` });
      b.addEventListener('click', () => SnoUI.focusTab(m.tab)); list.append(b);
    });
    body.append(list);
  },
  info(sim, body) {
    const d = sim.def;
    body.innerHTML = `<div class="lpad sno-info">
      <div class="spec-grid">${d.specs.map(([k, v]) => `<div><span>${esc(k)}</span><b>${esc(v)}</b></div>`).join('')}</div>
      <h3>ROS interfaces in this simulator</h3>
      <ul class="iface-list">${d.ifaces.map(([n, k, t, w]) => `<li><div class="ih"><code class="in">${esc(n)}</code><span class="ik k-${esc(k.split(' ')[0])}">${esc(k)}</span></div><div class="it">${esc(t)}</div><div class="iw">${md(w)}</div></li>`).join('')}</ul>
      ${d.jointDoc ? `<h3>Joints</h3><p class="sidecap" style="margin-top:4px">${md(d.jointDoc)}</p>` : ''}
      <h3>How close is this to the real robot?</h3><p class="sidecap" style="margin-top:4px">${md(d.realism)}</p>
      <p class="sno-disclaimer">${SNO_DISCLAIMER}</p></div>`;
  },
};
const SNO_DISCLAIMER = 'Unofficial educational simulators made for learning. Not affiliated with or endorsed by Hiwonder, Unitree, AgileX or Elephant Robotics. Models are simplified and behaviour is approximate: always check the vendor documentation before running code on real hardware.';

/* small reusable controls */
function snoJoystick(el, onMove, onEnd) {
  const knob = $('.knob', el); let drag = false;
  const move = (e) => { const r = el.getBoundingClientRect(), R = r.width / 2 - 20; let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2); const L = Math.hypot(dx, dy); if (L > R) { dx *= R / L; dy *= R / L; } knob.style.transform = `translate(${dx}px, ${dy}px)`; onMove(-dy / R, -dx / R); };
  el.addEventListener('pointerdown', (e) => { drag = true; el.setPointerCapture(e.pointerId); move(e); });
  el.addEventListener('pointermove', (e) => { if (drag) move(e); });
  const end = () => { if (!drag) return; drag = false; knob.style.transform = ''; onEnd(); };
  el.addEventListener('pointerup', end); el.addEventListener('pointercancel', end);
}
function snoHold(btn, down, up) {
  const d = (e) => { e.preventDefault(); btn.classList.add('on'); down(); };
  const u = () => { if (!btn.classList.contains('on')) return; btn.classList.remove('on'); up(); };
  btn.addEventListener('pointerdown', d); btn.addEventListener('pointerup', u); btn.addEventListener('pointerleave', u); btn.addEventListener('pointercancel', u);
}
function snoSeg(items, cur, onPick, label) {
  const g = h('div', { class: 'seg', role: 'group', 'aria-label': label || '' });
  items.forEach(([v, lab, ic]) => { const b = h('button', { 'aria-pressed': String(v === cur), html: (ic ? icon(ic) : '') + esc(lab) }); b.addEventListener('click', () => onPick(v)); g.append(b); });
  return g;
}
function snoSwitch(label, on, onToggle, sub) {
  const id = 'sw' + (++UID);
  const row = h('label', { class: 'sno-switch', for: id, html: `<span><b>${esc(label)}</b>${sub ? `<span>${sub}</span>` : ''}</span>` });
  const inp = h('input', { type: 'checkbox', id, role: 'switch' }); inp.checked = !!on; inp.addEventListener('change', () => onToggle(inp.checked));
  row.append(inp, h('i', { 'aria-hidden': 'true' })); return row;
}

/* ------------------------------ hub + page shell ------------------------------ */
const SNO_ART = {
  ainex: `<svg viewBox="0 0 200 150" aria-hidden="true"><ellipse cx="100" cy="140" rx="46" ry="6" fill="currentColor" opacity=".12"/><rect x="84" y="20" width="32" height="24" rx="6" fill="#2B3236"/><circle cx="100" cy="32" r="6" fill="#5EF2D6"/><circle cx="100" cy="32" r="2.6" fill="#0B1820"/><rect x="96" y="44" width="8" height="5" fill="#8E989E"/><rect x="78" y="49" width="44" height="40" rx="7" fill="#CDD4D8"/><rect x="88" y="56" width="24" height="16" rx="4" fill="#F26419"/><rect x="64" y="51" width="13" height="13" rx="3" fill="#1E2326"/><rect x="123" y="51" width="13" height="13" rx="3" fill="#1E2326"/><rect x="66" y="64" width="9" height="22" rx="3" fill="#B9C2C8"/><rect x="125" y="64" width="9" height="22" rx="3" fill="#B9C2C8"/><rect x="65" y="86" width="11" height="9" rx="2" fill="#1E2326"/><rect x="124" y="86" width="11" height="9" rx="2" fill="#1E2326"/><rect x="86" y="89" width="11" height="10" rx="2" fill="#1E2326"/><rect x="103" y="89" width="11" height="10" rx="2" fill="#1E2326"/><rect x="87" y="99" width="9" height="15" rx="2" fill="#B9C2C8"/><rect x="104" y="99" width="9" height="15" rx="2" fill="#B9C2C8"/><rect x="86" y="114" width="11" height="9" rx="2" fill="#1E2326"/><rect x="103" y="114" width="11" height="9" rx="2" fill="#1E2326"/><rect x="87" y="123" width="9" height="10" rx="2" fill="#B9C2C8"/><rect x="104" y="123" width="9" height="10" rx="2" fill="#B9C2C8"/><rect x="80" y="133" width="19" height="6" rx="2" fill="#16191B"/><rect x="101" y="133" width="19" height="6" rx="2" fill="#16191B"/></svg>`,
  r1: `<svg viewBox="0 0 200 150" aria-hidden="true"><ellipse cx="100" cy="142" rx="40" ry="5" fill="currentColor" opacity=".12"/><rect x="88" y="6" width="24" height="24" rx="9" fill="#E9EEF0" stroke="#8E989E"/><rect x="91" y="12" width="18" height="9" rx="4.5" fill="#0B1820"/><circle cx="96" cy="16.5" r="1.8" fill="#5EF2D6"/><circle cx="104" cy="16.5" r="1.8" fill="#5EF2D6"/><rect x="96" y="30" width="8" height="5" fill="#2B3236"/><path d="M80 36h40l-4 34H84z" fill="#E9EEF0" stroke="#8E989E"/><rect x="92" y="44" width="16" height="12" rx="3" fill="#2B3236"/><circle cx="76" cy="40" r="6" fill="#2B3236"/><circle cx="124" cy="40" r="6" fill="#2B3236"/><rect x="71" y="44" width="10" height="26" rx="5" fill="#E9EEF0" stroke="#8E989E"/><rect x="119" y="44" width="10" height="26" rx="5" fill="#E9EEF0" stroke="#8E989E"/><rect x="72" y="70" width="8" height="24" rx="4" fill="#2B3236"/><rect x="120" y="70" width="8" height="24" rx="4" fill="#2B3236"/><rect x="85" y="70" width="30" height="10" rx="4" fill="#2B3236"/><rect x="86" y="80" width="11" height="30" rx="5" fill="#E9EEF0" stroke="#8E989E"/><rect x="103" y="80" width="11" height="30" rx="5" fill="#E9EEF0" stroke="#8E989E"/><rect x="87" y="110" width="9" height="26" rx="4" fill="#2B3236"/><rect x="104" y="110" width="9" height="26" rx="4" fill="#2B3236"/><rect x="82" y="135" width="17" height="6" rx="3" fill="#16191B"/><rect x="101" y="135" width="17" height="6" rx="3" fill="#16191B"/></svg>`,
  limo: `<svg viewBox="0 0 200 150" aria-hidden="true"><ellipse cx="100" cy="122" rx="70" ry="6" fill="currentColor" opacity=".12"/><rect x="40" y="64" width="120" height="40" rx="8" fill="#2B3236"/><rect x="46" y="58" width="108" height="12" rx="4" fill="#8E989E"/><rect x="130" y="72" width="26" height="12" rx="3" fill="#0B1820"/><circle cx="137" cy="78" r="3" fill="#5EF2D6"/><circle cx="148" cy="78" r="3" fill="#5EF2D6"/><rect x="104" y="44" width="30" height="14" rx="3" fill="#1E2326"/><rect x="108" y="40" width="22" height="6" rx="3" fill="#F26419"/><circle cx="64" cy="104" r="16" fill="#16191B"/><circle cx="64" cy="104" r="6" fill="#B9C2C8"/><circle cx="136" cy="104" r="16" fill="#16191B"/><circle cx="136" cy="104" r="6" fill="#B9C2C8"/></svg>`,
  cobot: `<svg viewBox="0 0 200 150" aria-hidden="true"><ellipse cx="96" cy="132" rx="70" ry="6" fill="currentColor" opacity=".12"/><rect x="36" y="76" width="120" height="38" rx="8" fill="#2B3236"/><rect x="42" y="70" width="108" height="11" rx="4" fill="#8E989E"/><rect x="128" y="84" width="24" height="11" rx="3" fill="#0B1820"/><circle cx="60" cy="114" r="15" fill="#16191B"/><circle cx="60" cy="114" r="5.5" fill="#B9C2C8"/><circle cx="132" cy="114" r="15" fill="#16191B"/><circle cx="132" cy="114" r="5.5" fill="#B9C2C8"/><rect x="70" y="56" width="22" height="16" rx="4" fill="#F6F8F9" stroke="#8E989E"/><path d="M81 56L92 26" stroke="#F6F8F9" stroke-width="11" stroke-linecap="round"/><path d="M81 56L92 26" stroke="#8E989E" stroke-width="1.5" stroke-linecap="round" opacity=".6"/><circle cx="81" cy="56" r="6" fill="#2B3236"/><circle cx="92" cy="26" r="6" fill="#2B3236"/><path d="M92 26L128 34" stroke="#F6F8F9" stroke-width="9" stroke-linecap="round"/><circle cx="128" cy="34" r="5" fill="#2B3236"/><path d="M128 34v14" stroke="#2B3236" stroke-width="6"/><path d="M122 48v10M134 48v10" stroke="#2B3236" stroke-width="3.5" stroke-linecap="round"/><rect x="123" y="56" width="10" height="10" rx="2" fill="#E5484D"/></svg>`,
};
const SnoUI = {
  sims: {}, cur: null, root: null, view: null, raf: 0, last: 0, acc: 0, uiTimer: 0, keysBound: false,
  get(id) { if (!this.sims[id]) this.sims[id] = SNO_DEFS[id].make(SNO_DEFS[id]); return this.sims[id]; },
  mountHub(el) {
    const v = h('div', { class: 'view view-wide sno-hub' });
    v.innerHTML = `<div class="view-head"><div><h1>SnoBots Robots</h1><p>Virtual twins of the team's robots, running on real physics: mass, torque-limited motors, contacts and friction. Walk, drive, pick, and program them with ROS 2, right in your browser.</p></div></div>`;
    [['Humanoids', 'Biped robots with servo joints, walking gaits and cameras.', ['ainex', 'r1']], ['Autonomous cars', 'A four-mode mobile base with lidar, plus a version that carries a 6-axis arm.', ['limo', 'cobot']]].forEach(([title, sub, ids]) => {
      const sec = h('section', { class: 'sno-sec', 'aria-label': title, html: `<h2>${title}</h2><p class="sub">${sub}</p>` });
      const grid = h('div', { class: 'sno-grid' });
      ids.forEach(id => {
        const d = SNO_DEFS[id], m = S.sno.m[id] || {}, n = Object.keys(m).filter(k => d.missions.some(x => x.id === k)).length;
        const c = h('button', { class: 'sno-card', style: `--k:${d.color}`, 'aria-label': `Open the ${d.name} simulator` });
        c.innerHTML = `<div class="art">${SNO_ART[id]}</div><div class="body"><div class="maker">${esc(d.maker)}</div><h3>${esc(d.name)}</h3><p>${esc(d.blurb)}</p><div class="chips">${d.chipsHub.map(x => `<span class="chip">${esc(x)}</span>`).join('')}</div>
          <div class="prog"><div class="bar"><i style="width:${Math.round(n / d.missions.length * 100)}%"></i></div><span class="tabnum">${n}/${d.missions.length} missions</span></div><span class="open">Open simulator ${icon('arrowR')}</span></div>`;
        c.addEventListener('click', () => App.go('snobots-' + id));
        grid.append(c);
      });
      sec.append(grid); v.append(sec);
    });
    v.append(h('p', { class: 'sno-disclaimer', html: SNO_DISCLAIMER }));
    el.append(v);
  },
  mount(el, id) {
    this.unmount();
    const sim = this.get(id); sim.start(); this.cur = sim; this.root = el; const d = sim.def; STAMP_CLOCK = () => sim.t;
    el.innerHTML = '';
    const v = h('div', { class: 'lab sno-page', style: `--k:${d.color}` });
    v.innerHTML = `<div class="lab-head"><div class="sno-title"><button class="icon-btn sno-back" aria-label="Back to SnoBots Robots">${icon('arrowL')}</button><div><div class="maker">${esc(d.maker)} · ${esc(d.catLabel)}</div><h1>${esc(d.name)} <span class="sim-tag">simulator</span></h1></div></div><div class="topstats" data-stats></div></div>
      <div class="mission-bar" aria-live="polite"></div>
      <div class="lab-grid">
        <div><div class="arena-wrap sno-view" tabindex="0" aria-label="${esc(d.name)} 3D view. Drag to orbit, scroll or pinch to zoom, right-drag to pan.${d.keysHint ? ' ' + esc(d.keysHint) : ''}"><div class="hud"></div><div class="pip-label" hidden></div><div class="sno-imshow" hidden></div><div class="sno-load"><div class="spin"></div><p>Loading the 3D view…</p></div></div>
          <div class="arena-tools"></div></div>
        <div class="lab-panel"><div class="ltabs" role="tablist" aria-label="${esc(d.name)} tools"></div><div class="lbody"></div></div>
      </div>`;
    el.append(v);
    $('.sno-back', v).addEventListener('click', () => App.go('snobots'));
    this.wrap = $('.sno-view', v); this.hudEl = $('.hud', v); this.pipLabel = $('.pip-label', v);
    this.view = new SnoView(this.wrap, d.view);
    this.renderTools(); this.renderMissionBar(); this.renderTabs(); this.renderTab();
    this.load3D();
    this.last = performance.now(); this.acc = 0; cancelAnimationFrame(this.raf); this.raf = requestAnimationFrame((t) => this.frame(t));
    clearInterval(this.uiTimer); this.uiTimer = setInterval(() => this.tick(), 250);
    if (!this.keysBound) { this.keysBound = true; window.addEventListener('keydown', (e) => this.onKey(e, false)); window.addEventListener('keyup', (e) => this.onKey(e, true)); window.addEventListener('blur', () => { if (this.cur) this.cur.onKey(null, true, true); }); }
    App.renderStats();
  },
  load3D() {
    const sim = this.cur, view = this.view, ld = $('.sno-load', this.root);
    ld.innerHTML = '<div class="spin"></div><p>Loading the 3D view…</p>'; ld.hidden = false;
    SnoThree.load().then(T => {
      if (this.cur !== sim || this.view !== view) return;
      if (!view.init(T)) { ld.innerHTML = `${icon('warn')}<p><b>3D graphics are turned off in this browser.</b><br>The robot still runs: use the Control, Terminal, Code and Topics tabs.</p>`; return; }
      sim.buildScene(view, T); sim.sync(view, T); ld.hidden = true;
      if (sim.world) { view.onGrab = (hit) => sim.grabStart(hit); view.onDrag = (p) => sim.grabMove(p); view.onRelease = () => sim.grabEnd(); view.grabMode = !!sim.grabTool; }
      view.onFollow = () => this.renderTools();
      if (view.pips.length) this.pipLabel.hidden = !view.pipOn;
    }).catch(() => {
      if (this.cur !== sim) return;
      ld.innerHTML = `${icon('warn')}<p><b>The 3D engine could not be downloaded.</b><br>Check your connection and try again. The robot itself keeps running: the Control, Terminal, Code and Topics tabs work without 3D.</p>`;
      const b = h('button', { class: 'btn btn-sm btn-primary' }, 'Try again'); b.addEventListener('click', () => this.load3D()); ld.append(b);
    });
  },
  unmount() {
    cancelAnimationFrame(this.raf); this.raf = 0; clearInterval(this.uiTimer);
    if (this.view) this.view.dispose(); this.view = null;
    if (this.cur) { this.cur.onKey(null, true, true); if (Py.host === this.cur.pyHost && Py.phase === 'running') Py.stop(); if (this.cur.tabOff) { const off = this.cur.tabOff; this.cur.tabOff = null; off(); } this.cur.onHide(); }
    this.cur = null; this.root = null; STAMP_CLOCK = null;
  },
  frame(t) {
    if (!this.cur || !this.root) return;
    const sim = this.cur, raw = Math.max(0, (t - this.last) / 1000), dt = Math.min(0.1, raw); this.last = Math.max(this.last, t);
    const h0 = 1 / (sim.def.hz || 60); let n = 0;
    this.acc += sim.paused ? 0 : dt * (sim.timeScale || 1);
    const w0 = performance.now();
    while (this.acc >= h0 && n < 6) { sim.step(h0); this.acc -= h0; n++; }
    if (sim.stepOnce) { sim.step(h0); sim.stepOnce = 0; n++; }
    if (this.acc > h0) this.acc = 0;
    if (n && Py.host === sim.pyHost) Py.clock(sim.pyHost, sim.t);
    // real-time factor: simulated seconds per wall second (1.0 = real time)
    const R = sim.rtfAcc || (sim.rtfAcc = { sim: 0, wall: 0, cpu: 0 }); R.sim += n * h0; R.wall += dt; R.cpu += performance.now() - w0;
    if (R.wall >= 0.5) { sim.rtf = R.sim / R.wall; sim.cpu = R.cpu / (R.wall * 1000); R.sim = 0; R.wall = 0; R.cpu = 0; }
    const v = this.view;
    // adaptive quality: when most frames take longer than 1/40 s for 3 s, render fewer pixels, then skip the mirror pass
    if (v && v.ready && !sim.paused && !document.hidden && !S.sno.fullQ) {
      const Q = this.qa || (this.qa = { t: 0, slow: 0 }), fr = Math.min(raw, 1); Q.t += fr; if (fr > 0.025) Q.slow += fr;
      if (Q.t >= 3) { if (Q.slow > 0.6 * Q.t && (v.ql || 0) < 2) v.setQuality((v.ql || 0) + 1); Q.t = 0; Q.slow = 0; }
    }
    if (v && v.ready) {
      try { sim.sync(v, v.T); v.setTarget(sim.focusPos(), sim.focusYaw ? sim.focusYaw() : null); v.render(); if (sim.cameraFrame) sim.cameraFrame(v); } catch (e) { console.error(e); v.ready = false; }
      if (v.pips.length) { const p = v.pips[v.pipIdx], R = v.pipRect(); this.pipLabel.hidden = !v.pipOn; if (v.pipOn) { Object.assign(this.pipLabel.style, { right: (v.w - R.x - R.w - 2) + 'px', top: (R.y - 2) + 'px', width: (R.w + 4) + 'px', height: (R.h + 4) + 'px' }); const lb = p.label + (p.depth ? ' · depth' : ''); if (this.pipLabel.dataset.l !== lb) { this.pipLabel.dataset.l = lb; this.pipLabel.innerHTML = `<span>${esc(lb)}</span>`; } } }
    }
    this.raf = requestAnimationFrame((tt) => this.frame(tt));
  },
  tick() {
    const sim = this.cur; if (!sim || !this.root) return;
    this.hudEl.innerHTML = sim.hud();
    if (sim.tab === 'topics') SnoPanels.topicsLive(sim);
    if (sim.tab === 'phys') SnoPanels.physLive(sim);
    if (sim.tab === 'vision') SnoVision.tick(sim);
    sim.tickTab(sim.tab);
  },
  renderTools() {
    const sim = this.cur, bar = this.root && $('.arena-tools', this.root); if (!bar) return;
    const v = this.view; bar.innerHTML = '';
    const grp = (label) => { const g = h('div', { class: 'tgrp', role: 'group', 'aria-label': label }); bar.append(g); return g; };
    // icon-only buttons keep their name for screen readers and as a tooltip
    const tb = (label, ic, pressed, fn, title, into, ico) => { const b = h('button', { class: 'btn btn-sm' + (ico ? ' btn-ico' : ''), html: icon(ic) + (ico ? '' : esc(label)), title: title || label, 'aria-label': ico ? label : null }); if (pressed != null) b.setAttribute('aria-pressed', String(pressed)); b.addEventListener('click', fn); (into || bar).append(b); return b; };
    tb('Reset', 'replay', null, () => { sim.reset(); Sound.play('tap'); sim.event('reset'); }, 'Reset the robot and the world');
    if (sim.world) {
      const g = grp('Simulation time');
      tb(sim.paused ? 'Play' : 'Pause', sim.paused ? 'play' : 'pause', sim.paused || false, () => { sim.paused = !sim.paused; sim.event('pause', sim.paused); this.renderTools(); }, (sim.paused ? 'Resume' : 'Pause') + ' the physics (Space)', g, true);
      const st = tb('Step', 'arrowR', null, () => { sim.paused = true; sim.stepOnce = 1; this.renderTools(); }, 'Advance one control step (1/60 s) while paused', g, true); st.disabled = !sim.paused;
      tb((sim.timeScale || 1) + '×', 'clock', null, () => { const L = [1, 0.5, 0.25]; sim.timeScale = L[(L.indexOf(sim.timeScale || 1) + 1) % L.length]; this.renderTools(); }, 'Simulation speed: ' + (sim.timeScale || 1) + '× real time', g);
      tb('Grab', 'tools', !!sim.grabTool, () => { sim.grabTool = !sim.grabTool; if (v) v.grabMode = sim.grabTool; this.renderTools(); }, 'Drag a robot part or an object to push or lift it (or hold Ctrl and drag)');
    }
    const c = grp('Camera');
    tb('Follow', 'target', v ? v.follow : true, () => { if (v) { v.follow = !v.follow; this.renderTools(); } }, 'Keep the camera on the robot', c);
    tb('Chase', 'axes', v ? !!v.chase : false, () => { if (v) { v.chase = !v.chase; if (v.chase) v.follow = true; this.renderTools(); } }, 'Swing the camera behind the robot as it turns', c);
    if (sim.def.cams) tb('Camera', 'camera', v ? v.pipOn : true, () => { if (v) { if (v.pips.length > 1 && v.pipOn) { v.pipIdx = (v.pipIdx + 1) % (v.pips.length + 1); if (v.pipIdx === v.pips.length) { v.pipOn = false; v.pipIdx = 0; } } else v.pipOn = !v.pipOn; this.renderTools(); } }, 'Show the robot camera view', c);
    (sim.tools ? sim.tools() : []).forEach(([label, ic, pressed, fn, title]) => tb(label, ic, pressed, () => { fn(); this.renderTools(); }, title));
    bar.append(h('span', { class: 'grow' }), h('span', { class: 'kbd-hint', html: sim.def.kbd }));
  },
  renderMissionBar() {
    const sim = this.cur, bar = this.root && $('.mission-bar', this.root); if (!bar) return;
    if (sim.fix) return SnoFix.bar(sim, bar);
    const m = sim.current(), n = sim.def.missions.filter(x => sim.done(x.id)).length;
    if (!m) { bar.innerHTML = `${icon('trophy')}<div class="grow"><div class="mt">All ${sim.def.missions.length} ${esc(sim.def.name)} missions complete!</div><div class="mh">Try writing your own node in the Code tab, or visit another SnoBots robot.</div></div>`; return; }
    bar.innerHTML = `${icon('flag')}<div class="grow"><div class="mt">Mission ${sim.def.missions.indexOf(m) + 1} of ${sim.def.missions.length}: ${esc(m.title)} <span class="chip" style="margin-left:6px">+${m.xp} XP</span></div><div class="mh">${md(m.text)}</div></div><span class="sub tabnum" style="color:var(--muted);font-size:13px">${n} done</span>`;
    const b = h('button', { class: 'btn btn-sm btn-primary' }, 'Show me'); b.addEventListener('click', () => this.focusTab(m.tab)); bar.append(b);
  },
  renderTabs() {
    const sim = this.cur, bar = this.root && $('.ltabs', this.root); if (!bar) return; bar.innerHTML = '';
    sim.def.tabs.forEach(([id, ic, label]) => {
      const left = id === 'missions' ? sim.def.missions.filter(m => !sim.done(m.id)).length : id === 'fix' ? SnoFix.left(sim) : id === 'vision' ? SnoVision.left(sim) : 0;
      const b = h('button', { class: 'ltab', role: 'tab', id: 'st-' + id, 'aria-selected': String(sim.tab === id), 'aria-controls': 'sno-panel', html: icon(ic) + label + (left ? `<span class="cnt">${left}</span>` : '') });
      b.addEventListener('click', () => { sim.tab = id; this.renderTabs(); this.renderTab(); }); bar.append(b);
    });
  },
  renderTab() {
    const sim = this.cur, body = this.root && $('.lbody', this.root); if (!body) return;
    body.innerHTML = ''; body.id = 'sno-panel'; body.setAttribute('role', 'tabpanel'); body.setAttribute('aria-labelledby', 'st-' + sim.tab);
    sim.paintCode = null; if (sim.tabOff) { const off = sim.tabOff; sim.tabOff = null; try { off(); } catch (e) { console.error(e); } }
    if (SnoPanels[sim.tab]) SnoPanels[sim.tab](sim, body); else sim.renderTab(sim.tab, body);
  },
  focusTab(t, graph) { const sim = this.cur; if (!sim) return; if (graph) sim.topicView = 'graph'; if (!sim.def.tabs.some(x => x[0] === t)) t = sim.def.tabs[0][0]; sim.tab = t; this.renderTabs(); this.renderTab(); },
  refreshTab(t) { if (this.cur && this.root && this.cur.tab === t) { this.renderTabs(); this.renderTab(); } },
  onKey(e, up) {
    const sim = this.cur; if (!sim || !this.root || !App.view.startsWith('snobots-') || $('.player') || $('.modal-back') || $('.drawer')) return;
    const T = sim.tp && sim.tp.term(), tag = (e.target.tagName || '').toLowerCase();
    if (T && T.proc && T.proc.key && e.target.classList && e.target.classList.contains('lterm-input')) { if (e.key.length === 1) { e.preventDefault(); T.proc.key(e.key, up); } return; }
    if (tag === 'input' || tag === 'textarea' || tag === 'select' || e.metaKey || e.ctrlKey || e.altKey) return;
    const inSim = this.root.contains(document.activeElement) || document.activeElement === document.body;
    if (!inSim) return;
    if (e.key === ' ' && sim.world && tag !== 'button') { if (!up) { sim.paused = !sim.paused; this.renderTools(); } e.preventDefault(); return; }
    if (sim.onKey(e, up)) e.preventDefault();
  },
};
