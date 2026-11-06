/* ============================== animation stage ============================== */
let UID = 0;
const ANIM = {};
function mkCtx(stage) {
  const timers = new Set(), loops = [], cleanups = [];
  const ctx = {
    paused: false, stage,
    after(ms, fn) { const id = setTimeout(() => { timers.delete(id); fn(); }, ms); timers.add(id); return id; },
    every(ms, fn) { const id = setInterval(() => { if (!ctx.paused) fn(); }, ms); timers.add(id); return id; },
    loop(fn) {
      const L = { id: 0, last: performance.now(), t: 0 };
      const step = (now) => { const dt = Math.max(0, Math.min(0.05, (now - L.last) / 1000)); L.last = Math.max(L.last, now); if (!ctx.paused) { L.t += dt; fn(dt, L.t); } L.id = requestAnimationFrame(step); };
      L.id = requestAnimationFrame(step); loops.push(L);
    },
    onDestroy(fn) { cleanups.push(fn); },
    destroy() {
      timers.forEach(id => { clearTimeout(id); clearInterval(id); }); timers.clear();
      loops.forEach(L => cancelAnimationFrame(L.id));
      cleanups.forEach(f => { try { f(); } catch (e) { /* ignore */ } });
    },
  };
  return ctx;
}
function mountAnim(stage, scene) {
  const ctx = mkCtx(stage);
  stage.innerHTML = '';
  const fn = ANIM[scene.a];
  if (fn) { try { fn(stage, scene, ctx); } catch (e) { console.error(e); stage.innerHTML = '<div class="st-pad">This animation could not start.</div>'; } }
  return ctx;
}
function setPaused(stage, ctx, p) {
  ctx.paused = p; stage.classList.toggle('paused', p);
  stage.querySelectorAll('svg').forEach(s => { try { p ? s.pauseAnimations() : s.unpauseAnimations(); } catch (e) { /* ignore */ } });
}

/* ---------- svg building blocks ---------- */
const FLOWCLS = { msg: 'pkt', req: 'pkt pkt-srv', res: 'pkt pkt-res', tf: 'pkt pkt-tf' };
function edgePath(x1, y1, x2, y2, curve = 0) {
  if (!curve) return `M${x1.toFixed(1)} ${y1.toFixed(1)} L${x2.toFixed(1)} ${y2.toFixed(1)}`;
  const mx = (x1 + x2) / 2, my = (y1 + y2) / 2, dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy) || 1;
  return `M${x1.toFixed(1)} ${y1.toFixed(1)} Q${(mx - dy / len * curve).toFixed(1)} ${(my + dx / len * curve).toFixed(1)} ${x2.toFixed(1)} ${y2.toFixed(1)}`;
}
function edgeMid(x1, y1, x2, y2, curve = 0) {
  const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
  if (!curve) return [mx, my];
  const dx = x2 - x1, dy = y2 - y1, len = Math.hypot(dx, dy) || 1, cx = mx - dy / len * curve, cy = my + dx / len * curve;
  return [0.25 * x1 + 0.5 * cx + 0.25 * x2, 0.25 * y1 + 0.5 * cy + 0.25 * y2];
}
function envelope(cls = 'pkt', label = '') {
  return `<rect class="${cls}" x="-10" y="-7" width="20" height="14" rx="2.6"/><path d="M-9 -5.5L0 1L9 -5.5" fill="none" stroke="rgba(0,0,0,.35)" stroke-width="1.3"/>${label ? `<text class="t-s" y="-12" text-anchor="middle" style="font-weight:600;fill:var(--ink)">${esc(label)}</text>` : ''}`;
}
function flowDots(d, o = {}) {
  const n = o.n || 2, dur = o.dur || 2.2, cls = o.cls || 'pkt', delay = o.delay || 0;
  let s = '';
  for (let i = 0; i < n; i++) {
    const b = (delay + i * dur / n).toFixed(2);
    const body = o.shape === 'dot' ? `<circle class="${cls}" r="6"/>` : envelope(cls);
    s += `<g opacity="0">${body}<animateMotion dur="${dur}s" begin="${b}s" repeatCount="${o.once ? 1 : 'indefinite'}" path="${d}" fill="freeze"/><animate attributeName="opacity" values="0;1;1;0" keyTimes="0;.1;.88;1" dur="${dur}s" begin="${b}s" repeatCount="${o.once ? 1 : 'indefinite'}" fill="freeze"/></g>`;
  }
  return s;
}
function nodeSize(n) {
  const len = String(n.label).length;
  if (n.kind === 'topic') return { w: n.w || Math.max(92, len * 8.6 + 26), h: n.h || 38 };
  if (n.kind === 'hw' || n.kind === 'box' || n.kind === 'srv' || n.kind === 'param') return { w: n.w || Math.max(100, len * 9.2 + 30), h: n.h || 46 };
  return { w: n.w || Math.max(100, len * 9.4 + 30), h: n.h || 44 };
}
function clipBox(n, tx, ty) {
  const { w, h } = nodeSize(n); const dx = tx - n.x, dy = ty - n.y;
  if (!dx && !dy) return [n.x, n.y];
  const s = Math.min((w / 2 + 4) / Math.abs(dx || 1e-9), (h / 2 + 4) / Math.abs(dy || 1e-9));
  return [n.x + dx * s, n.y + dy * s];
}
function nodeMarkup(n, delay, pick) {
  const { w, h } = nodeSize(n), kind = n.kind || 'node';
  const rx = (kind === 'node' || kind === 'bad') ? h / 2 : kind === 'topic' ? 8 : 12;
  const cls = { node: 'node', topic: 'topic', srv: 'srv', param: 'param', box: 'box', hw: 'hw', bad: 'node bad' }[kind] || 'node';
  const inner = `<rect x="${n.x - w / 2}" y="${n.y - h / 2}" width="${w}" height="${h}" rx="${rx}"/>
    <text class="${kind === 'topic' ? 't-m' : 't-b'}" x="${n.x}" y="${n.y + 5}" text-anchor="middle">${esc(n.label)}</text>
    ${n.sub ? `<text class="t-s" x="${n.x}" y="${n.y + h / 2 + 18}" text-anchor="middle">${esc(n.sub)}</text>` : ''}`;
  const body = n.pulse ? `<g class="pulse">${inner}</g>` : inner;
  const st = delay ? `style="animation-delay:${delay}s"` : '';
  const pk = pick ? ` pickable" tabindex="0" role="button" aria-label="${esc(n.label)}` : '';
  return `<g class="${cls}${delay ? ' pop-in' : ''}${pk}" data-id="${esc(n.id)}" ${st}>${body}</g>`;
}
function graphSVG(o) {
  const W = o.vw || 600, H = o.vh || 340, uid = ++UID;
  const N = {}; o.nodes.forEach(n => { N[n.id] = n; });
  const mk = (id, c) => `<marker id="${id}${uid}" viewBox="0 0 10 10" refX="8.6" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="${c}"/></marker>`;
  const defs = `<defs>${mk('ar', 'fill-muted')}${mk('arb', 'fill-bad')}${mk('arg', 'fill-good')}</defs>`;
  const groups = (o.groups || []).map(g => `<g class="group${g.at != null ? ' appear' : ''}" ${g.at != null ? `style="animation-delay:${g.at}s"` : ''}><rect x="${g.x}" y="${g.y}" width="${g.w}" height="${g.h}" rx="18"/><text class="t-s" x="${g.x + 14}" y="${g.y + 22}" style="fill:var(--c-topic);font-weight:600">${esc(g.label)}</text></g>`).join('');
  let edges = '', flows = '';
  (o.edges || []).forEach((e, i) => {
    const A = N[e.a], B = N[e.b]; if (!A || !B) return;
    const [x1, y1] = clipBox(A, B.x, B.y), [x2, y2] = clipBox(B, A.x, A.y);
    const d = edgePath(x1, y1, x2, y2, e.curve || 0);
    const delay = o.reveal ? (e.at != null ? e.at : 0.3 + i * 0.35) : (e.at || 0);
    const cls = 'edge' + (e.dash ? ' dash' : '') + (e.bad ? ' bad' : '') + (e.good ? ' good' : '') + (delay ? ' appear' : '');
    const m = e.bad ? 'arb' : e.good ? 'arg' : 'ar';
    edges += `<path class="${cls}" ${delay ? `style="animation-delay:${delay}s"` : ''} data-a="${e.a}" data-b="${e.b}" d="${d}" ${e.noarrow ? '' : `marker-end="url(#${m}${uid})"`}/>`;
    if (e.label) { const [mx, my] = edgeMid(x1, y1, x2, y2, e.curve || 0); edges += `<text class="t-s${delay ? ' appear' : ''}" ${delay ? `style="animation-delay:${delay}s"` : ''} x="${mx}" y="${my - 9}" text-anchor="middle">${esc(e.label)}</text>`; }
    if (e.flow) flows += `<g class="flow" data-a="${e.a}" data-b="${e.b}">${flowDots(d, { n: e.n || 2, dur: e.dur || 2.2, cls: FLOWCLS[e.flow], delay: delay + 0.3, shape: e.shape })}</g>`;
  });
  const nodes = o.nodes.map((n, i) => nodeMarkup(n, o.reveal ? (n.at != null ? n.at : i * 0.22) : (n.at || 0), o.pick)).join('');
  const notes = (o.notes || []).map(t => `<text class="${t.cls || 't-s'}${t.at != null ? ' appear' : ''}" ${t.at != null ? `style="animation-delay:${t.at}s"` : ''} x="${t.x}" y="${t.y}" text-anchor="${t.anchor || 'middle'}">${esc(t.text)}</text>`).join('');
  return `<svg class="st" viewBox="0 0 ${W} ${H}" role="img" aria-label="${esc(o.alt || 'Diagram of ROS nodes and topics')}">${defs}${groups}${edges}${flows}${nodes}${notes}${o.extra || ''}</svg>`;
}
/* compact constructors used in course content */
const GN = (id, label, x, y, kind = 'node', more = {}) => Object.assign({ id, label, x, y, kind }, more);
const GE = (a, b, flow = 'msg', more = {}) => Object.assign({ a, b, flow }, more);
function arrowLine(x1, y1, x2, y2, axis, w = 4) {
  const a = Math.atan2(y2 - y1, x2 - x1), L = 11, W = 6.5;
  const bx = x2 - L * Math.cos(a), by = y2 - L * Math.sin(a);
  const p1 = [bx + W * Math.sin(a), by - W * Math.cos(a)], p2 = [bx - W * Math.sin(a), by + W * Math.cos(a)];
  return `<line x1="${x1.toFixed(1)}" y1="${y1.toFixed(1)}" x2="${bx.toFixed(1)}" y2="${by.toFixed(1)}" class="ax-${axis}" stroke-width="${w}" stroke-linecap="round"/><path d="M${x2.toFixed(1)} ${y2.toFixed(1)}L${p1[0].toFixed(1)} ${p1[1].toFixed(1)}L${p2[0].toFixed(1)} ${p2[1].toFixed(1)}z" class="f${axis}"/>`;
}
function triad(x, y, th, len = 46, label = '', showZ = false) {
  const c = Math.cos(th), s = Math.sin(th);
  let out = arrowLine(x, y, x + len * c, y - len * s, 'x') + arrowLine(x, y, x - len * s, y - len * c, 'y');
  if (showZ) out += `<circle cx="${x}" cy="${y}" r="7" class="fill-surface ax-z" stroke-width="3"/><circle cx="${x}" cy="${y}" r="2.5" class="fz"/>`;
  else out += `<circle cx="${x}" cy="${y}" r="4" class="fill-ink"/>`;
  if (label) out += `<text class="t-s" x="${x - 8}" y="${y + 20}" text-anchor="end" style="font-weight:600;fill:var(--ink)">${esc(label)}</text>`;
  return out;
}
function robotTop(scale = 1) {
  return `<g transform="scale(${scale})"><rect x="-24" y="-31" width="20" height="9" rx="3" class="fill-ink"/><rect x="-24" y="22" width="20" height="9" rx="3" class="fill-ink"/><rect x="-32" y="-24" width="62" height="48" rx="15" class="fill-surface stroke-ink" stroke-width="3"/><circle r="9" class="fill-s3 stroke-ink" stroke-width="2.4"/><path d="M15 -8L27 0L15 8z" class="fill-accent"/></g>`;
}
function makeCanvas(w, hgt) {
  const c = document.createElement('canvas'), dpr = Math.min(2, window.devicePixelRatio || 1);
  c.width = w * dpr; c.height = hgt * dpr; c.style.width = '100%'; c.style.height = 'auto'; c.style.aspectRatio = `${w} / ${hgt}`;
  const g = c.getContext('2d'); g.scale(dpr, dpr);
  return { c, g, w, h: hgt };
}
function drawTurtle(g, x, y, th, s = 1, shell = '#2F9E5B', label) {
  g.save(); g.translate(x, y); g.rotate(-th); g.scale(s, s);
  g.fillStyle = '#9CD35F';
  [[8, -10], [8, 10], [-8, -10], [-8, 10]].forEach(([lx, ly]) => { g.beginPath(); g.ellipse(lx, ly, 5, 3.4, 0, 0, 6.29); g.fill(); });
  g.beginPath(); g.ellipse(16, 0, 6, 5, 0, 0, 6.29); g.fill();
  g.beginPath(); g.moveTo(-13, 0); g.lineTo(-19, -2.4); g.lineTo(-19, 2.4); g.fill();
  g.fillStyle = shell; g.beginPath(); g.ellipse(0, 0, 13.5, 11, 0, 0, 6.29); g.fill();
  g.strokeStyle = 'rgba(0,0,0,.28)'; g.lineWidth = 1.6; g.beginPath(); g.ellipse(0, 0, 7.5, 6, 0, 0, 6.29); g.stroke();
  g.beginPath(); g.moveTo(-7.5, 0); g.lineTo(-13, 0); g.moveTo(7.5, 0); g.lineTo(13, 0); g.stroke();
  g.fillStyle = '#10261B'; g.beginPath(); g.arc(18.5, -2.2, 1.3, 0, 6.29); g.arc(18.5, 2.2, 1.3, 0, 6.29); g.fill();
  g.restore();
  if (label) { g.save(); g.font = '600 12px Lexend, system-ui, sans-serif'; g.fillStyle = '#fff'; g.textAlign = 'center'; g.fillText(label, x, y - 20 * s); g.restore(); }
}
const TSIM = { W: 11.09, bg: '#3F55E6', trail: 'rgba(214,220,255,.9)' };
function wrapAngle(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

/* ============================== templates ============================== */
ANIM.graph = (stage, o) => { stage.innerHTML = graphSVG(Object.assign({ reveal: true }, o)); };

ANIM.robotParts = (stage, o) => {
  const parts = [
    { id: 'camera', label: 'Camera', x: 176, y: 102, node: '/camera_driver' },
    { id: 'lidar', label: 'Lidar', x: 155, y: 66, node: '/lidar_driver' },
    { id: 'arm', label: 'Arm', x: 262, y: 140, node: '/arm_controller' },
    { id: 'wheels', label: 'Wheels', x: 205, y: 270, node: '/motor_driver' },
    { id: 'battery', label: 'Battery', x: 120, y: 228, node: '/battery_monitor' },
  ];
  const nodesMode = o.mode === 'nodes';
  let s = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A robot and its parts">
  <rect x="68" y="196" width="176" height="62" rx="16" class="fill-surface stroke-ink" stroke-width="3"/>
  <rect x="92" y="214" width="56" height="28" rx="6" class="fill-s2 stroke-ink" stroke-width="2.5"/>
  <rect x="98" y="220" width="10" height="16" rx="2" class="fill-good"/><rect x="111" y="220" width="10" height="16" rx="2" class="fill-good"/><rect x="124" y="220" width="10" height="16" rx="2" class="fill-good"/>
  <g><circle cx="104" cy="268" r="27" class="fill-ink"/><circle cx="104" cy="268" r="10" class="fill-s3"/><g class="spin"><path d="M104 248v40M84 268h40" class="stroke-muted" stroke-width="3"/></g></g>
  <g><circle cx="208" cy="268" r="27" class="fill-ink"/><circle cx="208" cy="268" r="10" class="fill-s3"/><g class="spin"><path d="M208 248v40M188 268h40" class="stroke-muted" stroke-width="3"/></g></g>
  <rect x="130" y="124" width="52" height="76" rx="10" class="fill-s2 stroke-ink" stroke-width="3"/>
  <rect x="110" y="78" width="92" height="50" rx="14" class="fill-surface stroke-ink" stroke-width="3"/>
  <circle cx="178" cy="103" r="12" class="fill-ink"/><circle cx="178" cy="103" r="5" class="fill-srv"/>
  <circle cx="134" cy="103" r="5" class="fill-node"/>
  <rect x="136" y="58" width="40" height="18" rx="7" class="fill-ink"/>
  <g class="spin" style="transform-box:view-box;transform-origin:156px 64px"><path d="M156 64 L196 54" class="stroke-bad" stroke-width="2.5" opacity=".7"/></g>
  <path d="M182 150 L240 130 L268 166" fill="none" class="stroke-ink" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>
  <path d="M182 150 L240 130 L268 166" fill="none" class="stroke-copper" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>
  <path d="M262 172 l-8 12 M274 170 l8 12" class="stroke-ink" stroke-width="4" stroke-linecap="round"/>`;
  parts.forEach((p, i) => {
    const d = 0.3 + i * 0.55;
    s += `<g class="pop-in" style="animation-delay:${d}s"><circle cx="${p.x}" cy="${p.y}" r="22" fill="none" class="stroke-accent" stroke-width="3" stroke-dasharray="5 4"/></g>`;
    if (nodesMode) {
      const ny = 46 + i * 62;
      s += `<path class="edge dash appear" style="animation-delay:${d + 0.2}s" d="M${p.x + 22} ${p.y} C ${p.x + 120} ${p.y}, 330 ${ny}, 372 ${ny}"/>`;
      s += nodeMarkup({ id: p.id, label: p.node, x: 478, y: ny, kind: 'node', w: 196 }, d + 0.35);
    } else {
      const lx = 320, ly = 50 + i * 60;
      s += `<path class="edge dash appear" style="animation-delay:${d + 0.15}s" d="M${p.x + 22} ${p.y} L${lx - 10} ${ly}"/>`;
      s += `<g class="slide-up" style="animation-delay:${d + 0.2}s"><rect x="${lx}" y="${ly - 20}" width="150" height="40" rx="12" class="fill-surface stroke-line" stroke-width="2"/><text class="t-b" x="${lx + 16}" y="${ly + 6}">${p.label}</text><text class="t-s" x="${lx + 136}" y="${ly + 6}" text-anchor="end">${['sees', 'measures', 'grabs', 'moves', 'powers'][i]}</text></g>`;
    }
  });
  s += '</svg>';
  stage.innerHTML = s;
};

ANIM.layers = (stage, o) => {
  const items = o.items || [];
  const n = items.length;
  stage.innerHTML = `<div class="layers">${items.map((it, i) => `
    <div class="layer${it.hl ? ' hl' : ''}" style="--k:${it.k || 'var(--c-node)'};animation-delay:${0.2 + (n - 1 - i) * 0.45}s">
      <div class="ic">${icon(it.icon || 'layers')}</div><div class="lt"><b>${md(it.title)}</b><span>${md(it.sub || '')}</span></div>${it.tag ? `<em>${esc(it.tag)}</em>` : ''}
    </div>`).join('')}</div>`;
};

ANIM.master = (stage, o) => {
  const mode = o.mode || 'ros1';
  const A = { x: 100, y: 70, l: '/camera' }, B = { x: 100, y: 270, l: '/detector' }, C = { x: 500, y: 70, l: '/planner' }, D = { x: 500, y: 270, l: '/motors' };
  const nodes = [A, B, C, D];
  const M = { x: 300, y: 170 };
  const nm = (p, d) => nodeMarkup({ id: p.l, label: p.l, x: p.x, y: p.y, w: 130 }, d);
  let s = '<svg class="st" viewBox="0 0 600 340" role="img" aria-label="How ROS nodes find each other">';
  const line = (p, q, cls, d) => `<path class="edge ${cls}${d ? ' appear' : ''}" ${d ? `style="animation-delay:${d}s"` : ''} d="M${p.x} ${p.y} L${q.x} ${q.y}"/>`;
  if (mode !== 'ros2') {
    nodes.forEach((p, i) => { s += line(p, M, 'dash', mode === 'ros1' ? 0.2 + i * 0.3 : 0); });
    if (mode === 'ros1') nodes.forEach((p, i) => { s += flowDots(`M${p.x} ${p.y} L${M.x} ${M.y}`, { n: 1, dur: 0.9, cls: 'pkt pkt-srv', delay: 0.3 + i * 0.3, once: true }); });
    s += `<g class="hw"><rect x="${M.x - 92}" y="${M.y - 30}" width="184" height="60" rx="14"/><text class="t-b" x="${M.x}" y="${M.y - 2}" text-anchor="middle" style="font-weight:600">ROS Master</text><text class="t-s" x="${M.x}" y="${M.y + 17}" text-anchor="middle">roscore</text></g>`;
    const dd = mode === 'ros1' ? 3 : 0;
    s += line({ x: A.x, y: A.y + 24 }, { x: B.x, y: B.y - 24 }, 'good', dd) + line({ x: C.x, y: C.y + 24 }, { x: D.x, y: D.y - 24 }, 'good', dd);
    s += flowDots(`M${A.x} ${A.y + 26} L${B.x} ${B.y - 26}`, { n: 2, dur: 1.8, delay: dd + 0.3 }) + flowDots(`M${C.x} ${C.y + 26} L${D.x} ${D.y - 26}`, { n: 2, dur: 1.8, delay: dd + 0.5 });
    if (mode === 'ros1') {
      s += `<text class="t-s appear" style="animation-delay:.2s" x="300" y="34" text-anchor="middle">1 · every node registers with the Master</text>`;
      s += `<g class="slide-up" style="animation-delay:1.8s"><rect x="180" y="214" width="240" height="34" rx="10" class="fill-surface stroke-line" stroke-width="2"/><text class="t-s" x="300" y="236" text-anchor="middle" style="fill:var(--ink)">2 · "who publishes /image?"</text></g>`;
      s += `<text class="t-s appear" style="animation-delay:3s" x="300" y="318" text-anchor="middle">3 · then nodes connect directly</text>`;
    } else {
      s += `<g class="pop-in" style="animation-delay:1.2s"><rect x="${M.x - 92}" y="${M.y - 30}" width="184" height="60" rx="14" class="fill-bad" opacity=".92"/><text class="t-b t-w" x="${M.x}" y="${M.y + 6}" text-anchor="middle" style="font-weight:600">Master crashed</text></g>`;
      s += nm({ x: 300, y: 300, l: '/new_node' }, 2);
      s += `<path class="edge bad appear" style="animation-delay:2.4s" d="M300 278 L300 202"/><text class="t-h appear" style="animation-delay:2.6s;fill:var(--bad)" x="318" y="246">?</text>`;
      s += `<text class="t-s appear" style="animation-delay:2.8s" x="420" y="325" text-anchor="middle" style="fill:var(--bad)">new nodes can't find anyone</text>`;
    }
    nodes.forEach((p, i) => { s += nm(p, mode === 'ros1' ? 0.05 + i * 0.1 : 0); });
  } else {
    nodes.forEach((p, i) => { s += `<circle cx="${p.x}" cy="${p.y}" r="20" fill="none" class="stroke-topic" stroke-width="3" opacity="0"><animate attributeName="r" values="22;120" dur="2.4s" begin="${i * 0.35}s" repeatCount="indefinite"/><animate attributeName="opacity" values=".55;0" dur="2.4s" begin="${i * 0.35}s" repeatCount="indefinite"/></circle>`; });
    const pairs = [[A, B, 1.4], [B, C, 1.9, -40], [C, D, 2.4]];
    pairs.forEach(([p, q, d, cv]) => {
      const dd = edgePath(p.x, p.y + (p.y < q.y ? 24 : -24) * (p.x === q.x ? 1 : 0), q.x, q.y + (p.y < q.y ? -24 : 24) * (p.x === q.x ? 1 : 0), cv || 0);
      s += `<path class="edge good appear" style="animation-delay:${d}s" d="${dd}"/>` + flowDots(dd, { n: 2, dur: 2, delay: d + 0.3 });
    });
    s += `<g class="slide-up" style="animation-delay:.6s"><rect x="215" y="150" width="170" height="40" rx="20" class="fill-s2 stroke-topic" stroke-width="2"/><text class="t-b" x="300" y="176" text-anchor="middle" style="fill:var(--c-topic);font-weight:600">DDS discovery</text></g>`;
    nodes.forEach(p => { s += nm(p, 0); });
    s += nm({ x: 300, y: 300, l: '/logger' }, 3.6);
    s += `<path class="edge good appear" style="animation-delay:4.2s" d="M164 276 Q 230 300 236 300"/>` + flowDots('M164 276 Q 230 300 236 300', { n: 1, dur: 1.4, delay: 4.4 });
    s += `<text class="t-s appear" style="animation-delay:3.8s" x="300" y="334" text-anchor="middle">new nodes join automatically</text>`;
  }
  stage.innerHTML = s + '</svg>';
};

ANIM.cards = (stage, o) => {
  const items = o.items || [], lay = o.layout || 'grid2';
  if (lay === 'timeline') {
    stage.innerHTML = `<div class="timeline">${items.map((it, i) => `<div class="tl-item${it.hl ? ' hl' : ''}" style="--k:${it.k || 'var(--c-node)'};animation-delay:${0.2 + i * 0.4}s"><div class="dot"></div><div class="when">${esc(it.when || '')}</div><h4>${md(it.title)}</h4><p>${md(it.text || '')}</p></div>`).join('')}</div>`;
    return;
  }
  let html = `<div class="cards ${lay}">`;
  items.forEach((it, i) => {
    const d = 0.15 + i * (o.step || 0.35);
    if (lay === 'row' && i > 0 && o.arrows !== false) html += `<div class="arrow-sep">${icon('arrowR')}</div>`;
    html += `<div class="icard${it.hl ? ' hl' : ''}" style="--k:${it.k || 'var(--c-node)'};animation-delay:${d}s">${it.tag ? `<span class="tag">${esc(it.tag)}</span>` : ''}${o.checks ? `<span class="check" style="animation-delay:${d + 0.5}s">${icon('check')}</span>` : ''}${it.icon ? `<div class="ic">${icon(it.icon)}</div>` : ''}<h4>${md(it.title)}</h4>${it.text ? `<p>${md(it.text)}</p>` : ''}${it.code ? `<pre>${esc(it.code)}</pre>` : ''}</div>`;
  });
  stage.innerHTML = html + '</div>';
};

ANIM.compare = (stage, o, ctx) => {
  const rows = o.rows || [];
  stage.innerHTML = `<div class="cmp"><div class="cmp-head"><span>${esc(o.left || 'Python')}</span><span>${esc(o.right || 'C++')}</span></div>${rows.map((r, i) => `<div class="cmp-row" style="animation-delay:${0.2 + i * 0.35}s"><div>${esc(r[0])}</div><div>${esc(r[1])}</div></div>`).join('')}</div>`;
  const rs = $$('.cmp-row', stage); let k = 0;
  ctx.after(0.4 * 1000 + rows.length * 350, () => { const tick = () => { rs.forEach((r, i) => r.classList.toggle('on', i === k % rs.length)); k++; }; tick(); ctx.every(1900, tick); });
};

ANIM.terminal = (stage, o, ctx) => {
  const panes = o.panes || [{ title: 'Terminal', lines: o.lines || [] }];
  stage.innerHTML = `<div class="htmlstage"><div class="term-wrap${panes.length > 1 ? ' two' : ''}">${panes.map(p => `<div class="term"><div class="bar"><i></i><i></i><i></i><span>${esc(p.title || 'Terminal')}</span></div><div class="body"></div></div>`).join('')}</div></div>`;
  const bodies = $$('.term .body', stage);
  panes.forEach((p, pi) => runTerm(bodies[pi], p.lines || [], ctx, pi * 500));
};
function runTerm(body, lines, ctx, startDelay) {
  let li = 0; let counter = 1; let stamp = 1730000000.12;
  const cursor = h('span', { class: 'cursor' });
  const fmt = (s) => String(s).replace(/\{n\}/g, counter).replace(/\{t\}/g, stamp.toFixed(3));
  const trim = () => { while (body.childNodes.length > 80) body.removeChild(body.firstChild); body.scrollTop = body.scrollHeight; };
  const next = () => {
    if (li >= lines.length) { body.append(cursor); return; }
    const L = lines[li++];
    if (L.c != null) {
      const row = h('span', { class: 'c' }); body.append(h('span', { class: 'p' }, '$ '), row, cursor);
      let k = 0; const txt = L.c;
      const type = () => { if (k <= txt.length) { row.textContent = txt.slice(0, k); k += 1 + (Math.random() < .3 ? 1 : 0); ctx.after(26, type); } else { row.textContent = txt; body.append('\n'); ctx.after(L.pause || 380, next); } };
      ctx.after(160, type);
    } else if (L.o != null) {
      cursor.remove(); body.append(h('span', { class: L.cls || 'o' }, fmt(L.o) + '\n')); trim(); ctx.after(L.pause || 140, next);
    } else if (L.stream) {
      cursor.remove(); let k = 0;
      const emit = () => { body.append(h('span', { class: L.cls || 'o' }, fmt(L.stream[k % L.stream.length]) + '\n')); k++; if (k % L.stream.length === 0) { counter++; stamp += (L.every || 900) / 1000; } trim(); };
      emit(); ctx.every(L.every || 900, emit);
    } else if (L.wait) { ctx.after(L.wait, next); } else next();
  };
  ctx.after(startDelay + 200, next);
}

ANIM.code = (stage, o, ctx) => {
  const lang = o.lang || 'py';
  stage.innerHTML = `<div class="htmlstage">${codeBlock(o.code, lang, { cls: 'focus' })}${o.steps || o.note ? `<div class="code-note"><span class="tag">${esc(o.tag || 'LOOK')}</span><span class="nt"></span></div>` : ''}</div>`;
  const code = $('.code', stage), lines = $$('.ln', code), note = $('.nt', stage);
  const ranges = (r) => { const out = new Set(); (Array.isArray(r[0]) ? r : [r]).forEach(([a, b]) => { for (let i = a; i <= (b || a); i++) out.add(i - 1); }); return out; };
  const show = (rng, txt) => {
    const set = ranges(rng);
    lines.forEach((l, i) => l.classList.toggle('hl', set.has(i)));
    if (note && txt != null) note.innerHTML = md(txt);
    const first = lines[Math.min(...set)];
    if (first) { const top = first.offsetTop - 30; code.scrollTo({ top: Math.max(0, top), behavior: REDUCED ? 'auto' : 'smooth' }); }
  };
  if (o.steps) { let k = 0; const tick = () => { const s = o.steps[k % o.steps.length]; show(s.lines, s.note); k++; }; tick(); ctx.every(o.every || 3600, tick); }
  else if (o.focus) { ctx.after(250, () => show(o.focus, o.note)); }
  else { code.classList.remove('focus'); if (note) note.innerHTML = md(o.note || ''); }
};

ANIM.tick = (stage, o, ctx) => {
  const sub = !!o.sub, P = 1.6;
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A timer triggers a callback that publishes a message">
    <text class="t-s" x="586" y="24" text-anchor="end">slow motion</text>
    <g transform="translate(78 150)"><circle r="44" class="fill-surface stroke-ink" stroke-width="3"/><circle r="3.5" class="fill-ink"/><line class="hand" x1="0" y1="0" x2="0" y2="-32" stroke-width="4" stroke-linecap="round" style="stroke:var(--accent)"/><text class="t-s" y="68" text-anchor="middle">timer · ${esc(o.period || '0.5 s')}</text></g>
    <path class="edge" d="M126 150 L176 150"/>
    <g class="cb"><rect x="180" y="124" width="118" height="52" rx="12" class="fill-s2 stroke-node" stroke-width="2.5"/><text class="t-m" x="239" y="156" text-anchor="middle">${sub ? 'tick()' : 'tick()'}</text></g>
    <rect x="${sub ? 318 : 330}" y="132" width="${sub ? 118 : 160}" height="36" rx="8" class="fill-s3" opacity=".6"/>
    ${topicLabel(sub ? 377 : 410, 196, '/chatter')}
    ${sub ? `<g class="cb2"><rect x="456" y="124" width="126" height="52" rx="12" class="fill-s2 stroke-node" stroke-width="2.5"/><text class="t-m" x="519" y="156" text-anchor="middle">on_msg(msg)</text></g>` : `<text class="t-s" x="${410}" y="120" text-anchor="middle">publish(msg)</text>`}
    <g class="pk" opacity="0">${envelope('pkt')}<text class="t-s pkl" y="-14" text-anchor="middle" style="fill:var(--ink);font-weight:600"></text></g>
    <foreignObject x="20" y="240" width="560" height="92"><div xmlns="http://www.w3.org/1999/xhtml" class="term" style="height:88px"><div class="body logs" style="min-height:0;padding:8px 12px;font-size:12.5px"></div></div></foreignObject>
  </svg>`;
  const hand = $('.hand', stage), pk = $('.pk', stage), pkl = $('.pkl', stage), cb = $('.cb rect', stage), cb2 = sub ? $('.cb2 rect', stage) : null, logs = $('.logs', stage);
  let n = 0, last = -1;
  const x0 = 240, x1 = sub ? 520 : 500;
  ctx.loop((dt, t) => {
    const ph = (t % P) / P, cyc = Math.floor(t / P);
    hand.setAttribute('transform', `rotate(${ph * 360})`);
    if (cyc !== last) {
      last = cyc; n = cyc;
      pkl.textContent = `Hello ROS ${n}`;
      if (!sub) addLog(logs, `[INFO] [talker]: Sent: Hello ROS ${n}`);
    }
    cb.style.fill = ph < 0.18 ? 'var(--c-node-soft)' : ''; cb.style.strokeWidth = ph < 0.18 ? 5 : '';
    const k = clamp((ph - 0.12) / 0.62, 0, 1);
    pk.setAttribute('opacity', k > 0 && k < 1 ? 1 : 0);
    pk.setAttribute('transform', `translate(${lerp(x0, x1, k)} 150)`);
    if (sub && cb2) {
      const hit = ph > 0.74 && ph < 0.92; cb2.style.fill = hit ? 'var(--c-node-soft)' : ''; cb2.style.strokeWidth = hit ? 5 : '';
      if (hit && !stage._logged) { stage._logged = cyc; addLog(logs, `[INFO] [listener]: Heard: Hello ROS ${n}`); }
      if (!hit && stage._logged === cyc) stage._logged = null;
    }
  });
};
function topicLabel(x, y, t) { return `<g class="topic"><rect x="${x - 54}" y="${y - 16}" width="108" height="32" rx="8"/><text class="t-m" x="${x}" y="${y + 5}" text-anchor="middle">${esc(t)}</text></g>`; }
function addLog(el, text) { el.append(h('div', {}, text)); while (el.children.length > 3) el.firstChild.remove(); }

ANIM.queue = (stage, o, ctx) => {
  const depth = o.depth || 5, slotW = 46, qx = 170, qy = 150;
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A message queue with limited depth">
    ${nodeMarkup({ id: 'p', label: '/talker', x: 82, y: 150, w: 120 })}
    <rect x="${qx - 8}" y="${qy - 32}" width="${depth * slotW + 16}" height="64" rx="14" class="fill-surface stroke-topic" stroke-width="2.5"/>
    ${Array.from({ length: depth }, (_, i) => `<rect x="${qx + i * slotW + 4}" y="${qy - 22}" width="${slotW - 8}" height="44" rx="8" class="fill-s2"/>`).join('')}
    <text class="t-s" x="${qx + depth * slotW / 2}" y="${qy - 44}" text-anchor="middle" style="fill:var(--c-topic);font-weight:600">outgoing queue · depth ${depth}</text>
    ${nodeMarkup({ id: 's', label: 'slow reader', x: 528, y: 150, w: 128 })}
    <text class="t-s" x="${qx + depth * slotW / 2}" y="252" text-anchor="middle">new messages join on the left</text>
    <text class="t-s drop" x="${qx + depth * slotW / 2}" y="276" text-anchor="middle" style="fill:var(--bad);font-weight:600">dropped: 0</text>
    <g class="pkts"></g></svg>`;
  const G = $('.pkts', stage), dropT = $('.drop', stage);
  let q = [], flying = [], id = 0, dropped = 0, tPub = 0, tSub = 0;
  const mkPkt = () => { const g = document.createElementNS('http://www.w3.org/2000/svg', 'g'); g.innerHTML = envelope('pkt') + `<text class="t-s" y="5" x="0" text-anchor="middle" style="fill:#2b1a00;font-weight:700;font-size:10px">${++id}</text>`; G.append(g); return { g, id, x: 140, y: qy, tx: 140, ty: qy, vy: 0, fall: false, a: 1 }; };
  ctx.loop((dt) => {
    tPub += dt; tSub += dt;
    if (tPub > 0.55) { tPub = 0; const p = mkPkt(); if (q.length >= depth) { const old = q.pop(); old.fall = true; flying.push(old); dropped++; dropT.textContent = 'dropped: ' + dropped; } q.unshift(p); }
    if (tSub > 1.25 && q.length) { tSub = 0; const p = q.pop(); p.tx = 528; p.done = true; flying.push(p); }
    q.forEach((p, i) => { p.tx = qx + i * slotW + slotW / 2; p.ty = qy; });
    [...q, ...flying].forEach(p => {
      if (p.fall) { p.vy += 600 * dt; p.y += p.vy * dt; p.x += 40 * dt; p.a -= dt * 1.2; }
      else { p.x += (p.tx - p.x) * Math.min(1, dt * 9); p.y += (p.ty - p.y) * Math.min(1, dt * 9); if (p.done && Math.abs(p.tx - p.x) < 4) p.a -= dt * 4; }
      p.g.setAttribute('transform', `translate(${p.x.toFixed(1)} ${p.y.toFixed(1)})`); p.g.setAttribute('opacity', Math.max(0, p.a).toFixed(2));
      if (p.fall) p.g.querySelector('rect').style.fill = 'var(--bad)';
    });
    flying = flying.filter(p => { if (p.a <= 0) { p.g.remove(); return false; } return true; });
  });
};

ANIM.tree = (stage, o) => {
  stage.innerHTML = `<div class="tree">${(o.items || []).map((it, i) => `<div class="row${it.dir ? '' : ' file'}${it.hl ? ' hl' : ''}" style="padding-left:${it.d * 22}px;animation-delay:${0.15 + (it.at != null ? it.at : i) * (o.step || 0.28)}s">${it.dir ? folderIco() : fileIco()}<span class="nm">${esc(it.n)}</span>${it.note ? `<span class="note">${md(it.note)}</span>` : ''}</div>`).join('')}</div>`;
};
function folderIco() { return '<svg class="ico" viewBox="0 0 24 24" aria-hidden="true"><path d="M3 6.5A1.5 1.5 0 0 1 4.5 5h5l2 2.2h8A1.5 1.5 0 0 1 21 8.7v9.8A1.5 1.5 0 0 1 19.5 20h-15A1.5 1.5 0 0 1 3 18.5z" fill="currentColor"/></svg>'; }
function fileIco() { return '<svg class="ico ico-stroke" viewBox="0 0 24 24" aria-hidden="true"><path d="M6 3h8l4 4v14H6z"/><path d="M14 3v4h4"/></svg>'; }

ANIM.service = (stage, o, ctx) => {
  const mode = o.mode || 'basic';
  const req = o.req || 'a: 2, b: 3', res = o.res || 'sum: 5';
  if (mode === 'compare') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="Topic stream compared with a service round trip">
      <text class="t-h" x="24" y="34">Topic</text><text class="t-s" x="90" y="34">one-way stream, many messages</text>
      ${nodeMarkup({ id: 'a', label: 'publisher', x: 92, y: 88, w: 130 })}${topicLabel(300, 88, '/scan')}${nodeMarkup({ id: 'b', label: 'subscriber', x: 508, y: 88, w: 136 })}
      <path class="edge" d="M158 88 L244 88"/><path class="edge" d="M356 88 L438 88"/>
      ${flowDots('M158 88 L244 88', { n: 3, dur: 1.2 })}${flowDots('M356 88 L438 88', { n: 3, dur: 1.2, delay: .4 })}
      <line x1="20" y1="160" x2="580" y2="160" class="stroke-line" stroke-width="2" stroke-dasharray="4 6"/>
      <text class="t-h" x="24" y="200">Service</text><text class="t-s" x="112" y="200">one request, one response</text>
      ${nodeMarkup({ id: 'c', label: 'client', x: 92, y: 262, w: 110 })}${nodeMarkup({ id: 's', label: 'server', x: 508, y: 262, w: 110, kind: 'srv' })}
      <path class="edge" d="M150 250 Q 300 214 450 250"/><path class="edge" d="M450 276 Q 300 312 150 276"/>
      <g class="rq" opacity="0">${envelope('pkt pkt-srv')}<text class="t-s" y="-13" text-anchor="middle" style="fill:var(--c-srv);font-weight:600">request</text></g>
      <g class="rs" opacity="0">${envelope('pkt pkt-res')}<text class="t-s" y="24" text-anchor="middle" style="fill:var(--good);font-weight:600">response</text></g></svg>`;
    roundTrip(stage, ctx, { y1: 250, y2: 276, cx1: 214, cx2: 312, x0: 150, x1: 450 });
    return;
  }
  if (mode === 'future') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A future is a ticket for a result that arrives later">
      ${nodeMarkup({ id: 'c', label: 'client node', x: 110, y: 90, w: 150 })}${nodeMarkup({ id: 's', label: '/add_two_ints', x: 490, y: 90, w: 170, kind: 'srv' })}
      <path class="edge" d="M186 80 L404 80"/><path class="edge" d="M404 102 L186 102"/>
      <g class="rq" opacity="0">${envelope('pkt pkt-srv')}<text class="t-s" y="-13" text-anchor="middle" style="fill:var(--c-srv);font-weight:600">${esc(req)}</text></g>
      <g class="rs" opacity="0">${envelope('pkt pkt-res')}<text class="t-s" y="25" text-anchor="middle" style="fill:var(--good);font-weight:600">${esc(res)}</text></g>
      <g transform="translate(40 168)"><rect width="200" height="74" rx="14" class="fill-surface stroke-line tk" stroke-width="3" stroke-dasharray="7 5"/>
        <text class="t-s" x="16" y="26" style="font-weight:600;fill:var(--ink)">Future</text><text class="t-m ft" x="16" y="54">pending…</text></g>
      <g transform="translate(300 168)"><rect width="260" height="74" rx="14" class="fill-s2"/><text class="t-s" x="16" y="26" style="font-weight:600;fill:var(--ink)">meanwhile: other callbacks run</text>
        <rect x="16" y="42" width="228" height="12" rx="6" class="fill-s3"/><rect class="bar fill-node" x="16" y="42" width="0" height="12" rx="6"/></g>
      <g class="done" opacity="0"><rect x="40" y="262" width="520" height="52" rx="12" class="fill-surface stroke-node" stroke-width="2.5"/><text class="t-m" x="60" y="294">on_done(future) → future.result().${esc(res.split(':')[0])} = ${esc((res.split(':')[1] || '').trim())}</text></g></svg>`;
    const rq = $('.rq', stage), rs = $('.rs', stage), ft = $('.ft', stage), tk = $('.tk', stage), bar = $('.bar', stage), done = $('.done', stage);
    const P = 5;
    ctx.loop((dt, t) => {
      const ph = (t % P) / P;
      const a = clamp(ph / 0.22, 0, 1), b = clamp((ph - 0.55) / 0.2, 0, 1);
      rq.setAttribute('opacity', a > 0 && a < 1 ? 1 : 0); rq.setAttribute('transform', `translate(${lerp(190, 400, a)} 80)`);
      rs.setAttribute('opacity', b > 0 && b < 1 ? 1 : 0); rs.setAttribute('transform', `translate(${lerp(400, 190, b)} 102)`);
      const ready = ph >= 0.75;
      ft.textContent = ready ? `done ✓ ${res}` : (ph > 0.02 ? 'pending…' : 'call_async(req)');
      tk.setAttribute('stroke-dasharray', ready ? '0' : '7 5'); tk.style.stroke = ready ? 'var(--good)' : '';
      bar.setAttribute('width', (228 * ((t * 0.9) % 1)).toFixed(1));
      done.setAttribute('opacity', ready ? 1 : 0);
    });
    return;
  }
  const clients = mode === 'clients' ? 3 : 1;
  const ys = clients === 1 ? [170] : [70, 170, 270];
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A service client sends a request and the server responds">
    ${ys.map((y, i) => nodeMarkup({ id: 'c' + i, label: clients === 1 ? 'client' : 'client ' + (i + 1), x: 96, y, w: 120 })).join('')}
    <g transform="translate(470 170)"><rect x="-82" y="-40" width="164" height="80" rx="16" class="fill-surface stroke-srv" stroke-width="3"/><text class="t-b" y="-8" text-anchor="middle" style="font-weight:600">server</text><text class="t-s" y="16" text-anchor="middle">${esc(o.name || '/add_two_ints')}</text>
      <g class="gear" transform="translate(60 -38)"><circle r="14" class="fill-srv"/><circle r="5" class="fill-surface"/></g></g>
    ${ys.map(y => `<path class="edge dash" d="M156 ${y - 8} Q 300 ${(y + 170) / 2 - 30} 388 ${162}"/>`).join('')}
    <g class="rq" opacity="0">${envelope('pkt pkt-srv')}<text class="t-s" y="-13" text-anchor="middle" style="fill:var(--c-srv);font-weight:600">${esc(req)}</text></g>
    <g class="rs" opacity="0">${envelope('pkt pkt-res')}<text class="t-s" y="25" text-anchor="middle" style="fill:var(--good);font-weight:600">${esc(res)}</text></g>
    ${clients > 1 ? '<text class="t-s" x="470" y="250" text-anchor="middle">requests handled one by one</text>' : ''}</svg>`;
  const rq = $('.rq', stage), rs = $('.rs', stage), gear = $('.gear', stage);
  const P = 3.2;
  ctx.loop((dt, t) => {
    const cyc = Math.floor(t / P), ph = (t % P) / P, y = ys[cyc % ys.length];
    const a = clamp(ph / 0.32, 0, 1), b = clamp((ph - 0.55) / 0.32, 0, 1);
    const qp = (k, up) => { const x0 = 156, x1 = 388, cx = 300, cy = (y + 170) / 2 - 30 * (up ? 1 : -1); const yy0 = y + (up ? -8 : 8), yy1 = up ? 162 : 178; const u = 1 - k; return [u * u * x0 + 2 * u * k * cx + k * k * x1, u * u * yy0 + 2 * u * k * cy + k * k * yy1]; };
    const [ax, ay] = qp(a, true), [bx, by] = qp(1 - b, false);
    rq.setAttribute('opacity', a > 0 && a < 1 ? 1 : 0); rq.setAttribute('transform', `translate(${ax.toFixed(1)} ${ay.toFixed(1)})`);
    rs.setAttribute('opacity', b > 0 && b < 1 ? 1 : 0); rs.setAttribute('transform', `translate(${bx.toFixed(1)} ${by.toFixed(1)})`);
    const busy = ph > 0.32 && ph < 0.56;
    gear.setAttribute('transform', `translate(60 -38) rotate(${busy ? t * 400 : 0}) scale(${busy ? 1.15 : 1})`);
  });
};
function roundTrip(stage, ctx, g) {
  const rq = $('.rq', stage), rs = $('.rs', stage), P = 3;
  const q = (k, y0, cy) => { const u = 1 - k; return [u * u * g.x0 + 2 * u * k * 300 + k * k * g.x1, u * u * y0 + 2 * u * k * cy + k * k * y0]; };
  ctx.loop((dt, t) => {
    const ph = (t % P) / P, a = clamp(ph / 0.38, 0, 1), b = clamp((ph - 0.55) / 0.38, 0, 1);
    const [ax, ay] = q(a, g.y1, g.cx1), [bx, by] = q(1 - b, g.y2, g.cx2);
    rq.setAttribute('opacity', a > 0 && a < 1 ? 1 : 0); rq.setAttribute('transform', `translate(${ax.toFixed(1)} ${ay.toFixed(1)})`);
    rs.setAttribute('opacity', b > 0 && b < 1 ? 1 : 0); rs.setAttribute('transform', `translate(${bx.toFixed(1)} ${by.toFixed(1)})`);
  });
}

ANIM.msgGen = (stage, o, ctx) => {
  const fields = o.fields || [['string', 'name', 'robot name'], ['float64', 'battery_voltage', 'decimal number'], ['bool', 'is_charging', 'true or false'], ['float64[]', 'wheel_speeds', 'list of any length']];
  const fname = o.name || 'RobotStatus.msg';
  if (o.mode === 'generate') {
    const py = `class RobotStatus:\n    name: str\n    battery_voltage: float\n    is_charging: bool\n    wheel_speeds: list[float]`;
    const cpp = `struct RobotStatus {\n  std::string name;\n  double battery_voltage;\n  bool is_charging;\n  std::vector<double> wheel_speeds;\n};`;
    stage.innerHTML = `<div class="gen">
      <div class="icard" style="--k:var(--c-topic);animation-delay:.1s"><h4>${esc(fname)}</h4><pre>${esc(fields.map(f => f[0] + ' ' + f[1]).join('\n'))}</pre></div>
      <div class="machine" style="animation-delay:.6s"><svg viewBox="0 0 120 120" aria-hidden="true"><g class="spin" style="transform-box:view-box;transform-origin:46px 50px"><path d="${gearPath(46, 50, 26, 19, 10)}" class="fill-topic"/><circle cx="46" cy="50" r="8" class="fill-surface"/></g><g class="spin" style="transform-box:view-box;transform-origin:82px 82px;animation-direction:reverse"><path d="${gearPath(82, 82, 18, 13, 8)}" class="fill-msg"/><circle cx="82" cy="82" r="5" class="fill-surface"/></g></svg><b>rosidl</b><span>code generator</span></div>
      <div class="outs"><div class="icard" style="--k:var(--c-node);animation-delay:1.2s"><h4>Python</h4><pre>${esc(py)}</pre></div><div class="icard" style="--k:var(--c-srv);animation-delay:1.6s"><h4>C++</h4><pre>${esc(cpp)}</pre></div></div></div>`;
    return;
  }
  stage.innerHTML = `<div class="msgfile"><div class="icard" style="--k:var(--c-topic);animation-delay:0s;opacity:1"><h4>${esc(fname)}</h4><div class="mf-lines"></div></div><div class="mf-notes"></div></div>`;
  const L = $('.mf-lines', stage), N = $('.mf-notes', stage);
  fields.forEach((f, i) => ctx.after(400 + i * 900, () => {
    L.append(h('div', { class: 'mf-line', html: `<span class="tok-t">${esc(f[0])}</span> ${esc(f[1])}` }));
    N.append(h('div', { class: 'mf-note', html: `<code>${esc(f[0])}</code> ${esc(f[2])}` }));
  }));
};
function gearPath(cx, cy, ro, ri, teeth) {
  const pts = [];
  for (let i = 0; i < teeth * 4; i++) { const a = i / (teeth * 4) * Math.PI * 2; const r = (i % 4 < 2) ? ro : ri; pts.push(`${(cx + r * Math.cos(a)).toFixed(1)} ${(cy + r * Math.sin(a)).toFixed(1)}`); }
  return 'M' + pts.join('L') + 'Z';
}

ANIM.knobs = (stage, o, ctx) => {
  const mode = o.mode || 'live';
  if (mode === 'intro') {
    const knobs = [['rate_hz', '2.0', 'var(--c-param)'], ['greeting', '"Hello"', 'var(--c-srv)'], ['max_speed', '0.5', 'var(--c-node)']];
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A node with parameter knobs">
      <rect x="40" y="40" width="300" height="250" rx="26" class="fill-surface stroke-node" stroke-width="3"/>
      <text class="t-h" x="190" y="78" text-anchor="middle">/talker</text>
      ${knobs.map((k, i) => `<g transform="translate(${95 + i * 95} 165)" class="pop-in" style="animation-delay:${0.3 + i * 0.3}s"><circle r="32" class="fill-s2 stroke-ink" stroke-width="3"/><g class="spin" style="animation-duration:${6 + i * 2}s;animation-direction:${i % 2 ? 'reverse' : 'normal'}"><line y1="-6" y2="-26" stroke="${k[2]}" stroke-width="6" stroke-linecap="round"/></g><circle r="6" class="fill-ink"/><text class="t-m" y="58" text-anchor="middle" style="font-size:12.5px">${k[0]}</text><text class="t-s" y="78" text-anchor="middle">${esc(k[1])}</text></g>`).join('')}
      <path class="edge" d="M340 165 L420 165"/>${topicLabel(478, 165, '/chatter')}
      ${flowDots('M340 165 L424 165', { n: 2, dur: 1.4 })}
      <text class="t-s" x="190" y="318" text-anchor="middle">settings live on the node, not in the code</text></svg>`;
    return;
  }
  const gate = mode === 'gate';
  stage.innerHTML = `<div class="play-ui knob-ui">
    <div><svg class="st" viewBox="0 0 300 260" role="img" aria-label="Parameter knob">
      <rect x="10" y="10" width="280" height="240" rx="24" class="fill-surface stroke-node" stroke-width="3"/>
      <text class="t-h" x="150" y="44" text-anchor="middle">/talker</text>
      <g transform="translate(150 132)"><circle r="58" class="fill-s2 stroke-ink" stroke-width="3"/>${Array.from({ length: 11 }, (_, i) => { const a = (-135 + i * 27) * Math.PI / 180; return `<line x1="${(66 * Math.sin(a)).toFixed(1)}" y1="${(-66 * Math.cos(a)).toFixed(1)}" x2="${(74 * Math.sin(a)).toFixed(1)}" y2="${(-74 * Math.cos(a)).toFixed(1)}" class="stroke-muted" stroke-width="2.5"/>`; }).join('')}<g class="knob-hand"><line y1="-10" y2="-48" stroke="var(--c-param)" stroke-width="8" stroke-linecap="round"/></g><circle r="10" class="fill-ink"/></g>
      <text class="t-m kv" x="150" y="226" text-anchor="middle">rate_hz = 2.0</text></svg></div>
    <div><div class="sliders"><div class="slider"><label for="kr${++UID}">rate_hz</label><input id="kr${UID}" type="range" min="${gate ? -2 : 0.5}" max="10" step="0.5" value="2"><output>2.0</output></div></div>
      ${gate ? '<div class="gatebox" aria-live="polite"></div>' : ''}
      <div class="cmdline"></div>
      <svg class="st" viewBox="0 0 300 70" style="margin-top:12px" aria-hidden="true"><path class="edge" d="M10 35 L290 35"/><g class="lane"></g><text class="t-s hzl" x="150" y="64" text-anchor="middle"></text></svg></div></div>`;
  const inp = $('input', stage), out = $('output', stage), hand = $('.knob-hand', stage), kv = $('.kv', stage), cmd = $('.cmdline', stage), lane = $('.lane', stage), gb = $('.gatebox', stage), hzl = $('.hzl', stage);
  let rate = 2, acc = 0; const pk = [];
  const apply = () => {
    const v = +inp.value; out.textContent = v.toFixed(1);
    const ok = !gate || v > 0;
    cmd.innerHTML = `<span style="color:var(--term-green)">$</span> ros2 param set /talker rate_hz ${v.toFixed(1)}<br><span style="color:${ok ? 'var(--term-ink)' : 'var(--term-red)'}">${ok ? 'Set parameter successful' : 'Setting parameter failed: rate_hz must be > 0'}</span>`;
    if (gb) gb.innerHTML = ok ? `<div class="gate ok">${icon('check')} on_params() accepted ${v.toFixed(1)}</div>` : `<div class="gate no">${icon('x')} on_params() rejected ${v.toFixed(1)}: keeping ${rate.toFixed(1)} Hz</div>`;
    if (ok) rate = v;
    hand.setAttribute('transform', `rotate(${-135 + (clamp(rate, 0, 10) / 10) * 270})`);
    kv.textContent = `rate_hz = ${rate.toFixed(1)}`; hzl.textContent = `publishing at ${rate.toFixed(1)} Hz`;
  };
  inp.addEventListener('input', apply); apply();
  ctx.loop((dt) => {
    acc += dt;
    if (rate > 0 && acc >= 1 / rate) { acc = 0; const g = document.createElementNS('http://www.w3.org/2000/svg', 'g'); g.innerHTML = envelope('pkt'); lane.append(g); pk.push({ g, x: 12 }); }
    for (let i = pk.length - 1; i >= 0; i--) { const p = pk[i]; p.x += 150 * dt; p.g.setAttribute('transform', `translate(${p.x.toFixed(1)} 35)`); if (p.x > 286) { p.g.remove(); pk.splice(i, 1); } }
  });
};

ANIM.launch = (stage, o, ctx) => {
  const mode = o.mode || 'collapse';
  if (mode === 'ns') {
    stage.innerHTML = graphSVG({
      reveal: true, alt: 'Two robots separated by namespaces',
      groups: [{ x: 14, y: 30, w: 572, h: 130, label: 'namespace: /robot1', at: 0.1 }, { x: 14, y: 186, w: 572, h: 130, label: 'namespace: /robot2', at: 0.5 }],
      nodes: [GN('t1', '/robot1/talker', 120, 104), GN('c1', '/robot1/chatter', 300, 104, 'topic'), GN('l1', '/robot1/listener', 478, 104), GN('t2', '/robot2/talker', 120, 260), GN('c2', '/robot2/chatter', 300, 260, 'topic'), GN('l2', '/robot2/listener', 478, 260)],
      edges: [GE('t1', 'c1'), GE('c1', 'l1'), GE('t2', 'c2'), GE('c2', 'l2')],
    });
    return;
  }
  if (mode === 'remap') {
    stage.innerHTML = graphSVG({
      reveal: false, alt: 'Remapping reroutes a topic',
      nodes: [GN('t', '/talker', 100, 170), GN('c', '/chatter', 320, 80, 'topic', { at: 0 }), GN('n', '/news', 320, 260, 'topic'), GN('l', '/listener', 510, 260)],
      edges: [{ a: 't', b: 'c', dash: true, at: 0 }, { a: 't', b: 'n', flow: 'msg', at: 1.6, good: true }, { a: 'n', b: 'l', flow: 'msg', at: 1.9 }],
      notes: [{ x: 320, y: 40, text: 'without remap: talker → chatter', at: 0 }, { x: 186, y: 314, text: '<remap from="chatter" to="/news"/>', cls: 't-m', at: 1.2 }],
      extra: `<g class="pop-in" style="animation-delay:1.2s"><circle cx="205" cy="170" r="18" class="fill-msg"/><path d="M196 170h18M208 163l7 7-7 7" class="stroke-ink" stroke-width="2.6" fill="none" stroke-linecap="round"/></g>`,
    });
    return;
  }
  const runs = ['ros2 run demo_nodes_cpp talker', 'ros2 run demo_nodes_py listener', 'ros2 run my_pkg planner', 'ros2 run my_pkg driver'];
  let s = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A launch file replaces many terminals">`;
  runs.forEach((r, i) => {
    s += `<g class="win" style="transition:transform .9s cubic-bezier(.5,0,.3,1), opacity .9s"><rect x="${20 + i * 12}" y="${20 + i * 46}" width="250" height="40" rx="9" fill="var(--term-bg)"/><circle cx="${34 + i * 12}" cy="${32 + i * 46}" r="3.5" fill="#FF6B5E"/><text x="${46 + i * 12}" y="${46 + i * 46}" style="font:12.5px var(--font-mono);fill:var(--term-ink)">$ ${esc(r)}</text></g>`;
  });
  s += `<g class="one" opacity="0" style="transition:opacity .6s"><rect x="20" y="120" width="290" height="62" rx="12" fill="var(--term-bg)"/><text x="36" y="146" style="font:13px var(--font-mono);fill:var(--term-green)">$</text><text x="50" y="146" style="font:13px var(--font-mono);fill:var(--term-ink)">ros2 launch my_pkg</text><text x="50" y="168" style="font:13px var(--font-mono);fill:var(--term-ink)">robot.launch.xml</text></g>`;
  const pos = [[470, 50], [470, 130], [470, 210], [470, 290]], names = ['/talker', '/listener', '/planner', '/driver'];
  names.forEach((n, i) => { s += `<g class="ln" opacity="0" style="transition:opacity .4s ${0.15 * i}s"><path class="edge" d="M312 151 C 360 151, 370 ${pos[i][1]}, ${pos[i][0] - 75} ${pos[i][1]}"/>${nodeMarkup({ id: n, label: n, x: pos[i][0], y: pos[i][1], w: 140 })}</g>`; });
  s += `<text class="t-s cap" x="150" y="320" text-anchor="middle">four terminals…</text></svg>`;
  stage.innerHTML = s;
  const wins = $$('.win', stage), one = $('.one', stage), lns = $$('.ln', stage), cap = $('.cap', stage);
  const cycle = () => {
    wins.forEach(w => { w.style.transform = ''; w.style.opacity = 1; }); one.setAttribute('opacity', 0); lns.forEach(l => l.setAttribute('opacity', 0)); cap.textContent = 'four terminals…';
    ctx.after(1700, () => { wins.forEach((w, i) => { w.style.transform = `translate(${-i * 12}px, ${120 - (20 + i * 46) + 10}px) scale(.98)`; w.style.opacity = 0; }); cap.textContent = '…one launch command'; });
    ctx.after(2500, () => one.setAttribute('opacity', 1));
    ctx.after(3100, () => { lns.forEach(l => l.setAttribute('opacity', 1)); cap.textContent = 'every node starts, with its settings'; });
  };
  cycle(); ctx.every(7500, cycle);
};

ANIM.frames = (stage, o, ctx) => {
  const mode = o.mode || 'intro';
  const uid = ++UID;
  if (mode === 'axes') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="Coordinate frame axes: x forward red, y left green, z up blue">
      <g transform="translate(250 190)">${arrowLine(0, 0, 170, 0, 'x', 7)}${arrowLine(0, 0, 0, -140, 'y', 7)}<circle r="16" class="fill-surface ax-z" stroke-width="5"/><circle r="6" class="fz"/>
      <text class="t-h" x="182" y="8" style="fill:var(--c-x)">x</text><text class="t-s" x="182" y="30">forward</text>
      <text class="t-h" x="-10" y="-152" text-anchor="end" style="fill:var(--c-y)">y</text><text class="t-s" x="14" y="-150">left</text>
      <text class="t-h" x="-30" y="44" style="fill:var(--c-z)">z</text><text class="t-s" x="-12" y="44">up (out of the screen)</text>
      <g transform="translate(90 70)" class="pop-in" style="animation-delay:.6s">${robotTop(.8)}</g></g>
      <text class="t-b" x="560" y="300" text-anchor="end">RGB = XYZ</text></svg>`;
    return;
  }
  if (mode === 'intro') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A robot camera sees a ball">
      <g>${triad(52, 300, 0, 50, 'world')}</g>
      <g transform="translate(250 200) rotate(-20)">${robotTop(1.15)}<g transform="translate(30 0)">${triad(0, 0, 0, 34, '', false)}</g></g>
      <circle cx="455" cy="118" r="17" class="fill-msg stroke-ink" stroke-width="2.5"/>
      <path class="edge dash appear" style="animation-delay:.6s" d="M286 190 L440 124"/>
      <g class="slide-up" style="animation-delay:.9s"><rect x="300" y="120" width="146" height="30" rx="9" class="fill-surface stroke-line" stroke-width="2"/><text class="t-s" x="373" y="140" text-anchor="middle" style="fill:var(--ink)">1.5 m from the camera</text></g>
      <g class="slide-up" style="animation-delay:1.8s"><text class="t-h" x="250" y="300" text-anchor="middle" style="fill:var(--c-topic)">…and from the robot? the room?</text></g>
      <path class="edge dash appear" style="animation-delay:1.8s" d="M80 290 L440 130" opacity=".5"/></svg>`;
    return;
  }
  if (mode === 'transform') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A transform: translation and rotation between two frames">
      <g>${triad(70, 290, 0, 54, 'world')}</g><path class="vec" d="" stroke="var(--c-param)" stroke-width="3.5" stroke-dasharray="8 6" fill="none"/>
      <g class="rb"></g>
      <g transform="translate(372 24)"><rect width="212" height="96" rx="14" class="fill-surface stroke-line" stroke-width="2"/><text class="t-s" x="14" y="26" style="font-weight:600;fill:var(--ink)">world → base_link</text><text class="t-m ro1" x="14" y="52"></text><text class="t-m ro2" x="14" y="78"></text></g></svg>`;
    const rb = $('.rb', stage), vec = $('.vec', stage), r1 = $('.ro1', stage), r2 = $('.ro2', stage);
    ctx.loop((dt, t) => {
      const u = t * 0.35, wx = 3.1 + 1.6 * Math.sin(u), wy = 2.0 + 0.9 * Math.sin(2 * u), th = Math.atan2(1.8 * Math.cos(2 * u), 1.6 * Math.cos(u));
      const sx = 70 + wx * 80, sy = 290 - wy * 80;
      rb.innerHTML = `<g transform="translate(${sx.toFixed(1)} ${sy.toFixed(1)}) rotate(${(-th * 180 / Math.PI).toFixed(1)})">${robotTop(.85)}</g>${triad(sx, sy, th, 40, '')}<text class="t-s" x="${(sx + 16).toFixed(1)}" y="${(sy + 34).toFixed(1)}" style="font-weight:600;fill:var(--ink)">base_link</text>`;
      vec.setAttribute('d', `M70 290 L${sx.toFixed(1)} ${sy.toFixed(1)}`);
      r1.textContent = `xyz: ${wx.toFixed(2)}, ${wy.toFixed(2)}, 0.00`;
      r2.textContent = `yaw: ${(th * 180 / Math.PI).toFixed(0)}°  (q.z ${Math.sin(th / 2).toFixed(2)}, q.w ${Math.cos(th / 2).toFixed(2)})`;
    });
    return;
  }
  // time: ghosts of past frames
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="Transforms are stored over time"><g>${triad(56, 300, 0, 48, 'world')}</g><g class="gh"></g><g class="rb"></g>
    <g transform="translate(360 20)"><rect width="224" height="122" rx="14" class="fill-surface stroke-line" stroke-width="2"/><text class="t-s" x="14" y="26" style="font-weight:600;fill:var(--ink)">TF2 buffer (≈ 10 s)</text><g class="bl"></g></g></svg>`;
  const rb = $('.rb', stage), gh = $('.gh', stage), bl = $('.bl', stage);
  const hist = []; let acc = 0;
  ctx.loop((dt, t) => {
    const u = t * 0.4, wx = 1.0 + ((u * 0.8) % 5.2), wy = 1.6 + 0.6 * Math.sin(u * 1.6), th = Math.atan2(0.96 * Math.cos(u * 1.6), 0.8);
    const sx = 56 + wx * 80, sy = 300 - wy * 80;
    acc += dt;
    if (acc > 0.6) { acc = 0; hist.unshift({ sx, sy, th, t }); if (hist.length > 4) hist.pop(); if (wx < 1.2) hist.length = 0; }
    rb.innerHTML = `<g transform="translate(${sx.toFixed(1)} ${sy.toFixed(1)}) rotate(${(-th * 180 / Math.PI).toFixed(1)})">${robotTop(.7)}</g>${triad(sx, sy, th, 36, '')}`;
    gh.innerHTML = hist.map((p, i) => `<g opacity="${(0.55 - i * 0.12).toFixed(2)}">${triad(p.sx, p.sy, p.th, 30, '')}<text class="t-s" x="${(p.sx - 6).toFixed(0)}" y="${(p.sy + 26).toFixed(0)}">t−${((t - p.t)).toFixed(1)}s</text></g>`).join('');
    bl.innerHTML = [{ sx, sy, t }, ...hist].slice(0, 4).map((p, i) => `<text class="t-m" x="14" y="${50 + i * 20}" style="font-size:12px;fill:${i ? 'var(--muted)' : 'var(--ink)'}">${i ? 't−' + (t - p.t).toFixed(1) + 's' : 'now   '}  x ${((p.sx - 56) / 80).toFixed(2)}  y ${((300 - p.sy) / 80).toFixed(2)}</text>`).join('');
  });
};

ANIM.tfTree = (stage, o, ctx) => {
  const frames = o.frames || [['map', null], ['odom', 'map'], ['base_link', 'odom'], ['camera_link', 'base_link'], ['lidar_link', 'base_link'], ['wheel_left', 'base_link'], ['wheel_right', 'base_link']];
  const from = o.from || 'camera_link', to = o.to || 'map';
  const kids = {}; frames.forEach(([f, p]) => { if (p) (kids[p] = kids[p] || []).push(f); });
  const root = frames.find(f => !f[1])[0];
  const pos = {}; let leaf = 0; const depthOf = {};
  const lay = (f, d) => { depthOf[f] = d; const ks = kids[f] || []; if (!ks.length) { pos[f] = [leaf++, d]; return pos[f][0]; } const xs = ks.map(k => lay(k, d + 1)); pos[f] = [(Math.min(...xs) + Math.max(...xs)) / 2, d]; return pos[f][0]; };
  lay(root, 0);
  const maxD = Math.max(...Object.values(depthOf)), nl = leaf;
  const X = i => 70 + i * (460 / Math.max(1, nl - 1)), Y = d => 44 + d * (250 / Math.max(1, maxD));
  const parentOf = Object.fromEntries(frames);
  const chain = []; let c = from; while (c) { chain.push(c); if (c === to) break; c = parentOf[c]; }
  let s = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="TF2 frame tree">`;
  frames.forEach(([f, p]) => { if (!p) return; const on = chain.includes(f) && chain.includes(p); s += `<path class="edge${on ? ' tfl' : ''}" data-c="${f}" d="M${X(pos[p][0])} ${Y(pos[p][1]) + 18} L${X(pos[f][0])} ${Y(pos[f][1]) - 18}" ${on ? 'style="stroke-width:3.5"' : ''}/>`; });
  frames.forEach(([f], i) => { const on = chain.includes(f); s += `<g class="pop-in fr" data-f="${f}" style="animation-delay:${0.1 + i * 0.12}s"><rect x="${X(pos[f][0]) - 62}" y="${Y(pos[f][1]) - 18}" width="124" height="36" rx="10" class="${on ? 'fill-s2 stroke-param' : 'fill-surface stroke-line'}" stroke-width="2.5"/><text class="t-m" x="${X(pos[f][0])}" y="${Y(pos[f][1]) + 5}" text-anchor="middle" style="font-size:13px">${esc(f)}</text></g>`; });
  s += `<text class="t-s chainl" x="300" y="334" text-anchor="middle" style="fill:var(--c-param);font-weight:600"></text></svg>`;
  stage.innerHTML = s;
  const chainL = $('.chainl', stage);
  const hl = $$('.tfl', stage);
  let k = 0;
  const step = () => {
    const n = k % (chain.length + 2);
    chainL.textContent = chain.slice(0, Math.min(n + 1, chain.length)).join('  →  ');
    hl.forEach(e => { const ci = chain.indexOf(e.dataset.c); e.style.stroke = ci >= 0 && ci < n ? 'var(--c-param)' : ''; });
    k++;
  };
  step(); ctx.every(700, step);
};

ANIM.quat = (stage, o, ctx) => {
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="A heading angle and its quaternion">
    <circle cx="170" cy="170" r="118" fill="none" class="stroke-line" stroke-width="2.5"/>
    <path class="arc" fill="none" stroke="var(--c-param)" stroke-width="5" stroke-linecap="round"/>
    <g class="tt"></g>
    <g transform="translate(330 46)"><rect width="250" height="250" rx="18" class="fill-surface stroke-line" stroke-width="2"/>
      <text class="t-s" x="20" y="34" style="font-weight:600;fill:var(--ink)">heading θ</text><text class="t-h th" x="20" y="64"></text>
      <text class="t-s" x="20" y="104" style="font-weight:600;fill:var(--ink)">quaternion</text>
      ${['x', 'y', 'z', 'w'].map((k, i) => `<text class="t-m" x="20" y="${136 + i * 28}">${k}</text><rect x="44" y="${124 + i * 28}" width="120" height="14" rx="7" class="fill-s2"/><rect class="qb qb${k} fill-param" x="104" y="${124 + i * 28}" width="0" height="14" rx="3"/><text class="t-m qv${k}" x="234" y="${136 + i * 28}" text-anchor="end"></text>`).join('')}
    </g></svg>`;
  const tt = $('.tt', stage), arc = $('.arc', stage), th = $('.th', stage);
  const bars = Object.fromEntries(['x', 'y', 'z', 'w'].map(k => [k, [$('.qb' + k, stage), $('.qv' + k, stage)]]));
  ctx.loop((dt, t) => {
    const a = ((t * 0.5) % (Math.PI * 2));
    const ang = a > Math.PI ? a - 2 * Math.PI : a;
    const ex = 170 + 60 * Math.cos(a), ey = 170 - 60 * Math.sin(a);
    arc.setAttribute('d', `M230 170 A60 60 0 ${a > Math.PI ? 1 : 0} 0 ${ex.toFixed(1)} ${ey.toFixed(1)}`);
    tt.innerHTML = `<g transform="translate(170 170) rotate(${(-a * 180 / Math.PI).toFixed(1)})"><ellipse cx="30" rx="10" ry="8" fill="#9CD35F"/><ellipse rx="26" ry="21" fill="#2F9E5B"/><ellipse rx="14" ry="11" fill="none" stroke="rgba(0,0,0,.25)" stroke-width="2"/></g>` + arrowLine(170, 170, 170 + 104 * Math.cos(a), 170 - 104 * Math.sin(a), 'x', 5);
    th.textContent = `${(ang * 180 / Math.PI).toFixed(0)}°  =  ${ang.toFixed(2)} rad`;
    const q = { x: 0, y: 0, z: Math.sin(ang / 2), w: Math.cos(ang / 2) };
    for (const k in q) { const [b, v] = bars[k]; const wv = q[k] * 60; b.setAttribute('x', (104 + Math.min(0, wv)).toFixed(1)); b.setAttribute('width', Math.abs(wv).toFixed(1)); v.textContent = q[k].toFixed(3); }
  });
};

/* turtlesim-style canvas animations */
ANIM.turtle = (stage, o, ctx) => {
  const mode = o.mode || 'circle';
  const S = 420, k = S / TSIM.W;
  const { c, g } = makeCanvas(S, S);
  const wrap = h('div', { class: 'htmlstage', style: 'grid-template-columns:minmax(0,1fr) minmax(0,.8fr);align-items:center' });
  const side = h('div', { class: 'tside' });
  wrap.append(h('div', {}, c), side); stage.append(wrap);
  const trail = document.createElement('canvas'); trail.width = S * 2; trail.height = S * 2; const tg = trail.getContext('2d'); tg.scale(2, 2);
  const P = (x, y) => [x * k, S - y * k];
  const T = [{ x: 5.54, y: 5.54, th: 0, name: 'turtle1', col: '#2F9E5B' }];
  const keys = { up: 0, down: 0, left: 0, right: 0 };
  let script = [], si = 0, st = 0;
  if (mode === 'teleop') {
    script = [['up', 1.3], ['left', .55], ['up', 1.0], ['right', .9], ['up', .8], ['left', 1.2], ['up', 1.1], ['right', .6], ['down', .6], ['left', .8], ['up', 1.2]];
    side.innerHTML = `<div class="keys"><span></span><kbd data-k="up">${icon('arrowU')}</kbd><span></span><kbd data-k="left">${icon('arrowL')}</kbd><kbd data-k="down">${icon('arrowD')}</kbd><kbd data-k="right">${icon('arrowR')}</kbd></div><div class="cmdline tw"></div>`;
  } else if (mode === 'circle') {
    side.innerHTML = `<div class="cmdline">ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"</div><p class="sidecap">Forward speed + turn rate = a circle.</p>`;
  } else if (mode === 'spawn') {
    side.innerHTML = `<div class="cmdline">$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"<br><br><span class="resp" style="opacity:0;transition:opacity .4s">response:<br>turtlesim.srv.Spawn_Response(name='turtle2')</span></div>`;
  } else if (mode === 'relative') {
    T.push({ x: 3.2, y: 3.4, th: 0.5, name: 'turtle2', col: '#1E6FD9' });
    side.innerHTML = `<div class="readout"><b>turtle1 seen from turtle2</b><div class="ro"></div></div>`;
  } else if (mode === 'train') {
    T.push({ x: 2.5, y: 2.5, th: 0, name: 'turtle2', col: '#1E6FD9' }, { x: 1.5, y: 1.5, th: 0, name: 'turtle3', col: '#C2410C' });
    side.innerHTML = `<p class="sidecap">turtle2 follows turtle1.<br>turtle3 follows turtle2.<br><br>Same follower node, different parameters.</p>`;
  }
  let spawned = mode !== 'spawn', t0 = 0;
  const kb = $$('kbd', side), tw = $('.tw', side), ro = $('.ro', side), resp = $('.resp', side);
  ctx.loop((dt, t) => {
    const A = T[0];
    let v = 0, w = 0;
    if (mode === 'teleop') {
      st += dt; if (st > script[si][1]) { st = 0; si = (si + 1) % script.length; }
      const key = script[si][0]; Object.keys(keys).forEach(q => { keys[q] = q === key ? 1 : 0; });
      v = keys.up ? 2 : keys.down ? -2 : 0; w = keys.left ? 2 : keys.right ? -2 : 0;
      if (A.x < 1.5 || A.x > 9.6 || A.y < 1.5 || A.y > 9.6) { const back = Math.atan2(5.5 - A.y, 5.5 - A.x); A.th = A.th + wrapAngle(back - A.th) * Math.min(1, dt * 3); }
      kb.forEach(e => e.classList.toggle('on', !!keys[e.dataset.k]));
      if (tw) tw.innerHTML = `/turtle1/cmd_vel<br>linear.x: <b>${v.toFixed(1)}</b>   angular.z: <b>${w.toFixed(1)}</b>`;
    } else if (mode === 'circle') { v = 2.0; w = 1.8; }
    else if (mode === 'spawn') { t0 += dt; if (!spawned && t0 > 1.4) { spawned = true; T.push({ x: 2, y: 2, th: 0, name: 'turtle2', col: '#1E6FD9', pop: 0 }); if (resp) resp.style.opacity = 1; } if (t0 > 7) { t0 = 0; spawned = false; T.length = 1; if (resp) resp.style.opacity = 0; } }
    else if (mode === 'relative') { const u = t * 0.35; A.x = 6.6 + 2.2 * Math.cos(u); A.y = 6.2 + 2.2 * Math.sin(u); A.th = u + Math.PI / 2; }
    else if (mode === 'train') { const u = t * 0.5; const nx = 5.54 + 3.4 * Math.sin(u), ny = 5.54 + 2.6 * Math.sin(2 * u); A.th = Math.atan2(ny - A.y, nx - A.x); A.x = nx; A.y = ny; }
    if (mode === 'teleop' || mode === 'circle') {
      const [px0, py0] = P(A.x, A.y);
      A.th += w * dt; A.x = clamp(A.x + v * Math.cos(A.th) * dt, 0.3, 10.8); A.y = clamp(A.y + v * Math.sin(A.th) * dt, 0.3, 10.8);
      const [px1, py1] = P(A.x, A.y); tg.strokeStyle = TSIM.trail; tg.lineWidth = 2; tg.beginPath(); tg.moveTo(px0, py0); tg.lineTo(px1, py1); tg.stroke();
    }
    if (mode === 'train') {
      for (let i = 1; i < T.length; i++) { const L = T[i - 1], F = T[i]; const dx = L.x - F.x, dy = L.y - F.y; const ang = wrapAngle(Math.atan2(dy, dx) - F.th); const dist = Math.hypot(dx, dy); const fv = Math.min(3, 1.4 * Math.max(0, dist - 0.9)); F.th += clamp(5 * ang, -4, 4) * dt; const [a0, b0] = P(F.x, F.y); F.x += fv * Math.cos(F.th) * dt; F.y += fv * Math.sin(F.th) * dt; const [a1, b1] = P(F.x, F.y); tg.strokeStyle = i === 1 ? 'rgba(160,200,255,.7)' : 'rgba(255,190,150,.7)'; tg.lineWidth = 2; tg.beginPath(); tg.moveTo(a0, b0); tg.lineTo(a1, b1); tg.stroke(); }
      const [a0, b0] = P(A.x, A.y); tg.fillStyle = TSIM.trail; tg.fillRect(a0 - 1, b0 - 1, 2, 2);
    }
    g.fillStyle = TSIM.bg; g.fillRect(0, 0, S, S);
    g.drawImage(trail, 0, 0, S, S);
    if (mode === 'relative') {
      const B = T[1]; const [ax, ay] = P(A.x, A.y), [bx, by] = P(B.x, B.y);
      const dx = A.x - B.x, dy = A.y - B.y, c2 = Math.cos(B.th), s2 = Math.sin(B.th);
      const lx = c2 * dx + s2 * dy, ly = -s2 * dx + c2 * dy;
      const [fx, fy] = P(B.x + lx * c2, B.y + lx * s2);
      g.setLineDash([6, 5]); g.lineWidth = 2.5; g.strokeStyle = '#fff'; g.beginPath(); g.moveTo(bx, by); g.lineTo(ax, ay); g.stroke();
      g.setLineDash([]); g.strokeStyle = '#FF7B7B'; g.lineWidth = 4; g.beginPath(); g.moveTo(bx, by); g.lineTo(fx, fy); g.stroke();
      g.strokeStyle = '#7BEA9C'; g.beginPath(); g.moveTo(fx, fy); g.lineTo(ax, ay); g.stroke();
      drawAxes(g, bx, by, B.th, 46);
      if (ro) ro.innerHTML = `<div><span style="color:var(--c-x)">dx</span> = ${lx.toFixed(2)} m</div><div><span style="color:var(--c-y)">dy</span> = ${ly.toFixed(2)} m</div><div>distance = ${Math.hypot(lx, ly).toFixed(2)} m</div><div>angle = ${(Math.atan2(ly, lx) * 180 / Math.PI).toFixed(0)}°</div>`;
    }
    T.forEach((q, i) => { let s2 = 1.15; if (q.pop != null) { q.pop = Math.min(1, q.pop + dt * 3); s2 *= 0.4 + 0.6 * q.pop; } const [x, y] = P(q.x, q.y); drawTurtle(g, x, y, q.th, s2, q.col, T.length > 1 ? q.name : ''); });
    if (mode === 'circle') { g.font = '600 13px JetBrains Mono, monospace'; g.fillStyle = '#fff'; g.fillText('linear.x = 2.0   angular.z = 1.8', 12, 22); }
  });
};
function drawAxes(g, x, y, th, L) {
  const ax = (dx, dy, col) => { g.strokeStyle = col; g.lineWidth = 4; g.lineCap = 'round'; g.beginPath(); g.moveTo(x, y); g.lineTo(x + dx, y + dy); g.stroke(); };
  ax(L * Math.cos(th), -L * Math.sin(th), '#FF5A5F'); ax(-L * Math.sin(th), -L * Math.cos(th), '#4ADE80');
}

ANIM.turtlePlay = (stage, o, ctx) => {
  const S = 380, k = S / TSIM.W;
  const { c, g } = makeCanvas(S, S);
  const id = ++UID;
  stage.innerHTML = `<div class="play-ui"><div class="cv"></div><div>
    <div class="sliders">
      <div class="slider"><label for="lx${id}">linear.x</label><input id="lx${id}" type="range" min="-3" max="3" step="0.1" value="2"><output>2.0</output></div>
      <div class="slider"><label for="az${id}">angular.z</label><input id="az${id}" type="range" min="-3" max="3" step="0.1" value="1.8"><output>1.8</output></div>
    </div>
    <div class="btns"><button class="btn btn-primary btn-sm pub1">Publish once</button><button class="btn btn-sm pubr" aria-pressed="false">Publish at 1 Hz</button><button class="btn btn-ghost btn-sm clr">Clear</button></div>
    <div class="cmdline"></div></div></div>`;
  $('.cv', stage).append(c);
  const trail = document.createElement('canvas'); trail.width = S * 2; trail.height = S * 2; const tg = trail.getContext('2d'); tg.scale(2, 2);
  const lx = $(`#lx${id}`, stage), az = $(`#az${id}`, stage), cmd = $('.cmdline', stage), pubr = $('.pubr', stage);
  const T = { x: 5.54, y: 5.54, th: 0 }; let active = 0, rate = false, acc = 0, v = 0, w = 0;
  const upd = () => { $$('output', stage)[0].textContent = (+lx.value).toFixed(1); $$('output', stage)[1].textContent = (+az.value).toFixed(1); cmd.innerHTML = `<span style="color:var(--term-green)">$</span> ros2 topic pub ${rate ? '--rate 1' : '--once'} /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${(+lx.value).toFixed(1)}}, angular: {z: ${(+az.value).toFixed(1)}}}"`; };
  const fire = () => { v = +lx.value; w = +az.value; active = 1; Sound.play('tap'); };
  lx.addEventListener('input', upd); az.addEventListener('input', upd);
  $('.pub1', stage).addEventListener('click', fire);
  pubr.addEventListener('click', () => { rate = !rate; pubr.setAttribute('aria-pressed', rate); pubr.classList.toggle('btn-blue', rate); pubr.textContent = rate ? 'Stop (Ctrl+C)' : 'Publish at 1 Hz'; acc = 1; upd(); });
  $('.clr', stage).addEventListener('click', () => { tg.clearRect(0, 0, S, S); });
  upd();
  ctx.loop((dt) => {
    if (rate) { acc += dt; if (acc >= 1) { acc = 0; fire(); } }
    if (active > 0) {
      const dd = Math.min(dt, active); active -= dd;
      const [x0, y0] = [T.x * k, S - T.y * k];
      T.th += w * dd; T.x = clamp(T.x + v * Math.cos(T.th) * dd, 0.25, 10.85); T.y = clamp(T.y + v * Math.sin(T.th) * dd, 0.25, 10.85);
      tg.strokeStyle = TSIM.trail; tg.lineWidth = 2; tg.beginPath(); tg.moveTo(x0, y0); tg.lineTo(T.x * k, S - T.y * k); tg.stroke();
    }
    g.fillStyle = TSIM.bg; g.fillRect(0, 0, S, S); g.drawImage(trail, 0, 0, S, S);
    drawTurtle(g, T.x * k, S - T.y * k, T.th, 1.1);
  });
};

ANIM.follower = (stage, o, ctx) => {
  const S = 380, k = S / TSIM.W, id = ++UID;
  const { c, g } = makeCanvas(S, S);
  stage.innerHTML = `<div class="play-ui"><div class="cv"></div><div>
    <div class="sliders">
      <div class="slider"><label for="kl${id}">k_linear</label><input id="kl${id}" type="range" min="0" max="3" step="0.1" value="0.8"><output>0.8</output></div>
      <div class="slider"><label for="ka${id}">k_angular</label><input id="ka${id}" type="range" min="0" max="25" step="0.5" value="3"><output>3.0</output></div>
    </div>
    <div class="cmdline fc"></div><p class="sidecap fv" aria-live="polite"></p>
    <div class="btns"><button class="btn btn-sm rst">Reset turtles</button></div></div></div>`;
  $('.cv', stage).append(c);
  const kl = $(`#kl${id}`, stage), ka = $(`#ka${id}`, stage), fc = $('.fc', stage), fv = $('.fv', stage);
  const L = { x: 5.5, y: 5.5, th: 0 }, F = { x: 2, y: 2, th: 0, v: 0, w: 0 };
  const trail = []; let acc = 0, lt = 0;
  const upd = () => {
    $$('output', stage)[0].textContent = (+kl.value).toFixed(1); $$('output', stage)[1].textContent = (+ka.value).toFixed(1);
    fc.innerHTML = `cmd.linear.x  = <b>${(+kl.value).toFixed(1)}</b> * dist<br>cmd.angular.z = <b>${(+ka.value).toFixed(1)}</b> * angle`;
    const a = +ka.value, l = +kl.value;
    fv.textContent = a < 1 ? 'Turning gain is very low: the follower drifts wide and lags.' : a > 14 ? 'Turning gain is too high for a 10 Hz loop: watch it wobble and overshoot.' : l > 2.2 ? 'Fast and aggressive: it catches up, then overshoots.' : l < 0.3 ? 'Very gentle: it can never keep up.' : 'Nicely tuned: smooth and responsive.';
  };
  kl.addEventListener('input', upd); ka.addEventListener('input', upd); upd();
  $('.rst', stage).addEventListener('click', () => { F.x = 2; F.y = 2; F.th = 0; trail.length = 0; });
  ctx.loop((dt) => {
    lt += dt * 0.45;
    L.x = 5.54 + 3.6 * Math.sin(lt); const ny = 5.54 + 2.6 * Math.sin(2 * lt); L.th = Math.atan2(5.2 * Math.cos(2 * lt) * 0.45, 3.6 * Math.cos(lt) * 0.45); L.y = ny;
    acc += dt;
    if (acc >= 0.1) { acc = 0; const dx = L.x - F.x, dy = L.y - F.y; const lxr = Math.cos(F.th) * dx + Math.sin(F.th) * dy, lyr = -Math.sin(F.th) * dx + Math.cos(F.th) * dy; const dist = Math.hypot(lxr, lyr), ang = Math.atan2(lyr, lxr); F.v = clamp(+kl.value * dist, 0, 4); F.w = clamp(+ka.value * ang, -12, 12); }
    F.th += F.w * dt; F.x = clamp(F.x + F.v * Math.cos(F.th) * dt, 0.2, 10.9); F.y = clamp(F.y + F.v * Math.sin(F.th) * dt, 0.2, 10.9);
    trail.push([F.x, F.y]); if (trail.length > 240) trail.shift();
    g.fillStyle = TSIM.bg; g.fillRect(0, 0, S, S);
    g.strokeStyle = 'rgba(160,200,255,.75)'; g.lineWidth = 2; g.beginPath(); trail.forEach(([x, y], i) => { i ? g.lineTo(x * k, S - y * k) : g.moveTo(x * k, S - y * k); }); g.stroke();
    g.setLineDash([5, 5]); g.strokeStyle = 'rgba(255,255,255,.6)'; g.beginPath(); g.moveTo(F.x * k, S - F.y * k); g.lineTo(L.x * k, S - L.y * k); g.stroke(); g.setLineDash([]);
    drawTurtle(g, L.x * k, S - L.y * k, L.th, 1.1, '#2F9E5B', 'turtle1');
    drawTurtle(g, F.x * k, S - F.y * k, F.th, 1.1, '#1E6FD9', 'turtle2');
  });
};

ANIM.serial = (stage, o, ctx) => {
  const mode = o.mode || 'bytes';
  if (mode === 'arch') {
    stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="Computer running ROS 2 connected to a microcontroller that drives motors">
      <g class="slide-up" style="animation-delay:.1s"><rect x="20" y="96" width="170" height="120" rx="18" class="fill-surface stroke-node" stroke-width="3"/><text class="t-b" x="105" y="136" text-anchor="middle" style="font-weight:600">Computer</text><text class="t-s" x="105" y="158" text-anchor="middle">Ubuntu + ROS 2</text><text class="t-s" x="105" y="190" text-anchor="middle" style="fill:var(--c-node);font-weight:600">decides</text></g>
      <path d="M190 156 C 230 120, 250 196, 290 156" fill="none" class="stroke-ink" stroke-width="6" stroke-linecap="round"/>
      <text class="t-s" x="240" y="214" text-anchor="middle">USB serial</text>
      ${flowDots('M190 156 C 230 120, 250 196, 290 156', { n: 3, dur: 1.4, shape: 'dot', cls: 'fill-msg' })}
      <g class="slide-up" style="animation-delay:.5s"><rect x="290" y="106" width="130" height="100" rx="12" fill="#1C5E52" stroke="#0E3B33" stroke-width="3"/>${[0, 1, 2, 3, 4].map(i => `<rect x="${300 + i * 23}" y="98" width="8" height="10" class="fill-copper"/><rect x="${300 + i * 23}" y="204" width="8" height="10" class="fill-copper"/>`).join('')}<rect x="324" y="134" width="62" height="44" rx="5" fill="#111"/><text class="t-s" x="355" y="230" text-anchor="middle">microcontroller</text><text class="t-s" x="355" y="250" text-anchor="middle" style="fill:var(--accent);font-weight:600">precise timing</text></g>
      <path d="M420 140 L470 110 M420 172 L470 202" class="stroke-ink" stroke-width="3"/>
      <g class="slide-up" style="animation-delay:.9s"><g class="wl"><circle cx="510" cy="100" r="34" class="fill-ink"/><circle cx="510" cy="100" r="12" class="fill-s3"/></g><g class="wr"><circle cx="510" cy="212" r="34" class="fill-ink"/><circle cx="510" cy="212" r="12" class="fill-s3"/></g><text class="t-s" x="510" y="272" text-anchor="middle" style="fill:var(--good);font-weight:600">moves</text></g>
      <g class="spin" style="transform-box:view-box;transform-origin:510px 100px"><path d="M510 72v56M482 100h56" class="stroke-muted" stroke-width="3"/></g><g class="spin" style="transform-box:view-box;transform-origin:510px 212px"><path d="M510 184v56M482 212h56" class="stroke-muted" stroke-width="3"/></g></svg>`;
    return;
  }
  const garbage = mode === 'garbage', watchdog = mode === 'watchdog';
  stage.innerHTML = `<svg class="st" viewBox="0 0 600 340" role="img" aria-label="Characters travel over a serial cable">
    <rect x="16" y="110" width="150" height="100" rx="16" class="fill-surface stroke-node" stroke-width="3"/><text class="t-b" x="91" y="148" text-anchor="middle" style="font-weight:600">ROS 2 node</text><text class="t-s bd1" x="91" y="172" text-anchor="middle">115200 baud</text><text class="t-m sent" x="91" y="196" text-anchor="middle" style="font-size:13px"></text>
    <line x1="166" y1="160" x2="330" y2="160" class="stroke-ink cable" stroke-width="6" stroke-linecap="round"/>
    <g class="chars"></g>
    <rect x="330" y="96" width="140" height="128" rx="12" fill="#1C5E52" stroke="#0E3B33" stroke-width="3"/>
    <text x="400" y="124" text-anchor="middle" style="font:600 13px var(--font-body);fill:#DDF7EE">microcontroller</text>
    <rect x="344" y="134" width="112" height="28" rx="6" fill="#0B1F1A"/><text class="buf" x="400" y="153" text-anchor="middle" style="font:13px var(--font-mono);fill:#F7C95C"></text>
    <text class="prs" x="400" y="186" text-anchor="middle" style="font:600 13px var(--font-mono);fill:#DDF7EE"></text>
    <text class="bd2" x="400" y="210" text-anchor="middle" style="font:12px var(--font-body);fill:${garbage ? '#FF8A8A' : '#A9D9CB'}">${garbage ? '9600 baud' : '115200 baud'}</text>
    <g transform="translate(530 104)"><circle r="30" class="fill-ink"/><g class="w1"><path d="M0 -24v48M-24 0h48" class="stroke-muted" stroke-width="3"/></g><circle r="9" class="fill-s3"/><text class="t-s" y="50" text-anchor="middle">left</text></g>
    <g transform="translate(530 238)"><circle r="30" class="fill-ink"/><g class="w2"><path d="M0 -24v48M-24 0h48" class="stroke-muted" stroke-width="3"/></g><circle r="9" class="fill-s3"/><text class="t-s" y="50" text-anchor="middle">right</text></g>
    ${watchdog ? `<g transform="translate(330 262)"><rect width="140" height="12" rx="6" class="fill-s3"/><rect class="wdb fill-bad" width="0" height="12" rx="6"/><text class="t-s wdt" x="70" y="34" text-anchor="middle">watchdog</text></g>` : ''}
    <text class="t-s note" x="248" y="300" text-anchor="middle"></text></svg>`;
  const chars = $('.chars', stage), buf = $('.buf', stage), prs = $('.prs', stage), sent = $('.sent', stage), w1 = $('.w1', stage), w2 = $('.w2', stage), note = $('.note', stage), cable = $('.cable', stage);
  const wdb = $('.wdb', stage), wdt = $('.wdt', stage);
  const cmds = watchdog ? ['50,50\n', '50,50\n', '50,50\n', null, null, null] : ['60,60\n', '-40,40\n', '0,0\n'];
  let ci = 0, queue = [], fl = [], spdL = 0, spdR = 0, a1 = 0, a2 = 0, bufS = '', tNext = 0, sinceCmd = 0;
  const junk = 'ÿ¤~@#¿§ñ%&';
  ctx.loop((dt) => {
    tNext -= dt; sinceCmd += dt;
    if (tNext <= 0 && !queue.length && !fl.length) {
      const cmd = cmds[ci % cmds.length]; ci++;
      if (cmd) { queue = cmd.split(''); sent.textContent = JSON.stringify(cmd).slice(1, -1); cable.style.opacity = 1; } else { sent.textContent = '(node crashed)'; cable.style.opacity = .25; }
      tNext = watchdog ? 0.55 : 2.2;
    }
    if (queue.length && (!fl.length || fl[fl.length - 1].x > 196)) {
      const ch = queue.shift(); const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      const shown = garbage ? junk[(Math.random() * junk.length) | 0] : (ch === '\n' ? '\\n' : ch);
      t.textContent = shown; t.setAttribute('y', 150); t.setAttribute('text-anchor', 'middle'); t.setAttribute('style', `font:700 15px var(--font-mono);fill:${garbage ? 'var(--bad)' : 'var(--c-msg)'}`);
      chars.append(t); fl.push({ t, x: 172, ch: garbage ? shown : ch });
    }
    for (let i = fl.length - 1; i >= 0; i--) {
      const f = fl[i]; f.x += 190 * dt; f.t.setAttribute('x', f.x.toFixed(1));
      if (f.x >= 330) {
        f.t.remove(); fl.splice(i, 1);
        if (f.ch === '\n') {
          const parts = bufS.split(',').map(Number);
          if (parts.length === 2 && parts.every(Number.isFinite)) { spdL = parts[0]; spdR = parts[1]; prs.textContent = `L=${spdL}  R=${spdR}`; sinceCmd = 0; }
          bufS = '';
        } else {
          bufS += f.ch; if (garbage && bufS.length > 5) { prs.textContent = '?? parse error'; bufS = ''; spdL = spdR = 0; }
        }
        buf.textContent = bufS;
      }
    }
    if (watchdog) {
      const frac = clamp(sinceCmd / 0.5, 0, 1); wdb.setAttribute('width', (140 * frac).toFixed(1));
      if (sinceCmd > 0.5 && (spdL || spdR)) { spdL = spdR = 0; prs.textContent = 'STOP: timeout'; }
      wdt.textContent = sinceCmd > 0.5 ? 'no command for 0.5 s → motors off' : `watchdog ${sinceCmd.toFixed(2)} s`;
      wdt.style.fill = sinceCmd > 0.5 ? 'var(--bad)' : '';
    }
    note.textContent = garbage ? 'mismatched baud rates turn bytes into garbage' : watchdog ? 'commands stop → firmware stops the motors' : 'each line is one command: left,right';
    a1 += spdL * dt * 4; a2 += spdR * dt * 4;
    w1.setAttribute('transform', `rotate(${a1.toFixed(1)})`); w2.setAttribute('transform', `rotate(${a2.toFixed(1)})`);
  });
};

ANIM.diffdrive = (stage, o, ctx) => {
  const id = ++UID, B = 0.3;
  stage.innerHTML = `<div class="play-ui"><div><svg class="st" viewBox="0 0 320 320" role="img" aria-label="Differential drive robot"><g class="pathg"></g><g transform="translate(160 200)">
      <rect x="-62" y="-50" width="124" height="100" rx="26" class="fill-surface stroke-ink" stroke-width="3"/>
      <rect x="-78" y="-34" width="16" height="68" rx="5" class="fill-ink"/><rect x="62" y="-34" width="16" height="68" rx="5" class="fill-ink"/>
      <path d="M-14 -36 L0 -52 L14 -36z" class="fill-accent"/>
      <g class="al"></g><g class="ar"></g>
      <text class="t-m lv" x="-70" y="72" text-anchor="middle"></text><text class="t-m rv" x="70" y="72" text-anchor="middle"></text></g></svg></div>
    <div><div class="sliders">
      <div class="slider"><label for="v${id}">v (m/s)</label><input id="v${id}" type="range" min="-0.5" max="0.5" step="0.05" value="0.3"><output>0.30</output></div>
      <div class="slider"><label for="w${id}">w (rad/s)</label><input id="w${id}" type="range" min="-2" max="2" step="0.1" value="0.8"><output>0.8</output></div></div>
      <div class="cmdline eq"></div><p class="sidecap ex"></p></div></div>`;
  const vi = $(`#v${id}`, stage), wi = $(`#w${id}`, stage), eq = $('.eq', stage), ex = $('.ex', stage), al = $('.al', stage), ar = $('.ar', stage), lv = $('.lv', stage), rv = $('.rv', stage), pg = $('.pathg', stage);
  const arrow = (x, val) => { const L = clamp(val * 200, -70, 70); if (Math.abs(L) < 2) return ''; const y2 = -L; const col = val >= 0 ? 'var(--good)' : 'var(--bad)'; return `<line x1="${x}" y1="0" x2="${x}" y2="${y2 * 0.82}" stroke="${col}" stroke-width="6" stroke-linecap="round"/><path d="M${x - 9} ${y2 * 0.8} L${x} ${y2} L${x + 9} ${y2 * 0.8}" fill="${col}"/>`; };
  const upd = () => {
    const v = +vi.value, w = +wi.value, l = v - w * B / 2, r = v + w * B / 2;
    $$('output', stage)[0].textContent = v.toFixed(2); $$('output', stage)[1].textContent = w.toFixed(1);
    al.innerHTML = arrow(-70, l); ar.innerHTML = arrow(70, r);
    lv.textContent = l.toFixed(2); rv.textContent = r.toFixed(2);
    eq.innerHTML = `left  = v − w·b/2 = <b>${l.toFixed(2)}</b> m/s<br>right = v + w·b/2 = <b>${r.toFixed(2)}</b> m/s<br><span style="color:var(--term-dim)">b = track width = ${B} m</span>`;
    ex.textContent = Math.abs(v) < 0.01 && Math.abs(w) > 0.05 ? 'Wheels spin in opposite directions: the robot turns in place.' : Math.abs(w) < 0.05 ? 'Same speed on both wheels: straight line.' : (w > 0 ? 'Right wheel faster: it curves left.' : 'Left wheel faster: it curves right.');
    let x = 0, y = 0, th = Math.PI / 2, d = 'M160 200';
    for (let i = 0; i < 40; i++) { const dt = 0.1; th += w * dt; x += v * Math.cos(th) * dt * 220; y += v * Math.sin(th) * dt * 220; d += ` L${(160 + x).toFixed(1)} ${(200 - y).toFixed(1)}`; }
    pg.innerHTML = `<path d="${d}" fill="none" stroke="var(--c-topic)" stroke-width="3" stroke-dasharray="6 6"/>`;
  };
  vi.addEventListener('input', upd); wi.addEventListener('input', upd); upd();
};
