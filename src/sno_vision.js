/* ============================== SnoBots: Vision lab ============================== */
// The Vision tab is a small rqt_image_view: it subscribes to any sensor_msgs/Image topic and draws it, and a click
// reads a pixel's RGB and OpenCV HSV values (handy for picking thresholds). It also lists the vision missions and
// loads their starter code. cv2.imshow() windows from your Python node appear over the 3D view.
const SnoVision = {
  // draw an image buffer on a canvas: rgb8, bgr8, rgba8, bgra8, mono8, 8UC1/3/4, mono16/16UC1, 32FC1
  draw(cv, w, h, enc, data) {
    if (!cv || !data || !(w > 0) || !(h > 0)) return;
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    const g = cv.getContext('2d'), im = g.createImageData(w, h), o = im.data, n = w * h, E = String(enc).toLowerCase();
    if (E === 'rgb8' || E === 'bgr8' || E === '8uc3') { const r = E === 'rgb8' ? 0 : 2; for (let i = 0, j = 0, k = 0; i < n; i++, j += 3, k += 4) { o[k] = data[j + r]; o[k + 1] = data[j + 1]; o[k + 2] = data[j + 2 - r]; o[k + 3] = 255; } }
    else if (E === 'rgba8' || E === 'bgra8' || E === '8uc4') { const r = E === 'rgba8' ? 0 : 2; for (let i = 0, j = 0; i < n; i++, j += 4) { o[j] = data[j + r]; o[j + 1] = data[j + 1]; o[j + 2] = data[j + 2 - r]; o[j + 3] = 255; } }
    else if (E === 'mono8' || E === '8uc1') { for (let i = 0, k = 0; i < n; i++, k += 4) { o[k] = o[k + 1] = o[k + 2] = data[i]; o[k + 3] = 255; } }
    else if (E === 'mono16' || E === '16uc1' || E === '32fc1') {
      const b = data.slice ? data.slice(0, n * (E === '32fc1' ? 4 : 2)).buffer : data, v = E === '32fc1' ? new Float32Array(b) : new Uint16Array(b);
      let lo = Infinity, hi = -Infinity; for (let i = 0; i < n; i++) { const x = v[i]; if (Number.isFinite(x) && x > 0) { if (x < lo) lo = x; if (x > hi) hi = x; } }
      const sc = hi > lo ? 255 / (hi - lo) : 0; for (let i = 0, k = 0; i < n; i++, k += 4) { const x = v[i], y = Number.isFinite(x) && x > 0 ? (x - lo) * sc : 0; o[k] = o[k + 1] = o[k + 2] = y; o[k + 3] = 255; }
    } else return;
    g.putImageData(im, 0, 0);
  },
  // OpenCV's 8-bit HSV: H in 0..179, S and V in 0..255
  hsv(r, g, b) {
    const v = Math.max(r, g, b), mn = Math.min(r, g, b), d = v - mn, s = v ? Math.round(255 * d / v) : 0;
    let hh = 0; if (d) { hh = v === r ? 60 * (g - b) / d : v === g ? 120 + 60 * (b - r) / d : 240 + 60 * (r - g) / d; if (hh < 0) hh += 360; }
    return [Math.round(hh / 2) % 180, s, v];
  },
  missions(sim) { return sim.def.missions.filter(m => m.vision); },
  left(sim) { return this.missions(sim).filter(m => !sim.done(m.id)).length; },
  /* ---------- the image viewer ---------- */
  topics(sim) { return [...sim.bus.topics.values()].filter(t => sim.bus.types(t.name).includes('sensor_msgs/msg/Image')).map(t => t.name).sort(); },
  view(sim, topic) {
    this.unview(sim);
    const B = sim.bus, node = '/image_view'; if (!topic) return;
    if (!B.nodes.has(node)) B.addNode(node, 'tool');
    const V = sim.vis = { topic, n: 0, t0: performance.now(), hz: 0, last: null, un: null };
    V.un = B.subscribe(node, topic, 'sensor_msgs/msg/Image', (m) => {
      V.n++; V.last = m;
      const cv = SnoUI.cur === sim && SnoUI.root && $('.vis-img', SnoUI.root); if (cv) { this.draw(cv, m.width, m.height, m.encoding, m.data); this.cross(sim); }
    });
    sim.tabOff = () => this.unview(sim);
  },
  unview(sim) { const V = sim.vis; if (V && V.un) V.un(); sim.vis = null; if (sim.bus.nodes.has('/image_view')) sim.bus.removeNode('/image_view'); },
  tick(sim) {
    const V = sim.vis, el = SnoUI.root && $('.vis-info', SnoUI.root); if (!V || !el) return;
    const now = performance.now(), dt = (now - V.t0) / 1000; if (dt >= 1) { V.hz = V.n / dt; V.n = 0; V.t0 = now; }
    const m = V.last;
    el.textContent = m ? `${V.hz.toFixed(1)} Hz · ${m.width}×${m.height} · ${m.encoding}` : sim.camCfg && !sim.camRunning() ? 'the camera driver is not running' : !SnoUI.view || !SnoUI.view.ready ? 'no images: the 3D view is off in this browser' : 'waiting for images…';
    const sel = $('#vis-topic', SnoUI.root); if (sel && sel.options.length !== this.topics(sim).length) this.fillTopics(sim, sel);
  },
  fillTopics(sim, sel) {
    const ts = this.topics(sim), cur = sim.vis ? sim.vis.topic : sim.visTopic;
    sel.innerHTML = ts.map(t => `<option value="${esc(t)}"${t === cur ? ' selected' : ''}>${esc(t)}</option>`).join('') || '<option value="">(no image topics)</option>';
  },
  // click a pixel: its RGB and HSV, the numbers you need for cv2.inRange
  pick(sim, e) {
    const V = sim.vis, m = V && V.last, cv = e.currentTarget; if (!m) return;
    const r = cv.getBoundingClientRect(), x = Math.max(0, Math.min(m.width - 1, Math.floor((e.clientX - r.left) / r.width * m.width))), y = Math.max(0, Math.min(m.height - 1, Math.floor((e.clientY - r.top) / r.height * m.height)));
    sim.visPick = { x, y }; this.cross(sim); this.readout(sim);
  },
  readout(sim) {
    const V = sim.vis, m = V && V.last, P = sim.visPick, el = SnoUI.root && $('.vis-pix', SnoUI.root); if (!m || !P || !el) return;
    const E = m.encoding.toLowerCase(), ch = { rgb8: 3, bgr8: 3, '8uc3': 3, rgba8: 4, bgra8: 4, '8uc4': 4, mono8: 1, '8uc1': 1 }[E];
    if (!ch) { el.textContent = `x ${P.x}, y ${P.y} · ${m.encoding} pixels cannot be read here`; return; }
    const i = (P.y * m.width + P.x) * ch, d = m.data; let rr, gg, bb;
    if (ch === 1) rr = gg = bb = d[i]; else if (E.startsWith('rgb')) { rr = d[i]; gg = d[i + 1]; bb = d[i + 2]; } else { bb = d[i]; gg = d[i + 1]; rr = d[i + 2]; }
    const [H, Sv, Vv] = this.hsv(rr, gg, bb);
    el.innerHTML = `<span class="vis-sw" style="background:rgb(${rr},${gg},${bb})"></span><b>x ${P.x}, y ${P.y}</b> · BGR (${bb}, ${gg}, ${rr}) · HSV (${H}, ${Sv}, ${Vv})`;
  },
  cross(sim) {
    const P = sim.visPick, c = SnoUI.root && $('.vis-cross', SnoUI.root), V = sim.vis, m = V && V.last; if (!c) return;
    if (!P || !m) { c.hidden = true; return; }
    c.hidden = false; c.style.left = ((P.x + 0.5) / m.width * 100) + '%'; c.style.top = ((P.y + 0.5) / m.height * 100) + '%';
    if (sim.visPickLive !== false) this.readout(sim);
  },
  /* ---------- starter code ---------- */
  loadTpl(sim, key) {
    const tpls = sim.def.py, t = tpls[key]; if (!t) return;
    const id = sim.id, cur = S.sno.code[id], was = tpls[S.sno.tpl[id]];
    const go = () => { S.sno.tpl[id] = key; S.sno.code[id] = t.code; save({ local: true }); sim.errLine = 0; SnoUI.focusTab('code'); toast(`Loaded <b>${esc(t.label)}</b>. Press Run.`, { mood: 'happy', ms: 2400 }); };
    if (cur && cur !== t.code && (!was || cur !== was.code)) modal({ mood: 'think', title: 'Replace your code?', body: `Loading “${esc(t.label)}” replaces what is in the editor.`, actions: [{ label: 'Load it', cls: 'btn-primary', onClick: go }, { label: 'Cancel', cls: 'btn-ghost' }] });
    else go();
  },
  /* ---------- the Vision tab ---------- */
  panel(sim, body) {
    const w = h('div', { class: 'lpad vis' }); body.append(w);
    const ts = this.topics(sim), topic = ts.includes(sim.visTopic) ? sim.visTopic : (ts.find(t => /image_raw$/.test(t)) || ts[0] || '');
    const tpls = sim.def.py, vt = Object.entries(tpls).filter(([, t]) => t.vision);
    w.innerHTML = `<div class="vis-viewer">
        <div class="vis-bar"><label for="vis-topic">${icon('camera')}<span class="sr">Image topic</span></label><select class="sel" id="vis-topic"></select><span class="vis-info" aria-live="off">waiting for images…</span></div>
        <div class="vis-frame"><canvas class="vis-img" width="320" height="240" tabindex="0" aria-label="Camera image. Click or press Enter to read the pixel under the marker."></canvas><span class="vis-cross" hidden></span></div>
        <p class="vis-pix">Click the image to read a pixel's BGR and HSV values (OpenCV ranges: H 0–179, S and V 0–255).</p>
      </div>
      <p class="sidecap">${md(sim.def.visionNote || '')}</p>
      <h4>Vision missions <span>${this.missions(sim).length - this.left(sim)} of ${this.missions(sim).length} done</span></h4>
      <ul class="vis-missions"></ul>
      <h4>OpenCV starter code <span>loads into the Code tab</span></h4>
      <div class="vis-tpls"></div>`;
    const sel = $('#vis-topic', w); this.fillTopics(sim, sel); if (topic) sel.value = topic;
    sel.addEventListener('change', () => { sim.visTopic = sel.value; sim.visPick = null; this.view(sim, sel.value); this.cross(sim); });
    const cv = $('.vis-img', w);
    cv.addEventListener('click', (e) => this.pick(sim, e));
    cv.addEventListener('keydown', (e) => {
      const m = sim.vis && sim.vis.last; if (!m) return; const P = sim.visPick || (sim.visPick = { x: m.width >> 1, y: m.height >> 1 });
      const st = e.shiftKey ? 10 : 1, k = { ArrowLeft: [-st, 0], ArrowRight: [st, 0], ArrowUp: [0, -st], ArrowDown: [0, st] }[e.key];
      if (k) { e.preventDefault(); P.x = Math.max(0, Math.min(m.width - 1, P.x + k[0])); P.y = Math.max(0, Math.min(m.height - 1, P.y + k[1])); this.cross(sim); }
      else if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); this.cross(sim); }
    });
    const ul = $('.vis-missions', w);
    this.missions(sim).forEach(m => {
      const done = sim.done(m.id), li = h('li', { class: done ? 'done' : '' });
      li.innerHTML = `<span class="vis-m-ic">${icon(done ? 'check' : 'target')}</span><div><b>${esc(m.title)}</b> <span class="xp">+${m.xp} XP</span><p>${md(m.text)}</p></div>`;
      if (m.tpl && tpls[m.tpl]) { const b = h('button', { class: 'btn btn-sm btn-ghost' }, 'Starter code'); b.addEventListener('click', () => this.loadTpl(sim, m.tpl)); $('div', li).append(b); }
      ul.append(li);
    });
    const tb = $('.vis-tpls', w);
    vt.forEach(([k, t]) => { const b = h('button', { class: 'btn btn-sm' }, `${icon('python')}${esc(t.label)}`); b.addEventListener('click', () => this.loadTpl(sim, k)); tb.append(b); });
    sim.visTopic = topic; this.view(sim, topic);
    if (sim.vis && sim.vis.last) { this.draw(cv, sim.vis.last.width, sim.vis.last.height, sim.vis.last.encoding, sim.vis.last.data); }
    this.tick(sim);
  },
  /* ---------- cv2.imshow windows over the 3D view ---------- */
  imshow(sim, d, bin) {
    const W = sim.imwin || (sim.imwin = new Map()), muted = sim.immute || (sim.immute = new Set());
    if (d.type === 'imclose') { if (d.name) W.delete(d.name); else { W.clear(); muted.clear(); } this.paintWins(sim); return; }
    if (!bin || muted.has(d.name)) return;
    if (!W.has(d.name) && W.size >= 4) { if (!sim._imwarn) { sim._imwarn = true; sim.pyLog('[WARN] only the first 4 cv2.imshow windows are shown', 'warn'); } return; }
    W.set(d.name, { w: d.w, h: d.h, c: d.c, data: bin });
    this.paintWins(sim, d.name);
  },
  paintWins(sim, only) {
    const box = SnoUI.cur === sim && SnoUI.root && $('.sno-imshow', SnoUI.root); if (!box) return;
    const W = sim.imwin || new Map();
    $$('.imwin', box).forEach(f => { if (!W.has(f.dataset.name)) f.remove(); });
    for (const [name, im] of W) {
      if (only && name !== only) continue;
      let f = [...box.children].find(x => x.dataset.name === name);
      if (!f) {
        f = h('figure', { class: 'imwin', 'data-name': name });
        f.innerHTML = `<figcaption><span></span><button class="imx" aria-label="Close window">${icon('x')}</button></figcaption><canvas role="img"></canvas>`;
        $('span', f).textContent = name; $('canvas', f).setAttribute('aria-label', `OpenCV window: ${name}`);
        $('button', f).addEventListener('click', () => { sim.immute.add(name); W.delete(name); f.remove(); box.hidden = !W.size; });
        box.append(f);
      }
      this.draw($('canvas', f), im.w, im.h, im.c === 1 ? 'mono8' : im.c === 4 ? 'bgra8' : 'bgr8', im.data);
    }
    box.hidden = !W.size;
  },
};
