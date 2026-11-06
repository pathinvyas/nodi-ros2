/* ============================== exercises ============================== */
const EX = {};
const normCmd = (s) => String(s).trim().replace(/[“”]/g, '"').replace(/[‘’]/g, "'").replace(/'/g, '"').replace(/\s+/g, ' ').replace(/\s*([{},])\s*/g, '$1').replace(/:\s+/g, ':').replace(/;$/, '');
const KICK = {
  mcq: ['Choose the answer', 'target'], tf: ['True or false?', 'sparkle'], fill: ['Fill in the blanks', 'code'], build: ['Build the answer', 'cards'],
  order: ['Put them in order', 'topics'], match: ['Match the pairs', 'graph'], type: ['Type the command', 'terminal'], bug: ['Find the bug', 'bug'],
  pick: ['Tap the right part', 'node'], turtle: ['Drive the turtle', 'drive'],
};
function exTitle(q) { return h('h2', { class: 'q-title', html: md(q) }); }

EX.mcq = (ex, api) => {
  const el = h('div');
  el.append(exTitle(ex.q));
  if (ex.code) el.append(h('div', { class: 'ex-code', html: codeBlock(ex.code, ex.lang || 'py') }));
  const order = shuffle(ex.o.map((_, i) => i));
  const two = ex.o.length === 4 && ex.o.every(t => plain(t).length <= 24);
  const box = h('div', { class: 'opts' + (two ? ' two' : ''), role: 'radiogroup', 'aria-label': 'Answers' });
  let sel = -1, locked = false;
  const btns = order.map((oi, n) => {
    const b = h('button', { class: 'opt', role: 'radio', 'aria-checked': 'false', html: `<span class="key">${n + 1}</span><span>${md(ex.o[oi])}</span>` });
    b.dataset.oi = oi; b.addEventListener('click', () => choose(n)); box.append(b); return b;
  });
  function choose(n) { if (locked) return; sel = n; btns.forEach((b, i) => { b.classList.toggle('sel', i === n); b.setAttribute('aria-checked', String(i === n)); }); Sound.play('tap'); api.changed(); }
  el.append(box);
  return {
    el, ready: () => sel >= 0, key: (k) => { const n = parseInt(k, 10) - 1; if (n >= 0 && n < btns.length) choose(n); },
    check() { locked = true; const ok = +btns[sel].dataset.oi === ex.a; btns.forEach(b => { b.disabled = true; if (+b.dataset.oi === ex.a) b.classList.add('right'); }); if (!ok) btns[sel].classList.add('wrong'); return { ok, answer: md(ex.o[ex.a]) }; },
    solve() { choose(btns.findIndex(b => +b.dataset.oi === ex.a)); },
  };
};

EX.tf = (ex, api) => {
  const el = h('div');
  el.append(h('div', { class: 'tfbox', html: nodi('think', 'bob') + `<div class="speech">${md(ex.q)}</div>` }));
  const box = h('div', { class: 'opts two', role: 'radiogroup', 'aria-label': 'True or false' });
  let sel = null, locked = false;
  const mk = (v, n) => { const b = h('button', { class: 'opt', role: 'radio', 'aria-checked': 'false', html: `<span class="key">${n}</span><span style="font:600 18px var(--font-display)">${v ? 'True' : 'False'}</span>` }); b.addEventListener('click', () => choose(v)); box.append(b); return b; };
  const bT = mk(true, 1), bF = mk(false, 2);
  function choose(v) { if (locked) return; sel = v; bT.classList.toggle('sel', v); bF.classList.toggle('sel', !v); bT.setAttribute('aria-checked', String(v)); bF.setAttribute('aria-checked', String(!v)); Sound.play('tap'); api.changed(); }
  el.append(box);
  return {
    el, ready: () => sel !== null, key: (k) => { if (k === '1') choose(true); if (k === '2') choose(false); },
    check() { locked = true; const ok = sel === ex.a; [bT, bF].forEach(b => { b.disabled = true; }); (ex.a ? bT : bF).classList.add('right'); if (!ok) (sel ? bT : bF).classList.add('wrong'); return { ok, answer: ex.a ? 'True' : 'False' }; },
    solve() { choose(ex.a); },
  };
};

EX.fill = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q));
  const parts = ex.s.split('___'), nb = parts.length - 1;
  const code = h('div', { class: 'code fill-code', style: 'padding:14px 16px;margin-bottom:24px' });
  const blanks = [];
  parts.forEach((p, i) => {
    code.append(document.createTextNode(p));
    if (i < nb) { const b = h('button', { class: 'blank', 'aria-label': `Blank ${i + 1}` }, ' '); b.addEventListener('click', () => unfill(i)); code.append(b); blanks.push(b); }
  });
  const tiles = shuffle([...ex.b, ...(ex.x || [])].map((t, i) => ({ t, i })));
  const bank = h('div', { class: 'tiles' });
  const filled = Array(nb).fill(null);
  let locked = false;
  const tbtn = tiles.map((o) => { const b = h('button', { class: 'tile' }, o.t); b.addEventListener('click', () => fill(o, b)); bank.append(b); return b; });
  function mark() { const nx = filled.indexOf(null); blanks.forEach((b, i) => { b.classList.toggle('filled', filled[i] !== null); b.classList.toggle('next', i === nx); }); }
  function fill(o, b) { if (locked || b.classList.contains('ghost')) return; const k = filled.indexOf(null); if (k < 0) return; filled[k] = { o, b }; blanks[k].textContent = o.t; b.classList.add('ghost'); Sound.play('tap'); mark(); api.changed(); }
  function unfill(k) { if (locked || !filled[k]) return; filled[k].b.classList.remove('ghost'); filled[k] = null; blanks[k].textContent = ' '; mark(); api.changed(); }
  mark(); el.append(code, bank);
  const full = parts.map((p, i) => p + (i < nb ? ex.b[i] : '')).join('');
  return {
    el, ready: () => filled.every(Boolean),
    check() { locked = true; let ok = true; filled.forEach((f, i) => { const good = f.o.t === ex.b[i]; ok = ok && good; blanks[i].classList.add(good ? 'right' : 'wrong'); }); return { ok, answer: `<code>${esc(full)}</code>` }; },
    solve() { ex.b.forEach((ans) => { const k = tiles.findIndex((o, j) => o.t === ans && !tbtn[j].classList.contains('ghost')); if (k >= 0) fill(tiles[k], tbtn[k]); }); },
  };
};

function tileSequence(ex, api, opts) {
  const vertical = opts.vertical;
  const el = h('div'); el.append(exTitle(ex.q));
  const items = vertical ? ex.a.slice() : [...ex.a, ...(ex.x || [])];
  let order = shuffle(items.map((t, i) => ({ t, i })));
  if (vertical && order.every((o, i) => o.i === i)) order = order.reverse();
  const line = h('div', { class: vertical ? 'order-list' : 'answer-line', 'aria-label': 'Your answer' });
  const bank = h('div', { class: 'tiles' + (vertical ? ' col' : '') });
  const picked = []; let locked = false;
  const cls = 'tile' + (opts.words ? ' word' : '');
  const bt = order.map(o => { const b = h('button', { class: cls }, o.t); b.addEventListener('click', () => add(o, b)); bank.append(b); return b; });
  function render() {
    line.innerHTML = '';
    picked.forEach((p, k) => { const b = h('button', { class: cls, html: vertical ? `<span class="n">${k + 1}</span><span>${esc(p.o.t)}</span>` : esc(p.o.t) }); b.addEventListener('click', () => remove(k)); line.append(b); });
  }
  function add(o, b) { if (locked || b.classList.contains('ghost')) return; picked.push({ o, b }); b.classList.add('ghost'); Sound.play('tap'); render(); api.changed(); }
  function remove(k) { if (locked) return; picked[k].b.classList.remove('ghost'); picked.splice(k, 1); render(); api.changed(); }
  el.append(line, bank);
  const target = ex.a.join(' ');
  return {
    el, ready: () => vertical ? picked.length === ex.a.length : picked.length > 0,
    check() {
      locked = true; const got = picked.map(p => p.o.t).join(' ');
      const ok = got === target || (ex.alt || []).some(a => a.join(' ') === got);
      line.style.outline = `3px solid var(${ok ? '--good' : '--bad'})`; line.style.borderRadius = '14px';
      return { ok, answer: vertical ? ex.a.map((t, i) => `${i + 1}. ${esc(t)}`).join('<br>') : (opts.words ? esc(target) : `<code>${esc(target)}</code>`) };
    },
    solve() { ex.a.forEach(t => { const k = order.findIndex((o, j) => o.t === t && !bt[j].classList.contains('ghost')); if (k >= 0) add(order[k], bt[k]); }); },
  };
}
EX.build = (ex, api) => tileSequence(ex, api, { words: ex.words });
EX.order = (ex, api) => tileSequence(ex, api, { vertical: true, words: true });

EX.match = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q || 'Match the pairs'));
  const L = shuffle(ex.p.map((p, i) => ({ t: p[0], i }))), R = shuffle(ex.p.map((p, i) => ({ t: p[1], i })));
  const box = h('div', { class: 'match' }), cl = h('div', { class: 'col' }), cr = h('div', { class: 'col' });
  let sl = null, sr = null, done = 0, mistakes = 0;
  const mk = (o, side) => { const b = h('button', { class: 'opt', html: md(o.t) }); b.addEventListener('click', () => pick(side, o, b)); (side === 'L' ? cl : cr).append(b); o.b = b; return b; };
  L.forEach(o => mk(o, 'L')); R.forEach(o => mk(o, 'R'));
  function pick(side, o, b) {
    if (b.classList.contains('done')) return;
    if (side === 'L') { if (sl) sl.b.classList.remove('sel'); sl = o; } else { if (sr) sr.b.classList.remove('sel'); sr = o; }
    b.classList.add('sel'); Sound.play('tap');
    if (sl && sr) {
      const a = sl, c = sr; sl = sr = null;
      if (a.i === c.i) { [a.b, c.b].forEach(x => { x.classList.remove('sel'); x.classList.add('right'); setTimeout(() => { x.classList.remove('right'); x.classList.add('done'); x.disabled = true; }, 260); }); done++; if (done === ex.p.length) { api.changed(); setTimeout(() => api.autoCheck(), 450); } }
      else { mistakes++; Sound.play('bad'); [a.b, c.b].forEach(x => { x.classList.remove('sel'); x.classList.add('wrong'); setTimeout(() => x.classList.remove('wrong'), 450); }); }
    }
  }
  box.append(cl, cr); el.append(box);
  return {
    el, ready: () => done === ex.p.length,
    check: () => ({ ok: true, perfect: mistakes === 0, answer: '' }),
    solve() { ex.p.forEach((_, i) => { const a = L.find(o => o.i === i), c = R.find(o => o.i === i); pick('L', a, a.b); pick('R', c, c.b); }); },
  };
};

EX.type = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q));
  const id = 'ty' + (++UID);
  const box = h('div', { class: 'typebox' });
  const inp = h('input', { id, type: 'text', autocomplete: 'off', autocapitalize: 'off', spellcheck: 'false', placeholder: ex.ph || 'type here…', 'aria-label': 'Command' });
  box.append(h('span', { class: 'ps1' }, '$'), inp);
  inp.addEventListener('input', () => api.changed());
  const out = h('div', { class: 'type-out' });
  const hint = h('p', { class: 'hint-line' });
  if (ex.e) { const hb = h('button', { class: 'btn btn-ghost btn-sm', style: 'margin-top:10px;padding:0' }, 'Show a hint'); hb.addEventListener('click', () => { hint.innerHTML = icon('sparkle') + ' ' + md(ex.e); hb.remove(); }); el.append(box, hb, hint, out); }
  else el.append(box, out);
  setTimeout(() => inp.focus(), 60);
  return {
    el, ready: () => inp.value.trim().length > 0, input: inp,
    check() {
      inp.disabled = true; const g = normCmd(inp.value);
      const ok = [ex.a, ...(ex.alt || [])].some(a => normCmd(a) === g);
      box.style.borderColor = `var(${ok ? '--good' : '--bad'})`;
      if (ok && ex.out) out.innerHTML = `<div class="term" style="margin-top:12px"><div class="body" style="min-height:0">${esc(ex.out)}</div></div>`;
      return { ok, answer: `<code>${esc(ex.a)}</code>`, noExplain: true };
    },
    solve() { inp.value = ex.a; api.changed(); },
  };
};

EX.bug = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q));
  const lines = ex.code.split('\n');
  const code = h('div', { class: 'code buglines', role: 'listbox', 'aria-label': 'Code lines' });
  let sel = -1, locked = false;
  const rows = lines.map((l, i) => { const r = h('div', { class: 'ln', role: 'option', tabindex: '0', html: `<span class="no">${i + 1}</span>${hlLine(l, ex.lang || 'py') || ' '}` }); r.addEventListener('click', () => choose(i)); r.addEventListener('keydown', (e) => { if (e.key === ' ') { e.preventDefault(); choose(i); } }); code.append(r); return r; });
  function choose(i) { if (locked) return; sel = i; rows.forEach((r, k) => { r.classList.toggle('sel', k === i); r.setAttribute('aria-selected', String(k === i)); }); Sound.play('tap'); api.changed(); }
  el.append(code);
  return {
    el, ready: () => sel >= 0,
    check() { locked = true; const ok = sel === ex.a; rows[ex.a].classList.add('right'); if (!ok) rows[sel].classList.add('wrong'); rows.forEach(r => r.classList.remove('sel')); return { ok, answer: `line ${ex.a + 1}: <code>${esc(lines[ex.a].trim())}</code>` }; },
    solve() { choose(ex.a); },
  };
};

EX.pick = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q));
  const st = h('div', { class: 'stage', html: graphSVG(Object.assign({}, ex.g, { pick: true, reveal: false })) });
  el.append(st);
  let sel = null, locked = false;
  const groups = $$('[data-id]', st);
  const choose = (id) => { if (locked) return; sel = id; groups.forEach(g => g.classList.toggle('picked', g.dataset.id === id)); Sound.play('tap'); api.changed(); };
  groups.forEach(g => { g.addEventListener('click', () => choose(g.dataset.id)); g.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); choose(g.dataset.id); } }); });
  const label = (id) => (ex.g.nodes.find(n => n.id === id) || {}).label || id;
  return {
    el, ready: () => sel !== null,
    check() { locked = true; const ok = sel === ex.a; groups.forEach(g => { g.classList.remove('picked'); if (g.dataset.id === ex.a) g.classList.add('pick-right'); else if (g.dataset.id === sel) g.classList.add('pick-wrong'); }); return { ok, answer: `<code>${esc(label(ex.a))}</code>` }; },
    solve() { choose(ex.a); },
  };
};

EX.turtle = (ex, api) => {
  const el = h('div'); el.append(exTitle(ex.q));
  const S2 = 320, k = S2 / TSIM.W, id = ++UID;
  const { c, g } = makeCanvas(S2, S2);
  const wrap = h('div', { class: 'play-ui', style: 'padding:0' });
  const left = h('div'); left.append(c);
  const right = h('div', { html: `<div class="sliders">
    <div class="slider"><label for="tl${id}">linear.x</label><input id="tl${id}" type="range" min="-2" max="2" step="0.1" value="0"><output>0.0</output></div>
    <div class="slider"><label for="ta${id}">angular.z</label><input id="ta${id}" type="range" min="-2" max="2" step="0.1" value="0"><output>0.0</output></div></div>
    <div class="cmdline tc"></div><div class="btns"><button class="btn btn-blue btn-sm pv">${icon('play')} Preview 3 s</button></div>` });
  wrap.append(left, right); el.append(wrap);
  const lx = $(`#tl${id}`, right), az = $(`#ta${id}`, right), tc = $('.tc', right);
  let touched = false, anim = null, locked = false;
  const start = { x: 5.54, y: 5.54, th: 0 };
  const path = (v, w) => { const pts = []; let x = start.x, y = start.y, th = start.th; for (let i = 0; i <= 60; i++) { pts.push([x, y, th]); const dt = 0.05; th += w * dt; x += v * Math.cos(th) * dt; y += v * Math.sin(th) * dt; } return pts; };
  const draw = (upto) => {
    const v = +lx.value, w = +az.value, pts = path(v, w);
    g.fillStyle = TSIM.bg; g.fillRect(0, 0, S2, S2);
    g.setLineDash([5, 5]); g.strokeStyle = 'rgba(255,255,255,.55)'; g.lineWidth = 2; g.beginPath(); pts.forEach(([x, y], i) => { i ? g.lineTo(x * k, S2 - y * k) : g.moveTo(x * k, S2 - y * k); }); g.stroke(); g.setLineDash([]);
    const n = upto == null ? 0 : Math.min(pts.length - 1, upto);
    if (upto != null) { g.strokeStyle = TSIM.trail; g.lineWidth = 3; g.beginPath(); pts.slice(0, n + 1).forEach(([x, y], i) => { i ? g.lineTo(x * k, S2 - y * k) : g.moveTo(x * k, S2 - y * k); }); g.stroke(); }
    const p = pts[n]; drawTurtle(g, p[0] * k, S2 - p[1] * k, p[2], 1);
    $$('output', right)[0].textContent = v.toFixed(1); $$('output', right)[1].textContent = w.toFixed(1);
    tc.textContent = `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${v.toFixed(1)}}, angular: {z: ${w.toFixed(1)}}}"`;
  };
  const onIn = () => { touched = true; if (anim) { cancelAnimationFrame(anim); anim = null; } draw(); api.changed(); };
  lx.addEventListener('input', onIn); az.addEventListener('input', onIn);
  $('.pv', right).addEventListener('click', () => { touched = true; api.changed(); let i = 0; if (anim) cancelAnimationFrame(anim); const step = () => { draw(i); i++; if (i <= 60) anim = requestAnimationFrame(step); else anim = null; }; step(); Sound.play('tap'); });
  draw();
  const inR = (v, r) => v >= r[0] - 1e-9 && v <= r[1] + 1e-9;
  const desc = (name, r) => r[0] >= 0.05 && r[1] > 50 ? `${name} > 0` : r[1] <= -0.05 && r[0] < -50 ? `${name} < 0` : (Math.abs(r[0]) <= 0.2 && Math.abs(r[1]) <= 0.2) ? `${name} ≈ 0` : `${name} between ${r[0]} and ${r[1]}`;
  return {
    el, ready: () => touched,
    check() { locked = true; lx.disabled = az.disabled = true; void locked; const ok = inR(+lx.value, ex.lin) && inR(+az.value, ex.ang); return { ok, answer: `<code>${desc('linear.x', ex.lin)}</code> and <code>${desc('angular.z', ex.ang)}</code>` }; },
    solve() { lx.value = ex.lin[0] > 50 ? 1 : clamp((ex.lin[0] + Math.min(ex.lin[1], 2)) / 2, -2, 2).toFixed(1); az.value = clamp((ex.ang[0] + Math.min(ex.ang[1], 2)) / 2, -2, 2).toFixed(1); if (ex.ang[1] > 50) az.value = Math.max(+ex.ang[0], 0.5) + 0.5; if (ex.lin[1] > 50) lx.value = Math.max(+ex.lin[0], 0.5) + 0.5; onIn(); },
  };
};

/* ============================== lesson player ============================== */
const Player = {
  s: null, el: null, ctx: null,
  open(sess) {
    if (sess.battery && !S.settings.unlimited && batteryNow() <= 0) { outOfBattery(() => Player.open(sess)); return; }
    this.close(true);
    const s = this.s = Object.assign({ story: [], queue: [], combo: 0, bestCombo: 0, correct: 0, firstTry: new Set(), missed: new Set(), retried: new Set(), si: 0, started: Date.now(), phase: 'story' }, sess);
    s.total = s.queue.length;
    s.phase = s.story.length ? 'story' : (s.intro ? 'intro' : 'ex');
    const el = this.el = h('div', { class: 'player', role: 'dialog', 'aria-modal': 'true', 'aria-label': s.title });
    el.innerHTML = `<div class="player-top"><button class="icon-btn p-close" aria-label="Quit">${icon('x')}</button><div class="progress" role="progressbar" aria-valuemin="0" aria-valuemax="100" aria-label="Lesson progress"><i></i></div><div class="pill p-bat"></div></div>
      <div class="player-body"><div class="player-inner"></div></div>
      <div class="player-foot"><div class="foot-inner"><div class="fb"></div><div class="p-btns" style="display:flex;gap:12px"></div></div></div>`;
    document.body.append(el); document.body.style.overflow = 'hidden';
    $('.p-close', el).addEventListener('click', () => this.quit());
    this.keyH = (e) => this.onKey(e); document.addEventListener('keydown', this.keyH);
    this.render();
  },
  close(silent) {
    if (this.ctx) { this.ctx.destroy(); this.ctx = null; }
    Voice.stop();
    if (this.el) { this.el.remove(); this.el = null; document.body.style.overflow = ''; document.removeEventListener('keydown', this.keyH); }
    if (!silent && typeof App !== 'undefined') App.refresh();
  },
  quit() {
    const s = this.s;
    if (!s || s.phase === 'result') { this.close(); return; }
    modal({ mood: 'sad', title: 'Leave this lesson?', body: 'Your progress in this lesson will be lost.', actions: [{ label: 'Keep learning', cls: 'btn-primary' }, { label: 'Quit', cls: 'btn-ghost', onClick: () => this.close() }] });
  },
  inner() { return $('.player-inner', this.el); },
  setProgress() {
    const s = this.s, tot = s.story.length + s.total;
    const done = (s.phase === 'story' ? s.si : s.story.length) + s.correct;
    const p = tot ? Math.round(done / tot * 100) : 0;
    const bar = $('.progress', this.el); bar.setAttribute('aria-valuenow', p); $('i', bar).style.width = (s.phase === 'result' ? 100 : p) + '%';
    const bat = $('.p-bat', this.el);
    if (!s.battery) { bat.innerHTML = ''; return; }
    const b = batteryNow();
    bat.className = 'pill p-bat' + (b <= 1 && !S.settings.unlimited ? ' low' : '');
    bat.innerHTML = icon('battery') + `<span class="tabnum">${S.settings.unlimited ? '∞' : b}</span>`;
    bat.setAttribute('aria-label', S.settings.unlimited ? 'Unlimited battery' : `${b} battery cells left`);
  },
  foot(state, html) { const f = $('.player-foot', this.el); f.className = 'player-foot' + (state ? ' ' + state : ''); const fb = $('.fb', f); fb.className = 'fb'; fb.innerHTML = html || ''; const pb = $('.p-btns', f); pb.innerHTML = ''; return pb; },
  render() {
    const s = this.s; if (this.ctx) { this.ctx.destroy(); this.ctx = null; }
    Voice.stop();
    this.setProgress();
    const inner = this.inner(); inner.innerHTML = ''; $('.player-body', this.el).scrollTop = 0;
    if (s.phase === 'story') return this.renderScene();
    if (s.phase === 'intro') return this.renderIntro();
    if (s.phase === 'ex') return this.renderEx();
  },
  renderIntro() {
    const s = this.s, inner = this.inner();
    inner.innerHTML = `<div class="lesson-intro">${nodi('wave', 'wave bob').replace('class="nodi wave', 'style="width:150px" class="nodi wave')}<h2 style="font-size:30px;margin:12px 0 8px">${esc(s.title)}</h2><p style="color:var(--muted);max-width:46ch;margin:0 auto">${s.intro}</p></div>`;
    const btns = this.foot('');
    btns.innerHTML = '';
    btns.append(h('button', { class: 'btn btn-primary', onclick: () => { s.phase = 'ex'; this.render(); } }, 'Start'));
  },
  renderScene() {
    const s = this.s, sc = s.story[s.si], inner = this.inner();
    const head = h('div', { class: 'scene-head', html: nodi(s.si === 0 ? 'happy' : 'idle', s.si === 0 ? 'wave bob' : 'bob') + `<div style="min-width:0"><div class="q-kicker" style="margin:0 0 6px">${icon('book')} Learn · ${s.si + 1} of ${s.story.length}</div><h2>${md(sc.h)}</h2></div>` });
    const stage = h('div', { class: 'stage' }), host = h('div', { class: 'stage-host' });
    const ctrl = h('div', { class: 'stage-ctrl' });
    const bReplay = h('button', { class: 'icon-btn', 'aria-label': 'Replay animation', title: 'Replay', html: icon('replay') });
    const bPause = h('button', { class: 'icon-btn', 'aria-label': 'Pause animation', title: 'Pause', html: icon('pause') });
    const bVoice = h('button', { class: 'icon-btn', 'aria-label': 'Read aloud', title: 'Read aloud', html: icon('sound') });
    ctrl.append(bReplay, bPause); if (Voice.ok) ctrl.append(bVoice);
    stage.append(host, ctrl);
    const cap = h('p', { class: 'caption', html: md(sc.t) });
    const dots = h('div', { class: 'dots', 'aria-hidden': 'true', html: s.story.map((_, i) => `<i class="${i === s.si ? 'on' : ''}"></i>`).join('') });
    inner.append(head, stage, cap, dots);
    const mount = () => { if (this.ctx) this.ctx.destroy(); this.ctx = mountAnim(host, sc); paused = false; bPause.innerHTML = icon('pause'); stage.classList.remove('paused'); };
    let paused = false;
    bReplay.addEventListener('click', mount);
    bPause.addEventListener('click', () => { paused = !paused; setPaused(stage, this.ctx, paused); bPause.innerHTML = icon(paused ? 'play' : 'pause'); bPause.setAttribute('aria-label', paused ? 'Play animation' : 'Pause animation'); });
    bVoice.addEventListener('click', () => Voice.say(sc.h + '. ' + sc.t));
    mount();
    if (S.settings.voice) Voice.say(sc.h + '. ' + sc.t);
    const btns = this.foot('', '');
    const back = h('button', { class: 'btn btn-ghost', onclick: () => this.prevScene(), html: icon('arrowL') + 'Back' }); if (s.si === 0) back.disabled = true;
    const last = s.si === s.story.length - 1;
    const next = h('button', { class: 'btn btn-primary', onclick: () => this.nextScene(), html: (last ? 'Start practice' : 'Next') + icon('arrowR') });
    $('.fb', this.el).append(back); $('.fb', this.el).classList.add('btn-only');
    btns.append(next);
    setTimeout(() => next.focus({ preventScroll: true }), 30);
  },
  nextScene() { const s = this.s; Sound.play('tap'); if (s.si < s.story.length - 1) { s.si++; } else { s.phase = s.queue.length ? 'ex' : 'result'; } if (s.phase === 'result') return this.finish(); this.render(); },
  prevScene() { const s = this.s; if (s.si > 0) { s.si--; this.render(); } },
  renderEx() {
    const s = this.s, item = s.cur = s.queue[0], ex = item.ex, inner = this.inner();
    const [kt, ki] = KICK[ex.k] || ['Question', 'target'];
    inner.append(h('div', { class: 'q-kicker', html: icon(ki) + esc(kt) + (s.retried.has(item.key) ? ' · <span style="color:var(--bad)">try again</span>' : '') }));
    const api = { changed: () => this.updateCheck(), autoCheck: () => { if (s.comp && s.comp.ready() && !s.checked) this.check(); } };
    s.comp = EX[ex.k](ex, api); s.checked = false;
    inner.append(s.comp.el);
    const btns = this.foot('', '');
    const skip = (ex.k === 'type' || ex.k === 'turtle') ? h('button', { class: 'btn btn-ghost', onclick: () => this.check(true) }, "I don't know") : null;
    if (skip) { $('.fb', this.el).append(skip); $('.fb', this.el).classList.add('btn-only'); }
    this.checkBtn = h('button', { class: 'btn btn-primary', onclick: () => this.check() }, 'Check');
    btns.append(this.checkBtn);
    this.updateCheck();
  },
  updateCheck() { if (this.checkBtn && this.s && !this.s.checked) this.checkBtn.disabled = !this.s.comp.ready(); },
  check(giveUp) {
    const s = this.s; if (s.checked) return;
    if (!giveUp && !s.comp.ready()) return;
    s.checked = true;
    const item = s.cur, ex = item.ex;
    const r = giveUp ? Object.assign(s.comp.check ? safeCheck(s.comp) : {}, { ok: false }) : s.comp.check();
    S.stats.answered++;
    const ok = r.ok;
    if (ok) {
      S.stats.correct++; s.correct++; s.combo++; s.bestCombo = Math.max(s.bestCombo, s.combo); S.stats.bestCombo = Math.max(S.stats.bestCombo, s.combo);
      if (!s.missed.has(item.key) && r.perfect !== false) s.firstTry.add(item.key);
      if (r.perfect === false) s.missed.add(item.key);
      s.queue.shift();
      if (s.kind === 'practice') S.mistakes = S.mistakes.filter(k => k !== item.key);
      Sound.play('good');
      if ([3, 5, 10, 15, 20].includes(s.combo)) setTimeout(() => { toast(`<b>${s.combo} in a row!</b><br>${s.combo >= 10 ? 'You are on fire. Keep going!' : 'Nice streak, keep it up.'}`, { mood: 'happy', ms: 1800, cls: 'combo' }); Sound.play('combo'); }, 350);
    } else {
      s.combo = 0; s.missed.add(item.key); s.retried.add(item.key);
      s.queue.push(s.queue.shift());
      if (item.key && !S.mistakes.includes(item.key)) { S.mistakes.push(item.key); if (S.mistakes.length > 60) S.mistakes.shift(); }
      if (s.battery && !S.settings.unlimited) { batteryNow(); S.battery = Math.max(0, S.battery - 1); if (S.battery < BATTERY_MAX && S.battery === BATTERY_MAX - 1) S.batteryTs = Date.now(); }
      Sound.play('bad');
    }
    save();
    this.setProgress();
    const titles = ok ? ['Nice!', 'Great job!', 'Correct!', 'Nailed it!', 'Awesome!', 'Exactly!'] : ['Not quite', 'Almost!', 'Oops, not this time'];
    const title = (ok && r.perfect === false) ? 'All matched!' : titles[(Math.random() * titles.length) | 0];
    let detail = '';
    if (!ok && r.answer) detail += `<div class="fb-detail"><b>Correct answer:</b> ${r.answer}</div>`;
    if (ex.e && !(ok && r.noExplain)) detail += `<div class="fb-detail">${md(ex.e)}</div>`;
    const askBtn = (!ok && Ask.available) ? `<button class="fb-ask">Ask Nodi why</button>` : '';
    const btns = this.foot(ok ? 'ok' : 'no', `<div class="fb-icon">${icon(ok ? 'check' : 'x')}</div><div class="fb-text" role="status"><div class="fb-title">${esc(title)}</div>${detail}${askBtn}</div>`);
    const ab = $('.fb-ask', this.el);
    if (ab) ab.addEventListener('click', () => Ask.openDrawer(`I'm learning ROS 2 and got this question wrong: "${plain(ex.q || '')}". The correct answer is: ${plain(String(r.answer || '').replace(/<[^>]+>/g, ''))}. Can you explain why, briefly?`));
    btns.innerHTML = '';
    const cont = h('button', { class: 'btn ' + (ok ? 'btn-good' : 'btn-bad'), onclick: () => this.advance() }, 'Continue');
    btns.append(cont); this.checkBtn = null;
    setTimeout(() => cont.focus({ preventScroll: true }), 30);
  },
  advance() {
    const s = this.s;
    if (s.battery && !S.settings.unlimited && batteryNow() <= 0 && s.queue.length) { outOfBattery(() => this.render(), () => this.close()); return; }
    if (!s.queue.length) return this.finish();
    this.render();
  },
  finish() {
    const s = this.s; s.phase = 'result';
    if (this.ctx) { this.ctx.destroy(); this.ctx = null; }
    const total = Math.max(1, s.total), acc = s.total ? s.firstTry.size / total : 1;
    const secs = Math.round((Date.now() - s.started) / 1000);
    let xp = 0, gears = 0, note = '';
    const stars = acc >= 0.95 ? 3 : acc >= 0.75 ? 2 : 1;
    if (s.kind === 'lesson') {
      const prev = S.lessons[s.lesson.id];
      xp = (prev ? 5 : 10) + (acc === 1 ? 5 : 0) + (s.bestCombo >= 5 ? 2 : 0); gears = 3 + (acc === 1 ? 2 : 0);
      S.lessons[s.lesson.id] = { stars: Math.max(stars, prev ? prev.stars : 0), best: Math.max(acc, prev ? prev.best : 0), plays: (prev ? prev.plays : 0) + 1, at: Date.now() };
    } else if (s.kind === 'cp') {
      const prev = S.cps[s.unit.id]; xp = prev ? 10 : 20; gears = prev ? 3 : 10;
      S.cps[s.unit.id] = { best: Math.max(acc, prev ? prev.best : 0), at: Date.now() };
    } else if (s.kind === 'final') {
      xp = S.final ? 15 : 50; gears = S.final ? 5 : 25; S.final = { best: Math.max(acc, S.final ? S.final.best : 0), at: Date.now() };
    } else {
      xp = 5 + Math.min(10, s.correct); gears = 2; S.stats.practice++;
      if (!S.settings.unlimited && batteryNow() < BATTERY_MAX) { S.battery = Math.min(BATTERY_MAX, S.battery + 1); note = 'Practice recharged one battery cell.'; }
    }
    if (acc === 1 && s.kind !== 'practice') S.stats.perfect++;
    const newDay = bumpStreak();
    addXP(xp); S.gears += gears;
    save(); checkBadges(); save();
    Sound.play('done'); confetti();
    $$('.toast.combo').forEach(t => t.remove());
    const inner = this.inner(); inner.innerHTML = '';
    const ttl = s.kind === 'final' ? 'You graduated!' : s.kind === 'cp' ? 'Checkpoint cleared!' : s.kind === 'practice' ? 'Practice complete!' : 'Lesson complete!';
    const msg = acc === 1 ? 'Flawless: every answer right on the first try.' : acc >= 0.8 ? 'Great work. Your mistakes are saved in Practice for later.' : 'Done! Mistakes are part of learning; review them anytime in Practice.';
    const mm = Math.floor(secs / 60), ss = String(secs % 60).padStart(2, '0');
    inner.append(h('div', { class: 'result-wrap', html: `${nodi('happy', 'cheer')}<h1 class="result-title">${ttl}</h1><p style="color:var(--ink-2)">${esc(msg)}</p>
      <div class="result-tiles"><div class="rtile" style="--t:var(--c-msg)"><div class="h">Total XP</div><div class="v">${icon('bolt')}${xp}</div></div><div class="rtile" style="--t:var(--good)"><div class="h">Accuracy</div><div class="v">${icon('target')}${Math.round(acc * 100)}%</div></div><div class="rtile" style="--t:var(--c-srv)"><div class="h">Time</div><div class="v">${icon('clock')}${mm}:${ss}</div></div></div>
      <p class="result-note">${icon('gear')} +${gears} gears${newDay ? ` · ${icon('bolt')} ${S.streak}-day streak!` : ''}${note ? ' · ' + esc(note) : ''}</p>` }));
    const lab = s.unit && s.unit.lab && (s.kind === 'cp' || (s.lesson && s.unit.lessons[s.unit.lessons.length - 1] === s.lesson));
    const btns = this.foot('', '');
    if (lab) $('.fb', this.el).classList.add('btn-only');
    if (lab) $('.fb', this.el).append(h('button', { class: 'btn', onclick: () => { this.close(); App.go('lab'); Lab.focusTab(s.unit.lab.tab); }, html: icon('lab') + 'Try it in the Robot Lab' }));
    const cont = h('button', { class: 'btn btn-primary', onclick: () => { this.close(); App.afterSession(s); } }, 'Continue');
    btns.append(cont); this.setProgress();
    setTimeout(() => cont.focus({ preventScroll: true }), 30);
  },
  onKey(e) {
    if (!this.el || $('.modal-back') || $('.drawer')) return;
    const s = this.s, tag = (e.target.tagName || '').toLowerCase();
    if (e.key === 'Escape') { this.quit(); return; }
    if (s.phase === 'story') { if (e.key === 'ArrowRight') { e.preventDefault(); this.nextScene(); } else if (e.key === 'ArrowLeft') { e.preventDefault(); this.prevScene(); } return; }
    if (s.phase === 'ex') {
      if (e.key === 'Enter' && !e.repeat) {
        if (tag === 'button' && !e.target.classList.contains('btn')) return;
        e.preventDefault();
        if (s.checked) this.advance(); else this.check();
        return;
      }
      if (!s.checked && /^[1-9]$/.test(e.key) && tag !== 'input' && s.comp.key) s.comp.key(e.key);
    }
  },
};
function safeCheck(comp) { try { return comp.check(); } catch (e) { return {}; } }
function outOfBattery(onRecharge, onQuit) {
  const canBuy = S.gears >= 30;
  const acts = [];
  if (canBuy) acts.push({ label: 'Recharge for 30 gears', cls: 'btn-primary', onClick: () => { S.gears -= 30; S.battery = BATTERY_MAX; S.batteryTs = Date.now(); save(); Sound.play('coin'); onRecharge && onRecharge(); } });
  acts.push({ label: 'Practice to recharge', cls: canBuy ? '' : 'btn-primary', onClick: () => { Player.close(); App.go('practice'); } });
  acts.push({ label: onQuit ? 'Quit lesson' : 'Not now', cls: 'btn-ghost', onClick: () => { onQuit ? onQuit() : null; } });
  const mins = Math.max(1, Math.ceil((REGEN_MS - (Date.now() - S.batteryTs)) / 60000));
  modal({ mood: 'sad', title: 'Battery empty', body: `Each mistake drains one cell. The next cell recharges in about ${mins} min. You can also turn on unlimited battery in Profile.`, actions: acts, dismiss: false });
}
