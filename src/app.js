/* ============================== app shell & views ============================== */
const VIEWS = ['learn', 'lab', 'snobots', 'snobots-ainex', 'snobots-r1', 'snobots-limo', 'snobots-cobot', 'practice', 'notebook', 'ask', 'profile'];
const NAV = [['learn', 'learn', 'Learn'], ['lab', 'lab', 'Robot Lab'], ['snobots', 'snobots', 'SnoBots Robots'], ['practice', 'practice', 'Practice'], ['notebook', 'notebook', 'Notebook'], ['ask', 'ask', 'Ask Nodi'], ['profile', 'profile', 'Profile']];
const OFFS = [0, 46, 70, 46, 0, -46, -70, -46];

const App = {
  view: 'learn',
  init() {
    const root = $('#app');
    root.innerHTML = `<div class="app">
      <nav class="nav" aria-label="Main">
        <div class="brand">${nodi('happy', '')}<div class="brand-txt"><div class="brand-name">Nodi</div><div class="brand-tag">Learn ROS 2</div></div></div>
        ${NAV.map(([v, ic, label]) => `<button class="nav-item${v === 'ask' ? ' ask-nav' : ''}" data-v="${v}" ${v === 'ask' ? 'hidden' : ''}>${icon(ic)}<span>${label}</span></button>`).join('')}
        <div class="nav-spacer"></div>
        <div class="nav-foot">ROS 2 Jazzy · progress saves automatically</div>
      </nav>
      <main class="main"><div class="mobilebar"><div class="brand" style="padding:0">${nodi('happy', '').replace('class="nodi', 'style="width:34px;height:38px" class="nodi')}</div><div class="topstats" data-stats></div></div><div id="view"></div></main>
      <aside class="aside" aria-label="Your progress"></aside></div>
      <button class="ask-fab" hidden aria-label="Ask Nodi">${nodi('idle', '')}</button>`;
    $$('.nav-item', root).forEach(b => b.addEventListener('click', () => this.go(b.dataset.v)));
    $('.ask-fab', root).addEventListener('click', () => Ask.openDrawer());
    window.addEventListener('hashchange', () => { const v = location.hash.slice(1); if (VIEWS.includes(v) && v !== this.view) this.go(v, true); });
    const start = location.hash.slice(1);
    this.go(VIEWS.includes(start) ? start : 'learn', true);
    checkBadges(true); save({ local: true });
    setInterval(() => this.renderStats(), 30000);
    document.addEventListener('click', closePops);
    Cloud.init(); Ask.init();
  },
  go(v, fromHash) {
    if (v === 'ask' && !Ask.available) v = 'learn';
    if (this.view === 'lab' && v !== 'lab') Lab.unmount();
    if (this.view.startsWith('snobots-') && v !== this.view) SnoUI.unmount();
    this.view = v;
    const appEl = $('.app'); if (appEl) appEl.classList.toggle('wide', v === 'lab' || v.startsWith('snobots-'));
    if (!fromHash && location.hash.slice(1) !== v) { try { history.replaceState(null, '', '#' + v); } catch (e) { location.hash = v; } }
    $$('.nav-item').forEach(b => { if (b.dataset.v === v || (b.dataset.v === 'snobots' && v.startsWith('snobots'))) b.setAttribute('aria-current', 'page'); else b.removeAttribute('aria-current'); });
    this.refresh();
    window.scrollTo(0, 0);
    if (v === 'learn') setTimeout(() => this.scrollToCurrent(false), 60);
  },
  refresh() {
    const el = $('#view'); if (!el) return;
    const v = this.view;
    if (v !== 'lab' && !v.startsWith('snobots-')) { el.innerHTML = ''; }
    if (v === 'learn') renderLearn(el);
    else if (v === 'lab') Lab.mount(el);
    else if (v === 'snobots') SnoUI.mountHub(el);
    else if (v.startsWith('snobots-')) { if (!SnoUI.cur || SnoUI.cur.id !== v.slice(8) || SnoUI.root !== el) SnoUI.mount(el, v.slice(8)); }
    else if (v === 'practice') renderPractice(el);
    else if (v === 'notebook') renderNotebook(el);
    else if (v === 'ask') renderAsk(el);
    else if (v === 'profile') renderProfile(el);
    this.renderStats(); this.renderAside();
  },
  renderStats() {
    const b = batteryNow(), st = streakNow();
    const html = `<button class="stat streak${S.lastDay === dayKey(0) ? ' on' : ''}" data-k="streak" aria-label="${st} day streak">${icon('bolt')}<span class="tabnum">${st}</span></button>
      <button class="stat gears" data-k="gears" aria-label="${S.gears} gears">${icon('gear')}<span class="tabnum">${S.gears}</span></button>
      <button class="stat battery${b <= 1 && !S.settings.unlimited ? ' low' : ''}" data-k="battery" aria-label="Battery ${S.settings.unlimited ? 'unlimited' : b + ' of 5'}">${icon('battery')}<span class="tabnum">${S.settings.unlimited ? '∞' : b}</span></button>`;
    $$('[data-stats]').forEach(n => { n.innerHTML = html; $$('.stat', n).forEach(x => x.addEventListener('click', () => statInfo(x.dataset.k))); });
  },
  renderAside() {
    const a = $('.aside'); if (!a || getComputedStyle(a).display === 'none') return;
    const info = pathInfo(), it = info.items[info.cur];
    const tx = todayXP(), pct = Math.min(1, tx / S.goal), C = 2 * Math.PI * 30;
    const earned = BADGES.filter(b => S.badges[b.id]);
    a.innerHTML = `<div class="topstats" data-stats></div>
      <div class="card"><div class="goal"><svg class="ring" viewBox="0 0 76 76" role="img" aria-label="Daily goal ${tx} of ${S.goal} XP"><circle class="bg" cx="38" cy="38" r="30"/><circle class="fg" cx="38" cy="38" r="30" stroke-dasharray="${C.toFixed(1)}" stroke-dashoffset="${(C * (1 - pct)).toFixed(1)}"/><text x="38" y="38">${Math.round(pct * 100)}%</text></svg>
        <div><h3>Daily goal</h3><p class="sub">${tx >= S.goal ? 'Done for today. Nice!' : `${S.goal - tx} XP to go today`}</p><p class="sub tabnum">${tx} / ${S.goal} XP</p></div></div></div>
      ${it ? `<div class="card" style="--u:${it.unit ? it.unit.color : 'var(--accent)'}"><p class="sub" style="text-transform:uppercase;letter-spacing:.1em;font-size:12px;font-weight:600">Next up</p><div class="next-up" style="margin:10px 0 14px"><div class="dot">${icon(it.type === 'lesson' ? (it.lesson.icon || 'play') : it.type === 'cp' ? 'chip' : 'trophy')}</div><div><b style="font-family:var(--font-display);font-size:17px">${esc(itemTitle(it))}</b><div class="sub">${it.unit ? 'Unit ' + it.unit.id.slice(1) + ' · ' + esc(it.unit.title) : 'All units'}</div></div></div><button class="btn btn-primary btn-wide" data-start>Start</button></div>` : ''}
      <div class="card lab-promo"><h3>Robot Lab</h3><p class="sub">Drive a robot, echo its topics and run your own Python node.</p><canvas width="584" height="240" aria-hidden="true"></canvas><button class="btn btn-primary btn-wide" data-lab>Open the lab</button></div>
      <div class="card"><h3>Badges</h3><p class="sub">${earned.length} of ${BADGES.length} earned</p><div class="mini-badges">${(earned.length ? earned.slice(-6) : BADGES.slice(0, 4)).map(b => `<span class="chip" title="${esc(b.desc)}" style="${earned.includes(b) ? '' : 'opacity:.5'}">${icon(b.icon)}${esc(b.name)}</span>`).join('')}</div></div>`;
    this.renderStats();
    const sb = $('[data-start]', a); if (sb) sb.addEventListener('click', () => startItem(it));
    $('[data-lab]', a).addEventListener('click', () => this.go('lab'));
    promoAnim($('.lab-promo canvas', a));
  },
  scrollToCurrent(smooth = true) {
    const cur = $('.pad.current');
    if (cur) { const r = cur.getBoundingClientRect(); const y = window.scrollY + r.top - innerHeight * 0.42; window.scrollTo({ top: Math.max(0, y), behavior: smooth && !REDUCED ? 'smooth' : 'auto' }); }
  },
  afterSession(s) {
    this.refresh();
    if (this.view === 'learn') setTimeout(() => this.scrollToCurrent(true), 120);
    if (s.unit && s.kind === 'cp' && unitDone(s.unit.id)) {
      const nu = UNITS[UNITS.indexOf(s.unit) + 1];
      setTimeout(() => toast(`<b>Unit ${s.unit.id.slice(1)} complete!</b><br>${nu ? 'Unlocked: ' + esc(nu.title) : 'Final exam unlocked!'}`, { mood: 'happy', ms: 3400 }), 400);
    }
  },
  showAsk() { $$('.ask-nav').forEach(b => { b.hidden = false; }); const f = $('.ask-fab'); if (f) f.hidden = false; },
  hideAsk() { $$('.ask-nav').forEach(b => { b.hidden = true; }); const f = $('.ask-fab'); if (f) f.hidden = true; if (this.view === 'ask') this.go('learn'); },
};
function itemTitle(it) { return it.type === 'lesson' ? it.lesson.title : it.type === 'cp' ? `Unit ${it.unit.id.slice(1)} checkpoint` : 'Final exam'; }
function statInfo(k) {
  if (k === 'streak') { const st = streakNow(); modal({ mood: st ? 'happy' : 'idle', title: `${st}-day streak`, body: S.lastDay === dayKey(0) ? 'You already learned today. Come back tomorrow to keep the streak going.' : 'Finish any lesson or practice today to extend your streak.' + (S.bestStreak ? ` Your best is ${S.bestStreak} days.` : ''), actions: [{ label: 'Got it', cls: 'btn-primary' }] }); }
  else if (k === 'gears') modal({ mood: 'wow', title: `${S.gears} gears`, body: 'You earn gears for lessons, checkpoints and Robot Lab missions. Spend 30 gears to fully recharge your battery.', actions: [{ label: 'Got it', cls: 'btn-primary' }] });
  else {
    const b = batteryNow();
    const acts = [{ label: 'Got it', cls: 'btn-primary' }];
    if (!S.settings.unlimited && b < BATTERY_MAX && S.gears >= 30) acts.unshift({ label: 'Recharge for 30 gears', cls: 'btn-blue', onClick: () => { S.gears -= 30; S.battery = BATTERY_MAX; save(); Sound.play('coin'); App.renderStats(); App.renderAside(); } });
    modal({ mood: b ? 'idle' : 'sad', title: S.settings.unlimited ? 'Unlimited battery' : `Battery ${b} / ${BATTERY_MAX}`, body: S.settings.unlimited ? 'Mistakes never drain your battery. You can change this in Profile.' : 'Each mistake in a lesson drains one cell. One cell recharges every 10 minutes, and finishing a practice session recharges one.', actions: acts });
  }
}
function promoAnim(cv) {
  if (!cv) return;
  const g = cv.getContext('2d'); let t = 0;
  if (App._promo) cancelAnimationFrame(App._promo);
  const step = () => {
    if (!cv.isConnected) return;
    t += 0.016; const W = cv.width, H = cv.height;
    g.fillStyle = '#13211C'; g.fillRect(0, 0, W, H);
    g.strokeStyle = 'rgba(255,255,255,.05)'; g.lineWidth = 1; for (let x = 0; x < W; x += 40) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke(); } for (let y = 0; y < H; y += 40) { g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke(); }
    g.fillStyle = '#3A4F46'; g.fillRect(380, 40, 60, 90); g.beginPath(); g.arc(150, 170, 26, 0, 7); g.fill();
    const x = W / 2 + 190 * Math.sin(t * 0.6), y = H / 2 + 70 * Math.sin(t * 1.2), th = Math.atan2(84 * Math.cos(t * 1.2), 114 * Math.cos(t * 0.6));
    g.strokeStyle = 'rgba(255,107,94,.28)'; for (let i = 0; i < 36; i++) { const a = th + i / 36 * 6.283; g.beginPath(); g.moveTo(x, y); g.lineTo(x + 70 * Math.cos(a), y + 70 * Math.sin(a)); g.stroke(); }
    g.save(); g.translate(x, y); g.rotate(th); g.fillStyle = '#F4F8F6'; g.beginPath(); g.arc(0, 0, 16, 0, 7); g.fill(); g.fillStyle = '#F26419'; g.beginPath(); g.moveTo(9, -6); g.lineTo(18, 0); g.lineTo(9, 6); g.fill(); g.restore();
    App._promo = requestAnimationFrame(step);
  };
  if (REDUCED) { t = 2; step(); cancelAnimationFrame(App._promo); } else step();
}

/* ---------- learn: the path ---------- */
function renderLearn(root) {
  const info = pathInfo();
  const v = h('div', { class: 'view' });
  const total = info.items.length, done = info.items.filter(isDone).length;
  v.append(h('div', { class: 'view-head', html: `<div><h1>Your ROS 2 path</h1><p>12 units, from your first node to driving real motors. ${done} of ${total} levels done.</p></div>` }));
  let gi = 0;
  UNITS.forEach((u, ui) => {
    const items = info.items.filter(it => it.unit === u);
    const firstIdx = info.items.indexOf(items[0]);
    const locked = itemState(firstIdx, info) === 'locked';
    const sec = h('section', { class: 'unit' + (locked ? ' locked' : ''), style: `--u:${u.color}`, 'aria-label': `Unit ${ui + 1}: ${u.title}` });
    const chip = h('header', { class: 'unit-chip', html: `<span class="notch"></span><div><div class="unit-kicker">Unit ${ui + 1}</div><h2>${esc(u.title)}</h2><p>${esc(u.sub)}</p></div>` });
    const nb = h('button', { class: 'btn btn-white btn-sm', html: icon('notebook') + 'Notes' }); nb.addEventListener('click', () => { App.go('notebook'); setTimeout(() => setNoteFilter(u.id), 0); });
    chip.append(nb); sec.append(chip);
    const trail = h('div', { class: 'trail' });
    const rows = items.length, H = rows * 104;
    let svg = `<svg class="trace" viewBox="0 0 420 ${H}" style="width:420px;left:50%;margin-left:-210px;height:${H}px" aria-hidden="true">`;
    const pts = items.map((it, i) => [210 + OFFS[(gi + i) % 8], i * 104 + 45]);
    for (let i = 0; i < pts.length - 1; i++) {
      const [x1, y1] = pts[i], [x2, y2] = pts[i + 1], lit = isDone(items[i]);
      svg += `<path class="${lit ? 'lit' : ''}" d="M${x1} ${y1} C ${x1} ${y1 + 52}, ${x2} ${y2 - 52}, ${x2} ${y2}"/>`;
    }
    svg += '</svg>';
    trail.innerHTML = svg;
    items.forEach((it, i) => {
      const idx = info.items.indexOf(it), st = itemState(idx, info), off = OFFS[(gi + i) % 8];
      const row = h('div', { class: 'padrow', style: `--x:${off}px` });
      const ic = st === 'locked' ? (it.type === 'cp' ? 'chip' : 'lock') : st === 'done' ? (it.type === 'cp' ? 'trophy' : 'check') : it.type === 'cp' ? 'chip' : (it.lesson.icon || 'play');
      const pad = h('button', { class: `pad ${st}${it.type === 'cp' ? ' cp' : ''}`, 'aria-label': `${itemTitle(it)}: ${st === 'done' ? 'completed' : st === 'locked' ? 'locked' : 'start'}`, html: icon(ic) });
      row.append(pad);
      if (st === 'current') row.append(h('div', { class: 'bubble-start', style: `transform:translateX(calc(-50% + ${off}px));--c:${u.color}`, text: 'START' }));
      if (st === 'done' && it.type === 'lesson') { const n = S.lessons[it.id].stars || 1; row.append(h('div', { class: 'stars', 'aria-label': `${n} of 3 stars`, html: [1, 2, 3].map(k => `<span class="${k <= n ? 'on' : ''}">${icon('star')}</span>`).join('') })); }
      if (st === 'current') {
        const left = off >= 0;
        const m = h('div', { class: 'path-mascot', style: `top:28px;${left ? `left:calc(50% + ${off - 190}px)` : `left:calc(50% + ${off + 80}px)`}`, html: `<div class="say">${esc(mascotLine(it))}</div>${nodi('wave', 'wave bob')}` });
        row.append(m);
      }
      pad.addEventListener('click', (e) => { e.stopPropagation(); togglePop(row, it, st, u); });
      trail.append(row);
    });
    gi += items.length;
    sec.append(trail);
    v.append(sec);
  });
  const fi = info.items.length - 1, fst = itemState(fi, info);
  const fin = h('section', { class: 'card final-card', style: `--u:var(--accent);margin-top:20px` });
  fin.innerHTML = `<div class="padrow" style="height:auto"><button class="pad cp ${fst}" style="--u:var(--accent)" aria-label="Final exam">${icon(fst === 'locked' ? 'lock' : 'trophy')}</button></div><h2 style="margin-top:12px">Final exam</h2><p class="sub" style="color:var(--muted);margin:6px 0 14px">15 questions from the whole course. Pass it to graduate.</p>`;
  const fb = h('button', { class: 'btn btn-primary', text: fst === 'done' ? 'Retake +15 XP' : fst === 'locked' ? 'Finish all units first' : 'Start +50 XP' });
  if (fst === 'locked') fb.disabled = true; else fb.addEventListener('click', () => startItem(info.items[fi]));
  fin.append(fb); v.append(fin);
  root.append(v);
  const jump = h('button', { class: 'jump', 'aria-label': 'Jump to your current lesson', html: icon('arrowD') });
  jump.addEventListener('click', () => App.scrollToCurrent(true));
  root.append(jump);
}
function mascotLine(it) {
  if (it.type === 'cp') return 'Checkpoint time! Show what you learned.';
  const u = it.unit, i = u.lessons.indexOf(it.lesson);
  if (lessonsDoneCount() === 0) return 'Hi, I\'m Nodi! Tap START to begin.';
  return i === 0 ? `New unit: ${u.title}!` : 'Keep going, you\'re doing great!';
}
function closePops() { $$('.pop').forEach(p => p.remove()); }
function togglePop(row, it, st, u) {
  const had = $('.pop', row); closePops(); if (had) return;
  const isL = it.type === 'lesson';
  const pos = isL ? `Lesson ${u.lessons.indexOf(it.lesson) + 1} of ${u.lessons.length} · ${it.lesson.story.length} scenes · ${it.lesson.ex.length} exercises` : `${Math.min(8, u.lessons.reduce((n, l) => n + l.ex.length, 0))} questions from Unit ${u.id.slice(1)}`;
  const pop = h('div', { class: 'pop' + (st === 'locked' ? ' locked' : ''), style: `--c:${u.color}`, role: 'dialog', 'aria-label': itemTitle(it) });
  pop.innerHTML = `<h3>${esc(itemTitle(it))}</h3><p>${st === 'locked' ? 'Finish the levels above to unlock this one.' : pos}</p>`;
  const b = h('button', { class: 'btn btn-white btn-wide' });
  if (st === 'locked') { b.textContent = 'Locked'; b.disabled = true; }
  else { const prev = isL ? S.lessons[it.id] : S.cps[it.unit.id]; b.textContent = prev ? `Review +${isL ? 5 : 10} XP` : `Start +${isL ? 10 : 20} XP`; b.addEventListener('click', (e) => { e.stopPropagation(); closePops(); startItem(it); }); }
  pop.append(b); pop.addEventListener('click', e => e.stopPropagation());
  row.append(pop);
  setTimeout(() => b.focus({ preventScroll: true }), 20);
}
function startItem(it) {
  if (!it) return;
  if (it.type === 'lesson') {
    Player.open({ kind: 'lesson', title: it.lesson.title, unit: it.unit, lesson: it.lesson, story: it.lesson.story, queue: it.lesson.ex.map((ex, i) => ({ key: `${it.lesson.id}#${i}`, ex })), battery: true });
  } else if (it.type === 'cp') {
    const pool = it.unit.lessons.flatMap(l => l.ex.map((ex, i) => ({ key: `${l.id}#${i}`, ex })));
    Player.open({ kind: 'cp', title: `Unit ${it.unit.id.slice(1)} checkpoint`, unit: it.unit, queue: sample(pool, Math.min(8, pool.length)), battery: true, intro: `Eight questions from <b>${esc(it.unit.title)}</b>. Clear it to unlock the next unit. Mistakes cost battery, and every wrong answer comes back until you get it right.` });
  } else {
    const pool = UNITS.flatMap(u => sample(u.lessons.flatMap(l => l.ex.map((ex, i) => ({ key: `${l.id}#${i}`, ex })).filter(x => x.ex.k !== 'match')), 2));
    Player.open({ kind: 'final', title: 'Final exam', queue: sample(pool, 15), battery: true, intro: 'Fifteen questions from all twelve units. Take your time: you can do this.' });
  }
}

/* ---------- practice ---------- */
function doneLessonExercises() { return allLessons().filter(x => S.lessons[x.lesson.id]).flatMap(x => x.lesson.ex.map((ex, i) => ({ key: `${x.lesson.id}#${i}`, ex }))); }
function renderPractice(root) {
  const v = h('div', { class: 'view view-wide' });
  v.append(h('div', { class: 'view-head', html: '<div><h1>Practice</h1><p>Strengthen what you know. Practice never costs battery, and finishing a session recharges one cell.</p></div>' }));
  const mistakes = S.mistakes.map(exByKey).filter(Boolean);
  const pool = doneLessonExercises();
  if (!pool.length && !mistakes.length) {
    v.append(h('div', { class: 'card empty', html: `${nodi('think', 'bob')}<h2>Nothing to practice yet</h2><p style="margin:8px 0 16px">Finish your first lesson and your review sessions will appear here.</p>` }, h('button', { class: 'btn btn-primary', onclick: () => App.go('learn') }, 'Go to lessons')));
    root.append(v); return;
  }
  const cmds = pool.filter(x => x.ex.k === 'type' || (x.ex.k === 'build' && /^(ros2|sudo|colcon|source)/.test(x.ex.a[0] || '')));
  const grid = h('div', { class: 'grid-cards' });
  const card = (k, ks, ic, title, text, btn, fn, dis) => {
    const c = h('div', { class: 'card pcard', style: `--k:${k};--k-soft:${ks}`, html: `<div class="ic">${icon(ic)}</div><h3>${title}</h3><p class="sub grow">${text}</p>` });
    const b = h('button', { class: 'btn btn-primary btn-wide' }, btn); if (dis) b.disabled = true; else b.addEventListener('click', fn);
    c.append(b); grid.append(c);
  };
  card('var(--bad)', 'var(--bad-soft)', 'replay', 'Fix your mistakes', mistakes.length ? `${mistakes.length} question${mistakes.length > 1 ? 's' : ''} you missed earlier. Get them right to clear them.` : 'No mistakes waiting. Impressive!', 'Start', () => Player.open({ kind: 'practice', title: 'Fix your mistakes', queue: mistakes.slice(-10), battery: false }), !mistakes.length);
  card('var(--c-srv)', 'var(--c-srv-soft)', 'sparkle', 'Quick review', 'Eight random questions from lessons you have finished.', 'Start', () => Player.open({ kind: 'practice', title: 'Quick review', queue: sample(pool, Math.min(8, pool.length)), battery: false }), !pool.length);
  card('var(--c-node)', 'var(--c-node-soft)', 'terminal', 'Command drill', cmds.length ? `Type and build the ros2 commands you have learned (${cmds.length} available).` : 'Unlocks after lessons with terminal commands.', 'Start', () => Player.open({ kind: 'practice', title: 'Command drill', queue: sample(cmds, Math.min(8, cmds.length)), battery: false }), !cmds.length);
  card('var(--c-topic)', 'var(--c-topic-soft)', 'cards', 'Flashcards', 'Flip through the key ideas from your notebook.', 'Open', () => openFlashcards());
  { const sd = SNO_IDS.reduce((n, id) => n + Object.keys((S.sno.m || {})[id] || {}).length, 0), st = SNO_IDS.reduce((n, id) => n + SNO_DEFS[id].missions.length, 0); card('#E5532B', 'color-mix(in srgb, #E5532B 14%, var(--surface))', 'snobots', 'SnoBots Robots', `${sd} of ${st} missions on the team's robots: AiNex, R1, LIMO and LIMO Cobot.`, 'Open', () => App.go('snobots')); }
  card('var(--accent)', 'var(--accent-soft)', 'lab', 'Robot Lab missions', `${Object.keys(S.lab.missions).length} of ${MISSIONS.length} missions complete. Practice on a live robot.`, 'Open lab', () => App.go('lab'));
  v.append(grid); root.append(v);
}
function openFlashcards() {
  const cards = UNITS.filter(unitStarted).flatMap(u => u.notes.map(n => ({ u, n })));
  if (!cards.length) { toast('Finish a lesson to unlock flashcards.', { mood: 'think' }); return; }
  const deck = shuffle(cards); let i = 0, flipped = false;
  const back = h('div', { class: 'modal-back', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Flashcards' });
  const box = h('div', { class: 'modal', style: 'width:min(620px,100%);text-align:left' });
  const draw = () => {
    const { u, n } = deck[i];
    box.innerHTML = `<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:14px"><span class="chip"><span class="unit-dot" style="--u:${u.color};display:inline-block;width:10px;height:10px;border-radius:50%;background:${u.color}"></span>Unit ${u.id.slice(1)}</span><span class="sub tabnum" style="color:var(--muted)">${i + 1} / ${deck.length}</span><button class="icon-btn fx" aria-label="Close">${icon('x')}</button></div>
      <div class="flash"><div class="flash-card${flipped ? ' flipped' : ''}" tabindex="0" role="button" aria-label="Flip card"><div class="flash-face"><div><h2>${esc(n[0])}</h2><p class="sub" style="color:var(--muted);margin-top:10px">Tap to flip</p></div></div><div class="flash-face back"><div><p>${esc(n[1])}</p>${n[2] ? `<pre style="text-align:left;background:var(--term-bg);color:var(--term-ink);padding:10px 12px;border-radius:10px;font:12.5px/1.5 var(--font-mono);white-space:pre-wrap;margin-top:12px">${esc(n[2])}</pre>` : ''}</div></div></div></div>
      <div style="display:flex;gap:10px;justify-content:space-between;margin-top:18px"><button class="btn fp">${icon('arrowL')}Prev</button><button class="btn btn-primary fn">Next${icon('arrowR')}</button></div>`;
    const fc = $('.flash-card', box); fc.addEventListener('click', () => { flipped = !flipped; fc.classList.toggle('flipped', flipped); });
    fc.addEventListener('keydown', (e) => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); fc.click(); } });
    $('.fp', box).addEventListener('click', () => { i = (i - 1 + deck.length) % deck.length; flipped = false; draw(); });
    $('.fn', box).addEventListener('click', () => { i = (i + 1) % deck.length; flipped = false; draw(); });
    $('.fx', box).addEventListener('click', close);
    fc.focus();
  };
  const onKey = (e) => { if (e.key === 'Escape') close(); if (e.key === 'ArrowRight') $('.fn', box).click(); if (e.key === 'ArrowLeft') $('.fp', box).click(); };
  const close = () => { back.remove(); document.removeEventListener('keydown', onKey); };
  document.addEventListener('keydown', onKey);
  back.addEventListener('click', (e) => { if (e.target === back) close(); });
  back.append(box); document.body.append(back); draw();
}

/* ---------- notebook ---------- */
let NOTE_FILTER = 'all', NOTE_Q = '';
function setNoteFilter(f) { NOTE_FILTER = f; if (App.view === 'notebook') App.refresh(); }
function renderNotebook(root) {
  const v = h('div', { class: 'view view-wide' });
  v.append(h('div', { class: 'view-head', html: '<div><h1>Notebook</h1><p>Your ROS 2 cheat sheet. A unit\'s cards unlock when you finish one of its lessons.</p></div>' }));
  const sb = h('label', { class: 'searchbar', html: icon('search') });
  const inp = h('input', { type: 'search', placeholder: 'Search commands and ideas', value: NOTE_Q, 'aria-label': 'Search the notebook' });
  sb.append(inp); v.append(sb);
  const fr = h('div', { class: 'filter-row', role: 'group', 'aria-label': 'Filter by unit' });
  [['all', 'All units'], ...UNITS.map((u, i) => [u.id, `Unit ${i + 1}`])].forEach(([id, label]) => { const c = h('button', { class: 'chip' + (NOTE_FILTER === id ? ' on' : ''), 'aria-pressed': String(NOTE_FILTER === id) }, label); c.addEventListener('click', () => { NOTE_FILTER = id; draw(); $$('.chip', fr).forEach(x => { const on = x === c; x.classList.toggle('on', on); x.setAttribute('aria-pressed', String(on)); }); }); fr.append(c); });
  v.append(fr);
  const cols = h('div', { class: 'notes-cols' }); v.append(cols);
  const draw = () => {
    cols.innerHTML = ''; const q = NOTE_Q.toLowerCase();
    UNITS.forEach((u, ui) => {
      if (NOTE_FILTER !== 'all' && NOTE_FILTER !== u.id) return;
      if (!unitStarted(u)) { if (!q) cols.append(h('div', { class: 'card note-card locked', style: `--u:${u.color}`, html: `<span class="unit-dot"></span><span class="term-k">Unit ${ui + 1}: ${esc(u.title)}</span><p>${icon('lock')} Finish a lesson in this unit to unlock ${u.notes.length} cards.</p>` })); return; }
      u.notes.forEach(n => {
        if (q && !(n.join(' ').toLowerCase().includes(q))) return;
        const c = h('div', { class: 'card note-card', style: `--u:${u.color}`, html: `<span class="unit-dot" title="Unit ${ui + 1}"></span><span class="term-k">${esc(n[0])}</span><p>${esc(n[1])}</p>` });
        if (n[2]) { const sn = h('div', { class: 'snip', html: `<pre>${esc(n[2])}</pre>` }); const cb = h('button', { class: 'icon-btn', 'aria-label': 'Copy', title: 'Copy', html: icon('copy') }); cb.addEventListener('click', () => copyText(n[2])); sn.append(cb); c.append(sn); }
        cols.append(c);
      });
    });
    if (!cols.children.length) cols.append(h('div', { class: 'empty', html: `${nodi('think', 'bob')}<p>No cards match “${esc(NOTE_Q)}”.</p>` }));
  };
  inp.addEventListener('input', () => { NOTE_Q = inp.value; draw(); });
  draw(); root.append(v);
}
async function copyText(t) {
  try { await navigator.clipboard.writeText(t); toast('<b>Copied</b> to your clipboard.', { ms: 1600 }); Sound.play('tap'); }
  catch (e) { toast('Copying isn\'t allowed here. Select the text and copy it instead.', { mood: 'think' }); }
}

/* ---------- profile ---------- */
function renderProfile(root) {
  const v = h('div', { class: 'view view-wide' });
  const acc = S.stats.answered ? Math.round(S.stats.correct / S.stats.answered * 100) : 0;
  const nLessons = allLessons().length;
  const head = h('div', { class: 'profile-head', html: nodi('happy', 'wave bob') });
  const nm = h('div');
  const ni = h('input', { class: 'name-input', value: S.name, placeholder: 'Your name', 'aria-label': 'Your name', maxlength: '40' });
  ni.addEventListener('change', () => { S.name = ni.value.trim(); save(); });
  nm.append(ni, h('p', { class: 'sub', style: 'color:var(--muted);margin-top:6px', text: `Learning ROS 2 since ${new Date(S.created).toLocaleDateString(undefined, { month: 'long', day: 'numeric', year: 'numeric' })}` }));
  head.append(nm); v.append(head);
  const tiles = [['bolt', 'var(--accent)', S.xp, 'Total XP'], ['clock', '#E5532B', `${streakNow()}`, `Day streak · best ${S.bestStreak}`], ['book', 'var(--c-node)', `${lessonsDoneCount()} / ${nLessons}`, 'Lessons done'], ['target', 'var(--good)', `${acc}%`, 'Accuracy'], ['flag', 'var(--c-srv)', `${Object.keys(S.lab.missions).length} / ${MISSIONS.length}`, 'Lab missions'], ['gear', 'var(--c-msg)', S.gears, 'Gears']];
  v.append(h('div', { class: 'stat-grid', html: tiles.map(([ic, k, val, lbl]) => `<div class="stat-tile"><span style="color:${k}">${icon(ic)}</span><div><b class="tabnum">${val}</b><span>${lbl}</span></div></div>`).join('') }));
  const wk = h('div', { class: 'card', style: 'margin-top:16px' });
  wk.innerHTML = `<h3>XP this week</h3><p class="sub">Daily goal: ${S.goal} XP</p>`;
  wk.append(weekChart()); v.append(wk);
  const bd = h('div', { class: 'card', style: 'margin-top:16px' });
  bd.innerHTML = `<h3>Badges</h3><p class="sub" style="margin-bottom:12px">${Object.keys(S.badges).length} of ${BADGES.length} earned</p><div class="badges">${BADGES.map(b => `<div class="badge${S.badges[b.id] ? '' : ' locked'}" style="--k:${b.k}"><div class="medal">${icon(S.badges[b.id] ? b.icon : 'lock')}</div><h4>${esc(b.name)}</h4><p>${esc(b.desc)}</p></div>`).join('')}</div>`;
  v.append(bd);
  const st = h('div', { class: 'card', style: 'margin-top:16px' });
  st.innerHTML = '<h3>Settings</h3>';
  const set = h('div', { class: 'settings' });
  const tog = (key, title, sub) => {
    const id = 'set-' + key, row = h('div', { class: 'set-row', html: `<label class="lbl" for="${id}"><b>${title}</b><span>${sub}</span></label>` });
    const sw = h('span', { class: 'switch' }); const cb = h('input', { type: 'checkbox', id, role: 'switch' }); cb.checked = !!S.settings[key];
    cb.addEventListener('change', () => { S.settings[key] = cb.checked; save(); if (key === 'sound' && cb.checked) Sound.play('good'); App.renderStats(); });
    sw.append(cb, h('i')); row.append(sw); set.append(row);
  };
  tog('sound', 'Sound effects', 'Chimes for right answers, buzzes for mistakes.');
  if (Voice.ok) tog('voice', 'Read lessons aloud', 'Nodi narrates each story scene.');
  tog('unlimited', 'Unlimited battery', 'Mistakes never drain your battery.');
  tog('unlockAll', 'Unlock all lessons', 'Jump to any unit, for learners who already know the basics.');
  const gr = h('div', { class: 'set-row', html: '<label class="lbl" for="set-goal"><b>Daily goal</b><span>How much XP to aim for each day.</span></label>' });
  const sel = h('select', { class: 'sel', id: 'set-goal' }); [[10, 'Casual · 10 XP'], [20, 'Regular · 20 XP'], [30, 'Serious · 30 XP'], [50, 'Intense · 50 XP']].forEach(([n, l]) => { const o = h('option', { value: n }, l); if (S.goal === n) o.selected = true; sel.append(o); });
  sel.addEventListener('change', () => { S.goal = +sel.value; save(); App.renderAside(); });
  gr.append(sel); set.append(gr);
  const rr = h('div', { class: 'set-row', html: '<div class="lbl"><b>Reset progress</b><span>Start the whole course from scratch.</span></div>' });
  const rb = h('button', { class: 'btn btn-sm', style: '--fg:var(--bad)' }, 'Reset');
  rb.addEventListener('click', () => modal({ mood: 'sad', title: 'Reset all progress?', body: 'XP, streak, lessons, badges and lab missions will be erased. This cannot be undone.', actions: [{ label: 'Keep my progress', cls: 'btn-primary' }, { label: 'Erase everything', cls: 'btn-bad', onClick: () => { const keep = S.settings; S = freshState(); S.settings = keep; save(); App.go('learn'); toast('Fresh start! Nodi is ready when you are.', { mood: 'happy' }); } }] }));
  rr.append(rb); set.append(rr);
  st.append(set); v.append(st);
  root.append(v);
}
function weekChart() {
  const days = Array.from({ length: 7 }, (_, i) => { const k = dayKey(i - 6); const d = new Date(); d.setDate(d.getDate() + i - 6); return { k, xp: S.history[k] || 0, lbl: i === 6 ? 'Today' : d.toLocaleDateString(undefined, { weekday: 'short' }) }; });
  const W = 560, H = 200, pl = 34, pb = 26, pt = 18, max = Math.max(S.goal, ...days.map(d => d.xp)) * 1.15;
  const y = (v) => H - pb - (v / max) * (H - pb - pt), bw = (W - pl) / 7;
  let s = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="XP earned on each of the last 7 days">`;
  [0, S.goal].filter((g, i) => i === 0 || y(0) - y(g) >= 14).forEach(g => { s += `<line class="gl" x1="${pl}" x2="${W}" y1="${y(g).toFixed(1)}" y2="${y(g).toFixed(1)}"/><text class="lbl" x="${pl - 8}" y="${(y(g) + 4).toFixed(1)}" style="text-anchor:end">${g}</text>`; });
  days.forEach((d, i) => {
    const x = pl + i * bw + bw * 0.2, w = bw * 0.6 - 2, top = y(d.xp), hgt = (H - pb) - top;
    if (d.xp > 0) s += `<path class="bar" d="M${x} ${H - pb} V${(top + 4).toFixed(1)} Q${x} ${top.toFixed(1)} ${x + 4} ${top.toFixed(1)} H${x + w - 4} Q${x + w} ${top.toFixed(1)} ${x + w} ${(top + 4).toFixed(1)} V${H - pb} Z"/>`;
    s += `<text class="lbl" x="${x + w / 2}" y="${H - 8}">${esc(d.lbl)}</text>`;
    if (i === 6 && d.xp > 0) s += `<text class="val" x="${x + w / 2}" y="${(top - 6).toFixed(1)}">${d.xp}</text>`;
    s += `<rect class="hit" x="${pl + i * bw}" y="${pt}" width="${bw}" height="${H - pb - pt}" fill="transparent" data-i="${i}"/>`;
    void hgt;
  });
  s += `<line x1="${pl}" x2="${W}" y1="${H - pb}" y2="${H - pb}" style="stroke:var(--line)"/></svg>`;
  const wrap = h('div', { class: 'chart', style: 'position:relative' });
  wrap.innerHTML = s + `<table class="sr"><caption>XP per day</caption><tr>${days.map(d => `<th>${esc(d.lbl)}</th>`).join('')}</tr><tr>${days.map(d => `<td>${d.xp}</td>`).join('')}</tr></table>`;
  const tip = h('div', { style: 'position:absolute;pointer-events:none;background:var(--surface);border:2px solid var(--line);border-radius:10px;padding:4px 8px;font:600 13px var(--font-body);display:none;white-space:nowrap' });
  wrap.append(tip);
  $$('.hit', wrap).forEach(r => {
    const show = () => { const d = days[+r.dataset.i]; tip.textContent = `${d.lbl}: ${d.xp} XP`; tip.style.display = 'block'; const bb = r.getBoundingClientRect(), wb = wrap.getBoundingClientRect(); tip.style.left = (bb.left - wb.left + bb.width / 2 - tip.offsetWidth / 2) + 'px'; tip.style.top = '0px'; };
    r.addEventListener('mouseenter', show); r.addEventListener('mouseleave', () => { tip.style.display = 'none'; });
  });
  return wrap;
}

/* ---------- Ask Nodi (doubt solving with Claude) ---------- */
const Ask = {
  available: false, sample: null, turns: [], busy: false, ctl: null, views: new Set(),
  RULES: `You are Nodi, a friendly robot tutor inside a beginner course on ROS 2 (Jazzy Jalisco on Ubuntu 24.04). The course covers: what ROS is, ROS 1 vs ROS 2 and DDS, the ros2 command line and turtlesim, publishers and subscribers in Python (rclpy) and C++ (rclcpp), services, custom interfaces, parameters, launch files (XML and Python), TF2 broadcasters and listeners, and talking to microcontrollers over USB serial. The app also has a Robot Lab: a 2D differential-drive robot whose /robot_sim node publishes /odom (nav_msgs/msg/Odometry), /scan (sensor_msgs/msg/LaserScan with 360 rays, ranges[0] straight ahead, counterclockwise) and /bumper, subscribes to /cmd_vel (geometry_msgs/msg/Twist), and has a simulated ros2 terminal plus a Python runner for rclpy-style nodes.
There is also a SnoBots Robots section with 3D simulators of the learner's team robots (unofficial, simplified): Hiwonder AiNex (24-joint small humanoid; /cmd_vel walking, /joint_commands sensor_msgs/msg/JointState targets in radians, /action_group std_msgs/msg/String, /color_detection from the head camera), Unitree R1 (26-joint humanoid; modes damp/stand/walk/run/lowlevel via the /loco/set_mode service, /lowcmd per-motor PD commands q, dq, kp, kd, tau while hanging on a gantry, /lowstate), AgileX LIMO (four motion modes: 4WD differential, Ackermann with 0.4 m minimum radius, tracked, mecanum; /scan has 360 rays from -pi to pi so ranges[180] is straight ahead; /limo_status, /lane_error, /traffic_light, SLAM /map, /goal_pose navigation) and the LIMO Cobot (LIMO base plus a myCobot-280-style 6-axis arm: /arm/angles_goal in degrees, /arm/coords_goal in mm, /arm/gripper 0-100). These simulators run on the app's own MuJoCo-style rigid-body physics engine (not MuJoCo itself): link masses and inertia, torque-limited motors, contacts with friction. The humanoids walk with a model-based controller (footstep planner, linear inverted pendulum / DCM, capture-point steps) and can fall; a virtual hoist stands them back up. LIMO /odom comes from wheel encoders and drifts, and SLAM or navigation publish the map -> odom correction. Python nodes in SnoBots run on simulated time (use_sim_time is true), so their timers pause when the simulation is paused. The Physics tab has pause/step, contact and centre-of-mass overlays, Ctrl+drag pushing and domain randomization. Topic names there are simplified simulator names; for real hardware point learners to the vendor documentation.
Answer the learner's question clearly and briefly: usually under 170 words. Use plain language, one quick analogy when it helps, and short fenced code or terminal examples. Prefer ROS 2 Jazzy APIs. If a question is unrelated to ROS, robotics, Linux, Python or C++, help briefly and steer back to learning. Do not recommend particular video creators or channels.`,
  SUGG: ['What is the difference between a topic and a service?', 'Why do I need to source setup.bash?', 'Explain QoS like I am five', 'How do I make the lab robot avoid walls?'],
  async init() {
    try {
      if (!window.claude || typeof window.claude.use !== 'function') return;
      const s = await window.claude.use('sample');
      if (!s) return;
      this.sample = s; this.available = true; App.showAsk();
    } catch (e) { /* unavailable */ }
  },
  disable() { this.available = false; App.hideAsk(); },
  mountChat(container, opts = {}) {
    const thread = h('div', { class: 'chat', 'aria-live': 'polite' });
    const sugg = h('div', { class: 'sugg' });
    const comp = h('form', { class: 'composer' });
    const ta = h('textarea', { rows: '1', placeholder: 'Ask about ROS 2…', 'aria-label': 'Your question' });
    const send = h('button', { class: 'btn btn-primary', type: 'submit', 'aria-label': 'Send', html: icon('send') });
    comp.append(ta, send);
    const view = { thread, sugg, ta, send, scroller: opts.scroller || null };
    this.views.add(view);
    this.renderThread(view);
    this.SUGG.forEach(q => { const c = h('button', { class: 'chip', type: 'button' }, q); c.addEventListener('click', () => this.ask(q)); sugg.append(c); });
    ta.addEventListener('input', () => { ta.style.height = 'auto'; ta.style.height = Math.min(160, ta.scrollHeight) + 'px'; });
    ta.addEventListener('keydown', (e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); comp.requestSubmit(); } });
    comp.addEventListener('submit', (e) => { e.preventDefault(); if (this.busy) { this.ctl && this.ctl.abort(); return; } const q = ta.value.trim(); if (q) { ta.value = ''; ta.style.height = 'auto'; this.ask(q); } });
    container.append(thread, sugg, comp);
    if (opts.prefill) setTimeout(() => this.ask(opts.prefill), 50);
    return view;
  },
  renderThread(view) {
    view.thread.innerHTML = '';
    if (!this.turns.length) view.thread.append(h('div', { class: 'msgrow', html: `${nodi('happy', '')}<div class="bub"><p>Hi! I'm Nodi. Stuck on a ROS 2 idea, a command or some code? Ask me anything.</p></div>` }));
    this.turns.forEach(t => view.thread.append(this.bubble(t.role, t.content, t.err)));
    if (view.sugg) view.sugg.style.display = this.turns.length ? 'none' : '';
    this.scroll(view);
  },
  bubble(role, text, err) {
    if (role === 'user') return h('div', { class: 'msgrow me', html: `<div class="bub">${esc(text)}</div>` });
    return h('div', { class: 'msgrow', html: `${nodi(err ? 'sad' : 'idle', '')}<div class="bub">${err ? `<p>${esc(text)}</p>` : renderMD(text)}</div>` });
  },
  scroll(view) { const sc = view.scroller || document.scrollingElement; if (view.scroller) sc.scrollTop = sc.scrollHeight; else window.scrollTo(0, document.body.scrollHeight); },
  setBusy(b) { this.busy = b; this.views.forEach(v => { if (!v.thread.isConnected) { this.views.delete(v); return; } v.send.innerHTML = icon(b ? 'stop' : 'send'); v.send.setAttribute('aria-label', b ? 'Stop' : 'Send'); }); },
  async ask(q) {
    if (this.busy || !this.sample) return;
    this.turns.push({ role: 'user', content: q });
    const live = [];
    this.views.forEach(v => { if (!v.thread.isConnected) { this.views.delete(v); return; } this.renderThread(v); const row = h('div', { class: 'msgrow', html: `${nodi('think', '')}<div class="bub"><span class="typing"><span></span><span></span><span></span></span></div>` }); v.thread.append(row); live.push({ v, b: $('.bub', row) }); this.scroll(v); });
    this.setBusy(true);
    const ctl = this.ctl = new AbortController();
    const msgs = [{ role: 'user', content: this.RULES }, ...this.turns.filter(t => !t.err).slice(-12)];
    try {
      const { text, truncated } = await this.sample(msgs, { cache: false, signal: ctl.signal, onText: ({ text: tx }) => { live.forEach(({ v, b }) => { b.innerHTML = renderMD(tx); this.scroll(v); }); } });
      this.turns.push({ role: 'assistant', content: text + (truncated ? '\n\n*(answer cut short: ask me to continue)*' : '') });
    } catch (e) {
      const code = e && e.code;
      if (code === 'cancelled') { if (e.text) this.turns.push({ role: 'assistant', content: e.text + '\n\n*(stopped)*' }); else this.turns.pop(); }
      else if (['not_granted', 'sampling_disabled', 'not_declared', 'capability_disabled', 'capability_removed'].includes(code)) { this.turns.push({ role: 'assistant', content: 'Ask Nodi isn\'t available in this viewer. The course and the Robot Lab still work fully.', err: true }); setTimeout(() => this.disable(), 2500); }
      else if (code === 'rate_limited') this.turns.push({ role: 'assistant', content: 'Lots of questions right now! Give it a minute and ask again.', err: true });
      else if (code === 'session_expired') this.turns.push({ role: 'assistant', content: 'Your session expired. Sign in again, then ask me once more.', err: true });
      else if (code === 'refused') this.turns.push({ role: 'assistant', content: 'I can\'t help with that one. Try asking it a different way?', err: true });
      else this.turns.push({ role: 'assistant', content: (e && e.text ? e.text + '\n\n' : '') + 'Something went wrong on my side. Please try again.', err: !(e && e.text) });
    }
    this.setBusy(false);
    this.views.forEach(v => { if (v.thread.isConnected) this.renderThread(v); });
  },
  openDrawer(prefill) {
    if (!this.available) return;
    const back = h('div', { class: 'drawer-back' }), dr = h('div', { class: 'drawer', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Ask Nodi' });
    dr.innerHTML = `<div class="drawer-head">${nodi('happy', '')}<h3>Ask Nodi</h3><button class="icon-btn dx" aria-label="Close">${icon('x')}</button></div>`;
    const body = h('div', { class: 'drawer-body' }); dr.append(body);
    const close = () => { back.remove(); dr.remove(); document.removeEventListener('keydown', onKey); };
    const onKey = (e) => { if (e.key === 'Escape') { e.stopPropagation(); close(); } };
    $('.dx', dr).addEventListener('click', close); back.addEventListener('click', close); document.addEventListener('keydown', onKey);
    document.body.append(back, dr);
    const v = this.mountChat(body, { scroller: body, prefill });
    dr.append($('.composer', body)); dr.append(v.sugg);
    setTimeout(() => v.ta.focus(), 50);
  },
};
function renderAsk(root) {
  const v = h('div', { class: 'view' });
  v.append(h('div', { class: 'view-head', html: '<div><h1>Ask Nodi</h1><p>Your ROS 2 tutor. Ask about any idea, command or error message.</p></div>' }));
  root.append(v);
  Ask.mountChat(v);
}
