// Validates the course data: every lesson scene uses a known animation, every exercise has a
// well-formed answer key, and every source file is clean UTF-8 (no U+FFFD replacement characters).
const fs = require('fs'), vm = require('vm'), path = require('path');
const SRC = path.join(__dirname, '..', 'src');
const ctx = { console, Math, JSON, Date, setTimeout, clearTimeout, performance: { now: () => 0 } };
vm.createContext(ctx);
const code = ['core.js', 'anim.js', 'content1.js', 'content2.js', 'lesson.js'].map(f => fs.readFileSync(path.join(SRC, f), 'utf8')).join('\n') + '\n;this.__U = UNITS; this.__A = ANIM; this.__EX = EX;';
vm.runInContext(code, ctx);
const U = ctx.__U, A = ctx.__A, EX = ctx.__EX;
let errs = [], warns = [], nL = 0, nEx = 0, nSc = 0;
const E = (m) => errs.push(m), W = (m) => warns.push(m);
U.forEach(u => {
  if (!u.notes || u.notes.length < 4) W(`${u.id}: only ${u.notes && u.notes.length} notes`);
  u.lessons.forEach(l => {
    nL++;
    if (l.story.length < 3) W(`${l.id}: short story ${l.story.length}`);
    if (l.ex.length < 5) W(`${l.id}: only ${l.ex.length} exercises`);
    l.story.forEach((s, i) => {
      nSc++;
      const id = `${l.id} scene ${i + 1} (${s.a})`;
      if (!A[s.a]) E(`${id}: unknown animation`);
      if (!s.h || !s.t) E(`${id}: missing title or text`);
      if (s.a === 'code' && !s.code) E(`${id}: code scene without code`);
      if (s.a === 'code' && s.focus) { const n = s.code.split('\n').length; const rs = Array.isArray(s.focus[0]) ? s.focus : [s.focus]; rs.forEach(([a, b]) => { if (a < 1 || (b || a) > n) E(`${id}: focus ${a}-${b} outside ${n} lines`); }); }
      if (s.a === 'code' && s.steps) { const n = s.code.split('\n').length; s.steps.forEach(st => { const [a, b] = st.lines; if (a < 1 || (b || a) > n) E(`${id}: step ${a}-${b} outside ${n} lines`); }); }
      if (s.a === 'terminal' && !(s.panes && s.panes.length)) E(`${id}: terminal without panes`);
      if (s.a === 'graph') { const ids = new Set(s.nodes.map(n => n.id)); (s.edges || []).forEach(e => { if (!ids.has(e.a) || !ids.has(e.b)) E(`${id}: edge ${e.a}->${e.b} refers to missing node`); }); s.nodes.forEach(n => { if (n.x < 30 || n.x > 570 || n.y < 20 || n.y > 330) W(`${id}: node ${n.id} near edge (${n.x},${n.y})`); }); }
      if ((s.a === 'cards' || s.a === 'layers') && !(s.items && s.items.length)) E(`${id}: no items`);
      if (s.a === 'tree' && !(s.items && s.items.length)) E(`${id}: no tree items`);
      if (s.a === 'compare' && !(s.rows && s.rows.length)) E(`${id}: no rows`);
    });
    l.ex.forEach((x, i) => {
      nEx++;
      const id = `${l.id}#${i} (${x.k})`;
      if (!EX[x.k]) E(`${id}: unknown exercise type`);
      if (x.k !== 'match' && !x.q) E(`${id}: no question`);
      if (x.k === 'mcq') { if (!(x.o.length >= 2 && x.a >= 0 && x.a < x.o.length)) E(`${id}: bad answer index`); if (new Set(x.o).size !== x.o.length) E(`${id}: duplicate options`); }
      if (x.k === 'tf' && typeof x.a !== 'boolean') E(`${id}: tf answer not boolean`);
      if (x.k === 'fill') { const nb = x.s.split('___').length - 1; if (nb !== x.b.length) E(`${id}: ${nb} blanks vs ${x.b.length} answers`); (x.x || []).forEach(d => { if (x.b.includes(d)) E(`${id}: distractor equals an answer: ${d}`); }); }
      if (x.k === 'build') { if (!x.a.length) E(`${id}: empty answer`); (x.x || []).forEach(d => { if (x.a.includes(d)) W(`${id}: distractor duplicates answer token ${d}`); }); }
      if (x.k === 'order') { if (x.a.length < 3) W(`${id}: short order`); if (new Set(x.a).size !== x.a.length) E(`${id}: duplicate order items`); }
      if (x.k === 'match') { if (x.p.length < 3) W(`${id}: few pairs`); if (new Set(x.p.map(p => p[0])).size !== x.p.length || new Set(x.p.map(p => p[1])).size !== x.p.length) E(`${id}: duplicate match sides`); }
      if (x.k === 'type' && typeof x.a !== 'string') E(`${id}: type answer not string`);
      if (x.k === 'bug') { const n = x.code.split('\n').length; if (!(x.a >= 0 && x.a < n)) E(`${id}: bug line ${x.a} outside ${n}`); }
      if (x.k === 'pick') { if (!x.g.nodes.some(n => n.id === x.a)) E(`${id}: pick answer missing`); }
      if (x.k === 'turtle') { if (!(x.lin[0] <= x.lin[1] && x.ang[0] <= x.ang[1])) E(`${id}: bad ranges`); }
    });
  });
});
// encoding check: a U+FFFD in a source file means some text was mangled on the way in
for (const f of fs.readdirSync(SRC)) { const t = fs.readFileSync(path.join(SRC, f), 'utf8'); const i = t.indexOf('\uFFFD'); if (i >= 0) E(`${f}: replacement character at offset ${i}`); }
console.log(`units ${U.length} · lessons ${nL} · scenes ${nSc} · exercises ${nEx} · notes ${U.reduce((n, u) => n + u.notes.length, 0)}`);
warns.forEach(e => console.log('  warn', e));
errs.forEach(e => console.log('  FAIL', e));
console.log(errs.length ? `\n${errs.length} problems` : `\ncourse data OK (${warns.length} warnings)`);
process.exitCode = errs.length ? 1 : 0;
