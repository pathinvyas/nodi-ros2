/* ============================== Library: books and guides to read in the app ============================== */
// Each book either ships with the app (file: a PDF in library/, next to index.html) or is "bring your own copy"
// (own: true) for copyrighted books we may not share: a reader adds their own PDF and it stays in their browser.
// To add a book: put the PDF in library/, add an entry below and, if you like, a cover image in library/covers/
// (the build inlines covers written as 'LIBCOVER:<file>').
const LIBRARY = [
  {
    id: 'github-guide', title: "Patty's Guide to GitHub", sub: 'From your first commit to your first pull request', by: 'Pathin',
    file: 'library/pattys-guide-to-github.pdf', cover: 'LIBCOVER:pattys-guide-to-github.jpg', pages: 19, size: 190778, color: '#1B2A44',
    tags: ['Git', 'GitHub', 'Handbook'],
    blurb: 'A no-fluff handbook: the mental model, the daily loop, branches, pull requests, merge conflicts and an undo cookbook, with a cheat sheet, a glossary and a practice session for two.',
    toc: { "Patty's Guideto GitHub": 'Cover', 'Read this firstBefore you start': 'Read this first · Before you start' },
  },
  {
    id: 'amr2', title: 'Introduction to Autonomous Mobile Robots', sub: 'Second edition', by: 'Roland Siegwart, Illah R. Nourbakhsh and Davide Scaramuzza',
    pub: 'The MIT Press, 2011', own: true, pages: 473, color: '#0F5C6E', kicker: 'Textbook',
    link: 'https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/',
    tags: ['Locomotion', 'Kinematics', 'Perception', 'Localization', 'Navigation'],
    blurb: 'The classic course text on mobile robots, from wheels and legs through sensors, localization and mapping to planning and navigation, with problems at the end of every chapter.',
  },
  {
    id: 'rmc1', title: 'Robot Modeling and Control', sub: 'First edition', by: 'Mark W. Spong, Seth Hutchinson and M. Vidyasagar',
    pub: 'John Wiley & Sons, 2006', own: true, pages: 491, color: '#4B2A7A', kicker: 'Textbook',
    link: 'https://bcs.wiley.com/he-bcs/Books?action=index&bcsId=2983&itemId=0471649902',
    tags: ['Kinematics', 'Jacobians', 'Dynamics', 'Control', 'Visual servoing'],
    blurb: 'The standard course text on robot arms: rigid motions and homogeneous transforms, forward and inverse kinematics, the Jacobian, trajectory planning, dynamics, then joint, force, nonlinear and vision-based control, with worked examples and problems in every chapter.',
  },
  {
    id: 'cirp1', title: 'A Concise Introduction to Robot Programming with ROS2', sub: 'First edition', by: 'Francisco Martín Rico',
    pub: 'CRC Press, 2023', own: true, pages: 265, color: '#8A3A14', kicker: 'Textbook',
    link: 'https://routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS2/Rico/p/book/9781032264653',
    tags: ['ROS 2', 'C++ and Python', 'State machines', 'TF', 'Behavior trees'],
    blurb: 'A hands-on ROS 2 course built around a simulated mobile robot: workspaces, nodes, topics and launch files, a bump-and-go behavior as a state machine in C++ and Python, the TF system, obstacle avoidance and object tracking, then behavior trees for patrolling.',
  },
];
const PDFJS_BASE = window.LIB_PDFJS_BASE || 'https://cdnjs.cloudflare.com/ajax/libs/pdf.js/3.11.174/';
const fmtSize = (b) => b >= 1048576 ? (b / 1048576).toFixed(b >= 10485760 ? 0 : 1) + ' MB' : Math.max(1, Math.round(b / 1024)) + ' KB';

/* PDFs a reader adds live in this browser only (IndexedDB), never on a server. Falls back to memory for the visit. */
const LibStore = {
  mem: new Map(), dbp: null, persistent: null,
  db() {
    if (!this.dbp) this.dbp = new Promise((res) => {
      try {
        const rq = indexedDB.open('nodi-library', 1);
        rq.onupgradeneeded = () => { if (!rq.result.objectStoreNames.contains('pdfs')) rq.result.createObjectStore('pdfs', { keyPath: 'id' }); };
        rq.onsuccess = () => { this.persistent = true; res(rq.result); };
        rq.onerror = rq.onblocked = () => { this.persistent = false; res(null); };
      } catch (e) { this.persistent = false; res(null); }
    });
    return this.dbp;
  },
  async run(mode, fn) {
    const db = await this.db(); if (!db) return undefined;
    return new Promise((res, rej) => {
      try {
        const t = db.transaction('pdfs', mode), rq = fn(t.objectStore('pdfs'));
        t.oncomplete = () => res(rq ? rq.result : undefined);
        t.onerror = t.onabort = () => rej(t.error || new Error('Browser storage failed'));
      } catch (e) { rej(e); }
    });
  },
  async list() {
    let rows = [];
    try { rows = (await this.run('readonly', (st) => st.getAll())) || []; } catch (e) { rows = []; }
    const ids = new Set(rows.map(r => r.id));
    return rows.concat([...this.mem.values()].filter(r => !ids.has(r.id))).sort((a, b) => a.added - b.added);
  },
  async get(id) {
    if (this.mem.has(id)) return this.mem.get(id);
    try { return await this.run('readonly', (st) => st.get(id)); } catch (e) { return undefined; }
  },
  // returns true when the PDF will still be here next visit
  async put(rec) {
    try { if (await this.db()) { await this.run('readwrite', (st) => st.put(rec)); this.mem.delete(rec.id); return true; } } catch (e) { /* quota or private mode */ }
    this.mem.set(rec.id, rec); return false;
  },
  async del(id) { this.mem.delete(id); try { await this.run('readwrite', (st) => st.delete(id)); } catch (e) { /* ignore */ } },
};

let PDFJS_P = null;
function loadPdfJs() {
  if (window.pdfjsLib) return Promise.resolve(window.pdfjsLib);
  return PDFJS_P || (PDFJS_P = new Promise((res, rej) => {
    const s = document.createElement('script'); s.src = PDFJS_BASE + 'pdf.min.js'; s.async = true;
    s.onload = () => { try { window.pdfjsLib.GlobalWorkerOptions.workerSrc = PDFJS_BASE + 'pdf.worker.min.js'; res(window.pdfjsLib); } catch (e) { PDFJS_P = null; rej(e); } };
    s.onerror = () => { PDFJS_P = null; s.remove(); rej(new Error('The PDF reader could not load. Check your internet connection, then try again.')); };
    document.head.append(s);
  }));
}

/* Ask for a PDF from this device; resolves to a stored record or null. */
function libPickPdf(id, title) {
  return new Promise((resolve) => {
    const inp = h('input', { type: 'file', accept: 'application/pdf,.pdf', hidden: '' });
    inp.addEventListener('change', async () => { const f = inp.files && inp.files[0]; inp.remove(); resolve(f ? await libAddFile(f, id, title) : null); });
    document.body.append(inp); inp.click();
  });
}
async function libAddFile(file, id, title) {
  const head = new Uint8Array(await file.slice(0, 5).arrayBuffer());
  if (String.fromCharCode(...head) !== '%PDF-') { toast(`“${esc(file.name)}” is not a PDF.`, { mood: 'think' }); return null; }
  const name = file.name.replace(/\.pdf$/i, '');
  const rec = { id: id || 'local:' + Date.now().toString(36), title: title || name.replace(/[_-]+/g, ' ').replace(/\s+/g, ' ').trim() || 'Untitled PDF', name: file.name, size: file.size, added: Date.now(), blob: file };
  const kept = await LibStore.put(rec);
  toast(kept ? `Added “${esc(rec.title)}”. It stays in this browser.` : `Opened “${esc(rec.title)}” for this visit. This browser can't keep it, so add it again next time.`, { mood: 'happy' });
  return rec;
}

const LIB_GRAD = ['#2C74E0', '#0E9384', '#6B55E8', '#D23D84', '#C2873C', '#1F9D57'];
function libCover(b) {
  if (b.cover && !b.cover.startsWith('LIBCOVER:')) return `<div class="lib-cover"><img src="${b.cover}" alt="" loading="lazy"></div>`;
  return `<div class="lib-cover lib-cover-css" style="--b:${b.color || '#2C74E0'}" aria-hidden="true"><span class="k">${esc(b.kicker || (b.local ? 'Your PDF' : 'Book'))}</span><b>${esc(b.title)}</b>${b.sub ? `<span class="e">${esc(b.sub)}</span>` : ''}${b.pub ? `<span class="p">${esc(b.pub.replace(/,.*$/, ''))}</span>` : ''}</div>`;
}
function libProgress(id) {
  const p = (S.lib && S.lib.pos || {})[id];
  if (!p || !p.n) return '';
  const pct = Math.round(100 * p.p / p.n);
  return `<div class="lib-prog"><div class="bar"><i style="width:${pct}%"></i></div><span class="tabnum">Page ${p.p} of ${p.n}</span></div>`;
}

function renderLibrary(root) {
  const v = h('div', { class: 'view view-wide lib' });
  v.append(h('div', { class: 'view-head', html: '<div><h1>Library</h1><p>Books and guides to read right here. Nodi keeps your place in each one.</p></div>' }));
  const shelf = h('div', { class: 'lib-shelf' }); v.append(shelf);
  v.append(h('div', { class: 'lib-h2', html: `<h2>${icon('pkg')} On this device</h2><p class="sub">Read your own PDFs here too. They stay in this browser and are never uploaded.</p>` }));
  const mine = h('div', { class: 'lib-shelf lib-mine' }); v.append(mine);
  root.append(v);
  const draw = async () => {
    const recs = await LibStore.list(), have = new Set(recs.map(r => r.id));
    shelf.innerHTML = ''; mine.innerHTML = '';
    LIBRARY.forEach(b => shelf.append(libCard(b, b.own ? have.has('own:' + b.id) : true, draw)));
    recs.filter(r => r.id.startsWith('local:')).forEach(r => mine.append(libCard({ id: r.id, title: r.title, sub: r.name, local: true, size: r.size, pages: r.pages, color: LIB_GRAD[r.added % LIB_GRAD.length] }, true, draw)));
    const add = h('button', { class: 'lib-add', html: `${icon('plus')}<b>Add a PDF</b><span>or drop one here</span>` });
    add.addEventListener('click', async () => { const r = await libPickPdf(); if (r) { await draw(); Reader.open(r.id); } });
    ['dragenter', 'dragover'].forEach(ev => add.addEventListener(ev, (e) => { e.preventDefault(); add.classList.add('over'); }));
    add.addEventListener('dragleave', () => add.classList.remove('over'));
    add.addEventListener('drop', async (e) => { e.preventDefault(); add.classList.remove('over'); const f = e.dataTransfer.files && e.dataTransfer.files[0]; if (f) { const r = await libAddFile(f); if (r) await draw(); } });
    mine.append(add);
  };
  draw();
}

function libCard(b, ready, redraw) {
  const pos = (S.lib && S.lib.pos || {})[b.id];
  const c = h('article', { class: 'card lib-book' + (ready ? '' : ' lib-locked'), style: `--b:${b.color || 'var(--accent)'}` });
  const meta = [b.pages ? `${b.pages} pages` : '', b.size ? `PDF, ${fmtSize(b.size)}` : '', b.pub || ''].filter(Boolean).join(' · ');
  c.innerHTML = libCover(b) + `<div class="lib-info">
      <h3>${esc(b.title)}</h3>${b.sub && !b.local ? `<p class="lib-sub">${esc(b.sub)}</p>` : ''}${b.by ? `<p class="lib-by">by ${esc(b.by)}</p>` : ''}
      ${b.tags ? `<div class="lib-tags">${b.tags.map(t => `<span class="chip">${esc(t)}</span>`).join('')}</div>` : ''}
      ${b.blurb ? `<p class="lib-blurb">${esc(b.blurb)}</p>` : ''}
      ${meta ? `<p class="lib-meta">${esc(meta)}</p>` : ''}
      ${ready ? libProgress(b.id) : `<p class="lib-note">${icon('lock')}<span>This book is copyrighted, so Nodi can't share the PDF. If you have a copy, add it from your device. It stays in this browser and is never uploaded.</span></p>`}
      <div class="lib-actions"></div></div>`;
  const acts = $('.lib-actions', c), key = b.own ? 'own:' + b.id : b.id;
  if (ready) {
    const rd = h('button', { class: 'btn btn-primary btn-sm', html: `${icon('book')}${pos && pos.p > 1 ? 'Continue' : 'Read'}` });
    rd.addEventListener('click', () => Reader.open(b.id, rd)); acts.append(rd);
    if (b.file) acts.append(h('a', { class: 'btn btn-sm', href: b.file, target: '_blank', rel: 'noopener', html: `${icon('external')}Open PDF` }));
    if (b.own || b.local) {
      const rm = h('button', { class: 'btn btn-ghost btn-sm', html: `${icon('x')}${b.own ? 'Remove your copy' : 'Remove'}` });
      rm.addEventListener('click', () => modal({ mood: 'think', title: `Remove “${b.title}” from this browser?`, body: b.own ? 'Your reading place is kept. You can add your copy again any time.' : 'The PDF is deleted from this browser. Your file on your device is not touched.', actions: [{ label: 'Remove', cls: 'btn-bad', onClick: async () => { await LibStore.del(key); if (b.local && S.lib.pos) { delete S.lib.pos[b.id]; save({ local: true }); } redraw(); } }, { label: 'Keep it', cls: 'btn-ghost' }] }));
      acts.append(rm);
    }
  } else {
    const add = h('button', { class: 'btn btn-primary btn-sm', html: `${icon('plus')}Add your copy` });
    add.addEventListener('click', async () => { const r = await libPickPdf(key, b.title); if (r) { await redraw(); Reader.open(b.id); } });
    acts.append(add);
    if (b.link) acts.append(h('a', { class: 'btn btn-sm', href: b.link, target: '_blank', rel: 'noopener', html: `${icon('external')}Publisher page` }));
  }
  return c;
}

/* ------------------------------ the reader: PDF.js pages, rendered lazily as you scroll ------------------------------ */
const libWait = (fn, ms) => new Promise((res) => { const t0 = Date.now(); const tick = () => { if (fn()) res(true); else if (Date.now() - t0 > ms) res(false); else setTimeout(tick, 40); }; tick(); });
// search text: one character per character, folded to lower case without accents, so positions map back to the page
const libFold = (ch) => ch.normalize('NFD').charAt(0).toLowerCase().charAt(0);
const Reader = {
  el: null, pdf: null, book: null, slots: [], n: 0, cur: 1, zoom: 1, cssW: 600, gen: 0, io: null, opener: null,
  texts: [], find: null,
  async open(id, opener) {
    if (this.el) this.close();
    const book = LIBRARY.find(b => b.id === id) || null;
    const rec = !book || book.own ? await LibStore.get(book ? 'own:' + id : id) : null;
    if (book && book.own && !rec) { toast('Add your copy of this book first.', { mood: 'think' }); return; }
    if (!book && !rec) { toast('That PDF is no longer in this browser.', { mood: 'think' }); return; }
    this.book = book || { id, title: rec.title, local: true }; this.id = id; this.opener = opener || document.activeElement;
    this.gen++; this.zoom = 1; this.cur = 1; this.n = 0; this.slots = []; this.texts = []; this.find = { q: '', matches: [], byPage: new Map(), cur: -1, scanned: 0, gen: 0, open: false };
    const el = this.el = h('div', { class: 'reader', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Reading ' + this.book.title });
    el.innerHTML = `<div class="reader-top">
        <button class="icon-btn" data-a="close" aria-label="Back to the library" title="Back (Esc)">${icon('arrowL')}</button>
        <div class="reader-title"><b>${esc(this.book.title)}</b>${this.book.sub && !this.book.local ? `<span>${esc(this.book.sub)}</span>` : ''}</div>
        <div class="reader-tools">
          <button class="icon-btn" data-a="find" aria-label="Search in this book" title="Search (Ctrl+F)" aria-expanded="false" hidden>${icon('search')}</button>
          <button class="icon-btn" data-a="toc" aria-label="Contents" title="Contents" aria-expanded="false" hidden>${icon('toc')}</button>
          <label class="reader-pg"><span class="sr">Page</span><input type="number" inputmode="numeric" min="1" value="1" aria-label="Page number"><span class="tabnum" data-n>/ …</span></label>
          <span class="reader-zoom"><button class="icon-btn" data-a="zout" aria-label="Zoom out" title="Zoom out (−)">${icon('minus')}</button><button class="btn btn-sm btn-ghost" data-a="fit" title="Fit the page width (0)">Fit</button><button class="icon-btn" data-a="zin" aria-label="Zoom in" title="Zoom in (+)">${icon('plus')}</button></span>
        </div></div>
      <div class="reader-find" role="search" hidden>
        <label class="reader-find-box">${icon('search')}<span class="sr">Search in this book</span><input type="search" placeholder="Search in this book" autocomplete="off" spellcheck="false" enterkeyhint="search"></label>
        <span class="reader-find-n tabnum" aria-live="polite"></span>
        <button class="icon-btn" data-a="fprev" aria-label="Previous match" title="Previous (Shift+Enter)">${icon('arrowU')}</button>
        <button class="icon-btn" data-a="fnext" aria-label="Next match" title="Next (Enter)">${icon('arrowD')}</button>
        <button class="btn btn-sm btn-ghost" data-a="hits" aria-expanded="false" title="List every match">${icon('toc')}<span>All results</span></button>
        <button class="icon-btn" data-a="fclose" aria-label="Close search" title="Close search (Esc)">${icon('x')}</button>
      </div>
      <div class="reader-body" tabindex="-1"><div class="reader-pages"></div><div class="reader-msg">${nodi('think', 'bob')}<p>Opening the book…</p></div></div>
      <nav class="reader-drawer reader-toc" aria-label="Contents" hidden><div class="reader-drawer-h"><b>Contents</b><button class="icon-btn" data-a="toc" aria-label="Close contents">${icon('x')}</button></div><ol></ol></nav>
      <nav class="reader-drawer reader-hits" aria-label="Search results" hidden><div class="reader-drawer-h"><b>Results</b><button class="icon-btn" data-a="hits" aria-label="Close results">${icon('x')}</button></div><p class="reader-hits-n"></p><ol></ol></nav>`;
    document.body.append(el); document.body.classList.add('reader-open');
    el.addEventListener('click', (e) => { const a = e.target.closest('[data-a]'); if (!a) return; const k = a.dataset.a;
      if (k === 'close') this.close(); else if (k === 'zin') this.setZoom(this.zoom * 1.25); else if (k === 'zout') this.setZoom(this.zoom / 1.25); else if (k === 'fit') this.setZoom(1);
      else if (k === 'toc') this.drawer('toc'); else if (k === 'hits') this.drawer('hits'); else if (k === 'find') this.openFind(); else if (k === 'fclose') this.closeFind();
      else if (k === 'fnext') this.step(1); else if (k === 'fprev') this.step(-1); });
    const inp = $('.reader-pg input', el);
    inp.addEventListener('change', () => this.goto(parseInt(inp.value, 10) || 1));
    inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); this.goto(parseInt(inp.value, 10) || 1); $('.reader-body', el).focus(); } e.stopPropagation(); });
    const fi = $('.reader-find input', el);
    fi.addEventListener('input', () => { clearTimeout(this.ft); this.ft = setTimeout(() => this.search(fi.value), 250); });
    fi.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); clearTimeout(this.ft); if (this.find.q !== libNorm(fi.value)) this.search(fi.value); else this.step(e.shiftKey ? -1 : 1); }
      else if (e.key === 'Escape') { e.preventDefault(); this.closeFind(); }
      e.stopPropagation();
    });
    this.body = $('.reader-body', el); this.pagesEl = $('.reader-pages', el);
    this.onKey = (e) => this.key(e); document.addEventListener('keydown', this.onKey);
    this.onResize = () => { clearTimeout(this.rt); this.rt = setTimeout(() => this.layout(true), 150); }; window.addEventListener('resize', this.onResize);
    this.body.addEventListener('scroll', () => { if (!this.sp) this.sp = requestAnimationFrame(() => { this.sp = 0; this.track(); }); }, { passive: true });
    setTimeout(() => $('[data-a="close"]', el).focus(), 30);
    const gen = this.gen;
    try {
      const lib = await loadPdfJs();
      const src = rec ? { data: new Uint8Array(await rec.blob.arrayBuffer()) } : { url: book.file };
      const pdf = await lib.getDocument(Object.assign(src, { isEvalSupported: false })).promise;
      if (gen !== this.gen) { pdf.destroy(); return; }
      this.pdf = pdf; this.n = pdf.numPages;
      const p1 = await pdf.getPage(1); this.base = p1.getViewport({ scale: 1 });
      if (rec && !rec.pages) { rec.pages = this.n; LibStore.put(rec); }
      $('[data-n]', el).textContent = '/ ' + this.n; inp.max = String(this.n);
      $('.reader-msg', el).remove(); $('[data-a="find"]', el).hidden = false;
      for (let i = 1; i <= this.n; i++) { const s = h('div', { class: 'reader-page', 'data-p': String(i) }); this.pagesEl.append(s); this.slots.push({ el: s, i, state: null, ratio: this.base.height / this.base.width }); }
      this.io = new IntersectionObserver((ents) => ents.forEach(en => { const s = this.slots[+en.target.dataset.p - 1]; if (en.isIntersecting) this.render(s); else this.free(s); }), { root: this.body, rootMargin: '120% 0px' });
      this.layout(false);
      const saved = (S.lib.pos || {})[this.id];
      if (saved && saved.p > 1 && saved.p <= this.n) this.goto(saved.p, true);
      this.slots.forEach(s => this.io.observe(s.el));
      this.track(); this.buildToc();
    } catch (e) {
      if (gen !== this.gen) return;
      const m = $('.reader-msg', el) || h('div', { class: 'reader-msg' }); if (!m.parentNode) this.body.append(m);
      m.innerHTML = `${nodi('sad', '')}<p>${esc(e && e.message ? e.message : 'This PDF could not be opened.')}</p>`;
      const retry = h('button', { class: 'btn btn-primary' }, 'Try again'); retry.addEventListener('click', () => this.open(id, this.opener)); m.append(retry);
    }
  },
  // page width in CSS pixels: fit the reading column, then zoom
  layout(keep) {
    if (!this.el || !this.n) return;
    const keepPage = this.cur, frac = keep ? this.pageFrac() : 0;
    const fitW = Math.max(240, Math.min(this.body.clientWidth - (this.body.clientWidth < 640 ? 16 : 48), 920));
    const w = Math.round(fitW * this.zoom); if (w === this.cssW && keep) return;
    this.cssW = w; this.pagesEl.style.setProperty('--pw', w + 'px');
    this.slots.forEach(s => { s.el.style.height = Math.round(w * s.ratio) + 'px'; if (s.state) this.free(s, true); });
    if (keep) { this.goto(keepPage, true, frac); this.slots.forEach(s => { const r = s.el.getBoundingClientRect(), b = this.body.getBoundingClientRect(); if (r.bottom > b.top - b.height && r.top < b.bottom + b.height) this.render(s); }); }
  },
  pageFrac() { const s = this.slots[this.cur - 1]; if (!s) return 0; return Math.max(0, Math.min(1, (this.body.scrollTop - s.el.offsetTop) / s.el.offsetHeight)); },
  setZoom(z) { this.zoom = Math.max(0.5, Math.min(3, Math.round(z * 100) / 100)); this.layout(true); },
  async render(s) {
    if (!s || s.state || !this.pdf) return;
    s.state = 'loading'; s.tlReady = false; const gen = this.gen, w = this.cssW;
    try {
      const page = await this.pdf.getPage(s.i); if (gen !== this.gen || s.state !== 'loading') return;
      const vp1 = page.getViewport({ scale: 1 }), scale = w / vp1.width, vp = page.getViewport({ scale });
      s.ratio = vp1.height / vp1.width; s.el.style.height = Math.round(vp.height) + 'px';
      const dpr = Math.min(2, window.devicePixelRatio || 1), cv = document.createElement('canvas');
      cv.width = Math.floor(vp.width * dpr); cv.height = Math.floor(vp.height * dpr); cv.style.width = Math.round(vp.width) + 'px'; cv.style.height = Math.round(vp.height) + 'px';
      s.task = page.render({ canvasContext: cv.getContext('2d', { alpha: false }), viewport: vp, transform: dpr !== 1 ? [dpr, 0, 0, dpr, 0, 0] : null });
      await s.task.promise; s.task = null;
      if (gen !== this.gen || s.state !== 'loading' || w !== this.cssW) return;
      s.el.textContent = ''; s.el.append(cv); s.state = 'done';
      // invisible text on top of the picture, so you can select, copy and search the words on the page
      const tl = h('div', { class: 'textLayer' }); s.el.style.setProperty('--scale-factor', String(scale)); s.el.append(tl);
      const tc = await page.getTextContent(); if (s.state !== 'done' || gen !== this.gen) return;
      this.noteText(s.i, tc);
      s.divs = []; s.strs = []; s.painted = [];
      await window.pdfjsLib.renderTextLayer({ textContentSource: tc, container: tl, viewport: vp, textDivs: s.divs, textContentItemsStr: s.strs }).promise;
      if (s.state !== 'done' || gen !== this.gen) return;
      s.tlReady = true; this.paint(s);
    } catch (e) { if (s.state === 'loading') s.state = null; }
  },
  free(s, force) {
    if (!s || !s.state) return;
    if (!force && Math.abs(s.i - this.cur) < 3) return;
    if (s.task) { try { s.task.cancel(); } catch (e) { /* already done */ } s.task = null; }
    s.el.textContent = ''; s.state = null; s.tlReady = false; s.divs = s.strs = s.painted = null;
  },
  // jumps are instant, like most PDF viewers: smooth scrolling across hundreds of pages would render every page on the way
  goto(p, instant, frac = 0) {
    p = Math.max(1, Math.min(this.n || 1, p)); const s = this.slots[p - 1]; if (!s) return;
    this.body.scrollTop = s.el.offsetTop - 12 + frac * s.el.offsetHeight;
    this.setCur(p);
  },
  // the current page is the one crossing the upper third of the view
  track() {
    if (!this.slots.length) return;
    const y = this.body.scrollTop + this.body.clientHeight / 3; let lo = 0, hi = this.slots.length - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (this.slots[mid].el.offsetTop <= y) lo = mid; else hi = mid - 1; }
    this.setCur(lo + 1);
  },
  setCur(p) {
    if (p === this.cur && this.saved) return; this.cur = p;
    const inp = this.el && $('.reader-pg input', this.el); if (inp && document.activeElement !== inp) inp.value = String(p);
    clearTimeout(this.st); this.st = setTimeout(() => { if (!this.n) return; S.lib.pos = S.lib.pos || {}; S.lib.pos[this.id] = { p: this.cur, n: this.n, t: Date.now() }; this.saved = true; save({ local: true }); }, 500);
  },
  key(e) {
    if (!this.el || $('.modal-back')) return;
    if ((e.ctrlKey || e.metaKey) && !e.altKey && e.key.toLowerCase() === 'f' && this.n) { e.preventDefault(); this.openFind(); return; }
    if (e.key === 'Escape') {
      e.preventDefault();
      const open = $$('.reader-drawer', this.el).find(d => !d.hidden);
      if (open) this.drawer(open.classList.contains('reader-hits') ? 'hits' : 'toc', false); else if (this.find.open) this.closeFind(); else this.close();
      return;
    }
    if (e.target && /^(INPUT|TEXTAREA)$/.test(e.target.tagName)) return;
    if (e.key === 'ArrowRight' || e.key === 'PageDown' || e.key === 'j') { e.preventDefault(); this.goto(this.cur + 1); }
    else if (e.key === 'ArrowLeft' || e.key === 'PageUp' || e.key === 'k') { e.preventDefault(); this.goto(this.cur - 1); }
    else if (e.key === 'Home') { e.preventDefault(); this.goto(1); }
    else if (e.key === 'End') { e.preventDefault(); this.goto(this.n); }
    else if ((e.key === '+' || e.key === '=') && !e.metaKey && !e.ctrlKey) { e.preventDefault(); this.setZoom(this.zoom * 1.25); }
    else if (e.key === '-' && !e.metaKey && !e.ctrlKey) { e.preventDefault(); this.setZoom(this.zoom / 1.25); }
    else if (e.key === '0' && !e.metaKey && !e.ctrlKey) { e.preventDefault(); this.setZoom(1); }
  },
  async buildToc() {
    let items = [];
    try { items = (await this.pdf.getOutline()) || []; } catch (e) { items = []; }
    if (!items.length || !this.el) return;
    const ren = (this.book && this.book.toc) || {};
    // some PDF tools glue a heading's label to its title ("Chapter 1The mental model"): put the gap back
    const clean = (t) => ren[t] || String(t || '').replace(/^((?:Chapter|Part|Appendix) [\dA-Z]+)(?=[A-Z][a-z])/, '$1 · ').trim();
    const pageOf = async (dest) => { try { const d = typeof dest === 'string' ? await this.pdf.getDestination(dest) : dest; return d ? (await this.pdf.getPageIndex(d[0])) + 1 : 0; } catch (e) { return 0; } };
    const ol = $('.reader-toc ol', this.el);
    for (const it of items) {
      const li = h('li'); const p = await pageOf(it.dest);
      const b = h('button', { html: `<span>${esc(clean(it.title))}</span><span class="tabnum">${p || ''}</span>` }); if (p) b.addEventListener('click', () => { this.goto(p); this.drawer('toc', false); });
      li.append(b);
      if (it.items && it.items.length) {
        const sub = h('ol');
        for (const c of it.items) { const pc = await pageOf(c.dest); const bb = h('button', { html: `<span>${esc(clean(c.title))}</span><span class="tabnum">${pc || ''}</span>` }); if (pc) bb.addEventListener('click', () => { this.goto(pc); this.drawer('toc', false); }); sub.append(h('li', {}, bb)); }
        li.append(sub);
      }
      ol.append(li);
    }
    if (this.el) $('.reader-top [data-a="toc"]', this.el).hidden = false;
  },
  // side panels: 'toc' (contents) or 'hits' (search results); only one is open at a time
  drawer(which, on) {
    if (!this.el) return;
    const d = $(which === 'hits' ? '.reader-hits' : '.reader-toc', this.el), other = $(which === 'hits' ? '.reader-toc' : '.reader-hits', this.el);
    const btn = $(which === 'hits' ? '.reader-find [data-a="hits"]' : '.reader-top [data-a="toc"]', this.el);
    const show = on == null ? d.hidden : on; d.hidden = !show; if (btn) btn.setAttribute('aria-expanded', String(show));
    if (show && !other.hidden) { other.hidden = true; }
    if (show) { if (which === 'hits') this.listHits(); const b = $('ol button', d); (b || $('.reader-drawer-h button', d)).focus(); } else if (btn && !btn.closest('[hidden]')) btn.focus();
  },

  /* ---------- search: every page's text is read once (in the background), matches are highlighted on the page ---------- */
  noteText(p, tc) {
    if (this.texts[p]) return this.texts[p];
    let norm = '', orig = '', sp = true, ti = -1; const item = [], off = [];
    const push = (n, o, it, k) => { norm += n; orig += o; item.push(it); off.push(k); };
    for (const it of tc.items) {
      if (it.str === undefined) continue; ti++;
      const str = it.str; let joined = false;
      for (let k = 0; k < str.length; k++) {
        const ch = str[k];
        if (/\s/.test(ch)) { if (!sp) { push(' ', ' ', ti, k); sp = true; } continue; }
        // a hyphen ending a line splits one word over two lines ("knowl-" / "edge"): drop it so the word can be found
        if ((ch === '-' || ch === '­') && k === str.length - 1 && it.hasEOL && k > 0 && /\p{L}/u.test(str[k - 1])) { joined = true; continue; }
        push(libFold(ch), ch, ti, k); sp = false;
      }
      if (it.hasEOL && !joined && !sp) { push(' ', ' ', -1, -1); sp = true; }
    }
    return (this.texts[p] = { norm, orig, item: Int32Array.from(item), off: Int32Array.from(off) });
  },
  async pageText(p) {
    if (this.texts[p]) return this.texts[p];
    const page = await this.pdf.getPage(p); const tc = await page.getTextContent();
    return this.noteText(p, tc);
  },
  openFind() {
    if (!this.el || !this.n) return;
    const bar = $('.reader-find', this.el), fi = $('input', bar);
    bar.hidden = false; this.find.open = true; $('.reader-top [data-a="find"]', this.el).setAttribute('aria-expanded', 'true');
    fi.focus(); fi.select();
  },
  closeFind() {
    if (!this.el) return;
    clearTimeout(this.ft); $('.reader-find', this.el).hidden = true; this.drawer('hits', false);
    const f = this.find; f.gen++; f.open = false; f.q = ''; f.matches = []; f.byPage = new Map(); f.cur = -1;
    this.slots.forEach(s => this.paint(s)); this.count();
    const b = $('.reader-top [data-a="find"]', this.el); b.setAttribute('aria-expanded', 'false'); b.focus();
  },
  async search(raw) {
    if (!this.el || !this.find.open) return;
    const f = this.find, q = libNorm(raw), gen = ++f.gen;
    f.q = q; f.matches = []; f.byPage = new Map(); f.cur = -1; f.scanned = 0; f.from = this.cur;
    this.slots.forEach(s => this.paint(s)); this.count(); this.listHits();
    if (q.length < 2) return;
    for (let p = 1; p <= this.n; p++) {
      let T; try { T = await this.pageText(p); } catch (e) { T = null; }
      if (gen !== f.gen || !this.el) return;
      if (T) {
        const list = [];
        for (let i = T.norm.indexOf(q); i >= 0; i = T.norm.indexOf(q, i + q.length)) { list.push(f.matches.length); f.matches.push({ p, a: i, b: i + q.length }); }
        if (list.length) { f.byPage.set(p, list); const s = this.slots[p - 1]; if (s && s.tlReady) this.paint(s); }
      }
      f.scanned = p;
      // jump to the first match at or after the page you were reading, as soon as it turns up
      if (f.cur < 0 && f.matches.length && f.matches[f.matches.length - 1].p >= f.from) this.show(f.matches.findIndex(m => m.p >= f.from));
      if (p % 8 === 0 || p === this.n) { this.count(); if (!$('.reader-hits', this.el).hidden) this.listHits(); await new Promise(r => setTimeout(r, 0)); if (gen !== f.gen) return; }
    }
    if (f.cur < 0 && f.matches.length) this.show(0);
    this.count(); this.listHits();
  },
  step(d) {
    const f = this.find, n = f.matches.length; if (!n) { if (f.q !== libNorm($('.reader-find input', this.el).value)) this.search($('.reader-find input', this.el).value); return; }
    this.show(((f.cur < 0 ? (d > 0 ? -1 : 0) : f.cur) + d + n) % n);
  },
  // make match i the current one: mark it, bring its page in, and scroll it into view once the text layer is there
  async show(i) {
    const f = this.find, m = f.matches[i]; if (!m) return;
    const prev = f.matches[f.cur]; f.cur = i; this.count();
    [prev && prev.p, m.p].forEach(p => { if (p) this.paint(this.slots[p - 1]); });
    const hit = $(`.reader-hits li[data-m="${i}"] button`, this.el); $$('.reader-hits button.on', this.el).forEach(b => b.classList.remove('on')); if (hit) hit.classList.add('on');
    const s = this.slots[m.p - 1], vis = () => { const r = s.el.getBoundingClientRect(), b = this.body.getBoundingClientRect(); return r.bottom > b.top && r.top < b.bottom; };
    if (!vis()) this.goto(m.p, true);
    this.render(s);
    if (!(await libWait(() => s.tlReady || f.cur !== i || !this.el, 6000)) || f.cur !== i || !this.el) return;
    const sp = $(`.highlight[data-m="${i}"]`, s.el); if (!sp) return;
    const r = sp.getBoundingClientRect(), b = this.body.getBoundingClientRect();
    if (r.top < b.top + 48 || r.bottom > b.bottom - 48) this.body.scrollTop += r.top - b.top - b.height / 3;
    if (r.left < b.left + 16 || r.right > b.right - 16) this.body.scrollLeft += r.left - b.left - b.width / 3;
  },
  count() {
    if (!this.el) return;
    const f = this.find, n = f.matches.length, out = $('.reader-find-n', this.el), busy = f.q.length >= 2 && f.scanned < this.n;
    out.textContent = f.q.length < 2 ? '' : n ? `${f.cur >= 0 ? f.cur + 1 : '–'} of ${n}${busy ? '+' : ''}` : busy ? `Searching… ${Math.round(100 * f.scanned / this.n)}%` : 'No matches';
    out.classList.toggle('none', f.q.length >= 2 && !n && !busy);
    $$('.reader-find [data-a="fnext"], .reader-find [data-a="fprev"]', this.el).forEach(b => { b.disabled = !n; });
  },
  // the results panel: each match with a little of the text around it
  listHits() {
    if (!this.el) return;
    const f = this.find, d = $('.reader-hits', this.el); if (d.hidden) return;
    const ol = $('ol', d), cap = 300, n = f.matches.length;
    $('.reader-hits-n', d).textContent = f.q.length < 2 ? 'Type at least two letters to search.' : n ? `${n} match${n === 1 ? '' : 'es'}${f.scanned < this.n ? ' so far' : ''}${n > cap ? `, showing the first ${cap}` : ''}` : f.scanned < this.n ? 'Searching…' : 'No matches in this book.';
    ol.textContent = '';
    f.matches.slice(0, cap).forEach((m, i) => {
      const T = this.texts[m.p], a = Math.max(0, m.a - 46), b = Math.min(T.orig.length, m.b + 60);
      const btn = h('button', { class: i === f.cur ? 'on' : null }); btn.append(h('span', { class: 'tabnum hit-p' }, 'p. ' + m.p));
      const snip = h('span', { class: 'hit-t' }, (a > 0 ? '…' : '') + T.orig.slice(a, m.a), h('mark', {}, T.orig.slice(m.a, m.b)), T.orig.slice(m.b, b) + (b < T.orig.length ? '…' : ''));
      btn.append(snip); btn.addEventListener('click', () => { this.show(i); if (window.innerWidth < 900) this.drawer('hits', false); });
      ol.append(h('li', { 'data-m': String(i) }, btn));
    });
  },
  // put this page's matches into its text layer as highlight spans (or take them out)
  paint(s) {
    if (!s || !s.tlReady || !s.divs) return;
    (s.painted || []).forEach(di => { if (s.divs[di]) s.divs[di].textContent = s.strs[di]; });
    s.painted = [];
    const f = this.find, ids = f.byPage.get(s.i), T = this.texts[s.i]; if (!ids || !T) return;
    const per = new Map();
    for (const mi of ids) {
      const m = f.matches[mi];
      for (let k = m.a; k < m.b; k++) {
        const it = T.item[k]; if (it < 0) continue; const o = T.off[k];
        let arr = per.get(it); if (!arr) per.set(it, arr = []);
        const last = arr[arr.length - 1]; if (last && last[2] === mi) last[1] = o + 1; else arr.push([o, o + 1, mi]);
      }
    }
    for (const [di, ranges] of per) {
      const div = s.divs[di], str = s.strs[di]; if (!div || str == null) continue;
      ranges.sort((x, y) => x[0] - y[0]); div.textContent = ''; let pos = 0;
      for (const [a, b, mi] of ranges) {
        if (a < pos) continue;
        if (a > pos) div.append(str.slice(pos, a));
        div.append(h('span', { class: 'highlight' + (mi === f.cur ? ' selected' : ''), 'data-m': String(mi) }, str.slice(a, b))); pos = b;
      }
      if (pos < str.length) div.append(str.slice(pos));
      s.painted.push(di);
    }
  },
  close() {
    if (!this.el) return;
    clearTimeout(this.st); clearTimeout(this.ft);
    if (this.n) { S.lib.pos = S.lib.pos || {}; S.lib.pos[this.id] = { p: this.cur, n: this.n, t: Date.now() }; save({ local: true }); }
    this.gen++; if (this.find) this.find.gen++; if (this.io) this.io.disconnect(); this.io = null;
    document.removeEventListener('keydown', this.onKey); window.removeEventListener('resize', this.onResize);
    if (this.pdf) { try { this.pdf.destroy(); } catch (e) { /* ignore */ } } this.pdf = null;
    this.el.remove(); this.el = null; this.slots = []; this.texts = []; this.n = 0; this.saved = false; document.body.classList.remove('reader-open');
    if (App.view === 'library') App.refresh();
    const o = this.opener; this.opener = null; if (o && o.isConnected) o.focus();
  },
};
function libNorm(raw) { let out = '', sp = true; for (const ch of String(raw || '')) { if (/\s/.test(ch)) { if (!sp) { out += ' '; sp = true; } } else { out += libFold(ch); sp = false; } } return out.trim(); }
