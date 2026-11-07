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
const Reader = {
  el: null, pdf: null, book: null, slots: [], n: 0, cur: 1, zoom: 1, cssW: 600, gen: 0, io: null, opener: null,
  async open(id, opener) {
    if (this.el) this.close();
    const book = LIBRARY.find(b => b.id === id) || null;
    const rec = !book || book.own ? await LibStore.get(book ? 'own:' + id : id) : null;
    if (book && book.own && !rec) { toast('Add your copy of this book first.', { mood: 'think' }); return; }
    if (!book && !rec) { toast('That PDF is no longer in this browser.', { mood: 'think' }); return; }
    this.book = book || { id, title: rec.title, local: true }; this.id = id; this.opener = opener || document.activeElement;
    this.gen++; this.zoom = 1; this.cur = 1; this.n = 0; this.slots = [];
    const el = this.el = h('div', { class: 'reader', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Reading ' + this.book.title });
    el.innerHTML = `<div class="reader-top">
        <button class="icon-btn" data-a="close" aria-label="Back to the library" title="Back (Esc)">${icon('arrowL')}</button>
        <div class="reader-title"><b>${esc(this.book.title)}</b>${this.book.sub && !this.book.local ? `<span>${esc(this.book.sub)}</span>` : ''}</div>
        <div class="reader-tools">
          <button class="icon-btn" data-a="toc" aria-label="Contents" title="Contents" aria-expanded="false" hidden>${icon('toc')}</button>
          <label class="reader-pg"><span class="sr">Page</span><input type="number" inputmode="numeric" min="1" value="1" aria-label="Page number"><span class="tabnum" data-n>/ …</span></label>
          <span class="reader-zoom"><button class="icon-btn" data-a="zout" aria-label="Zoom out" title="Zoom out (−)">${icon('minus')}</button><button class="btn btn-sm btn-ghost" data-a="fit" title="Fit the page width (0)">Fit</button><button class="icon-btn" data-a="zin" aria-label="Zoom in" title="Zoom in (+)">${icon('plus')}</button></span>
        </div></div>
      <div class="reader-body" tabindex="-1"><div class="reader-pages"></div><div class="reader-msg">${nodi('think', 'bob')}<p>Opening the book…</p></div></div>
      <nav class="reader-toc" aria-label="Contents" hidden><div class="reader-toc-h"><b>Contents</b><button class="icon-btn" data-a="toc" aria-label="Close contents">${icon('x')}</button></div><ol></ol></nav>`;
    document.body.append(el); document.body.classList.add('reader-open');
    el.addEventListener('click', (e) => { const a = e.target.closest('[data-a]'); if (!a) return; const k = a.dataset.a;
      if (k === 'close') this.close(); else if (k === 'zin') this.setZoom(this.zoom * 1.25); else if (k === 'zout') this.setZoom(this.zoom / 1.25); else if (k === 'fit') this.setZoom(1); else if (k === 'toc') this.toggleToc(); });
    const inp = $('.reader-pg input', el);
    inp.addEventListener('change', () => this.goto(parseInt(inp.value, 10) || 1));
    inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); this.goto(parseInt(inp.value, 10) || 1); $('.reader-body', el).focus(); } e.stopPropagation(); });
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
      $('.reader-msg', el).remove();
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
    s.state = 'loading'; const gen = this.gen, w = this.cssW;
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
      // invisible text on top of the picture, so you can select, copy and find words on the page
      const tl = h('div', { class: 'textLayer' }); s.el.style.setProperty('--scale-factor', String(scale)); s.el.append(tl);
      const tc = await page.getTextContent(); if (s.state !== 'done' || gen !== this.gen) return;
      window.pdfjsLib.renderTextLayer({ textContentSource: tc, container: tl, viewport: vp, textDivs: [] });
    } catch (e) { if (s.state === 'loading') s.state = null; }
  },
  free(s, force) {
    if (!s || !s.state) return;
    if (!force && Math.abs(s.i - this.cur) < 3) return;
    if (s.task) { try { s.task.cancel(); } catch (e) { /* already done */ } s.task = null; }
    s.el.textContent = ''; s.state = null;
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
    const toc = $('.reader-toc', this.el);
    if (e.key === 'Escape') { e.preventDefault(); if (!toc.hidden) this.toggleToc(false); else this.close(); return; }
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
      const b = h('button', { html: `<span>${esc(clean(it.title))}</span><span class="tabnum">${p || ''}</span>` }); if (p) b.addEventListener('click', () => { this.goto(p); this.toggleToc(false); });
      li.append(b);
      if (it.items && it.items.length) {
        const sub = h('ol');
        for (const c of it.items) { const pc = await pageOf(c.dest); const bb = h('button', { html: `<span>${esc(clean(c.title))}</span><span class="tabnum">${pc || ''}</span>` }); if (pc) bb.addEventListener('click', () => { this.goto(pc); this.toggleToc(false); }); sub.append(h('li', {}, bb)); }
        li.append(sub);
      }
      ol.append(li);
    }
    if (this.el) $('.reader-top [data-a="toc"]', this.el).hidden = false;
  },
  toggleToc(on) {
    const t = $('.reader-toc', this.el), btn = $('.reader-top [data-a="toc"]', this.el); if (!t) return;
    const show = on == null ? t.hidden : on; t.hidden = !show; btn.setAttribute('aria-expanded', String(show));
    if (show) { const b = $('button:not([data-a])', t); b && b.focus(); } else btn.focus();
  },
  close() {
    if (!this.el) return;
    clearTimeout(this.st);
    if (this.n) { S.lib.pos = S.lib.pos || {}; S.lib.pos[this.id] = { p: this.cur, n: this.n, t: Date.now() }; save({ local: true }); }
    this.gen++; if (this.io) this.io.disconnect(); this.io = null;
    document.removeEventListener('keydown', this.onKey); window.removeEventListener('resize', this.onResize);
    if (this.pdf) { try { this.pdf.destroy(); } catch (e) { /* ignore */ } } this.pdf = null;
    this.el.remove(); this.el = null; this.slots = []; this.n = 0; this.saved = false; document.body.classList.remove('reader-open');
    if (App.view === 'library') App.refresh();
    const o = this.opener; this.opener = null; if (o && o.isConnected) o.focus();
  },
};
