'use strict';
/* ============================== core helpers ============================== */
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
const REDUCED = (() => { try { return matchMedia('(prefers-reduced-motion: reduce)').matches; } catch (e) { return false; } })();
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const lerp = (a, b, t) => a + (b - a) * t;
const rand = (a, b) => a + Math.random() * (b - a);
function shuffle(arr) { const a = arr.slice(); for (let i = a.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [a[i], a[j]] = [a[j], a[i]]; } return a; }
function sample(arr, n) { return shuffle(arr).slice(0, n); }
const esc = s => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
/* inline markup used in course text: **bold**, `code`, ==highlight== */
function md(s) {
  return esc(s)
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    .replace(/==([^=]+)==/g, '<mark>$1</mark>');
}
const plain = s => String(s).replace(/`|\*\*|==/g, '');
function h(tag, props, ...kids) {
  const el = document.createElement(tag);
  if (props) for (const [k, v] of Object.entries(props)) {
    if (v == null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'html') el.innerHTML = v;
    else if (k === 'text') el.textContent = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const kid of kids.flat()) if (kid != null && kid !== false) el.append(kid.nodeType ? kid : document.createTextNode(String(kid)));
  return el;
}
function dayKey(offset = 0) {
  const d = new Date(); d.setDate(d.getDate() + offset);
  return d.getFullYear() + '-' + String(d.getMonth() + 1).padStart(2, '0') + '-' + String(d.getDate()).padStart(2, '0');
}
const sleep = ms => new Promise(r => setTimeout(r, ms));
function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

/* ============================== icons ============================== */
const GEAR_PATH = (() => {
  const pts = []; const teeth = 8;
  for (let i = 0; i < teeth * 4; i++) {
    const a = (i / (teeth * 4)) * Math.PI * 2 - Math.PI / 2;
    const r = (i % 4 === 0 || i % 4 === 1) ? 10.5 : 7.8;
    pts.push((12 + r * Math.cos(a)).toFixed(2) + ' ' + (12 + r * Math.sin(a)).toFixed(2));
  }
  return 'M' + pts.join('L') + 'Z';
})();
const ICONS = {
  learn: '<path d="M3 11l9-7 9 7"/><path d="M5.5 9.5V20h13V9.5"/><path d="M10 20v-5.5h4V20"/>',
  lab: '<rect x="5" y="8" width="14" height="11" rx="3.5"/><path d="M12 8V5"/><circle cx="12" cy="4" r="1.4"/><circle cx="9.5" cy="13.5" r="1.3" fill="currentColor"/><circle cx="14.5" cy="13.5" r="1.3" fill="currentColor"/><path d="M2.5 12v4M21.5 12v4"/>',
  practice: '<path d="M6.5 6.5v11M17.5 6.5v11M3 10v4M21 10v4M6.5 12h11"/>',
  notebook: '<path d="M6 3h11a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2H6z"/><path d="M6 3v18M10 8h5M10 12h5"/>',
  profile: '<circle cx="12" cy="8" r="4"/><path d="M4 21c1.5-4 4.5-6 8-6s6.5 2 8 6"/>',
  ask: '<path d="M4 5h16v11H9.5L4 20z"/><path d="M8 9h8M8 12.5h5"/>',
  bolt: '<path d="M13.5 2L4.5 14h6.5l-1 8 9-12.5h-6.5z" fill="currentColor" stroke="none"/>',
  gear: '<path d="' + GEAR_PATH + '" fill="currentColor" stroke="none"/><circle cx="12" cy="12" r="3.2" style="fill:var(--surface)" stroke="none"/>',
  battery: '<rect x="2" y="7" width="17.5" height="10" rx="2.6"/><path d="M22 10.5v3"/><rect x="4.6" y="9.6" width="12.3" height="4.8" rx="1" fill="currentColor" stroke="none"/>',
  star: '<path d="M12 2.8l2.85 5.9 6.45.85-4.75 4.45 1.2 6.4L12 17.3l-5.75 3.1 1.2-6.4L2.7 9.55l6.45-.85z" fill="currentColor" stroke="none"/>',
  check: '<path d="M5 12.5l4.6 4.6L19.2 7.5" stroke-width="3"/>',
  x: '<path d="M6 6l12 12M18 6L6 18" stroke-width="2.6"/>',
  lock: '<rect x="5" y="10.5" width="14" height="10" rx="2.6" fill="currentColor" stroke="none"/><path d="M8 10.5V8a4 4 0 0 1 8 0v2.5"/>',
  play: '<path d="M8 5.2v13.6l11-6.8z" fill="currentColor" stroke="none"/>',
  pause: '<path d="M8.5 5.5v13M15.5 5.5v13" stroke-width="3"/>',
  replay: '<path d="M4.5 12a7.5 7.5 0 1 0 2.4-5.5"/><path d="M4.5 4v4.5H9"/>',
  sound: '<path d="M4 9.2v5.6h3.6L12.5 19V5L7.6 9.2z" fill="currentColor" stroke="none"/><path d="M16 9a4.5 4.5 0 0 1 0 6M18.6 6.5a8 8 0 0 1 0 11"/>',
  mute: '<path d="M4 9.2v5.6h3.6L12.5 19V5L7.6 9.2z" fill="currentColor" stroke="none"/><path d="M16.5 9.5l5 5M21.5 9.5l-5 5"/>',
  arrowR: '<path d="M5 12h14M13 6l6 6-6 6"/>',
  arrowL: '<path d="M19 12H5M11 6l-6 6 6 6"/>',
  arrowD: '<path d="M12 5v14M6 13l6 6 6-6"/>',
  arrowU: '<path d="M12 19V5M6 11l6-6 6 6"/>',
  chip: '<rect x="6" y="6" width="12" height="12" rx="2.2"/><path d="M9.5 3v3M14.5 3v3M9.5 18v3M14.5 18v3M3 9.5h3M3 14.5h3M18 9.5h3M18 14.5h3"/><circle cx="12" cy="12" r="2" fill="currentColor"/>',
  trophy: '<path d="M8 4h8v5a4 4 0 0 1-8 0z"/><path d="M8 6H5a3 3 0 0 0 3 4.5M16 6h3a3 3 0 0 1-3 4.5M12 13v4M8.5 20.5h7M10 17h4"/>',
  terminal: '<rect x="3" y="4" width="18" height="16" rx="2.6"/><path d="M7 9.5l3 2.5-3 2.5M12.5 15h4.5"/>',
  code: '<path d="M8 7l-5 5 5 5M16 7l5 5-5 5M14 4l-4 16"/>',
  graph: '<circle cx="6" cy="6" r="2.4"/><circle cx="18" cy="8" r="2.4"/><circle cx="7" cy="18" r="2.4"/><circle cx="18" cy="18" r="2.4"/><path d="M8.3 6.4l7.4 1.2M6.4 8.4l.4 7.2M9.4 18h6.2M18 10.4v5.2"/>',
  drive: '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="3.6" fill="currentColor" stroke="none"/>',
  topics: '<path d="M4 7h16M4 12h10M4 17h13"/>',
  flag: '<path d="M5.5 21V4M5.5 4.5h11l-2.2 4 2.2 4h-11"/>',
  book: '<path d="M3.5 5h6a2.5 2.5 0 0 1 2.5 2.5V20a2 2 0 0 0-2-2H3.5z"/><path d="M20.5 5h-6A2.5 2.5 0 0 0 12 7.5V20a2 2 0 0 1 2-2h6.5z"/>',
  copy: '<rect x="8.5" y="8.5" width="11.5" height="11.5" rx="2"/><path d="M15.5 8.5V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v9.5a1 1 0 0 0 1 1h3.5"/>',
  search: '<circle cx="11" cy="11" r="6.5"/><path d="M16 16l4.5 4.5"/>',
  sliders: '<path d="M4 6h9M17 6h3M4 12h3M11 12h9M4 18h11M19 18h1"/><circle cx="15" cy="6" r="2"/><circle cx="9" cy="12" r="2"/><circle cx="17" cy="18" r="2"/>',
  target: '<circle cx="12" cy="12" r="8.5"/><circle cx="12" cy="12" r="4.5"/><circle cx="12" cy="12" r="1.2" fill="currentColor"/>',
  cards: '<rect x="3" y="7.5" width="13.5" height="12" rx="2"/><path d="M7.5 4.5H19a2 2 0 0 1 2 2V16"/>',
  node: '<circle cx="6" cy="12" r="3"/><circle cx="18" cy="6" r="3"/><circle cx="18" cy="18" r="3"/><path d="M8.7 10.6l6.6-3.3M8.7 13.4l6.6 3.3"/>',
  msg: '<rect x="3" y="6" width="18" height="12" rx="2.2"/><path d="M3.8 7.2L12 13l8.2-5.8"/>',
  clock: '<circle cx="12" cy="12" r="8.6"/><path d="M12 7.4V12l3.2 2"/>',
  radio: '<path d="M12 11v10M9 21h6"/><circle cx="12" cy="9" r="2"/><path d="M8.2 5.6a5 5 0 0 0 0 6.8M15.8 5.6a5 5 0 0 1 0 6.8M5.6 3a8.6 8.6 0 0 0 0 12M18.4 3a8.6 8.6 0 0 1 0 12"/>',
  bell: '<path d="M3.5 18.5h17M6 18.5a6 6 0 0 1 12 0M12 9.5V7.5M10 7.5h4"/>',
  truck: '<path d="M2.5 6.5h11.5v9.5H2.5zM14 10h4.2l3.3 3.2V16H14z"/><circle cx="7" cy="17.5" r="1.9"/><circle cx="17.5" cy="17.5" r="1.9"/>',
  knob: '<circle cx="12" cy="12" r="8.2"/><path d="M12 12l4.2-4.2"/><path d="M5 19.5l1.4-1.4M19 19.5l-1.4-1.4"/>',
  shield: '<path d="M12 3l7.5 3v5.2c0 4.9-3.2 8.4-7.5 9.8-4.3-1.4-7.5-4.9-7.5-9.8V6z"/>',
  camera: '<rect x="3" y="7" width="18" height="12.5" rx="2.6"/><circle cx="12" cy="13.2" r="3.6"/><path d="M8 7l1.6-2.6h4.8L16 7"/>',
  layers: '<path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/>',
  pkg: '<path d="M3.5 7.5L12 3l8.5 4.5v9L12 21l-8.5-4.5z"/><path d="M3.5 7.5L12 12l8.5-4.5M12 12v9"/>',
  world: '<circle cx="12" cy="12" r="9"/><path d="M3 12h18M12 3c3.2 3.6 3.2 14.4 0 18M12 3c-3.2 3.6-3.2 14.4 0 18"/>',
  axes: '<path d="M5.5 18.5V5M5.5 18.5H19M5.5 18.5l8.5-8.5"/>',
  bug: '<rect x="7" y="8" width="10" height="12.5" rx="5"/><path d="M12 8v12.5M7 13H3M21 13h-4M7.5 17.5l-3 2M16.5 17.5l3 2M8.5 8.5L6.5 5.5M15.5 8.5l2-3"/>',
  sparkle: '<path d="M12 2.5l2 5.7 5.8 2-5.8 2L12 18l-2-5.8-5.8-2 5.8-2z" fill="currentColor" stroke="none"/><path d="M19 15.5l.8 2.2 2.2.8-2.2.8-.8 2.2-.8-2.2-2.2-.8 2.2-.8z" fill="currentColor" stroke="none"/>',
  warn: '<path d="M12 3.5l9.5 16.5h-19z"/><path d="M12 10v4.2M12 17.2v.1"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  minus: '<path d="M5 12h14"/>',
  toc: '<path d="M9 6h11M9 12h11M9 18h11"/><circle cx="4.5" cy="6" r="1.2"/><circle cx="4.5" cy="12" r="1.2"/><circle cx="4.5" cy="18" r="1.2"/>',
  external: '<path d="M14 4h6v6M20 4l-8.5 8.5"/><path d="M18 14v4.5a1.5 1.5 0 0 1-1.5 1.5h-11A1.5 1.5 0 0 1 4 18.5v-11A1.5 1.5 0 0 1 5.5 6H10"/>',
  stop: '<rect x="6.5" y="6.5" width="11" height="11" rx="2" fill="currentColor" stroke="none"/>',
  send: '<path d="M4 12l16-8-6 16-2.6-6.4z"/>',
  usb: '<path d="M12 3v13.5M9.2 5.8L12 3l2.8 2.8M7.5 10.5v2.2H12M16.5 8.5v4.2H12"/><circle cx="12" cy="18.8" r="2"/>',
  wheel: '<circle cx="12" cy="12" r="8.4"/><circle cx="12" cy="12" r="2.4"/><path d="M12 3.6v6M12 14.4v6M3.6 12h6M14.4 12h6"/>',
  launch: '<path d="M12 3c3.5 2.2 5 5.5 5 9.5l-2.5 3h-5L7 12.5C7 8.5 8.5 5.2 12 3z"/><circle cx="12" cy="9.5" r="1.8"/><path d="M9.5 15.5L8 20l4-2 4 2-1.5-4.5"/>',
  tf: '<path d="M6 18V8M6 18h10M6 18l-3 3"/><path d="M14 10V4M14 10h6M14 10l-2.5 2.5" opacity=".6"/>',
  brain: '<path d="M9 4a3 3 0 0 0-3 3 3 3 0 0 0-2 5 3 3 0 0 0 2 5 3 3 0 0 0 3 3h1V4z"/><path d="M15 4a3 3 0 0 1 3 3 3 3 0 0 1 2 5 3 3 0 0 1-2 5 3 3 0 0 1-3 3h-1V4z"/>',
  python: '<path d="M7 8h10M7 12h6M7 16h8"/><rect x="3" y="4" width="18" height="16" rx="3"/>',
  tools: '<path d="M14.6 6.4a4 4 0 0 0 5 5L12.8 18.2a2.4 2.4 0 0 1-3.4-3.4z"/><path d="M5 4l3 3M4 9l2-2 3 3-2 2z"/>',
  ticket: '<path d="M3.5 7.5h17v3a2 2 0 0 0 0 3v3h-17v-3a2 2 0 0 0 0-3z"/><path d="M14 7.5v9" stroke-dasharray="2 2"/>',
  map: '<path d="M3.5 6l5.5-2 6 2 5.5-2v14l-5.5 2-6-2-5.5 2z"/><path d="M9 4v14M15 6v14"/>',
  snobots: '<rect x="8" y="2.5" width="8" height="6" rx="2"/><circle cx="12" cy="5.5" r="1.1" fill="currentColor"/><rect x="7.5" y="9.5" width="9" height="7" rx="2"/><path d="M7.5 11l-3 4M16.5 11l3 4M10 16.5v5M14 16.5v5M8.5 21.5h3M12.5 21.5h3"/>',
};
function icon(name, cls = '') {
  return `<svg class="ico ico-stroke ${cls}" viewBox="0 0 24 24" aria-hidden="true">${ICONS[name] || ''}</svg>`;
}

/* ============================== mascot: Nodi ============================== */
function nodi(mood = 'idle', cls = 'bob') {
  const K = '#1C2B25', G = mood === 'sad' ? '#86B8FF' : '#5EF2D6';
  let eyes, mouth, brows = '';
  switch (mood) {
    case 'happy':
      eyes = `<path d="M40 47q7-9 14 0M66 47q7-9 14 0" stroke="${G}" stroke-width="4" fill="none" stroke-linecap="round"/>`;
      mouth = `<path d="M50 53q10 10 20 0z" fill="${G}"/>`; break;
    case 'sad':
      eyes = `<ellipse cx="47" cy="48" rx="5" ry="5.5" fill="${G}"/><ellipse cx="73" cy="48" rx="5" ry="5.5" fill="${G}"/>`;
      brows = `<path d="M40 38.5l12 3.5M80 38.5l-12 3.5" stroke="${G}" stroke-width="3" stroke-linecap="round"/>`;
      mouth = `<path d="M53 60q7-5.5 14 0" stroke="${G}" stroke-width="3" fill="none" stroke-linecap="round"/>`; break;
    case 'wow':
      eyes = `<circle cx="47" cy="45" r="7.5" fill="${G}"/><circle cx="73" cy="45" r="7.5" fill="${G}"/><circle cx="49.5" cy="42.5" r="2.4" fill="#173129"/><circle cx="75.5" cy="42.5" r="2.4" fill="#173129"/>`;
      mouth = `<circle cx="60" cy="57" r="3.8" fill="${G}"/>`; break;
    case 'think':
      eyes = `<ellipse cx="49" cy="43" rx="5" ry="6.5" fill="${G}"/><ellipse cx="75" cy="43" rx="5" ry="6.5" fill="${G}"/>`;
      mouth = `<path d="M54 57h12" stroke="${G}" stroke-width="3" stroke-linecap="round"/>`; break;
    default:
      eyes = `<ellipse cx="47" cy="45" rx="5.5" ry="7" fill="${G}"/><ellipse cx="73" cy="45" rx="5.5" ry="7" fill="${G}"/>`;
      mouth = `<path d="M53 55q7 5 14 0" stroke="${G}" stroke-width="3" fill="none" stroke-linecap="round"/>`;
  }
  return `<svg class="nodi ${cls}" viewBox="0 0 120 140" aria-hidden="true">
  <ellipse cx="60" cy="134" rx="30" ry="4.5" fill="rgba(0,0,0,.13)"/>
  <g class="body-g">
    <g class="arm-l"><rect x="9" y="79" width="23" height="11" rx="5.5" fill="#E3EAE6" stroke="${K}" stroke-width="2.6" transform="rotate(-35 30 84)"/></g>
    <g class="arm-r"><rect x="88" y="79" width="23" height="11" rx="5.5" fill="#E3EAE6" stroke="${K}" stroke-width="2.6" transform="rotate(35 90 84)"/></g>
    <rect x="34" y="113" width="52" height="15" rx="7.5" fill="#3A4A43" stroke="${K}" stroke-width="2.6"/>
    <circle cx="45" cy="120.5" r="3" fill="#97A8A0"/><circle cx="60" cy="120.5" r="3" fill="#97A8A0"/><circle cx="75" cy="120.5" r="3" fill="#97A8A0"/>
    <rect x="30" y="71" width="60" height="45" rx="15" fill="#F4F8F6" stroke="${K}" stroke-width="3"/>
    <circle cx="60" cy="93.5" r="9.5" fill="#FDE6D8" stroke="#F26419" stroke-width="2.6"/>
    <circle cx="60" cy="93.5" r="3.4" fill="#F26419"/>
    <path d="M60 22V10" stroke="${K}" stroke-width="3" stroke-linecap="round"/>
    <circle class="ant-tip" cx="60" cy="8.5" r="5.8" fill="#F26419" stroke="${K}" stroke-width="2.6"/>
    <rect x="52" y="65" width="16" height="9" rx="3" fill="#C9D4CF" stroke="${K}" stroke-width="2.6"/>
    <rect x="9.5" y="35" width="11" height="22" rx="5.5" fill="#0E9384" stroke="${K}" stroke-width="2.6"/>
    <rect x="99.5" y="35" width="11" height="22" rx="5.5" fill="#0E9384" stroke="${K}" stroke-width="2.6"/>
    <rect x="18" y="20" width="84" height="51" rx="20" fill="#F4F8F6" stroke="${K}" stroke-width="3"/>
    <rect x="26.5" y="27.5" width="67" height="36" rx="13" fill="#173129"/>
    <g class="eyes">${eyes}</g>${brows}${mouth}
    <circle cx="34.5" cy="55" r="3.4" fill="#F26419" opacity=".55"/><circle cx="85.5" cy="55" r="3.4" fill="#F26419" opacity=".55"/>
  </g></svg>`;
}

/* ============================== code highlighter ============================== */
const HLR = (() => {
  const str = /f?(?:"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*')/;
  const num = /\b\d+(?:\.\d+)?(?:e-?\d+)?(?:ms|s)?\b/;
  const rules = {
    py: [[/#.*/, 'c'], [str, 's'], [/\b(?:def|class|import|from|return|if|elif|else|for|while|in|not|and|or|is|None|True|False|try|except|finally|with|as|lambda|pass|raise|break|continue)\b/, 'k'], [/\b(?:self|super|print|len|min|max|abs|range|int|float|str|bool|list|round|sum)\b/, 't'], [num, 'n'], [/[A-Za-z_]\w*(?=\()/, 'f']],
    cpp: [[/\/\/.*/, 'c'], [/"(?:\\.|[^"\\])*"/, 's'], [/#\s*include|\b(?:auto|class|public|private|protected|return|const|using|namespace|template|typename|this|new|if|else|for|while|true|false|nullptr|override|virtual|struct|static|void|int|double|float|bool|size_t|char)\b/, 'k'], [/\b(?:std|rclcpp|std_msgs|example_interfaces|msg|srv)\b(?=::)/, 't'], [num, 'n'], [/[A-Za-z_]\w*(?=\s*[(<])/, 'f'], [/<[\w/.]+\.hpp>/, 's']],
    cmake: [[/#.*/, 'c'], [/"(?:\\.|[^"\\])*"/, 's'], [/\$\{[^}]+\}/, 'a'], [/\b(?:REQUIRED|TARGETS|DESTINATION|DIRECTORY)\b/, 'k'], [/[A-Za-z_]\w*(?=\()/, 'f']],
    xml: [[/<!--.*?-->/, 'c'], [/<\/?[\w:-]+|\/?>/, 'f'], [/[\w:-]+(?==)/, 'a'], [/"[^"]*"/, 's']],
    yaml: [[/#.*/, 'c'], [/^\s*[\w_]+(?=:)/, 'f'], [/"[^"]*"|'[^']*'/, 's'], [num, 'n'], [/\b(?:true|false)\b/, 'k']],
    sh: [[/#.*/, 'c'], [/"(?:\\.|[^"\\])*"|'[^']*'/, 's'], [/\b(?:ros2|colcon|source|sudo|cd|ls|rqt_graph|rqt|python3|pip)\b/, 'f'], [/\s--?[\w-]+/, 'a'], [/\$\w+/, 'k']],
    msg: [[/#.*/, 'c'], [/^---$/, 'k'], [/\b(?:bool|byte|char|float32|float64|int8|uint8|int16|uint16|int32|uint32|int64|uint64|string|wstring)\b(?:\[\d*\])?/, 't'], [/[A-Za-z_]+\/[A-Za-z_]+(?:\[\])?/, 't'], [num, 'n']],
  };
  for (const k in rules) rules[k] = rules[k].map(([re, c]) => [new RegExp(re.source, 'y'), c]);
  return rules;
})();
function hlLine(line, lang) {
  const rules = HLR[lang];
  if (!rules) return esc(line);
  let i = 0, out = '', txt = '';
  outer: while (i < line.length) {
    for (const [re, cls] of rules) {
      re.lastIndex = i;
      const m = re.exec(line);
      if (m && m[0].length) {
        if (txt) { out += esc(txt); txt = ''; }
        out += `<span class="tok-${cls}">${esc(m[0])}</span>`;
        i += m[0].length;
        continue outer;
      }
    }
    const w = /^[A-Za-z_]\w*/.exec(line.slice(i));
    if (w) { txt += w[0]; i += w[0].length; } else { txt += line[i++]; }
  }
  return out + esc(txt);
}
function codeBlock(code, lang = 'py', opts = {}) {
  const lines = String(code).replace(/\n$/, '').split('\n');
  const html = lines.map((l, i) => `<span class="ln" data-i="${i}"><span class="no">${i + 1}</span>${hlLine(l, lang) || ' '}</span>`).join('');
  return `<div class="code ${opts.cls || ''}" role="region" aria-label="${esc(opts.label || 'Code')}">${html}</div>`;
}

/* ============================== state & saving ============================== */
const STORE_KEY = 'nodi-ros2-academy-v1';
const BATTERY_MAX = 5, REGEN_MS = 10 * 60 * 1000;
function freshState() {
  return {
    v: 1, created: Date.now(), updatedAt: 0, name: '',
    xp: 0, gears: 30, battery: BATTERY_MAX, batteryTs: Date.now(),
    streak: 0, bestStreak: 0, lastDay: '', goal: 20, history: {},
    lessons: {}, cps: {}, final: null, mistakes: [], badges: {},
    stats: { answered: 0, correct: 0, bestCombo: 0, perfect: 0, practice: 0, ranCode: 0 },
    settings: { sound: true, voice: false, unlimited: false, unlockAll: false },
    lab: { missions: {}, code: null, tpl: 'avoid', hist: [] },
    sno: { m: {}, code: {}, tpl: {}, actions: {}, hist: [], fix: {} },
    lib: { pos: {} },
  };
}
function migrate(o) {
  const f = freshState();
  if (!o || typeof o !== 'object') return f;
  const s = Object.assign(f, o);
  s.stats = Object.assign(freshState().stats, o.stats || {});
  s.settings = Object.assign(freshState().settings, o.settings || {});
  s.lab = Object.assign(freshState().lab, o.lab || {});
  s.sno = Object.assign(freshState().sno, o.sno || {});
  s.lib = Object.assign(freshState().lib, o.lib || {});
  if (!Array.isArray(s.mistakes)) s.mistakes = [];
  return s;
}
let S = (() => { try { const raw = localStorage.getItem(STORE_KEY); if (raw) return migrate(JSON.parse(raw)); } catch (e) { /* storage unavailable */ } return freshState(); })();
function save(opts = {}) {
  S.updatedAt = Date.now();
  try { localStorage.setItem(STORE_KEY, JSON.stringify(S)); } catch (e) { /* ignore */ }
  if (!opts.local) Cloud.schedule();
}
/* Durable per-learner progress through the artifact's private per-user store, when the viewer offers it. */
const Cloud = {
  ref: null, busy: false, again: false, timer: null, last: '', ready: false,
  async init() {
    try {
      if (!window.claude || typeof window.claude.use !== 'function') return;
      const [db, user] = await Promise.all([window.claude.use('db'), window.claude.use('user')]);
      if (!db || !user) return;
      const id = await user.id();
      if (!id) return;
      this.ref = db.collection('data/users/' + id).doc('progress');
      const snap = await Promise.race([this.ref.get(), sleep(10000).then(() => null)]);
      this.ready = true;
      if (snap && snap.exists) {
        const d = snap.data() || {};
        if (typeof d.blob === 'string' && (d.updatedAt || 0) > (S.updatedAt || 0)) {
          S = migrate(JSON.parse(d.blob));
          this.last = d.blob;
          save({ local: true });
          if (typeof App !== 'undefined') App.refresh();
          return;
        }
      }
      if (S.updatedAt) this.schedule();
    } catch (e) { this.ref = null; }
  },
  schedule() {
    if (!this.ref || !this.ready) return;
    clearTimeout(this.timer);
    this.timer = setTimeout(() => this.push(), 1800);
  },
  async push() {
    if (!this.ref) return;
    if (this.busy) { this.again = true; return; }
    const blob = JSON.stringify(S);
    if (blob === this.last) return;
    this.busy = true;
    try { await this.ref.set({ blob, updatedAt: S.updatedAt }); this.last = blob; }
    catch (e) { if (e && (e.code === 'invalid_argument' || e.code === 'revoked' || e.code === 'quota_exceeded')) this.ref = null; }
    this.busy = false;
    if (this.again) { this.again = false; this.schedule(); }
  },
};

/* ============================== sound & voice ============================== */
const Sound = {
  ctx: null,
  ensure() {
    if (!this.ctx) { const AC = window.AudioContext || window.webkitAudioContext; if (!AC) return null; this.ctx = new AC(); }
    if (this.ctx.state === 'suspended') this.ctx.resume().catch(() => {});
    return this.ctx;
  },
  tone(f, at, dur, type = 'sine', vol = 0.12) {
    const c = this.ctx, t = c.currentTime + at, o = c.createOscillator(), g = c.createGain();
    o.type = type; o.frequency.setValueAtTime(f, t);
    g.gain.setValueAtTime(0.0001, t); g.gain.exponentialRampToValueAtTime(vol, t + 0.015); g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    o.connect(g).connect(c.destination); o.start(t); o.stop(t + dur + 0.05);
  },
  play(kind) {
    if (!S.settings.sound) return;
    try {
      if (!this.ensure()) return;
      const T = (...a) => this.tone(...a);
      if (kind === 'good') { T(660, 0, .12, 'sine', .14); T(990, .09, .24, 'sine', .12); }
      else if (kind === 'bad') { T(200, 0, .16, 'square', .045); T(150, .12, .26, 'square', .045); }
      else if (kind === 'tap') { T(540, 0, .05, 'triangle', .05); }
      else if (kind === 'done') { [523, 659, 784, 1047].forEach((f, i) => T(f, i * .11, .32, 'triangle', .11)); }
      else if (kind === 'combo') { [880, 1175, 1568].forEach((f, i) => T(f, i * .06, .15, 'sine', .08)); }
      else if (kind === 'coin') { T(988, 0, .08, 'square', .045); T(1319, .07, .22, 'square', .045); }
      else if (kind === 'beep') { T(880, 0, .08, 'sine', .06); }
      else if (kind === 'bump') { T(120, 0, .12, 'sawtooth', .05); }
    } catch (e) { /* audio refused */ }
  },
};
const Voice = {
  ok: typeof window !== 'undefined' && 'speechSynthesis' in window,
  say(text) {
    if (!this.ok) return;
    try { speechSynthesis.cancel(); const u = new SpeechSynthesisUtterance(plain(text).replace(/\//g, ' slash ').replace(/_/g, ' ')); u.rate = 1.02; u.pitch = 1.1; speechSynthesis.speak(u); } catch (e) { /* unsupported */ }
  },
  stop() { try { if (this.ok) speechSynthesis.cancel(); } catch (e) { /* ignore */ } },
};

/* ============================== feedback UI ============================== */
function toast(html, opts = {}) {
  let box = $('.toasts');
  if (!box) { box = h('div', { class: 'toasts', 'aria-live': 'polite' }); document.body.append(box); }
  const t = h('div', { class: 'toast' + (opts.cls ? ' ' + opts.cls : ''), html: (opts.mood ? nodi(opts.mood, '') : '') + `<div>${html}</div>` });
  box.append(t);
  setTimeout(() => { t.classList.add('out'); setTimeout(() => t.remove(), 320); }, opts.ms || 2800);
}
function modal({ mood = 'idle', title, body = '', actions = [], dismiss = true }) {
  const back = h('div', { class: 'modal-back', role: 'dialog', 'aria-modal': 'true', 'aria-label': title });
  const box = h('div', { class: 'modal' });
  box.innerHTML = (mood ? nodi(mood, mood === 'happy' ? 'cheer' : 'bob') : '') + `<h2>${esc(title)}</h2>` + (body ? `<p>${body}</p>` : '');
  const acts = h('div', { class: 'acts' });
  const close = () => { back.remove(); document.removeEventListener('keydown', onKey); };
  actions.forEach(a => acts.append(h('button', { class: 'btn ' + (a.cls || ''), onclick: () => { close(); a.onClick && a.onClick(); } }, a.label)));
  box.append(acts); back.append(box);
  const onKey = (e) => { if (e.key === 'Escape' && dismiss) close(); };
  document.addEventListener('keydown', onKey);
  if (dismiss) back.addEventListener('click', (e) => { if (e.target === back) close(); });
  document.body.append(back);
  setTimeout(() => { const b = $('button', acts); b && b.focus(); }, 30);
  return { close };
}
function confetti(n = 150) {
  if (REDUCED) return;
  const c = h('canvas', { class: 'confetti' }); document.body.append(c);
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  c.width = innerWidth * dpr; c.height = innerHeight * dpr;
  const ctx = c.getContext('2d'); ctx.scale(dpr, dpr);
  const cols = ['--accent', '--good', '--c-topic', '--c-msg', '--c-srv', '--c-param', '--c-node'].map(cssVar);
  const P = Array.from({ length: n }, () => ({ x: innerWidth / 2 + rand(-80, 80), y: innerHeight * .35, vx: rand(-9, 9), vy: rand(-15, -5), r: rand(0, 6.28), vr: rand(-.3, .3), w: rand(6, 11), hh: rand(4, 7), c: cols[(Math.random() * cols.length) | 0] }));
  const t0 = performance.now();
  (function frame(t) {
    const k = (t - t0) / 1000; ctx.clearRect(0, 0, innerWidth, innerHeight);
    P.forEach(p => { p.vy += .42; p.vx *= .99; p.x += p.vx; p.y += p.vy; p.r += p.vr; ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(p.r); ctx.globalAlpha = Math.max(0, 1 - k / 2.6); ctx.fillStyle = p.c; ctx.fillRect(-p.w / 2, -p.hh / 2, p.w, p.hh); ctx.restore(); });
    if (k < 2.6) requestAnimationFrame(frame); else c.remove();
  })(t0);
}
/* tiny markdown for tutor answers */
function renderMD(src) {
  const inline = (t) => esc(t).replace(/`([^`]+)`/g, '<code>$1</code>').replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>').replace(/(^|[^*])\*([^*\n]+)\*/g, '$1<em>$2</em>');
  const parts = String(src).split('```');
  let html = '';
  parts.forEach((p, i) => {
    if (i % 2) { const nl = p.indexOf('\n'); html += `<pre>${esc((nl >= 0 ? p.slice(nl + 1) : p).replace(/\n$/, ''))}</pre>`; return; }
    const lines = p.split('\n'); let list = null, para = [];
    const flushP = () => { if (para.length) { html += `<p>${inline(para.join(' '))}</p>`; para = []; } };
    const flushL = () => { if (list) { html += `<${list.t}>${list.items.map(x => `<li>${inline(x)}</li>`).join('')}</${list.t}>`; list = null; } };
    for (const raw of lines) {
      const l = raw.trim();
      const ul = /^[-*•]\s+(.*)/.exec(l), ol = /^\d+[.)]\s+(.*)/.exec(l), hd = /^#{1,4}\s+(.*)/.exec(l);
      if (ul || ol) { flushP(); const t = ul ? 'ul' : 'ol'; if (!list || list.t !== t) { flushL(); list = { t, items: [] }; } list.items.push((ul || ol)[1]); }
      else if (hd) { flushP(); flushL(); html += `<p><strong>${inline(hd[1])}</strong></p>`; }
      else if (!l) { flushP(); flushL(); }
      else { flushL(); para.push(l); }
    }
    flushP(); flushL();
  });
  return html;
}

/* ============================== progress model ============================== */
let UNITS = []; // filled by course content
function allLessons() { return UNITS.flatMap(u => u.lessons.map(l => ({ unit: u, lesson: l }))); }
function lessonById(id) { for (const u of UNITS) for (const l of u.lessons) if (l.id === id) return { unit: u, lesson: l }; return null; }
function exByKey(key) { const [lid, i] = key.split('#'); const f = lessonById(lid); return f && f.lesson.ex[+i] ? { key, ex: f.lesson.ex[+i], unit: f.unit, lesson: f.lesson } : null; }
function seqItems() {
  const items = [];
  UNITS.forEach(u => { u.lessons.forEach((l, i) => items.push({ type: 'lesson', unit: u, lesson: l, id: l.id, idx: i })); items.push({ type: 'cp', unit: u, id: 'cp-' + u.id }); });
  items.push({ type: 'final', id: 'final' });
  return items;
}
function isDone(it) { return it.type === 'lesson' ? !!S.lessons[it.id] : it.type === 'cp' ? !!S.cps[it.unit.id] : !!S.final; }
function pathInfo() {
  const items = seqItems();
  let cur = items.findIndex(it => !isDone(it));
  if (cur < 0) cur = items.length;
  return { items, cur };
}
function itemState(i, info) {
  const it = info.items[i];
  if (isDone(it)) return 'done';
  if (i === info.cur) return 'current';
  if (S.settings.unlockAll) return 'open';
  return 'locked';
}
function unitDone(uid) { const u = UNITS.find(x => x.id === uid); return !!u && u.lessons.every(l => S.lessons[l.id]) && !!S.cps[uid]; }
function unitStarted(u) { return u.lessons.some(l => S.lessons[l.id]) || S.settings.unlockAll; }
function lessonsDoneCount() { return Object.keys(S.lessons).length; }

/* ============================== rewards ============================== */
function batteryNow() {
  if (S.battery >= BATTERY_MAX) { S.batteryTs = Date.now(); return S.battery; }
  const gained = Math.floor((Date.now() - S.batteryTs) / REGEN_MS);
  if (gained > 0) { S.battery = Math.min(BATTERY_MAX, S.battery + gained); S.batteryTs += gained * REGEN_MS; if (S.battery >= BATTERY_MAX) S.batteryTs = Date.now(); }
  return S.battery;
}
function streakNow() { return (S.lastDay === dayKey(0) || S.lastDay === dayKey(-1)) ? S.streak : 0; }
function todayXP() { return S.history[dayKey(0)] || 0; }
function addXP(n) {
  const before = todayXP();
  S.xp += n; S.history[dayKey(0)] = before + n;
  const keys = Object.keys(S.history).sort(); while (keys.length > 60) delete S.history[keys.shift()];
  if (before < S.goal && before + n >= S.goal) setTimeout(() => { toast(`<b>Daily goal reached!</b><br>${S.goal} XP today. Nodi is proud of you.`, { mood: 'happy', ms: 3600 }); Sound.play('coin'); }, 1600);
}
function bumpStreak() {
  const d = dayKey(0);
  if (S.lastDay === d) return false;
  S.streak = S.lastDay === dayKey(-1) ? S.streak + 1 : 1;
  S.lastDay = d; S.bestStreak = Math.max(S.bestStreak, S.streak);
  return true;
}
const BADGES = [
  { id: 'boot', name: 'First boot', desc: 'Finish your first lesson', icon: 'play', k: 'var(--c-node)', test: () => lessonsDoneCount() >= 1 },
  { id: 'flawless', name: 'Flawless', desc: 'Finish a lesson with no mistakes', icon: 'star', k: '#E0A100', test: () => S.stats.perfect >= 1 },
  { id: 'combo', name: 'Combo x10', desc: '10 right answers in a row', icon: 'bolt', k: 'var(--accent)', test: () => S.stats.bestCombo >= 10 },
  { id: 'streak3', name: 'Warm boot', desc: 'Keep a 3-day streak', icon: 'bolt', k: '#E5532B', test: () => S.bestStreak >= 3 },
  { id: 'streak7', name: 'Always on', desc: 'Keep a 7-day streak', icon: 'clock', k: '#C2410C', test: () => S.bestStreak >= 7 },
  { id: 'talker', name: 'Talker', desc: 'Finish Unit 3: publishers and subscribers', icon: 'msg', k: 'var(--c-msg)', test: () => unitDone('u3') },
  { id: 'bilingual', name: 'Bilingual', desc: 'Finish the Python and C++ topic units', icon: 'code', k: 'var(--c-srv)', test: () => unitDone('u3') && unitDone('u4') },
  { id: 'desk', name: 'Service desk', desc: 'Finish both service units', icon: 'bell', k: '#DB3D7F', test: () => unitDone('u5') && unitDone('u6') },
  { id: 'architect', name: 'Interface architect', desc: 'Finish the custom interfaces unit', icon: 'pkg', k: '#5653D6', test: () => unitDone('u7') },
  { id: 'knobs', name: 'Knob turner', desc: 'Finish the parameters unit', icon: 'knob', k: '#EE5A24', test: () => unitDone('u8') },
  { id: 'conductor', name: 'Conductor', desc: 'Finish the launch files unit', icon: 'launch', k: '#0D93B0', test: () => unitDone('u9') },
  { id: 'frames', name: 'Frame wizard', desc: 'Finish both TF2 units', icon: 'axes', k: '#B144C8', test: () => unitDone('u10') && unitDone('u11') },
  { id: 'hardware', name: 'Hardware hacker', desc: 'Finish the microcontroller unit', icon: 'usb', k: '#D93A3A', test: () => unitDone('u12') },
  { id: 'driver', name: 'Test driver', desc: 'Complete 3 Robot Lab missions', icon: 'drive', k: 'var(--c-node)', test: () => Object.keys(S.lab.missions).length >= 3 },
  { id: 'coder', name: 'First node', desc: 'Run your own Python node in the Robot Lab', icon: 'python', k: 'var(--c-topic)', test: () => S.stats.ranCode >= 1 },
  { id: 'legend', name: 'Lab legend', desc: 'Complete every Robot Lab mission', icon: 'trophy', k: '#E0A100', test: () => typeof MISSIONS !== 'undefined' && Object.keys(S.lab.missions).length >= MISSIONS.length },
  { id: 'snobot', name: 'SnoBots rookie', desc: 'Complete a mission on any SnoBots robot', icon: 'snobots', k: '#E5532B', test: () => !!S.sno && Object.values(S.sno.m || {}).some(m => Object.keys(m).length > 0) },
  { id: 'fixer', name: 'Field engineer', desc: 'Fix a broken robot with ros2 commands', icon: 'bug', k: '#D93A3A', test: () => !!S.sno && Object.keys(S.sno.fix || {}).length >= 1 },
  { id: 'oncall', name: 'On call', desc: 'Fix every broken-robot ticket on LIMO', icon: 'tools', k: '#0D93B0', test: () => !!S.sno && typeof FIX_TICKETS !== 'undefined' && FIX_TICKETS.limo.every(t => (S.sno.fix || {})['limo:' + t.id]) },
  { id: 'fleet', name: 'Fleet commander', desc: 'Complete a mission on all four SnoBots robots', icon: 'snobots', k: '#2F6FD0', test: () => !!S.sno && ['ainex', 'r1', 'limo', 'cobot'].every(r => Object.keys((S.sno.m || {})[r] || {}).length > 0) },
  { id: 'grad', name: 'ROS 2 graduate', desc: 'Pass the final exam', icon: 'sparkle', k: 'var(--accent)', test: () => !!S.final },
];
function checkBadges(quiet) {
  const fresh = [];
  BADGES.forEach(b => { if (!S.badges[b.id] && b.test()) { S.badges[b.id] = Date.now(); fresh.push(b); } });
  if (fresh.length && !quiet) fresh.forEach((b, i) => setTimeout(() => { toast(`<b>Badge unlocked: ${esc(b.name)}</b><br>${esc(b.desc)}`, { mood: 'wow', ms: 3600 }); Sound.play('coin'); }, 900 + i * 1400));
  return fresh;
}
