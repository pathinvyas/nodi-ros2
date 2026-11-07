// Checks the library list: every shipped book is a real PDF in library/ with its cover,
// and "bring your own copy" books (copyrighted) never ship a file.
const fs = require('fs'), path = require('path');
const { test, assert, run } = require('./harness');
const ROOT = path.join(__dirname, '..');
const src = fs.readFileSync(path.join(ROOT, 'src', 'library.js'), 'utf8');
const LIBRARY = new Function(src.slice(src.indexOf('const LIBRARY'), src.indexOf('const PDFJS_BASE')) + '\nreturn LIBRARY;')();

test('every book has an id, a title and a description, and ids are unique', () => {
  const ids = new Set();
  for (const b of LIBRARY) {
    assert(b.id && /^[a-z0-9-]+$/.test(b.id), `bad id ${b.id}`);
    assert(!ids.has(b.id), `duplicate id ${b.id}`); ids.add(b.id);
    assert(b.title && b.blurb, `${b.id} needs a title and a blurb`);
  }
  return `${LIBRARY.length} books`;
});

test('shipped books are PDFs in library/ with the page count and size the shelf shows', () => {
  for (const b of LIBRARY.filter(b => !b.own)) {
    assert(b.file && b.file.startsWith('library/'), `${b.id} has no library/ file`);
    const f = path.join(ROOT, b.file); assert(fs.existsSync(f), `${b.file} is missing`);
    const buf = fs.readFileSync(f);
    assert(buf.subarray(0, 5).toString('latin1') === '%PDF-', `${b.file} is not a PDF`);
    if (b.size) assert(b.size === buf.length, `${b.id}: size ${b.size} but the file is ${buf.length} bytes`);
    const pages = (buf.toString('latin1').match(/\/Type\s*\/Page(?![s\w])/g) || []).length;
    if (b.pages && pages) assert(b.pages === pages, `${b.id}: ${b.pages} pages listed, the PDF has ${pages}`);
  }
});

test('covers exist and copyrighted books ship no file', () => {
  for (const b of LIBRARY) {
    const m = /^LIBCOVER:(.+)$/.exec(b.cover || '');
    if (m) assert(fs.existsSync(path.join(ROOT, 'library', 'covers', m[1])), `${b.id}: cover ${m[1]} missing`);
    if (b.own) assert(!b.file, `${b.id} is bring-your-own-copy, so it must not ship a file`);
  }
  const listed = new Set(LIBRARY.filter(b => b.file).map(b => path.basename(b.file)));
  const extra = fs.readdirSync(path.join(ROOT, 'library')).filter(f => f.endsWith('.pdf') && !listed.has(f));
  assert(!extra.length, `PDFs in library/ that no book lists: ${extra.join(', ')}`);
});

run();
