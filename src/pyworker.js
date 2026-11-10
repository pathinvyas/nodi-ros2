/* Web Worker: hosts Pyodide and the learner's rclpy node, isolated from the page so a stuck loop can be stopped. */
let py = null, runSeq = 0;
const fn = {};
const post = (m, transfer) => postMessage(m, transfer || []);
// packages the learner's imports need: anything Pyodide ships (numpy, opencv-python for cv2…), plus numpy for cv_bridge
async function loadFor(code) {
  const say = (s) => { if (/^Loading /.test(s)) post({ type: 'busy', text: s + ' (first run only)…' }); };
  const err = (s) => post({ type: 'out', text: s, err: true });
  if (/^\s*(import|from)\s+[^\n#]*\b(cv_bridge|cv2)\b/m.test(code) && !py.loadedPackages.numpy) await py.loadPackage('numpy', { messageCallback: say, errorCallback: err });
  await py.loadPackagesFromImports(code, { messageCallback: say, errorCallback: err });
  if (/^\s*(import|from)\s+[^\n#]*\bcv2\b/m.test(code)) { post({ type: 'busy', text: 'Starting OpenCV…' }); fn._lab_patch_cv2(); }
}
self.onmessage = async (ev) => {
  const m = ev.data || {};
  try {
    if (m.type === 'init') {
      post({ type: 'status', text: 'Downloading Python…' });
      importScripts(m.base + 'pyodide.js');
      post({ type: 'status', text: 'Starting Python…' });
      py = await loadPyodide({
        indexURL: m.base,
        stdout: (s) => post({ type: 'out', text: s }),
        stderr: (s) => post({ type: 'out', text: s, err: true }),
      });
      py.registerJsModule('labbridge', {
        post: (s) => post({ type: 'bridge', json: s }),
        // a message with a byte array (an image): the bytes travel next to the JSON instead of inside it
        post_bin: (s, b) => { const c = new Uint8Array(b); post({ type: 'bridge', json: s, bin: c }, [c.buffer]); },
      });
      py.runPython(m.shim);
      ['_lab_tick', '_lab_clock', '_lab_deliver', '_lab_run', '_lab_stop', '_lab_set_param', '_lab_patch_cv2'].forEach((k) => { fn[k] = py.globals.get(k); });
      post({ type: 'ready', version: py.version });
      let n = 0;
      setInterval(() => {
        try { fn._lab_tick(); } catch (e) { post({ type: 'fatal', text: String((e && e.message) || e) }); }
        if (++n % 10 === 0) post({ type: 'beat' });
      }, 20);
      return;
    }
    if (!py) return;
    if (m.type === 'run') {
      const my = ++runSeq;
      try { await loadFor(m.code); } catch (e) { post({ type: 'idle' }); post({ type: 'out', text: `Could not load a package your code imports: ${(e && e.message) || e}`, err: true }); }
      post({ type: 'idle' });
      if (my !== runSeq) return; // stopped or restarted while packages were loading
      if (typeof m.simT === 'number') fn._lab_run(m.code, m.simT); else fn._lab_run(m.code);
    }
    else if (m.type === 'clock') fn._lab_clock(m.t);
    else if (m.type === 'msg') { if (m.bin) fn._lab_deliver(m.topic, m.msgType, m.json, m.bin); else fn._lab_deliver(m.topic, m.msgType, m.json); }
    else if (m.type === 'stop') { runSeq++; fn._lab_stop(); }
    else if (m.type === 'param') post({ type: 'paramResult', id: m.id, json: fn._lab_set_param(m.node, m.name, m.json, m.ptype || null) });
  } catch (e) {
    post({ type: 'fatal', text: String((e && e.message) || e) });
  }
};
