/* Web Worker: hosts Pyodide and the learner's rclpy node, isolated from the page so a stuck loop can be stopped. */
let py = null;
const fn = {};
self.onmessage = async (ev) => {
  const m = ev.data || {};
  try {
    if (m.type === 'init') {
      postMessage({ type: 'status', text: 'Downloading Python…' });
      importScripts(m.base + 'pyodide.js');
      postMessage({ type: 'status', text: 'Starting Python…' });
      py = await loadPyodide({
        indexURL: m.base,
        stdout: (s) => postMessage({ type: 'out', text: s }),
        stderr: (s) => postMessage({ type: 'out', text: s, err: true }),
      });
      py.registerJsModule('labbridge', { post: (s) => postMessage({ type: 'bridge', json: s }) });
      py.runPython(m.shim);
      ['_lab_tick', '_lab_clock', '_lab_deliver', '_lab_run', '_lab_stop', '_lab_set_param'].forEach((k) => { fn[k] = py.globals.get(k); });
      postMessage({ type: 'ready', version: py.version });
      let n = 0;
      setInterval(() => {
        try { fn._lab_tick(); } catch (e) { postMessage({ type: 'fatal', text: String((e && e.message) || e) }); }
        if (++n % 10 === 0) postMessage({ type: 'beat' });
      }, 20);
      return;
    }
    if (!py) return;
    if (m.type === 'run') { if (typeof m.simT === 'number') fn._lab_run(m.code, m.simT); else fn._lab_run(m.code); }
    else if (m.type === 'clock') fn._lab_clock(m.t);
    else if (m.type === 'msg') fn._lab_deliver(m.topic, m.msgType, m.json);
    else if (m.type === 'stop') fn._lab_stop();
    else if (m.type === 'param') postMessage({ type: 'paramResult', id: m.id, json: fn._lab_set_param(m.node, m.name, m.json, m.ptype || null) });
  } catch (e) {
    postMessage({ type: 'fatal', text: String((e && e.message) || e) });
  }
};
