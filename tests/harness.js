// Loads the app's simulator sources into Node (no browser, no DOM) so the physics
// and the robot simulators can be tested headless. Paths are relative to this repo.
const fs = require('fs'), path = require('path');
const SRC = path.join(__dirname, '..', 'src');
const read = (f) => fs.readFileSync(path.join(SRC, f), 'utf8');

// Pure modules: kinematics + physics engine (+ optional walking controller).
function loadModules(files, names) {
  const code = files.map(read).join('\n;\n');
  return new Function(code + `\nreturn {${names.join(',')}};`)();
}

// The four SnoBots simulators with their ROS bus, minus everything that touches the page.
function loadSims() {
  const files = ['sno_kin.js', 'sno_phys.js', 'sno_walk.js', 'sno_humanoid.js', 'sno_robots.js', 'sno_car.js', 'sno_defs.js', 'sno_fix.js'];
  const code = files.map(read).join('\n;\n');
  const lab = read('lab.js'), core = read('sno_core.js');
  const busSrc = lab.slice(lab.indexOf('const QOS_DEF'), lab.indexOf('const Bus = makeBus();'));
  const simSrc = core.slice(core.indexOf('class SnoSim {'), core.indexOf('/* ------------------------------ generic panels'));
  const stub = `const Sound={play(){}};const S={sno:{m:{},code:{},tpl:{},actions:{},hist:[],fix:{}},gears:0,streak:0};let XP=0;
    function save(){}function toast(){}function esc(s){return String(s)}function confetti(){}function addXP(x){XP+=x}
    function bumpStreak(){return false}function checkBadges(){}const App={renderStats(){}};
    const SnoUI={refreshTab(){},renderTools(){},renderMissionBar(){},renderTabs(){},renderTab(){},focusTab(){},root:null,cur:null};const Py={host:null};
    function md(s){return String(s)}function plain(s){return String(s)}function icon(){return ''}function h(){return {}}function $(){return null}function $$(){return []}
    function stampNow(){return {sec:0,nanosec:0}}function stampSec(){return '0.000000000'}function zeros36(){return new Array(36).fill(0)}class TermPanel{constructor(){}}
    function snoPhysViz(){} const document=undefined; const IFACES={};\n`;
  return new Function(stub + busSrc + '\n' + simSrc + '\n' + code +
    '\nreturn { SNO_DEFS, K3, SnoFix, FIX_TICKETS, S, getXP: () => XP };')();
}

// Make a simulator ready to step, the way the app does when you open its page.
function makeSim(L, id) {
  const sim = L.SNO_DEFS[id].make(L.SNO_DEFS[id]);
  sim.started = true; sim.setupRos();
  return sim;
}

// Tiny test runner: test(name, fn) collects; run() prints PASS/FAIL and sets the exit code.
const tests = [];
const test = (name, fn) => tests.push({ name, fn });
const assert = (cond, msg) => { if (!cond) throw new Error(msg); };
async function run() {
  let failed = 0;
  for (const t of tests) {
    const t0 = Date.now();
    try { const note = await t.fn(); console.log(`PASS  ${t.name}${note ? '  (' + note + ')' : ''}  ${Date.now() - t0} ms`); }
    catch (e) { failed++; console.log(`FAIL  ${t.name}\n      ${e.message}`); }
  }
  console.log(failed ? `\n${failed} of ${tests.length} failed` : `\nall ${tests.length} passed`);
  process.exitCode = failed ? 1 : 0;
}
module.exports = { loadModules, loadSims, makeSim, test, assert, run };
