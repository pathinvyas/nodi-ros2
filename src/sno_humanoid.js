/* ============================== SnoBots: humanoids (Hiwonder AiNex, Unitree R1) ==============================
   Kinematic biped: a footstep planner moves the feet, the pelvis follows the commanded velocity, and a damped
   least-squares IK solves both 6-joint legs every frame so planted feet never slide. Actions are keyframes;
   a planted support foot carries the body during them. Balance is an inverted pendulum with a PD ankle
   strategy, so pushes can be recovered or end in a fall. */

/* ---------- robot descriptions (URDF-like) ---------- */
function huAinex() {
  const J = [], V = {}, M = {};
  const add = (j, vis) => { J.push(j); if (vis) V[j.child] = vis; };
  V.base_link = [{ s: 'box', size: [0.05, 0.096, 0.014], xyz: [0, 0, -0.002], m: 'alu' }, { s: 'box', size: [0.03, 0.026, 0.02], xyz: [0, 0.035, -0.004], m: 'servo' }, { s: 'box', size: [0.03, 0.026, 0.02], xyz: [0, -0.035, -0.004], m: 'servo' }];
  for (const [s, p] of [[1, 'l_'], [-1, 'r_']]) {
    add({ name: p + 'hip_yaw', parent: 'base_link', child: p + 'hip_yaw_link', xyz: [0, s * 0.035, -0.012], axis: 'z', lo: -0.9, hi: 0.9 }, [{ s: 'box', size: [0.022, 0.03, 0.006], xyz: [0, 0, -0.012], m: 'alu' }]);
    add({ name: p + 'hip_roll', parent: p + 'hip_yaw_link', child: p + 'hip_roll_link', xyz: [0, 0, -0.028], axis: 'x', lo: -0.7, hi: 0.7 }, [{ s: 'box', size: [0.034, 0.03, 0.026], xyz: [0, 0, 0.004], m: 'servo' }]);
    add({ name: p + 'hip_pitch', parent: p + 'hip_roll_link', child: p + 'thigh_link', xyz: [0, 0, 0], axis: 'y', lo: -1.9, hi: 1.0 }, [
      { s: 'box', size: [0.018, 0.03, 0.06], xyz: [0, 0, -0.042], m: 'alu' }, { s: 'box', size: [0.026, 0.005, 0.07], xyz: [0, 0.019, -0.044], m: 'alu' }, { s: 'box', size: [0.026, 0.005, 0.07], xyz: [0, -0.019, -0.044], m: 'alu' },
      { s: 'box', size: [0.032, 0.03, 0.026], xyz: [0, 0, -0.08], m: 'servo' }]);
    add({ name: p + 'knee', parent: p + 'thigh_link', child: p + 'shin_link', xyz: [0, 0, -0.085], axis: 'y', lo: -0.05, hi: 2.4 }, [
      { s: 'box', size: [0.02, 0.03, 0.056], xyz: [0, 0, -0.04], m: 'alu' }, { s: 'box', size: [0.024, 0.005, 0.064], xyz: [0, 0.018, -0.042], m: 'alu' }, { s: 'box', size: [0.024, 0.005, 0.064], xyz: [0, -0.018, -0.042], m: 'alu' },
      { s: 'box', size: [0.032, 0.03, 0.026], xyz: [0, 0, -0.077], m: 'servo' }]);
    add({ name: p + 'ank_pitch', parent: p + 'shin_link', child: p + 'ankle_link', xyz: [0, 0, -0.085], axis: 'y', lo: -1.5, hi: 1.5 }, [{ s: 'box', size: [0.026, 0.03, 0.018], xyz: [0, 0, -0.006], m: 'servo' }]);
    add({ name: p + 'ank_roll', parent: p + 'ankle_link', child: p + 'foot_link', xyz: [0, 0, 0], axis: 'x', lo: -0.7, hi: 0.7 }, [
      { s: 'box', size: [0.03, 0.03, 0.016], xyz: [0, 0, -0.016], m: 'alu' }, { s: 'box', size: [0.11, 0.064, 0.007], xyz: [0.008, 0, -0.0265], m: 'alu' }, { s: 'box', size: [0.106, 0.06, 0.003], xyz: [0.008, 0, -0.0285], m: 'rubber' }]);
    add({ name: p + 'sole_joint', parent: p + 'foot_link', child: p + 'sole_link', type: 'fixed', xyz: [0.008, 0, -0.03] });
    add({ name: p + 'sho_pitch', parent: 'torso_link', child: p + 'shoulder_link', xyz: [0, s * 0.072, 0.098], axis: 'y', lo: -3.1, hi: 3.1 }, [{ s: 'box', size: [0.026, 0.012, 0.03], xyz: [0, s * 0.006, 0], m: 'alu' }]);
    add({ name: p + 'sho_roll', parent: p + 'shoulder_link', child: p + 'upper_arm_link', xyz: [0, s * 0.018, 0], axis: 'x', lo: s > 0 ? -0.3 : -1.9, hi: s > 0 ? 1.9 : 0.3 }, [
      { s: 'box', size: [0.03, 0.026, 0.03], xyz: [0, s * 0.004, -0.006], m: 'servo' }, { s: 'box', size: [0.02, 0.022, 0.048], xyz: [0, s * 0.004, -0.038], m: 'alu' }, { s: 'box', size: [0.028, 0.026, 0.026], xyz: [0, s * 0.004, -0.066], m: 'servo' }]);
    add({ name: p + 'el_pitch', parent: p + 'upper_arm_link', child: p + 'forearm_link', xyz: [0, s * 0.004, -0.07], axis: 'y', lo: -2.2, hi: 0.2 }, [{ s: 'box', size: [0.018, 0.02, 0.036], xyz: [0, 0, -0.02], m: 'alu' }]);
    add({ name: p + 'el_yaw', parent: p + 'forearm_link', child: p + 'wrist_link', xyz: [0, 0, -0.036], axis: 'z', lo: -1.6, hi: 1.6 }, [{ s: 'box', size: [0.024, 0.026, 0.024], xyz: [0, 0, -0.012], m: 'servo' }, { s: 'box', size: [0.008, 0.022, 0.036], xyz: [-0.008, 0, -0.04], m: 'alu' }]);
    add({ name: p + 'gripper', parent: p + 'wrist_link', child: p + 'finger_link', xyz: [0.004, 0, -0.026], axis: '-y', lo: 0, hi: 0.9 }, [{ s: 'box', size: [0.006, 0.02, 0.034], xyz: [0.002, 0, -0.017], m: 'alu' }]);
  }
  add({ name: 'torso_joint', parent: 'base_link', child: 'torso_link', type: 'fixed', xyz: [0, 0, 0] }, [
    { s: 'box', size: [0.068, 0.1, 0.1], xyz: [0, 0, 0.062], m: 'shell' }, { s: 'box', size: [0.006, 0.07, 0.054], xyz: [0.035, 0, 0.068], m: 'accent' },
    { s: 'cyl', size: [0.008, 0.004], ax: 'x', xyz: [0.039, 0, 0.083], m: 'glow' }, { s: 'box', size: [0.03, 0.084, 0.078], xyz: [-0.046, 0, 0.066], m: 'dark' },
    { s: 'box', size: [0.03, 0.03, 0.034], xyz: [0, 0.058, 0.098], m: 'servo' }, { s: 'box', size: [0.03, 0.03, 0.034], xyz: [0, -0.058, 0.098], m: 'servo' }, { s: 'box', size: [0.03, 0.034, 0.014], xyz: [0, 0, 0.117], m: 'servo' }]);
  add({ name: 'head_pan', parent: 'torso_link', child: 'neck_link', xyz: [0, 0, 0.124], axis: 'z', lo: -1.5, hi: 1.5 }, [{ s: 'box', size: [0.02, 0.026, 0.022], xyz: [0, 0, 0.01], m: 'alu' }]);
  add({ name: 'head_tilt', parent: 'neck_link', child: 'head_link', xyz: [0, 0, 0.022], axis: 'y', lo: -0.8, hi: 1.0 }, [
    { s: 'box', size: [0.054, 0.07, 0.05], xyz: [0.004, 0, 0.027], m: 'dark' }, { s: 'box', size: [0.004, 0.062, 0.042], xyz: [0.0315, 0, 0.027], m: 'grey' },
    { s: 'cyl', size: [0.0115, 0.008], ax: 'x', xyz: [0.034, 0, 0.028], m: 'glass' }, { s: 'cyl', size: [0.0152, 0.003], ax: 'x', xyz: [0.0325, 0, 0.028], m: 'glow' }]);
  add({ name: 'camera_joint', parent: 'head_link', child: 'camera_link', type: 'fixed', xyz: [0.039, 0, 0.028] });
  // OpenCV's view of the same camera: z forward, x right, y down (REP 103 optical frame)
  add({ name: 'camera_optical_joint', parent: 'camera_link', child: 'camera_optical_frame', type: 'fixed', rpy: [-Math.PI / 2, 0, -Math.PI / 2] });
  add({ name: 'imu_joint', parent: 'torso_link', child: 'imu_link', type: 'fixed', xyz: [0, 0, 0.05] });
  return { root: 'base_link', joints: J, visuals: V, masses: M };
}
function huR1() {
  const J = [], V = {}, M = {};
  const add = (j, vis, m) => { J.push(j); if (vis) V[j.child] = vis; if (m) M[j.child] = m; };
  V.base_link = [{ s: 'box', size: [0.13, 0.2, 0.09], xyz: [0, 0, -0.012], m: 'dark' }, { s: 'box', size: [0.14, 0.21, 0.028], xyz: [0, 0, 0.03], m: 'shell' }];
  M.base_link = [3.5, [0, 0, 0]];
  for (const [s, side] of [[1, 'left'], [-1, 'right']]) {
    const p = side + '_';
    add({ name: p + 'hip_pitch_joint', parent: 'base_link', child: p + 'hip_pitch_link', xyz: [0, s * 0.075, -0.035], axis: 'y', lo: -2.5, hi: 2.8 }, [{ s: 'cyl', size: [0.048, 0.06], ax: 'y', xyz: [0, s * 0.012, 0], m: 'dark' }], [1.0, [0, s * 0.01, 0]]);
    add({ name: p + 'hip_roll_joint', parent: p + 'hip_pitch_link', child: p + 'hip_roll_link', xyz: [0, 0, -0.045], axis: 'x', lo: s > 0 ? -0.4 : -1.6, hi: s > 0 ? 1.6 : 0.4 }, [{ s: 'cyl', size: [0.042, 0.07], ax: 'x', xyz: [0, 0, -0.004], m: 'dark' }], [1.1, [0, 0, -0.01]]);
    add({ name: p + 'hip_yaw_joint', parent: p + 'hip_roll_link', child: p + 'hip_yaw_link', xyz: [0, 0, -0.05], axis: 'z', lo: -2.7, hi: 2.7 }, [{ s: 'cyl', size: [0.04, 0.05], ax: 'z', xyz: [0, 0, -0.012], m: 'dark' }, { s: 'cap', size: [0.052, 0.11], ax: 'z', xyz: [0, 0, -0.12], m: 'shell' }, { s: 'box', size: [0.004, 0.05, 0.1], xyz: [0.052, 0, -0.12], m: 'accent' }], [2.0, [0, 0, -0.12]]);
    add({ name: p + 'knee_joint', parent: p + 'hip_yaw_link', child: p + 'knee_link', xyz: [0, 0, -0.24], axis: 'y', lo: -0.08, hi: 2.8 }, [{ s: 'cyl', size: [0.046, 0.076], ax: 'y', m: 'dark' }, { s: 'cap', size: [0.04, 0.16], ax: 'z', xyz: [0, 0, -0.135], m: 'shell' }, { s: 'cap', size: [0.034, 0.09], ax: 'z', xyz: [-0.03, 0, -0.11], m: 'dark' }], [1.5, [-0.01, 0, -0.13]]);
    add({ name: p + 'ankle_pitch_joint', parent: p + 'knee_link', child: p + 'ankle_pitch_link', xyz: [0, 0, -0.27], axis: 'y', lo: -0.95, hi: 0.65 }, [{ s: 'cyl', size: [0.028, 0.05], ax: 'y', m: 'dark' }], [0.2, [0, 0, 0]]);
    add({ name: p + 'ankle_roll_joint', parent: p + 'ankle_pitch_link', child: p + 'ankle_roll_link', xyz: [0, 0, 0], axis: 'x', lo: -0.35, hi: 0.35 }, [{ s: 'box', size: [0.21, 0.09, 0.03], xyz: [0.025, 0, -0.028], m: 'dark' }, { s: 'box', size: [0.12, 0.08, 0.02], xyz: [0.008, 0, -0.008], m: 'shell' }, { s: 'box', size: [0.212, 0.092, 0.006], xyz: [0.025, 0, -0.0445], m: 'rubber' }], [0.6, [0.02, 0, -0.03]]);
    add({ name: p + 'sole_joint', parent: p + 'ankle_roll_link', child: p + 'sole_link', type: 'fixed', xyz: [0.025, 0, -0.0475] });
    add({ name: p + 'shoulder_pitch_joint', parent: 'torso_link', child: p + 'shoulder_pitch_link', xyz: [0, s * 0.155, 0.24], axis: 'y', lo: -3.0, hi: 2.6 }, [{ s: 'cyl', size: [0.042, 0.05], ax: 'y', m: 'dark' }], [0.4, [0, 0, 0]]);
    add({ name: p + 'shoulder_roll_joint', parent: p + 'shoulder_pitch_link', child: p + 'shoulder_roll_link', xyz: [0, s * 0.035, 0], axis: 'x', lo: s > 0 ? -0.3 : -2.2, hi: s > 0 ? 2.2 : 0.3 }, [{ s: 'cyl', size: [0.036, 0.062], ax: 'x', xyz: [0, 0, -0.01], m: 'dark' }], [0.5, [0, 0, -0.02]]);
    add({ name: p + 'shoulder_yaw_joint', parent: p + 'shoulder_roll_link', child: p + 'shoulder_yaw_link', xyz: [0, 0, -0.055], axis: 'z', lo: -2.6, hi: 2.6 }, [{ s: 'cap', size: [0.036, 0.09], ax: 'z', xyz: [0, 0, -0.07], m: 'shell' }], [0.9, [0, 0, -0.08]]);
    add({ name: p + 'elbow_joint', parent: p + 'shoulder_yaw_link', child: p + 'elbow_link', xyz: [0, 0, -0.165], axis: 'y', lo: -2.4, hi: 0.5 }, [{ s: 'cyl', size: [0.035, 0.064], ax: 'y', m: 'dark' }, { s: 'cap', size: [0.031, 0.08], ax: 'z', xyz: [0, 0, -0.065], m: 'shell' }], [0.6, [0, 0, -0.06]]);
    add({ name: p + 'wrist_roll_joint', parent: p + 'elbow_link', child: p + 'wrist_roll_link', xyz: [0, 0, -0.125], axis: 'z', lo: -2.0, hi: 2.0 }, [{ s: 'cyl', size: [0.026, 0.02], ax: 'z', xyz: [0, 0, -0.005], m: 'dark' }, { s: 'box', size: [0.03, 0.062, 0.07], xyz: [0, 0, -0.045], m: 'dark' }, { s: 'box', size: [0.024, 0.056, 0.032], xyz: [0.004, 0, -0.094], m: 'grey' }, { s: 'box', size: [0.022, 0.014, 0.036], xyz: [0.014, s * 0.034, -0.04], m: 'grey' }], [0.35, [0, 0, -0.05]]);
  }
  add({ name: 'waist_yaw_joint', parent: 'base_link', child: 'waist_yaw_link', xyz: [0, 0, 0.045], axis: 'z', lo: -2.6, hi: 2.6 }, [{ s: 'cyl', size: [0.06, 0.04], ax: 'z', m: 'dark' }], [0.8, [0, 0, 0]]);
  add({ name: 'waist_pitch_joint', parent: 'waist_yaw_link', child: 'torso_link', xyz: [0, 0, 0.035], axis: 'y', lo: -0.4, hi: 0.6 }, [
    { s: 'box', size: [0.12, 0.17, 0.07], xyz: [0, 0, 0.03], m: 'dark' }, { s: 'box', size: [0.16, 0.25, 0.25], xyz: [0, 0, 0.165], m: 'shell' },
    { s: 'box', size: [0.006, 0.13, 0.09], xyz: [0.081, 0, 0.19], m: 'dark' }, { s: 'box', size: [0.005, 0.13, 0.008], xyz: [0.0825, 0, 0.13], m: 'accent' },
    { s: 'box', size: [0.06, 0.18, 0.2], xyz: [-0.1, 0, 0.16], m: 'grey' }, { s: 'sph', size: [0.05], xyz: [0, 0.13, 0.24], m: 'shell' }, { s: 'sph', size: [0.05], xyz: [0, -0.13, 0.24], m: 'shell' },
    { s: 'cyl', size: [0.035, 0.04], ax: 'z', xyz: [0, 0, 0.29], m: 'dark' }], [6.0, [-0.01, 0, 0.16]]);
  add({ name: 'head_yaw_joint', parent: 'torso_link', child: 'neck_link', xyz: [0, 0, 0.3], axis: 'z', lo: -1.3, hi: 1.3 }, [{ s: 'cyl', size: [0.03, 0.03], ax: 'z', xyz: [0, 0, 0.015], m: 'dark' }], [0.3, [0, 0, 0.01]]);
  add({ name: 'head_pitch_joint', parent: 'neck_link', child: 'head_link', xyz: [0, 0, 0.035], axis: 'y', lo: -0.5, hi: 0.7 }, [
    { s: 'box', size: [0.12, 0.13, 0.11], xyz: [0, 0, 0.06], m: 'shell' }, { s: 'sph', size: [0.068], xyz: [-0.006, 0, 0.088], m: 'shell' },
    { s: 'box', size: [0.012, 0.112, 0.05], xyz: [0.058, 0, 0.066], m: 'glass' }, { s: 'sph', size: [0.0085], xyz: [0.0635, 0.028, 0.068], m: 'glow' }, { s: 'sph', size: [0.0085], xyz: [0.0635, -0.028, 0.068], m: 'glow' }], [1.0, [0, 0, 0.06]]);
  add({ name: 'camera_joint', parent: 'head_link', child: 'camera_link', type: 'fixed', xyz: [0.066, 0, 0.068] });
  return { root: 'base_link', joints: J, visuals: V, masses: M };
}

/* ---------- physics: collision shapes (simplified like URDF collision meshes) and servo models ---------- */
function huAinexPhys() {
  const coll = { base_link: false, torso_link: [{ t: 'box', size: [0.08, 0.1, 0.135], xyz: [-0.005, 0, 0.055] }], head_link: [{ t: 'box', size: [0.06, 0.072, 0.052], xyz: [0.004, 0, 0.026] }], neck_link: false, camera_link: false, camera_optical_frame: false, imu_link: false };
  for (const p of ['l_', 'r_']) Object.assign(coll, {
    [p + 'hip_yaw_link']: false, [p + 'hip_roll_link']: false, [p + 'ankle_link']: false, [p + 'sole_link']: false, [p + 'shoulder_link']: false, [p + 'forearm_link']: false, [p + 'finger_link']: false,
    [p + 'thigh_link']: [{ t: 'box', size: [0.032, 0.044, 0.09], xyz: [0, 0, -0.045] }],
    [p + 'shin_link']: [{ t: 'box', size: [0.032, 0.042, 0.09], xyz: [0, 0, -0.045] }],
    [p + 'foot_link']: [{ t: 'box', size: [0.11, 0.064, 0.012], xyz: [0.008, 0, -0.024], mu: 1.0, tag: 'foot' }],
    [p + 'upper_arm_link']: [{ t: 'cap', size: [0.014, 0.03], xyz: [0, 0, -0.036] }],
    [p + 'wrist_link']: [{ t: 'cap', size: [0.013, 0.018], xyz: [-0.002, 0, -0.028] }],
  });
  // serial bus servos: stiff position loop, about 3.4 N·m stall and 6 rad/s no-load speed, geared rotor inertia and back-EMF damping
  return {
    name: 'ainex', mass: 1.9, coll, mu: 0.8,
    act: (n) => {
      const leg = /hip|knee|ank/.test(n), head = /head/.test(n), grip = /gripper/.test(n);
      return { kp: leg ? 30 : head ? 4 : grip ? 1.5 : 8, kd: leg ? 0.15 : 0.04, tmax: leg ? 3.4 : head ? 1.2 : grip ? 0.6 : 2.5, wmax: 6.0, arm: leg ? 0.01 : 0.004, damp: leg ? 0.25 : 0.1, fric: 0.02 };
    },
  };
}

function huR1Phys() {
  const coll = { base_link: [{ t: 'box', size: [0.14, 0.2, 0.12], xyz: [0, 0, 0] }], waist_yaw_link: false, neck_link: false, camera_link: false,
    torso_link: [{ t: 'box', size: [0.17, 0.25, 0.27], xyz: [-0.015, 0, 0.16] }], head_link: [{ t: 'sph', size: [0.07], xyz: [0, 0, 0.075] }] };
  for (const p of ['left_', 'right_']) Object.assign(coll, {
    [p + 'hip_pitch_link']: false, [p + 'hip_roll_link']: false, [p + 'ankle_pitch_link']: false, [p + 'sole_link']: false, [p + 'shoulder_pitch_link']: false, [p + 'shoulder_roll_link']: false,
    [p + 'hip_yaw_link']: [{ t: 'cap', size: [0.052, 0.055], xyz: [0, 0, -0.12] }],
    [p + 'knee_link']: [{ t: 'cap', size: [0.042, 0.08], xyz: [-0.01, 0, -0.13] }],
    [p + 'ankle_roll_link']: [{ t: 'box', size: [0.21, 0.09, 0.02], xyz: [0.025, 0, -0.0375], mu: 1.0, tag: 'foot' }],
    [p + 'shoulder_yaw_link']: [{ t: 'cap', size: [0.036, 0.045], xyz: [0, 0, -0.07] }],
    [p + 'elbow_link']: [{ t: 'cap', size: [0.031, 0.04], xyz: [0, 0, -0.065] }],
    [p + 'wrist_roll_link']: [{ t: 'box', size: [0.032, 0.064, 0.11], xyz: [0, 0, -0.06] }],
  });
  // quasi-direct-drive joint motors run as PD loops (like the SDK's kp/kd), with torque limits per joint group
  return {
    name: 'r1', mass: 25, coll, mu: 0.8, masses: null,
    act: (n) => {
      const g = /knee/.test(n) ? [650, 15, 150, 0.04] : /hip/.test(n) ? [540, 13, 120, 0.035] : /ankle/.test(n) ? [350, 9, 60, 0.02] : /waist/.test(n) ? [540, 12, 90, 0.03] : /wrist|head/.test(n) ? [25, 0.8, 10, 0.006] : [80, 2.5, 35, 0.015];
      return { kp: g[0], kd: g[1], tmax: g[2], wmax: 25, arm: g[3], damp: 0.3, fric: 0.1 };
    },
  };
}

/* ---------- action groups written with logical joint names: L./R. prefix, ~ = offset from the standing pose ---------- */
const HU_ARMS_HOME = { 'L.sp': 0, 'R.sp': 0, 'L.sr': 0.14, 'R.sr': -0.14, 'L.ep': -0.3, 'R.ep': -0.3, 'L.ey': 0, 'R.ey': 0, 'L.sy': 0, 'R.sy': 0, 'L.wr': 0, 'R.wr': 0, 'L.gr': 0, 'R.gr': 0 };
const HU_LEGS_HOME = { '~L.hp': 0, '~L.kn': 0, '~L.ap': 0, '~R.hp': 0, '~R.kn': 0, '~R.ap': 0, '~L.hr': 0, '~R.hr': 0, '~L.ar': 0, '~R.ar': 0, '~L.hy': 0, '~R.hy': 0 };
const HU_HOME = Object.assign({}, HU_ARMS_HOME, HU_LEGS_HOME, { '~pan': 0, '~tilt': 0, wy: 0, wp: 0 });
const sym = (o) => { const r = {}; for (const [k, v] of Object.entries(o)) { const m = k.match(/^(~?)S\.(.+)$/); if (m) { r[m[1] + 'L.' + m[2]] = v; r[m[1] + 'R.' + m[2]] = /(sr|hr|ar|hy|ey|sy|wr)$/.test(m[2]) ? -v : v; } else r[k] = v; } return r; };
const HU_ACTIONS = {
  wave: { label: 'Wave', icon: 'sparkle', upper: true, frames: [[0.5, { 'R.sp': -2.75, 'R.sr': -0.3, 'R.ep': -0.35 }], [0.28, { 'R.ep': -1.15 }], [0.28, { 'R.ep': -0.3 }], [0.28, { 'R.ep': -1.15 }], [0.28, { 'R.ep': -0.3 }], [0.28, { 'R.ep': -1.15 }], [0.5, HU_ARMS_HOME]] },
  bow: { label: 'Bow', icon: 'arrowD', frames: [[0.6, sym({ '~S.hp': -0.6, 'S.sp': 0.15, 'S.sr': 0.08, 'S.ep': -0.2, '~tilt': 0.3 })], [0.7, {}], [0.6, Object.assign({}, HU_HOME)]] },
  raise_arms: { label: 'Arms up', icon: 'arrowU', upper: true, frames: [[0.6, sym({ 'S.sp': -2.95, 'S.sr': 0.18, 'S.ep': -0.1 })], [0.8, {}], [0.6, HU_ARMS_HOME]] },
  squat: { label: 'Squat', icon: 'arrowD', frames: [[0.7, sym({ '~S.hp': -0.55, '~S.kn': 1.05, '~S.ap': -0.5, 'S.sp': -1.35, 'S.ep': -0.1, 'S.sr': 0.1 })], [0.5, {}], [0.7, Object.assign({}, HU_HOME)]] },
  left_kick: { label: 'Left kick', icon: 'bolt', robots: ['ainex'], frames: [
    [0.6, sym({ 'S.sr': 0.35, sup: 'R' })],
    [0.35, { '~L.hp': -0.55, '~L.kn': 1.1, '~L.ap': -0.45, 'R.sp': -0.5, 'L.sp': 0.4, sup: 'R' }],
    [0.18, { '~L.hp': -1.0, '~L.kn': 0.1, '~L.ap': 0.12, 'R.sp': 0.4, 'L.sp': -0.5, sup: 'R' }],
    [0.4, { '~L.hp': -0.4, '~L.kn': 0.8, '~L.ap': -0.35, sup: 'R' }],
    [0.45, { '~L.hp': 0, '~L.kn': 0, '~L.ap': 0, sup: 'R' }], [0.8, Object.assign({}, HU_HOME, { sup: 'B' })]] },
  right_kick: { label: 'Right kick', icon: 'bolt', robots: ['ainex'], frames: null },
  nod: { label: 'Nod', icon: 'check', upper: true, frames: [[0.22, { '~tilt': 0.4 }], [0.22, { '~tilt': -0.2 }], [0.22, { '~tilt': 0.4 }], [0.3, { '~tilt': 0 }]] },
  shake: { label: 'Shake head', icon: 'x', upper: true, frames: [[0.2, { '~pan': 0.5 }], [0.3, { '~pan': -0.5 }], [0.3, { '~pan': 0.5 }], [0.25, { '~pan': 0 }]] },
  grab: { label: 'Open + close hands', icon: 'tools', upper: true, robots: ['ainex'], frames: [[0.4, sym({ 'S.sp': -1.2, 'S.ep': -0.3, 'S.gr': 0.85 })], [0.5, {}], [0.4, sym({ 'S.gr': 0 })], [0.6, HU_ARMS_HOME]] },
  handshake: { label: 'Handshake', icon: 'tools', upper: true, robots: ['r1'], frames: [[0.6, { 'R.sp': -0.85, 'R.sr': -0.12, 'R.ep': -0.55, 'R.wr': -1.4 }], [0.25, { 'R.sp': -0.7 }], [0.25, { 'R.sp': -0.95 }], [0.25, { 'R.sp': -0.7 }], [0.25, { 'R.sp': -0.95 }], [0.6, HU_ARMS_HOME]] },
  dance: { label: 'Dance', icon: 'sparkle', robots: ['r1'], frames: [
    [0.4, sym({ 'S.sp': -1.6, 'S.sr': 0.5, 'S.ep': -1.2, wy: 0.35, '~S.kn': 0.25, '~S.hp': -0.13, '~S.ap': -0.12 })], [0.4, sym({ 'S.sp': -1.0, 'S.sr': 0.3, 'S.ep': -1.6, wy: -0.35, '~S.kn': 0, '~S.hp': 0, '~S.ap': 0 })],
    [0.4, sym({ 'S.sp': -1.6, 'S.sr': 0.5, 'S.ep': -1.2, wy: 0.35, '~S.kn': 0.25, '~S.hp': -0.13, '~S.ap': -0.12 })], [0.4, sym({ 'S.sp': -1.0, 'S.sr': 0.3, 'S.ep': -1.6, wy: -0.35, '~S.kn': 0, '~S.hp': 0, '~S.ap': 0 })],
    [0.35, { 'L.sp': -2.9, 'R.sp': 0.4, 'L.sr': 0.2, 'R.sr': -0.6, wy: 0 }], [0.35, { 'R.sp': -2.9, 'L.sp': 0.4, 'R.sr': -0.2, 'L.sr': 0.6 }], [0.6, Object.assign({}, HU_HOME)]] },
  kungfu: { label: 'Kung fu', icon: 'bolt', robots: ['r1'], frames: [
    [0.45, sym({ '~S.hp': -0.35, '~S.kn': 0.7, '~S.ap': -0.35, 'S.sp': -0.4, 'S.ep': -1.9, 'S.sr': 0.25 })],
    [0.16, { 'R.sp': -1.55, 'R.ep': -0.05, 'R.sr': 0.05, wy: 0.4 }], [0.22, { 'R.sp': -0.4, 'R.ep': -1.9, 'R.sr': -0.25, wy: 0 }],
    [0.16, { 'L.sp': -1.55, 'L.ep': -0.05, 'L.sr': -0.05, wy: -0.4 }], [0.22, { 'L.sp': -0.4, 'L.ep': -1.9, 'L.sr': 0.25, wy: 0 }],
    [0.16, { 'R.sp': -1.55, 'R.ep': -0.05, 'R.sr': 0.05, wy: 0.4 }], [0.6, Object.assign({}, HU_HOME)]] },
};
HU_ACTIONS.right_kick.frames = HU_ACTIONS.left_kick.frames.map(([d, o]) => [d, Object.fromEntries(Object.entries(o).map(([k, v]) => { if (k === 'sup') return [k, v === 'R' ? 'L' : v === 'L' ? 'R' : v]; const m = k.match(/^(~?)([LR])\.(.+)$/); if (!m) return [k, v]; const nk = m[1] + (m[2] === 'L' ? 'R' : 'L') + '.' + m[3]; return [nk, /(sr|hr|ar|hy|ey|sy|wr)$/.test(m[3]) ? -v : v]; }))]);
HU_ACTIONS.right_kick.label = 'Right kick';

/* ---------- simulator: the robot is a physics body; controllers only set servo targets ---------- */
class HumanoidSim extends SnoSim {
  constructor(def) {
    super(def);
    const H = this.H = def.hu, desc = H.desc();
    this.desc = desc; this.kin = new KinRobot(desc); this.names = H.order ? H.order.slice() : this.kin.names; this.ctl = H.ctl;
    this.legSet = new Set(this.names.filter(n => H.legRe.test(n)));
    this.world = new PhysWorld({ h: 1 / 480, mu: 0.9 });
    const po = H.phys(); if (H.useMasses) po.masses = desc.masses;
    this.rb = physFromKin(this.kin, po); this.world.add(this.rb);
    this.gains = { kp: this.rb.kp.slice(), kd: this.rb.kd.slice(), tmax: this.rb.tmax.slice() };
    this.walker = new BipedWalker({ kin: this.kin, S: this.rb, sole: H.sole, footY: H.footY, z: H.standZ, g: Object.assign({}, H.walk) });
    Object.assign(this.walker.fb, H.fb || {}); this.fb0 = Object.assign({}, this.walker.fb);
    this.q = {}; this.qt = {}; this.dq = {}; this.tau = {}; this.names.forEach(n => { this.q[n] = 0; this.qt[n] = 0; this.dq[n] = 0; this.tau[n] = 0; });
    this.torsoB = this.rb.linkBody[H.hook[0]];
    this.solveHome();
    this.recorder = []; this.recT = 0.6; this.userActs = S.sno.actions[this.id] || [];
    this.pd = null; this.plot = []; this.phys = { ms: 0, steps: 0 };
    this.buildWorld();
    this.reset();
  }
  jn(k) { const m = k.match(/^([LR])\.(.+)$/); if (m) { const b = this.H.map[m[2]]; return b ? this.H.side[m[1]] + b : null; } return this.H.map[k] || null; }
  solveHome() {
    const H = this.H, home = this.home = {};
    Object.entries(HU_ARMS_HOME).concat([['pan', 0], ['tilt', 0], ['wy', 0], ['wp', 0]], Object.entries(H.homeExtra || {})).forEach(([k, v]) => { const n = this.jn(k); if (n) home[n] = v; });
    const q = {}; this.kin.names.forEach(n => { q[n] = home[n] || 0; });
    for (const s of ['l', 'r']) {
      const ch = this.kin.chain(this.kin.root, H.sole[s]);
      ch.forEach(j => { if (j.type !== 'fixed') q[j.name] = /knee/.test(j.name) ? 0.7 : /hip_pitch/.test(j.name) ? -0.35 : /ank_pitch|ankle_pitch/.test(j.name) ? -0.35 : 0; });
      const tgt = K3.mul(K3.inv(K3.fromRPY(0, 0, H.standZ, 0, 0, 0)), K3.fromRPY(0, (s === 'l' ? 1 : -1) * H.footY, 0, 0, 0, 0));
      this.kin.ik(ch, q, K3.I(), tgt, { iters: 80, wr: 1, lambda: 0.005 });
    }
    // centre of mass over the feet
    const pel = this.walker.balance(q, { x: 0, y: 0, z: H.standZ, yaw: 0 }, { l: { x: 0, y: H.footY, yaw: 0 }, r: { x: 0, y: -H.footY, yaw: 0 } });
    this.homePel = [pel.x, pel.y];
    this.names.forEach(n => { home[n] = q[n] != null ? q[n] : 0; });
    const T = this.kin.fk(home, K3.fromRPY(pel.x, pel.y, H.standZ, 0, 0, 0)), hk = K3.apply(T[H.hook[0]], ...H.hook[1]); this.hookZ = hk[2];
  }
  legNames() { return this.names.filter(n => this.legSet.has(n)); }
  resolve(frame) {
    const out = {};
    for (const [k, v] of Object.entries(frame)) {
      if (k === 'lean') continue;
      if (k === 'sup') { out.__sup = v; continue; }
      const rel = k[0] === '~', n = this.jn(rel ? k.slice(1) : k); if (!n) continue;
      const j = this.kin.J[n]; out[n] = clampN(rel ? this.home[n] + v : v, j.lo, j.hi);
    }
    const sup = out.__sup; delete out.__sup;
    return { q: out, sup };
  }
  reset() {
    const H = this.H, [x, y, yaw] = H.start, rb = this.rb, c = Math.cos(yaw), s = Math.sin(yaw), [px, py] = this.homePel;
    this.names.forEach(n => { this.q[n] = this.home[n]; this.qt[n] = this.home[n]; this.dq[n] = 0; });
    for (const n in rb.ji) { const k = rb.ji[n], v = this.home[n] || 0; rb.q[k] = v; rb.qt[k] = v; rb.vt[k] = 0; rb.tff[k] = 0; rb.mode[k] = 1; rb.kp[k] = this.gains.kp[k]; rb.kd[k] = this.gains.kd[k]; rb.tmax[k] = this.gains.tmax[k]; }
    rb.setYaw(x + c * px - s * py, y + s * px + c * py, H.standZ + 0.0008, yaw); rb.zeroVel();
    this.world.cache.clear(); this.hoist = null; this.gantry = null; this.pushLeft = 0; this.grab = null;
    this.objsReset();
    Object.assign(this.walker.fb, this.fb0); this.walker.reset(Object.assign({}, this.home));
    this.mode = 'stand'; this.act = null; this.ovl = null; this.queue = null; this.fallT = 0; this.v = [0, 0, 0];
    this.nav = { x, y, yaw }; this.stats = { dist: 0, turn: 0, side: 0, cliDist: 0, userDist: 0, userJoint: 0 }; this._pn = null; this.imuPrev = null;
    if (this.pd) this.pdReset();
    this.readState(0);
  }
  // stand the robot somewhere else (tools and tests; in the app the hoist puts it back on its feet)
  teleport(x, y, yaw) {
    const rb = this.rb, c = Math.cos(yaw), s = Math.sin(yaw), [px, py] = this.homePel;
    this.names.forEach(n => { this.q[n] = this.home[n]; this.qt[n] = this.home[n]; this.dq[n] = 0; });
    for (const n in rb.ji) { const k = rb.ji[n], v = this.home[n] || 0; rb.q[k] = v; rb.qt[k] = v; rb.vt[k] = 0; rb.tff[k] = 0; }
    rb.setYaw(x + c * px - s * py, y + s * px + c * py, this.H.standZ + 0.0008, yaw); rb.zeroVel(); this.world.cache.clear();
    this.walker.reset(Object.assign({}, this.home)); this.mode = 'stand'; this.act = null; this.ovl = null; this.queue = null; this.hoist = null;
    this.v = [0, 0, 0]; this._pn = null; this.imuPrev = null;
    this.readState(0);
  }
  /* ---------- world: obstacles and loose objects ---------- */
  buildWorld() { }
  objsReset() {
    if (!this.objs) this.objs = (this.H.objs || []).map(o => {
      const sys = o.kind === 'ball' ? physSphere('ball', o.r, o.m || 0.045, { roll: 0.0012, rest: 0.55, hollow: true, mu: 0.6 }) : physBox(o.kind, [o.size, o.size, o.size], o.m || 0.03, { mu: 0.6 });
      this.world.add(sys); return Object.assign({ vx: 0, vy: 0, sys }, o);
    });
    this.objs.forEach((o, i) => this.placeObj(o, this.H.objs[i].x, this.H.objs[i].y));
  }
  placeObj(o, x, y) { const z = o.kind === 'ball' ? o.r + 0.0003 : o.size / 2 + 0.0003; o.sys.setYaw(x, y, z, 0); o.sys.zeroVel(); Object.assign(o, { x, y, z, vx: 0, vy: 0 }); }
  /* ---------- ROS ---------- */
  setupRos() {
    const H = this.H, c = this.ctl;
    this.node(c, 'sim', H.params);
    this.cmdVelSub(c);
    this.bus.subscribe(c, '/joint_commands', 'sensor_msgs/msg/JointState', (m, from) => this.onJointCmd(m, from));
    this.bus.subscribe(c, H.actionTopic, 'std_msgs/msg/String', (m, from) => { const r = this.playAction(String(m.data || '').trim()); if (!r.ok) this.logWarn(r.reason); else this.event('action', { name: String(m.data).trim(), from }); });
    ['/joint_states', 'sensor_msgs/msg/JointState', '/imu', 'sensor_msgs/msg/Imu', '/odom', 'nav_msgs/msg/Odometry', '/tf', 'tf2_msgs/msg/TFMessage', '/tf_static', 'tf2_msgs/msg/TFMessage', H.stateTopic, 'std_msgs/msg/String'].forEach((x, i, a) => { if (i % 2 === 0) this.bus.advertise(c, x, a[i + 1]); });
    this.bus.topic('/tf_static', 'tf2_msgs/msg/TFMessage').latched = true;
    this.pub(c, '/tf_static', 'tf2_msgs/msg/TFMessage', this.tfMsg([[H.cameraParent, 'camera_link', this.kin.J.camera_joint.O]]));
    this.bus.service('/reset_world', 'std_srvs/srv/Empty', c, () => { this.reset(); this.event('service', '/reset_world'); return {}; });
    this.bus.service('/sim/push', 'std_srvs/srv/Trigger', c, () => { this.push(1); this.event('service', '/sim/push'); return { success: true, message: 'pushed' }; });
    this.bus.service(H.standSrv, 'std_srvs/srv/Trigger', c, () => { const r = this.standUp(); this.event('service', H.standSrv); return { success: r.ok, message: r.ok ? 'standing up' : r.reason }; });
    this.setupExtra();
  }
  setupExtra() { }
  behaviorsTick() { }
  runExe(T, pkg, exe) { humanoidExe(this, T, pkg, exe); }
  logWarn(t) { this.lastWarn = t; }
  onParam(node, k, v) { this.event('param', { node, k, v }); return { ok: true }; }
  onJointCmd(m, from) {
    const names = m.name || [], pos = m.position || [];
    if (!names.length) return;
    let legs = false, n0 = 0;
    names.forEach((n, i) => { const j = this.kin.J[n]; if (!j || j.type === 'fixed' || !Number.isFinite(pos[i])) return; this.qt[n] = clampN(pos[i], j.lo, j.hi); if (this.legSet.has(n)) legs = true; n0++; });
    if (this.mode === 'low') { names.forEach((n, i) => { if (this.pd && this.pd.qd[n] != null && Number.isFinite(pos[i])) this.pd.qd[n] = pos[i]; }); }
    else if (legs && (this.mode === 'stand' || this.mode === 'walk')) this.enterManual();
    const nd = this.bus.nodes.get(from); this.event('joint_cmd', { from, kind: nd ? (nd.user ? 'user' : nd.kind) : '', n: n0 });
  }
  /* ---------- modes ---------- */
  canMove() { return this.mode === 'stand' || this.mode === 'walk'; }
  enterManual() { if (this.mode === 'manual') return; this.legNames().forEach(n => { this.qt[n] = this.q[n]; }); this.mode = 'manual'; this.act = null; }
  stopWalkNow() { this.walker.setCmd(0, 0, 0); }
  toStand() { this.walker.reset(Object.assign({}, this.qt)); this.mode = 'stand'; }
  goStand() {
    if (this.mode === 'fallen' || this.mode === 'damp' || this.mode === 'fall') return this.standUp();
    if (this.mode === 'low') return this.enterMode('stand');
    if (this.mode === 'getup') return { ok: false, reason: 'already standing up' };
    this.ovl = null;
    this.startAct({ name: 'stand', frames: [[0.8, Object.assign({}, HU_HOME)]], legs: true });
    return { ok: true };
  }
  // stand back up: a hoist (like the gantry real robots use) lifts the torso while the servos return to the standing pose
  standUp() {
    if (this.mode === 'fallen' || this.mode === 'damp' || this.mode === 'fall') { this.startHoist(); return { ok: true }; }
    if (this.mode === 'low') { this.enterMode('stand'); return { ok: true }; }
    if (this.mode === 'manual') return this.goStand();
    return { ok: false, reason: 'already standing' };
  }
  startHoist(fromLow) {
    const p = this.hookPos(), T = this.rb.T[0], yaw = Math.atan2(T[1] * 0 + T[4], T[0]) || 0, fwd = [T[0], T[4]], up = [T[2], T[6]];
    // heading: where the robot's front (or its head, if lying) points
    const hd = Math.abs(T[8]) > 0.7 ? Math.atan2(up[1] * Math.sign(-T[8]), up[0] * Math.sign(-T[8])) : Math.atan2(fwd[1], fwd[0]);
    this.hoist = { t: 0, phase: fromLow ? 1 : 0, x: p[0], y: p[1], z0: p[2], yaw: Number.isFinite(hd) ? hd : yaw, ax: p[0], ay: p[1], az: p[2] };
    this.names.forEach(n => { this.qt[n] = this.home[n]; });
    this.act = null; this.ovl = null; this.mode = 'getup'; this.event('act_start', 'stand_up'); this.gantry = null;
  }
  hookPos() { return K3.apply(this.rb.T[this.torsoB], ...this.hookLocal()); }
  hoistSpot(x, y) { return [x, y]; }
  hookLocal() { const H = this.H, off = this.rb.linkOff[H.hook[0]]; return K3.apply(off, ...H.hook[1]); }
  enterMode(m) { this.mode = m; return { ok: true }; }
  playAction(name) {
    const a = HU_ACTIONS[name] || (this.userActs.find(x => x.name === name) ? { frames: this.userActs.find(x => x.name === name).frames, user: true } : null);
    if (!a) return { ok: false, reason: `unknown action '${name}'` };
    if (a.robots && !a.robots.includes(this.id)) return { ok: false, reason: `'${name}' is not available on this robot` };
    if (['fallen', 'fall', 'damp', 'low', 'getup'].includes(this.mode)) return { ok: false, reason: `cannot play '${name}' while ${this.modeLabel().toLowerCase()}` };
    const frames = a.user ? a.frames.map(([d, q]) => [d, q]) : a.frames;
    const legs = a.user ? frames.some(([, q]) => Object.keys(q).some(n => this.legSet.has(n))) : !a.upper;
    const act = { name, frames, legs, user: !!a.user };
    this.event('action', name);
    if (!legs) { this.ovl = this.mkAct(act); return { ok: true }; }
    if (this.walker.walking()) { this.queue = act; return { ok: true }; }
    this.startAct(act); return { ok: true };
  }
  mkAct(a) { return Object.assign({ i: 0, t: 0, from: Object.assign({}, this.qt), res: a.frames.map(([d, f]) => [d, a.user ? { q: f } : this.resolve(f)]) }, a); }
  startAct(a) {
    this.ovl = null; this.act = this.mkAct(a); this.mode = 'act'; this.queue = null;
    const c = this.rb.comState(); this.actRef = this.supportMid(); this.actRef.c = [c[0], c[1]];
    this.event('act_start', a.name);
  }
  supportMid() { const l = this.rb.linkT(this.H.sole.l), r = this.rb.linkT(this.H.sole.r); return { x: (l[3] + r[3]) / 2, y: (l[7] + r[7]) / 2, yaw: Math.atan2(l[4] + r[4], l[0] + r[0]) }; }
  /* ---------- per step ---------- */
  step(dt) {
    this.t += dt; this.teleopTick();
    const live = this.cmdLive(this.P(this.ctl, 'cmd_timeout'));
    this.behaviorsTick(dt);
    const n = 4, h = dt / n, t0 = typeof performance !== 'undefined' ? performance.now() : 0;
    this.world.h = h / 2;
    for (let i = 0; i < n; i++) { this.control(h, live); this.world.step(); this.world.step(); }
    if (t0) this.phys.ms = this.phys.ms * 0.9 + (performance.now() - t0) * 0.1;
    this.readState(dt);
    this.objStep(dt);
    this.publish(dt);
    this.camTick();
    this.tickStats(dt);
  }
  control(h, live) {
    const m = this.mode;
    this.upperTick(h);
    if (m === 'stand' || m === 'walk') this.locoCtl(h, live);
    else if (m === 'act') this.actCtl(h);
    else if (m === 'manual') this.servoAll(1);
    else if (m === 'fall' || m === 'fallen') this.fallCtl(h);
    else if (m === 'getup') this.hoistCtl(h);
    else if (m === 'damp') this.dampCtl();
    else if (m === 'low') this.lowCtl(h);
    if (this.pushLeft > 0) { this.pushLeft -= h; this.rb.push(this.torsoB, this.hookLocal(), this.pushF); }
    if (this.grab) this.grabCtl();
    this.checkFall(h);
  }
  gait() { return this.H.gait; }
  // servo targets straight from qt (scale: stiffness factor)
  servoAll(scale) {
    const rb = this.rb;
    for (const n in rb.ji) { const k = rb.ji[n]; rb.mode[k] = 1; rb.kp[k] = this.gains.kp[k] * scale; rb.kd[k] = this.gains.kd[k] * Math.max(scale, 0.3); rb.qt[k] = this.qt[n] != null ? this.qt[n] : rb.qt[k]; rb.vt[k] = 0; rb.tff[k] = 0; }
  }
  locoCtl(h, live) {
    const H = this.H, g = this.gait(), W = this.walker, rb = this.rb;
    const vmax = Math.min(g.maxV, this.P(this.ctl, g.fixed ? 'max_run_speed' : 'max_linear_speed') || g.maxV), wmax = Math.min(g.maxW, this.P(this.ctl, 'max_angular_speed') || g.maxW);
    let tv = [0, 0, 0];
    if (live && !this.queue) tv = [clampN(this.cmd.vx, -vmax * 0.6, vmax), clampN(this.cmd.vy, -g.maxVy, g.maxVy), clampN(this.cmd.wz, -wmax, wmax)];
    this.tv = tv; W.setCmd(tv[0], tv[1], tv[2]);
    // live parameters
    const bal = this.P(this.ctl, 'balance') !== false;
    W.fb.kc = bal ? this.fb0.kc : 0; W.fb.kstep = bal ? this.fb0.kstep : 0; W.g.pushStep = bal ? H.walk.pushStep : 1e9;
    if (!g.fixed) { const sh = this.P(this.ctl, 'step_height'), sp = this.P(this.ctl, 'step_period'), bh = this.P(this.ctl, 'body_height'); if (sh) W.g.stepH = sh; if (sp) { const k = sp / (H.walk.Tss + H.walk.Tds); W.g.Tss = H.walk.Tss * k; W.g.Tds = H.walk.Tds * k; } if (bh) W.z = bh; }
    else { W.g.Tss = g.Tss || H.walk.Tss; W.g.Tds = g.Tds || H.walk.Tds; W.g.stepH = g.stepH; }
    const ms = this._meas || (this._meas = { com: new Float64Array(6), pelvis: null, omega: [0, 0, 0] });
    rb.comState(ms.com); ms.pelvis = rb.T[0]; ms.omega[0] = rb.v[0]; ms.omega[1] = rb.v[1]; ms.omega[2] = rb.v[2];
    const q = W.update(h, ms);
    this.mode = W.mode === 'walk' ? 'walk' : 'stand';
    // arm swing against the swinging leg
    let swL = 0, swR = 0; const s = W.segs && W.segs[W.si];
    if (W.walking() && s && s.kind === 'ss' && this.P(this.ctl, 'arm_swing') !== false) {
      const p = W.steps[s.step], u = W.st / s.T, amp = g.arm * Math.min(1, Math.hypot(W.v[0], W.v[1]) / Math.max(0.05, g.maxV * 0.6) + 0.15), sg = Math.sin(Math.PI * u) * (p.s === 'l' ? 1 : -1);
      swL = amp * sg; swR = -amp * sg;
    }
    this.swingArm = this.swingArm ? [this.swingArm[0] + (swL - this.swingArm[0]) * 0.1, this.swingArm[1] + (swR - this.swingArm[1]) * 0.1] : [0, 0];
    const spL = this.jn('L.sp'), spR = this.jn('R.sp');
    for (const n in rb.ji) {
      const k = rb.ji[n]; rb.mode[k] = 1; rb.kp[k] = this.gains.kp[k]; rb.kd[k] = this.gains.kd[k];
      if (this.legSet.has(n)) { rb.qt[k] = q[n]; rb.tff[k] = W.ff[n] || 0; rb.vt[k] = (W.qd && W.qd[n]) || 0; this.qt[n] = q[n]; }
      else { rb.qt[k] = (this.qt[n] != null ? this.qt[n] : 0) + (n === spL ? this.swingArm[0] : n === spR ? this.swingArm[1] : 0); rb.tff[k] = 0; rb.vt[k] = 0; }
    }
    if (this.queue && W.mode === 'stand') { const a = this.queue; this.queue = null; this.startAct(a); }
  }
  // keyframe actions are open-loop servo targets, plus a small ankle balance (like a gyro-assisted action group)
  actCtl(h) {
    const a = this.act; if (!a) { this.toStand(); return; }
    a.t += h; const [d, f] = a.res[a.i], u = ease01(a.t / d);
    for (const [n, v] of Object.entries(f.q)) this.qt[n] = lerpN(a.from[n] != null ? a.from[n] : this.qt[n], v, u);
    this.servoAll(1);
    const sw = { L: 0, R: 1, B: 0.5 }, s0 = sw[a.supFrom || 'B'], s1 = f.sup ? sw[f.sup] : s0;
    if (this.H.ankleBal) this.postureBalance(lerpN(s0, s1, u));
    const rb = this.rb, ref = this.actRef, c = rb.comState(this._c6 || (this._c6 = new Float64Array(6))), om = this.walker.om;
    if (ref && this.H.ankleBal) {
      const cy = Math.cos(ref.yaw), sy = Math.sin(ref.yaw), ex = c[0] + c[3] / om - ref.x - (ref.c[0] - ref.x), ey = c[1] + c[4] / om - ref.y - (ref.c[1] - ref.y);
      const lx = cy * ex + sy * ey, ly = -sy * ex + cy * ey, [kp, kr] = this.H.ankleBal;
      for (const s of ['L', 'R']) { const ap = this.jn(s + '.ap'), ar = this.jn(s + '.ar'); rb.qt[rb.ji[ap]] += clampN(kp * lx, -0.25, 0.25); rb.qt[rb.ji[ar]] += clampN(-kr * ly, -0.2, 0.2); }
    }
    if (a.t >= d) {
      a.from = Object.assign({}, this.qt); a.supFrom = f.sup || a.supFrom; a.i++; a.t = 0;
      if (a.i >= a.res.length) { this.act = null; this.toStand(); this.event('action_done', a.name); }
    }
  }
  // static balance for a keyframe pose: ankles pitch the body and the legs tilt sideways (hip and ankle roll together,
  // torso upright) so the pose's centre of mass sits over the support point (w: 0 left foot, 1 right foot, 0.5 between)
  postureBalance(w) {
    const q = this.qt, W = this.walker, kin = this.kin, H = this.H, rb = this.rb, c = this._pc || (this._pc = [0, 0, 0]);
    const ap = [this.jn('L.ap'), this.jn('R.ap')], ar = [this.jn('L.ar'), this.jn('R.ar')], hr = [this.jn('L.hr'), this.jn('R.hr')];
    const rel = (qq) => {
      W.comLocal(qq, c); const T = kin.fk(qq, K3.I()), L = T[H.sole.l], R = T[H.sole.r];
      const mx = L[3] + (R[3] - L[3]) * w, my = L[7] + (R[7] - L[7]) * w, mz = Math.min(L[11], R[11]), F = w > 0.5 ? R : L;
      const dx = c[0] - mx, dy = c[1] - my, dz = c[2] - mz; return [F[0] * dx + F[4] * dy + F[8] * dz, F[1] * dx + F[5] * dy + F[9] * dz];
    };
    const q0 = Object.assign({}, q), e0 = rel(q0), d = 0.02;
    const qp = Object.assign({}, q0); ap.forEach(n => { qp[n] += d; }); const ep = rel(qp);
    const qr = Object.assign({}, q0); hr.forEach(n => { qr[n] += d; }); ar.forEach(n => { qr[n] -= d; }); const er = rel(qr);
    const sp = (ep[0] - e0[0]) / d, sr = (er[1] - e0[1]) / d;
    const tp = Math.abs(sp) > 1e-3 ? clampN(-e0[0] / sp, -0.35, 0.35) : 0, tr = Math.abs(sr) > 1e-3 ? clampN(-e0[1] / sr, -0.3, 0.3) : 0;
    ap.forEach(n => { rb.qt[rb.ji[n]] += tp; }); hr.forEach(n => { rb.qt[rb.ji[n]] += tr; }); ar.forEach(n => { rb.qt[rb.ji[n]] -= tr; });
  }
  upperTick(h) {
    const a = this.ovl; if (!a) return;
    a.t += h; const [d, f] = a.res[a.i], u = ease01(a.t / d);
    for (const [n, v] of Object.entries(f.q)) { if (this.legSet.has(n)) continue; this.qt[n] = lerpN(a.from[n] != null ? a.from[n] : this.qt[n], v, u); }
    if (a.t >= d) { Object.assign(a.from, this.qt); a.i++; a.t = 0; if (a.i >= a.res.length) { this.event('action_done', a.name); this.ovl = null; } }
  }
  /* ---------- falling ---------- */
  tilt() { return Math.acos(clampN(this.rb.T[0][10], -1, 1)); }
  checkFall(h) {
    if (!['stand', 'walk', 'act', 'manual'].includes(this.mode)) return;
    if (this.tilt() > this.H.fallAt) this.startFall();
  }
  startFall() {
    this.act = null; this.ovl = null; this.queue = null; this.walker.setCmd(0, 0, 0);
    this.names.forEach(n => { this.qt[n] = this.q[n]; });
    this.mode = 'fall'; this.fallT = 0; Sound.play('bump'); this.event('fall');
  }
  fallCtl(h) {
    // servos soften to protect themselves; once the body lies still the robot counts as fallen
    this.servoAll(0.35);
    const w = Math.hypot(this.rb.v[0], this.rb.v[1], this.rb.v[2]), sp = Math.hypot(this.rb.v[3], this.rb.v[4], this.rb.v[5]);
    if (this.mode === 'fall') { this.fallT = w < 0.4 && sp < 0.15 ? this.fallT + h : 0; if (this.fallT > 0.4) { this.mode = 'fallen'; Sound.play('bump'); this.event('fallen'); } }
  }
  dampCtl() { const rb = this.rb; for (const n in rb.ji) { const k = rb.ji[n]; rb.mode[k] = 2; rb.kd[k] = this.H.dampKd || 1; rb.vt[k] = 0; rb.tff[k] = 0; } }
  lowCtl() { }
  /* ---------- hoist: spring on the torso hook plus an attitude spring ---------- */
  hoistCtl(h) {
    const o = this.hoist, H = this.H, rb = this.rb, m = rb.totalMass, top = this.hookZ + H.hoistLift;
    o.t += h;
    const ph = [1.2, 1.1, 0.8];
    if (o.t > ph[o.phase]) {
      o.t = 0; o.phase++;
      if (o.phase === 2) { this.walker.reset(Object.assign({}, this.home)); this.walker.mode = 'stand'; }
      if (o.phase > 2) { this.hoist = null; this.mode = 'stand'; this.walker.reset(Object.assign({}, this.home)); this.event('stood_up'); this.event('action_done', 'stand_up'); return; }
    }
    const u = smooth01(o.t / ph[o.phase]);
    let k = 1, z;
    if (o.phase === 0) { z = lerpN(o.z0, top, u); const sp = this.hoistSpot(o.x, o.y); o.ax = lerpN(o.x, sp[0], u); o.ay = lerpN(o.y, sp[1], u); }
    else if (o.phase === 1) z = lerpN(top, this.hookZ, u);
    else { z = this.hookZ; k = 1 - u; }
    o.az = z;
    if (o.phase < 2) this.servoAll(1); else { const ms = this._meas || (this._meas = { com: new Float64Array(6), pelvis: null, omega: [0, 0, 0] }); rb.comState(ms.com); ms.pelvis = rb.T[0]; ms.omega = [rb.v[0], rb.v[1], rb.v[2]]; this.walker.setCmd(0, 0, 0); const q = this.walker.update(h, ms); for (const n in rb.ji) { const kk = rb.ji[n]; rb.mode[kk] = 1; rb.kp[kk] = this.gains.kp[kk]; rb.kd[kk] = this.gains.kd[kk]; if (this.legSet.has(n)) { rb.qt[kk] = q[n]; rb.tff[kk] = this.walker.ff[n] || 0; rb.vt[kk] = (this.walker.qd && this.walker.qd[n]) || 0; } else { rb.qt[kk] = this.qt[n]; rb.tff[kk] = 0; rb.vt[kk] = 0; } } }
    // translational spring-damper at the hook
    const p = this.hookPos(), v = rb.ptVel(this.torsoB, p), Ks = H.hoistK * m * k, Cs = 2 * Math.sqrt(H.hoistK * m * m) * 0.9 * Math.max(k, 0.25);
    rb.push(this.torsoB, this.hookLocal(), [Ks * (o.ax - p[0]) - Cs * v[0], Ks * (o.ay - p[1]) - Cs * v[1], Ks * (z - p[2]) - Cs * v[2]]);
    // attitude: turn the torso upright with the chosen heading
    const T = rb.T[0], R = K3.fromRPY(0, 0, 0, 0, 0, o.yaw), e = [0, 0, 0];
    for (let c = 0; c < 3; c++) { const ex = T[c], ey = T[4 + c], ez = T[8 + c], tx = R[c], ty = R[4 + c], tz = R[8 + c]; e[0] += 0.5 * (ey * tz - ez * ty); e[1] += 0.5 * (ez * tx - ex * tz); e[2] += 0.5 * (ex * ty - ey * tx); }
    const Kr = H.hoistKr * m * k, Cr = H.hoistCr * m * Math.max(k, 0.25);
    rb.pushTorque(0, [Kr * e[0] - Cr * rb.v[0], Kr * e[1] - Cr * rb.v[1], Kr * e[2] * 0.5 - Cr * rb.v[2]]);
  }
  /* ---------- pushes and dragging ---------- */
  push(strength) {
    if (!['stand', 'walk', 'act', 'manual'].includes(this.mode)) return;
    const yaw = this.nav.yaw, a = yaw + (Math.random() < 0.5 ? 0 : Math.PI) + (Math.random() - 0.5) * 0.8, dv = this.H.pushDv * strength, dur = 0.05, F = dv * this.rb.totalMass / dur;
    this.pushF = [F * Math.cos(a), F * Math.sin(a), 0]; this.pushLeft = dur; this.pushT = this.t; this.event('push', strength);
  }
  applyPayload(kg) { const I = kg * 0.004; this.rb.setPayload(this.torsoB, kg, [-0.02, 0, this.H.hook[1][2] * 0.6], [I, 0, 0, 0, I, 0, 0, 0, I]); }
  drDefaults() { return { mu: 1, mass: 0, motor: 1, noise: 0 }; }
  /* ---------- state read-back ---------- */
  readState(dt) {
    const rb = this.rb, T = rb.T[0];
    this.names.forEach(n => { const k = rb.ji[n]; if (k == null) return; this.q[n] = rb.q[k]; this.dq[n] = rb.v[6 + k]; this.tau[n] = rb.tau[k]; });
    this.Tr = new Float64Array(T); this.kin.fk(this.q, this.Tr);
    const yaw = Math.atan2(T[4], T[0]); this.nav = { x: T[3], y: T[7], yaw };
    const c = Math.cos(yaw), s = Math.sin(yaw), vb = [c * rb.v[3] + s * rb.v[4], -s * rb.v[3] + c * rb.v[4], rb.v[2]];
    if (dt > 0) this.v = this.v.map((x, i) => x + (vb[i] - x) * 0.2); else this.v = [0, 0, 0];
    this.phys.contacts = this.world.nc;
    if (dt > 0 && this.every('pbuf', dt, 0.05)) {
      const P = this.pbuf || (this.pbuf = []), bl = rb.linkBody[this.H.sole.l], br = rb.linkBody[this.H.sole.r];
      P.push([this.t, [rb.cfz[bl], rb.cfz[br], rb.totalMass * 9.81]]); if (P.length > 130) P.shift();
    }
  }
  plotSpec() { return { title: 'ground force on each foot', labels: ['left foot', 'right foot', 'body weight m·g'], unit: 'N', lo: 0 }; }
  physDoc() {
    return `Every link has mass and inertia (≈ ${this.rb.totalMass.toFixed(1)} kg in total) and every joint is a torque-limited servo, solved by a rigid-body engine at ${Math.round(1 / this.world.h)} Hz with soft contacts and Coulomb friction. `
      + 'Walking is model-based: footsteps from <code>/cmd_vel</code>, a centre-of-mass plan from the linear inverted pendulum (DCM), and a stabilizer that shifts the body or the next footstep when the robot is pushed. '
      + 'Hold <kbd>Ctrl</kbd> and drag a body part (or use <b>Grab</b>) to shove it like in MuJoCo. Randomize the world to see how robust the controller is.';
  }
  objStep() {
    if (!this.objs) return;
    for (const o of this.objs) { const S = o.sys; o.x = S.pos[0]; o.y = S.pos[1]; o.z = S.pos[2]; o.vx = S.v[3]; o.vy = S.v[4]; this.objTick(o); }
  }
  objTick() { }
  /* ---------- publishing ---------- */
  publish(dt) {
    const c = this.ctl, H = this.H;
    if (this.every('js', dt, 0.04)) { const m = this.jointStateMsg(this.names, this.q, this.dq); m.effort = this.names.map(n => +(this.tau[n] || 0).toFixed(3)); this.pub(c, '/joint_states', 'sensor_msgs/msg/JointState', m); }
    if (this.every('imu', dt, 0.02)) this.pubImu();
    if (this.every('odom', dt, 0.05)) {
      const T = this.Tr, q = K3.quat(T), st = stampNow();
      this.pub(c, '/odom', 'nav_msgs/msg/Odometry', { header: { stamp: st, frame_id: 'odom' }, child_frame_id: 'base_link', pose: { pose: { position: { x: T[3], y: T[7], z: T[11] }, orientation: q }, covariance: zeros36() }, twist: { twist: { linear: { x: +this.v[0].toFixed(4), y: +this.v[1].toFixed(4), z: 0 }, angular: { x: 0, y: 0, z: +this.v[2].toFixed(4) } }, covariance: zeros36() } });
      this.pub(c, '/tf', 'tf2_msgs/msg/TFMessage', this.tfMsg([['odom', 'base_link', T]]));
    }
    const st = this.modeLabel();
    if (st !== this.lastState || this.every('state', dt, 0.5)) { this.lastState = st; this.pub(c, H.stateTopic, 'std_msgs/msg/String', { data: this.stateString() }); }
    this.publishExtra(dt);
  }
  publishExtra() { }
  stateString() { return this.mode === 'walk' ? 'walking' : this.mode === 'act' && this.act ? 'action:' + this.act.name : this.mode === 'getup' ? 'getting_up' : this.mode === 'stand' ? 'standing' : this.mode; }
  // IMU from the physics state: gyro in the body frame, accelerometer = specific force (acceleration minus gravity)
  pubImu() {
    const rb = this.rb, T = rb.T[0], q = K3.quat(T), now = this.t, w = [rb.v[0], rb.v[1], rb.v[2]], v = [rb.v[3], rb.v[4], rb.v[5]], p = this.imuPrev, n = this.noise ? this.noise.imu : 0;
    const dt = p ? Math.max(1e-3, now - p.t) : 1, a = p ? [(v[0] - p.v[0]) / dt, (v[1] - p.v[1]) / dt, (v[2] - p.v[2]) / dt + 9.81] : [0, 0, 9.81]; this.imuPrev = { t: now, v };
    const wb = K3.rot(K3.inv(T), w[0], w[1], w[2]), ab = K3.rot(K3.inv(T), a[0], a[1], a[2]), rn = () => n * (Math.random() * 2 - 1);
    this.pub(this.ctl, '/imu', 'sensor_msgs/msg/Imu', { header: { stamp: stampNow(), frame_id: 'imu_link' }, orientation: q, orientation_covariance: new Array(9).fill(0), angular_velocity: { x: +(wb[0] + rn() * 0.05).toFixed(4), y: +(wb[1] + rn() * 0.05).toFixed(4), z: +(wb[2] + rn() * 0.05).toFixed(4) }, angular_velocity_covariance: new Array(9).fill(0), linear_acceleration: { x: +(ab[0] + rn()).toFixed(3), y: +(ab[1] + rn()).toFixed(3), z: +(ab[2] + rn()).toFixed(3) }, linear_acceleration_covariance: new Array(9).fill(0) });
  }
  frames() { return ['odom', ...this.kin.links]; }
  lookup(a, b) { const T = this.kin.T, A = a === 'odom' ? K3.I() : T[a], B = b === 'odom' ? K3.I() : T[b]; return A && B ? K3.mul(K3.inv(A), B) : null; }
  modeLabel() { return { stand: 'Standing', walk: 'Walking', act: 'Action', manual: 'Posing (servo mode)', fall: 'Falling', fallen: 'Fallen', getup: 'Standing up (hoist)', damp: 'Damped', low: 'Low-level (gantry)' }[this.mode] || this.mode; }
  focusPos() { const T = this.Tr; return [T[3], T[7], Math.max(this.H.camZ, T[11] * 0.6)]; }
  focusYaw() { return this.nav.yaw; }
  tickStats(dt) {
    const s = this.stats, n = this.nav, p = this._pn || { x: n.x, y: n.y, yaw: n.yaw };
    if (this.mode === 'walk') {
      const dx = n.x - p.x, dy = n.y - p.y, f = Math.cos(n.yaw) * dx + Math.sin(n.yaw) * dy, l = -Math.sin(n.yaw) * dx + Math.cos(n.yaw) * dy, d = Math.hypot(dx, dy);
      s.dist += Math.max(0, f); s.turn += Math.abs(wrapPi(n.yaw - p.yaw)); s.side += Math.abs(l);
      if (this.cmdLive(1) && this.cmd.kind === 'cli') s.cliDist += d; if (this.cmdLive(1) && this.cmd.kind === 'user') s.userDist += d;
    }
    this._pn = { x: n.x, y: n.y, yaw: n.yaw };
    this.missionTick(dt);
  }
  missionTick() { }
  /* ---------- 3D ---------- */
  buildScene(view, T) {
    this.view3 = view; const H = this.H;
    this.model = snoBuildModel(T, this.kin, this.desc.visuals, view.mats); view.world.add(this.model.root);
    if (this.def.cams) view.addPip(this.model.links.camera_link, { label: H.camLabel, vfov: H.camVfov, near: 0.01, far: 30, hide: [] });
    this.buildEnv(view, T);
    if (!this.gantryMesh) this.buildHoist(view, T);
    snoPhysViz(this, view, T);
  }
  buildHoist(view, T) {
    const g = this.hoistMesh = new T.Group(), s = this.H.hoistScale || 1, mats = view.mats;
    [-0.6, 0.6].forEach(y => { g.add(snoMesh(T, { s: 'box', size: [0.05 * s, 0.05 * s, 1.6 * s], xyz: [0, y * s, 0.8 * s], m: 'yellow' }, mats)); g.add(snoMesh(T, { s: 'box', size: [0.45 * s, 0.07 * s, 0.04 * s], xyz: [0, y * s, 0.02 * s], m: 'dark' }, mats)); });
    g.add(snoMesh(T, { s: 'box', size: [0.07 * s, 1.27 * s, 0.07 * s], xyz: [0, 0, 1.6 * s], m: 'yellow' }, mats));
    this.ropeMesh = snoMesh(T, { s: 'cyl', size: [0.004 * s + 0.002, 1], m: 'dark' }, mats); g.add(this.ropeMesh);
    g.visible = false; view.world.add(g);
  }
  syncHoist() {
    const g = this.hoistMesh; if (!g) return;
    const o = this.hoist || this.gantry, on = !!o && !(this.hoist && this.hoist.phase === 2 && this.hoist.t > 0.5);
    g.visible = on; if (!on) return;
    const s = this.H.hoistScale || 1, p = this.hookPos(), top = 1.6 * s - 0.02;
    g.position.set(o.ax, o.ay, 0); g.rotation.z = (o.yaw || 0) + Math.PI / 2;
    const L = Math.max(0.02, top - p[2]); this.ropeMesh.scale.set(1, 1, L); this.ropeMesh.position.set(0, 0, p[2] + L / 2);
  }
  buildEnv() { }
  sync() { if (this.model) this.model.pose(this.q, this.Tr); this.syncEnv(); this.syncHoist(); if (this.physViz) this.physViz.sync(); }
  syncEnv() { }
  /* ---------- keyboard ---------- */
  onKey(e, up, release) {
    if (release) { this.keys = {}; if (this._tele && (this._tele.vx || this._tele.vy || this._tele.wz)) this.teleop(0, 0, 0); return false; }
    const map = { ArrowUp: 'f', w: 'f', W: 'f', ArrowDown: 'b', s: 'b', S: 'b', ArrowLeft: 'l', a: 'l', A: 'l', ArrowRight: 'r', d: 'r', D: 'r', q: 'sl', Q: 'sl', e: 'sr', E: 'sr' };
    const k = map[e.key]; if (!k) return false;
    this.keys = this.keys || {}; this.keys[k] = !up;
    const K = this.keys, g = this.gait(), sp = this.speed || g.maxV * 0.7;
    this.teleop(((K.f ? 1 : 0) - (K.b ? 1 : 0)) * sp, ((K.sl ? 1 : 0) - (K.sr ? 1 : 0)) * g.maxVy * 0.8, ((K.l ? 1 : 0) - (K.r ? 1 : 0)) * g.maxW * 0.6);
    return true;
  }
  hud() {
    const n = this.nav, d = (v) => (v * 180 / Math.PI).toFixed(0);
    return `<span>${esc(this.modeLabel())}${this.act ? ': ' + esc(this.act.name) : ''}${this.ovl ? ' + ' + esc(this.ovl.name) : ''}</span><span>x ${n.x.toFixed(2)}  y ${n.y.toFixed(2)}  θ ${d(n.yaw)}°</span><span>v ${this.v[0].toFixed(2)} m/s  ω ${this.v[2].toFixed(2)}</span><span>steps ${this.walker.stepsDone}</span>${this.hudExtra()}`;
  }
  hudExtra() { return ''; }
  /* ---------- panels ---------- */
  renderTab(tab, body) { if (tab === 'control') this.tabControl(body); else if (tab === 'joints') this.tabJoints(body); }
  tabControl(body) {
    const g = this.gait(); this.speed = this.speed || +(g.maxV * 0.7).toFixed(2);
    const w = h('div', { class: 'sno-ctl' });
    if (this.modesUI) w.append(this.modesUI());
    const jw = h('div', { class: 'joy-wrap', html: `<div class="joy" role="application" aria-label="Joystick: up/down walks, left/right turns"><div class="knob"></div></div>
      <div><div class="dpad" aria-label="Walk buttons"><button data-d="sl" aria-label="Side-step left">${icon('arrowL')}<small>side</small></button><button data-d="f" aria-label="Walk forward">${icon('arrowU')}</button><button data-d="sr" aria-label="Side-step right">${icon('arrowR')}<small>side</small></button><button data-d="l" aria-label="Turn left">${icon('replay')}</button><button data-d="s" aria-label="Stop">${icon('stop')}</button><button data-d="r" aria-label="Turn right" class="mirror">${icon('replay')}</button><span></span><button data-d="b" aria-label="Walk backward">${icon('arrowD')}</button><span></span></div>
      <div class="sliders" style="margin-top:12px"><div class="slider"><label for="hs${this.id}">speed</label><input id="hs${this.id}" type="range" min="0.02" max="${g.maxV}" step="0.01" value="${this.speed}"><output>${this.speed.toFixed(2)}</output></div></div></div>` });
    w.append(jw);
    const sp = $('input', jw); sp.addEventListener('input', () => { this.speed = +sp.value; $('output', jw).textContent = this.speed.toFixed(2); });
    snoJoystick($('.joy', jw), (fy, fx) => this.teleop(+(fy * this.speed).toFixed(3), 0, +(fx * g.maxW * 0.8).toFixed(3)), () => this.teleop(0, 0, 0));
    $$('.dpad button', jw).forEach(b => { const dd = b.dataset.d, v = { f: [1, 0, 0], b: [-0.6, 0, 0], l: [0, 0, 0.7], r: [0, 0, -0.7], sl: [0, 0.8, 0], sr: [0, -0.8, 0], s: [0, 0, 0] }[dd]; snoHold(b, () => this.teleop(v[0] * this.speed, v[1] * g.maxVy, v[2] * g.maxW), () => this.teleop(0, 0, 0)); });
    w.append(h('div', { class: 'twist-readout sno-read' }));
    const ag = h('div', { class: 'act-grid', 'aria-label': 'Action groups' });
    const acts = Object.entries(HU_ACTIONS).filter(([, a]) => !a.robots || a.robots.includes(this.id));
    acts.forEach(([k, a]) => { const b = h('button', { class: 'act', html: icon(a.icon) + esc(a.label) }); b.addEventListener('click', () => { const r = this.playAction(k); if (r.ok) { Sound.play('tap'); this.event('ui_action', k); } else toast(esc(r.reason), { mood: 'think' }); }); ag.append(b); });
    this.userActs.forEach(a => { const b = h('button', { class: 'act user', html: icon('play') + esc(a.name) }); b.addEventListener('click', () => { const r = this.playAction(a.name); if (r.ok) this.event('ui_action', a.name); else toast(esc(r.reason), { mood: 'think' }); }); ag.append(b); });
    w.append(h('h4', { html: `Action groups <span>publish <code>${this.H.actionTopic}</code></span>` }), ag);
    const bal = h('div', { class: 'row-btns' });
    const pb = h('button', { class: 'btn btn-sm', html: icon('bolt') + 'Push' }); pb.addEventListener('click', () => this.push(1));
    const bb = h('button', { class: 'btn btn-sm btn-bad', html: icon('bolt') + 'Big push' }); bb.addEventListener('click', () => this.push(3.2));
    const sb = h('button', { class: 'btn btn-sm btn-good', html: icon('arrowU') + 'Stand up' }); sb.addEventListener('click', () => { const r = this.standUp(); if (!r.ok) toast(esc(r.reason), { mood: 'think' }); }); sb.title = 'A hoist lifts the robot and sets it back on its feet';
    bal.append(pb, bb, sb);
    w.append(h('h4', { html: 'Balance <span>DCM feedback + capture steps</span>' }), bal, snoSwitch('Balance controller', this.P(this.ctl, 'balance') !== false, (on) => { this.setP(this.ctl, 'balance', on); this.event('param', { node: this.ctl, k: 'balance', v: on }); }, 'Off: no centre-of-mass feedback and no recovery steps, so a push that it could normally catch knocks it over.'));
    if (this.extraControls) this.extraControls(w);
    body.append(h('div', { class: 'lpad' }, w));
    this.tickTab('control');
  }
  tickTab(tab) {
    const root = SnoUI.root; if (!root) return;
    if (tab === 'control') { const el = $('.sno-read', root); if (el) el.innerHTML = this.readout(); if (this.tickControlExtra) this.tickControlExtra(root); }
    if (tab === 'joints') this.tickJoints(root);
  }
  readout() {
    const T = this.bus.topics.get('/cmd_vel'), act = this.bus.active('/cmd_vel'), m = T && T.last;
    return `state <b>${esc(this.stateString())}</b>  ·  /cmd_vel ${act ? '<span style="color:var(--term-green)">● publishing</span>' : '<span style="color:var(--term-dim)">○ idle</span>'}<br>linear.x <b>${m ? pyFloat(+(+m.linear.x).toFixed(2)) : '0.0'}</b>  linear.y <b>${m ? pyFloat(+(+m.linear.y).toFixed(2)) : '0.0'}</b>  angular.z <b>${m ? pyFloat(+(+m.angular.z).toFixed(2)) : '0.0'}</b>${this.lastWarn ? `<br><span style="color:var(--term-yellow)">${esc(this.lastWarn)}</span>` : ''}`;
  }
  tabJoints(body) {
    const wrap = h('div', { class: 'lpad sno-joints' });
    if (this.pdPanel && this.mode === 'low') wrap.append(this.pdPanel());
    wrap.append(h('p', { class: 'sidecap', style: 'margin-top:0', html: this.mode === 'low' ? 'Sliders set each motor\'s target <code>q</code>; the PD loop above decides how hard it pulls.' : `Sliders send position targets like <code>/joint_commands</code>. Moving a leg switches the robot to <b>servo mode</b> (no walking); press <b>Stand</b> to go back.` }));
    const tools = h('div', { class: 'row-btns' });
    const st = h('button', { class: 'btn btn-sm btn-good', html: icon('arrowU') + 'Stand' }); st.addEventListener('click', () => { const r = this.goStand(); if (r && r.ok === false) toast(esc(r.reason), { mood: 'think' }); });
    const zr = h('button', { class: 'btn btn-sm', html: icon('replay') + 'Home arms' }); zr.addEventListener('click', () => { this.names.filter(n => !this.legSet.has(n)).forEach(n => { this.qt[n] = this.home[n]; }); });
    tools.append(st, zr); wrap.append(tools);
    this.H.groups.forEach(([title, keys]) => {
      const g = h('div', { class: 'jgroup' }); g.append(h('h4', {}, title));
      keys.forEach(k => {
        const n = this.jn(k); if (!n) return; const j = this.kin.J[n], id = 'j' + (++UID);
        const row = h('div', { class: 'slider jrow', html: `<label for="${id}" title="${n}">${esc(n)}</label><input id="${id}" type="range" min="${j.lo.toFixed(3)}" max="${j.hi.toFixed(3)}" step="0.01" value="${(this.qt[n] || 0).toFixed(3)}" data-j="${n}"><output>${Math.round((this.q[n] || 0) * 57.3)}°</output>` });
        $('input', row).addEventListener('input', (e) => this.sliderMove(n, +e.target.value));
        g.append(row);
      });
      wrap.append(g);
    });
    // action recorder
    const rec = h('div', { class: 'recorder' });
    rec.innerHTML = `<h4>Action group recorder <span>pose the robot, then add frames</span></h4><div class="row-btns"><label class="mini">frame time <input type="number" min="0.1" max="3" step="0.1" value="${this.recT}" aria-label="Frame duration in seconds"> s</label><button class="btn btn-sm" data-r="add">${icon('plus')}Add frame</button><button class="btn btn-sm btn-good" data-r="play">${icon('play')}Play</button><button class="btn btn-sm btn-ghost" data-r="clear">Clear</button></div><ol class="frames"></ol><div class="row-btns"><button class="btn btn-sm btn-primary" data-r="save">Save as action</button><button class="btn btn-sm btn-ghost" data-r="copy">${icon('copy')}Copy as Python</button></div>`;
    const fl = $('.frames', rec), drawF = () => { fl.innerHTML = this.recorder.map((f, i) => `<li>Frame ${i + 1} · ${f[0].toFixed(1)} s · ${Object.keys(f[1]).length} joints</li>`).join('') || '<li class="empty">No frames yet.</li>'; };
    drawF();
    $('input', rec).addEventListener('change', (e) => { this.recT = clampN(+e.target.value || 0.6, 0.1, 3); });
    $('[data-r="add"]', rec).addEventListener('click', () => { const q = {}; this.names.forEach(n => { q[n] = +(this.qt[n] || 0).toFixed(3); }); this.recorder.push([this.recT, q]); drawF(); Sound.play('tap'); this.event('rec_add', this.recorder.length); });
    $('[data-r="clear"]', rec).addEventListener('click', () => { this.recorder = []; drawF(); });
    $('[data-r="play"]', rec).addEventListener('click', () => { if (this.recorder.length < 1) return toast('Add at least one frame first.', { mood: 'think' }); const a = { name: 'recorded', frames: this.recorder.map(([d, q]) => [d, q]), user: true }; a.legs = a.frames.some(([, q]) => Object.keys(q).some(n => this.legSet.has(n))); if (this.mode === 'low' || this.mode === 'fallen' || this.mode === 'damp') return toast('Stand the robot up first.', { mood: 'think' }); if (a.legs) this.startAct(a); else this.ovl = this.mkAct(a); this.event('rec_play', this.recorder.length); });
    $('[data-r="save"]', rec).addEventListener('click', () => {
      if (this.recorder.length < 1) return toast('Add at least one frame first.', { mood: 'think' });
      const nm = 'my_action_' + (this.userActs.length + 1); this.userActs.push({ name: nm, frames: this.recorder.slice() }); S.sno.actions[this.id] = this.userActs.slice(-8); save({ local: true });
      toast(`Saved <b>${nm}</b>. Play it from the Control tab or with:<br><code>ros2 topic pub --once ${this.H.actionTopic} std_msgs/msg/String "{data: ${nm}}"</code>`, { mood: 'happy', ms: 5200 });
    });
    $('[data-r="copy"]', rec).addEventListener('click', () => { if (!this.recorder.length) return toast('Add a frame first.', { mood: 'think' }); copyText(this.recorderPy()); toast('Copied a Python node that plays your frames.', { mood: 'happy' }); });
    wrap.append(rec);
    body.append(wrap);
  }
  sliderMove(n, v) {
    if (this.mode === 'low' && this.pd) { this.pd.qd[n] = v; return; }
    if (this.mode === 'fall' || this.mode === 'getup') return;
    this.qt[n] = v;
    if (this.legSet.has(n) && (this.mode === 'stand' || this.mode === 'walk' || this.mode === 'act')) { this.act = null; this.enterManual(); this.qt[n] = v; }
    this.event('slider', n);
  }
  tickJoints(root) {
    $$('.jrow', root).forEach(r => { const inp = $('input', r), n = inp.dataset.j; if (document.activeElement !== inp) inp.value = (this.mode === 'low' && this.pd ? this.pd.qd[n] : this.qt[n]) || 0; $('output', r).textContent = Math.round((this.q[n] || 0) * 57.3) + '°'; });
    if (this.tickPd) this.tickPd(root);
  }
  recorderPy() {
    const fr = this.recorder.map(([d, q]) => `    (${d.toFixed(1)}, {${Object.entries(q).filter(([n]) => Math.abs(q[n] - (this.home[n] || 0)) > 0.005 || this.legSet.has(n)).map(([n, v]) => `'${n}': ${v.toFixed(3)}`).join(', ')}}),`).join('\n');
    return `import rclpy\nfrom rclpy.node import Node\nfrom sensor_msgs.msg import JointState\n\nFRAMES = [\n${fr}\n]\n\n\nclass Player(Node):\n    def __init__(self):\n        super().__init__('action_player')\n        self.pub = self.create_publisher(JointState, '/joint_commands', 10)\n        self.i = 0\n        self.timer = self.create_timer(FRAMES[0][0], self.next_frame)\n        self.next_frame()\n\n    def next_frame(self):\n        if self.i >= len(FRAMES):\n            self.get_logger().info('done')\n            self.timer.cancel()\n            return\n        seconds, pose = FRAMES[self.i]\n        msg = JointState()\n        msg.name = list(pose.keys())\n        msg.position = [float(v) for v in pose.values()]\n        self.pub.publish(msg)\n        self.get_logger().info(f'frame {self.i + 1}')\n        self.i += 1\n\n\ndef main():\n    rclpy.init()\n    rclpy.spin(Player())\n\n\nmain()\n`;
  }
}
