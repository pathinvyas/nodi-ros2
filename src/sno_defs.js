/* ============================== SnoBots: robot definitions ============================== */
const HU_TABS = [['control', 'drive', 'Control'], ['joints', 'sliders', 'Joints'], ['phys', 'chip', 'Physics'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions'], ['info', 'book', 'Info']];
const HU_KBD = 'Walk: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> · side-step <kbd>Q</kbd><kbd>E</kbd> · drag to orbit';
const CAR_KBD = 'Drive: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> · slide <kbd>Q</kbd><kbd>E</kbd> · click the floor = goal';
const legs6 = (p, n) => n.map(x => p + x);
const PY_HEAD = 'import math\nimport rclpy\nfrom rclpy.node import Node\n';
const PY_MAIN = (cls) => `\n\ndef main():\n    rclpy.init()\n    rclpy.spin(${cls}())\n\n\nmain()\n`;

function humanoidExe(sim, T, pkg, exe) {
  const acts = Object.keys(HU_ACTIONS);
  if (!acts.includes(exe)) return T.print('This executable is not simulated here.', 'w');
  const node = '/action_client', topic = sim.H.actionTopic; sim.bus.addNode(node, 'cli'); sim.bus.advertise(node, topic, 'std_msgs/msg/String');
  T.print(`[INFO] [${(Date.now() / 1000).toFixed(3)}] [action_client]: requesting action '${exe}' on ${topic}`);
  sim.pub(node, topic, 'std_msgs/msg/String', { data: exe });
  setTimeout(() => sim.bus.removeNode(node), 300);
  T.print(sim.lastWarn && sim.lastWarn.includes(exe) ? `[WARN] ${sim.lastWarn}` : `[INFO] [action_client]: done`, sim.lastWarn && sim.lastWarn.includes(exe) ? 'w' : 'o');
}

const SNO_DEFS = {
  ainex: {
    id: 'ainex', name: 'AiNex', maker: 'Hiwonder', catLabel: 'Humanoid', color: '#E5532B', hz: 60, cams: true,
    blurb: 'A 41.5 cm, 24-joint ROS humanoid with a camera head. Walk it, pose it, record action groups, track colours and kick a ball.',
    chipsHub: ['24 DOF', '41.5 cm', 'head camera'], tabs: HU_TABS, kbd: HU_KBD, keysHint: 'W A S D walk, Q E side-step.', teleop: [0.1, 0.6],
    mainNodes: ['/ainex_controller', '/color_detector'],
    make: (d) => new AinexSim(d),
    view: { dist: 1.15, az: -2.45, el: 0.4, target: [-0.35, 0, 0.16], minDist: 0.3, maxDist: 5, shadow: 1.7, grid: null, fog: [5, 14] },
    hu: {
      desc: huAinex, ctl: '/ainex_controller', legRe: /hip|knee|ank/, sole: { l: 'l_sole_link', r: 'r_sole_link' }, cameraParent: 'head_link',
      map: { hy: 'hip_yaw', hr: 'hip_roll', hp: 'hip_pitch', kn: 'knee', ap: 'ank_pitch', ar: 'ank_roll', sp: 'sho_pitch', sr: 'sho_roll', ep: 'el_pitch', ey: 'el_yaw', gr: 'gripper', pan: 'head_pan', tilt: 'head_tilt' },
      side: { L: 'l_', R: 'r_' }, homeExtra: { tilt: 0.32 },
      order: [...legs6('l_', ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll']), ...legs6('r_', ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll']), ...legs6('l_', ['sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw', 'gripper']), ...legs6('r_', ['sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw', 'gripper']), 'head_pan', 'head_tilt'],
      standZ: 0.222, footY: 0.036, footL: 0.11, footW: 0.064, start: [-0.35, 0, 0], phys: huAinexPhys,
      walk: { Tss: 0.32, Tds: 0.08, T0: 0.3, stepH: 0.025, zmpIn: 0.008, zmpX: 0, maxFwd: 0.085, maxBack: 0.05, minSep: 0.92, maxSide: 0.045, maxYaw: 0.5, maxYawIn: 0.15, acc: [0.4, 0.3, 2.0], dz: 0.004, land: 0.003, pushStep: 0.025, horizon: 3, zmpMargin: 0.015 },
      fb: { kc: 1.0, kstep: 1.0, dead: 0.012 }, hook: ['torso_link', [0, 0, 0.125]], hoistLift: 0.06, hoistK: 400, hoistKr: 4, hoistCr: 0.5, hoistScale: 0.32, pushDv: 0.45, fallAt: 0.65, ankleBal: [0.8, 0.8],
      params: [['max_linear_speed', 0.15, 'double', [0.02, 0.21]], ['max_angular_speed', 0.8, 'double', [0.1, 1.2]], ['cmd_timeout', 0.8, 'double', [0.1, 5]], ['step_height', 0.025, 'double', [0.008, 0.045]], ['step_period', 0.42, 'double', [0.3, 0.8]], ['body_height', 0.222, 'double', [0.195, 0.232]], ['arm_swing', true, 'bool'], ['balance', true, 'bool']],
      actionTopic: '/action_group', stateTopic: '/walking/state', standSrv: '/stand_up',
      gait: { stepH: 0.025, maxV: 0.21, maxVy: 0.06, maxW: 0.9, arm: 0.28 },
      camZ: 0.15, camLabel: 'head camera · 120° wide', camVfov: 80, hfov: 110 * Math.PI / 180,
      groups: [['Head', ['pan', 'tilt']], ['Left arm', ['L.sp', 'L.sr', 'L.ep', 'L.ey', 'L.gr']], ['Right arm', ['R.sp', 'R.sr', 'R.ep', 'R.ey', 'R.gr']], ['Left leg', ['L.hy', 'L.hr', 'L.hp', 'L.kn', 'L.ap', 'L.ar']], ['Right leg', ['R.hy', 'R.hr', 'R.hp', 'R.kn', 'R.ap', 'R.ar']]],
      objs: [{ kind: 'ball', x: 0.3, y: 0.06, z: 0.035, r: 0.035 }, { kind: 'block', color: 'red', x: 0.85, y: 0.42, z: 0.025, r: 0.03, size: 0.05 }, { kind: 'block', color: 'green', x: 0.55, y: -0.6, z: 0.025, r: 0.03, size: 0.05 }, { kind: 'block', color: 'blue', x: -0.6, y: 0.55, z: 0.025, r: 0.03, size: 0.05 }],
    },
    pkgs: { ainex_demos: ['wave', 'bow', 'squat', 'raise_arms', 'left_kick', 'right_kick', 'nod', 'shake', 'color_tracking.launch.py', 'color_reaction.launch.py'] },
    chips: ['ros2 node list', 'ros2 topic echo /walking/state', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"', 'ros2 topic pub --once /action_group std_msgs/msg/String "{data: wave}"', 'ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [head_pan], position: [0.8]}"', 'ros2 topic echo /color_detection', 'ros2 param set /color_detector target_color any', 'ros2 launch ainex_demos color_tracking.launch.py', 'ros2 run tf2_ros tf2_echo base_link camera_link'],
    help: `AiNex simulator terminal (ROS 2 Jazzy).
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.3}}"
  ros2 topic pub --once /action_group std_msgs/msg/String "{data: wave}"     (bow, squat, left_kick, nod, …)
  ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [r_sho_pitch], position: [-2.5]}"
  ros2 topic echo /joint_states | /imu --once | /color_detection | /walking/state
  ros2 param set /ainex_controller step_height 0.035      ros2 param set /color_detector target_color blue
  ros2 service call /stand_up std_srvs/srv/Trigger        ros2 service call /sim/push std_srvs/srv/Trigger
  ros2 launch ainex_demos color_tracking.launch.py        ros2 run ainex_demos left_kick
  ros2 run tf2_ros tf2_echo base_link l_sole_link         ros2 run teleop_twist_keyboard teleop_twist_keyboard
Keys: Up/Down history · Tab completes · Ctrl+C stops · Ctrl+L clears`,
    missions: [
      { id: 'm1', title: 'First steps', text: 'Walk AiNex forward 1 m with the joystick, the arrow buttons or W/A/S/D.', tab: 'control', xp: 15 },
      { id: 'm2', title: 'Turn around', text: 'Turn 180° while walking (joystick left or right, or A/D).', tab: 'control', xp: 15 },
      { id: 'm3', title: 'Crab walk', text: 'Side-step 30 cm with the side buttons or Q/E. That is `linear.y` on /cmd_vel.', tab: 'control', xp: 15 },
      { id: 'm4', title: 'Say hello', text: 'Play the **Wave** action group, or publish `wave` on /action_group.', tab: 'control', xp: 10 },
      { id: 'm5', title: 'Pose master', text: 'In the Joints tab, record an action group with at least 2 frames and play it.', tab: 'joints', xp: 25 },
      { id: 'm6', title: 'Eyes on red', text: 'Turn on head tracking and keep the red block centred in the camera for 3 s.', tab: 'control', xp: 20 },
      { id: 'm7', title: 'Goal!', text: 'Walk or kick the orange ball into the goal. The kick actions help.', tab: 'control', xp: 30 },
      { id: 'm8', title: 'Terminal walker', text: 'Walk 0.5 m using only `ros2 topic pub` on /cmd_vel.', tab: 'term', xp: 20 },
      { id: 'm9', title: 'Back on your feet', text: 'Knock AiNex over (Big push, or turn balance off first), then press **Stand up**: the hoist sets it back on its feet.', tab: 'control', xp: 20 },
      { id: 'm10', title: 'Code it', text: 'Run a Python node that publishes /joint_commands or /cmd_vel. The wave template is a good start.', tab: 'code', xp: 30 },
    ],
    specs: [['Maker', 'Hiwonder'], ['Type', 'Small ROS humanoid'], ['Size', '193 × 135 × 415 mm'], ['Joints', '24 (legs 12, arms + grippers 10, head 2)'], ['Actuators', 'Serial bus servos'], ['Controller', 'Raspberry Pi 4B / 5'], ['Camera', '120° wide-angle, 2-DOF head'], ['IMU', '3-axis gyro, accelerometer, compass'], ['Walking', 'Inverse-kinematics gait'], ['Software', 'ROS, Gazebo, action-group editor']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Walking velocity: `linear.x` forward, `linear.y` sideways, `angular.z` turn'], ['/joint_commands', 'subscribe', 'sensor_msgs/msg/JointState', 'Joint targets by name (radians). Leg joints switch to servo mode'], ['/action_group', 'subscribe', 'std_msgs/msg/String', 'wave, bow, squat, raise_arms, left_kick, right_kick, nod, shake, grab, or your saved actions'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'All 24 joints, 25 Hz'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Orientation, rates, acceleration, 50 Hz'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Walking odometry, 20 Hz'], ['/walking/state', 'publish', 'std_msgs/msg/String', 'standing, walking, action:&lt;name&gt;, fallen, getting_up'], ['/color_detection', 'publish', 'snobots_msgs/msg/ColorDetection', 'Largest `target_color` blob in the head camera, 15 Hz'], ['/stand_up', 'service', 'std_srvs/srv/Trigger', 'Get up after a fall'], ['/sim/push', 'service', 'std_srvs/srv/Trigger', 'Simulator only: push the robot'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot, ball and blocks'], ['/ainex_controller', 'parameters', 'double / bool', '`step_height`, `step_period`, `body_height`, `max_linear_speed`, `arm_swing`, `balance`'], ['/color_detector', 'parameters', 'string / double', '`target_color` (red, green, blue, any), `min_area`']],
    jointDoc: 'Joint names: `l_`/`r_` + `hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll` (legs), `sho_pitch, sho_roll, el_pitch, el_yaw, gripper` (arms), plus `head_pan, head_tilt`. Angles are radians and follow the right-hand rule: pitch joints turn about +y (negative swings an arm forward and up), roll about +x, yaw about +z. Knees bend positive, elbows bend negative.',
    realism: 'Physics comes from the simulator\'s own rigid-body engine, built in the style of MuJoCo but not MuJoCo itself. Every link has mass and inertia, every servo is a torque-limited motor, and the feet touch the ground through contacts with friction, so a bad step can really tip the robot over. The walk is model-based: footsteps from `/cmd_vel`, a centre-of-mass plan from the linear inverted pendulum, and leg inverse kinematics solved every control tick. That is close in spirit to the real robot\'s inverse-kinematics gait, but the real walking engine has its own parameters, and real bus servos have backlash and gear play that are not modelled here. After a fall, a virtual hoist (like a lab gantry) lifts the robot back onto its feet. Topic names follow plain ROS 2 conventions and are simplified; the real kit ships its own packages, launch files and action-group editor.',
    pyHint: 'Pick a template, press Run (or Ctrl+Enter) and watch AiNex. Your node really publishes on the graph: check the Topics tab while it runs.',
    py: {
      wave: { label: 'Wave the right arm (/joint_commands)', code: PY_HEAD + `from sensor_msgs.msg import JointState


class Waver(Node):
    """Wave the right arm by publishing joint targets (radians)."""

    def __init__(self):
        super().__init__('waver')
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        self.t += 0.05
        cmd = JointState()
        cmd.name = ['r_sho_pitch', 'r_sho_roll', 'r_el_pitch']
        elbow = -0.7 + 0.45 * math.sin(5.0 * self.t)
        cmd.position = [-2.7, -0.3, elbow]
        if self.t > 6.0:
            cmd.position = [0.0, -0.14, -0.3]
            self.get_logger().info('done waving')
            self.timer.cancel()
        self.pub.publish(cmd)
` + PY_MAIN('Waver') },
      square: { label: 'Walk a square (/cmd_vel)', code: PY_HEAD + `from geometry_msgs.msg import Twist


class Square(Node):
    """Walk a square: 4 x (walk 3 s, turn 90 degrees)."""

    def __init__(self):
        super().__init__('square_walker')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t = 0.0
        self.side = 0
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        self.t += 0.1
        cmd = Twist()
        if self.t < 3.0:
            cmd.linear.x = 0.12
        elif self.t < 5.0:
            cmd.angular.z = math.pi / 4      # 90 degrees in 2 s
        else:
            self.t = 0.0
            self.side += 1
            self.get_logger().info(f'side {self.side} done')
        if self.side >= 4:
            self.get_logger().info('square complete')
            self.timer.cancel()
            cmd = Twist()
        self.pub.publish(cmd)
` + PY_MAIN('Square') },
      follow: { label: 'Walk to the red block (/color_detection)', code: PY_HEAD + `from geometry_msgs.msg import Twist
from snobots_msgs.msg import ColorDetection


class FollowRed(Node):
    """Turn toward the red block and walk to it, using the head camera."""

    def __init__(self):
        super().__init__('follow_red')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ColorDetection, '/color_detection', self.on_detect, 10)

    def on_detect(self, msg):
        cmd = Twist()
        if not msg.found:
            cmd.angular.z = 0.5              # search: turn on the spot
        else:
            cmd.angular.z = -1.0 * msg.x      # x > 0 means the block is right of centre
            if msg.area < 0.004:              # small blob = still far away
                cmd.linear.x = 0.12
            else:
                self.get_logger().info('reached the red block!')
        self.pub.publish(cmd)
` + PY_MAIN('FollowRed') },
    },
  },
  r1: {
    id: 'r1', name: 'Unitree R1', maker: 'Unitree', catLabel: 'Humanoid', color: '#2F6FD0', hz: 60, cams: true,
    blurb: 'A 1.2 m humanoid with a state machine (damp, stand, walk, run) and low-level PD control of all 26 motors while it hangs from a gantry.',
    chipsHub: ['26 joints', '1.2 m', 'PD control'], tabs: HU_TABS, kbd: HU_KBD, keysHint: 'W A S D walk, Q E side-step.', teleop: [0.5, 1.0],
    mainNodes: ['/r1_controller'],
    make: (d) => new R1Sim(d),
    view: { dist: 3.0, az: -2.45, el: 0.3, target: [0, 0, 0.6], minDist: 0.8, maxDist: 16, shadow: 3.2, grid: [40, 40], fog: [12, 40] },
    hu: {
      desc: huR1, ctl: '/r1_controller', legRe: /hip|knee|ankle/, sole: { l: 'left_sole_link', r: 'right_sole_link' }, cameraParent: 'head_link',
      map: { hp: 'hip_pitch_joint', hr: 'hip_roll_joint', hy: 'hip_yaw_joint', kn: 'knee_joint', ap: 'ankle_pitch_joint', ar: 'ankle_roll_joint', sp: 'shoulder_pitch_joint', sr: 'shoulder_roll_joint', sy: 'shoulder_yaw_joint', ep: 'elbow_joint', wr: 'wrist_roll_joint', pan: 'head_yaw_joint', tilt: 'head_pitch_joint', wy: 'waist_yaw_joint', wp: 'waist_pitch_joint' },
      side: { L: 'left_', R: 'right_' },
      order: [...legs6('left_', ['hip_pitch_joint', 'hip_roll_joint', 'hip_yaw_joint', 'knee_joint', 'ankle_pitch_joint', 'ankle_roll_joint']), ...legs6('right_', ['hip_pitch_joint', 'hip_roll_joint', 'hip_yaw_joint', 'knee_joint', 'ankle_pitch_joint', 'ankle_roll_joint']), 'waist_yaw_joint', 'waist_pitch_joint', ...legs6('left_', ['shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint', 'elbow_joint', 'wrist_roll_joint']), ...legs6('right_', ['shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint', 'elbow_joint', 'wrist_roll_joint']), 'head_yaw_joint', 'head_pitch_joint'],
      standZ: 0.66, footY: 0.085, footL: 0.21, footW: 0.09, start: [0, 0, 0], phys: huR1Phys, useMasses: true,
      walk: { Tss: 0.34, Tds: 0.06, T0: 0.3, stepH: 0.06, zmpIn: 0.02, zmpX: 0, maxFwd: 0.32, maxBack: 0.16, minSep: 0.8, maxSide: 0.14, maxYaw: 0.6, maxYawIn: 0.2, acc: [1.0, 0.6, 2.5], dz: 0.012, land: 0.002, pushStep: 0.05, horizon: 3, zmpMargin: 0.03 },
      fb: { kc: 1.0, kstep: 1.0, dead: 0.06 }, hook: ['torso_link', [0, 0, 0.32]], hoistLift: 0.14, hoistK: 400, hoistKr: 6, hoistCr: 2, hoistScale: 1.3, pushDv: 0.35, fallAt: 0.6, ankleBal: [0.4, 0.4], dampKd: 4, hangLift: 0.12,
      params: [['max_linear_speed', 0.4, 'double', [0.05, 0.45]], ['max_run_speed', 0.55, 'double', [0.3, 0.6]], ['max_angular_speed', 1.0, 'double', [0.1, 1.5]], ['cmd_timeout', 0.8, 'double', [0.1, 5]], ['step_height', 0.06, 'double', [0.03, 0.09]], ['step_period', 0.4, 'double', [0.34, 0.6]], ['body_height', 0.66, 'double', [0.62, 0.675]], ['arm_swing', true, 'bool'], ['balance', true, 'bool']],
      actionTopic: '/action', stateTopic: '/loco/state', standSrv: '/loco/stand_up',
      gait: { stepH: 0.06, maxV: 0.45, maxVy: 0.25, maxW: 1.0, arm: 0.35 },
      run: { fixed: true, Tss: 0.3, Tds: 0.06, stepH: 0.06, maxV: 0.55, maxVy: 0.2, maxW: 1.0, arm: 0.6 },
      camZ: 0.5, camLabel: 'head camera', camVfov: 60, hfov: 80 * Math.PI / 180,
      groups: [['Head & waist', ['pan', 'tilt', 'wy', 'wp']], ['Left arm', ['L.sp', 'L.sr', 'L.sy', 'L.ep', 'L.wr']], ['Right arm', ['R.sp', 'R.sr', 'R.sy', 'R.ep', 'R.wr']], ['Left leg', ['L.hp', 'L.hr', 'L.hy', 'L.kn', 'L.ap', 'L.ar']], ['Right leg', ['R.hp', 'R.hr', 'R.hy', 'R.kn', 'R.ap', 'R.ar']]],
      objs: [],
    },
    pkgs: { r1_demos: ['wave', 'bow', 'dance', 'kungfu', 'handshake', 'squat', 'raise_arms'] },
    chips: ['ros2 topic echo /loco/state', 'ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: run}"', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"', 'ros2 topic pub --once /action std_msgs/msg/String "{data: dance}"', 'ros2 service call /sim/push std_srvs/srv/Trigger', 'ros2 topic echo /imu --once', 'ros2 interface show snobots_msgs/msg/LowCmd', 'ros2 run tf2_ros tf2_echo base_link left_sole_link'],
    help: `Unitree R1 simulator terminal (ROS 2 Jazzy).
  ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: walk}"     (damp, stand, walk, run, lowlevel)
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}, angular: {z: 0.4}}"
  ros2 topic pub --once /action std_msgs/msg/String "{data: kungfu}"            (dance, wave, handshake, bow, …)
  ros2 topic echo /loco/state | /lowstate --once | /imu --once | /joint_states --once
  ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [left_shoulder_pitch_joint], position: [-1.57]}"
  ros2 interface show snobots_msgs/msg/LowCmd       (low-level PD commands: Low-level mode only)
  ros2 service call /sim/push std_srvs/srv/Trigger    ros2 param set /r1_controller balance false
  ros2 run r1_demos dance    ros2 run tf2_ros tf2_echo base_link head_link
Keys: Up/Down history · Tab completes · Ctrl+C stops · Ctrl+L clears`,
    missions: [
      { id: 'm1', title: 'Walk 2 m', text: 'Walk R1 2 m forward with the joystick, the buttons or W/A/S/D.', tab: 'control', xp: 15 },
      { id: 'm2', title: 'Run!', text: 'Switch to **Run** mode and reach 0.45 m/s.', tab: 'control', xp: 20 },
      { id: 'm3', title: 'Show time', text: 'Play Dance, Kung fu, Wave, Handshake or Bow to the end.', tab: 'control', xp: 15 },
      { id: 'm4', title: 'Push recovery', text: 'With balance on, survive a small **Push** without falling.', tab: 'control', xp: 15 },
      { id: 'm5', title: 'Damp and rise', text: 'Switch to **Damp** and watch R1 slump under gravity, then press **Stand** so the gantry hoist lifts it back up.', tab: 'control', xp: 20 },
      { id: 'm6', title: 'Mode by service', text: 'Change mode from the Terminal: `ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: run}"`.', tab: 'term', xp: 20 },
      { id: 'm7', title: 'PD tuner', text: 'In **Low-level** mode, hold left_shoulder_pitch_joint at −90° (arm straight forward) within 3° for 2 s. Gravity pulls the arm down, so raise kp or add τ_ff.', tab: 'joints', xp: 30 },
      { id: 'm8', title: 'Terminal walker', text: 'Walk 1 m using only `ros2 topic pub` on /cmd_vel.', tab: 'term', xp: 20 },
      { id: 'm9', title: 'Code it', text: 'Run a Python node that walks R1 1 m, or sends 20 /lowcmd messages.', tab: 'code', xp: 30 },
    ],
    specs: [['Maker', 'Unitree'], ['Type', 'Full-size humanoid'], ['Size', '≈ 1210 × 357 × 190 mm'], ['Weight', '≈ 25–29 kg (by version)'], ['Joints', '20 (R1 Air), 24–26 (R1), up to 40 with dexterous hands (EDU)'], ['Arm length', '435 mm (upper arm + forearm)'], ['Head', 'Binocular camera'], ['Compute', '8-core CPU'], ['Battery', 'about 1 hour'], ['This sim', '26 joints: legs 12, waist 2, arms 10, head 2']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Walk and run velocity (in walk or run mode)'], ['/action', 'subscribe', 'std_msgs/msg/String', 'dance, kungfu, wave, handshake, bow, squat, raise_arms'], ['/joint_commands', 'subscribe', 'sensor_msgs/msg/JointState', 'Joint targets by name; in Low-level mode they set q_des'], ['/lowcmd', 'subscribe', 'snobots_msgs/msg/LowCmd', 'Per-motor q, dq, kp, kd, τ (Low-level mode only)'], ['/lowstate', 'publish', 'snobots_msgs/msg/LowState', 'Per-motor q, dq, estimated torque, temperature, 25 Hz'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'All 26 joints, 25 Hz'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Pelvis orientation and rates, 50 Hz'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Walking odometry, 20 Hz'], ['/loco/state', 'publish', 'std_msgs/msg/String', 'standing, walking, running, damp, lowlevel, fallen…'], ['/loco/set_mode', 'service', 'snobots_msgs/srv/SetMode', 'damp, stand, walk, run, lowlevel'], ['/loco/stand_up', 'service', 'std_srvs/srv/Trigger', 'Get up from damp or after a fall'], ['/sim/push', 'service', 'std_srvs/srv/Trigger', 'Simulator only: push the robot'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot'], ['/r1_controller', 'parameters', 'double / bool', '`max_linear_speed` (walk), `max_run_speed`, `step_height`, `step_period`, `body_height`, `balance`']],
    jointDoc: 'Motor index order (used by /lowcmd and /lowstate): 0–5 left leg (`hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll`), 6–11 right leg, 12 `waist_yaw`, 13 `waist_pitch`, 14–18 left arm (`shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll`), 19–23 right arm, 24 `head_yaw`, 25 `head_pitch`. Full names end in `_joint`, for example `left_knee_joint`. In /lowcmd, `mode` 1 applies a motor command and 0 keeps the current one.',
    realism: 'Physics comes from the simulator\'s own MuJoCo-style rigid-body engine (not MuJoCo itself): about 25 kg spread over the links, torque- and speed-limited joint motors, and foot contacts with friction. Unitree trains walking policies for its humanoids, R1 included, with reinforcement learning in simulators such as MuJoCo and Isaac Lab. This simulator uses a model-based controller instead (footstep planning, a linear-inverted-pendulum centre-of-mass plan and capture-point stepping), so its gait, top speed and push recovery differ from the real robot\'s. Getting up after a fall uses a virtual gantry hoist. Low-level mode follows the same idea as Unitree\'s SDK (each motor gets q, dq, kp, kd and a feed-forward torque) with the robot hanging from a gantry rope, but topic and message names are simplified for the simulator. Never try low-level code on a real humanoid without a gantry or harness and an emergency stop.',
    pyHint: 'Pick a template and press Run. For the PD template, switch R1 to Low-level mode first (Control tab).',
    py: {
      walk: { label: 'Walk a plan (/cmd_vel)', code: PY_HEAD + `from geometry_msgs.msg import Twist

PLAN = [  # (seconds, forward m/s, turn rad/s)
    (3.0, 0.6, 0.0),
    (2.0, 0.0, 0.8),
    (3.0, 0.6, 0.0),
    (2.0, 0.0, -0.8),
    (1.0, 0.0, 0.0),
]


class Walker(Node):
    def __init__(self):
        super().__init__('r1_walker')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.i = 0
        self.t = 0.0
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        seconds, vx, wz = PLAN[self.i]
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub.publish(cmd)
        self.t += 0.1
        if self.t >= seconds:
            self.t = 0.0
            self.i += 1
            if self.i >= len(PLAN):
                self.get_logger().info('plan finished')
                self.timer.cancel()
            else:
                self.get_logger().info(f'step {self.i + 1}: vx={PLAN[self.i][1]} wz={PLAN[self.i][2]}')
` + PY_MAIN('Walker') },
      pd: { label: 'PD hold the left arm (/lowcmd)', code: PY_HEAD + `from snobots_msgs.msg import LowCmd, LowState

LEFT_SHOULDER_PITCH = 14   # motor index (see the Info tab)
TARGET = -1.57             # arm straight forward


class PDHold(Node):
    """Low-level control: hold the left arm forward with a PD loop.
    Switch R1 to Low-level mode first, so it hangs from the gantry."""

    def __init__(self):
        super().__init__('pd_hold')
        self.pub = self.create_publisher(LowCmd, '/lowcmd', 10)
        self.create_subscription(LowState, '/lowstate', self.on_state, 10)
        self.q = 0.0
        self.n = 0
        self.timer = self.create_timer(0.02, self.tick)

    def on_state(self, msg):
        self.q = msg.motor_state[LEFT_SHOULDER_PITCH].q

    def tick(self):
        cmd = LowCmd()                 # 26 motor commands, mode 0 = unchanged
        m = cmd.motor_cmd[LEFT_SHOULDER_PITCH]
        m.mode = 1                     # apply this command
        m.q = TARGET
        m.kp = 60.0                    # stiffness, N*m/rad
        m.kd = 2.0                     # damping, N*m*s/rad
        m.tau = -3.5                   # feed-forward torque against gravity
        self.pub.publish(cmd)
        self.n += 1
        if self.n % 50 == 0:
            err = math.degrees(TARGET - self.q)
            self.get_logger().info(f'q={self.q:.3f} rad   error={err:.1f} deg')
` + PY_MAIN('PDHold') },
      tilt: { label: 'Tilt alarm (/imu)', code: PY_HEAD + `from sensor_msgs.msg import Imu
from std_msgs.msg import String


class TiltWatch(Node):
    """Turn the IMU quaternion into roll and pitch; warn when R1 tilts. Try the Push buttons."""

    def __init__(self):
        super().__init__('tilt_watch')
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.alarm = self.create_publisher(String, '/tilt_alarm', 10)
        self.worst = 0.0

    def on_imu(self, msg):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        tilt = math.degrees(math.hypot(roll, pitch))
        if tilt > self.worst + 1.0:
            self.worst = tilt
            self.get_logger().warn(f'new max tilt: {tilt:.1f} deg')
            self.alarm.publish(String(data=f'tilt {tilt:.1f}'))
` + PY_MAIN('TiltWatch') },
    },
  },
  limo: {
    id: 'limo', name: 'LIMO', maker: 'AgileX', catLabel: 'Autonomous car', color: '#30A46C', hz: 60, cams: true, mSlam: 'm5',
    blurb: 'A small ROS car with four motion modes, lidar and a depth camera. Map the city table, navigate, follow lanes and stop at red lights.',
    chipsHub: ['4 motion modes', 'lidar + SLAM', 'fix-it tickets'], tabs: [['control', 'drive', 'Control'], ['phys', 'chip', 'Physics'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions'], ['fix', 'bug', 'Fix it'], ['info', 'book', 'Info']],
    kbd: CAR_KBD, keysHint: 'W A S D drive, Q E slide in mecanum mode. Click the floor to send a navigation goal.', teleop: [0.5, 1.0],
    mainNodes: ['/limo_base', '/ydlidar_node', '/lane_detector'], arena: LIMO_ARENA, desc: () => limoDesc(),
    make: (d) => new LimoSim(d),
    view: { dist: 1.55, az: 2.2, el: 0.5, target: [1.6, 0.62, 0.1], minDist: 0.4, maxDist: 10, shadow: 3.6, grid: null, fog: [9, 26] },
    pkgs: { limo_bringup: ['slam.launch.py', 'navigation.launch.py', 'lane_follow.launch.py'] },
    chips: ['ros2 topic echo /limo_status --once', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"', 'ros2 param set /limo_base motion_mode mecanum', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}"', 'ros2 launch limo_bringup slam.launch.py', 'ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 3.95, y: 1.58}}}"', 'ros2 topic echo /lane_error', 'ros2 run tf2_ros tf2_echo odom laser_link'],
    help: `LIMO simulator terminal (ROS 2 Jazzy).
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
  ros2 param set /limo_base motion_mode ackermann        (diff, ackermann, tracked, mecanum)
  ros2 topic echo /limo_status --once | /scan --once | /odom | /lane_error | /traffic_light
  ros2 launch limo_bringup slam.launch.py | navigation.launch.py | lane_follow.launch.py
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 3.95, y: 1.58}}}"
  ros2 param set /lane_follower speed 0.4      ros2 param set /navigator max_vel 0.6
  ros2 service call /reset_world std_srvs/srv/Empty     ros2 run tf2_ros tf2_echo odom laser_link
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   (Shift + J/L slides in mecanum mode)
The robot's own nodes can be stopped and started again, as on the real robot (Fix it tab: broken-robot tickets):
  ros2 run limo_base limo_base_node --ros-args -p motion_mode:=mecanum
  ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node     ros2 run slam_toolbox async_slam_toolbox_node
  ros2 run tf2_ros static_transform_publisher --x 0.2 --frame-id base_link --child-frame-id tool_link
  ros2 run topic_tools relay /scan /scan_copy     ros2 topic info /scan --verbose     ros2 topic echo /rosout
Keys: Up/Down history · Tab completes · Ctrl+C stops · Ctrl+L clears`,
    missions: [
      { id: 'm1', title: 'First drive', text: 'Drive 2 m with the joystick, the buttons or W/A/S/D.', tab: 'control', xp: 15 },
      { id: 'm2', title: 'Four modes', text: 'Drive at least 30 cm in each motion mode: 4WD diff, Ackermann, Tracks and Mecanum.', tab: 'control', xp: 25 },
      { id: 'm3', title: 'Sideways!', text: 'In Mecanum mode, slide 50 cm sideways (side buttons, Q/E or `linear.y`).', tab: 'control', xp: 15 },
      { id: 'm4', title: 'Read the status', text: 'Run `ros2 topic echo /limo_status --once` and find `motion_mode`.', tab: 'term', xp: 15 },
      { id: 'm5', title: 'Cartographer', text: 'Turn on SLAM mapping and map 70% of the arena.', tab: 'control', xp: 30 },
      { id: 'm6', title: 'Park it', text: 'Park in slot **P1**, lined up with the slot. Navigation helps: click inside P1.', tab: 'control', xp: 25 },
      { id: 'm7', title: 'Lane keeper', text: 'Complete one full lap in the right-hand lane (Lane following, or your own node).', tab: 'control', xp: 30 },
      { id: 'm8', title: 'Red means stop', text: 'Stop before the white stop line while the traffic light is red.', tab: 'control', xp: 20 },
      { id: 'm9', title: 'Terminal pilot', text: 'Drive 1 m using only `ros2 topic pub` on /cmd_vel.', tab: 'term', xp: 20 },
      { id: 'm10', title: 'Code it', text: 'Run a Python node that drives LIMO 1 m. Try the obstacle stopper.', tab: 'code', xp: 30 },
    ],
    specs: [['Maker', 'AgileX Robotics'], ['Type', 'ROS mobile robot, four motion modes'], ['Size', '322 × 220 × 251 mm'], ['Weight', '≈ 4.2–4.8 kg'], ['Payload', 'up to 4 kg'], ['Max speed', '1 m/s'], ['Wheelbase / track', '200 / 175 mm'], ['Min turning radius', '0.4 m (Ackermann)'], ['Lidar', 'EAI X2L or T-mini Pro (by version)'], ['Depth camera', 'Orbbec Dabai'], ['Compute', 'Jetson Nano, Orin Nano or Intel NUC'], ['ROS', 'ROS 1 Noetic, ROS 2 Foxy / Humble']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', '`linear.x`, `angular.z`, and `linear.y` in mecanum mode'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Wheel odometry, 20 Hz (odom = arena frame here)'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Yaw and rates, 20 Hz'], ['/limo_status', 'publish', 'limo_msgs/msg/LimoStatus', '`motion_mode`: 0 diff/tracked, 1 Ackermann, 2 mecanum. Also battery and state'], ['/scan', 'publish', 'sensor_msgs/msg/LaserScan', '360 rays from −π to π: ranges[180] is straight ahead, 10 Hz'], ['/lane_error', 'publish', 'std_msgs/msg/Float64', 'Metres to the lane centre at the look-ahead point (+ = left), only when a lane is seen'], ['/traffic_light', 'publish', 'std_msgs/msg/String', 'red, yellow, green, or none'], ['/map', 'publish', 'nav_msgs/msg/OccupancyGrid', 'From /slam_toolbox when SLAM is on, 5 cm cells'], ['/goal_pose', 'subscribe', 'geometry_msgs/msg/PoseStamped', 'Goal for /navigator (click the floor or the map)'], ['/plan', 'publish', 'nav_msgs/msg/Path', 'Path from A* on the arena map'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot and the map'], ['/limo_base', 'parameters', 'string / double', '`motion_mode`, `max_linear_speed`, `max_angular_speed`, `cmd_timeout`']],
    realism: 'The base runs on the simulator\'s own MuJoCo-style physics: a rigid chassis on four torque-limited hub motors, where tyre friction decides what really happens. Skid turns scrub and slip, mecanum rollers only grip across their axis, and Ackermann steering rolls about one turning centre (0.4 m minimum radius). `/odom` is integrated from the wheel encoders (with IMU heading in the skid modes), so it drifts like a real robot\'s. SLAM and navigation publish the `map → odom` correction; here that correction is computed from the true pose, while real SLAM estimates it from lidar scans and makes small errors of its own. `/limo_status` uses the same fields as the real limo_msgs/LimoStatus. On the real robot, lane and traffic-light detection come from camera images; here they are computed from the map, and switching modes is a click instead of swapping wheels.',
    pyHint: 'Pick a template, press Run and watch LIMO. Note the lidar order: ranges[180] is straight ahead.',
    py: {
      stop: { label: 'Obstacle stopper (/scan)', code: PY_HEAD + `from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Cautious(Node):
    """Drive forward; turn away when something is closer than 0.45 m in front.
    Use 4WD diff or Tracks mode (Ackermann cannot turn on the spot)."""

    def __init__(self):
        super().__init__('cautious_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, scan):
        mid = len(scan.ranges) // 2              # -pi..pi, so the middle is straight ahead
        window = scan.ranges[mid - 20: mid + 21]  # +/- 20 degrees
        front = min([r for r in window if math.isfinite(r)], default=float('inf'))
        cmd = Twist()
        if front > 0.45:
            cmd.linear.x = 0.3
        else:
            cmd.angular.z = 1.0                  # turn left on the spot
        self.pub.publish(cmd)
` + PY_MAIN('Cautious') },
      lane: { label: 'Lane keeper (/lane_error)', code: PY_HEAD + `from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String


class LaneKeeper(Node):
    """Proportional steering from /lane_error. Waits while the light is red."""

    def __init__(self):
        super().__init__('lane_keeper')
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('speed', 0.25)
        self.light = 'none'
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64, '/lane_error', self.on_error, 10)
        self.create_subscription(String, '/traffic_light', self.on_light, 10)

    def on_light(self, msg):
        self.light = msg.data

    def on_error(self, msg):
        cmd = Twist()
        if self.light != 'red':
            cmd.linear.x = self.get_parameter('speed').value
        cmd.angular.z = self.get_parameter('kp').value * msg.data
        self.pub.publish(cmd)
` + PY_MAIN('LaneKeeper') },
      goto: { label: 'Go to a point (/odom)', code: PY_HEAD + `from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

GOAL = (4.4, 0.62)   # metres in the odom frame (here the same as the arena)


class GoTo(Node):
    def __init__(self):
        super().__init__('go_to_point')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.done = False

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        dx, dy = GOAL[0] - p.x, GOAL[1] - p.y
        dist = math.hypot(dx, dy)
        err = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        cmd = Twist()
        if dist > 0.05:
            cmd.linear.x = min(0.4, dist) * max(0.0, math.cos(err))
            cmd.angular.z = 1.5 * err
        elif not self.done:
            self.done = True
            self.get_logger().info('arrived!')
        self.pub.publish(cmd)
` + PY_MAIN('GoTo') },
    },
  },
  cobot: {
    id: 'cobot', name: 'LIMO Cobot', maker: 'AgileX', catLabel: 'Autonomous car + arm', color: '#B144C8', hz: 60, cams: true, mSlam: null, cobot: true,
    blurb: 'A LIMO base carrying a 6-axis arm. Drive to the table, pick cubes with inverse kinematics and sort them into the matching bins.',
    chipsHub: ['6-axis arm', '280 mm reach', 'pick & place'], tabs: [['control', 'drive', 'Drive'], ['arm', 'tools', 'Arm'], ['phys', 'chip', 'Physics'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions'], ['info', 'book', 'Info']],
    kbd: CAR_KBD, keysHint: 'W A S D drive, Q E slide in mecanum mode. Click the floor to send a navigation goal.', teleop: [0.4, 1.0],
    mainNodes: ['/limo_base', '/arm_driver', '/ydlidar_node'], arena: COBOT_ARENA, desc: () => cobotDesc(),
    make: (d) => new CobotSim(d),
    view: { dist: 1.35, az: 2.6, el: 0.45, target: [1.2, 2.2, 0.18], minDist: 0.3, maxDist: 10, shadow: 3.4, grid: null, fog: [9, 26] },
    pkgs: { limo_bringup: ['slam.launch.py', 'navigation.launch.py'], cobot_demos: ['pick_place.launch.py', 'go_home', 'open_gripper', 'close_gripper'] },
    chips: ['ros2 topic echo /arm/angles --once', 'ros2 topic pub --once /arm/angles_goal snobots_msgs/msg/ArmAngles "{angles: [0, 0, 0, 0, 0, 0], speed: 50}"', 'ros2 topic pub --once /arm/gripper std_msgs/msg/Int32 "{data: 0}"', 'ros2 service call /arm/go_home std_srvs/srv/Trigger', 'ros2 topic pub --once /arm/coords_goal snobots_msgs/msg/ArmCoords "{x: 180, y: 0, z: 40, rx: 180, ry: 0, rz: 0, speed: 50}"', 'ros2 launch cobot_demos pick_place.launch.py', 'ros2 run tf2_ros tf2_echo arm_base tcp'],
    help: `LIMO Cobot simulator terminal (ROS 2 Jazzy).
  ros2 topic pub --once /arm/angles_goal snobots_msgs/msg/ArmAngles "{angles: [0, -30, -60, 0, 0, 0], speed: 50}"
  ros2 topic pub --once /arm/coords_goal snobots_msgs/msg/ArmCoords "{x: 200, y: 0, z: 50, rx: 180, ry: 0, rz: 0, speed: 50}"
  ros2 topic pub --once /arm/gripper std_msgs/msg/Int32 "{data: 100}"     (0 closed … 100 open)
  ros2 service call /arm/go_home std_srvs/srv/Trigger
  ros2 topic echo /arm/angles --once | /arm/coords --once | /joint_states --once
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
  ros2 launch cobot_demos pick_place.launch.py      ros2 launch limo_bringup navigation.launch.py
  ros2 run tf2_ros tf2_echo arm_base tcp
Keys: Up/Down history · Tab completes · Ctrl+C stops · Ctrl+L clears`,
    missions: [
      { id: 'm1', title: 'Base twist', text: 'Turn joint1 (the arm base) to ±90° in the Arm tab.', tab: 'arm', xp: 10 },
      { id: 'm2', title: 'Gripper check', text: 'Open the gripper fully, then close it.', tab: 'arm', xp: 10 },
      { id: 'm3', title: 'Go home', text: 'Run `ros2 service call /arm/go_home std_srvs/srv/Trigger` in the Terminal.', tab: 'term', xp: 15 },
      { id: 'm4', title: 'Angles from the terminal', text: 'Publish /arm/angles_goal with `ros2 topic pub --once`.', tab: 'term', xp: 20 },
      { id: 'm5', title: 'First pick', text: 'Grab any cube. Park facing the PICK table, then use **Pick nearest cube** and close the gripper.', tab: 'arm', xp: 25 },
      { id: 'm6', title: 'Sorted!', text: 'Drop a cube into the bin of the same colour.', tab: 'arm', xp: 25 },
      { id: 'm7', title: 'Mobile manipulator', text: 'Sort all three cubes, by hand or with the Pick & place demo.', tab: 'control', xp: 30 },
      { id: 'm8', title: 'Code it', text: 'Run a Python node that publishes /arm/angles_goal, or drives the base 1 m.', tab: 'code', xp: 30 },
    ],
    specs: [['Maker', 'AgileX Robotics'], ['Type', 'Mobile manipulator: LIMO PRO base + myCobot 280 arm'], ['Size', '322 × 220 × 550 mm'], ['Weight', '≈ 5.6 kg'], ['Arm', '6 DOF, 280 mm working radius'], ['Arm payload', '250 g'], ['Arm weight', '850 g'], ['Base', 'Four motion modes, lidar, depth camera'], ['Software', 'ROS, SLAM navigation, MoveIt'], ['This sim', 'DH chain + numeric IK, 40 mm parallel gripper']],
    ifaces: [['/arm/angles_goal', 'subscribe', 'snobots_msgs/msg/ArmAngles', '6 joint angles in degrees + speed (like `send_angles`)'], ['/arm/coords_goal', 'subscribe', 'snobots_msgs/msg/ArmCoords', 'Gripper pose x y z (mm), rx ry rz (deg) in the arm base frame (like `send_coords`)'], ['/arm/gripper', 'subscribe', 'std_msgs/msg/Int32', '0 closed … 100 open (like `set_gripper_value`)'], ['/arm/angles', 'publish', 'snobots_msgs/msg/ArmAngles', 'Current angles, 10 Hz'], ['/arm/coords', 'publish', 'snobots_msgs/msg/ArmCoords', 'Current gripper pose, 10 Hz'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'joint1 … joint6 + gripper, 20 Hz'], ['/arm/go_home', 'service', 'std_srvs/srv/Trigger', 'Fold the arm'], ['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Base velocity'], ['/odom, /scan, /imu, /limo_status', 'publish', 'as on LIMO', 'Base sensors'], ['/goal_pose', 'subscribe', 'geometry_msgs/msg/PoseStamped', 'Navigation goal (click the floor)']],
    jointDoc: 'Arm joints `joint1` (base) … `joint6` (wrist roll); all zero points the arm straight up. Limits (°): ±168, ±135, ±150, ±145, ±165, ±175. Coordinates for /arm/coords_goal are in the `arm_base` frame: x forward, y left, z up, millimetres; rx 180, ry 0 points the gripper down.',
    realism: 'The arm uses a myCobot-280-style DH chain (about 280 mm reach) and numeric inverse kinematics. It moves through the same physics engine as the base, with link masses, torque-limited servos and real contacts, so the fingers can nudge or knock a cube over. The real arm is driven with pymycobot (send_angles, send_coords, set_gripper_value) or Elephant Robotics\' ROS packages; the `/arm/*` topics are simplified simulator names. Grasping is partly simplified: once both fingers squeeze a cube, it is locked to the gripper so it cannot slip, whereas a real gripper holds it by friction alone. The base slips and its odometry drifts just like LIMO\'s.',
    pyHint: 'Pick a template and press Run. Park facing the PICK table before running the pick template.',
    py: {
      dance: { label: 'Arm poses (/arm/angles_goal)', code: PY_HEAD + `from snobots_msgs.msg import ArmAngles
from std_msgs.msg import Int32

POSES = [  # degrees for joint1..joint6, like send_angles(angles, speed)
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [45.0, -30.0, -40.0, 0.0, 0.0, 0.0],
    [-45.0, -30.0, -40.0, 0.0, 0.0, 0.0],
    [0.0, 30.0, -130.0, 10.0, 0.0, 0.0],
]


class ArmPoses(Node):
    def __init__(self):
        super().__init__('arm_poses')
        self.pub = self.create_publisher(ArmAngles, '/arm/angles_goal', 10)
        self.grip = self.create_publisher(Int32, '/arm/gripper', 10)
        self.i = 0
        self.timer = self.create_timer(1.6, self.next_pose)
        self.next_pose()

    def next_pose(self):
        if self.i >= len(POSES):
            self.get_logger().info('done')
            self.timer.cancel()
            return
        msg = ArmAngles()
        msg.angles = POSES[self.i]
        msg.speed = 60
        self.pub.publish(msg)
        self.grip.publish(Int32(data=0 if self.i % 2 else 100))
        self.get_logger().info(f'pose {self.i + 1}: {POSES[self.i]}')
        self.i += 1
` + PY_MAIN('ArmPoses') },
      pick: { label: 'Pick in front (/arm/coords_goal)', code: PY_HEAD + `from snobots_msgs.msg import ArmCoords
from std_msgs.msg import Int32

# Park facing the PICK table first, with a cube straight ahead: drive at the table until you stop
# about 1 cm away (or click the floor just in front of a cube and let navigation park you).
STEPS = [  # (seconds, x, y, z in mm in the arm base frame, gripper 0-100)
    (2.0, 210.0, 0.0, 60.0, 100),     # above the cube, gripper open
    (2.0, 210.0, 0.0, -32.0, 100),    # down around the cube
    (1.0, 210.0, 0.0, -32.0, 0),      # close the gripper
    (2.0, 150.0, 0.0, 120.0, 0),      # lift
]


class Picker(Node):
    def __init__(self):
        super().__init__('picker')
        self.arm = self.create_publisher(ArmCoords, '/arm/coords_goal', 10)
        self.grip = self.create_publisher(Int32, '/arm/gripper', 10)
        self.i = 0
        self.timer = self.create_timer(0.1, self.tick)
        self.wait = 0.0

    def tick(self):
        self.wait -= 0.1
        if self.wait > 0:
            return
        if self.i >= len(STEPS):
            self.get_logger().info('finished: is the cube in the gripper?')
            self.timer.cancel()
            return
        seconds, x, y, z, g = STEPS[self.i]
        self.arm.publish(ArmCoords(x=x, y=y, z=z, rx=180.0, ry=0.0, rz=0.0, speed=50))
        self.grip.publish(Int32(data=g))
        self.get_logger().info(f'step {self.i + 1}: ({x:.0f}, {y:.0f}, {z:.0f}) gripper {g}')
        self.wait = seconds
        self.i += 1
` + PY_MAIN('Picker') },
      drive: { label: 'Drive the base (/cmd_vel)', code: PY_HEAD + `from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Rover(Node):
    """Drive forward and stop 0.4 m before anything in front."""

    def __init__(self):
        super().__init__('rover')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, scan):
        mid = len(scan.ranges) // 2
        front = min([r for r in scan.ranges[mid - 15: mid + 16] if math.isfinite(r)], default=float('inf'))
        cmd = Twist()
        cmd.linear.x = 0.25 if front > 0.4 else 0.0
        self.pub.publish(cmd)
` + PY_MAIN('Rover') },
    },
  },
};
