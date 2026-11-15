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
    blurb: 'A 41.5 cm, 24-joint ROS humanoid with a camera head. Walk it, pose it, record action groups, kick a ball, fix broken robots and write OpenCV nodes for its camera.',
    chipsHub: ['24 DOF', 'head camera + OpenCV', 'fix-it tickets'], tabs: [...HU_TABS.slice(0, -1), ['fix', 'bug', 'Fix it'], ['vision', 'camera', 'Vision'], HU_TABS[HU_TABS.length - 1]], kbd: HU_KBD, keysHint: 'W A S D walk, Q E side-step.', teleop: [0.1, 0.6],
    mainNodes: ['/ainex_controller', '/robot_state_publisher', '/usb_cam', '/color_detector', '/ball_locator'],
    make: (d) => new AinexSim(d),
    view: { dist: 1.15, az: -2.45, el: 0.4, target: [-0.35, 0, 0.16], minDist: 0.3, maxDist: 5, shadow: 1.7, grid: null, fog: [5, 14] },
    hu: {
      desc: huAinex, ctl: '/ainex_controller', legRe: /hip|knee|ank/, sole: { l: 'l_sole_link', r: 'r_sole_link' }, cameraParent: 'head_link', staticJoints: AINEX_STATIC,
      map: { hy: 'hip_yaw', hr: 'hip_roll', hp: 'hip_pitch', kn: 'knee', ap: 'ank_pitch', ar: 'ank_roll', sp: 'sho_pitch', sr: 'sho_roll', ep: 'el_pitch', ey: 'el_yaw', gr: 'gripper', pan: 'head_pan', tilt: 'head_tilt' },
      side: { L: 'l_', R: 'r_' }, homeExtra: { tilt: 0.32 },
      order: [...legs6('l_', ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll']), ...legs6('r_', ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ank_pitch', 'ank_roll']), ...legs6('l_', ['sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw', 'gripper']), ...legs6('r_', ['sho_pitch', 'sho_roll', 'el_pitch', 'el_yaw', 'gripper']), 'head_pan', 'head_tilt'],
      standZ: 0.222, footY: 0.036, footL: 0.11, footW: 0.064, start: [-0.35, 0, 0], phys: huAinexPhys,
      walk: { Tss: 0.32, Tds: 0.08, T0: 0.3, stepH: 0.025, zmpIn: 0.008, zmpX: 0, maxFwd: 0.085, maxBack: 0.05, minSep: 0.92, maxSide: 0.045, maxYaw: 0.5, maxYawIn: 0.15, acc: [0.4, 0.3, 2.0], dz: 0.004, land: 0.003, pushStep: 0.025, horizon: 3, zmpMargin: 0.015 },
      fb: { kc: 1.0, kstep: 1.0, dead: 0.012 }, hook: ['torso_link', [0, 0, 0.125]], hoistLift: 0.06, hoistK: 400, hoistKr: 4, hoistCr: 0.5, hoistScale: 0.32, pushDv: 0.45, fallAt: 0.65, ankleBal: [0.8, 0.8],
      params: [['max_linear_speed', 0.15, 'double', [0.02, 0.21]], ['max_angular_speed', 0.8, 'double', [0.1, 1.2]], ['cmd_timeout', 0.8, 'double', [0.1, 5]], ['step_height', 0.025, 'double', [0.008, 0.045]], ['step_period', 0.42, 'double', [0.3, 0.8]], ['body_height', 0.222, 'double', [0.195, 0.232]], ['arm_swing', true, 'bool'], ['balance', true, 'bool']],
      actionTopic: '/action_group', stateTopic: '/walking/state', standSrv: '/stand_up',
      gait: { stepH: 0.025, maxV: 0.21, maxVy: 0.06, maxW: 0.9, arm: 0.28 },
      camZ: 0.15, camLabel: 'head camera · 120° wide', camVfov: AINEX_CAM.vfov, hfov: AINEX_CAM.hfov,
      groups: [['Head', ['pan', 'tilt']], ['Left arm', ['L.sp', 'L.sr', 'L.ep', 'L.ey', 'L.gr']], ['Right arm', ['R.sp', 'R.sr', 'R.ep', 'R.ey', 'R.gr']], ['Left leg', ['L.hy', 'L.hr', 'L.hp', 'L.kn', 'L.ap', 'L.ar']], ['Right leg', ['R.hy', 'R.hr', 'R.hp', 'R.kn', 'R.ap', 'R.ar']]],
      objs: [{ kind: 'ball', x: 0.3, y: 0.06, z: 0.035, r: 0.035 }, { kind: 'block', color: 'red', x: 0.85, y: 0.42, z: 0.025, r: 0.03, size: 0.05 }, { kind: 'block', color: 'green', x: 0.55, y: -0.6, z: 0.025, r: 0.03, size: 0.05 }, { kind: 'block', color: 'blue', x: -0.6, y: 0.55, z: 0.025, r: 0.03, size: 0.05 }],
    },
    pkgs: { ainex_demos: ['wave', 'bow', 'squat', 'raise_arms', 'left_kick', 'right_kick', 'nod', 'shake', 'color_tracking.launch.py', 'color_reaction.launch.py'] },
    chips: ['ros2 node list', 'ros2 topic echo /walking/state', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"', 'ros2 topic pub --once /action_group std_msgs/msg/String "{data: wave}"', 'ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [head_pan], position: [0.8]}"', 'ros2 topic echo /color_detection', 'ros2 param set /color_detector target_color any', 'ros2 launch ainex_demos color_tracking.launch.py', 'ros2 topic hz /image_raw', 'ros2 topic echo /ball_position', 'ros2 run tf2_ros tf2_echo base_link camera_optical_frame'],
    help: `AiNex simulator terminal (ROS 2 Jazzy).
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.3}}"
  ros2 topic pub --once /action_group std_msgs/msg/String "{data: wave}"     (bow, squat, left_kick, nod, …)
  ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [r_sho_pitch], position: [-2.5]}"
  ros2 topic echo /joint_states | /imu --once | /color_detection | /ball_position | /walking/state
  ros2 topic hz /image_raw      ros2 topic echo /camera_info --once      ros2 run usb_cam usb_cam_node_exe
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
      { id: 'v1', title: 'Camera check', text: 'In the Terminal, measure the head camera\'s frame rate with `ros2 topic hz /image_raw`, then read its intrinsics once: `ros2 topic echo /camera_info --once`.', tab: 'term', xp: 15, vision: true },
      { id: 'v2', title: 'Watch the ball', text: 'Run the **Ball tracker (head)** and walk AiNex around with W A S D: your node must keep the orange ball in the middle of the picture for 5 s of walking, by turning the head.', tab: 'vision', xp: 30, vision: true, tpl: 'ball_head' },
      { id: 'v3', title: 'Walk to the ball', text: 'Run **Walk to the ball**: your node must walk AiNex up to the orange ball and stop with it 10 to 25 cm in front, roughly centred.', tab: 'vision', xp: 35, vision: true, tpl: 'ball_walk' },
      { id: 'v4', title: 'Tag reader', text: 'Run the **Tag reader** and walk AiNex around the pitch: your node must publish the IDs of 3 different ArUco tags on `/marker_id`.', tab: 'vision', xp: 30, vision: true, tpl: 'aruco' },
      { id: 'v5', title: 'Vision striker', text: 'Score a goal from a kick your node asks for: walk to the ball, line up with the goal (tag 0 sits above it), then publish `left_kick` or `right_kick` on `/action_group`.', tab: 'vision', xp: 50, vision: true, tpl: 'ball_walk' },
    ],
    visionNote: 'AiNex\'s head camera (`/usb_cam`) publishes `/image_raw` (320 × 240 `rgb8`, 15 Hz, a 120° wide view) and `/camera_info`. Aim it with `/joint_commands` (`head_pan`, `head_tilt`). Write OpenCV nodes in the **Code** tab: `cv_bridge` converts images to numpy arrays, and `cv2.imshow()` windows appear over the 3D view. On the pitch: the orange ball, three coloured blocks, and ArUco tags (`DICT_4X4_50`, 15 cm) on posts, plus a 20 cm tag 0 above the goal.',
    specs: [['Maker', 'Hiwonder'], ['Type', 'Small ROS humanoid'], ['Size', '193 × 135 × 415 mm'], ['Joints', '24 (legs 12, arms + grippers 10, head 2)'], ['Actuators', 'Serial bus servos'], ['Controller', 'Raspberry Pi 4B / 5'], ['Camera', '120° wide-angle, 2-DOF head'], ['IMU', '3-axis gyro, accelerometer, compass'], ['Walking', 'Inverse-kinematics gait'], ['Software', 'ROS, Gazebo, action-group editor']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Walking velocity: `linear.x` forward, `linear.y` sideways, `angular.z` turn'], ['/joint_commands', 'subscribe', 'sensor_msgs/msg/JointState', 'Joint targets by name (radians). Leg joints switch to servo mode'], ['/action_group', 'subscribe', 'std_msgs/msg/String', 'wave, bow, squat, raise_arms, left_kick, right_kick, nod, shake, grab, or your saved actions'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'All 24 joints, 25 Hz'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Orientation, rates, acceleration, 50 Hz'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Walking odometry, 20 Hz'], ['/walking/state', 'publish', 'std_msgs/msg/String', 'standing, walking, action:&lt;name&gt;, fallen, getting_up'], ['/image_raw', 'publish', 'sensor_msgs/msg/Image', 'Head camera (`/usb_cam`): 320 × 240 `rgb8`, 15 Hz, rendered while something subscribes'], ['/camera_info', 'publish', 'sensor_msgs/msg/CameraInfo', 'Pinhole intrinsics `K` (no distortion), frame `camera_optical_frame`'], ['/color_detection', 'publish', 'snobots_msgs/msg/ColorDetection', 'Largest `target_color` blob, one message per camera frame'], ['/ball_position', 'publish', 'geometry_msgs/msg/PointStamped', 'The orange ball on the floor, in `base_link`, while it is in view (`/ball_locator`, uses TF)'], ['/stand_up', 'service', 'std_srvs/srv/Trigger', 'Get up after a fall'], ['/sim/push', 'service', 'std_srvs/srv/Trigger', 'Simulator only: push the robot'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot, ball and blocks'], ['/ainex_controller', 'parameters', 'double / bool', '`step_height`, `step_period`, `body_height`, `max_linear_speed`, `arm_swing`, `balance`'], ['/color_detector', 'parameters', 'string / double', '`target_color` (red, green, blue, any), `min_area`'], ['/usb_cam', 'parameters', 'string / double', '`frame_id`, `framerate`, `camera_name`, … (read once at start-up)'], ['/ball_locator', 'parameters', 'double / string', '`ball_radius`, `target_frame`']],
    jointDoc: 'Joint names: `l_`/`r_` + `hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll` (legs), `sho_pitch, sho_roll, el_pitch, el_yaw, gripper` (arms), plus `head_pan, head_tilt`. Angles are radians and follow the right-hand rule: pitch joints turn about +y (negative swings an arm forward and up), roll about +x, yaw about +z. Knees bend positive, elbows bend negative.',
    realism: 'Physics comes from the simulator\'s own rigid-body engine, built in the style of MuJoCo but not MuJoCo itself. Every link has mass and inertia, every servo is a torque-limited motor, and the feet touch the ground through contacts with friction, so a bad step can really tip the robot over. The walk is model-based: footsteps from `/cmd_vel`, a centre-of-mass plan from the linear inverted pendulum, and leg inverse kinematics solved every control tick. That is close in spirit to the real robot\'s inverse-kinematics gait, but the real walking engine has its own parameters, and real bus servos have backlash and gear play that are not modelled here. After a fall, a virtual hoist (like a lab gantry) lifts the robot back onto its feet. The head camera is rendered from the head\'s pose in the 3D view: 320 × 240 through a 120° (diagonal) lens, modelled as an ideal pinhole, so it has none of the barrel distortion a real wide-angle lens has. Topic names follow plain ROS 2 conventions and are simplified; the real kit ships its own packages, launch files and action-group editor.',
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
      cam: { label: 'Camera viewer (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    """Show what AiNex's head camera sees, and the edges OpenCV finds in it."""

    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.frames = 0

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   # OpenCV images are BGR
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 150)
        self.frames += 1
        if self.frames % 30 == 1:
            h, w = frame.shape[:2]
            b, g, r = frame[h // 2, w // 2]
            self.get_logger().info(f'{w}x{h} image, centre pixel BGR ({b}, {g}, {r}), mean brightness {gray.mean():.0f}')
        cv2.imshow('camera', frame)   # windows appear over the 3D view
        cv2.imshow('edges', edges)
        cv2.waitKey(1)
` + PY_MAIN('CameraViewer') },
      ball_head: { label: 'Ball tracker: head (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

ORANGE_LO = np.array([5, 120, 100])    # OpenCV HSV: H 0-179, S and V 0-255
ORANGE_HI = np.array([22, 255, 255])
KP = 0.4                               # radians of head turn per unit of image error
MAX_TILT = 0.85                        # look further down and the camera sees AiNex's own orange chest


class BallWatcher(Node):
    """Keep the orange ball in the middle of the picture by turning the head."""

    def __init__(self):
        super().__init__('ball_watcher')
        self.bridge = CvBridge()
        self.pan, self.tilt = 0.0, 0.32      # the head's angles, kept up to date from /joint_states
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Image, '/image_raw', self.on_image, 10)

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'head_pan':
                self.pan = pos
            elif name == 'head_tilt':
                self.tilt = pos

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), ORANGE_LO, ORANGE_HI)
        m = cv2.moments(mask)
        if m['m00'] > 30 * 255:                    # at least about 30 orange pixels
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            ex = (cx - w / 2) / (w / 2)            # -1 (left edge) .. +1 (right edge)
            ey = (cy - h / 2) / (h / 2)            # -1 (top) .. +1 (bottom)
            cmd = JointState()
            cmd.name = ['head_pan', 'head_tilt']
            cmd.position = [self.pan - KP * ex,                    # head_pan turns left when positive
                            min(self.tilt + KP * ey, MAX_TILT)]    # head_tilt looks down when positive
            self.pub.publish(cmd)
            cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 0), 2)
        cv2.imshow('ball', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)
` + PY_MAIN('BallWatcher') },
      ball_walk: { label: 'Walk to the ball (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

ORANGE_LO = np.array([5, 120, 100])
ORANGE_HI = np.array([22, 255, 255])
HEAD_TILT = 0.75     # radians: look down at the floor in front of the feet
AT_FEET = 0.7        # the ball's centre this far down the picture (0 top, 1 bottom): about 20 cm away, so stop


class BallWalker(Node):
    """Find the orange ball, turn to face it and walk up to it."""

    def __init__(self):
        super().__init__('ball_walker')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.head = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.create_timer(1.0, self.look_down)

    def look_down(self):
        cmd = JointState()
        cmd.name = ['head_pan', 'head_tilt']
        cmd.position = [0.0, HEAD_TILT]
        self.head.publish(cmd)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), ORANGE_LO, ORANGE_HI)
        m = cv2.moments(mask)
        cmd = Twist()
        if m['m00'] > 30 * 255:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            ex = (cx - w / 2) / (w / 2)           # -1 (left) .. +1 (right)
            if cy / h < AT_FEET:
                cmd.angular.z = -0.8 * ex         # turn toward the ball
                cmd.linear.x = 0.10 if abs(ex) < 0.3 else 0.0
            else:                                 # arrived: an all-zero Twist makes AiNex stop and stand
                self.get_logger().info('the ball is at my feet', throttle_duration_sec=2.0)
            cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 0), 2)
        else:
            cmd.angular.z = 0.4                   # nothing seen: turn on the spot and look around
        self.pub.publish(cmd)
        cv2.imshow('ball', frame)
        cv2.waitKey(1)
` + PY_MAIN('BallWalker') },
      aruco: { label: 'Tag reader (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

TAG_SIZE = 0.15   # metres: the side of the black square (tag 0 above the goal is 0.20)


class TagReader(Node):
    """Detect ArUco tags (DICT_4X4_50), publish their IDs and estimate where they are."""

    def __init__(self):
        super().__init__('tag_reader')
        self.bridge = CvBridge()
        self.K, self.D = None, None
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # sub-pixel corners: much better poses
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.pub = self.create_publisher(Int32, '/marker_id', 10)
        self.create_subscription(CameraInfo, '/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/image_raw', self.on_image, 10)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)   # fx, fy, cx, cy
        self.D = np.array(msg.d, dtype=np.float64)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, i in zip(corners, ids.flatten()):
                self.pub.publish(Int32(data=int(i)))
                if self.K is None:
                    continue
                s = (0.20 if i == 0 else TAG_SIZE) / 2   # corners: top-left, top-right, bottom-right, bottom-left
                square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
                ok, rvec, tvec = cv2.solvePnP(square, c.reshape(4, 2), self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    x, y, z = tvec.flatten()   # camera optical frame: x right, y down, z forward
                    self.get_logger().info(f'tag {i}: {z:.2f} m ahead, {x:+.2f} m to the right',
                                           throttle_duration_sec=1.0)
        cv2.imshow('tags', frame)
        cv2.waitKey(1)
` + PY_MAIN('TagReader') },
    },
  },
  r1: {
    id: 'r1', name: 'Unitree R1', maker: 'Unitree', catLabel: 'Humanoid', color: '#2F6FD0', hz: 60, cams: true,
    blurb: 'A 1.2 m humanoid with a state machine (damp, stand, walk, run), low-level PD control of all 26 motors in a gantry, an RGB-D head camera with a visitor to follow, and broken robots to fix.',
    chipsHub: ['26 joints', 'RGB-D camera + OpenCV', 'fix-it tickets'], tabs: [...HU_TABS.slice(0, -1), ['fix', 'bug', 'Fix it'], ['vision', 'camera', 'Vision'], HU_TABS[HU_TABS.length - 1]], kbd: HU_KBD, keysHint: 'W A S D walk, Q E side-step.', teleop: [0.5, 1.0],
    mainNodes: ['/r1_controller', '/robot_state_publisher', '/head_camera', '/person_locator'],
    make: (d) => new R1Sim(d),
    view: { dist: 3.0, az: -2.45, el: 0.3, target: [0, 0, 0.6], minDist: 0.8, maxDist: 16, shadow: 3.2, grid: [40, 40], fog: [12, 40] },
    hu: {
      desc: huR1, ctl: '/r1_controller', legRe: /hip|knee|ankle/, sole: { l: 'left_sole_link', r: 'right_sole_link' }, cameraParent: 'head_link', staticJoints: R1_STATIC,
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
      camZ: 0.5, camLabel: 'head camera · RGB-D', camVfov: R1_CAM.vfov, hfov: R1_CAM.hfov,
      groups: [['Head & waist', ['pan', 'tilt', 'wy', 'wp']], ['Left arm', ['L.sp', 'L.sr', 'L.sy', 'L.ep', 'L.wr']], ['Right arm', ['R.sp', 'R.sr', 'R.sy', 'R.ep', 'R.wr']], ['Left leg', ['L.hp', 'L.hr', 'L.hy', 'L.kn', 'L.ap', 'L.ar']], ['Right leg', ['R.hp', 'R.hr', 'R.hy', 'R.kn', 'R.ap', 'R.ar']]],
      objs: [],
    },
    pkgs: { r1_demos: ['wave', 'bow', 'dance', 'kungfu', 'handshake', 'squat', 'raise_arms', 'follow_me.launch.py'] },
    chips: ['ros2 node list', 'ros2 topic echo /loco/state', 'ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: run}"', 'ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}"', 'ros2 topic pub --once /action std_msgs/msg/String "{data: dance}"', 'ros2 service call /sim/push std_srvs/srv/Trigger', 'ros2 interface show snobots_msgs/msg/LowCmd', 'ros2 topic hz /head_camera/color/image_raw', 'ros2 topic echo /person_position', 'ros2 launch r1_demos follow_me.launch.py', 'ros2 run tf2_ros tf2_echo base_link camera_optical_frame'],
    help: `Unitree R1 simulator terminal (ROS 2 Jazzy).
  ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: walk}"     (damp, stand, walk, run, lowlevel)
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}, angular: {z: 0.4}}"
  ros2 topic pub --once /action std_msgs/msg/String "{data: kungfu}"            (dance, wave, handshake, bow, …)
  ros2 topic echo /loco/state | /lowstate --once | /imu --once | /joint_states --once
  ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [left_shoulder_pitch_joint], position: [-1.57]}"
  ros2 interface show snobots_msgs/msg/LowCmd       (low-level PD commands: Low-level mode only)
  ros2 service call /sim/push std_srvs/srv/Trigger    ros2 param set /r1_controller balance false
  ros2 run r1_demos dance    ros2 run tf2_ros tf2_echo base_link camera_optical_frame
  ros2 topic hz /head_camera/color/image_raw      ros2 topic echo /head_camera/depth/camera_info --once
  ros2 topic echo /person_position                ros2 launch r1_demos follow_me.launch.py
  ros2 param get /person_locator depth_scale      ros2 run r1_camera head_camera_node
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
      { id: 'v1', title: 'Camera check', text: 'In the Terminal, measure the head camera\'s frame rate with `ros2 topic hz /head_camera/color/image_raw`, then read the depth camera\'s intrinsics once: `ros2 topic echo /head_camera/depth/camera_info --once`.', tab: 'term', xp: 15, vision: true },
      { id: 'v2', title: 'How far away?', text: 'Run **How far is the visitor?**: your node must publish the visitor\'s distance in metres (the depth of their violet vest) on `/visitor_distance` as std_msgs/Float32, 10 times within 10 cm.', tab: 'vision', xp: 30, vision: true, tpl: 'distance' },
      { id: 'v3', title: 'Eyes on the visitor', text: 'Set the visitor to **Stroll** (Control tab) and run the **Head tracker**: your node must keep the violet vest in the middle of the picture for 5 s by turning R1\'s head.', tab: 'vision', xp: 30, vision: true, tpl: 'head' },
      { id: 'v4', title: 'Follow me', text: 'With the visitor strolling, run **Follow the visitor**: your node must keep R1 0.7 to 1.4 m away from them for 10 s, walking with `/cmd_vel`.', tab: 'vision', xp: 40, vision: true, tpl: 'follow' },
      { id: 'v5', title: 'A point in 3D', text: 'Run **Where is the visitor? (3D)**: turn the vest\'s pixel and its depth into a 3D point with the camera\'s intrinsics, and publish it on `/visitor_point` as geometry_msgs/PointStamped. 10 points within 25 cm of the visitor count.', tab: 'vision', xp: 50, vision: true, tpl: 'point' },
    ],
    visionNote: 'R1\'s head camera (`/head_camera`) publishes colour on `/head_camera/color/image_raw` (320 × 240 `rgb8`, 15 Hz, 90° wide) and depth on `/head_camera/depth/image_raw` (`16UC1` in millimetres, aligned pixel for pixel with the colour image; 0 means no depth), each with its own `camera_info`. Aim the head with `/joint_commands` (`head_yaw_joint`, `head_pitch_joint`). The visitor wears a violet vest: make them stand or stroll in the Control tab. Pick the depth topic above and click the picture to read a pixel\'s depth.',
    specs: [['Maker', 'Unitree'], ['Type', 'Full-size humanoid'], ['Size', '≈ 1210 × 357 × 190 mm'], ['Weight', '≈ 25–29 kg (by version)'], ['Joints', '20 (R1 Air), 24–26 (R1), up to 40 with dexterous hands (EDU)'], ['Arm length', '435 mm (upper arm + forearm)'], ['Head', 'Binocular depth camera, 2-DOF neck'], ['Compute', '8-core CPU'], ['Battery', 'about 1 hour'], ['This sim', '26 joints: legs 12, waist 2, arms 10, head 2; RGB-D head camera, 320 × 240 at 15 Hz']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Walk and run velocity (in walk or run mode)'], ['/action', 'subscribe', 'std_msgs/msg/String', 'dance, kungfu, wave, handshake, bow, squat, raise_arms'], ['/joint_commands', 'subscribe', 'sensor_msgs/msg/JointState', 'Joint targets by name; in Low-level mode they set q_des'], ['/lowcmd', 'subscribe', 'snobots_msgs/msg/LowCmd', 'Per-motor q, dq, kp, kd, τ (Low-level mode only)'], ['/lowstate', 'publish', 'snobots_msgs/msg/LowState', 'Per-motor q, dq, estimated torque, temperature, 25 Hz'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'All 26 joints, 25 Hz'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Pelvis orientation and rates, 50 Hz'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Walking odometry, 20 Hz'], ['/loco/state', 'publish', 'std_msgs/msg/String', 'standing, walking, running, damp, lowlevel, fallen…'], ['/loco/set_mode', 'service', 'snobots_msgs/srv/SetMode', 'damp, stand, walk, run, lowlevel'], ['/loco/stand_up', 'service', 'std_srvs/srv/Trigger', 'Get up from damp or after a fall'], ['/sim/push', 'service', 'std_srvs/srv/Trigger', 'Simulator only: push the robot'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot'], ['/r1_controller', 'parameters', 'double / bool', '`max_linear_speed` (walk), `max_run_speed`, `cmd_timeout`, `step_height`, `step_period`, `body_height`, `balance`'], ['/head_camera/color/image_raw', 'publish', 'sensor_msgs/msg/Image', 'Head camera colour (`/head_camera`): 320 × 240 `rgb8`, 15 Hz, rendered while someone subscribes'], ['/head_camera/depth/image_raw', 'publish', 'sensor_msgs/msg/Image', 'Depth aligned to the colour image: `16UC1` millimetres, 0 = no depth (0.2 to 6 m)'], ['/head_camera/color/camera_info', 'publish', 'sensor_msgs/msg/CameraInfo', 'fx = fy = 160 px, cx 159.5, cy 119.5, no distortion; the depth camera_info is the same'], ['/person_position', 'publish', 'geometry_msgs/msg/PointStamped', '`/person_locator`: the visitor in `base_link`, up to 15 Hz'], ['/person_locator', 'parameters', 'double / string', '`depth_scale` (depth units to metres), `target_frame`, `max_range`'], ['/follow_me', 'parameters', 'double', '`follow_distance`, `head_kp`, `max_speed`, while the follow-me demo runs']],
    jointDoc: 'Motor index order (used by /lowcmd and /lowstate): 0–5 left leg (`hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll`), 6–11 right leg, 12 `waist_yaw`, 13 `waist_pitch`, 14–18 left arm (`shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll`), 19–23 right arm, 24 `head_yaw`, 25 `head_pitch`. Full names end in `_joint`, for example `left_knee_joint`. In /lowcmd, `mode` 1 applies a motor command and 0 keeps the current one.',
    realism: 'Physics comes from the simulator\'s own MuJoCo-style rigid-body engine (not MuJoCo itself): about 25 kg spread over the links, torque- and speed-limited joint motors, and foot contacts with friction. Unitree trains walking policies for its humanoids, R1 included, with reinforcement learning in simulators such as MuJoCo and Isaac Lab. This simulator uses a model-based controller instead (footstep planning, a linear-inverted-pendulum centre-of-mass plan and capture-point stepping), so its gait, top speed and push recovery differ from the real robot\'s. Getting up after a fall uses a virtual gantry hoist. Low-level mode follows the same idea as Unitree\'s SDK (each motor gets q, dq, kp, kd and a feed-forward torque) with the robot hanging from a gantry rope, but topic and message names are simplified for the simulator. Never try low-level code on a real humanoid without a gantry or harness and an emergency stop. The head camera simplifies the binocular depth camera to a registered colour and depth pair from an ideal pinhole (no lens distortion), 320 × 240 at 15 Hz and 90° wide; the real camera\'s images are larger and wider. Its depth noise grows with distance squared, as with stereo. The camera driver, vision nodes and their topic names are the simulator\'s own.',
    pyHint: 'Pick a template and press Run. For the PD template, switch R1 to Low-level mode first (Control tab). The camera templates are in the Vision tab.',
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
      cam: { label: 'RGB-D viewer (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RGBDViewer(Node):
    """Show R1's colour image and its depth image, coloured from near (red) to far (blue)."""

    def __init__(self):
        super().__init__('rgbd_viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/head_camera/color/image_raw', self.on_color, 10)
        self.create_subscription(Image, '/head_camera/depth/image_raw', self.on_depth, 10)
        self.frames = 0

    def on_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   # OpenCV images are BGR
        cv2.imshow('colour', frame)                                       # windows appear over the 3D view
        cv2.waitKey(1)

    def on_depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg)    # uint16 millimetres (16UC1); 0 = no depth there
        h, w = depth.shape
        self.frames += 1
        valid = depth[depth > 0]
        if self.frames % 30 == 1 and valid.size:
            mm = int(depth[h // 2, w // 2])
            self.get_logger().info(f'centre pixel {mm} mm = {mm / 1000:.2f} m; nearest {valid.min() / 1000:.2f} m, farthest {valid.max() / 1000:.2f} m')
        scaled = np.clip(depth, 0, 6000).astype(np.float32) * (255.0 / 6000.0)   # 0 to 6 m becomes 0 to 255
        vis = cv2.applyColorMap((255 - scaled).astype(np.uint8), cv2.COLORMAP_JET)
        vis[depth == 0] = 0                                                     # no depth: black
        cv2.imshow('depth', vis)
        cv2.waitKey(1)
` + PY_MAIN('RGBDViewer') },
      distance: { label: 'How far is the visitor? (depth)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

VEST_LO = np.array([125, 80, 40])     # OpenCV HSV (H 0-179, S and V 0-255): the visitor's violet vest
VEST_HI = np.array([155, 255, 255])


class VisitorDistance(Node):
    """Find the violet vest in the colour image and read its distance from the aligned depth image."""

    def __init__(self):
        super().__init__('visitor_distance')
        self.bridge = CvBridge()
        self.depth = None
        self.pub = self.create_publisher(Float32, '/visitor_distance', 10)
        self.create_subscription(Image, '/head_camera/depth/image_raw', self.on_depth, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.on_color, 10)

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)    # uint16 millimetres, on the colour image's pixel grid

    def on_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), VEST_LO, VEST_HI)
        if self.depth is not None and cv2.countNonZero(mask) > 40:
            d = self.depth[(mask > 0) & (self.depth > 0)]        # the depth under the vest, holes left out
            if d.size:
                metres = float(np.median(d)) / 1000.0            # the median shrugs off noise and edge pixels
                self.pub.publish(Float32(data=metres))
                x, y, w, h = cv2.boundingRect(mask)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f'{metres:.2f} m', (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                self.get_logger().info(f'the visitor is {metres:.2f} m away', throttle_duration_sec=1.0)
        cv2.imshow('visitor', frame)
        cv2.waitKey(1)
` + PY_MAIN('VisitorDistance') },
      head: { label: 'Head tracker (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

VEST_LO = np.array([125, 80, 40])
VEST_HI = np.array([155, 255, 255])
KP = 0.4     # radians of head turn per unit of image error (the picture is about 1.57 rad wide)


class HeadTracker(Node):
    """Keep the visitor's violet vest in the middle of the picture by turning R1's head."""

    def __init__(self):
        super().__init__('head_tracker')
        self.bridge = CvBridge()
        self.yaw, self.pitch = 0.0, 0.0      # the head's angles, kept up to date from /joint_states
        self.pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.on_image, 10)

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == 'head_yaw_joint':
                self.yaw = pos
            elif name == 'head_pitch_joint':
                self.pitch = pos

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), VEST_LO, VEST_HI)
        m = cv2.moments(mask)
        if m['m00'] > 40 * 255:                    # at least about 40 violet pixels
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            ex = (cx - w / 2) / (w / 2)            # -1 (left edge) .. +1 (right edge)
            ey = (cy - h / 2) / (h / 2)            # -1 (top) .. +1 (bottom)
            cmd = JointState()
            cmd.name = ['head_yaw_joint', 'head_pitch_joint']
            cmd.position = [self.yaw - KP * ex,      # head_yaw turns left when positive
                            self.pitch + KP * ey]    # head_pitch looks down when positive
            self.pub.publish(cmd)
            cv2.circle(frame, (int(cx), int(cy)), 10, (0, 255, 0), 2)
        cv2.imshow('visitor', frame)
        cv2.imshow('vest mask', mask)
        cv2.waitKey(1)
` + PY_MAIN('HeadTracker') },
      follow: { label: 'Follow the visitor (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

VEST_LO = np.array([125, 80, 40])
VEST_HI = np.array([155, 255, 255])
KEEP = 0.8         # metres of depth to keep between the camera and the vest
KP_TURN = 1.0      # rad/s of turning per unit of image error
KP_WALK = 0.6      # m/s per metre of distance error


class Follower(Node):
    """Walk after the visitor: turn toward the violet vest, and walk until its depth is KEEP metres."""

    def __init__(self):
        super().__init__('follower')
        self.bridge = CvBridge()
        self.depth = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/head_camera/depth/image_raw', self.on_depth, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.on_color, 10)

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def on_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), VEST_LO, VEST_HI)
        cmd = Twist()
        if self.depth is not None and cv2.countNonZero(mask) > 40:
            m = cv2.moments(mask)
            ex = (m['m10'] / m['m00'] - w / 2) / (w / 2)       # -1 (left) .. +1 (right)
            d = self.depth[(mask > 0) & (self.depth > 0)]
            if d.size:
                dist = float(np.median(d)) / 1000.0
                cmd.angular.z = -KP_TURN * ex                   # turn toward the vest
                if abs(ex) < 0.4:                               # roughly facing it: close the gap to KEEP
                    cmd.linear.x = max(-0.1, min(0.35, KP_WALK * (dist - KEEP)))
                self.get_logger().info(f'visitor at {dist:.2f} m', throttle_duration_sec=1.0)
        self.pub.publish(cmd)      # nobody seen: an all-zero Twist, so R1 stands still
        cv2.imshow('follow', frame)
        cv2.waitKey(1)
` + PY_MAIN('Follower') },
      point: { label: 'Where is the visitor? (3D)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

VEST_LO = np.array([125, 80, 40])
VEST_HI = np.array([155, 255, 255])


class VisitorPoint(Node):
    """Turn the vest's pixel and its depth into a 3D point with the camera's intrinsics (the pinhole model)."""

    def __init__(self):
        super().__init__('visitor_point')
        self.bridge = CvBridge()
        self.depth, self.K = None, None
        self.pub = self.create_publisher(PointStamped, '/visitor_point', 10)
        self.create_subscription(CameraInfo, '/head_camera/color/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/head_camera/depth/image_raw', self.on_depth, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.on_color, 10)

    def on_info(self, msg):
        self.K = list(msg.k)      # [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def on_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), VEST_LO, VEST_HI)
        if self.K is None or self.depth is None or cv2.countNonZero(mask) < 40:
            return
        d = self.depth[(mask > 0) & (self.depth > 0)]
        if d.size == 0:
            return
        m = cv2.moments(mask)
        u, v = m['m10'] / m['m00'], m['m01'] / m['m00']   # the vest's centre, in pixels
        z = float(np.median(d)) / 1000.0                   # metres along the optical axis
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        p = PointStamped()
        p.header = msg.header             # same time and frame as the image: camera_optical_frame
        p.point.x = (u - cx) * z / fx     # right
        p.point.y = (v - cy) * z / fy     # down
        p.point.z = z                     # forward
        self.pub.publish(p)
        self.get_logger().info(f'visitor at x {p.point.x:+.2f}, y {p.point.y:+.2f}, z {p.point.z:.2f} m in {msg.header.frame_id}', throttle_duration_sec=1.0)
` + PY_MAIN('VisitorPoint') },
    },
  },
  limo: {
    id: 'limo', name: 'LIMO', maker: 'AgileX', catLabel: 'Autonomous car', color: '#30A46C', hz: 60, cams: true, mSlam: 'm5',
    blurb: 'A small ROS car with four motion modes, lidar and a depth camera. Map the city table, navigate, follow lanes and stop at red lights.',
    chipsHub: ['4 motion modes', 'lidar + SLAM', 'fix-it tickets'], tabs: [['control', 'drive', 'Control'], ['phys', 'chip', 'Physics'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions'], ['fix', 'bug', 'Fix it'], ['vision', 'camera', 'Vision'], ['info', 'book', 'Info']],
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
Camera (Vision tab: image viewer, OpenCV starter code):
  ros2 topic hz /camera/color/image_raw     ros2 topic bw /camera/color/image_raw
  ros2 topic echo /camera/color/camera_info --once     ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
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
      { id: 'v1', title: 'Camera check', text: 'Measure the camera rate with `ros2 topic hz /camera/color/image_raw`, then read its intrinsics with `ros2 topic echo /camera/color/camera_info --once`.', tab: 'term', xp: 15, vision: true },
      { id: 'v2', title: 'Follow the yellow line', text: 'Run the **Line follower** and let your node drive LIMO 4 m along the yellow line, steering from camera images only.', tab: 'vision', xp: 30, vision: true, tpl: 'line' },
      { id: 'v3', title: 'The magenta ball', text: 'The **Colour tracker** hunts red. Change its HSV range so it finds the **magenta** ball, and stop 20 to 50 cm in front of it, facing it.', tab: 'vision', xp: 35, vision: true, tpl: 'color' },
      { id: 'v4', title: 'Marker reader', text: 'Run the **ArUco marker reader** and drive around: your node must publish the IDs of 4 different markers on `/marker_id`.', tab: 'vision', xp: 30, vision: true, tpl: 'aruco' },
      { id: 'v5', title: 'Dock at marker 7', text: 'Write a node that parks the camera 0.5 m in front of marker 7, centred and square to it (within 5 cm and 6°), then stops. `solvePnP` gives you the distance and angle. Tip: one small marker gives a noisy angle, so steer onto its centre line early, then keep it in the middle of the image.', tab: 'vision', xp: 50, vision: true, tpl: 'aruco' },
    ],
    visionNote: 'LIMO\'s front camera publishes `/camera/color/image_raw` (320 × 240 `rgb8`, 10 Hz) and `/camera/color/camera_info`. Write OpenCV nodes in the **Code** tab: `cv_bridge` converts images to numpy arrays, and `cv2.imshow()` windows appear over the 3D view. Around the arena: ArUco markers (`DICT_4X4_50`, 12 cm) on the walls and the kiosk, a red ball and a magenta ball you can push or grab (Ctrl + drag).',
    specs: [['Maker', 'AgileX Robotics'], ['Type', 'ROS mobile robot, four motion modes'], ['Size', '322 × 220 × 251 mm'], ['Weight', '≈ 4.2–4.8 kg'], ['Payload', 'up to 4 kg'], ['Max speed', '1 m/s'], ['Wheelbase / track', '200 / 175 mm'], ['Min turning radius', '0.4 m (Ackermann)'], ['Lidar', 'EAI X2L or T-mini Pro (by version)'], ['Depth camera', 'Orbbec Dabai'], ['Compute', 'Jetson Nano, Orin Nano or Intel NUC'], ['ROS', 'ROS 1 Noetic, ROS 2 Foxy / Humble']],
    ifaces: [['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', '`linear.x`, `angular.z`, and `linear.y` in mecanum mode'], ['/odom', 'publish', 'nav_msgs/msg/Odometry', 'Wheel odometry, 20 Hz (odom = arena frame here)'], ['/imu', 'publish', 'sensor_msgs/msg/Imu', 'Yaw and rates, 20 Hz'], ['/limo_status', 'publish', 'limo_msgs/msg/LimoStatus', '`motion_mode`: 0 diff/tracked, 1 Ackermann, 2 mecanum. Also battery and state'], ['/scan', 'publish', 'sensor_msgs/msg/LaserScan', '360 rays from −π to π: ranges[180] is straight ahead, 10 Hz'], ['/lane_error', 'publish', 'std_msgs/msg/Float64', 'Metres to the lane centre at the look-ahead point (+ = left), only when a lane is seen'], ['/traffic_light', 'publish', 'std_msgs/msg/String', 'red, yellow, green, or none'], ['/map', 'publish', 'nav_msgs/msg/OccupancyGrid', 'From /slam_toolbox when SLAM is on, 5 cm cells'], ['/goal_pose', 'subscribe', 'geometry_msgs/msg/PoseStamped', 'Goal for /navigator (click the floor or the map)'], ['/plan', 'publish', 'nav_msgs/msg/Path', 'Path from A* on the arena map'], ['/reset_world', 'service', 'std_srvs/srv/Empty', 'Reset the robot and the map'], ['/camera/color/image_raw', 'publish', 'sensor_msgs/msg/Image', '320 × 240 `rgb8`, 10 Hz, frame `camera_color_optical_frame` (rendered only while something subscribes)'], ['/camera/color/camera_info', 'publish', 'sensor_msgs/msg/CameraInfo', 'Pinhole intrinsics `k`, no distortion'], ['/marker_id', 'subscribe', 'std_msgs/msg/Int32', 'Marker IDs your node reports (Vision missions)'], ['/limo_base', 'parameters', 'string / double', '`motion_mode`, `max_linear_speed`, `max_angular_speed`, `cmd_timeout`']],
    realism: 'The base runs on the simulator\'s own MuJoCo-style physics: a rigid chassis on four torque-limited hub motors, where tyre friction decides what really happens. Skid turns scrub and slip, mecanum rollers only grip across their axis, and Ackermann steering rolls about one turning centre (0.4 m minimum radius). `/odom` is integrated from the wheel encoders (with IMU heading in the skid modes), so it drifts like a real robot\'s. SLAM and navigation publish the `map → odom` correction; here that correction is computed from the true pose, while real SLAM estimates it from lidar scans and makes small errors of its own. `/limo_status` uses the same fields as the real limo_msgs/LimoStatus. On the real robot, lane and traffic-light detection come from camera images; here `/lane_detector` computes them from the map, and switching modes is a click instead of swapping wheels. The camera itself is real enough for OpenCV: its images are rendered from the robot\'s camera pose with the intrinsics in `camera_info`, so detection and `solvePnP` work as on the robot. It has no lens distortion, noise, motion blur or auto exposure.',
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
      cam: { label: 'Camera viewer (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):
    """Show what LIMO's camera sees, and the edges OpenCV finds in it."""

    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.frames = 0

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   # OpenCV images are BGR
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 150)
        self.frames += 1
        if self.frames % 20 == 1:
            h, w = frame.shape[:2]
            b, g, r = frame[h // 2, w // 2]
            self.get_logger().info(f'{w}x{h} image, centre pixel BGR ({b}, {g}, {r}), mean brightness {gray.mean():.0f}')
        cv2.imshow('camera', frame)   # windows appear over the 3D view
        cv2.imshow('edges', edges)
        cv2.waitKey(1)
` + PY_MAIN('CameraViewer') },
      line: { label: 'Line follower (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# yellow in OpenCV's HSV: hue 0-179, saturation and value 0-255
YELLOW_LO = np.array([15, 70, 80])
YELLOW_HI = np.array([40, 255, 255])


class LineFollower(Node):
    """Follow the yellow line: threshold it in HSV, find its centre with moments, steer towards it."""

    def __init__(self):
        super().__init__('line_follower')
        self.declare_parameter('speed', 0.15)   # m/s
        self.declare_parameter('kp', 0.006)     # rad/s per pixel of error
        self.declare_parameter('offset', 0)     # pixels: try -50 to keep the line on your left (right lane)
        self.bridge = CvBridge()
        self.last_err = 0.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.6):, :]               # the floor just ahead of LIMO
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        m = cv2.moments(mask, True)                 # binary image: m00 = number of yellow pixels
        cmd = Twist()
        if m['m00'] > 20:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            err = cx - (w / 2 + self.get_parameter('offset').value)   # + means the line is to the right
            cmd.linear.x = self.get_parameter('speed').value
            cmd.angular.z = -self.get_parameter('kp').value * err
            self.last_err = err
            cv2.circle(roi, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        else:
            cmd.linear.x = 0.05                     # lost the line: creep and turn towards where it was
            cmd.angular.z = -0.5 if self.last_err > 0 else 0.5
        self.pub.publish(cmd)
        cv2.imshow('mask', mask)
        cv2.imshow('line', roi)
        cv2.waitKey(1)
` + PY_MAIN('LineFollower') },
      color: { label: 'Colour tracker (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# red wraps around hue 0 in OpenCV's HSV (0-179), so it needs two ranges
RANGES = [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (179, 255, 255))]
STOP_AREA = 0.06    # stop when the blob covers 6% of the image (a 10 cm ball, about 30 cm away)


class ColourTracker(Node):
    """Find the biggest blob of a colour, turn to centre it and drive up to it."""

    def __init__(self):
        super().__init__('colour_tracker')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros((h, w), np.uint8)
        for lo, hi in RANGES:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # remove specks
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cmd = Twist()
        if contours:
            blob = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(blob) / (w * h)
            (x, y), r = cv2.minEnclosingCircle(blob)
            err = (x - w / 2) / (w / 2)             # -1 (far left) .. +1 (far right)
            cmd.angular.z = -1.2 * err
            if area < STOP_AREA and abs(err) < 0.3:
                cmd.linear.x = 0.15
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
        else:
            cmd.angular.z = 0.5                     # nothing seen: turn on the spot and search
        self.pub.publish(cmd)
        cv2.imshow('tracker', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)
` + PY_MAIN('ColourTracker') },
      aruco: { label: 'ArUco marker reader (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

MARKER_SIZE = 0.12   # metres: the side of the black square


class MarkerReader(Node):
    """Detect ArUco markers (DICT_4X4_50), publish their IDs and estimate where they are."""

    def __init__(self):
        super().__init__('marker_reader')
        self.bridge = CvBridge()
        self.K, self.D = None, None
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # sub-pixel corners: much better poses
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        s = MARKER_SIZE / 2   # marker corners: top-left, top-right, bottom-right, bottom-left
        self.square = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
        self.pub = self.create_publisher(Int32, '/marker_id', 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)   # fx, fy, cx, cy
        self.D = np.array(msg.d, dtype=np.float64)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, i in zip(corners, ids.flatten()):
                self.pub.publish(Int32(data=int(i)))
                if self.K is None:
                    continue
                ok, rvec, tvec = cv2.solvePnP(self.square, c.reshape(4, 2), self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    x, y, z = tvec.flatten()   # camera optical frame: x right, y down, z forward
                    self.get_logger().info(f'marker {i}: {z:.2f} m ahead, {x:+.2f} m to the right',
                                           throttle_duration_sec=1.0)
        cv2.imshow('markers', frame)
        cv2.waitKey(1)
` + PY_MAIN('MarkerReader') },
    },
  },
  cobot: {
    id: 'cobot', name: 'LIMO Cobot', maker: 'AgileX', catLabel: 'Autonomous car + arm', color: '#B144C8', hz: 60, cams: true, mSlam: null, cobot: true,
    blurb: 'A LIMO base carrying a 6-axis arm with a camera on its gripper. Find cubes by colour, pick them with inverse kinematics, sort them into the matching bins, and fix the robot when it breaks.',
    chipsHub: ['6-axis arm', 'camera on the gripper', 'fix-it tickets'], tabs: [['control', 'drive', 'Drive'], ['arm', 'tools', 'Arm'], ['phys', 'chip', 'Physics'], ['term', 'terminal', 'Terminal'], ['code', 'code', 'Code'], ['topics', 'topics', 'Topics'], ['missions', 'flag', 'Missions'], ['fix', 'bug', 'Fix it'], ['vision', 'camera', 'Vision'], ['info', 'book', 'Info']],
    kbd: CAR_KBD, keysHint: 'W A S D drive, Q E slide in mecanum mode. Click the floor to send a navigation goal.', teleop: [0.4, 1.0],
    mainNodes: ['/limo_base', '/arm_driver', '/ydlidar_node', '/arm_camera', '/cube_detector'], arena: COBOT_ARENA, desc: () => cobotDesc(),
    make: (d) => new CobotSim(d),
    view: { dist: 1.35, az: 2.6, el: 0.45, target: [1.2, 2.2, 0.18], minDist: 0.3, maxDist: 10, shadow: 3.4, grid: null, fog: [9, 26] },
    pkgs: { limo_bringup: ['slam.launch.py', 'navigation.launch.py'], cobot_demos: ['pick_place.launch.py', 'go_home', 'open_gripper', 'close_gripper'] },
    chips: ['ros2 topic echo /arm/angles --once', 'ros2 topic pub --once /arm/angles_goal snobots_msgs/msg/ArmAngles "{angles: [0, 0, 0, 0, 0, 0], speed: 50}"', 'ros2 topic pub --once /arm/gripper std_msgs/msg/Int32 "{data: 0}"', 'ros2 service call /arm/go_home std_srvs/srv/Trigger', 'ros2 topic pub --once /arm/coords_goal snobots_msgs/msg/ArmCoords "{x: 180, y: 0, z: 40, rx: 180, ry: 0, rz: 0, speed: 50}"', 'ros2 launch cobot_demos pick_place.launch.py', 'ros2 run tf2_ros tf2_echo arm_base tcp', 'ros2 topic hz /arm_camera/image_raw', 'ros2 topic echo /cube_detections', 'ros2 service call /arm/servos std_srvs/srv/SetBool "{data: true}"'],
    help: `LIMO Cobot simulator terminal (ROS 2 Jazzy).
  ros2 topic pub --once /arm/angles_goal snobots_msgs/msg/ArmAngles "{angles: [0, -30, -60, 0, 0, 0], speed: 50}"
  ros2 topic pub --once /arm/coords_goal snobots_msgs/msg/ArmCoords "{x: 200, y: 0, z: 50, rx: 180, ry: 0, rz: 0, speed: 50}"
  ros2 topic pub --once /arm/gripper std_msgs/msg/Int32 "{data: 100}"     (0 closed … 100 open)
  ros2 service call /arm/go_home std_srvs/srv/Trigger
  ros2 topic echo /arm/angles --once | /arm/coords --once | /joint_states --once
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
  ros2 launch cobot_demos pick_place.launch.py      ros2 launch limo_bringup navigation.launch.py
  ros2 run tf2_ros tf2_echo arm_base tcp     ros2 run tf2_ros tf2_echo arm_base arm_camera_optical_frame
  ros2 topic hz /arm_camera/image_raw        ros2 topic echo /cube_detections
  ros2 service call /arm/servos std_srvs/srv/SetBool "{data: true}"     (false releases them)
  ros2 param get /cube_detector cube_size    ros2 run usb_cam usb_cam_node_exe
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
      { id: 'v1', title: 'Camera check', text: 'In the Terminal, measure the gripper camera\'s frame rate with `ros2 topic hz /arm_camera/image_raw`, then read its intrinsics once: `ros2 topic echo /arm_camera/camera_info --once`.', tab: 'term', xp: 15, vision: true },
      { id: 'v2', title: 'Which colours?', text: 'Park at a cube (Drive tab) and press **Look down** (Arm tab), then run **Which colours?**: your node must publish the colours in the picture on `/cubes_seen` as std_msgs/String (like `red` or `green,red`), right 10 times.', tab: 'vision', xp: 25, vision: true, tpl: 'colours' },
      { id: 'v3', title: 'Centre it', text: 'Park at the red cube, look down, and run **Centre the red cube**: your node must move the arm until the red cube sits in the middle of the picture for 2 s.', tab: 'vision', xp: 35, vision: true, tpl: 'centre' },
      { id: 'v4', title: 'How far down?', text: 'Run **How far down is it?**: from the red cube\'s width in pixels and its real width (30 mm), publish the centre of its top face on `/cube_point` as geometry_msgs/PointStamped in the camera\'s frame. 10 points within 15 mm count.', tab: 'vision', xp: 40, vision: true, tpl: 'size' },
      { id: 'v5', title: 'Pick by sight', text: 'Park at the red cube and run **Pick by sight**: your node finds the cube with the camera, centres it, moves the gripper over it and picks it up.', tab: 'vision', xp: 50, vision: true, tpl: 'vpick' },
    ],
    visionNote: 'The camera on the gripper (`/arm_camera`, a usb_cam) publishes `/arm_camera/image_raw` (320 × 240 `rgb8`, 15 Hz) and `/arm_camera/camera_info` (fx = fy = 200, no distortion) in `arm_camera_optical_frame`. It looks along the gripper from 30 mm beside it: with the gripper pointing down, the top of the picture is the arm\'s +x and its right is the arm\'s −y. Park at a cube (Drive tab) and press **Look down** (Arm tab) to see it from above. The cubes are 30 mm wide. Move the arm with `/arm/coords_goal` (mm, in `arm_base`) and read where it is on `/arm/coords`.',
    specs: [['Maker', 'AgileX Robotics'], ['Type', 'Mobile manipulator: LIMO PRO base + myCobot 280 arm'], ['Size', '322 × 220 × 550 mm'], ['Weight', '≈ 5.6 kg'], ['Arm', '6 DOF, 280 mm working radius'], ['Arm payload', '250 g'], ['Arm weight', '850 g'], ['Base', 'Four motion modes, lidar, depth camera'], ['Software', 'ROS, SLAM navigation, MoveIt'], ['This sim', 'DH chain + numeric IK, 40 mm parallel gripper; a 320 × 240 camera on the gripper']],
    ifaces: [['/arm/angles_goal', 'subscribe', 'snobots_msgs/msg/ArmAngles', '6 joint angles in degrees + speed (like `send_angles`)'], ['/arm/coords_goal', 'subscribe', 'snobots_msgs/msg/ArmCoords', 'Gripper pose x y z (mm), rx ry rz (deg) in the arm base frame (like `send_coords`)'], ['/arm/gripper', 'subscribe', 'std_msgs/msg/Int32', '0 closed … 100 open (like `set_gripper_value`)'], ['/arm/angles', 'publish', 'snobots_msgs/msg/ArmAngles', 'Current angles, 10 Hz'], ['/arm/coords', 'publish', 'snobots_msgs/msg/ArmCoords', 'Current gripper pose, 10 Hz'], ['/joint_states', 'publish', 'sensor_msgs/msg/JointState', 'joint1 … joint6 + gripper, 20 Hz'], ['/arm_camera/image_raw', 'publish', 'sensor_msgs/msg/Image', 'Camera on the gripper (`/arm_camera`): 320 × 240 `rgb8`, 15 Hz, rendered while someone subscribes'], ['/arm_camera/camera_info', 'publish', 'sensor_msgs/msg/CameraInfo', 'fx = fy = 200 px, cx 159.5, cy 119.5, no distortion'], ['/cube_detections', 'publish', 'snobots_msgs/msg/CubeDetection', '`/cube_detector`: each cube it sees, its colour and centre in `arm_base` (distance from its known size)'], ['/arm/servos', 'service', 'std_srvs/srv/SetBool', 'true powers the servos, false releases them for teaching by hand'], ['/arm_driver', 'parameters', 'int / bool / string', '`speed`, `gripper_reversed`, `reference_frame` (arm_base or base_link)'], ['/cube_detector', 'parameters', 'double / string', '`cube_size` (m), `target_frame`, `min_pixels`'], ['/arm/go_home', 'service', 'std_srvs/srv/Trigger', 'Fold the arm'], ['/cmd_vel', 'subscribe', 'geometry_msgs/msg/Twist', 'Base velocity'], ['/odom, /scan, /imu, /limo_status', 'publish', 'as on LIMO', 'Base sensors'], ['/goal_pose', 'subscribe', 'geometry_msgs/msg/PoseStamped', 'Navigation goal (click the floor)']],
    jointDoc: 'Arm joints `joint1` (base) … `joint6` (wrist roll); all zero points the arm straight up. Limits (°): ±168, ±135, ±150, ±145, ±165, ±175. Coordinates for /arm/coords_goal are in the `arm_base` frame: x forward, y left, z up, millimetres; rx 180, ry 0 points the gripper down.',
    realism: 'The arm uses a myCobot-280-style DH chain (about 280 mm reach) and numeric inverse kinematics. It moves through the same physics engine as the base, with link masses, torque-limited servos and real contacts, so the fingers can nudge or knock a cube over. The real arm is driven with pymycobot (send_angles, send_coords, set_gripper_value) or Elephant Robotics\' ROS packages; the `/arm/*` topics are simplified simulator names. Grasping is partly simplified: once both fingers squeeze a cube, it is locked to the gripper so it cannot slip, whereas a real gripper holds it by friction alone. The base slips and its odometry drifts just like LIMO\'s. The camera on the gripper is an ideal pinhole (no lens distortion), and its white balance setting is modelled as a colour cast over the picture. The arm driver, the camera on the gripper and the cube detector are the simulator\'s own versions of such nodes.',
    pyHint: 'Pick a template and press Run. Park facing the PICK table before running the pick template. The camera templates are in the Vision tab.',
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
      cam: { label: 'Cube viewer (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# OpenCV HSV ranges (H 0-179) for the three cubes, measured on this camera's picture: lit tops look paler than the
# paint. They leave out the rest of the view: LIMO's dark-blue deck, its cyan lights and the lidar's orange ring.
# Red wraps around 0, so it has two.
COLOURS = {
    'red': [((0, 100, 120), (4, 255, 255)), ((170, 100, 120), (179, 255, 255))],
    'green': [((55, 80, 100), (81, 255, 255))],
    'blue': [((100, 100, 130), (125, 255, 255))],
}
DRAW = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}   # BGR


def mask_of(hsv, name):
    mask = None
    for lo, hi in COLOURS[name]:
        part = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    return mask


class CubeViewer(Node):
    """Show the camera on the gripper with every coloured cube outlined and labelled."""

    def __init__(self):
        super().__init__('cube_viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/arm_camera/image_raw', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for name in COLOURS:
            contours, _ = cv2.findContours(mask_of(hsv, name), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < 40:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), DRAW[name], 2)
                cv2.putText(frame, f'{name} {w}px', (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, DRAW[name], 1)
        cv2.imshow('arm camera', frame)
        cv2.waitKey(1)
` + PY_MAIN('CubeViewer') },
      colours: { label: 'Which colours? (OpenCV)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# the Cube viewer's ranges, measured on this camera's picture
COLOURS = {
    'red': [((0, 100, 120), (4, 255, 255)), ((170, 100, 120), (179, 255, 255))],
    'green': [((55, 80, 100), (81, 255, 255))],
    'blue': [((100, 100, 130), (125, 255, 255))],
}
MIN_PIXELS = 60     # smaller blobs are noise or a cube too far away


class WhichColours(Node):
    """Say which cube colours are in the picture, on /cubes_seen."""

    def __init__(self):
        super().__init__('which_colours')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, '/cubes_seen', 10)
        self.create_subscription(Image, '/arm_camera/image_raw', self.on_image, 10)

    def on_image(self, msg):
        hsv = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2HSV)
        seen = []
        for name, ranges in COLOURS.items():
            pixels = sum(cv2.countNonZero(cv2.inRange(hsv, np.array(lo), np.array(hi))) for lo, hi in ranges)
            if pixels >= MIN_PIXELS:
                seen.append(name)
        if seen:
            self.pub.publish(String(data=','.join(seen)))
            self.get_logger().info('I see: ' + ', '.join(seen), throttle_duration_sec=1.0)
` + PY_MAIN('WhichColours') },
      centre: { label: 'Centre the red cube (visual servoing)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from snobots_msgs.msg import ArmCoords
from cv_bridge import CvBridge
import cv2
import numpy as np

RED = [((0, 100, 120), (4, 255, 255)), ((170, 100, 120), (179, 255, 255))]   # as in the Cube viewer
GAIN = 0.6        # how much of the error to correct in each move
CUBE_MM = 30.0    # the cube's real width turns pixels into millimetres


class CentreCube(Node):
    """Move the arm until the red cube sits in the middle of the picture (image-based visual servoing)."""

    def __init__(self):
        super().__init__('centre_cube')
        self.bridge = CvBridge()
        self.coords = None
        self.last_move = 0.0
        self.pub = self.create_publisher(ArmCoords, '/arm/coords_goal', 10)
        self.create_subscription(ArmCoords, '/arm/coords', self.on_coords, 10)
        self.create_subscription(Image, '/arm_camera/image_raw', self.on_image, 10)

    def on_coords(self, msg):
        self.coords = msg     # where the gripper is: x y z in mm, in arm_base

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(*[cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in RED])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if self.coords is not None and contours:
            x, y, bw, bh = cv2.boundingRect(max(contours, key=cv2.contourArea))
            u, v = x + bw / 2 - w / 2, y + bh / 2 - h / 2     # pixels from the middle: right, down
            mm = CUBE_MM / max(bw, bh)                          # millimetres per pixel at the cube
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_move > 0.5 and (abs(u) > 3 or abs(v) > 3):
                goal = ArmCoords()
                # gripper pointing down: the top of the picture is the arm's +x, its right is the arm's -y
                goal.x = self.coords.x - GAIN * v * mm
                goal.y = self.coords.y - GAIN * u * mm
                goal.z = self.coords.z
                goal.rx, goal.ry, goal.rz = 180.0, 0.0, 0.0
                goal.speed = 40
                self.pub.publish(goal)
                self.last_move = now
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        cv2.imshow('centre', frame)
        cv2.waitKey(1)
` + PY_MAIN('CentreCube') },
      size: { label: 'How far down is it? (known size)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

RED = [((0, 100, 120), (4, 255, 255)), ((170, 100, 120), (179, 255, 255))]   # as in the Cube viewer
CUBE_M = 0.03     # the cube's real width, in metres


class CubeDistance(Node):
    """One camera has no depth, but a known size gives distance: z = fx * real width / width in pixels."""

    def __init__(self):
        super().__init__('cube_distance')
        self.bridge = CvBridge()
        self.K = None
        self.pub = self.create_publisher(PointStamped, '/cube_point', 10)
        self.create_subscription(CameraInfo, '/arm_camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/arm_camera/image_raw', self.on_image, 10)

    def on_info(self, msg):
        self.K = list(msg.k)      # [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(*[cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in RED])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if self.K is None or not contours:
            return
        x, y, bw, bh = cv2.boundingRect(max(contours, key=cv2.contourArea))
        if max(bw, bh) < 6:
            return
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        z = fx * CUBE_M / max(bw, bh)      # how far the top face is along the optical axis
        u, v = x + bw / 2, y + bh / 2      # the top face's centre, in pixels
        p = PointStamped()
        p.header = msg.header              # the camera's frame and time
        p.point.x = (u - cx) * z / fx      # right
        p.point.y = (v - cy) * z / fy      # down
        p.point.z = z                      # forward, out of the lens
        self.pub.publish(p)
        self.get_logger().info(f'red cube {z * 1000:.0f} mm from the camera ({max(bw, bh)} px wide)', throttle_duration_sec=1.0)
` + PY_MAIN('CubeDistance') },
      vpick: { label: 'Pick by sight (visual servoing)', vision: true, code: PY_HEAD + `from sensor_msgs.msg import Image
from snobots_msgs.msg import ArmCoords
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

RED = [((0, 100, 120), (4, 255, 255)), ((170, 100, 120), (179, 255, 255))]   # as in the Cube viewer
FX = 200.0          # focal length in pixels (from /arm_camera/camera_info)
CUBE_MM = 30.0
CAM_AHEAD = 30.0    # the camera sits 30 mm ahead of the gripper (arm +x when it points down)
CAM_ABOVE = 56.0    # ... and 56 mm above the gripper's grasp point


class PickBySight(Node):
    """Look down, centre the red cube under the camera, move the gripper over it, then grab it."""

    def __init__(self):
        super().__init__('pick_by_sight')
        self.bridge = CvBridge()
        self.coords, self.cube = None, None
        self.state, self.t0, self.centred = 'look', 0.0, 0
        self.arm = self.create_publisher(ArmCoords, '/arm/coords_goal', 10)
        self.grip = self.create_publisher(Int32, '/arm/gripper', 10)
        self.create_subscription(ArmCoords, '/arm/coords', self.on_coords, 10)
        self.create_subscription(Image, '/arm_camera/image_raw', self.on_image, 10)
        self.create_timer(0.1, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def go(self, state):
        self.state, self.t0 = state, self.now()
        self.get_logger().info(state)

    def move(self, x, y, z, speed=40):
        g = ArmCoords()
        g.x, g.y, g.z, g.rx, g.ry, g.rz, g.speed = float(x), float(y), float(z), 180.0, 0.0, 0.0, speed
        self.arm.publish(g)

    def on_coords(self, msg):
        self.coords = msg

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(*[cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in RED])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.cube = None
        if contours:
            x, y, bw, bh = cv2.boundingRect(max(contours, key=cv2.contourArea))
            if max(bw, bh) >= 6:
                self.cube = (x + bw / 2 - w / 2, y + bh / 2 - h / 2, max(bw, bh))   # u, v from the middle, width
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.imshow('pick by sight', frame)
        cv2.waitKey(1)

    def step(self):
        c, t = self.coords, self.now() - self.t0
        if c is None:
            return
        if self.state == 'look':                          # look down at the table
            self.grip.publish(Int32(data=100))
            self.move(150, 0, 120, 60)
            self.go('find')
        elif self.state == 'find' and t > 2.5:
            self.go('centre' if self.cube else 'look')
        elif self.state == 'centre' and t > 0.5 and self.cube:
            u, v, width = self.cube
            mm = CUBE_MM / width
            if abs(u) < 4 and abs(v) < 4:
                self.centred += 1
                if self.centred >= 3:                     # steady in the middle: work out the height
                    self.depth = FX * CUBE_MM / width     # mm from the camera to the cube's top
                    self.move(c.x + CAM_AHEAD, c.y, c.z)  # put the gripper where the camera was looking
                    self.go('over')
            else:
                self.centred = 0
                self.move(c.x - 0.6 * v * mm, c.y - 0.6 * u * mm, c.z)
                self.t0 = self.now()
        elif self.state == 'over' and t > 1.5:
            # the cube's middle is half a cube below its top: lower the grasp point there
            self.move(c.x, c.y, c.z + CAM_ABOVE - self.depth - CUBE_MM / 2, 30)
            self.go('down')
        elif self.state == 'down' and t > 2.5:
            self.grip.publish(Int32(data=0))
            self.go('grip')
        elif self.state == 'grip' and t > 1.2:
            self.move(c.x, c.y, c.z + 120)
            self.go('done')
` + PY_MAIN('PickBySight') },
    },
  },
};
