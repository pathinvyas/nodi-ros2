/* ============================== SnoBots: broken-robot missions ("Fix it") ============================== */
// Each ticket starts the robot with a real fault in its ROS graph: a crashed driver, a bad parameter, a leftover remap,
// mismatched QoS, a missing transform, a wrong frame_id or the wrong message type. You find the cause and fix it from
// the terminal with ros2 commands, the way you would on a real robot. A ticket's setup is the robot's start-up
// configuration (see LimoSim.restoreRobot, AinexSim.restoreRobot and R1Sim.restoreRobot); its check reads the live simulation, so any fix
// that works, counts.
const FIX_TICKETS = {
  limo: [
    {
      id: 'lidar-down', title: "The map won't grow", level: 1, xp: 40, skills: ['nodes', 'topics'], from: 'Sam',
      report: 'I started SLAM like always, but the map stays grey and the red lidar dots are gone from the 3D view. Nothing looks unplugged. Can you check?',
      goal: 'Get `/scan` publishing again so SLAM can build the map.',
      setup: () => ({ lidar: false, died: 'lidar', slam: true }),
      check: (sim, st) => ({ items: [['`/scan` has a publisher', !!sim.procs.lidar && sim.bus.active(sim.tn.scan || '/scan')], ['SLAM is receiving scans', st.slam >= 15]] }),
      hints: ['When data stops, first check what is running: `ros2 node list`. Compare it with the launch output in the robot log.', 'SLAM subscribes to `/scan`. Who publishes it? `ros2 topic info /scan`', 'Start the driver again: `ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node`. It keeps running in that terminal, so open a second terminal (+) for more commands.'],
      fix: 'ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node',
      debrief: 'The lidar driver had crashed at start-up, so nothing published `/scan`. ROS raises no alarm when a publisher disappears: subscribers like SLAM simply wait. Real drivers die when the USB device is not ready at boot, a cable is loose or the port name is wrong, and the launch output says so (**process has died**). Your first checks are `ros2 node list` (is it running?) and `ros2 topic info` (how many publishers?). A launch file can restart a crashed node for you with `respawn=True`.',
    },
    {
      id: 'slow', title: 'LIMO is crawling', level: 1, xp: 40, skills: ['parameters'], from: 'Priya',
      report: 'I push the joystick all the way and LIMO barely moves. The motors sound fine. Did someone change a setting for the open-house demo?',
      goal: 'Get LIMO driving at 0.4 m/s or faster.',
      setup: () => ({ base: { params: { max_linear_speed: 0.08 } }, joy: 'twist' }),
      check: (sim, st) => ({ items: [[`LIMO reaches 0.4 m/s (best so far ${st.vmax.toFixed(2)} m/s)`, st.vmax >= 0.38]] }),
      hints: ['Compare what you ask for with what LIMO does: drive with the joystick while `ros2 topic echo /odom` runs, and look at `twist.twist.linear.x`.', 'Drivers clamp commands with parameters. List them: `ros2 param list /limo_base`', '`ros2 param get /limo_base max_linear_speed`, then `ros2 param set /limo_base max_linear_speed 1.0` (write 1.0, not 1: it is a double).'],
      fix: 'ros2 param set /limo_base max_linear_speed 1.0',
      debrief: '`max_linear_speed` had been turned down to 0.08 m/s, so the driver clipped every command. Parameters let you tune a node without changing its code, and they are easy to forget after a demo. `ros2 param list`, `get` and `dump` show what a node is really using. To make a value stick, put it in the node\'s YAML parameter file instead of setting it by hand each time.',
    },
    {
      id: 'remap', title: 'LIMO ignores the joystick', level: 2, xp: 60, skills: ['remapping', 'topics'], from: 'Kai',
      report: "Since Alex's two-robot demo, LIMO doesn't react to the joystick at all. The Control tab says `/cmd_vel` is being published, so the joystick works. What's going on?",
      goal: 'Drive LIMO half a metre with the joystick (Control tab or W A S D).',
      setup: () => ({ base: { remaps: [['cmd_vel', '/limo/cmd_vel']] }, joy: 'twist' }),
      check: (sim, st) => ({ items: [[`LIMO drives from the joystick (${Math.min(st.joy, 0.5).toFixed(2)} of 0.5 m)`, st.joy >= 0.5]] }),
      hints: ['Who listens on `/cmd_vel`? `ros2 topic info /cmd_vel`', 'What does the base actually subscribe to? `ros2 node info /limo_base`', 'Restart the driver without the leftover remap: `ros2 run limo_base limo_base_node`. Or bridge the two names: `ros2 run topic_tools relay /cmd_vel /limo/cmd_vel`'],
      fix: 'ros2 run limo_base limo_base_node',
      debrief: 'The base had been started with a remap left over from a two-robot setup (`cmd_vel:=/limo/cmd_vel`), so it listened on `/limo/cmd_vel` while the joystick published `/cmd_vel`. Two nodes only talk when the names match exactly. Remaps and namespaces are how you run several robots side by side, and `ros2 node info` shows the names a node really ended up with.',
    },
    {
      id: 'qos', title: "Echo works, SLAM doesn't", level: 2, xp: 70, skills: ['QoS'], from: 'Jordan',
      report: "The lidar is fine: `ros2 topic echo /scan` prints ranges all day. But SLAM never adds anything to the map. Someone 'optimised' the lidar settings yesterday to save Wi-Fi bandwidth.",
      goal: 'Get SLAM receiving scans so the map grows.',
      setup: () => ({ lidar: { params: { 'qos_overrides./scan.publisher.reliability': 'best_effort' } }, slam: true }),
      check: (sim, st) => ({ items: [['SLAM is receiving scans', st.slam >= 15]] }),
      hints: ["If echo sees the data but a node doesn't, compare how they connect: `ros2 topic info /scan --verbose`", 'Compare Reliability for the publisher and the subscription. A RELIABLE subscription cannot take messages from a BEST_EFFORT publisher, so the two never connect.', 'Restart the lidar reliably: `ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p qos_overrides./scan.publisher.reliability:=reliable` (or restart SLAM with `--ros-args -p qos_overrides./scan.subscription.reliability:=best_effort`).'],
      fix: 'ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p qos_overrides./scan.publisher.reliability:=reliable',
      debrief: 'The lidar published `/scan` as BEST_EFFORT, but SLAM asked for RELIABLE. A subscription only connects to a publisher that offers at least what it requests, so the two never connected, and each side logged one quiet warning about incompatible QoS. `ros2 topic echo` adapts its own QoS to the publishers, which is why it worked and made the problem confusing. `ros2 topic info --verbose` shows every endpoint\'s profile. Sensor data is often sent best effort on purpose, so make sure every subscriber agrees.',
    },
    {
      id: 'tf-missing', title: 'Where is the lidar?', level: 2, xp: 70, skills: ['TF'], from: 'Robin',
      report: 'SLAM keeps printing warnings about the laser pose and the map stays empty, even though the lidar spins. For reference: the lidar sits 9 cm in front of the centre of `base_link` and 18 cm above it, facing forward.',
      goal: 'Publish `base_link` → `laser_link` with the right numbers so SLAM builds the map.',
      setup: () => ({ laser_tf: false, slam: true }),
      check: (sim, st) => { const e = fixTfError(sim, 'laser_link'); return { items: [['`base_link` → `laser_link` exists', !!e], ['…at the lidar mount', !!e && e.ok], ['SLAM is receiving scans', st.slam >= 15]], note: e && !e.ok ? fixOff(e, 'The frame exists, but it is') + ' Stop that publisher (Ctrl+C) and try again with the numbers from the report.' : '' }; },
      hints: ["Read SLAM's warning in the robot log, then check the tree: `ros2 run tf2_ros tf2_echo base_link laser_link`", 'Fixed offsets between parts of a robot are static transforms on `/tf_static`. See what is there: `ros2 topic echo /tf_static`', 'Publish it yourself: `ros2 run tf2_ros static_transform_publisher --x 0.09 --y 0 --z 0.18 --frame-id base_link --child-frame-id laser_link`'],
      fix: 'ros2 run tf2_ros static_transform_publisher --x 0.09 --y 0 --z 0.18 --frame-id base_link --child-frame-id laser_link',
      debrief: "Nothing published the transform from `base_link` to `laser_link`, so SLAM couldn't tell where each scan was taken from. Fixed offsets like a sensor mount are static transforms, usually published by `robot_state_publisher` from the robot's URDF, or by a `static_transform_publisher` in the launch file. Measure carefully: a wrong offset raises no error, it just quietly smears the map.",
    },
    {
      id: 'frame-id', title: 'Two names for one lidar', level: 3, xp: 90, skills: ['TF', 'parameters'], from: 'Sam',
      report: "We copied the lidar settings from another robot, and now SLAM can't place the scans. But `tf2_echo base_link laser_link` works fine! I don't get it.",
      goal: "Make the scans' frame part of the TF tree so SLAM builds the map.",
      setup: () => ({ lidar: { params: { frame_id: 'laser' } }, slam: true }),
      check: (sim, st) => { const fr = String(sim.P('/ydlidar_node', 'frame_id') || ''), e = fr ? fixTfError(sim, fr) : null; return { items: [["The scans' frame is in the TF tree", !!e], ['…at the lidar mount', !!e && e.ok], ['SLAM is receiving scans', st.slam >= 15]], note: e && !e.ok ? fixOff(e, `'${fr}' is in the tree now, but it is`) : '' }; },
      hints: ['Which frame do the scans say they were taken in? `ros2 topic echo /scan --once`, then look at `header.frame_id`.', 'Does the tree know that frame? `ros2 run tf2_ros tf2_echo base_link laser`', 'Restart the driver with the right name: `ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p frame_id:=laser_link` (a zero-offset static transform from `laser_link` to `laser` works too).'],
      fix: 'ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args -p frame_id:=laser_link',
      debrief: "Every sensor message names its frame in `header.frame_id`. The driver was stamping scans with `laser`, while this robot's tree calls the lidar `laser_link`, so lookups failed even though the transform existed. It happens all the time after copying settings between robots. Check a message's `frame_id` against `tf2_echo`, and keep frame names in one place, such as the URDF.",
    },
    {
      id: 'stamped', title: 'Same topic, different language', level: 3, xp: 90, skills: ['message types'], from: 'Priya',
      report: "Since the Jazzy update, the joystick publishes on `/cmd_vel` and the base subscribes to `/cmd_vel`, but LIMO won't move. Same topic name, so what else could be wrong?",
      goal: 'Drive LIMO half a metre with the joystick.',
      setup: () => ({ joy: 'stamped' }),
      check: (sim, st) => ({ items: [[`LIMO drives from the joystick (${Math.min(st.joy, 0.5).toFixed(2)} of 0.5 m)`, st.joy >= 0.5]] }),
      hints: ["The same name doesn't mean the same message: `ros2 topic list -t`", 'Which type does each side use? `ros2 topic info /cmd_vel --verbose`', 'Make the joystick send plain Twist again: `ros2 param set /teleop_panel publish_stamped_twist false`'],
      fix: 'ros2 param set /teleop_panel publish_stamped_twist false',
      debrief: "The joystick sent `geometry_msgs/msg/TwistStamped`, but the base expects `geometry_msgs/msg/Twist`. A publisher and a subscription only connect when both the name and the type match, and nothing warns you when they don't. Jazzy made this mix-up common: some packages now expect TwistStamped (ros2_control's diff drive controller, for example) while many drivers still use Twist, and teleop nodes have a parameter to choose. A real teleop node reads that parameter at start-up, so there you would restart it with `--ros-args -p publish_stamped_twist:=false`.",
    },
  ],
  ainex: [
    {
      id: 'cam-down', title: 'AiNex is blind', level: 1, xp: 40, skills: ['nodes', 'topics'], from: 'Mei',
      report: "Head tracking is on, but AiNex just stares ahead when I hold the red block in front of it, and the Vision tab shows no picture. The camera's light is on. It worked yesterday!",
      goal: 'Get the head camera publishing `/image_raw` again so the colour detector sees the block.',
      setup: () => ({ cam: false, died: 'cam', tracking: true }),
      check: (sim, st) => ({ items: [['`/image_raw` has a publisher', fixPubs(sim, '/image_raw') > 0], ['The colour detector is getting images', st.det >= 15]] }),
      hints: ['Start where the data begins: `ros2 topic info /image_raw`. Who publishes it?', 'Is the camera driver running? `ros2 node list`, then read the robot log in the Fix it tab: what happened to `usb_cam_node_exe`?', 'Start it again: `ros2 run usb_cam usb_cam_node_exe`. It keeps running in that terminal, so open a second terminal (+) for more commands.'],
      fix: 'ros2 run usb_cam usb_cam_node_exe',
      debrief: 'The camera driver died at start-up because `/dev/video0` was not there yet (a USB camera can take a moment to appear after boot), so nothing published `/image_raw` and the colour detector simply waited. Check a sensor the same way every time: `ros2 node list` (is the driver running?), `ros2 topic info` (how many publishers?), and the launch output for the reason. A launch file can retry a crashed driver for you with `respawn=True`.',
    },
    {
      id: 'min-area', title: 'Head tracking ignores the red block', level: 1, xp: 40, skills: ['parameters'], from: 'Theo',
      report: "The camera works (I can see the red block in the Vision tab) and head tracking is on, but AiNex won't look at the block. Someone tuned the colour detector at the expo so it would ignore red T-shirts in the crowd.",
      goal: 'Head tracking keeps the red block centred for 2 seconds.',
      setup: () => ({ det: { params: { min_area: 0.02 } }, tracking: true }),
      check: (sim, st) => ({ items: [['The colour detector reports the red block', st.red >= 10], [`Head tracking keeps it centred (${Math.min(st.trk, 2).toFixed(1)} of 2 s)`, st.trk >= 2]] }),
      hints: ['See what the detector says: `ros2 topic echo /color_detection`. Is `found` ever true?', 'Detectors have thresholds. List them with `ros2 param list /color_detector`, then `ros2 param get` each one. `area` is the fraction of the picture a blob covers: how big could a 5 cm block be?', 'Lower the size threshold: `ros2 param set /color_detector min_area 0.0001`'],
      fix: 'ros2 param set /color_detector min_area 0.0001',
      debrief: '`min_area` was 0.02, so a blob had to cover 2% of the picture to count. The red block is 5 cm across and over a metre away, so in a 120° wide-angle picture it covers about 0.03%, and every detection was thrown away. Thresholds are tuned for one place and one distance, and wide-angle lenses make things look much smaller than you expect. Echo a detector\'s output and check its parameters before you blame the camera.',
    },
    {
      id: 'trip', title: 'AiNex trips over its own feet', level: 2, xp: 60, skills: ['parameters'], from: 'Priya',
      report: 'Someone tuned the gait so AiNex walks more quietly on the demo table. Now it stumbles and falls after a few steps. Standing and waving are fine.',
      goal: 'Walk AiNex half a metre without falling.',
      setup: () => ({ ctl: { params: { step_height: 0.008 } } }),
      check: (sim, st) => ({ items: [[`AiNex walks 0.5 m without falling (${Math.min(st.walk, 0.5).toFixed(2)} of 0.5 m)`, st.walk >= 0.5]] }),
      hints: ['Walk a few steps (W, or the joystick) and watch the feet closely in the 3D view. What does the swinging foot do?', 'Gait settings are parameters of the walking controller: `ros2 param dump /ainex_controller`. Which one decides how high a foot lifts?', 'Lift the feet again: `ros2 param set /ainex_controller step_height 0.025`. If AiNex is lying down, press Stand up in the Control tab first.'],
      fix: 'ros2 param set /ainex_controller step_height 0.025',
      debrief: '`step_height` had been lowered to 8 mm. The swinging foot barely left the floor, scuffed it and tripped the robot. A gait is a set of trade-offs: lower steps are quieter but leave no margin for an uneven floor or a little body sway. Change one gait parameter at a time, test on the floor, and keep the values that work in the controller\'s parameter file.',
    },
    {
      id: 'cam-remap', title: 'Pictures, but nobody looks', level: 2, xp: 60, skills: ['remapping', 'topics'], from: 'Kai',
      report: "Since Alex's two-robot demo, head tracking does nothing. The camera is fine: in the Vision tab I can pick its picture and see the pitch. But the colour detector never finds anything.",
      goal: 'Get the camera images to the colour detector.',
      setup: () => ({ cam: { remaps: [['image_raw', '/ainex/image_raw'], ['camera_info', '/ainex/camera_info']] }, tracking: true }),
      check: (sim, st) => ({ items: [['The colour detector is getting images', st.det >= 15]] }),
      hints: ['Which image topics exist? `ros2 topic list`, then `ros2 topic info` on each one.', 'What does each node really use? `ros2 node info /usb_cam` and `ros2 node info /color_detector`', 'Restart the driver without the leftover remap: `ros2 run usb_cam usb_cam_node_exe`. Or bridge the two names: `ros2 run topic_tools relay /ainex/image_raw /image_raw`'],
      fix: 'ros2 run usb_cam usb_cam_node_exe',
      debrief: 'The camera had been started with remaps left over from a two-robot setup (`image_raw:=/ainex/image_raw`), so it published `/ainex/image_raw` while the detector waited on `/image_raw`. Nothing warns you: a publisher and a subscription on different names simply never meet. `ros2 node info` shows the names a node really ended up with. For several robots, give each robot its own namespace instead of renaming topics one at a time.',
    },
    {
      id: 'cam-qos', title: 'Echo sees pictures, the detector does not', level: 2, xp: 70, skills: ['QoS'], from: 'Jordan',
      report: "`ros2 topic hz /image_raw` says the camera runs at 15 Hz, but the colour detector never sees a thing and head tracking is dead. Someone changed the camera's settings yesterday to save Wi-Fi bandwidth.",
      goal: 'Get the camera images to the colour detector.',
      setup: () => ({ cam: { params: { 'qos_overrides./image_raw.publisher.reliability': 'best_effort' } }, tracking: true }),
      check: (sim, st) => ({ items: [['The colour detector is getting images', st.det >= 15]] }),
      hints: ["If a tool gets the data but a node doesn't, compare how they connect: `ros2 topic info /image_raw --verbose`", 'Compare Reliability for the publisher and the subscription, and read the warnings in the robot log. A RELIABLE subscription cannot take messages from a BEST_EFFORT publisher.', 'Restart the camera reliably: `ros2 run usb_cam usb_cam_node_exe --ros-args -p qos_overrides./image_raw.publisher.reliability:=reliable` (or restart the detector with `ros2 run ainex_vision color_detector --ros-args -p qos_overrides./image_raw.subscription.reliability:=best_effort`).'],
      fix: 'ros2 run usb_cam usb_cam_node_exe --ros-args -p qos_overrides./image_raw.publisher.reliability:=reliable',
      debrief: 'The camera published `/image_raw` as BEST_EFFORT, but the colour detector asked for RELIABLE. A subscription only connects to a publisher that offers at least what it requests, so the two never connected, and each side logged one warning about incompatible QoS. `ros2 topic hz` and `echo` adapt their QoS to the publisher, which is why they worked. Images are often sent best effort on purpose (a late frame is useless), so make every image subscriber agree, for example with the sensor-data QoS profile.',
    },
    {
      id: 'frame-id', title: 'Where is the ball?', level: 3, xp: 90, skills: ['TF', 'parameters'], from: 'Sam',
      report: "We copied the camera settings from our other robot. The pictures look perfect, but `/ball_position` stays silent and the ball locator keeps complaining about a frame. `tf2_echo base_link camera_optical_frame` works, so the tree is fine, right?",
      goal: "Make the images' frame part of the TF tree so the ball locator can place the ball.",
      setup: () => ({ cam: { params: { frame_id: 'camera' } } }),
      check: (sim, st) => { const fr = String(sim.P('/usb_cam', 'frame_id') || ''), e = fr ? fixTfError(sim, fr, 'camera_optical_frame') : null; return { items: [["The images' frame is in the TF tree", !!e], ['…at the camera, with z pointing forward', !!e && e.ok], ['`/ball_locator` places the ball', st.ballOk >= 10]], note: e && !e.ok ? fixOff(e, `'${fr}' is in the tree now, but it is`, 'so the ball lands in the wrong place') + ' Stop that publisher (Ctrl+C) and try again.' : '' }; },
      hints: ['Which frame do the images say they were taken in? `ros2 topic echo /camera_info --once`, then look at `header.frame_id`.', 'Does the tree know that frame? `ros2 run tf2_ros tf2_echo base_link camera`', "Restart the camera with this robot's frame name: `ros2 run usb_cam usb_cam_node_exe --ros-args -p frame_id:=camera_optical_frame` (a zero-offset static transform from `camera_optical_frame` to `camera` works too)."],
      fix: 'ros2 run usb_cam usb_cam_node_exe --ros-args -p frame_id:=camera_optical_frame',
      debrief: "Every image names the frame it was taken in, in `header.frame_id`. The copied settings stamped the images `camera`, but in AiNex's tree the camera is `camera_optical_frame`, so every lookup failed even though the camera's transform existed. Mind the optical convention as well: image frames point z forward, x right and y down, while `camera_link` points x forward. If you bridge the names with a static transform, hang it from `camera_optical_frame` with no rotation, not from `camera_link`.",
    },
    {
      id: 'stamped', title: "AiNex won't walk from the joystick", level: 3, xp: 90, skills: ['message types'], from: 'Priya',
      report: "After the Jazzy update the joystick publishes on `/cmd_vel` and the walking controller subscribes to `/cmd_vel`, but AiNex won't take a single step. Publishing from the terminal with `ros2 topic pub` works fine!",
      goal: 'Walk AiNex half a metre with the joystick (Control tab or W A S D).',
      setup: () => ({ joy: 'stamped' }),
      check: (sim, st) => ({ items: [[`AiNex walks from the joystick (${Math.min(st.joy, 0.5).toFixed(2)} of 0.5 m)`, st.joy >= 0.5]] }),
      hints: ["The same name doesn't mean the same message: `ros2 topic list -t`", 'Which type does each side use? `ros2 topic info /cmd_vel --verbose`', 'Make the joystick send plain Twist again: `ros2 param set /teleop_panel publish_stamped_twist false`'],
      fix: 'ros2 param set /teleop_panel publish_stamped_twist false',
      debrief: "The joystick sent `geometry_msgs/msg/TwistStamped`, but the walking controller expects `geometry_msgs/msg/Twist`. A publisher and a subscription only connect when both the name and the type match, and nothing warns you when they don't. `ros2 topic pub` worked because you typed the type yourself. Jazzy made this mix-up common, since some packages now take TwistStamped velocity commands while others still take Twist.",
    },
  ],
  r1: [
    {
      id: 'cam-down', title: "R1 doesn't see anyone", level: 1, xp: 40, skills: ['nodes', 'topics'], from: 'Mei',
      report: "Follow me is switched on and the visitor is standing right in front of R1, but R1 just stares straight ahead. The Vision tab has no picture at all. It worked at yesterday's open day!",
      goal: 'Get the head camera publishing again so the person locator gets images.',
      setup: () => ({ cam: false, died: 'cam', follow: true, guest: 'stand' }),
      check: (sim, st) => ({ items: [['`/head_camera/color/image_raw` has a publisher', fixPubs(sim, '/head_camera/color/image_raw') > 0], ['The person locator is getting images', st.locC >= 15]] }),
      hints: ['Start where the data begins: `ros2 topic info /head_camera/color/image_raw`. Who publishes it?', 'Is the camera driver running? `ros2 node list`, then read the robot log in the Fix it tab: what happened to `head_camera_node`?', 'Start it again: `ros2 run r1_camera head_camera_node`. It keeps running in that terminal, so open a second terminal (+) for more commands.'],
      fix: 'ros2 run r1_camera head_camera_node',
      debrief: 'The camera driver died at start-up because it found no stereo camera on USB yet (a camera can take a moment to appear after boot), so nothing published colour or depth and the person locator simply waited. Check a sensor the same way every time: `ros2 node list` (is the driver running?), `ros2 topic info` (how many publishers?), and the launch output for the reason. A launch file can retry a crashed driver for you with `respawn=True`.',
    },
    {
      id: 'mode', title: 'R1 ignores the joystick', level: 1, xp: 40, skills: ['services'], from: 'Ravi',
      report: "I was the last one to use R1, for the PD lab in the gantry. Now it just hangs there and the joystick does nothing at all. I'm sure I didn't break anything!",
      goal: 'Walk R1 half a metre with the joystick (Control tab or W A S D).',
      setup: () => ({ lowlevel: true, joy: 'twist' }),
      check: (sim, st) => { const m = sim.locoStateRaw(); return { items: [[`R1 is in a mode that walks (now: ${m})`, ['standing', 'walking', 'running', 'run_ready'].includes(m)], [`R1 walks from the joystick (${Math.min(st.joy, 0.5).toFixed(2)} of 0.5 m)`, st.joy >= 0.5]] }; },
      hints: ['What state is the robot in? `ros2 topic echo /loco/state`, and read the controller\'s warning in the robot log.', 'Modes are changed with a service. List them with `ros2 service list`, and see what one takes: `ros2 interface show snobots_msgs/srv/SetMode`', 'Ask for walking mode: `ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: walk}"`. The gantry sets R1 down on its feet.'],
      fix: 'ros2 service call /loco/set_mode snobots_msgs/srv/SetMode "{mode: walk}"',
      debrief: "R1 was still in low-level mode from the PD lab: its motors obeyed `/lowcmd` only, and the locomotion controller ignored `/cmd_vel` (the robot log said so). Robots like R1 run a state machine (damp, stand, walk, run, low-level), and many commands only work in some states. Check the state first (`/loco/state`), then change it with the service the robot offers. A service fits the job: you ask, and you get an answer back, here `success: true` and the new mode.",
    },
    {
      id: 'fm-remap', title: 'Head yes, feet no', level: 2, xp: 60, skills: ['remapping', 'topics'], from: 'Kai',
      report: "Since Alex's two-robot demo, follow me is half broken: R1 turns its head to look at the visitor, but it won't take a single step toward them. Driving with the joystick works fine.",
      goal: 'Make follow me walk R1 over to the visitor.',
      setup: () => ({ follow: { remap: '/r1_b/cmd_vel' }, guest: 'stand' }),
      check: (sim, st) => ({ items: [['The person locator finds the visitor', st.loc >= 5], [`Follow me walks R1 toward the visitor (${Math.min(st.fm, 0.5).toFixed(2)} of 0.5 m)`, st.fm >= 0.5]] }),
      hints: ['Who publishes `/cmd_vel`, and who listens? `ros2 topic info /cmd_vel --verbose`. Then look for other velocity topics: `ros2 topic list`', 'What does the follow-me node really publish? `ros2 node info /follow_me`', 'Bridge the two names: `ros2 run topic_tools relay /r1_b/cmd_vel /cmd_vel`. Or restart follow me without the leftover remap: switch it off and on again in the Control tab.'],
      fix: 'ros2 run topic_tools relay /r1_b/cmd_vel /cmd_vel',
      debrief: "The follow-me node had been started with a remap left over from a two-robot demo (`cmd_vel:=/r1_b/cmd_vel`), so its walking commands went to a robot that wasn't there, while its head commands on `/joint_commands` still reached R1. A node that half works is a strong clue: compare what it publishes (`ros2 node info`) with what the robot subscribes to. To run several robots, give each one its own namespace instead of renaming topics one at a time.",
    },
    {
      id: 'look-away', title: 'R1 looks away from people', level: 2, xp: 60, skills: ['parameters', 'control'], from: 'Theo',
      report: 'Follow me is on and the visitor is right there, but every time R1 notices them it turns its head away, as if it were shy. Someone changed a setting for the mirror-image demo at the expo.',
      goal: 'R1 keeps its eyes on the visitor for 3 seconds in a row.',
      setup: () => ({ follow: { head_kp: -0.6 }, guest: 'stand' }),
      check: (sim, st) => ({ items: [['The person locator finds the visitor', st.loc >= 5], [`R1 looks at the visitor (${Math.min(st.head, 3).toFixed(1)} of 3 s in a row)`, st.head >= 3]] }),
      hints: ['Watch the head in the 3D view, or `ros2 topic echo /joint_commands`: which way does each command turn it?', 'The follow-me node turns the head with a proportional controller. `ros2 param list /follow_me`, then `ros2 param get` its gain.', 'A gain with the wrong sign pushes away from the target. Flip it back: `ros2 param set /follow_me head_kp 0.6`'],
      fix: 'ros2 param set /follow_me head_kp 0.6',
      debrief: '`head_kp` was −0.6. A proportional controller turns by kp times the error, so a negative gain turned the head away from the visitor, harder the further off-centre they were. The visitor left the picture, the head drifted back, saw them and turned away again. Sign errors are among the most common control bugs (a mirrored image, a flipped axis, a motor wired backwards). Try a new control loop with a small gain first, and check that it moves the right way.',
    },
    {
      id: 'depth-qos', title: 'Depth at 15 Hz, but nobody is found', level: 2, xp: 70, skills: ['QoS'], from: 'Jordan',
      report: "`ros2 topic hz /head_camera/depth/image_raw` says depth runs at 15 Hz, and the colour images are fine too. But follow me never moves, and the person locator complains about missing depth. Someone changed the camera's settings yesterday to save Wi-Fi bandwidth.",
      goal: 'Get the depth images to the person locator so it places the visitor.',
      setup: () => ({ cam: { params: { 'qos_overrides./head_camera/depth/image_raw.publisher.reliability': 'best_effort' } }, follow: true, guest: 'stand' }),
      check: (sim, st) => ({ items: [['The person locator places the visitor', st.locOk >= 10]] }),
      hints: ["If a tool gets the data but a node doesn't, compare how they connect: `ros2 topic info /head_camera/depth/image_raw --verbose`", 'Compare Reliability for the publisher and the subscription, and read the warnings in the robot log. A RELIABLE subscription cannot take messages from a BEST_EFFORT publisher.', 'Restart the camera reliably: `ros2 run r1_camera head_camera_node --ros-args -p qos_overrides./head_camera/depth/image_raw.publisher.reliability:=reliable` (or restart the locator with `ros2 run r1_vision person_locator --ros-args -p qos_overrides./head_camera/depth/image_raw.subscription.reliability:=best_effort`).'],
      fix: 'ros2 run r1_camera head_camera_node --ros-args -p qos_overrides./head_camera/depth/image_raw.publisher.reliability:=reliable',
      debrief: 'The camera published depth as BEST_EFFORT, but the person locator asked for RELIABLE. A subscription only connects to a publisher that offers at least what it requests, so depth never reached the locator while colour (still RELIABLE) did: it saw pictures but could not place anyone in them. `ros2 topic hz` adapts its QoS to the publisher, which is why it worked. For camera streams best effort is often the right choice (a late frame is useless), so make every subscriber agree, for example with the sensor-data QoS profile.',
    },
    {
      id: 'depth-scale', title: 'The visitor is a kilometre away', level: 3, xp: 90, skills: ['units', 'parameters'], from: 'Sam',
      report: "We copied the person locator's settings from our other robot, whose camera sends depth as 32-bit floats in metres. Since then follow me ignores the visitor, and the log says they are over a thousand metres away. They're standing right there!",
      goal: 'Make `/person_position` put the visitor where they really are.',
      setup: () => ({ loc: { params: { depth_scale: 1.0 } }, follow: true, guest: 'stand' }),
      check: (sim, st) => ({ items: [['`/person_position` is within 10 cm of the visitor', st.locOk >= 10]] }),
      hints: ["What unit is R1's depth in? `ros2 topic echo /head_camera/depth/image_raw --once` and look at `encoding`. Or pick the depth topic in the Vision tab and click the visitor: compare the raw number with how far away they stand.", 'The locator multiplies each raw depth value by a scale to get metres: `ros2 param get /person_locator depth_scale`', '`16UC1` depth is whole millimetres, so the scale is 0.001: `ros2 param set /person_locator depth_scale 0.001`'],
      fix: 'ros2 param set /person_locator depth_scale 0.001',
      debrief: "R1's camera sends depth as `16UC1`, whole millimetres; the other robot's camera sends `32FC1`, metres as floats. ROS uses both (REP 118), and the image's `encoding` field says which one you have. With `depth_scale` 1.0, a visitor 2 m away read as 2000 m, far beyond `max_range`, so they were ignored. Check the encoding before you trust the numbers, and convert units in one place.",
    },
    {
      id: 'optical', title: 'A frame went missing', level: 3, xp: 90, skills: ['TF'], from: 'Robin',
      report: "We put last year's robot description back on R1. Since then the person locator only says that `camera_optical_frame` does not exist. Last year's description has `camera_link`, which looks forward along x. Images use the optical convention instead: z forward, x right, y down, from the same spot.",
      goal: 'Publish `camera_link` → `camera_optical_frame` with the right rotation, so the person locator places the visitor.',
      setup: () => ({ oldUrdf: true, follow: true, guest: 'stand' }),
      check: (sim, st) => { const e = fixTfError(sim, 'camera_optical_frame', 'camera_optical_frame'); return { items: [['`camera_optical_frame` is in the TF tree', !!e], ['…at the camera, with z forward, x right and y down', !!e && e.ok], ['`/person_position` is within 10 cm of the visitor', st.locOk >= 10]], note: e && !e.ok ? fixOff(e, 'The frame exists, but it is', 'so the visitor lands in the wrong place') + ' Stop that publisher (Ctrl+C) and try again.' : '' }; },
      hints: ["Read the locator's warning in the robot log, then look at the tree: `ros2 run tf2_ros tf2_echo camera_link camera_optical_frame`", 'From `camera_link` (x forward, y left, z up) to the optical frame (z forward, x right, y down) is a pure rotation, with no offset. Which turn about x, then about z, takes one to the other?', 'Publish it: `ros2 run tf2_ros static_transform_publisher --roll -1.5708 --yaw -1.5708 --frame-id camera_link --child-frame-id camera_optical_frame`'],
      fix: 'ros2 run tf2_ros static_transform_publisher --roll -1.5708 --yaw -1.5708 --frame-id camera_link --child-frame-id camera_optical_frame',
      debrief: "Last year's description had no `camera_optical_frame`, so the locator could not move image points into `base_link`. Cameras get two frames on purpose: `camera_link` follows the robot convention (x forward, y left, z up, REP 103) and the optical frame follows the image (z forward, x right, y down). The fixed rotation between them is a roll of −90° and then a yaw of −90°. Leave it out, or get it wrong, and every 3D point from the camera lands in the wrong place, often in the floor or the ceiling. Keep the optical joint in the URDF, so robot_state_publisher sends it with the rest.",
    },
  ],
};
const fixPubs = (sim, topic) => { const T = sim.bus.topics.get(topic); return T ? T.pubs.size : 0; };
// how far a published frame is from the sensor it should name (the lidar unless told otherwise), both relative to the robot's root
function fixTfError(sim, frame, link = 'laser_link') {
  const M = sim.lookup(sim.kin.root, frame); if (!M) return null;
  const T = sim.kin.T, A = K3.mul(K3.inv(T[sim.kin.root]), T[link]);
  const cm = Math.hypot(M[3] - A[3], M[7] - A[7], M[11] - A[11]) * 100;
  let tr = 0; [0, 1, 2, 4, 5, 6, 8, 9, 10].forEach(i => { tr += M[i] * A[i]; });   // trace(Rm^T Ra): the angle between the two orientations
  const deg = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI;
  return { cm, deg, ok: cm <= 1.5 && deg <= 3 };
}
const fixOff = (e, lead, so = 'so the map smears') => `${lead} ${e.cm.toFixed(1)} cm${e.deg > 3 ? ` and ${e.deg.toFixed(0)}°` : ''} away from the real mount, ${so}.`;
const fixLevel = (n) => `<span class="fix-lvl" role="img" aria-label="Difficulty ${n} of 3">${[1, 2, 3].map(i => `<i class="${i <= n ? 'on' : ''}"></i>`).join('')}</span>`;
const fixTime = (s) => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, '0')}`;
const SnoFix = {
  list(sim) { return FIX_TICKETS[sim.id] || []; },
  store() { return (S.sno.fix = S.sno.fix || {}); },
  rec(sim, id) { return this.store()[sim.id + ':' + id]; },
  left(sim) { return this.list(sim).filter(t => !this.rec(sim, t.id)).length; },
  ticket(sim) { return sim.fix ? this.list(sim).find(t => t.id === sim.fix.id) : null; },
  stopAll(sim) {
    (sim.tp ? sim.tp.terms : []).forEach(T => { if (T.proc) T.stop(true); });
    if (typeof Py !== 'undefined' && Py.host === sim.pyHost && Py.phase === 'running') Py.stop();
  },
  start(sim, id) {
    const tk = this.list(sim).find(t => t.id === id); if (!tk) return;
    this.stopAll(sim);
    sim.fix = null; sim.logs = []; sim.reset();
    sim.onLog = () => this.logged(sim);
    sim.restoreRobot(tk.setup(sim));
    sim.fix = { id, t0: sim.t, hints: 0, shown: false, solved: false, stats: Object.assign({ slam: 0, joy: 0, vmax: 0 }, sim.fixStats0 ? sim.fixStats0() : {}), res: null, sig: '' };
    const T = sim.tp && sim.tp.term(); if (T) T.print(`\n── Ticket: ${tk.title} ──\n${sim.def.name} was restarted. Its launch output is in the Fix it tab (and on /rosout).\nFind the fault and fix it with ros2 commands.`, 'i');
    Sound.play('tap'); SnoUI.renderMissionBar(); SnoUI.focusTab('fix'); this.paintLog(sim);
    sim.event('fix_start', id);
  },
  quit(sim) {
    this.stopAll(sim);
    sim.fix = null; sim.onLog = null; sim.restoreRobot({});
    const T = sim.tp && sim.tp.term(); if (T) T.print(`\n${sim.def.name} was restarted normally.`, 'd');
    SnoUI.renderMissionBar(); SnoUI.renderTabs(); SnoUI.refreshTab('fix');
  },
  tick(sim, dt) {
    const F = sim.fix, tk = this.ticket(sim); if (!F || !tk || F.solved) return;
    const st = F.stats, pos = sim.fixPos ? sim.fixPos() : [sim.x, sim.y], prev = F.prev || pos, d = Math.hypot(pos[0] - prev[0], pos[1] - prev[1]); F.prev = pos;
    // joystick distance counts only when the joystick's own commands reach the base (directly or through a relay)
    const tl = sim._tele, joy = tl && sim.t - tl.last < 0.35 && (tl.vx || tl.vy || tl.wz), src = sim.cmd.src || '';
    if (joy && sim.cmdLive(0.35) && (src === '/teleop_panel' || src.startsWith('/relay')) && d < 0.5 && (!sim.fixWalking || sim.fixWalking())) st.joy += d;
    if (sim.cmdLive(0.35)) st.vmax = Math.max(st.vmax, Math.abs(sim.fixVx ? sim.fixVx() : sim.vx || 0));
    if (sim.fixTick) sim.fixTick(st, dt, d);
    F.acc = (F.acc || 0) + dt; if (F.acc < 0.25) return; F.acc = 0;
    const res = tk.check(sim, st); res.ok = res.items.every(x => x[1]);
    const sig = JSON.stringify(res); if (sig !== F.sig) { F.sig = sig; F.res = res; this.paint(sim); }
    if (res.ok) this.complete(sim);
  },
  hint(sim) { const F = sim.fix, tk = this.ticket(sim); if (!F || F.solved || F.hints >= tk.hints.length) return; F.hints++; sim.event('fix_hint', F.hints); this.paint(sim, true); },
  showFix(sim) { const F = sim.fix; if (!F || F.solved) return; F.shown = true; F.hints = this.ticket(sim).hints.length; this.paint(sim, true); },
  xp(tk, F) { return F.shown ? Math.round(tk.xp / 4) : Math.max(Math.round(tk.xp / 2), tk.xp - 10 * F.hints); },
  complete(sim) {
    const F = sim.fix, tk = this.ticket(sim); if (!F || F.solved) return;
    F.solved = true; F.secs = Math.round(sim.t - F.t0);
    const store = this.store(), k = sim.id + ':' + tk.id, old = store[k], xp = old ? 0 : this.xp(tk, F);
    store[k] = { t: old ? Math.min(old.t, F.secs) : F.secs, hints: old ? Math.min(old.hints, F.hints) : F.hints, shown: old ? old.shown && F.shown : F.shown, at: Date.now() };
    let nd = false; if (xp) { addXP(xp); S.gears += 5; nd = bumpStreak(); }
    save(); checkBadges(); save();
    Sound.play('done'); confetti(90);
    toast(`<b>Fixed: ${esc(tk.title)}</b><br>${xp ? `+${xp} XP · +5 gears` : 'Fixed again'} · ${fixTime(F.secs)}${nd ? ` · ${S.streak}-day streak` : ''}`, { mood: 'happy', ms: 3600 });
    App.renderStats(); this.paint(sim, true); SnoUI.renderTabs();
    sim.event('fix_done', tk.id);
  },
  /* ---------- painting ---------- */
  logged(sim) { if (this._lt) return; this._lt = setTimeout(() => { this._lt = 0; this.paintLog(sim); }, 120); },
  logHtml(sim, n) {
    const lines = (sim.logs || []).slice(-n);
    return lines.length ? lines.map(([lv, l]) => `<div class="${lv === 'ERROR' || lv === 'FATAL' ? 'e' : lv === 'WARN' ? 'w' : ''}">${esc(l)}</div>`).join('') : '<div class="d">(no log lines yet)</div>';
  },
  lastWarn(sim) { const l = (sim.logs || []).slice().reverse().find(x => x[0] !== 'INFO'); return l ? l[1] : ''; },
  paintLog(sim) {
    if (SnoUI.cur !== sim || !SnoUI.root) return;
    $$('.fix-logbox', SnoUI.root).forEach(b => { const stick = b.scrollHeight - b.scrollTop - b.clientHeight < 30; b.innerHTML = this.logHtml(sim, +b.dataset.n || 14); if (stick || !b.dataset.seen) b.scrollTop = b.scrollHeight; b.dataset.seen = '1'; });
    const w = $('.mission-bar .fix-last', SnoUI.root); if (w) { const t = this.lastWarn(sim); w.textContent = t; w.hidden = !t; }
  },
  // full: the hint list or the solved state changed, so redraw the whole tab; otherwise only the checks move
  paint(sim, full) {
    if (SnoUI.cur !== sim || !SnoUI.root) return;
    if (full) SnoUI.renderMissionBar();
    const s = $('.fix-strip', SnoUI.root); if (s) this.paintStrip(sim, s);
    if (sim.tab !== 'fix') return;
    const box = $('.fix-checks', SnoUI.root);
    if (full || !box) { SnoUI.renderTab(); return; }
    const res = sim.fix && sim.fix.res; if (!res) return;
    box.innerHTML = this.checksHtml(res);
    const nt = $('.fix-note', SnoUI.root); if (nt) { nt.hidden = !res.note; nt.innerHTML = res.note ? md(res.note) : ''; }
  },
  checksHtml(res) { return res.items.map(([l, ok]) => `<li class="${ok ? 'ok' : ''}">${icon(ok ? 'check' : 'target')}<span>${md(l)}</span><span class="sr">${ok ? ' (done)' : ' (not yet)'}</span></li>`).join(''); },
  // a command from a hint goes into an idle terminal, ready to run with Enter
  toTerm(sim, cmd) {
    const tp = sim.tp; if (!tp) return;
    let i = tp.terms.findIndex(t => !t.proc);
    if (i < 0 && tp.terms.length < 3) { const n = Math.max(...tp.terms.map(t => t.n)) + 1, T = new Term(n, tp.ctxFn()); T.print('New terminal. ROS 2 Jazzy is sourced.', 'd'); tp.terms.push(T); i = tp.terms.length - 1; }
    if (i < 0) { toast('Every terminal is busy: press Ctrl+C in one of them first.', { mood: 'think' }); return; }
    tp.ti = i; tp.terms[i].draft = cmd; SnoUI.focusTab('term');
  },
  cmdify(sim, root) {
    $$('.fix-hints code, .fix-sol code', root).forEach(c => {
      const t = c.textContent.trim(); if (!/^ros2 /.test(t)) return;
      c.classList.add('fix-cmd'); c.tabIndex = 0; c.setAttribute('role', 'button'); c.title = 'Put this command in a terminal';
      const go = () => this.toTerm(sim, t);
      c.addEventListener('click', go); c.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); go(); } });
    });
  },
  /* ---------- the mission bar while a ticket is open ---------- */
  bar(sim, bar) {
    const F = sim.fix, tk = this.ticket(sim);
    if (!tk) { sim.fix = null; return SnoUI.renderMissionBar(); }
    if (F.solved) {
      bar.innerHTML = `${icon('check')}<div class="grow"><div class="mt">Fixed: ${esc(tk.title)} <span class="chip" style="margin-left:6px">${fixTime(F.secs)}</span></div><div class="mh">Read what was wrong in the Fix it tab, or take the next ticket.</div></div>`;
      const nx = this.next(sim, tk.id), b = h('button', { class: 'btn btn-sm btn-primary' }, nx ? 'Next ticket' : 'All tickets');
      b.addEventListener('click', () => { if (nx) this.start(sim, nx.id); else this.quit(sim); }); bar.append(b);
      return;
    }
    bar.innerHTML = `${icon('bug')}<div class="grow"><div class="mt">Ticket: ${esc(tk.title)} ${fixLevel(tk.level)}</div><div class="mh">Fixed when: ${md(tk.goal)}</div><div class="fix-last" aria-live="off" title="Latest warning or error in the robot log" hidden></div></div>`;
    const b = h('button', { class: 'btn btn-sm' }, 'Ticket'); b.addEventListener('click', () => SnoUI.focusTab('fix')); bar.append(b);
    this.paintLog(sim);
  },
  next(sim, id) { const L = this.list(sim), i = L.findIndex(t => t.id === id); return L.slice(i + 1).find(t => !this.rec(sim, t.id)) || L.find(t => !this.rec(sim, t.id) && t.id !== id) || null; },
  /* ---------- the Fix it tab ---------- */
  panel(sim, body) {
    const F = sim.fix, tk = this.ticket(sim), w = h('div', { class: 'lpad fix' }); body.append(w);
    if (!tk) {
      const L = this.list(sim), done = L.length - this.left(sim);
      w.innerHTML = `<div class="fix-intro"><h3>${icon('bug')}Broken-robot missions</h3><p>Each ticket restarts ${esc(sim.def.name)} with a real fault in its ROS graph, the kind that eats an afternoon on a real robot. Read the bug report, find the cause and fix it from the <b>Terminal</b> with <code>ros2</code> commands. You get up to three terminals, so one can run a node while you keep digging in another.</p><p class="sub">${done} of ${L.length} fixed · hints cost 10 XP each</p></div><ul class="fix-list"></ul>`;
      const list = $('.fix-list', w);
      L.forEach(t => {
        const r = this.rec(sim, t.id), q = plain(t.report), b = h('button', { class: 'fix-card' + (r ? ' done' : '') });
        b.innerHTML = `<span class="fix-card-top">${fixLevel(t.level)}<span class="fix-skills">${t.skills.map(x => `<span class="chip">${esc(x)}</span>`).join('')}</span><span class="xp">${r ? `${icon('check')}Fixed in ${fixTime(r.t)}` : `+${t.xp} XP`}</span></span><b>${esc(t.title)}</b><span class="fix-q">${esc(q.length > 118 ? q.slice(0, 115).replace(/\s+\S*$/, '') + '…' : q)}</span>`;
        b.addEventListener('click', () => this.start(sim, t.id)); list.append(h('li', null, b));
      });
      return;
    }
    const res = F.res || Object.assign({ note: '' }, tk.check(sim, F.stats)), i = this.list(sim).indexOf(tk), old = this.rec(sim, tk.id);
    w.innerHTML = `<div class="fix-head"><div class="grow"><div class="fix-kick">Ticket ${i + 1} of ${this.list(sim).length} ${fixLevel(tk.level)}${tk.skills.map(x => `<span class="chip">${esc(x)}</span>`).join('')}</div><h3>${esc(tk.title)}</h3></div><div class="fix-acts"><button class="btn btn-sm btn-ghost" data-a="restart" title="Start this ticket again from the broken robot">${icon('replay')}Restart</button><button class="btn btn-sm btn-ghost" data-a="quit" title="Restart ${esc(sim.def.name)} normally">${icon('x')}Close ticket</button></div></div>
      <blockquote class="fix-report"><p>${md(tk.report)}</p><cite>${esc(tk.from)}, on the team chat</cite></blockquote>
      ${F.solved ? `<div class="fix-done"><div class="fix-done-h">${icon('check')}<b>Fixed in ${fixTime(F.secs)}</b>${F.hints ? `<span>${F.shown ? 'with the fix shown' : `with ${F.hints} hint${F.hints > 1 ? 's' : ''}`}</span>` : '<span>no hints</span>'}</div><h4>What was wrong</h4><p>${md(tk.debrief)}</p><div class="fix-acts"></div></div>` : `<div class="fix-go"><button class="btn btn-sm btn-primary" data-a="term">${icon('terminal')}Open the terminal</button><span class="sub">${old ? `You fixed this before in ${fixTime(old.t)}.` : `Worth ${this.xp(tk, F)} XP`}</span></div>`}
      <div class="fix-goal"><h4>Fixed when</h4><p>${md(tk.goal)}</p><ul class="fix-checks">${this.checksHtml(res)}</ul><p class="fix-note"${res.note ? '' : ' hidden'}>${res.note ? md(res.note) : ''}</p></div>
      <div class="fix-log"><h4>Robot log <span>launch output, newest last</span></h4><div class="fix-logbox" data-n="40" role="log" aria-label="Robot log" tabindex="0"></div></div>
      ${F.solved ? '' : `<div class="fix-hints"><h4>Stuck?</h4>${F.hints ? `<ol>${tk.hints.slice(0, F.hints).map(x => `<li>${md(x)}</li>`).join('')}</ol>` : '<p class="sub">Try the terminal first. Each hint costs 10 XP.</p>'}${F.shown ? `<p class="fix-sol">The fix: <code>${esc(tk.fix)}</code></p>` : ''}<div class="fix-acts">${F.hints < tk.hints.length ? `<button class="btn btn-sm" data-a="hint">${icon('ask')}Hint ${F.hints + 1} of ${tk.hints.length}</button>` : ''}${F.shown ? '' : '<button class="btn btn-sm btn-ghost" data-a="show">Show the fix</button>'}</div>${F.hints ? '<p class="sub">Click a command to put it in a terminal.</p>' : ''}</div>`}`;
    if (F.solved) {
      const acts = $('.fix-done .fix-acts', w), nx = this.next(sim, tk.id);
      if (nx) { const b = h('button', { class: 'btn btn-sm btn-primary' }, `Next: ${nx.title}`); b.addEventListener('click', () => this.start(sim, nx.id)); acts.append(b); }
      const all = h('button', { class: 'btn btn-sm btn-ghost' }, 'All tickets'); all.addEventListener('click', () => this.quit(sim)); acts.append(all);
    }
    w.addEventListener('click', (e) => {
      const a = e.target.closest('[data-a]'); if (!a) return; const k = a.dataset.a;
      if (k === 'hint') this.hint(sim); else if (k === 'show') this.showFix(sim); else if (k === 'restart') this.start(sim, tk.id); else if (k === 'quit') this.quit(sim); else if (k === 'term') SnoUI.focusTab('term');
    });
    this.cmdify(sim, w); this.paintLog(sim);
  },
  // above the terminal while a ticket is open: what you are fixing and the newest robot log lines
  strip(sim, body) {
    const tk = this.ticket(sim); if (!tk) return;
    const s = h('div', { class: 'fix-strip' });
    s.innerHTML = `<div class="fix-strip-h">${icon('bug')}<b></b><span class="fix-strip-st"></span><button class="btn btn-sm btn-ghost">Ticket</button></div><div class="fix-logbox" data-n="3" aria-label="Newest robot log lines"></div>`;
    $('button', s).addEventListener('click', () => SnoUI.focusTab('fix'));
    body.append(s); this.paintStrip(sim, s); this.paintLog(sim);
  },
  paintStrip(sim, s) {
    const F = sim.fix, tk = this.ticket(sim); if (!F || !tk) return;
    const res = F.res, n = res ? res.items.filter(x => x[1]).length : 0, m = res ? res.items.length : 0;
    $('b', s).textContent = (F.solved ? 'Fixed: ' : 'Ticket: ') + tk.title;
    $('.fix-strip-st', s).textContent = F.solved ? 'done' : m ? `${n} of ${m} checks` : '';
    s.classList.toggle('ok', !!F.solved);
  },
};
