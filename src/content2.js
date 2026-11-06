/* ============================== course content: units 7-12 ============================== */
Object.assign(CODE, {
  ifaceCmake: `find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/SetSpeed.srv"
)`,
  ifaceXml: `<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>`,
  paramPy: `class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('greeting', 'Hello')
        rate = self.get_parameter('rate_hz').value
        self.greeting = self.get_parameter('greeting').value
        self.timer = self.create_timer(1.0 / rate, self.tick)`,
  paramYaml: `talker:
  ros__parameters:
    rate_hz: 5.0
    greeting: "Hi"`,
  paramCb: `from rcl_interfaces.msg import SetParametersResult

def on_params(self, params):
    for p in params:
        if p.name == 'rate_hz':
            if p.value <= 0.0:
                return SetParametersResult(successful=False)
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / p.value, self.tick)
    return SetParametersResult(successful=True)`,
  launchXml: `<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
  <node pkg="demo_nodes_py" exec="listener" name="listener"/>
</launch>`,
  launchXml2: `<node pkg="my_py_pkg" exec="listener"
      name="ear" namespace="robot1">
  <param name="greeting" value="Hi"/>
  <remap from="chatter" to="/news"/>
</node>`,
  launchPy: `from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rate = LaunchConfiguration('rate_hz')
    return LaunchDescription([
        DeclareLaunchArgument('rate_hz', default_value='2.0'),
        Node(package='my_py_pkg', executable='talker',
             parameters=[{'rate_hz': rate}]),
        Node(package='my_py_pkg', executable='listener',
             remappings=[('chatter', 'news')]),
    ])`,
  bcast: `import math
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster

class TurtleTF(Node):
    def __init__(self):
        super().__init__('turtle_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

    def on_pose(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.rotation.z = math.sin(pose.theta / 2)
        t.transform.rotation.w = math.cos(pose.theta / 2)
        self.br.sendTransform(t)`,
  listen: `from tf2_ros import Buffer, TransformListener, TransformException
import rclpy.time

class Follower(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.1, self.follow)

    def follow(self):
        try:
            t = self.buffer.lookup_transform(
                'turtle2', 'turtle1', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().info(f'Not yet: {e}')
            return
        dx = t.transform.translation.x
        dy = t.transform.translation.y`,
  follow: `dist = math.sqrt(dx ** 2 + dy ** 2)
angle = math.atan2(dy, dx)

cmd = Twist()
cmd.linear.x = 0.8 * dist
cmd.angular.z = 3.0 * angle
self.pub.publish(cmd)`,
  driver: `import serial
from geometry_msgs.msg import Twist

def clamp(x):
    return int(max(-100, min(100, x)))

class SerialDriver(Node):
    def __init__(self):
        super().__init__('serial_driver')
        self.declare_parameter('port', '/dev/ttyACM0')
        port = self.get_parameter('port').value
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        left = clamp(100 * (v - 0.15 * w))
        right = clamp(100 * (v + 0.15 * w))
        self.ser.write(f'{left},{right}\\n'.encode())`,
});

UNITS.push(
/* ---------------------------------------------------------------- UNIT 7 */
{
  id: 'u7', title: 'Custom interfaces', sub: 'Design your own messages and services', color: '#5653D6',
  notes: [
    ['.msg syntax', 'One field per line: type then name.', 'string name\nfloat64 battery_voltage\nbool is_charging\nfloat64[] wheel_speeds'],
    ['Naming', 'Files are CamelCase (RobotStatus.msg); fields are snake_case (battery_voltage).'],
    ['Interface package', 'An ament_cmake package holding msg/ and srv/ folders.', 'ros2 pkg create --build-type ament_cmake my_interfaces'],
    ['Generate', 'List interfaces in CMakeLists.txt.', 'rosidl_generate_interfaces(${PROJECT_NAME} "msg/RobotStatus.msg")'],
    ['package.xml', 'Generator to build, runtime to run, and the interface group.', '<member_of_group>rosidl_interface_packages</member_of_group>'],
    ['Use it', 'Python import, C++ snake_case header.', 'from my_interfaces.msg import RobotStatus\n#include "my_interfaces/msg/robot_status.hpp"'],
  ],
  lessons: [
    {
      id: 'u7l1', title: 'Design your own message', icon: 'pkg',
      story: [
        { a: 'cards', layout: 'grid2', h: 'Plenty of types ship with ROS', t: 'Common data already has standard types. Use them when you can: tools like rviz2 understand them out of the box.',
          items: [{ icon: 'msg', title: 'std_msgs', text: 'String, Bool, Int32, Float64…', k: 'var(--c-msg)' }, { icon: 'axes', title: 'geometry_msgs', text: 'Twist, Pose, Point, TransformStamped…', k: 'var(--c-srv)' }, { icon: 'camera', title: 'sensor_msgs', text: 'LaserScan, Image, Imu, JointState…', k: 'var(--c-node)' }, { icon: 'map', title: 'nav_msgs', text: 'Odometry, Path, OccupancyGrid…', k: 'var(--c-topic)' }] },
        { a: 'msgGen', h: 'When you need your own', t: 'For your own data, write a `.msg` file with one field per line, written as `type name`. Here is `RobotStatus.msg`.' },
        { a: 'cards', layout: 'grid3', h: 'Field types', t: 'Fields can be primitive types, arrays of any length (`float64[]`), fixed-size arrays (`float64[3]`) or even other messages.',
          items: [{ title: 'bool', text: 'true or false', k: 'var(--c-node)' }, { title: 'int32 / int64', text: 'whole numbers', k: 'var(--c-srv)' }, { title: 'float32 / float64', text: 'decimal numbers', k: 'var(--c-topic)' }, { title: 'string', text: 'text', k: 'var(--c-msg)' }, { title: 'float64[]', text: 'list of any length', k: 'var(--c-param)' }, { title: 'geometry_msgs/Point', text: 'a nested message', k: 'var(--accent)' }] },
        { a: 'msgGen', mode: 'generate', h: 'One file, two languages', t: 'At build time, **rosidl** generates a Python class and a C++ struct from your `.msg`, so both languages share exactly the same data layout.' },
        { a: 'cards', layout: 'grid3', h: 'Naming rules', t: 'Interface **file names** are CamelCase (`RobotStatus.msg`) and **fields** are snake_case. Keep interfaces in their own package so any package can depend on them.',
          items: [{ icon: 'check', title: 'RobotStatus.msg', text: 'CamelCase file name', k: 'var(--good)' }, { icon: 'check', title: 'battery_voltage', text: 'snake_case field', k: 'var(--good)' }, { icon: 'pkg', title: 'my_interfaces', text: 'its own package', k: 'var(--c-topic)' }] },
      ],
      ex: [
        MC('Which file name is valid for a ROS 2 message?', ['`RobotStatus.msg`', '`robot_status.msg`', '`robotStatus.msg`', '`Robot-Status.msg`'], 0),
        BUILD('Write a field for the battery voltage.', ['float64', 'battery_voltage'], ['BatteryVoltage', 'string', '=']),
        MC('How do you declare a list of floats of any length?', ['`float64[] readings`', '`list<float64> readings`', '`float64 readings[]`', '`readings: float64[]`'], 0),
        TF('Custom interfaces are best kept in their own package.', true),
        MATCH('Match each type to an example value.', [['bool', 'true'], ['int32', '-42'], ['float64', '3.14'], ['string', '"hello"']]),
        MC('You need a field holding a position. Best choice?', ['`geometry_msgs/Point position`', '`string position`', '`float64 position`', '`Point.msg position`'], 0),
      ],
    },
    {
      id: 'u7l2', title: 'Build & use interfaces', icon: 'tools',
      story: [
        { a: 'tree', h: 'An interface package', t: 'Create it with `ros2 pkg create --build-type ament_cmake my_interfaces`, then add `msg/` and `srv/` folders. Interfaces are generated by CMake, so use `ament_cmake` even if your nodes are Python.',
          items: [{ d: 0, n: 'my_interfaces/', dir: true }, { d: 1, n: 'CMakeLists.txt', hl: true }, { d: 1, n: 'package.xml', hl: true }, { d: 1, n: 'msg/', dir: true }, { d: 2, n: 'RobotStatus.msg' }, { d: 1, n: 'srv/', dir: true }, { d: 2, n: 'SetSpeed.srv' }] },
        { a: 'code', lang: 'cmake', code: CODE.ifaceCmake, focus: [3, 6], h: 'Tell CMake to generate', t: '`rosidl_generate_interfaces` lists every interface file to generate.' },
        { a: 'code', lang: 'xml', code: CODE.ifaceXml, h: 'Three lines in package.xml', t: 'Declare the generator for building, the runtime for running, and mark the package as an interface package.',
          steps: [{ lines: [1, 1], note: 'needed to build the interfaces' }, { lines: [2, 2], note: 'needed to use them at runtime' }, { lines: [3, 3], note: 'marks this as an interface package' }] },
        { a: 'cards', layout: 'row', h: 'Using it from code', t: 'Python imports the class by name. C++ includes a **snake_case** header. Add `<depend>my_interfaces</depend>` to every package that uses it.',
          items: [{ icon: 'pkg', title: 'RobotStatus.msg', text: 'in my_interfaces', k: 'var(--c-topic)' }, { icon: 'python', title: 'Python', code: 'from my_interfaces.msg import RobotStatus', k: 'var(--c-msg)' }, { icon: 'chip', title: 'C++', code: '#include "my_interfaces/msg/robot_status.hpp"\nmy_interfaces::msg::RobotStatus', k: 'var(--c-srv)' }] },
        { a: 'terminal', h: 'Check your work', t: 'After building and sourcing, `ros2 interface show` prints your new type: proof that generation worked.',
          panes: [{ title: 'Terminal', lines: [{ c: 'colcon build --packages-select my_interfaces' }, { o: 'Starting >>> my_interfaces\nFinished <<< my_interfaces [4.81s]\n\nSummary: 1 package finished [5.02s]' }, { c: 'source install/setup.bash' }, { c: 'ros2 interface show my_interfaces/msg/RobotStatus' }, { o: 'string name\nfloat64 battery_voltage\nbool is_charging\nfloat64[] wheel_speeds' }] }] },
      ],
      ex: [
        FILL('Generate the message.', '___(${PROJECT_NAME}\n  "msg/RobotStatus.msg"\n)', ['rosidl_generate_interfaces'], ['add_executable', 'ament_package', 'find_package'], null, 'cmake'),
        MC('Which C++ header provides my_interfaces/msg/RobotStatus?', ['`my_interfaces/msg/robot_status.hpp`', '`my_interfaces/msg/RobotStatus.hpp`', '`my_interfaces/RobotStatus.h`', '`robot_status/msg.hpp`'], 0),
        MC('How do you import it in Python?', ['`from my_interfaces.msg import RobotStatus`', '`import my_interfaces.RobotStatus`', '`from my_interfaces import robot_status`', '`#include my_interfaces`'], 0),
        ORDER('Order the steps.', ['Create an ament_cmake package', 'Write msg/RobotStatus.msg', 'Edit CMakeLists.txt and package.xml', 'colcon build', 'source install/setup.bash', 'ros2 interface show my_interfaces/msg/RobotStatus']),
        TF('You can generate custom interfaces directly inside an `ament_python` package.', false, 'Generation runs through CMake, so interfaces live in an ament_cmake package.'),
        MATCH('Match each package.xml line to its job.', [['rosidl_default_generators', 'Builds the interfaces'], ['rosidl_default_runtime', 'Uses them at runtime'], ['rosidl_interface_packages', 'Marks an interface package'], ['<depend>my_interfaces</depend>', 'Lets another package use them']]),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 8 */
{
  id: 'u8', title: 'Parameters', sub: 'Configure nodes without editing code', color: '#EE5A24',
  lab: { tab: 'term', text: 'Change the robot\'s speed limit with ros2 param set' },
  notes: [
    ['Parameter', 'A named setting owned by one node: bool, int, double, string or arrays.'],
    ['Declare and read', 'The default value also sets the type.', "self.declare_parameter('rate_hz', 2.0)\nrate = self.get_parameter('rate_hz').value"],
    ['Set at startup', 'Override with --ros-args.', 'ros2 run my_py_pkg talker --ros-args -p rate_hz:=5.0'],
    ['YAML file', 'Node name, then ros__parameters (two underscores).', 'talker:\n  ros__parameters:\n    rate_hz: 5.0'],
    ['Runtime CLI', 'List, read, change and save parameters.', 'ros2 param list\nros2 param get /talker rate_hz\nros2 param set /talker rate_hz 10.0\nros2 param dump /talker'],
    ['Validate changes', 'Reject bad values in a callback.', 'self.add_on_set_parameters_callback(self.on_params)'],
  ],
  lessons: [
    {
      id: 'u8l1', title: 'Configurable nodes', icon: 'knob',
      story: [
        { a: 'knobs', mode: 'intro', h: 'Parameters are a node\'s settings', t: '**Parameters** are named settings that belong to a node, like knobs on a device: a publish rate, a greeting, a speed limit. You change them without editing code.' },
        { a: 'code', lang: 'py', code: CODE.paramPy, focus: [4, 5], h: 'Declare with a default', t: '`declare_parameter(name, default)` creates a parameter. The default also sets its **type**: `2.0` makes it a double.' },
        { a: 'code', lang: 'py', code: CODE.paramPy, focus: [6, 8], h: 'Read the value', t: '`get_parameter(name).value` returns the current value. Here it sets the timer period: `1.0 / rate`.' },
        { a: 'terminal', h: 'Set it at startup', t: 'Override parameters when you start a node with `--ros-args -p name:=value`.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 run my_py_pkg talker --ros-args -p rate_hz:=5.0 -p greeting:=Hi' }, { stream: ['[INFO] [{t}] [talker]: Hi {n}'], every: 260 }] }] },
        { a: 'code', lang: 'yaml', code: CODE.paramYaml, h: 'Or load a YAML file', t: 'Keep settings in a YAML file: the node name, then `ros__parameters` (two underscores!), then the values. Load it with `--ros-args --params-file talker.yaml`.', focus: [2, 2], note: 'two underscores in ros__parameters' },
      ],
      ex: [
        FILL('Start talker at 5 Hz.', 'ros2 run my_py_pkg talker ___ -p rate_hz___5.0', ['--ros-args', ':='], ['--params', '=', '-r'], null, 'sh'),
        MC('Which key holds the parameters in a YAML file?', ['`ros__parameters`', '`ros_parameters`', '`parameters`', '`params`'], 0),
        BUG('This parameter file is silently ignored. Tap the bug.', `talker:
  ros_parameters:
    rate_hz: 5.0`, 1, 'It needs two underscores: `ros__parameters`.', 'yaml'),
        TF('Parameters are global: every node shares the same values.', false, 'Each node owns its own parameters. ROS 1 had a global parameter server.'),
        MC('`declare_parameter(\'rate_hz\', 2.0)` makes the parameter a…', ['double', 'string', 'integer', 'boolean'], 0),
        MC('What happens if you `get_parameter()` a name you never declared?', ['An exception: declare it first', 'It returns 0', 'It returns None silently', 'ROS declares it for you'], 0),
      ],
    },
    {
      id: 'u8l2', title: 'Change parameters live', icon: 'sliders',
      story: [
        { a: 'terminal', h: 'Inspect at runtime', t: '`ros2 param list` shows every node\'s parameters. `use_sim_time` comes for free with every node. `ros2 param get` reads a single value.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 param list' }, { o: '/talker:\n  greeting\n  rate_hz\n  start_type_description_service\n  use_sim_time' }, { c: 'ros2 param get /talker rate_hz' }, { o: 'Double value is: 2.0' }] }] },
        { a: 'knobs', mode: 'live', h: 'Turn a knob while it runs', t: '`ros2 param set /talker rate_hz 10.0` changes the value inside a running node. Drag the slider and watch the command and the message rate.' },
        { a: 'knobs', mode: 'gate', h: 'Validate changes in a callback', t: 'Register `add_on_set_parameters_callback(cb)`. Returning `SetParametersResult(successful=False)` rejects a bad value, like a negative rate. Try dragging below zero.' },
        { a: 'code', lang: 'py', code: CODE.paramCb, focus: [8, 9], h: 'React to new values', t: 'Reading a parameter once at startup is not enough: if the rate changes, the node must recreate its timer inside the callback.' },
        { a: 'terminal', h: 'Save your settings', t: '`ros2 param dump /talker` prints every value as YAML: a ready-made `--params-file` for next time.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 param dump /talker' }, { o: '/talker:\n  ros__parameters:\n    greeting: Hello\n    rate_hz: 10.0\n    start_type_description_service: true\n    use_sim_time: false' }] }] },
      ],
      ex: [
        TYPE('Change /talker\'s rate_hz to 10.0.', 'ros2 param set /talker rate_hz 10.0', 'Pattern: ros2 param set <node> <name> <value>.', { out: 'Set parameter successful' }),
        MATCH('Match each command to its job.', [['ros2 param list', 'Show parameter names'], ['ros2 param get', 'Read one value'], ['ros2 param set', 'Change a value'], ['ros2 param dump', 'Print all as YAML']]),
        MC('Your callback returns `SetParametersResult(successful=False)`. What happens?', ['The change is rejected and the old value stays', 'The node crashes', 'The value changes anyway', 'ROS retries the change'], 0),
        MC('You change rate_hz at runtime, but the publish rate stays the same. Why?', ['The timer was created once; the node must recreate it when the value changes', 'Parameters can\'t change at runtime', 'A YAML file overrides it', 'rate_hz is read-only'], 0),
        FILL('C++: read rate_hz as a double.', 'double rate = get_parameter("rate_hz").___();', ['as_double'], ['value', 'to_double', 'get'], null, 'cpp'),
        TF('`use_sim_time` is a parameter that every node gets automatically.', true),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 9 */
{
  id: 'u9', title: 'Launch files', sub: 'Start a whole system with one command', color: '#0D93B0',
  lab: { tab: 'term', text: 'Launch an autonomous wander node' },
  notes: [
    ['ros2 launch', 'Start everything described in a launch file.', 'ros2 launch my_pkg talk.launch.xml'],
    ['XML node', 'Package, executable and optional name.', '<node pkg="demo_nodes_cpp" exec="talker" name="talker"/>'],
    ['Namespace', 'Prefix a node and its relative topics: chatter → /robot1/chatter.', '<node pkg="my_pkg" exec="talker" namespace="robot1"/>'],
    ['Remap', 'Rename a topic without touching code.', '<remap from="chatter" to="/news"/>'],
    ['Python launch', 'Must define generate_launch_description().', 'def generate_launch_description():\n    return LaunchDescription([...])'],
    ['Launch argument', 'Declare, then pass with name:=value.', 'ros2 launch my_py_pkg talk.launch.py rate_hz:=5.0'],
  ],
  lessons: [
    {
      id: 'u9l1', title: 'One command, many nodes', icon: 'launch',
      story: [
        { a: 'launch', mode: 'collapse', h: 'Too many terminals', t: 'A real robot runs dozens of nodes. Starting each one in its own terminal is painful. A **launch file** starts them all, with their settings, in one command.' },
        { a: 'code', lang: 'xml', code: CODE.launchXml, focus: [2, 3], h: 'An XML launch file', t: 'Each `<node>` names a package (`pkg`), an executable (`exec`) and optionally a `name`. Run it with `ros2 launch my_pkg talk.launch.xml`.' },
        { a: 'code', lang: 'xml', code: CODE.launchXml2, h: 'Names, namespaces, params, remaps', t: 'Set parameters with `<param>`, rename topics with `<remap>`, and group nodes with `namespace`.',
          steps: [{ lines: [1, 2], note: 'a unique name and a namespace' }, { lines: [3, 3], note: 'set a parameter at launch' }, { lines: [4, 4], note: 'reroute chatter to /news' }] },
        { a: 'launch', mode: 'ns', h: 'Namespaces keep robots apart', t: 'With `namespace="robot1"`, the node becomes `/robot1/talker` and its relative topic `chatter` becomes `/robot1/chatter`. Two robots, no collisions.' },
        { a: 'launch', mode: 'remap', h: 'Remapping reroutes topics', t: '`<remap from="chatter" to="/news"/>` makes a node use `/news` instead of `chatter`, without changing a line of its code.' },
      ],
      ex: [
        FILL('Start the demo talker.', '<node ___="demo_nodes_cpp" ___="talker"/>', ['pkg', 'exec'], ['package', 'executable', 'node'], null, 'xml'),
        MC('With `namespace="robot1"`, the topic `chatter` becomes…', ['`/robot1/chatter`', '`/chatter/robot1`', '`robot1_chatter`', '`/chatter`'], 0),
        MC('A node publishes `chatter`, but you need it on `/news`. Which tag?', ['`<remap from="chatter" to="/news"/>`', '`<param name="chatter" value="/news"/>`', '`<topic rename="/news"/>`', '`<node name="/news"/>`'], 0),
        TF('Topic names that start with `/` are absolute: a namespace does not change them.', true),
        MC('Two listeners in one launch file are both named `listener`. How do you fix it?', ['Give them different names or namespaces', 'Remove one: only one listener is allowed', 'Rename the package', 'Add a remap'], 0),
        TYPE('Run talk.launch.xml from the package my_pkg.', 'ros2 launch my_pkg talk.launch.xml', 'Pattern: ros2 launch <package> <file>.'),
      ],
    },
    {
      id: 'u9l2', title: 'Python launch & arguments', icon: 'python',
      story: [
        { a: 'code', lang: 'py', code: CODE.launchPy, focus: [6, 8], h: 'Python launch files', t: 'A Python launch file defines `generate_launch_description()`, which returns a `LaunchDescription`: a list of things to start and do.' },
        { a: 'code', lang: 'py', code: CODE.launchPy, focus: [[7, 7], [9, 11]], h: 'Launch arguments', t: '`DeclareLaunchArgument` creates an argument with a default. `LaunchConfiguration(\'rate_hz\')` is a **substitution**: its value is filled in at launch time.' },
        { a: 'terminal', h: 'Pass arguments', t: 'Pass values as `name:=value`. `--show-args` lists the arguments a launch file accepts.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 launch my_py_pkg talk.launch.py --show-args' }, { o: "Arguments (pass arguments as '<name>:=<value>'):\n\n    'rate_hz':\n        no description given\n        (default: '2.0')" }, { c: 'ros2 launch my_py_pkg talk.launch.py rate_hz:=5.0' }, { o: '[INFO] [launch]: Default logging verbosity is set to INFO\n[INFO] [talker-1]: process started with pid [4120]\n[INFO] [listener-2]: process started with pid [4122]' }] }] },
        { a: 'compare', left: 'XML', right: 'Python', h: 'XML or Python?', t: 'Both describe the same things. XML is shorter to read; Python lets you add logic such as loops and conditions.',
          rows: [['<node pkg="p" exec="e"/>', 'Node(package="p", executable="e")'], ['<arg name="rate_hz" default="2.0"/>', 'DeclareLaunchArgument("rate_hz", default_value="2.0")'], ['$(var rate_hz)', 'LaunchConfiguration("rate_hz")'], ['<include file="..."/>', 'IncludeLaunchDescription(...)']] },
        { a: 'cards', layout: 'grid2', h: 'Install your launch files', t: '`ros2 launch` looks in the package\'s **share** folder, so install the `launch/` directory and rebuild.',
          items: [{ icon: 'python', title: 'ament_python: setup.py', code: "(os.path.join('share', package_name, 'launch'),\n glob('launch/*'))", k: 'var(--c-msg)' }, { icon: 'chip', title: 'ament_cmake: CMakeLists.txt', code: 'install(DIRECTORY launch\n  DESTINATION share/${PROJECT_NAME})', k: 'var(--c-srv)' }] },
      ],
      ex: [
        FILL('Name the required function.', 'def ___():\n    return LaunchDescription([...])', ['generate_launch_description'], ['main', 'launch', 'generate_launch']),
        MC('How do you pass rate_hz = 5.0?', ['`ros2 launch my_py_pkg talk.launch.py rate_hz:=5.0`', '`ros2 launch my_py_pkg talk.launch.py -p rate_hz=5.0`', '`ros2 launch rate_hz=5.0`', 'Edit the file every time'], 0),
        MATCH('Match XML to Python.', [['<node>', 'Node(...)'], ['<arg>', 'DeclareLaunchArgument(...)'], ['$(var x)', "LaunchConfiguration('x')"], ['<include>', 'IncludeLaunchDescription(...)']]),
        TF('`ros2 launch` finds launch files in the package\'s share folder once they are installed.', true),
        MC('What is `LaunchConfiguration(\'rate_hz\')`?', ['A substitution filled in with the argument value at launch time', 'A Python float', 'A new node', 'A parameter file'], 0),
        BUG('ros2 launch can\'t find the launch description. Tap the bug.', `from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch():
    return LaunchDescription([
        Node(package='my_py_pkg', executable='talker'),
    ])`, 3, 'The function must be named `generate_launch_description`.'),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 10 */
{
  id: 'u10', title: 'TF2: frames & broadcasters', sub: 'Teach your robot where things are', color: '#B144C8',
  lab: { tab: 'term', text: 'Run tf2_echo on the lab robot' },
  notes: [
    ['Frame', 'A coordinate system: an origin plus x (red), y (green), z (blue) axes. On robots x is forward, y left, z up.'],
    ['Transform', 'Translation (x, y, z) plus rotation, usually a quaternion (x, y, z, w).'],
    ['TF tree', 'Every frame has exactly one parent. TF2 chains transforms to relate any two frames.'],
    ['Yaw → quaternion', 'For a 2D heading θ: z = sin(θ/2), w = cos(θ/2).', 't.transform.rotation.z = math.sin(theta / 2)\nt.transform.rotation.w = math.cos(theta / 2)'],
    ['Broadcasters', 'TransformBroadcaster → /tf for moving parts. StaticTransformBroadcaster → /tf_static for fixed parts.'],
    ['TF tools', 'Print a live transform or draw the tree.', 'ros2 run tf2_ros tf2_echo world turtle1\nros2 run tf2_tools view_frames'],
  ],
  lessons: [
    {
      id: 'u10l1', title: 'Frames everywhere', icon: 'axes',
      story: [
        { a: 'frames', mode: 'intro', h: 'Where is the ball?', t: 'The camera sees a ball 1.5 m ahead **of the camera**. But where is it relative to the robot\'s wheels, or the room? Every sensor has its own point of view: its own **frame**.' },
        { a: 'frames', mode: 'axes', h: 'A frame is an origin plus axes', t: 'Frames are drawn as three arrows: **x red, y green, z blue** (RGB = XYZ). On a robot, x points forward, y to the left and z up.' },
        { a: 'frames', mode: 'transform', h: 'Transforms connect frames', t: 'A **transform** says how one frame sits relative to another: a **translation** (x, y, z) plus a **rotation**, usually stored as a **quaternion** (x, y, z, w).' },
        { a: 'tfTree', h: 'TF2 keeps a tree', t: '**TF2** tracks every frame in a **tree**. Each frame has exactly one parent, such as `map → odom → base_link → camera_link`. TF2 chains transforms to relate any two frames.' },
        { a: 'frames', mode: 'time', h: 'Transforms change over time', t: 'As the robot drives, `world → base_link` keeps changing. TF2 **time-stamps** and buffers transforms, so you can ask where things were a moment ago.' },
      ],
      ex: [
        MC('On a ROS robot, which axis points forward?', ['x', 'y', 'z', 'w'], 0),
        MATCH('Match the colors.', [['Red arrow', 'x axis'], ['Green arrow', 'y axis'], ['Blue arrow', 'z axis'], ['Quaternion', 'Rotation (x, y, z, w)']]),
        MC('How many parents can a frame have in a TF2 tree?', ['Exactly one (the root has none)', 'Two', 'Any number', 'None'], 0),
        TF('A transform contains a translation and a rotation.', true),
        ORDER('Order the hops TF2 chains to go from the camera up to the map.', ['camera_link', 'base_link', 'odom', 'map']),
        MC('Why do robots store rotations as quaternions?', ['They avoid gimbal lock and combine cleanly', 'They use fewer numbers than angles', 'They are easier to read', 'ROS forbids angles'], 0),
      ],
    },
    {
      id: 'u10l2', title: 'Write a broadcaster', icon: 'tf',
      story: [
        { a: 'graph', h: 'From pose to TF', t: 'turtlesim publishes `/turtle1/pose` (x, y, theta) but no transforms. Our node turns each pose into a transform on `/tf`.',
          nodes: [GN('ts', '/turtlesim', 100, 100), GN('ps', '/turtle1/pose', 310, 100, 'topic'), GN('br', '/tf_broadcaster', 495, 100), GN('tf', '/tf', 495, 250, 'topic'), GN('echo', '/tf2_echo', 260, 250)],
          edges: [GE('ts', 'ps'), GE('ps', 'br'), GE('br', 'tf', 'tf'), GE('tf', 'echo', 'tf')] },
        { a: 'code', lang: 'py', code: CODE.bcast, focus: [9, 10], h: 'Broadcaster + subscription', t: 'Create a `TransformBroadcaster` and subscribe to the pose topic.' },
        { a: 'code', lang: 'py', code: CODE.bcast, focus: [13, 16], h: 'Stamp it and name the frames', t: 'Each `TransformStamped` carries a time stamp, a parent (`frame_id`) and a child (`child_frame_id`).' },
        { a: 'quat', h: 'Heading to quaternion', t: 'For a 2D heading θ the quaternion is **z = sin(θ/2)** and **w = cos(θ/2)**, with x = y = 0. Watch the numbers as the turtle turns.' },
        { a: 'terminal', h: 'Check with the CLI', t: '`ros2 run tf2_ros tf2_echo world turtle1` prints the live transform. `ros2 run tf2_tools view_frames` saves a PDF of the whole tree.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 run tf2_ros tf2_echo world turtle1' }, { stream: ['At time {t}\n- Translation: [5.544, 5.544, 0.000]\n- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.383, 0.924]\n- Rotation: in RPY (degree) [0.000, -0.000, 45.000]'], every: 1000 }] }] },
        { a: 'cards', layout: 'grid2', h: 'Static vs dynamic', t: 'Parts that never move relative to each other, like a camera bolted to the chassis, use a **static** transform published on `/tf_static`. Try `ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.2 --frame-id base_link --child-frame-id camera_link`.',
          items: [{ icon: 'drive', title: 'TransformBroadcaster', text: 'Moving things: the robot in the world, wheels, arm joints. Published on /tf.', k: 'var(--c-param)' }, { icon: 'camera', title: 'StaticTransformBroadcaster', text: 'Bolted-on parts: camera mount, lidar mount. Published on /tf_static.', k: 'var(--c-srv)' }] },
      ],
      ex: [
        FILL('Name the frames.', "t.header.frame_id = '___'\nt.child_frame_id = '___'", ['world', 'turtle1'], ['/tf', 'pose', 'base']),
        MC('Heading θ = 0. What is the quaternion (x, y, z, w)?', ['(0, 0, 0, 1)', '(0, 0, 1, 0)', '(1, 0, 0, 0)', '(0, 0, 0, 0)'], 0, 'z = sin(0) = 0 and w = cos(0) = 1.'),
        MC('A camera is bolted to the chassis. Which broadcaster fits?', ['StaticTransformBroadcaster', 'TransformBroadcaster at 100 Hz', 'No transform is needed', 'A service'], 0),
        TYPE('Print the live transform from world to turtle1.', 'ros2 run tf2_ros tf2_echo world turtle1', 'tf2_echo lives in the tf2_ros package.'),
        TF('`TransformBroadcaster` publishes on the `/tf` topic.', true),
        MC('Heading θ = π (facing the other way). What is w?', ['0', '1', '-1', '0.5'], 0, 'w = cos(π/2) = 0, and z = sin(π/2) = 1.'),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 11 */
{
  id: 'u11', title: 'TF2: listeners & followers', sub: 'Use transforms to chase a turtle', color: '#6E9A12',
  notes: [
    ['Buffer + listener', 'The listener subscribes to /tf and /tf_static and fills the buffer.', 'self.buffer = Buffer()\nself.listener = TransformListener(self.buffer, self)'],
    ['lookup_transform', 'Where is the source frame, seen from the target frame? Time() means latest.', "self.buffer.lookup_transform('turtle2', 'turtle1', rclpy.time.Time())"],
    ['TransformException', 'Thrown when a transform is not available yet. Catch it and retry.'],
    ['Distance and angle', 'From dx and dy in the follower\'s frame.', 'dist = math.sqrt(dx ** 2 + dy ** 2)\nangle = math.atan2(dy, dx)'],
    ['P-controller', 'Command proportional to the error. Too high a gain wobbles; too low lags.', 'cmd.angular.z = 3.0 * angle'],
  ],
  lessons: [
    {
      id: 'u11l1', title: 'Listening to transforms', icon: 'radio',
      story: [
        { a: 'graph', h: 'Buffer + listener', t: 'A **TransformListener** subscribes to `/tf` and `/tf_static` and fills a **Buffer** with recent transforms, about 10 seconds of history by default.',
          nodes: [GN('tf', '/tf', 100, 110, 'topic'), GN('tfs', '/tf_static', 100, 240, 'topic'), GN('li', 'TransformListener', 320, 175, 'box'), GN('bu', 'Buffer (≈10 s)', 510, 175, 'hw')],
          edges: [GE('tf', 'li', 'tf'), GE('tfs', 'li', 'tf'), GE('li', 'bu', 'tf')] },
        { a: 'code', lang: 'py', code: CODE.listen, focus: [7, 8], h: 'Two lines to set it up', t: 'Create the `Buffer`, then a `TransformListener` that feeds it. Keep both as attributes so they stay alive.' },
        { a: 'code', lang: 'py', code: CODE.listen, focus: [12, 14], h: 'Ask a question', t: '`lookup_transform(target, source, time)` answers: where is the **source** frame, seen from the **target** frame? `rclpy.time.Time()` means "the latest available".' },
        { a: 'turtle', mode: 'relative', h: 'Seen from turtle2', t: 'Looking up turtle1 in turtle2\'s frame gives `dx` (ahead of turtle2) and `dy` (to its left), which is exactly what a follower needs.' },
        { a: 'code', lang: 'py', code: CODE.listen, focus: [15, 17], h: 'Transforms may not exist yet', t: 'Right after startup the buffer may be empty. Catch `TransformException` and simply try again on the next timer tick.' },
      ],
      ex: [
        FILL('Set up the listener.', 'self.buffer = ___()\nself.listener = TransformListener(self.buffer, ___)', ['Buffer', 'self'], ['Listener', 'TransformBroadcaster', 'None']),
        MC('What does `rclpy.time.Time()` mean in `lookup_transform`?', ['Use the latest available transform', 'Use exactly the current clock time', 'Wait one second', 'Use the very first transform ever received'], 0),
        MC('`lookup_transform(\'turtle2\', \'turtle1\', ...)` gives you…', ['turtle1\'s position seen from turtle2', 'turtle2\'s position seen from turtle1', 'The distance in pixels', 'The world frame'], 0),
        TF('You should catch `TransformException` because the transform may not be available yet.', true),
        MC('In turtle2\'s frame, turtle1 is at dx = 3, dy = 0. Where is it?', ['3 m straight ahead', '3 m to the left', '3 m behind', 'On top of turtle2'], 0),
        ORDER('Order the listener steps.', ['Create a Buffer', 'Create a TransformListener', 'Start a timer', 'lookup_transform in the callback', 'Use dx and dy']),
      ],
    },
    {
      id: 'u11l2', title: 'Follow the leader', icon: 'target',
      story: [
        { a: 'terminal', h: 'Set the stage', t: 'Spawn a second turtle and run one broadcaster per turtle. A parameter tells each broadcaster which turtle to track.',
          panes: [{ title: 'Terminal', lines: [{ c: "ros2 service call /spawn turtlesim/srv/Spawn \"{x: 2.0, y: 1.0, theta: 0.0, name: 'turtle2'}\"" }, { o: "response:\nturtlesim.srv.Spawn_Response(name='turtle2')" }, { c: 'ros2 run my_py_pkg turtle_tf_broadcaster --ros-args -p turtle:=turtle2' }, { c: 'ros2 run my_py_pkg turtle_follower' }] }] },
        { a: 'turtle', mode: 'relative', h: 'Distance and angle', t: 'From `dx` and `dy`: distance = `sqrt(dx² + dy²)` and angle = `atan2(dy, dx)`. A positive angle means the leader is to the left.' },
        { a: 'code', lang: 'py', code: CODE.follow, h: 'A proportional controller', t: 'Drive faster when far away and turn harder when the angle is large. Speeds **proportional** to the error make a **P-controller**.',
          steps: [{ lines: [1, 2], note: 'how far, and in which direction' }, { lines: [5, 5], note: 'forward speed grows with distance' }, { lines: [6, 6], note: 'turn rate grows with the angle' }] },
        { a: 'follower', h: 'Tune the gains', t: 'Drag the sliders. With too little gain the follower lags behind; with too much it wobbles and overshoots.' },
        { a: 'turtle', mode: 'train', h: 'Challenge: a turtle train', t: 'Chain followers: turtle3 follows turtle2, which follows turtle1. Same code, different parameters. That is the power of TF2.' },
      ],
      ex: [
        FILL('Compute the heading error.', 'angle = math.___(dy, dx)', ['atan2'], ['atan', 'tan', 'sqrt']),
        MC('dx = 0, dy = 2 (the leader is to the left). Which way should the follower turn?', ['Left: positive angular.z', 'Right: negative angular.z', 'No turn needed', 'Drive backwards'], 0),
        MC('The follower overshoots and wobbles. What should you try first?', ['Lower the gains', 'Raise the gains', 'Publish less often', 'Use a static transform'], 0),
        TURTLE('turtle1 is straight ahead of turtle2. Drive turtle2 toward it without turning.', [0.3, 99], [-0.15, 0.15], 'Straight ahead means angle 0: forward speed only.'),
        TF('A positive angle from atan2(dy, dx) means the leader is on the left.', true),
        MC('dx = 3, dy = 4. How far away is the leader?', ['5', '7', '12', '1'], 0, 'sqrt(9 + 16) = 5.'),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 12 */
{
  id: 'u12', title: 'Talking to microcontrollers', sub: 'Bridge ROS 2 to real motors', color: '#D93A3A',
  lab: { tab: 'drive', text: 'Drive the lab robot and watch its cmd_vel' },
  notes: [
    ['Two computers', 'The ROS 2 computer decides; the microcontroller handles real-time I/O like motor PWM and encoders.'],
    ['Serial protocol', 'A newline-terminated text line per command, e.g. left,right in percent.', '60,60\\n'],
    ['Baud rate', 'Both sides must agree, e.g. 115200. Mismatches produce garbage.'],
    ['Serial permissions', 'Let your user open /dev/ttyACM0 without sudo.', 'sudo usermod -a -G dialout $USER'],
    ['pyserial', 'Open the port from Python.', "ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)"],
    ['Differential drive', 'Mix forward speed v and turn rate w into two wheels.', 'left = v - w * b / 2\nright = v + w * b / 2'],
    ['Watchdog', 'Stop the motors if commands stop arriving for ~0.5 s.'],
    ['micro-ROS', 'A tiny ROS 2 client on the microcontroller that joins the graph through an agent on the host.'],
  ],
  lessons: [
    {
      id: 'u12l1', title: 'Brains and reflexes', icon: 'usb',
      story: [
        { a: 'serial', mode: 'arch', h: 'Two computers, two jobs', t: 'ROS 2 runs on a computer such as a laptop or a **single-board computer** (for example a Raspberry Pi running Ubuntu). A **microcontroller** handles the real-time work: motor PWM, encoders and sensors.' },
        { a: 'cards', layout: 'grid3', h: 'Brain and reflexes', t: 'Like your brain and spinal cord: the computer decides **what** to do, and the microcontroller makes it happen **precisely**, thousands of times per second.',
          items: [{ icon: 'brain', title: 'Computer + ROS 2', text: 'Planning, vision, decisions.', k: 'var(--c-node)' }, { icon: 'chip', title: 'Microcontroller', text: 'Fast reflexes and precise timing.', k: 'var(--accent)' }, { icon: 'wheel', title: 'Motors & sensors', text: 'Muscles and nerves.', k: 'var(--c-srv)' }] },
        { a: 'serial', mode: 'bytes', h: 'Talking over USB serial', t: 'The simplest link is a USB serial port. Agree on a tiny text protocol: for example `60,60` followed by a newline means "left motor 60%, right motor 60%". The newline marks the end of each command.' },
        { a: 'serial', mode: 'garbage', h: 'Baud rates must match', t: 'Both sides must use the same speed, for example **115200 baud**. If they don\'t match, the bytes turn into garbage and the robot gets confused.' },
        { a: 'terminal', h: 'Linux setup', t: 'The board shows up as a device such as `/dev/ttyACM0` or `/dev/ttyUSB0`. Add yourself to the `dialout` group to use it without sudo, then log out and back in.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ls /dev/ttyACM*' }, { o: '/dev/ttyACM0' }, { c: 'sudo usermod -a -G dialout $USER' }, { o: '(log out and back in for the change to apply)', cls: 'd' }, { c: 'python3 -m pip install pyserial' }, { o: 'Successfully installed pyserial-3.5' }] }] },
        { a: 'cards', layout: 'grid2', h: 'What about micro-ROS?', t: '**micro-ROS** runs a tiny ROS 2 client on the microcontroller itself and joins the graph through an **agent** on the computer. It takes more setup, but then the board speaks ROS directly.',
          items: [{ icon: 'usb', title: 'Serial bridge', text: 'Your own text protocol over USB. Quick to start; your driver node translates.', k: 'var(--c-msg)' }, { icon: 'node', title: 'micro-ROS', text: 'ROS 2 on the board, connected through an agent. More setup, native topics.', k: 'var(--c-topic)' }] },
      ],
      ex: [
        MC('Why add a microcontroller next to ROS 2?', ['Precise, real-time control of motors and sensors', 'To run rviz2', 'Because ROS 2 can\'t run on Linux', 'To store parameters'], 0),
        MC('The serial monitor shows garbage characters. Most likely cause?', ['The baud rates don\'t match', 'The topic name is wrong', 'The battery is empty', 'package.xml is missing'], 0),
        TYPE('Add your user to the group that may use serial ports.', 'sudo usermod -a -G dialout $USER', 'The group is called dialout.', { alt: ['sudo usermod -aG dialout $USER'] }),
        TF('With micro-ROS, the microcontroller joins ROS 2 through an agent running on the host computer.', true),
        MATCH('Match each item to its meaning.', [['/dev/ttyACM0', 'USB serial device'], ['115200', 'Baud rate'], ['\\n', 'End of one command'], ['dialout', 'Group for serial access']]),
        FILL('Open the port in Python with pyserial.', "ser = serial.___('/dev/ttyACM0', ___, timeout=0.1)", ['Serial', '115200'], ['open', 'Port', 'USB']),
      ],
    },
    {
      id: 'u12l2', title: 'A ROS driver node', icon: 'wheel',
      story: [
        { a: 'graph', h: 'The driver node', t: 'A **driver node** subscribes to `/cmd_vel`, turns each Twist into wheel commands and writes them to the serial port. Every other node simply publishes Twists.',
          nodes: [GN('tel', '/teleop', 90, 80), GN('pl', '/planner', 90, 260), GN('cv', '/cmd_vel', 270, 170, 'topic'), GN('dr', '/serial_driver', 455, 170), GN('mcu', 'microcontroller', 455, 295, 'hw')],
          edges: [GE('tel', 'cv'), GE('pl', 'cv'), GE('cv', 'dr'), { a: 'dr', b: 'mcu', flow: 'msg', shape: 'dot', label: 'USB serial' }] },
        { a: 'code', lang: 'py', code: CODE.driver, focus: [10, 12], h: 'Port as a parameter', t: 'The port is a **parameter**, so the same code works on any machine: `--ros-args -p port:=/dev/ttyUSB0`.' },
        { a: 'diffdrive', h: 'From Twist to two wheels', t: 'A differential-drive robot mixes forward speed `v` and turn rate `w`: left = v − w·b/2 and right = v + w·b/2, where b is the distance between the wheels. Move the sliders.' },
        { a: 'serial', mode: 'watchdog', h: 'Safety first: a watchdog', t: 'If commands stop arriving (a crashed node, a pulled cable), the firmware should stop the motors after a short timeout such as 0.5 s. Never trust the link blindly.' },
        { a: 'graph', h: 'Putting it all together', t: 'A camera node publishes images, a vision node finds a colored ball, a decision node publishes `/cmd_vel`, and the driver moves the motors: **perception → decision → action**, all as ROS nodes.',
          nodes: [GN('cam', '/camera', 80, 55), GN('img', '/image', 250, 55, 'topic'), GN('bt', '/ball_tracker', 445, 55), GN('tg', '/target', 445, 150, 'topic'), GN('ch', '/chaser', 250, 150), GN('cv', '/cmd_vel', 80, 150, 'topic'), GN('dr', '/serial_driver', 110, 260), GN('mcu', 'microcontroller', 320, 260, 'hw'), GN('mot', 'motors', 500, 260, 'hw')],
          edges: [GE('cam', 'img'), GE('img', 'bt'), GE('bt', 'tg'), GE('tg', 'ch'), GE('ch', 'cv'), GE('cv', 'dr'), GE('dr', 'mcu', 'msg', { shape: 'dot' }), GE('mcu', 'mot', 'msg', { shape: 'dot' })] },
      ],
      ex: [
        MC('v = 0.5 and w = 0. What do the wheels do?', ['Both turn at the same speed: straight ahead', 'Only the left wheel turns', 'They spin in opposite directions', 'Nothing'], 0),
        MC('To turn left in place, the wheel speeds should be…', ['Left backward, right forward', 'Left forward, right backward', 'Both forward', 'Both stopped'], 0),
        MC('Why stop the motors when commands stop arriving?', ['So the robot halts safely if a node or cable fails', 'To save battery between messages', 'ROS requires it', 'To reset odometry'], 0),
        FILL('Send the command line.', "self.ser.___(f'{left},{right}\\n'.___())", ['write', 'encode'], ['print', 'publish', 'decode']),
        ORDER('Order the pipeline from eyes to wheels.', ['Camera node', 'Vision node', 'Decision node', 'Serial driver node', 'Microcontroller', 'Motors']),
        TF('With a driver node, other nodes control the robot just by publishing Twist on /cmd_vel.', true),
      ],
    },
  ],
},
);
