/* ============================== course content: units 1-6 ============================== */
const MC = (q, o, a, e, x = {}) => Object.assign({ k: 'mcq', q, o, a, e }, x);
const TF = (q, a, e) => ({ k: 'tf', q, a, e });
const FILL = (q, s, b, x, e, lang = 'py') => ({ k: 'fill', q, s, b, x, e, lang });
const BUILD = (q, a, x, e, o = {}) => Object.assign({ k: 'build', q, a, x, e }, o);
const ORDER = (q, a, e) => ({ k: 'order', q, a, e });
const MATCH = (q, p, e) => ({ k: 'match', q, p, e });
const TYPE = (q, a, e, o = {}) => Object.assign({ k: 'type', q, a, e }, o);
const BUG = (q, code, a, e, lang = 'py') => ({ k: 'bug', q, code, a, e, lang });
const PICK = (q, g, a, e) => ({ k: 'pick', q, g, a, e });
const TURTLE = (q, lin, ang, e) => ({ k: 'turtle', q, lin, ang, e });

const CODE = {
  pubPy: `import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.count = 0

    def tick(self):
        msg = String()
        msg.data = f'Hello ROS {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()`,
  subPy: `import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            String, 'chatter', self.on_msg, 10)

    def on_msg(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()`,
  setupPy: `entry_points={
    'console_scripts': [
        'talker = my_py_pkg.talker:main',
        'listener = my_py_pkg.listener:main',
    ],
},`,
  cmake: `find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS talker
  DESTINATION lib/\${PROJECT_NAME})`,
  pubCpp: `#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker") {
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(500ms, [this]() { tick(); });
  }
private:
  void tick() {
    std_msgs::msg::String msg;
    msg.data = "Hello ROS " + std::to_string(count_++);
    pub_->publish(msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}`,
  subCpp: `sub_ = create_subscription<std_msgs::msg::String>(
  "chatter", 10,
  [this](const std_msgs::msg::String & msg) {
    RCLCPP_INFO(get_logger(), "Heard: %s", msg.data.c_str());
  });`,
  srvFile: `# example_interfaces/srv/AddTwoInts
int64 a
int64 b
---
int64 sum`,
  srvPy: `from example_interfaces.srv import AddTwoInts

class Adder(Node):
    def __init__(self):
        super().__init__('adder')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.handle)

    def handle(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b}')
        return response`,
  cliPy: `class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for server...')

    def ask(self, a, b):
        req = AddTwoInts.Request()
        req.a, req.b = a, b
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_done)

    def on_done(self, future):
        self.get_logger().info(f'Sum: {future.result().sum}')`,
  srvCpp: `#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class Adder : public rclcpp::Node {
public:
  Adder() : Node("adder") {
    srv_ = create_service<AddTwoInts>("add_two_ints",
      [this](const std::shared_ptr<AddTwoInts::Request> req,
             std::shared_ptr<AddTwoInts::Response> res) {
        res->sum = req->a + req->b;
        RCLCPP_INFO(get_logger(), "%ld + %ld", req->a, req->b);
      });
  }
private:
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
};`,
  cliCpp: `auto client = create_client<AddTwoInts>("add_two_ints");
while (!client->wait_for_service(1s)) {
  RCLCPP_INFO(get_logger(), "Waiting for server...");
}
auto req = std::make_shared<AddTwoInts::Request>();
req->a = 2;
req->b = 3;
client->async_send_request(req,
  [this](rclcpp::Client<AddTwoInts>::SharedFuture future) {
    RCLCPP_INFO(get_logger(), "Sum: %ld", future.get()->sum);
  });`,
};

const TWIST_SHOW = `# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z`;

UNITS.push(
/* ---------------------------------------------------------------- UNIT 1 */
{
  id: 'u1', title: 'Meet ROS 2', sub: 'What ROS is and why robots need it', color: '#0F9F91',
  notes: [
    ['ROS 2', 'Robot Operating System: open-source middleware (not an OS) that connects robot programs and ships tools and libraries.'],
    ['Node', 'A program that does one job, like driving motors or reading a camera.', 'ros2 node list'],
    ['Topic', 'A named channel. Publishers send messages to it, subscribers receive them.', 'ros2 topic list'],
    ['Message', 'A typed packet of data sent on a topic, such as geometry_msgs/msg/Twist.'],
    ['ROS graph', 'All running nodes and the topics, services and actions connecting them.', 'rqt_graph'],
    ['DDS', 'Data Distribution Service: the networking layer under ROS 2. Gives automatic discovery, no master needed.'],
    ['QoS', 'Quality of Service settings (reliability, history depth…) chosen per publisher and subscriber.'],
    ['Distro', 'A ROS 2 release. Jazzy Jalisco (May 2024) is an LTS release supported until 2029, built for Ubuntu 24.04.'],
  ],
  lessons: [
    {
      id: 'u1l1', title: 'Why robots need ROS', icon: 'lab',
      story: [
        { a: 'robotParts', h: 'A robot is a team of parts', t: 'A mobile robot has a camera, a laser scanner (**lidar**), an arm, wheels and a battery. Each part needs its own software, and they all have to share data.' },
        { a: 'robotParts', mode: 'nodes', h: 'One small program per job', t: 'In ROS each job runs as a **node**: a small program like `/camera_driver` or `/motor_driver`. You can write, test and restart each node on its own.' },
        { a: 'graph', h: 'Nodes talk with messages', t: 'Nodes send **messages** to each other over named channels called **topics**. All the nodes and topics together form the **ROS graph**.',
          nodes: [GN('cam', '/camera', 95, 70), GN('img', '/image', 300, 70, 'topic'), GN('det', '/detector', 495, 70), GN('tgt', '/target', 495, 170, 'topic'), GN('pl', '/planner', 300, 170), GN('cv', '/cmd_vel', 300, 270, 'topic'), GN('mot', '/motor_driver', 110, 270)],
          edges: [GE('cam', 'img'), GE('img', 'det'), GE('det', 'tgt'), GE('tgt', 'pl'), GE('pl', 'cv'), GE('cv', 'mot')] },
        { a: 'layers', h: 'Middleware, not an operating system', t: 'Despite the name, ROS is not an operating system. It is **middleware**: a layer on top of Linux (or Windows and macOS) that handles communication, tools and libraries.',
          items: [{ title: 'Your nodes', sub: 'camera driver, planner, motor driver…', icon: 'node', k: 'var(--c-node)' }, { title: 'ROS 2', sub: 'messaging · tools · libraries', icon: 'layers', k: 'var(--accent)', hl: true, tag: 'middleware' }, { title: 'Operating system', sub: 'Ubuntu Linux (also Windows, macOS)', icon: 'terminal', k: 'var(--c-srv)' }, { title: 'Hardware', sub: 'computer, sensors, motors', icon: 'chip', k: 'var(--muted)' }] },
        { a: 'cards', layout: 'grid2', h: 'A whole toolbox comes with it', t: 'You also get command-line tools, visualizers, simulators and thousands of open-source packages, so you never start from zero.',
          items: [{ icon: 'terminal', title: 'Tools', text: '`ros2` CLI, rqt, rviz2 for 3D views', k: 'var(--c-srv)' }, { icon: 'map', title: 'Libraries', text: 'Navigation, TF2 transforms, arm planning', k: 'var(--c-topic)' }, { icon: 'camera', title: 'Drivers', text: 'Ready-made nodes for cameras, lidars, IMUs', k: 'var(--c-node)' }, { icon: 'world', title: 'Community', text: 'Open source and used from research labs to factories', k: 'var(--accent)' }] },
      ],
      ex: [
        MC('ROS is best described as…', ['Middleware that lets robot programs talk to each other', 'An operating system that replaces Linux', 'A programming language for robots', 'A robot made by one company'], 0, 'ROS runs on top of an operating system and provides messaging, tools and libraries.'),
        TF('In ROS, a robot is usually controlled by one giant program.', false, 'ROS encourages many small nodes, each with one job.'),
        MATCH('Match each word to its meaning.', [['Node', 'A program that does one job'], ['Message', 'A packet of data'], ['Topic', 'A named channel for messages'], ['Middleware', 'The layer connecting programs']]),
        BUILD('ROS stands for…', ['Robot', 'Operating', 'System'], ['Remote', 'Online', 'Software'], 'Robot Operating System, even though it is middleware rather than a real OS.', { words: true }),
        MC('Why split a robot\'s software into many nodes?', ['Each part can be built, tested and restarted on its own', 'It makes the robot lighter', 'ROS only allows one line of code per node', 'So that nodes cannot share data'], 0, 'Small nodes are easier to understand, reuse and debug.'),
        PICK('Tap the node that publishes camera images.', { nodes: [GN('cam', '/camera_driver', 110, 120), GN('img', '/image', 320, 170, 'topic'), GN('det', '/detector', 500, 100), GN('view', '/viewer', 500, 250)], edges: [GE('cam', 'img'), GE('img', 'det'), GE('img', 'view')] }, 'cam', 'Arrows point from a publisher into a topic, and from a topic out to its subscribers.'),
      ],
    },
    {
      id: 'u1l2', title: 'ROS 1 vs ROS 2', icon: 'node',
      story: [
        { a: 'master', mode: 'ros1', h: 'ROS 1 had a central Master', t: 'In ROS 1, every node first registered with one central **ROS Master** (started with `roscore`) to find the others. Then the nodes connected directly.' },
        { a: 'master', mode: 'crash', h: 'A single point of failure', t: 'If the Master crashed, nodes that were already connected kept talking, but nothing new could find anyone. Not great for a robot working in a factory.' },
        { a: 'master', mode: 'ros2', h: 'ROS 2: no Master needed', t: 'ROS 2 is built on **DDS** (Data Distribution Service). Nodes **discover each other automatically** on the network and talk peer to peer.' },
        { a: 'cards', layout: 'grid2', h: 'Built for real products', t: 'ROS 2 adds **Quality of Service (QoS)** settings, security, better real-time behavior and support for Linux, Windows and macOS.',
          items: [{ icon: 'sliders', title: 'QoS', text: 'Choose reliability, history depth and more for each connection.', k: 'var(--c-topic)' }, { icon: 'shield', title: 'Security', text: 'Encryption and access control for robot traffic.', k: 'var(--c-srv)' }, { icon: 'clock', title: 'Real-time friendly', text: 'Designed with deterministic control loops in mind.', k: 'var(--accent)' }, { icon: 'world', title: 'Multi-robot, multi-OS', text: 'Fleets of robots; Linux, Windows and macOS.', k: 'var(--c-node)' }] },
        { a: 'cards', layout: 'timeline', h: 'Releases have turtle names', t: 'ROS 2 releases ("distros") come out every May in alphabetical order. Even-year releases are **LTS** with five years of support. This course uses **Jazzy Jalisco** on Ubuntu 24.04. ROS 1 reached end-of-life in May 2025.',
          items: [{ when: 'May 2022', title: 'Humble Hawksbill', text: 'LTS', k: 'var(--c-srv)' }, { when: 'May 2023', title: 'Iron Irwini', text: 'short support', k: 'var(--muted)' }, { when: 'May 2024', title: 'Jazzy Jalisco', text: 'LTS · this course', k: 'var(--accent)', hl: true }, { when: 'May 2025', title: 'Kilted Kaiju', text: 'short support', k: 'var(--muted)' }] },
      ],
      ex: [
        MC('In ROS 1, how did nodes find each other?', ['They registered with the central ROS Master', 'They used DDS discovery', 'They read a shared config file', 'They never needed to'], 0),
        MC('What does ROS 2 use for discovery and transport?', ['DDS', 'HTTP', 'Bluetooth', 'The ROS Master'], 0, 'DDS (Data Distribution Service) is an industry standard for real-time publish/subscribe.'),
        TF('ROS 2 needs a master process running before nodes can talk.', false, 'Discovery is decentralized: there is no master in ROS 2.'),
        MATCH('Match the pairs.', [['ROS 1', 'Central master (roscore)'], ['ROS 2', 'Automatic discovery (DDS)'], ['QoS', 'Reliability and history settings'], ['LTS', 'Long-term support release']]),
        ORDER('Put these ROS 2 releases in order, oldest first.', ['Humble Hawksbill', 'Iron Irwini', 'Jazzy Jalisco', 'Kilted Kaiju'], 'Releases are alphabetical: H, I, J, K…'),
        MC('A company is starting a new warehouse robot fleet. Which should they pick?', ['ROS 2: decentralized, secure, QoS-aware', 'ROS 1: it has a Master', 'Either: they are identical', 'Neither: ROS is only for research'], 0, 'ROS 1 is end-of-life; ROS 2 was designed for products like this.'),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 2 */
{
  id: 'u2', title: 'The ros2 command line', sub: 'Run nodes, inspect topics and call services', color: '#7462F0',
  lab: { tab: 'term', text: 'Try these commands on a live robot' },
  notes: [
    ['Source the setup', 'Each new terminal needs ROS loaded before ros2 works.', 'source /opt/ros/jazzy/setup.bash'],
    ['ros2 run', 'Start one executable from a package.', 'ros2 run demo_nodes_cpp talker'],
    ['ros2 topic echo', 'Print every message arriving on a topic.', 'ros2 topic echo /chatter'],
    ['ros2 topic info / hz', 'Count publishers and subscribers, or measure the publish rate.', 'ros2 topic info /chatter\nros2 topic hz /chatter'],
    ['ros2 topic pub', 'Publish a message from the terminal.', 'ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"'],
    ['Twist', 'Velocity command: linear (m/s) and angular (rad/s). Ground robots use linear.x and angular.z.', 'ros2 interface show geometry_msgs/msg/Twist'],
    ['Interfaces', 'Messages (.msg) for topics, services (.srv) for request/response, actions (.action) for long goals.', 'ros2 interface list'],
    ['ros2 service call', 'Send a request to a service and print the response.', 'ros2 service call /clear std_srvs/srv/Empty'],
  ],
  lessons: [
    {
      id: 'u2l1', title: 'Your first commands', icon: 'terminal',
      story: [
        { a: 'terminal', h: 'Wake up the ros2 command', t: 'Whether ROS 2 is installed on Ubuntu or runs inside a Docker container, every new terminal must load it first with `source /opt/ros/jazzy/setup.bash`. Then the `ros2` command works.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 node list' }, { o: 'ros2: command not found', cls: 'e', pause: 700 }, { c: 'source /opt/ros/jazzy/setup.bash' }, { c: 'ros2 --help' }, { o: 'usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...' }, { o: 'Commands:', cls: 'd' }, { o: '  node      Various node related sub-commands\n  run       Run a package specific executable\n  service   Various service related sub-commands\n  topic     Various topic related sub-commands', cls: 'd' }] }] },
        { a: 'terminal', h: 'Run two nodes', t: '`ros2 run <package> <executable>` starts a node. Run the demo **talker** in one terminal and the **listener** in another. They find each other automatically.',
          panes: [{ title: 'Terminal 1', lines: [{ c: 'ros2 run demo_nodes_cpp talker' }, { stream: ["[INFO] [{t}] [talker]: Publishing: 'Hello World: {n}'"], every: 1000 }] }, { title: 'Terminal 2', lines: [{ wait: 1200 }, { c: 'ros2 run demo_nodes_py listener' }, { stream: ['[INFO] [{t}] [listener]: I heard: [Hello World: {n}]'], every: 1000 }] }] },
        { a: 'terminal', h: 'See what is running', t: '`ros2 node list` shows the running nodes and `ros2 topic list` shows the topics. `/parameter_events` and `/rosout` exist automatically because every node uses them.',
          panes: [{ title: 'Terminal 3', lines: [{ c: 'ros2 node list' }, { o: '/listener\n/talker' }, { c: 'ros2 topic list' }, { o: '/chatter\n/parameter_events\n/rosout' }, { c: 'ros2 topic info /chatter' }, { o: 'Type: std_msgs/msg/String\nPublisher count: 1\nSubscription count: 1' }] }] },
        { a: 'terminal', h: 'Listen in on a topic', t: '`ros2 topic echo /chatter` prints every message on the topic. `ros2 topic hz /chatter` measures how often messages arrive.',
          panes: [{ title: 'echo', lines: [{ c: 'ros2 topic echo /chatter' }, { stream: ["data: 'Hello World: {n}'", '---'], every: 500 }] }, { title: 'hz', lines: [{ wait: 900 }, { c: 'ros2 topic hz /chatter' }, { stream: ['average rate: 1.000\n\tmin: 1.000s max: 1.000s std dev: 0.0002s window: {n}'], every: 1000 }] }] },
        { a: 'graph', h: 'Draw the graph', t: 'Run `rqt_graph` to see nodes and topics as a live diagram. It is the quickest way to understand a running system.',
          nodes: [GN('t', '/talker', 110, 170), GN('c', '/chatter', 300, 170, 'topic'), GN('l', '/listener', 490, 170)], edges: [GE('t', 'c'), GE('c', 'l')],
          notes: [{ x: 300, y: 60, text: 'rqt_graph', cls: 't-h' }, { x: 300, y: 250, text: 'nodes publish into topics; topics deliver to subscribers' }] },
      ],
      ex: [
        TYPE('Start the talker node from the demo_nodes_cpp package.', 'ros2 run demo_nodes_cpp talker', 'Pattern: ros2 run <package> <executable>.', { out: "[INFO] [1730000000.512] [talker]: Publishing: 'Hello World: 1'" }),
        BUILD('List the running nodes.', ['ros2', 'node', 'list'], ['topic', 'run', 'show']),
        MC('Which command prints the messages flowing on /chatter?', ['`ros2 topic echo /chatter`', '`ros2 topic list /chatter`', '`ros2 node echo /chatter`', '`ros2 run /chatter`'], 0),
        MC('In a new terminal, `ros2` says "command not found". What did you forget?', ['To source the ROS 2 setup file', 'To restart the computer', 'To run rqt_graph', 'To install Python'], 0, 'Run `source /opt/ros/jazzy/setup.bash` (many people add it to ~/.bashrc).'),
        FILL('Show how many publishers and subscribers /chatter has.', 'ros2 ___ ___ /chatter', ['topic', 'info'], ['node', 'hz', 'list'], null, 'sh'),
        MATCH('Match each command to what it does.', [['ros2 node list', 'Show running nodes'], ['ros2 topic echo', 'Print messages on a topic'], ['ros2 topic hz', 'Measure the publish rate'], ['rqt_graph', 'Draw the node graph']]),
      ],
    },
    {
      id: 'u2l2', title: 'Drive a turtle', icon: 'drive',
      story: [
        { a: 'turtle', mode: 'teleop', h: 'Meet turtlesim', t: '**turtlesim** is a tiny simulator that ships with ROS 2. Start it with `ros2 run turtlesim turtlesim_node`, then drive with `ros2 run turtlesim turtle_teleop_key` and the arrow keys.' },
        { a: 'graph', h: 'Keys become messages', t: 'The teleop node publishes **velocity commands** on `/turtle1/cmd_vel`. turtlesim subscribes, moves the turtle and publishes its position on `/turtle1/pose`.',
          nodes: [GN('tel', '/teleop_turtle', 115, 90), GN('cv', '/turtle1/cmd_vel', 330, 90, 'topic'), GN('ts', '/turtlesim', 500, 190), GN('ps', '/turtle1/pose', 300, 270, 'topic')],
          edges: [GE('tel', 'cv'), GE('cv', 'ts'), GE('ts', 'ps')] },
        { a: 'terminal', h: 'What is inside a Twist?', t: 'Every topic has a message **type**. `/turtle1/cmd_vel` uses `geometry_msgs/msg/Twist`: a **linear** and an **angular** velocity. For a ground robot, `linear.x` is forward speed and `angular.z` is the turn rate.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 topic type /turtle1/cmd_vel' }, { o: 'geometry_msgs/msg/Twist' }, { c: 'ros2 interface show geometry_msgs/msg/Twist' }, { o: TWIST_SHOW }] }] },
        { a: 'turtlePlay', h: 'Your turn: publish a Twist', t: 'Set the speeds and publish. This is exactly what `ros2 topic pub` does from the terminal. A positive `angular.z` turns left (counterclockwise).' },
        { a: 'cards', layout: 'grid3', h: 'Three kinds of interfaces', t: '`ros2 interface list` shows every known type. **Messages** (`.msg`) are for topics, **services** (`.srv`) for request and response, and **actions** (`.action`) for long-running goals with feedback.',
          items: [{ icon: 'msg', title: 'Message', text: 'Topics: one-way data streams.', code: 'geometry_msgs/msg/Twist', k: 'var(--c-msg)' }, { icon: 'bell', title: 'Service', text: 'One request, one response.', code: 'turtlesim/srv/Spawn', k: 'var(--c-srv)' }, { icon: 'truck', title: 'Action', text: 'Long goals with feedback; can be canceled.', code: 'turtlesim/action/RotateAbsolute', k: 'var(--c-param)' }] },
      ],
      ex: [
        TURTLE('Make the turtle drive a counterclockwise circle.', [0.3, 99], [0.3, 99], 'Forward speed plus a positive turn rate draws a counterclockwise circle.'),
        MC('Which Twist field makes a ground robot drive forward?', ['`linear.x`', '`linear.z`', '`angular.x`', '`angular.z`'], 0),
        TF('A negative `angular.z` turns the turtle clockwise.', true, 'Positive rotation about z (pointing up) is counterclockwise, so negative is clockwise.'),
        FILL('Print the fields of the Twist type.', 'ros2 interface ___ geometry_msgs/msg/___', ['show', 'Twist'], ['list', 'echo', 'Pose'], null, 'sh'),
        BUILD('Publish one Twist that drives the turtle forward.', ['ros2', 'topic', 'pub', '--once', '/turtle1/cmd_vel', 'geometry_msgs/msg/Twist', '"{linear: {x: 2.0}}"'], ['echo', '/turtle1/pose']),
        MC('Which interface fits "rotate to 90° and report progress while turning"?', ['An action', 'A message', 'A service', 'A parameter'], 0, 'Long tasks with feedback are actions. turtlesim even has /turtle1/rotate_absolute.'),
      ],
    },
    {
      id: 'u2l3', title: 'Call a service', icon: 'bell',
      story: [
        { a: 'service', h: 'Ask and get an answer', t: 'A **service** is a request and a response, like asking a question. turtlesim offers services such as `/spawn`, `/clear` and `/reset`.', req: "x: 2.0, y: 2.0", res: "name: 'turtle2'", name: '/spawn' },
        { a: 'terminal', h: 'List and inspect services', t: '`ros2 service list` shows the services, `ros2 service type /spawn` gives the type, and `ros2 interface show turtlesim/srv/Spawn` shows the request and response fields.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 service list' }, { o: '/clear\n/kill\n/reset\n/spawn\n/turtle1/set_pen\n/turtle1/teleport_absolute\n/turtle1/teleport_relative\n…' }, { c: 'ros2 service type /spawn' }, { o: 'turtlesim/srv/Spawn' }, { c: 'ros2 interface show turtlesim/srv/Spawn' }, { o: 'float32 x\nfloat32 y\nfloat32 theta\nstring name # Optional.  A unique name will be created and returned if this is empty\n---\nstring name' }] }] },
        { a: 'turtle', mode: 'spawn', h: 'Spawn a second turtle', t: 'Calling `/spawn` with a position asks turtlesim to create a new turtle. The response returns its name.' },
        { a: 'terminal', h: 'Some services need no data', t: '`std_srvs/srv/Empty` has no fields at all. Calling `/clear` simply erases the trails.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 service call /clear std_srvs/srv/Empty' }, { o: 'requester: making request: std_srvs.srv.Empty_Request()\n\nresponse:\nstd_srvs.srv.Empty_Response()' }] }] },
      ],
      ex: [
        MC('What does a service give you that a topic does not?', ['A response to each request', 'A continuous data stream', 'Faster messages', 'No need for a type'], 0),
        TYPE('Clear the turtlesim drawing with the /clear service (type std_srvs/srv/Empty).', 'ros2 service call /clear std_srvs/srv/Empty', 'Pattern: ros2 service call <service> <type> [arguments].', { out: 'requester: making request: std_srvs.srv.Empty_Request()\n\nresponse:\nstd_srvs.srv.Empty_Response()' }),
        BUILD('Find the type of the /spawn service.', ['ros2', 'service', 'type', '/spawn'], ['topic', 'call', 'list']),
        TF('Spawning two turtles with the same name gives you two identical turtles.', false, 'turtlesim refuses: every turtle name must be unique.'),
        MATCH('Match each command to its job.', [['ros2 service list', 'Show available services'], ['ros2 service type', 'Show a service\'s type'], ['ros2 service call', 'Send a request'], ['ros2 interface show', 'Show the request and response fields']]),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 3 */
{
  id: 'u3', title: 'Publishers & subscribers in Python', sub: 'Write your first rclpy nodes', color: '#E58A00',
  lab: { tab: 'code', text: 'Write a Python node that drives a robot' },
  notes: [
    ['Workspace', 'A folder like ~/ros2_ws with your packages in src/. Building creates build/, install/ and log/.'],
    ['Create a Python package', 'Scaffold a package with dependencies.', 'ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy std_msgs'],
    ['Entry point', 'setup.py maps an executable name to a function so ros2 run can start it.', "'talker = my_py_pkg.talker:main'"],
    ['Build and source', 'Build from the workspace root, then load the result.', 'colcon build --symlink-install\nsource install/setup.bash'],
    ['Publisher', 'Type, topic and queue depth.', "self.pub = self.create_publisher(String, 'chatter', 10)"],
    ['Timer', 'Calls a function periodically.', 'self.timer = self.create_timer(0.5, self.tick)'],
    ['Subscription', 'Type, topic, callback and queue depth.', "self.create_subscription(String, 'chatter', self.on_msg, 10)"],
    ['spin()', 'Keeps a node alive and runs its callbacks until shutdown.', 'rclpy.spin(node)'],
  ],
  lessons: [
    {
      id: 'u3l1', title: 'Workspaces & packages', icon: 'pkg',
      story: [
        { a: 'tree', h: 'A workspace holds your code', t: 'A **workspace** is a folder, often `~/ros2_ws`, with a `src/` directory for your packages. Building creates `build/`, `install/` and `log/` next to it.',
          items: [{ d: 0, n: 'ros2_ws/', dir: true }, { d: 1, n: 'src/', dir: true, note: 'your packages live here', hl: true }, { d: 2, n: 'my_py_pkg/', dir: true }, { d: 1, n: 'build/', dir: true, note: 'temporary build files', at: 4 }, { d: 1, n: 'install/', dir: true, note: 'built output you source', at: 5 }, { d: 1, n: 'log/', dir: true, note: 'build logs', at: 6 }] },
        { a: 'terminal', h: 'Create a Python package', t: '`ros2 pkg create` scaffolds a package. `ament_python` is the build type for Python, and `--dependencies` writes them into `package.xml`.',
          panes: [{ title: 'Terminal', lines: [{ c: 'cd ~/ros2_ws/src' }, { c: 'ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy std_msgs' }, { o: 'going to create a new package\npackage name: my_py_pkg\ndestination directory: /home/you/ros2_ws/src\npackage format: 3\nbuild type: ament_python\ndependencies: [\'rclpy\', \'std_msgs\']\ncreating folder ./my_py_pkg\ncreating ./my_py_pkg/package.xml\ncreating source folder\ncreating ./my_py_pkg/setup.py' }] }] },
        { a: 'tree', h: 'Inside a Python package', t: '`package.xml` describes the package and its dependencies. `setup.py` lists **entry points**: the executables that `ros2 run` can start.',
          items: [{ d: 0, n: 'my_py_pkg/', dir: true }, { d: 1, n: 'package.xml', note: 'name, version, dependencies', hl: true }, { d: 1, n: 'setup.py', note: 'entry points (executables)', hl: true }, { d: 1, n: 'setup.cfg' }, { d: 1, n: 'resource/my_py_pkg' }, { d: 1, n: 'my_py_pkg/', dir: true }, { d: 2, n: '__init__.py' }, { d: 2, n: 'talker.py', note: 'your node' }, { d: 1, n: 'test/', dir: true }] },
        { a: 'code', lang: 'py', code: CODE.setupPy, focus: [3, 4], h: 'Register your node', t: 'An entry point maps an executable name to a function: `talker = my_py_pkg.talker:main` makes `ros2 run my_py_pkg talker` call `main()` inside `talker.py`.', note: 'executable name = module path : function' },
        { a: 'cards', layout: 'row', checks: true, h: 'Build, source, run', t: 'From the workspace root run `colcon build`, then `source install/setup.bash` so ROS can find your package, then `ros2 run`. Tip: `--symlink-install` lets Python edits apply without rebuilding.',
          items: [{ icon: 'tools', title: 'colcon build', text: 'in ~/ros2_ws', k: 'var(--c-srv)' }, { icon: 'terminal', title: 'source', text: 'install/setup.bash', k: 'var(--c-topic)' }, { icon: 'play', title: 'ros2 run', text: 'my_py_pkg talker', k: 'var(--good)' }] },
      ],
      ex: [
        ORDER('Put the workflow in order.', ['ros2 pkg create …', 'Write the node code', 'Add an entry point in setup.py', 'colcon build', 'source install/setup.bash', 'ros2 run my_py_pkg talker']),
        MC('Which tool builds a ROS 2 workspace?', ['`colcon`', '`catkin_make`', '`pip`', '`make install`'], 0, 'catkin was the ROS 1 build tool. ROS 2 uses colcon.'),
        FILL('Create a Python package.', 'ros2 pkg create --build-type ___ my_py_pkg --dependencies ___ std_msgs', ['ament_python', 'rclpy'], ['ament_cmake', 'rclcpp', 'catkin'], null, 'sh'),
        MATCH('What is each file or folder for?', [['package.xml', 'Name, version, dependencies'], ['setup.py', 'Entry points for executables'], ['src/', 'Where your packages live'], ['install/', 'Built output you source']]),
        TF('After `colcon build`, a brand new terminal can `ros2 run` your package without sourcing anything.', false, 'Every new terminal needs `source install/setup.bash` on top of the ROS setup.'),
        MC('Where do you run `colcon build`?', ['In the workspace root, such as ~/ros2_ws', 'Inside src/my_py_pkg', 'In /opt/ros/jazzy', 'Anywhere: it finds the workspace'], 0),
      ],
    },
    {
      id: 'u3l2', title: 'Write a publisher', icon: 'msg',
      story: [
        { a: 'code', lang: 'py', code: CODE.pubPy, focus: [5, 7], h: 'A node is a class', t: 'Your node is a Python class that inherits from `Node`. `super().__init__(\'talker\')` gives it its name: `/talker`.', note: 'class Talker(Node) → a node called /talker' },
        { a: 'code', lang: 'py', code: CODE.pubPy, focus: [8, 8], h: 'Create a publisher', t: '`create_publisher(String, \'chatter\', 10)` takes the **message type**, the **topic name** and a **queue depth**, the simplest QoS setting.', note: 'type · topic · queue depth' },
        { a: 'tick', period: '0.5 s', h: 'A timer keeps it ticking', t: '`create_timer(0.5, self.tick)` calls `tick()` every 0.5 s (2 Hz). Each tick builds a `String`, fills in `data` and publishes it.' },
        { a: 'code', lang: 'py', code: CODE.pubPy, focus: [19, 24], h: 'main(): init, spin, shut down', t: '`rclpy.init()` starts ROS communication. `rclpy.spin(node)` keeps the node alive and runs its callbacks. `shutdown()` cleans up after you press Ctrl+C.', note: 'spin() blocks here until shutdown' },
        { a: 'queue', depth: 5, h: 'What does the 10 mean?', t: 'The queue depth is how many outgoing messages may wait when they can\'t be delivered fast enough. When the queue is full, the **oldest** message is dropped.' },
      ],
      ex: [
        FILL('Complete the publisher.', "self.pub = self.___(String, 'chatter', ___)", ['create_publisher', '10'], ['create_subscription', 'publish', '0.5']),
        MC('How often does `create_timer(0.5, self.tick)` call `tick()`?', ['Every 0.5 seconds (2 Hz)', '5 times per second', 'Once, after 0.5 s', 'Every 5 seconds'], 0),
        BUG('This node never sends anything. Tap the buggy line.', `def tick(self):
    msg = String()
    msg.data = 'Hello'
    self.pub.publish(String)`, 3, 'It publishes the class `String` instead of the message `msg`.'),
        ORDER('Order the lines inside main().', ['rclpy.init()', 'node = Talker()', 'rclpy.spin(node)', 'node.destroy_node()', 'rclpy.shutdown()']),
        MC('What does `rclpy.spin(node)` do?', ['Keeps the node running and processes its callbacks', 'Spins the robot in place', 'Publishes one message', 'Builds the package'], 0),
        TF('When the queue is full, the newest message is dropped.', false, 'With the default "keep last" history, the oldest message makes room for the new one.'),
      ],
    },
    {
      id: 'u3l3', title: 'Write a subscriber', icon: 'radio',
      story: [
        { a: 'code', lang: 'py', code: CODE.subPy, focus: [8, 9], h: 'Subscribe to a topic', t: '`create_subscription` needs the **type**, the **topic name**, a **callback** and a queue depth. Type and topic must match the publisher.', note: 'pass self.on_msg without parentheses: ROS calls it for you' },
        { a: 'tick', sub: true, period: '0.5 s', h: 'Callbacks run on arrival', t: 'Every time a message arrives, ROS calls `on_msg(msg)`. You never call it yourself: `spin()` does.' },
        { a: 'graph', h: 'Many to many', t: 'A topic can have any number of publishers and subscribers. Publishers don\'t know who is listening. This **decoupling** makes systems easy to extend.',
          nodes: [GN('t1', '/talker', 100, 100), GN('t2', '/talker_2', 100, 250), GN('c', '/chatter', 300, 175, 'topic'), GN('a', '/listener_a', 495, 70), GN('b', '/listener_b', 495, 175), GN('d', '/logger', 495, 280)],
          edges: [GE('t1', 'c'), GE('t2', 'c'), GE('c', 'a'), GE('c', 'b'), GE('c', 'd')] },
        { a: 'graph', h: 'Types must match', t: 'If a subscriber expects a different type than the publisher sends, they will not connect and no messages arrive. `ros2 topic info -v /chatter` helps you spot it.',
          nodes: [GN('t', '/talker', 100, 160, 'node', { sub: 'sends std_msgs/String' }), GN('c', '/chatter', 300, 160, 'topic'), GN('l', '/listener', 495, 160, 'bad', { sub: 'expects std_msgs/Int32' })],
          edges: [GE('t', 'c'), { a: 'c', b: 'l', bad: true, label: 'type mismatch' }] },
        { a: 'terminal', h: 'Run them together', t: 'Build, source, then start each node in its own terminal. The listener logs every message the talker sends.',
          panes: [{ title: 'talker', lines: [{ c: 'ros2 run my_py_pkg talker' }, { stream: ['[INFO] [{t}] [talker]: Sent: Hello ROS {n}'], every: 900 }] }, { title: 'listener', lines: [{ wait: 1000 }, { c: 'ros2 run my_py_pkg listener' }, { stream: ['[INFO] [{t}] [listener]: Heard: Hello ROS {n}'], every: 900 }] }] },
      ],
      ex: [
        FILL('Complete the subscription.', "self.create_subscription(String, 'chatter', ___, 10)", ['self.on_msg'], ['self.on_msg()', 'on_msg', 'self.pub'], 'Pass the method itself. Adding () would call it right away.'),
        MC('When does `on_msg` run?', ['Each time a message arrives on chatter', 'Once when the node starts', 'Every 0.5 seconds', 'When main() calls it'], 0),
        TF('A topic can have several publishers and several subscribers.', true),
        MC('The talker publishes `std_msgs/msg/String` on /chatter. Your listener subscribes to /chatter with `Int32`. What happens?', ['No messages arrive: the types don\'t match', 'Strings are converted to numbers', 'The talker crashes', 'It works, just slower'], 0),
        PICK('Tap the node that subscribes to /chatter.', { nodes: [GN('talker', '/talker', 110, 170), GN('chatter', '/chatter', 300, 170, 'topic'), GN('listener', '/listener', 490, 170)], edges: [GE('talker', 'chatter'), GE('chatter', 'listener')] }, 'listener', 'The arrow from the topic points into its subscriber.'),
        MC('The talker sends "Hello ROS 3". What does this listener log?', ['`Heard: Hello ROS 3`', '`Heard: msg.data`', '`Hello ROS 3: Heard`', 'Nothing'], 0, null, { code: "def on_msg(self, msg):\n    self.get_logger().info(f'Heard: {msg.data}')", lang: 'py' }),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 4 */
{
  id: 'u4', title: 'Publishers & subscribers in C++', sub: 'The same ideas with rclcpp', color: '#2F78E4',
  notes: [
    ['rclcpp', 'The C++ client library. rclpy is the Python one. Both sit on rcl, rmw and DDS.'],
    ['C++ package', 'C++ packages use the ament_cmake build type.', 'ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp std_msgs'],
    ['CMakeLists.txt', 'find_package, add_executable, ament_target_dependencies, install.', 'install(TARGETS talker DESTINATION lib/${PROJECT_NAME})'],
    ['Templated publisher', 'The message type is a template parameter.', 'create_publisher<std_msgs::msg::String>("chatter", 10)'],
    ['Wall timer', 'Takes a std::chrono duration.', 'create_wall_timer(500ms, [this]() { tick(); })'],
    ['RCLCPP_INFO', 'printf-style logging. Strings need .c_str().', 'RCLCPP_INFO(get_logger(), "Heard: %s", msg.data.c_str());'],
  ],
  lessons: [
    {
      id: 'u4l1', title: 'C++ package anatomy', icon: 'code',
      story: [
        { a: 'layers', h: 'Same ROS, two languages', t: 'Python nodes use **rclpy** and C++ nodes use **rclcpp**. Both sit on the same core library (**rcl**) and DDS layers, so Python and C++ nodes talk to each other seamlessly.',
          items: [{ title: 'Your node', sub: 'Python or C++', icon: 'node', k: 'var(--c-node)' }, { title: 'rclpy / rclcpp', sub: 'client libraries', icon: 'code', k: 'var(--accent)', hl: true }, { title: 'rcl', sub: 'shared core, written in C', icon: 'chip', k: 'var(--c-topic)' }, { title: 'rmw', sub: 'middleware interface', icon: 'layers', k: 'var(--c-srv)' }, { title: 'DDS', sub: 'e.g. Fast DDS, Cyclone DDS', icon: 'world', k: 'var(--muted)' }] },
        { a: 'terminal', h: 'Create a C++ package', t: 'C++ packages use the `ament_cmake` build type. Code goes in `src/`, and `CMakeLists.txt` describes how to build it.',
          panes: [{ title: 'Terminal', lines: [{ c: 'ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp std_msgs' }, { o: 'going to create a new package\npackage name: my_cpp_pkg\nbuild type: ament_cmake\ndependencies: [\'rclcpp\', \'std_msgs\']\ncreating ./my_cpp_pkg/package.xml\ncreating source and include folder\ncreating ./my_cpp_pkg/CMakeLists.txt' }] }] },
        { a: 'code', lang: 'cmake', code: CODE.cmake, h: 'CMakeLists.txt in four moves', t: 'Find the dependencies, add an executable, link it against the ROS libraries, and **install** it so `ros2 run` can find it.',
          steps: [{ lines: [1, 2], note: 'find the packages you depend on' }, { lines: [4, 4], note: 'compile src/talker.cpp into an executable' }, { lines: [5, 5], note: 'link it with rclcpp and std_msgs' }, { lines: [7, 8], note: 'install it where `ros2 run` looks' }] },
        { a: 'cards', layout: 'grid2', h: 'Which language when?', t: 'Both languages are first-class. Python is quick for prototypes and high-level logic. C++ shines in performance-critical drivers and control loops.',
          items: [{ icon: 'python', title: 'Python · rclpy', text: 'No compile step, short code, great for prototypes, scripts and high-level behavior.', k: 'var(--c-msg)' }, { icon: 'chip', title: 'C++ · rclcpp', text: 'Fast and predictable: drivers, control loops, heavy processing.', k: 'var(--c-srv)' }] },
      ],
      ex: [
        MC('Which build type do C++ packages use?', ['`ament_cmake`', '`ament_python`', '`catkin`', '`cmake_python`'], 0),
        ORDER('Order the layers from your code down to the network.', ['Your node', 'rclcpp', 'rcl', 'rmw', 'DDS']),
        MATCH('Match each layer to its role.', [['rclpy', 'Python client library'], ['rclcpp', 'C++ client library'], ['rmw', 'Middleware interface'], ['DDS', 'Transport underneath']]),
        TF('If you forget the `install()` rule, `ros2 run` cannot find your C++ executable.', true),
        FILL('Declare the executable.', 'add_executable(___ src/talker.cpp)\nament_target_dependencies(talker ___ std_msgs)', ['talker', 'rclcpp'], ['rclpy', 'main', 'src'], null, 'cmake'),
        TF('A Python listener can receive messages from a C++ talker.', true, 'They share the same DDS layer and message definitions.'),
      ],
    },
    {
      id: 'u4l2', title: 'Talker & listener in C++', icon: 'msg',
      story: [
        { a: 'code', lang: 'cpp', code: CODE.pubCpp, focus: [2, 3], h: 'Headers for everything', t: 'Each message type has its own header. `std_msgs/msg/String` lives in `std_msgs/msg/string.hpp`: note the snake_case file name.', note: 'String → string.hpp' },
        { a: 'code', lang: 'cpp', code: CODE.pubCpp, focus: [8, 9], h: 'Templates pick the type', t: '`create_publisher<std_msgs::msg::String>("chatter", 10)` passes the message type as a **template parameter**. It returns a shared pointer, so you call `pub_->publish(msg)`.' },
        { a: 'code', lang: 'cpp', code: CODE.pubCpp, focus: [10, 10], h: 'Timers use chrono durations', t: '`create_wall_timer(500ms, ...)` takes a `std::chrono` duration. The `ms` suffix comes from `std::chrono_literals`, and the lambda calls `tick()`.' },
        { a: 'compare', left: 'Python (rclpy)', right: 'C++ (rclcpp)', h: 'Python and C++ side by side', t: 'The concepts map one to one. Learn them once and you can read nodes in either language.',
          rows: [["self.create_publisher(String, 'chatter', 10)", 'create_publisher<String>("chatter", 10)'], ['self.create_timer(0.5, self.tick)', 'create_wall_timer(500ms, [this]() { tick(); })'], ["self.get_logger().info('hi')", 'RCLCPP_INFO(get_logger(), "hi")'], ['rclpy.spin(node)', 'rclcpp::spin(node)']] },
        { a: 'code', lang: 'cpp', code: CODE.subCpp, h: 'The C++ listener', t: 'A subscription takes the topic, the depth and a callback, here a lambda. `RCLCPP_INFO` logs with printf-style formatting, so strings need `.c_str()`.', note: 'the lambda runs for every message' },
      ],
      ex: [
        MATCH('Match Python to C++.', [["create_timer(0.5, cb)", 'create_wall_timer(500ms, cb)'], ['get_logger().info()', 'RCLCPP_INFO(get_logger(), ...)'], ['rclpy.spin(node)', 'rclcpp::spin(node)'], ['import rclpy', '#include "rclcpp/rclcpp.hpp"']]),
        MC('Why `pub_->publish(msg)` with an arrow?', ['`pub_` is a shared pointer', 'Arrows are faster than dots', 'Arrows send messages', 'It is a typo for =>'], 0),
        MC('Which header provides `std_msgs::msg::String`?', ['`std_msgs/msg/string.hpp`', '`std_msgs/msg/String.hpp`', '`std_msgs/String.h`', '`rclcpp/string.hpp`'], 0),
        BUG('Which line will not compile?', `using namespace std::chrono_literals;
pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
timer_ = create_wall_timer(500, [this]() { tick(); });
pub_->publish(msg);`, 2, '`create_wall_timer` needs a duration such as `500ms`, not a plain number.', 'cpp'),
        FILL('Log the received string.', 'RCLCPP_INFO(get_logger(), "Heard: %s", msg.data.___());', ['c_str'], ['str', 'data', 'print'], null, 'cpp'),
        TF('`rclcpp::spin()` blocks and runs callbacks until shutdown.', true),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 5 */
{
  id: 'u5', title: 'Services in Python', sub: 'Request and response', color: '#DB3D7F',
  notes: [
    ['Service', 'A request/response pair served by one server. Clients ask, the server answers once.'],
    ['.srv file', 'Request fields, then ---, then response fields.', 'int64 a\nint64 b\n---\nint64 sum'],
    ['Server (Python)', 'The callback fills the response and returns it.', "self.create_service(AddTwoInts, 'add_two_ints', self.handle)"],
    ['Client (Python)', 'Wait for the server, then call asynchronously.', "self.cli = self.create_client(AddTwoInts, 'add_two_ints')\nfuture = self.cli.call_async(req)"],
    ['Future', 'A placeholder for a result that arrives later.', 'future.add_done_callback(self.on_done)'],
    ['Topic, service or action?', 'Topic: continuous data. Service: quick request/response. Action: long goal with feedback.'],
  ],
  lessons: [
    {
      id: 'u5l1', title: 'Request & response', icon: 'bell',
      story: [
        { a: 'service', mode: 'compare', h: 'Topics stream, services answer', t: 'A **topic** is a one-way stream: publish and forget. A **service** is a round trip: a client sends a **request** and the server sends back one **response**.' },
        { a: 'cards', layout: 'grid3', h: 'Everyday analogies', t: 'A topic is like a radio broadcast. A service is like ordering at a counter. An **action** is like a delivery with live tracking: a long goal with feedback.',
          items: [{ icon: 'radio', title: 'Topic = radio', text: 'Broadcasts to anyone tuned in. Nobody replies.', k: 'var(--c-msg)' }, { icon: 'bell', title: 'Service = counter', text: 'You ask, wait, and get your order.', k: 'var(--c-srv)' }, { icon: 'truck', title: 'Action = delivery', text: 'You order, get updates, and can cancel.', k: 'var(--c-param)' }] },
        { a: 'code', lang: 'msg', code: CODE.srvFile, h: 'The .srv format', t: 'A `.srv` file lists the **request** fields, then `---`, then the **response** fields. This is `example_interfaces/srv/AddTwoInts`.', note: 'above --- : request · below --- : response' },
        { a: 'service', mode: 'clients', h: 'One server, many clients', t: 'A service name should have exactly **one server**, but any number of clients can call it. Each request gets its own response.' },
        { a: 'cards', layout: 'grid3', h: 'Pick the right tool', t: 'Use topics for continuous data, services for quick request and response, and actions for long tasks that need feedback or canceling.',
          items: [{ icon: 'msg', title: 'Topic', text: 'Sensor data, robot state, anything continuous.', k: 'var(--c-msg)' }, { icon: 'bell', title: 'Service', text: 'Quick questions and commands: reset, spawn, set a mode.', k: 'var(--c-srv)' }, { icon: 'truck', title: 'Action', text: 'Long tasks: drive to a goal, move an arm.', k: 'var(--c-param)' }] },
      ],
      ex: [
        MC('Which job fits a service best?', ['Reset the robot\'s odometry once', 'Stream camera images at 30 Hz', 'Drive to a goal 50 m away with progress updates', 'Publish joint states continuously'], 0),
        ORDER('Build the AddTwoInts.srv file.', ['int64 a', 'int64 b', '---', 'int64 sum']),
        MC('In a `.srv` file, what does `---` separate?', ['The request from the response', 'Two different services', 'Comments from code', 'Python from C++'], 0),
        MATCH('Match each communication style.', [['Topic', 'Continuous one-way stream'], ['Service', 'Request and response'], ['Action', 'Long goal with feedback'], ['Parameter', 'A node\'s setting']]),
        TF('You should run exactly one server for each service name.', true),
        TF('After one request, a service client keeps receiving responses forever.', false, 'One request, one response.'),
      ],
    },
    {
      id: 'u5l2', title: 'Server & client in Python', icon: 'code',
      story: [
        { a: 'code', lang: 'py', code: CODE.srvPy, focus: [6, 7], h: 'Create the server', t: '`create_service(type, name, callback)` registers the service. Every incoming request calls `handle()`.' },
        { a: 'code', lang: 'py', code: CODE.srvPy, focus: [9, 12], h: 'Fill in and return the response', t: 'The callback receives the **request** and an empty **response**. Fill the response fields and `return response`.', note: 'forgetting `return response` is the classic bug' },
        { a: 'code', lang: 'py', code: CODE.cliPy, focus: [4, 6], h: 'Clients wait for the server', t: '`create_client` makes a client. `wait_for_service(timeout_sec=1.0)` returns `False` until the server is up, so we loop politely.' },
        { a: 'service', mode: 'future', h: 'call_async returns a Future', t: '`call_async(req)` returns immediately with a **Future**, a promise of a result. When the response arrives, `on_done` runs and `future.result()` holds the answer.', req: 'a: 2, b: 3', res: 'sum: 5' },
        { a: 'cards', layout: 'grid2', h: 'Avoid deadlocks', t: 'Never block waiting for a response *inside* another callback on a single-threaded executor: the response can\'t be processed while you\'re blocking. Use `call_async` with a done callback instead.',
          items: [{ icon: 'x', title: 'Inside a callback', text: 'Blocking for a reply freezes the node: the reply can never be handled.', k: 'var(--bad)' }, { icon: 'check', title: 'Do this instead', text: 'call_async + add_done_callback, then return.', k: 'var(--good)' }] },
      ],
      ex: [
        FILL('Register the service.', "self.srv = self.___(AddTwoInts, 'add_two_ints', ___)", ['create_service', 'self.handle'], ['create_client', 'self.handle()', 'call_async']),
        MC('What must a Python service callback return?', ['The filled-in response object', 'Nothing', 'The request', 'True or False'], 0),
        BUG('The server crashes on every request. Tap the bug.', `def handle(self, request, response):
    response.sum = request.a + request.b
    return request`, 2, 'It returns the request instead of the response.'),
        MC('What does `call_async()` return?', ['A Future that will hold the response', 'The response itself', 'An error code', 'Nothing'], 0),
        ORDER('Order the client steps.', ['create_client(...)', 'wait_for_service()', 'Build a Request', 'call_async(req)', 'Read future.result()']),
        MC('The client sends a = 7 and b = 5. What gets logged?', ['`Sum: 12`', '`Sum: 75`', '`Sum: 2`', '`Sum: a + b`'], 0, null, { code: "self.get_logger().info(f'Sum: {future.result().sum}')", lang: 'py' }),
      ],
    },
  ],
},
/* ---------------------------------------------------------------- UNIT 6 */
{
  id: 'u6', title: 'Services in C++', sub: 'Servers, clients and futures with rclcpp', color: '#1E9B57',
  notes: [
    ['Service header', 'Snake_case header for each service type.', '#include "example_interfaces/srv/add_two_ints.hpp"'],
    ['C++ server callback', 'Receives shared pointers; returns void; fill the response.', 'res->sum = req->a + req->b;'],
    ['C++ request', 'Requests are shared pointers too.', 'auto req = std::make_shared<AddTwoInts::Request>();'],
    ['async_send_request', 'Sends without blocking; the callback gets a SharedFuture.', 'client->async_send_request(req, callback);'],
    ['Blocking in a script', 'Only in a simple standalone client.', 'rclcpp::spin_until_future_complete(node, future);'],
  ],
  lessons: [
    {
      id: 'u6l1', title: 'A C++ service server', icon: 'bell',
      story: [
        { a: 'code', lang: 'cpp', code: CODE.srvCpp, focus: [2, 4], h: 'Include and alias', t: 'The service header is `example_interfaces/srv/add_two_ints.hpp`. A `using` alias keeps the long type name readable.' },
        { a: 'code', lang: 'cpp', code: CODE.srvCpp, focus: [9, 11], h: 'The callback gets two pointers', t: 'The callback receives **shared pointers** to the request and the response, so fields are read with `->`: `req->a`.' },
        { a: 'code', lang: 'cpp', code: CODE.srvCpp, focus: [12, 12], h: 'No return value', t: 'Unlike Python, the C++ callback returns `void`. You answer by filling `res->sum`, and ROS sends it back for you.', note: 'fill res, don\'t return it' },
        { a: 'terminal', h: 'Test it from the CLI', t: 'You can call any service from the terminal, which is perfect for testing a server before writing a client.',
          panes: [{ title: 'server', lines: [{ c: 'ros2 run my_cpp_pkg adder' }, { wait: 2600 }, { o: '[INFO] [1730000004.210] [adder]: 2 + 3' }] }, { title: 'client', lines: [{ wait: 900 }, { c: 'ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"' }, { o: 'requester: making request: example_interfaces.srv.AddTwoInts_Request(a=2, b=3)\n\nresponse:\nexample_interfaces.srv.AddTwoInts_Response(sum=5)' }] }] },
      ],
      ex: [
        MC('In a C++ service callback, how do you send the answer back?', ['Fill the response: `res->sum = ...`', '`return res;`', 'Publish it on a topic', 'Call `send_response()`'], 0),
        BUG('Which line will not compile?', `[this](const std::shared_ptr<AddTwoInts::Request> req,
       std::shared_ptr<AddTwoInts::Response> res) {
  res.sum = req->a + req->b;
}`, 2, '`res` is a shared pointer, so it needs `res->sum`.', 'cpp'),
        TF('In C++, the request and response arrive as shared pointers.', true),
        BUILD('Call /add_two_ints with a = 2 and b = 3.', ['ros2', 'service', 'call', '/add_two_ints', 'example_interfaces/srv/AddTwoInts', '"{a: 2, b: 3}"'], ['topic', 'pub']),
        FILL('Name the header.', '#include "example_interfaces/srv/___.hpp"', ['add_two_ints'], ['AddTwoInts', 'addTwoInts', 'add-two-ints'], 'C++ headers use snake_case versions of the type name.', 'cpp'),
      ],
    },
    {
      id: 'u6l2', title: 'A C++ client & futures', icon: 'ticket',
      story: [
        { a: 'code', lang: 'cpp', code: CODE.cliCpp, focus: [1, 4], h: 'Create and wait', t: '`create_client<AddTwoInts>()` returns a client. `wait_for_service(1s)` waits up to one second for the server to appear.' },
        { a: 'code', lang: 'cpp', code: CODE.cliCpp, focus: [5, 7], h: 'Requests are shared pointers too', t: 'Build the request with `std::make_shared<AddTwoInts::Request>()` and fill its fields.' },
        { a: 'service', mode: 'future', h: 'async_send_request', t: '`async_send_request(req, callback)` sends without blocking. When the response arrives, the callback receives a **SharedFuture**, and `future.get()` returns the response pointer.', req: 'a: 2, b: 3', res: 'sum: 5' },
        { a: 'cards', layout: 'grid2', h: 'Two ways to wait', t: 'In a tiny command-line client you can block with `rclcpp::spin_until_future_complete(node, future)`. Inside a running node, prefer the callback version so other callbacks keep running.',
          items: [{ icon: 'terminal', title: 'Standalone script', text: 'Block until done.', code: 'rclcpp::spin_until_future_complete(node, future);', k: 'var(--c-srv)' }, { icon: 'node', title: 'Inside a node', text: 'Use a response callback.', code: 'client->async_send_request(req, cb);', k: 'var(--good)' }] },
      ],
      ex: [
        FILL('Send the request.', 'client->___(req, callback);', ['async_send_request'], ['call_async', 'publish', 'send'], null, 'cpp'),
        MC('What does `future.get()` return inside the callback?', ['A shared pointer to the response', 'The request', 'A bool', 'The service name'], 0),
        MATCH('Match Python to C++.', [['create_service(...)', 'create_service<T>(...)'], ['call_async(req)', 'async_send_request(req, cb)'], ['future.result()', 'future.get()'], ['wait_for_service(timeout_sec=1.0)', 'wait_for_service(1s)']]),
        MC('Why use a response callback instead of blocking inside a node?', ['So the node keeps processing other callbacks', 'Callbacks use less memory', 'Blocking is illegal in C++', 'It makes the server faster'], 0),
        TF('Requests in C++ are usually created with `std::make_shared`.', true),
      ],
    },
  ],
},
);
