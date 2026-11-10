# rclpy (lab edition): a compact subset of the ROS 2 Python client library.
# It mirrors the real API shapes so your code transfers to a real robot, and it
# talks to the Robot Lab and SnoBots simulators through a message bridge.
import sys
import types
import array
import json
import math
import time
import traceback
import linecache
import labbridge

_state = {'inited': False, 'nodes': [], 'spinning': [], 'subs': {}, 'running': False, 'unwinding': False, 'sim_t': None}


def _now():
    # simulated seconds when a simulator drives the clock (use_sim_time), otherwise the wall clock
    t = _state['sim_t']
    return time.monotonic() if t is None else t


class _Spin(BaseException):
    """Raised by rclpy.spin(): the lab keeps the node alive and runs its callbacks."""


def _clean(v):
    if v is None or isinstance(v, (bool, str)):
        return v
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, _Msg):
        return _clean(_to_dict(v))
    return str(v)


def _post(**kw):
    labbridge.post(json.dumps(_clean(kw)))


def _post_bin(blob, **kw):
    # images: the bytes travel next to the JSON (as a typed array), not inside it
    from pyodide.ffi import to_js
    labbridge.post_bin(json.dumps(_clean(kw)), to_js(memoryview(blob)))


# ------------------------------------------------------------------ messages
class _Msg:
    _fields = ()
    _names = ()
    _kinds = {}
    _type = ''

    def __init__(self, **kw):
        for n, fac in self._fields:
            object.__setattr__(self, n, fac())
        for k, v in kw.items():
            setattr(self, k, v)

    def __setattr__(self, n, v):
        if n not in self._names:
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{n}'")
        kind = self._kinds.get(n)
        if kind is float:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise AssertionError(f"The '{n}' field must be of type 'float'")
            if isinstance(v, int):
                raise AssertionError(f"The '{n}' field must be of type 'float' (tip: write {v}.0 instead of {v})")
        elif kind is int:
            if isinstance(v, bool) or not isinstance(v, int):
                raise AssertionError(f"The '{n}' field must be of type 'int'")
        elif kind is bool:
            if not isinstance(v, bool):
                raise AssertionError(f"The '{n}' field must be of type 'bool'")
        elif kind is str:
            if not isinstance(v, str):
                raise AssertionError(f"The '{n}' field must be of type 'str'")
        elif isinstance(kind, type) and issubclass(kind, _Msg) and not isinstance(v, kind):
            raise AssertionError(f"The '{n}' field must be a sub message of type '{kind.__name__}'")
        elif kind is _bytes:
            v = _as_bytes(n, v)
        object.__setattr__(self, n, v)

    def __repr__(self):
        pkg, sub, nm = self._type.split('/')
        return f"{pkg}.{sub}.{nm}(" + ', '.join(f'{n}={getattr(self, n)!r}' for n in self._names) + ')'

    def __eq__(self, other):
        return type(other) is type(self) and all(getattr(self, n) == getattr(other, n) for n in self._names)


def _one():
    return 1.0


def _bytes():
    # a uint8[] field, stored like rclpy does: array.array('B')
    return array.array('B')


def _as_bytes(n, v):
    if isinstance(v, array.array) and v.typecode == 'B':
        return v
    if isinstance(v, (bytes, bytearray, memoryview)):
        return array.array('B', bytes(v))
    if hasattr(v, 'dtype') and hasattr(v, 'tobytes'):
        if str(v.dtype) != 'uint8':
            raise AssertionError(f"The '{n}' field must hold uint8 values, got a {v.dtype} array (tip: .astype(np.uint8), or use cv_bridge)")
        return array.array('B', v.tobytes())
    if isinstance(v, (list, tuple)):
        try:
            return array.array('B', v)
        except (OverflowError, TypeError):
            raise AssertionError(f"The '{n}' field must be a sequence of integers in [0, 255]")
    raise AssertionError(f"The '{n}' field must be bytes, a bytearray or an array of uint8")


def _z36():
    return [0.0] * 36


def _z9():
    return [0.0] * 9


def _z12():
    return [0.0] * 12


class _ListOf:
    """Field factory for a list of sub-messages (optionally pre-filled, like a fixed-size array)."""

    def __init__(self, cls, n=0):
        self.cls = cls
        self.n = n

    def __call__(self):
        return [self.cls() for _ in range(self.n)]


def _mk(pkg, name, fields):
    names = tuple(f[0] for f in fields)
    kinds = {}
    for n, fac in fields:
        if fac in (float, int, bool, str) or (isinstance(fac, type) and issubclass(fac, _Msg)):
            kinds[n] = fac
        elif fac is _one:
            kinds[n] = float
        elif fac is _bytes:
            kinds[n] = _bytes
    return type(name, (_Msg,), {'_fields': tuple(fields), '_names': names, '_kinds': kinds,
                                '_type': f'{pkg}/{name}', '__module__': pkg.replace('/', '.')})


def _to_dict(m):
    if isinstance(m, _Msg):
        return {n: _to_dict(getattr(m, n)) for n in m._names}
    if isinstance(m, (list, tuple)):
        return [_to_dict(x) for x in m]
    return m


def _from_dict(cls, d):
    m = cls()
    if not isinstance(d, dict):
        return m
    for n, fac in cls._fields:
        if n not in d:
            continue
        cur, v = getattr(m, n), d[n]
        if isinstance(fac, _ListOf) and isinstance(v, list):
            object.__setattr__(m, n, [_from_dict(fac.cls, x) for x in v])
            continue
        if isinstance(cur, _Msg):
            v = _from_dict(type(cur), v)
        elif isinstance(v, list):
            v = [float('inf') if x is None else x for x in v]
        elif isinstance(cur, float):
            v = float('inf') if v is None else float(v)
        object.__setattr__(m, n, v)
    return m


F, I, S, B = float, int, str, bool
_TimeMsg = _mk('builtin_interfaces/msg', 'Time', [('sec', I), ('nanosec', I)])
_DurationMsg = _mk('builtin_interfaces/msg', 'Duration', [('sec', I), ('nanosec', I)])
Header = _mk('std_msgs/msg', 'Header', [('stamp', _TimeMsg), ('frame_id', S)])
String = _mk('std_msgs/msg', 'String', [('data', S)])
Bool = _mk('std_msgs/msg', 'Bool', [('data', B)])
Int32 = _mk('std_msgs/msg', 'Int32', [('data', I)])
Int64 = _mk('std_msgs/msg', 'Int64', [('data', I)])
Float32 = _mk('std_msgs/msg', 'Float32', [('data', F)])
Float64 = _mk('std_msgs/msg', 'Float64', [('data', F)])
Empty = _mk('std_msgs/msg', 'Empty', [])
Vector3 = _mk('geometry_msgs/msg', 'Vector3', [('x', F), ('y', F), ('z', F)])
Point = _mk('geometry_msgs/msg', 'Point', [('x', F), ('y', F), ('z', F)])
Quaternion = _mk('geometry_msgs/msg', 'Quaternion', [('x', F), ('y', F), ('z', F), ('w', _one)])
Pose = _mk('geometry_msgs/msg', 'Pose', [('position', Point), ('orientation', Quaternion)])
PoseStamped = _mk('geometry_msgs/msg', 'PoseStamped', [('header', Header), ('pose', Pose)])
Twist = _mk('geometry_msgs/msg', 'Twist', [('linear', Vector3), ('angular', Vector3)])
PoseWithCovariance = _mk('geometry_msgs/msg', 'PoseWithCovariance', [('pose', Pose), ('covariance', _z36)])
TwistWithCovariance = _mk('geometry_msgs/msg', 'TwistWithCovariance', [('twist', Twist), ('covariance', _z36)])
Odometry = _mk('nav_msgs/msg', 'Odometry', [('header', Header), ('child_frame_id', S), ('pose', PoseWithCovariance), ('twist', TwistWithCovariance)])
LaserScan = _mk('sensor_msgs/msg', 'LaserScan', [('header', Header), ('angle_min', F), ('angle_max', F), ('angle_increment', F),
                                                 ('time_increment', F), ('scan_time', F), ('range_min', F), ('range_max', F), ('ranges', list), ('intensities', list)])
SetParametersResult = _mk('rcl_interfaces/msg', 'SetParametersResult', [('successful', B), ('reason', S)])
JointState = _mk('sensor_msgs/msg', 'JointState', [('header', Header), ('name', list), ('position', list), ('velocity', list), ('effort', list)])
RegionOfInterest = _mk('sensor_msgs/msg', 'RegionOfInterest', [('x_offset', I), ('y_offset', I), ('height', I), ('width', I), ('do_rectify', B)])
Image = _mk('sensor_msgs/msg', 'Image', [('header', Header), ('height', I), ('width', I), ('encoding', S), ('is_bigendian', I), ('step', I), ('data', _bytes)])
CameraInfo = _mk('sensor_msgs/msg', 'CameraInfo', [('header', Header), ('height', I), ('width', I), ('distortion_model', S), ('d', list), ('k', _z9), ('r', _z9), ('p', _z12),
                                                   ('binning_x', I), ('binning_y', I), ('roi', RegionOfInterest)])
Imu = _mk('sensor_msgs/msg', 'Imu', [('header', Header), ('orientation', Quaternion), ('orientation_covariance', _z9), ('angular_velocity', Vector3),
                                     ('angular_velocity_covariance', _z9), ('linear_acceleration', Vector3), ('linear_acceleration_covariance', _z9)])
Path = _mk('nav_msgs/msg', 'Path', [('header', Header), ('poses', _ListOf(PoseStamped))])
MapMetaData = _mk('nav_msgs/msg', 'MapMetaData', [('map_load_time', _TimeMsg), ('resolution', F), ('width', I), ('height', I), ('origin', Pose)])
OccupancyGrid = _mk('nav_msgs/msg', 'OccupancyGrid', [('header', Header), ('info', MapMetaData), ('data', list)])
LimoStatus = _mk('limo_msgs/msg', 'LimoStatus', [('header', Header), ('vehicle_state', I), ('control_mode', I), ('battery_voltage', F), ('error_code', I), ('motion_mode', I)])
ColorDetection = _mk('snobots_msgs/msg', 'ColorDetection', [('header', Header), ('color', S), ('found', B), ('x', F), ('y', F), ('area', F)])
MotorCmd = _mk('snobots_msgs/msg', 'MotorCmd', [('mode', I), ('q', F), ('dq', F), ('tau', F), ('kp', F), ('kd', F)])
LowCmd = _mk('snobots_msgs/msg', 'LowCmd', [('motor_cmd', _ListOf(MotorCmd, 26))])
MotorState = _mk('snobots_msgs/msg', 'MotorState', [('q', F), ('dq', F), ('tau_est', F), ('temperature', I)])
LowState = _mk('snobots_msgs/msg', 'LowState', [('tick', I), ('motor_state', _ListOf(MotorState))])
ArmAngles = _mk('snobots_msgs/msg', 'ArmAngles', [('angles', list), ('speed', I)])
ArmCoords = _mk('snobots_msgs/msg', 'ArmCoords', [('x', F), ('y', F), ('z', F), ('rx', F), ('ry', F), ('rz', F), ('speed', I)])


class ParameterDescriptor:
    def __init__(self, **kw):
        self.__dict__.update(kw)


# ------------------------------------------------------------------ time, logging, parameters
class Duration:
    def __init__(self, *, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        return _DurationMsg(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class Time:
    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=None):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        return _TimeMsg(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)

    def seconds_nanoseconds(self):
        return (self.nanoseconds // 10**9, self.nanoseconds % 10**9)

    def __sub__(self, other):
        if isinstance(other, Time):
            return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)
        return Time(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __add__(self, other):
        return Time(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __lt__(self, o):
        return self.nanoseconds < o.nanoseconds

    def __le__(self, o):
        return self.nanoseconds <= o.nanoseconds

    def __gt__(self, o):
        return self.nanoseconds > o.nanoseconds

    def __ge__(self, o):
        return self.nanoseconds >= o.nanoseconds


class Clock:
    def now(self):
        t = _state['sim_t']
        return Time(nanoseconds=int((time.time() if t is None else t) * 1e9))


class RcutilsLogger:
    def __init__(self, name):
        self.name = name
        self._seen = {}

    def _log(self, level, msg, throttle_duration_sec=None, once=False, skip_first=False, **kw):
        f = sys._getframe(2)
        key = (level, f.f_code.co_filename, f.f_lineno)
        now = time.monotonic()
        last = self._seen.get(key)
        if once and last is not None:
            return False
        if skip_first and last is None:
            self._seen[key] = now
            return False
        if throttle_duration_sec and last is not None and now - last < throttle_duration_sec:
            return False
        self._seen[key] = now
        _post(type='log', level=level, node=self.name, text=str(msg), t=time.time())
        return True

    def debug(self, msg, **kw):
        return False

    def info(self, msg, **kw):
        return self._log('INFO', msg, **kw)

    def warn(self, msg, **kw):
        return self._log('WARN', msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        return self._log('ERROR', msg, **kw)

    def fatal(self, msg, **kw):
        return self._log('FATAL', msg, **kw)


class ParameterNotDeclaredException(Exception):
    pass


class ParameterAlreadyDeclaredException(Exception):
    pass


class Parameter:
    class Type:
        NOT_SET, BOOL, INTEGER, DOUBLE, STRING, BYTE_ARRAY, BOOL_ARRAY, INTEGER_ARRAY, DOUBLE_ARRAY, STRING_ARRAY = range(10)

    def __init__(self, name, type_=None, value=None):
        self.name = name
        self.type_ = type_
        self.value = value

    def __repr__(self):
        return f'Parameter(name={self.name!r}, value={self.value!r})'


def _ptype(v):
    if v is None:
        return 'not_set'
    if isinstance(v, bool):
        return 'bool'
    if isinstance(v, int):
        return 'integer'
    if isinstance(v, float):
        return 'double'
    if isinstance(v, str):
        return 'string'
    if isinstance(v, (list, tuple)):
        return 'array'
    return 'other'


# ------------------------------------------------------------------ QoS
class QoSProfile:
    def __init__(self, **kw):
        self.depth = kw.get('depth', 10)
        self.__dict__.update(kw)


class ReliabilityPolicy:
    SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT = 0, 1, 2


class HistoryPolicy:
    SYSTEM_DEFAULT, KEEP_LAST, KEEP_ALL = 0, 1, 2


class DurabilityPolicy:
    SYSTEM_DEFAULT, TRANSIENT_LOCAL, VOLATILE = 0, 1, 2


qos_profile_sensor_data = QoSProfile(depth=5)
qos_profile_system_default = QoSProfile(depth=10)


def _check_qos(q):
    if isinstance(q, bool) or not (isinstance(q, (int, QoSProfile))):
        raise TypeError('qos_profile must be a queue depth (an int such as 10) or a QoSProfile')


def _check_msg_type(t, fn):
    if isinstance(t, type) and issubclass(t, _Msg):
        return
    if isinstance(t, _Msg):
        raise TypeError(f'{fn}() needs the message class, e.g. Twist, not an instance like Twist()')
    raise TypeError(f'{fn}() expected a message type such as Twist, got {t!r}')


# ------------------------------------------------------------------ node
class Publisher:
    def __init__(self, node, msg_type, topic):
        self.node = node
        self.msg_type = msg_type
        self.topic_name = topic

    @property
    def topic(self):
        return self.topic_name

    def publish(self, msg):
        if not self.node._alive:
            raise RuntimeError('cannot publish: this node was destroyed')
        if not isinstance(msg, self.msg_type):
            hint = ' (you passed the class itself: create a message first, e.g. msg = ' + self.msg_type.__name__ + '())' if msg is self.msg_type else ''
            raise TypeError(f'Expected {self.msg_type.__name__}, got {type(msg).__name__}{hint}')
        d = _to_dict(msg)
        blob = None
        for k, v in d.items():
            if isinstance(v, array.array):
                blob, d[k] = v, None
                break
        if blob is not None:
            _post_bin(blob, type='pub', node=self.node._fqn(), topic=self.topic_name, msgType=self.msg_type._type, data=d)
        else:
            _post(type='pub', node=self.node._fqn(), topic=self.topic_name, msgType=self.msg_type._type, data=d)

    def get_subscription_count(self):
        return 1

    def destroy(self):
        return True


class Subscription:
    def __init__(self, node, msg_type, topic, callback):
        self.node = node
        self.msg_type = msg_type
        self.topic_name = topic
        self.callback = callback
        self._destroyed = False

    def destroy(self):
        self._destroyed = True
        return True


class Timer:
    def __init__(self, period, callback):
        self._period = period
        self.timer_period_ns = int(period * 1e9)
        self.callback = callback
        self._next = _now() + period
        self._canceled = False

    def cancel(self):
        self._canceled = True

    def reset(self):
        self._canceled = False
        self._next = _now() + self._period

    def is_canceled(self):
        return self._canceled

    def destroy(self):
        self.cancel()


class Node:
    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None, use_global_arguments=True,
                 enable_rosout=True, start_parameter_services=True, parameter_overrides=None,
                 allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False, **kw):
        if not _state['inited']:
            raise RuntimeError('rclpy.init() has not been called: call rclpy.init() before creating a Node')
        if not isinstance(node_name, str) or not node_name or node_name[0].isdigit() or not all(c.isalnum() or c == '_' for c in node_name):
            raise ValueError(f"Invalid node name '{node_name}': use letters, digits and underscores, and don't start with a digit")
        self._name = node_name
        self._ns = ('/' + namespace.strip('/')) if namespace and namespace.strip('/') else ''
        self._params = {'use_sim_time': _state['sim_t'] is not None}
        self._param_cbs = []
        self._pubs, self._subs, self._timers = [], [], []
        self._alive = True
        self._logger = RcutilsLogger(node_name)
        _state['nodes'].append(self)
        _post(type='node', name=self._fqn())
        _post(type='param_declare', node=self._fqn(), name='use_sim_time', value=_state['sim_t'] is not None, ptype='bool')

    def _fqn(self):
        return self._ns + '/' + self._name

    def get_name(self):
        return self._name

    def get_namespace(self):
        return self._ns or '/'

    def get_fully_qualified_name(self):
        return self._fqn()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock()

    def _resolve(self, topic):
        if not isinstance(topic, str) or not topic:
            raise ValueError('topic name must be a non-empty string')
        if topic.startswith('~/'):
            return self._fqn() + '/' + topic[2:]
        if topic.startswith('/'):
            return topic
        return self._ns + '/' + topic

    def create_publisher(self, msg_type, topic, qos_profile=10, **kw):
        _check_msg_type(msg_type, 'create_publisher')
        _check_qos(qos_profile)
        p = Publisher(self, msg_type, self._resolve(topic))
        self._pubs.append(p)
        _post(type='pub_create', node=self._fqn(), topic=p.topic_name, msgType=msg_type._type)
        return p

    def create_subscription(self, msg_type, topic, callback, qos_profile=10, **kw):
        _check_msg_type(msg_type, 'create_subscription')
        if not callable(callback):
            raise TypeError('callback must be callable: pass the method itself, e.g. self.on_msg (no parentheses)')
        _check_qos(qos_profile)
        s = Subscription(self, msg_type, self._resolve(topic), callback)
        self._subs.append(s)
        _state['subs'].setdefault(s.topic_name, []).append(s)
        _post(type='sub_create', node=self._fqn(), topic=s.topic_name, msgType=msg_type._type)
        return s

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None, **kw):
        if not callable(callback):
            raise TypeError('timer callback must be callable: pass self.tick (no parentheses)')
        if isinstance(timer_period_sec, bool) or not isinstance(timer_period_sec, (int, float)) or timer_period_sec <= 0:
            raise ValueError('timer period must be a positive number of seconds')
        t = Timer(float(timer_period_sec), callback)
        self._timers.append(t)
        return t

    def create_rate(self, *a, **k):
        raise NotImplementedError('create_rate() is not supported in the Robot Lab: use create_timer() instead')

    def create_service(self, *a, **k):
        raise NotImplementedError("Services aren't available to Python nodes in the Robot Lab yet. Call the lab's services from the Terminal tab with ros2 service call.")

    create_client = create_service

    def destroy_timer(self, timer):
        timer.cancel()
        return True

    def destroy_subscription(self, sub):
        sub._destroyed = True
        return True

    def destroy_publisher(self, pub):
        return True

    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        if not isinstance(name, str) or not name:
            raise ValueError('parameter name must be a non-empty string')
        if name in self._params:
            raise ParameterAlreadyDeclaredException(f"Parameter '{name}' has already been declared")
        self._params[name] = value
        _post(type='param_declare', node=self._fqn(), name=name, value=value, ptype=_ptype(value))
        return Parameter(name, value=value)

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        out = []
        for item in parameters:
            if isinstance(item, (tuple, list)):
                nm, val = item[0], (item[1] if len(item) > 1 else None)
            else:
                nm, val = item, None
            out.append(self.declare_parameter(f'{namespace}.{nm}' if namespace else nm, val))
        return out

    def has_parameter(self, name):
        return name in self._params

    def get_parameter(self, name):
        if name not in self._params:
            raise ParameterNotDeclaredException(f"Parameter '{name}' has not been declared: call self.declare_parameter('{name}', default) first")
        return Parameter(name, value=self._params[name])

    def get_parameter_or(self, name, alternative_value=None):
        if name in self._params:
            return self.get_parameter(name)
        return alternative_value if isinstance(alternative_value, Parameter) else Parameter(name, value=alternative_value)

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def set_parameters(self, parameter_list):
        return [self._set_one(p) for p in parameter_list]

    def add_on_set_parameters_callback(self, callback):
        self._param_cbs.append(callback)

    def remove_on_set_parameters_callback(self, callback):
        if callback in self._param_cbs:
            self._param_cbs.remove(callback)

    def _set_one(self, p):
        if p.name not in self._params:
            return SetParametersResult(successful=False, reason=f"parameter '{p.name}' is not declared")
        old = self._params[p.name]
        if old is not None and _ptype(old) != _ptype(p.value):
            return SetParametersResult(successful=False, reason=f"Wrong parameter type, expected 'Type.{_ptype(old).upper()}' got 'Type.{_ptype(p.value).upper()}'")
        for cb in list(self._param_cbs):
            r = cb([p])
            if r is not None and not getattr(r, 'successful', True):
                return SetParametersResult(successful=False, reason=getattr(r, 'reason', '') or 'rejected by your parameter callback')
        self._params[p.name] = p.value
        _post(type='param_changed', node=self._fqn(), name=p.name, value=p.value, ptype=_ptype(p.value))
        return SetParametersResult(successful=True)

    def destroy_node(self):
        if _state['unwinding']:
            return True
        self._destroy()
        return True

    def _destroy(self):
        if not self._alive:
            return
        self._alive = False
        for t in self._timers:
            t.cancel()
        for s in self._subs:
            s._destroyed = True
            lst = _state['subs'].get(s.topic_name, [])
            if s in lst:
                lst.remove(s)
        if self in _state['spinning']:
            _state['spinning'].remove(self)
        if self in _state['nodes']:
            _state['nodes'].remove(self)
        _post(type='node_gone', name=self._fqn())


# ------------------------------------------------------------------ rclpy top level
class _InitCtx:
    def __enter__(self):
        return self

    def __exit__(self, et, ev, tb):
        if et is not None and issubclass(et, _Spin):
            return False
        shutdown()
        return False


def init(args=None, *, context=None, domain_id=None, signal_handler_options=None):
    _state['inited'] = True
    return _InitCtx()


def ok(context=None):
    return _state['inited'] and _state['running']


def shutdown(context=None, **kw):
    if _state['unwinding']:
        return
    _lab_stop()


try_shutdown = shutdown


def spin(node, executor=None):
    if not _state['inited']:
        raise RuntimeError('rclpy.init() must be called before rclpy.spin()')
    if not isinstance(node, Node):
        raise TypeError('rclpy.spin() expects a Node, e.g. rclpy.spin(node)')
    if node not in _state['spinning']:
        _state['spinning'].append(node)
    _state['unwinding'] = True
    raise _Spin()


def _no_loop(*a, **k):
    raise RuntimeError('spin_once() and spin_until_future_complete() loops are not supported in the Robot Lab: '
                       'put your logic in timers and subscription callbacks, then call rclpy.spin(node).')


spin_once = _no_loop
spin_until_future_complete = _no_loop


def create_node(node_name, **kw):
    return Node(node_name, **kw)


class ExternalShutdownException(Exception):
    pass


class SingleThreadedExecutor:
    def __init__(self, *a, **k):
        self._nodes = []

    def add_node(self, node):
        if node not in self._nodes:
            self._nodes.append(node)
        return True

    def remove_node(self, node):
        if node in self._nodes:
            self._nodes.remove(node)

    def spin(self):
        for n in self._nodes:
            if n not in _state['spinning']:
                _state['spinning'].append(n)
        _state['unwinding'] = True
        raise _Spin()

    spin_once = _no_loop
    spin_until_future_complete = _no_loop

    def shutdown(self, timeout_sec=None):
        return True


class MultiThreadedExecutor(SingleThreadedExecutor):
    pass


def _module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    sys.modules[name] = mod
    parent, _, child = name.rpartition('.')
    if parent and parent in sys.modules:
        setattr(sys.modules[parent], child, mod)
    return mod


_module('builtin_interfaces')
_module('builtin_interfaces.msg', Time=_TimeMsg, Duration=_DurationMsg)
_module('std_msgs')
_module('std_msgs.msg', Header=Header, String=String, Bool=Bool, Int32=Int32, Int64=Int64, Float32=Float32, Float64=Float64, Empty=Empty)
_module('geometry_msgs')
_module('geometry_msgs.msg', Vector3=Vector3, Point=Point, Quaternion=Quaternion, Pose=Pose, PoseStamped=PoseStamped, Twist=Twist,
        PoseWithCovariance=PoseWithCovariance, TwistWithCovariance=TwistWithCovariance)
_module('nav_msgs')
_module('nav_msgs.msg', Odometry=Odometry, Path=Path, MapMetaData=MapMetaData, OccupancyGrid=OccupancyGrid)
_module('sensor_msgs')
_module('sensor_msgs.msg', LaserScan=LaserScan, JointState=JointState, Imu=Imu, Image=Image, CameraInfo=CameraInfo, RegionOfInterest=RegionOfInterest)
_module('limo_msgs')
_module('limo_msgs.msg', LimoStatus=LimoStatus)
_module('snobots_msgs')
_module('snobots_msgs.msg', ColorDetection=ColorDetection, MotorCmd=MotorCmd, LowCmd=LowCmd, MotorState=MotorState, LowState=LowState, ArmAngles=ArmAngles, ArmCoords=ArmCoords)
_module('rcl_interfaces')
_module('rcl_interfaces.msg', SetParametersResult=SetParametersResult, ParameterDescriptor=ParameterDescriptor)
_rclpy = _module('rclpy', init=init, ok=ok, shutdown=shutdown, try_shutdown=try_shutdown, spin=spin, spin_once=spin_once,
                 spin_until_future_complete=spin_until_future_complete, create_node=create_node, __version__='lab-jazzy')
_module('rclpy.node', Node=Node)
_module('rclpy.qos', QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy, HistoryPolicy=HistoryPolicy, DurabilityPolicy=DurabilityPolicy,
        qos_profile_sensor_data=qos_profile_sensor_data, qos_profile_system_default=qos_profile_system_default)
_module('rclpy.time', Time=Time)
_module('rclpy.duration', Duration=Duration)
_module('rclpy.clock', Clock=Clock)
_module('rclpy.parameter', Parameter=Parameter)
_module('rclpy.exceptions', ParameterNotDeclaredException=ParameterNotDeclaredException, ParameterAlreadyDeclaredException=ParameterAlreadyDeclaredException)
_module('rclpy.executors', SingleThreadedExecutor=SingleThreadedExecutor, MultiThreadedExecutor=MultiThreadedExecutor, ExternalShutdownException=ExternalShutdownException)
_module('rclpy.logging', get_logger=lambda name: RcutilsLogger(name))
_module('rclpy.timer', Timer=Timer)
_module('rclpy.publisher', Publisher=Publisher)
_module('rclpy.subscription', Subscription=Subscription)


# ------------------------------------------------------------------ cv_bridge (lab edition): sensor_msgs/Image <-> numpy arrays
class CvBridgeError(TypeError):
    """Raised when an image cannot be converted, as in the real cv_bridge."""


_ENC = {'rgb8': (3, 'uint8'), 'bgr8': (3, 'uint8'), 'rgba8': (4, 'uint8'), 'bgra8': (4, 'uint8'), 'mono8': (1, 'uint8'), 'mono16': (1, 'uint16'),
        '8UC1': (1, 'uint8'), '8UC3': (3, 'uint8'), '8UC4': (4, 'uint8'), '16UC1': (1, 'uint16'), '32FC1': (1, 'float32')}
_ALIAS = {'8UC1': 'mono8', '8UC3': 'bgr8', '8UC4': 'bgra8', '16UC1': 'mono16'}


def _cv_convert(im, src, dst):
    import numpy as np
    if dst == 'passthrough' or src == dst:
        return im
    s, d = _ALIAS.get(src, src), _ALIAS.get(dst, dst)
    if dst not in _ENC:
        raise CvBridgeError(f'Unrecognized image encoding [{dst}]')
    if s == d:
        return im
    if s in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        rgb, alpha = (im[..., 2::-1] if s.startswith('bgr') else im[..., :3]), (im[..., 3] if s.endswith('a8') else None)
    elif s == 'mono8':
        rgb, alpha = np.repeat(im.reshape(im.shape[0], im.shape[1], 1), 3, axis=2), None
    else:
        raise CvBridgeError(f'[{src}] is not a color format. but [{dst}] is. The conversion does not make sense')
    if d == 'mono8':
        return np.clip(np.rint(rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114), 0, 255).astype(np.uint8)
    if d in ('rgb8', 'bgr8'):
        return np.ascontiguousarray(rgb if d == 'rgb8' else rgb[..., ::-1])
    if d in ('rgba8', 'bgra8'):
        a = alpha if alpha is not None else np.full(rgb.shape[:2], 255, np.uint8)
        return np.ascontiguousarray(np.dstack([rgb if d == 'rgba8' else rgb[..., ::-1], a]))
    raise CvBridgeError(f'Unsupported conversion from [{src}] to [{dst}]')


class CvBridge:
    """Converts between sensor_msgs/Image and numpy arrays (OpenCV images), like cv_bridge.CvBridge."""

    def imgmsg_to_cv2(self, img_msg, desired_encoding='passthrough'):
        import numpy as np
        enc = img_msg.encoding
        if enc not in _ENC:
            raise CvBridgeError(f'Unrecognized image encoding [{enc}]')
        ch, dt = _ENC[enc]
        dtype = np.dtype(dt)
        h, w, step = int(img_msg.height), int(img_msg.width), int(img_msg.step)
        row = w * ch * dtype.itemsize
        buf = np.frombuffer(img_msg.data, dtype=np.uint8)
        if h <= 0 or w <= 0 or step < row or buf.size < step * (h - 1) + row:
            raise CvBridgeError(f'image data ({buf.size} bytes) does not match height {h}, width {w}, step {step} and encoding {enc}')
        rows = buf[:h * step].reshape(h, step)[:, :row]
        im = np.ascontiguousarray(rows).view(dtype).reshape((h, w) if ch == 1 else (h, w, ch))
        if img_msg.is_bigendian and dtype.itemsize > 1:
            im = im.byteswap()
        return _cv_convert(im, enc, desired_encoding)

    def cv2_to_imgmsg(self, cvim, encoding='passthrough', header=None):
        import numpy as np
        if not isinstance(cvim, np.ndarray):
            raise TypeError('Your input type is not a numpy array')
        if cvim.ndim not in (2, 3):
            raise CvBridgeError(f'an image must have 2 or 3 dimensions, got shape {cvim.shape}')
        ch = 1 if cvim.ndim == 2 else int(cvim.shape[2])
        cv_type = {('uint8', 1): '8UC1', ('uint8', 3): '8UC3', ('uint8', 4): '8UC4', ('uint16', 1): '16UC1', ('float32', 1): '32FC1'}.get((str(cvim.dtype), ch))
        if cv_type is None:
            raise CvBridgeError(f'unsupported image: {cvim.dtype} with {ch} channel(s)')
        if encoding == 'passthrough':
            encoding = cv_type
        elif encoding not in _ENC:
            raise CvBridgeError(f'Unrecognized image encoding [{encoding}]')
        elif _ENC[encoding] != (ch, str(cvim.dtype)):
            raise CvBridgeError(f'encoding specified as {encoding}, but image has incompatible type {cv_type}')
        img = np.ascontiguousarray(cvim)
        msg = Image()
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(img.strides[0])
        msg.data = img.tobytes()
        if header is not None:
            msg.header = header
        return msg


_module('cv_bridge', CvBridge=CvBridge, CvBridgeError=CvBridgeError)


# ------------------------------------------------------------------ OpenCV windows: cv2.imshow shows the image over the 3D view
_imshow_t = {}


def _lab_imshow(winname, mat):
    import numpy as np
    a = np.asarray(mat)
    if a.ndim == 3 and a.shape[2] == 1:
        a = a[..., 0]
    if a.size == 0 or a.ndim not in (2, 3) or (a.ndim == 3 and a.shape[2] not in (3, 4)):
        raise ValueError(f'cv2.imshow() cannot show an array of shape {a.shape}: use a 2-D (gray) or 3-channel BGR image')
    if a.dtype == np.bool_:
        a = a.astype(np.uint8) * 255
    elif a.dtype in (np.uint16, np.int16):
        a = np.clip(a.astype(np.int32) >> 8, 0, 255).astype(np.uint8)   # like OpenCV: 16-bit values are divided by 256
    elif np.issubdtype(a.dtype, np.floating):
        a = np.clip(a * 255.0, 0, 255).astype(np.uint8)                  # like OpenCV: floats are shown as 0..1
    elif a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    name = str(winname)
    now = time.monotonic()
    if now - _imshow_t.get(name, -1.0) < 1 / 15:
        return
    _imshow_t[name] = now
    h, w = a.shape[:2]
    _post_bin(np.ascontiguousarray(a).tobytes(), type='imshow', name=name, h=int(h), w=int(w), c=1 if a.ndim == 2 else int(a.shape[2]))


def _lab_patch_cv2():
    import cv2
    if getattr(cv2, '_lab_patched', False):
        return
    none = lambda *a, **k: None  # noqa: E731
    cv2.imshow = _lab_imshow
    cv2.waitKey = cv2.waitKeyEx = lambda delay=0: -1
    cv2.pollKey = lambda: -1
    cv2.namedWindow = cv2.resizeWindow = cv2.moveWindow = cv2.setWindowTitle = cv2.setMouseCallback = none
    cv2.destroyWindow = lambda winname: _post(type='imclose', name=str(winname))
    cv2.destroyAllWindows = lambda: _post(type='imclose', name='')
    cv2._lab_patched = True


# ------------------------------------------------------------------ bridge entry points (called from JS)
def _crash(e):
    if isinstance(e, SyntaxError):
        text = ''.join(traceback.format_exception_only(type(e), e)).rstrip()
        _post(type='crash', text=text, line=e.lineno)
    else:
        frames = [f for f in traceback.extract_tb(e.__traceback__) if f.filename == 'my_node.py']
        lines = ['Traceback (most recent call last):']
        for f in frames:
            lines.append(f'  File "my_node.py", line {f.lineno}, in {f.name}')
            if f.line:
                lines.append(f'    {f.line}')
        lines.append(f'{type(e).__name__}: {e}')
        _post(type='crash', text='\n'.join(lines), line=frames[-1].lineno if frames else None)
    _lab_stop(True)


def _safe_call(fn, *args):
    try:
        fn(*args)
        return True
    except _Spin:
        _state['unwinding'] = False
        return True
    except SystemExit:
        _lab_stop()
        return False
    except BaseException as e:
        _crash(e)
        return False


def _lab_clock(t):
    # the simulator advanced: move the clock and run any timers that are due
    t = float(t)
    last = _state['sim_t']
    _state['sim_t'] = t
    if last is not None and t < last - 1e-6:
        for node in list(_state['nodes']):
            for tm in node._timers:
                tm._next = t + tm._period
    _lab_tick()


def _lab_tick():
    if not _state['running']:
        return
    now = _now()
    for node in list(_state['spinning']):
        for t in list(node._timers):
            if t._canceled or now < t._next:
                continue
            t._next += t._period
            if t._next < now:
                t._next = now + t._period
            if not _safe_call(t.callback):
                return


def _lab_deliver(topic, msg_type, js, binp=None):
    if not _state['running']:
        return
    subs = _state['subs'].get(topic)
    if not subs:
        return
    data = None
    for s in list(subs):
        if s._destroyed or s.node not in _state['spinning'] or s.msg_type._type != msg_type:
            continue
        if data is None:
            data = json.loads(js)
            if binp is not None:
                data['data'] = array.array('B', binp.to_bytes())
        if not _safe_call(s.callback, _from_dict(s.msg_type, data)):
            return


def _lab_stop(quiet=False):
    was = _state['running']
    for n in list(_state['nodes']):
        n._destroy()
    _state.update(inited=False, nodes=[], spinning=[], subs={}, running=False, unwinding=False)
    if was and not quiet:
        _post(type='stopped')


def _lab_run(code, sim_t=None):
    _lab_stop(True)
    _state.update(running=True, unwinding=False, sim_t=None if sim_t is None else float(sim_t))
    linecache.cache['my_node.py'] = (len(code), None, code.splitlines(True), 'my_node.py')
    try:
        compiled = compile(code, 'my_node.py', 'exec')
    except SyntaxError as e:
        _crash(e)
        return
    g = {'__name__': '__main__', '__file__': 'my_node.py'}
    try:
        exec(compiled, g)
    except _Spin:
        pass
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException as e:
        _state['unwinding'] = False
        _crash(e)
        return
    _state['unwinding'] = False
    if not _state['running']:
        return
    if _state['spinning']:
        _post(type='spinning', nodes=[n._fqn() for n in _state['spinning']])
    else:
        if _state['nodes']:
            _post(type='hint', text='Your node was created, but rclpy.spin(node) was never called, so its timers and callbacks will not run.')
        else:
            _post(type='hint', text='No node was created. Did you forget to call main()?')
        _lab_stop()


def _lab_set_param(node, name, js, ptype=None):
    try:
        v = json.loads(js)
    except Exception:
        return json.dumps({'ok': False, 'reason': 'could not read the value'})
    if ptype == 'double' and isinstance(v, int) and not isinstance(v, bool):
        v = float(v)
    for n in list(_state['nodes']):
        if n._fqn() == node and n._alive:
            try:
                r = n._set_one(Parameter(name, value=v))
            except BaseException as e:
                _crash(e)
                return json.dumps({'ok': False, 'reason': 'your parameter callback raised an exception'})
            return json.dumps({'ok': bool(r.successful), 'reason': r.reason})
    return json.dumps({'ok': False, 'reason': 'node not found'})
