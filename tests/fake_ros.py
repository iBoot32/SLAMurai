"""Just enough of rclpy and the message packages (Dashing API) to import and exercise our ROS nodes on a PC.
install() puts the fakes into sys.modules.  Messages are simple attribute bags; nothing here checks QoS."""
import sys
import time
import types


class NS(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _msg(_name, **fields):
    def make(**kw):
        o = NS()
        for k, v in fields.items():
            setattr(o, k, v() if callable(v) else v)
        o.__dict__.update(kw)
        return o
    make.__name__ = _name
    return make


def vec3():
    return NS(x=0.0, y=0.0, z=0.0)


def quat():
    return NS(x=0.0, y=0.0, z=0.0, w=1.0)


def pose():
    return NS(position=vec3(), orientation=quat())


def header():
    return NS(stamp=NS(sec=0, nanosec=0), frame_id='')


Twist = _msg('Twist', linear=vec3, angular=vec3)
PoseStamped = _msg('PoseStamped', header=header, pose=pose)
PoseWithCovarianceStamped = _msg('PoseWithCovarianceStamped', header=header,
                                 pose=lambda: NS(pose=pose(), covariance=[0.0] * 36))
Odometry = _msg('Odometry', header=header, child_frame_id='',
                pose=lambda: NS(pose=pose(), covariance=[0.0] * 36),
                twist=lambda: NS(twist=NS(linear=vec3(), angular=vec3()), covariance=[0.0] * 36))
Path = _msg('Path', header=header, poses=list)
LaserScan = _msg('LaserScan', header=header, angle_min=0.0, angle_max=0.0, angle_increment=0.0, ranges=list)
String = _msg('String', data='')
Int32 = _msg('Int32', data=0)
Log = _msg('Log', name='', msg='', level=20)


class Logger(object):
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    warn = error = debug = info


class Clock(object):
    def now(self):
        t = time.time()
        return NS(nanoseconds=int(t * 1e9), to_msg=lambda: NS(sec=int(t), nanosec=int((t % 1) * 1e9)))


class Pub(object):
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Node(object):
    registry = []

    def __init__(self, name='node', **kw):
        self.name = name
        self.pubs, self.subs, self.timers = {}, {}, []
        self._logger = Logger()
        Node.registry.append(self)

    def create_publisher(self, typ, topic, qos=None, **kw):
        self.pubs[topic] = Pub()
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos=None, **kw):
        self.subs[topic] = cb
        return cb

    def create_timer(self, period, cb, **kw):
        self.timers.append((period, cb))
        return cb

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Clock()

    def destroy_node(self):
        pass

    # test helpers
    def deliver(self, topic, msg):
        self.subs[topic](msg)

    def tick(self):
        for _, cb in self.timers:
            cb()


def install():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=0: time.sleep(min(timeout_sec, 0.001))
    node_m = types.ModuleType('rclpy.node')
    node_m.Node = Node
    qos_m = types.ModuleType('rclpy.qos')
    qos_m.QoSProfile = lambda **kw: NS(**kw)
    qos_m.QoSReliabilityPolicy = NS(BEST_EFFORT=1, RELIABLE=2)
    qos_m.qos_profile_sensor_data = NS(depth=5)
    mods = {'rclpy': rclpy, 'rclpy.node': node_m, 'rclpy.qos': qos_m}
    for pkg, names in (('geometry_msgs', {'Twist': Twist, 'PoseStamped': PoseStamped,
                                          'PoseWithCovarianceStamped': PoseWithCovarianceStamped}),
                       ('nav_msgs', {'Odometry': Odometry, 'Path': Path}), ('sensor_msgs', {'LaserScan': LaserScan}),
                       ('std_msgs', {'String': String, 'Int32': Int32}), ('rcl_interfaces', {'Log': Log})):
        top = types.ModuleType(pkg)
        sub = types.ModuleType(pkg + '.msg')
        for k, v in names.items():
            setattr(sub, k, v)
        top.msg = sub
        mods[pkg], mods[pkg + '.msg'] = top, sub
    sys.modules.update(mods)
    return Node
