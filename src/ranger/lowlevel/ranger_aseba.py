import logging; logger = logging.getLogger("ranger.lowlevel")

import math, time
import pkgutil, sys
import threading
from functools import partial

from aseba import Aseba
from ranger.helpers.helpers import valuefilter
from ranger.helpers.data_conversion import *
from ranger.helpers.odom import Odom
from ranger.introspection import introspection

MAX_SPEED = .16 #m.s^-1 on the wheels for ranger2

RANGER_ASEBA_SCRIPT = "/home/lemaigna/src/ranger2/aseba/RangerMain.aesl"

ID = dict(BEACON = 0,
          ROBOT1 = 1,
          ROBOT2 = 2,
          STATION1 = 4,
          STATION2 = 5)

ID["MYSTATION"] = ID["STATION1"] # TODO multirobot

BATTERY_LOW_THRESHOLD = 7200 #mV

_robot = None
def get_robot(dummy = False, immediate = False):
    """ Use this function to retrieve the (singleton) low-level
    robot accessor.

    :param immediate: (default: False) in immediate mode, robot's action are not
    asynchronous. Useful for debugging for instance.
    """
    global _robot
    if not _robot:
        _robot = _RangerLowLevel(dummy, immediate)
    return _robot


def _element_mul(list1, list2):
    return [ i * j for i, j in zip(list1, list2)] 

def _element_add(list1, list2):
    return [ i + j for i, j in zip(list1, list2)] 

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def _element_clip(list_i, min_val, max_val):
    return [clamp(i, min_val, max_val) for i in list_i]


class _RangerLowLevel():

    STATE = {
        # name of the field,    default?
        "accelerometer":        None,  # 3-axis accelerometer, in m.s^-2
        "sharp1":               None,  # IR sensor, bottom left, in m
        "sharp2":               None,  # IR sensor, bottom right, in m
        "battery":              None,  # battery level, in mV
        "bumper":               None,  # is front bumper active? (bool)
        "velocity_left":        None,   # left motor velocity, in rad.s^-1
        "velocity_right":       None,   # right motor velocity, in rad.s^-1
        "touch_left":           None,  # 3x3 bool matrix, touch sensors left
        "touch_rear":           None,  # 3x3 bool matrix, touch sensors rear
        "touch_right":          None,  # 3x3 bool matrix, touchsensors right
        "charging":             None,  # battery currently charging? (bool)
        "motor_current_left":   None,  # current in left motor, in mA
        "motor_current_right":  None,  # current in right motor, in mA
        "ir_left":              None,  # IR sensor, front-left, in m
        "ir_center":            None,  # IR sensor, front-center, in m
        "ir_right":             None,  # IR sensor, front-right, in m
        "lolette":              None,  # is the lolette present? (bool)
        "scale":                None,  # measured weight, in kg
        "freq_main":            0.0,  # update frequency of the main robot node
        "freq_neuil":           0.0,  # update frequency of the 'neuil' robot node
        "freq_rab":             0.0,  # update frequency of the 'Range and Bearing' robot node
        "x":                    0.0,  # x position of the robot, computed from odometry
        "y":                    0.0,  # y position of the robot, computed from odometry
        "theta":                0.0,  # orientation of the robot, computed from odometry
        "v":                    0.0,  # linear velocity, in robot's forward direction
        "w":                    0.0   # rotation velocity
        }

    def __init__(self, dummy = False, immediate = False):

        self.dummy = dummy
        self.immediate = immediate

        self.state = {}
        self.filteredvalues = {} # holds the filters for sensors that need filtering (like scale, IR sensors...)
        self.beacons = {}
        self.odom = Odom()

        # creates accessors for each of the fields in STATE
        self._init_accessors()

        if introspection:
            introspection.initiate(threading.current_thread().ident)

        #######################################################################
        #                       ASEBA initialization
        #######################################################################

        # init Aseba
        self.aseba = Aseba(dummy = dummy)

        # Basic check to be sure all the Ranger's ASEBA nodes are up and running
        nodes = self.aseba.get_nodes_list()
        if not dummy and len(nodes) != 3:
            logger.error("One of the Ranger Aseba node is not up!!")
            logger.debug("List of active nodes: {0}".format(nodes))
            raise Exception("Missing Aseba node")

        # (Re-)load the aesl scripts, mandatory for Aseba to know about
        # the list of available events
        self.aseba.load_scripts(RANGER_ASEBA_SCRIPT)

        # Register callbacks for the main events of the 3 nodes
        self.aseba.on_event("mainFeedbackWithEncoders", self._process_main_feedback)
        self.aseba.on_event("neuilFeedback", self._process_neuil_feedback)
        self.aseba.on_event("receiverFeedback", self._process_rab_feedback)

        # Condition variable that can be used to wait
        # until the next update of a given Aseba node
        self.main_update = threading.Condition()
        self.neuil_update = threading.Condition()
        self.rab_update = threading.Condition()
        self.update = threading.Condition() # get notified when any node is updated

        # Starts DBus thread (responsible for receiving events)
        self.aseba_thread = threading.Thread(target=self.aseba.run)
        self.aseba_thread.start()

        # Asks the nodes to send their events
        self._send_evt("enableEncoders", enable = 1)
        self._send_evt("enableFeedback", enable = 1)

        # Wait until we hear about the 2 main nodes ('main' and 'neuil')
        if not dummy:
            self.get_full_state()

        #######################################################################
        #                   End of ASEBA initialization
        #######################################################################


        # Import all modules under robots/actions/
        import ranger.actions
        path = sys.modules['ranger.actions'].__path__
        for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
            __import__('ranger.actions.' + module_name)

        # Dynamically add available actions (ie, actions defined with @action in
        # actions/* submodules.
        for action in self.available_actions():
            setattr(self, action.__name__, partial(action, self))
            logger.info("Added " + action.__name__ + " as available action.")

    def lefteye(self, x, y):
        self.eyes(lx=x, ly=y, rx=0, ry=0)

    def righteye(self, x, y):
        self.eyes(lx=0, ly=0, rx=x, ry=y)

    def eyes(self, lx, ly, 
                   rx = None, ry = None, 
                   l_upper_lid = 100, l_lower_lid = None, r_upper_lid = None, r_lower_lid = None):

        if rx is None:
            rx = -lx # negative because positive values converge toward center
        if ry is None:
            ry = ly
        if l_lower_lid is None:
            l_lower_lid = l_upper_lid
        if r_upper_lid is None:
            r_upper_lid = l_upper_lid
        if r_lower_lid is None:
            r_lower_lid = l_lower_lid

        self._send_evt("neuilEvent", lx, ly,
                                     rx, ry,
                                     l_upper_lid, l_lower_lid,
                                     r_upper_lid, r_lower_lid)

    def speed(self, l = None, r = None, v = None, w = None):
        """ Set the motor's speed, in {m, rad}.s^-1.

            Either call 
            >>> robot.speed(v = ..., w = ...)

            or:
            >>> robot.speed(l = ..., r = ...)

        """

        if v is not None or w is not None:
            if v is None:
                v = 0
            if w is None:
                w = 0
            # control in (v,w)
            l, r = self.odom.twist_to_motors(v, w)
        elif r is not None or l is not None:
            if r is None:
                r = l
            elif l is None:
                l = -r
        else:
            logger.warning("setSpeed called with no speed!")
            return

        l = clamp( l * 100. / MAX_SPEED, -100, 100)
        r = clamp( -r * 100. / MAX_SPEED, -100, 100) # -r !

        self._send_evt("setSpeed", l, r)

    def distanceto(self, x, y):
        return math.sqrt(math.pow(self.x - x, 2) + \
                         math.pow(self.y - y, 2))


    def angleto(self, x, y, relative = False):
        if relative:
            return self.angleto(y + self.state["y"], x + self.state["x"])
        else:
            return self.normalize_angle(
                    - self.state["theta"] + math.atan2(y, x)
                    )

    def normalize_angle(self, angle):
        """ Returns equivalent angle such as  -pi < angle <= pi
        """
        angle = angle % (2 * math.pi) # => angle > 0
        return angle if angle <= math.pi else (-math.pi + angle % math.pi)


    def get_full_state(self):
        """Blocks until the full state of the robot has been
        received from the low-level.
        """
        MAX_WAIT = 2 # seconds

        wait_duration = 0.

        while "accelerometer" not in self.state or \
              "lolette" not in self.state:
            time.sleep(0.1)
            wait_duration += 0.1

            if wait_duration > MAX_WAIT:
                raise Exception("The robot does not transmit its state!!")

    def wait(self, var, value = True, above = None, below = None):
        """ waits until the state value 'var' fulfill some condition:
                - if 'above' is set, until var > above
                - if 'below' is set, until var < below
                - else if value is set (default to True), until var = value
        """
        if self.dummy:
            return

        if var not in self.STATE:
            raise Exception("%s is not part of the robot state" % var)

        if var not in self.state:
            # value not yet read from the robot.
            logger.info("Waiting for %s to be published by the robot..." % var)
            self.update.acquire()
            while not var in self.state:
                self.update.wait()
            self.update.release()


        if above is not None:
            if self.state[var] > above: return

            self.update.acquire()
            while not self.state[var] > above:
                self.update.wait()
            self.update.release()

        elif below is not None:
            if self.state[var] < below: return

            self.update.acquire()
            while not self.state[var] < below:
                self.update.wait()
            self.update.release()

        else:
            if self.state[var] == value: return

            self.update.acquire()
            while not self.state[var] == value:
                self.update.wait()
            self.update.release()


    def _process_main_feedback(self, msg, with_encoders = True):
        self.state["accelerometer"] = [msg[0], msg[1], msg[2]]
        self.state["sharp1"] = self.filtered("ir_sharp1", linear_interpolation(msg[3], GP2Y0A41SK0F))

        self.state["sharp2"] = self.filtered("ir_sharp2", linear_interpolation(msg[4], GP2Y0A41SK0F))

        self.state["battery"] = self.filtered("battery", msg[5])
        self.state["bumper"] = msg[6]
        self.state["velocity_left"] = self.filtered("lspeed", msg[7])
        self.state["velocity_right"] = self.filtered("rspeed", -msg[8])

        def decompress_touch(state):
            raw = [bool(state & 1 << i) for i in range(9)]
            return [raw[0:9:3], raw[1:9:3], raw[2:9:3]]

        self.state["touch_left"] = decompress_touch(msg[10])
        self.state["touch_rear"] = decompress_touch(msg[11])
        self.state["touch_right"] = decompress_touch(msg[9])

        if with_encoders:
            # "encoders" = [msg[12], msg[13], msg[14], msg[15]]
            self.odom.update(-msg[14], msg[12]) # left, right
            x,y,th,v,w = self.odom.get()
            self.state["x"] = x
            self.state["y"] = y
            self.state["theta"] = self.normalize_angle(th)
            self.state["v"] = v
            self.state["w"] = w

        self.state["charging"] = msg[12 if not with_encoders else 16]
        self.state["motor_current_left"] = msg[13 if not with_encoders else 17]
        self.state["motor_current_right"] = msg[14 if not with_encoders else 18]

        self.state["freq_main"] = self.aseba.events_freq["mainFeedbackWithEncoders"]

        self.update.acquire()
        self.main_update.acquire()
        self.update.notifyAll()
        self.main_update.notifyAll()
        self.update.release()
        self.main_update.release()

    def _process_neuil_feedback(self, msg):

        self.state["ir_left"] = self.filtered("ir_left", linear_interpolation(msg[0], GP2Y0A41SK0F))
        self.state["ir_center"] = self.filtered("ir_center", linear_interpolation(msg[1], GP2Y0A02YK))
        self.state["ir_right"] = self.filtered("ir_right", linear_interpolation(msg[2], GP2Y0A41SK0F))
        self.state["lolette"] = msg[3]

        self.state["scale"] = self.filtered("scale", linear_interpolation(msg[4], SCALE))

        self.state["freq_neuil"] = self.aseba.events_freq["neuilFeedback"]

        self.update.acquire()
        self.neuil_update.acquire()
        self.update.notifyAll()
        self.neuil_update.notifyAll()
        self.update.release()
        self.neuil_update.release()


    def _process_rab_feedback(self, msg):
        """
        msg[0]: beacon ID
        msg[1]: angle where the beacon is seen, from the robot's R&B perspective
        msg[2]: distance
        msg[3]: angle where the robot is seen, from the beacon R&B perspective (ie, theta polar coord of robot in map)
        msg[4]: distance where the robot is seen, from the beacon R&B perspective (ie, r polar coord of robot in map)
        """
        id = msg[0]

        distance = msg[2]
        if distance > 0: # may be zero in case of error (somewhere...)
            beacon = Beacon(id = id,
                    distance = msg[2],
                    angle = msg[1],
                    data = [msg[3+i] for i in range(7)])

            self.beacons[id] = beacon

            self.state["freq_rab"] = self.aseba.events_freq["receiverFeedback"]

            self.update.acquire()
            self.rab_update.acquire()
            self.update.notifyAll()
            self.rab_update.notifyAll()
            self.update.release()
            self.rab_update.release()


    def _send_evt(self, id, *args, **kwargs):

        if not args:
            args = kwargs.values()
            logger.debug("Event %s(%s) sent." % (id, str(kwargs)))
        else:
            logger.debug("Event %s(%s) sent." % (id, str(args)))
        self.aseba.send_event(id, args)

    def _add_property(self, name, default):

        def getter(self):
            if name in self.state:
                return self.state[name]
            else:
                return None

        setattr(self.__class__,name, property(getter))

        if default is not None:
            self.state[name] = default


    def _init_accessors(self):

        for name, default in self.STATE.items():
            # the creation of the setters,getters must take place in a separate
            # function, else they all refer to the same field name... Python internals...
            self._add_property(name, default)

    def filtered(self, name, val):
        """ Helper to easily filter values (uses an accumulator to average a given 'name' quantity)
        """

        filter = self.filteredvalues.setdefault(name, valuefilter())
        filter.append(val)
        return filter.get()

    def available_actions(self):
        """ Iterate over all loaded modules, and retrieve actions (ie functions
        with the @action decorator).
        """
        actions = []
        path = sys.modules["ranger.actions"].__path__
        for loader, module_name, is_pkg in  pkgutil.walk_packages(path):
            m = sys.modules["ranger.actions." + module_name]
            for member in [getattr(m, fn) for fn in dir(m)]:
                if hasattr(member, "_action"):
                    actions.append(member)

        return actions

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def close(self):
        self.aseba.close()
        self.aseba_thread.join()


class Beacon:
    """
                               ^
                               |
                               |     /
                              /|.   /
                             / | `+.
                     Robot  /  |.'  `.
                           /   /    /
                          `-..-'`. /
                          .-'`.   `.
                   r  _.-'     `-/  `.
                   .-'
    Beacon      .-'
    ,---.    .-'--.
   /     _.-'      | theta
  (   ............,.........>
   \     /
    `---'

    """

    OBSOLETE_AFTER = 5 #seconds


    def __init__(self, id, distance, angle, data):

        self.update = time.time()
        self.id = id

        self.theta = - math.pi * data[0] / 1800. + 2 * math.pi

        # angle at which the beacon is seen by the robot's RaB
        self.orientation = math.atan2(math.sin(math.pi * angle / 1800.),
                                      math.cos(math.pi * angle / 1800.))

        self.r = data[1] / 1000.  # in meters

    def obsolete(self):
        if time.time() - self.update > self.OBSOLETE_AFTER:
            return True
        else:
            return False
