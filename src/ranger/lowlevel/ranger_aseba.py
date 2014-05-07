import logging
from robots.helpers.ansistrm import ColorizingStreamHandler

logger = logging.getLogger("ranger")
logger.setLevel(logging.INFO)

console = ColorizingStreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.addHandler(console)



import math, time

import threading # for threading.Condition

from robots import GenericRobot
robotlogger = logging.getLogger("robots")
robotlogger.addHandler(console)

from robots.helpers.helpers import enum
from robots.introspection import introspection

from aseba import Aseba
from ranger.helpers.data_conversion import *
from ranger.helpers.odom import Odom
from ranger.helpers.position import RangerPoseManager

MAX_SPEED = .16 #m.s^-1 on the wheels for ranger2

RANGER_ASEBA_SCRIPT = "/home/lemaigna/src/ranger2/aseba/RangerMain.aesl"

BATTERY_LOW_THRESHOLD = 7200 #mV

def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

Eyelids = enum("OPEN", "HALFOPEN", "CLOSED", "UNDEFINED") #undefined can occur if the 2 eyes have different lid position

class Ranger(GenericRobot):

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
        "w":                    0.0,   # rotation velocity
        "eyelids":              Eyelids.UNDEFINED   # state of the eyelids (open, half-open or closed)
        }

    def __init__(self, dummy = False, immediate = False):

        super(Ranger, self).__init__(actions = ["ranger.actions"], 
                                    dummy = dummy,
                                    immediate = immediate)


        self.beacons = {}
        self.odom = Odom()

        self.pose = RangerPoseManager(self)

        # creates accessors for each of the fields in STATE
        self.state.update(Ranger.STATE)

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

    def lefteye(self, x, y):
        self.eyes(lx=x, ly=y, rx=0, ry=0)

    def righteye(self, x, y):
        self.eyes(lx=0, ly=0, rx=x, ry=y)

    def eyes(self, lx, ly, 
                   rx = None, ry = None, 
                   l_upper_lid = 100, l_lower_lid = None, r_upper_lid = None, r_lower_lid = None):

        if rx is None:
            rx = lx
        if ry is None:
            ry = ly
        if l_lower_lid is None:
            l_lower_lid = l_upper_lid
        if r_upper_lid is None:
            r_upper_lid = l_upper_lid
        if r_lower_lid is None:
            r_lower_lid = l_lower_lid

        if l_upper_lid > 60 and \
           l_lower_lid > 60 and \
           r_upper_lid > 60 and \
           r_lower_lid > 60:
               self.state["eyelids"] = Eyelids.OPEN
        elif l_upper_lid < 60 and l_upper_lid > 10 and \
             l_lower_lid < 60 and l_lower_lid > 10 and \
             r_upper_lid < 60 and r_upper_lid > 10 and \
             r_lower_lid < 60 and r_lower_lid > 10:
               self.state["eyelids"] = Eyelids.HALFOPEN
        elif l_upper_lid == 0 and \
             l_lower_lid == 0 and \
             r_upper_lid == 0 and \
             r_lower_lid == 0:
               self.state["eyelids"] = Eyelids.CLOSED
        else:
               self.state["eyelids"] = Eyelids.UNDEFINED

        self._send_evt("EyeCartesianSet", 0, rx, ry)
        self._send_evt("EyeCartesianSet", 1, lx, ly)

        self._send_evt("openEyelid", 0, r_upper_lid)
        self._send_evt("openEyelid", 1, r_lower_lid)
        self._send_evt("openEyelid", 2, l_upper_lid)
        self._send_evt("openEyelid", 3, l_lower_lid)

    def led_pattern(self, pattern_id, repeat = False):
        self._send_evt("playLedVid", pattern_id, repeat)

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
                raise Exception("The robot does not transmit its state!! Check the connection to the aseba network.")

    def _process_main_feedback(self, msg, with_encoders = True):
        if introspection:
            introspection.ping()

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
            self.state["theta"] = self.pose.normalize_angle(th)
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

        self.theta = - math.pi * data[2] / 1800. + 2 * math.pi

        # angle at which the beacon is seen by the robot's RaB
        self.orientation = math.atan2(math.sin(math.pi * angle / 1800.),
                                      math.cos(math.pi * angle / 1800.))

        self.r = data[1] / 1000.  # in meters

        # beacon cartesian coordinates, *relative to the robot frame!*
        self.x = math.cos(self.orientation) * self.r
        self.y = math.sin(self.orientation) * self.r

    def obsolete(self):
        if time.time() - self.update > self.OBSOLETE_AFTER:
            return True
        else:
            return False
