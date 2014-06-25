import logging
import sys

logger = logging.getLogger("ranger")
logger_aseba = logging.getLogger("ranger.aseba")


import math, time
from collections import deque

import threading # for threading.Condition

from robots import GenericRobot

from robots.mw import ROS
from robots.helpers.helpers import enum
from robots.helpers.position import PoseManager # for normalize_angle
from robots.introspection import introspection

from aseba import Aseba
from ranger.behaviours.emotions import EmotionalState
from ranger.helpers.data_conversion import *
from ranger.helpers.odom import Odom
from ranger.helpers.position import RangerPoseManager
from ranger.res import MYSTATION

MAX_SPEED = .16 #m.s^-1 on the wheels for ranger2

BATTERY_MAX_LEVEL = 8400 #mV
BATTERY_MIN_CHARGED_LEVEL = 8000 #mV
BATTERY_LOW_THRESHOLD = 7200 #mV

# Required to access the list of events
RANGER_ASEBA_SCRIPT = "/home/lemaigna/src/ranger2/aseba/RangerMain.aesl"

def configure_logging():

    from robots.helpers.ansistrm import ConcurrentColorizingStreamHandler

    logger.setLevel(logging.INFO)
    logger_aseba.setLevel(logging.INFO)

    console = ConcurrentColorizingStreamHandler()
    console.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    console.setFormatter(formatter)
    logger.addHandler(console)
    logger_aseba.addHandler(console)

    robotlogger = logging.getLogger("robots")
    robotlogger.addHandler(console)


def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

class Ranger(GenericRobot):

    eyelids = enum(OPEN=((100,100), (100, 100)),
                HALFOPEN = ((50, 50), (50, 50)),
                CLOSED = ((0,0), (0, 0)))


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
        "eyelids":              None,  # state of the eyelids (open, half-open or closed)
        "asleep":               False # general awarness 
        }

    def __init__(self, dummy = False, immediate = False, default_logging=True, with_ros = True):

        if default_logging:
            configure_logging()

        super(Ranger, self).__init__(actions = ["ranger.actions"], 
                                    supports = ROS if with_ros else 0,
                                    dummy = dummy,
                                    immediate = immediate)


        self.beacons = {}
        self.odom = Odom()

        self.pose = RangerPoseManager(self)

        # creates accessors for each of the fields in STATE
        self.state.update(Ranger.STATE)
        self.state.eyelids = Ranger.eyelids.OPEN
        self.state.touches = TouchManager()

        self.innerstate = EmotionalState()

        #######################################################################
        #                       ASEBA initialization
        #######################################################################

        # init Aseba
        self.aseba = Aseba(dummy = dummy)

        self.aseba.load_events_list(RANGER_ASEBA_SCRIPT)

        # Basic check to be sure all the Ranger's ASEBA nodes are up and running
        nodes = self.aseba.get_nodes_list()
        if not dummy and len(nodes) != 3:
            logger.error("One of the Ranger Aseba node is not up!!")
            logger.error("List of active nodes: {0}".format(nodes))
            raise Exception("Missing Aseba node")

        # Register callbacks for the main events of the 3 nodes
        self.aseba.on_event("mainFeedbackWithEncoders", self._process_main_feedback)
        self.aseba.on_event("neuilFeedback", self._process_neuil_feedback)
        self.aseba.on_event("receiverFeedback", self._process_rab_feedback)

        # Condition variable that can be used to wait
        # until the next update of a given Aseba node
        self.main_update = threading.Event()
        self.neuil_update = threading.Event()
        self.rab_update = threading.Event()

        # Starts DBus thread (responsible for receiving events)
        self.aseba_thread = threading.Thread(target=self.aseba.run)
        self.aseba_thread.start()

        # Asks the nodes to send their events
        self._send_evt("enableEncoders", enable = 1)
        self._send_evt("enableFeedback", enable = 1)

        # Wait until we hear about the 2 main nodes ('main' and 'neuil')
        if not dummy:
            self.get_full_state()

    	if sys.version_info < (2, 7):
	    logger.warning("Python < 2.7 does not correctly supports timeouts on threading.Event! I may wait forever for Aseba updates in case of problems with Aseba network!")

        #######################################################################
        #                   End of ASEBA initialization
        #######################################################################

    def close(self):
        self.aseba.close()
        super(Ranger, self).close()


    def lefteye(self, x, y):
        """
        Move the pupil of the left eye

        :param x: horizontal position of the pupil, from -100 (left) to 100 (right)
        :param y: vertical position of the pupil, from -100 (bottom) to 100 (top)
        """
        self._send_evt("EyeCartesianSet", 1, -x, y)

    def righteye(self, x, y):
        """
        Move the pupil of the right eye

        :param x: horizontal position of the pupil, from -100 (left) to 100 (right)
        :param y: vertical position of the pupil, from -100 (bottom) to 100 (top)
        """
        self._send_evt("EyeCartesianSet", 0, -x, y)

    def leftlid(self, pos):
        """
        Open/close the left eyelid

        :param pos: position of lid, as a tuple (upper_lid, lower_lid) (values
        from 0 to 100, or None to keep the lid part in place)
        """
        if pos:
            upper, lower = pos

            if upper is not None: # may be 0!
                self._send_evt("openEyelid", 2, upper)
            if lower is not None: # may be 0!
                self._send_evt("openEyelid", 3, lower)


    def rightlid(self, pos):
        """
        Open/close the right eyelid

        :param pos: position of lid, as a tuple (upper_lid, lower_lid) (values
        from 0 to 100, or None to keep the lid part in place)
        """
        if pos:
            upper, lower = pos

            if upper is not None: # may be 0!
                self._send_evt("openEyelid", 0, upper)
            if lower is not None: # may be 0!
                self._send_evt("openEyelid", 1, lower)


    def eyes(self, eyes = None,
                   lids = None,
                   left = None, 
                   right = None,
                   left_lid = None,
                   right_lid = None):

        if eyes:
            x, y = eyes
            self.righteye(x, y)
            self.lefteye(x, y)
        else:
            if right:
                x, y = right
                self.righteye(x, y)

            if left:
                x, y = left
                self.lefteye(x, y)

        if lids:
            # lids can be either a tuple (upper, lower) shared by both eyes
            # or a tuple of tuples ((lu, ll), (ru, rl))
            l, r = lids
            if isinstance(l, tuple):
                self.leftlid(l)
                left_lid = l
                self.rightlid(r)
                right_lid = r
            else:
                self.leftlid(lids)
                self.rightlid(lids)
                left_lid = lids
                right_lid = lids

            self.state["eyelids"] = (left_lid, right_lid)

        else:
            if left_lid is not None:
                self.leftlid(left_lid)
                self.state["eyelids"] = (left_lid, self.state["eyelids"][1])
            if right_lid is not None:
                self.rightlid(right_lid)
                self.state["eyelids"] = (self.state["eyelids"][0], right_lid)



    def set_led(self, id, color):
        r, g, b = color
        self._send_evt("setLed", id, r, g, b)
 
    def set_all_leds(self, color):
        r, g, b = color
        self._send_evt("setAllLeds", r, g, b)


    def set_led_row(self, id, color):
        r, g, b = color
        self._send_evt("setLedsRow", id, r, g, b)

    def set_led_col(self, id, color):
        r, g, b = color
        self._send_evt("setLedsCol", id, r, g, b)


    def turn_off_leds(self):
        self._send_evt("turnOffLeds")


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
                l = r
        else:
            logger.warning("setSpeed called with no speed!")
            return

        l = clamp( l * 100. / MAX_SPEED, -100, 100)
        r = clamp( -r * 100. / MAX_SPEED, -100, 100) # -r !

        self._send_evt("setSpeed", l, r)

    def wait_for_state_update(self, timeout = None):
        """
        TODO: Not too good... what is in general the semantic of 'wait_for_state'? seem to be very robot dependent...
        """
    	if sys.version_info < (2, 7):
            self.main_update.wait()
	    return

        return self.main_update.wait(timeout)

    def loglevel(self, level):
        super(Ranger, self).loglevel(level)
        logger.setLevel(level)

    def get_full_state(self, timeout = 2):
        """Blocks until the full state of the robot has been
        received from the low-level.
        """
    	if sys.version_info < (2, 7):
            self.main_update.wait()
            self.neuil_update.wait()
	    return

        if self.main_update.wait(timeout) is None:
            raise RuntimeError("'main' node does not transmit its state!! Check the connection to the aseba network.")

        if self.neuil_update.wait(timeout) is None:
            raise RuntimeError("'neuil' node does not transmit its state!! Check the connection to the aseba network.")

        if self.rab_update.wait(timeout) is None:
            logger.warning("I did not see any beacons during initialization! I can not set my absolute location.")
        else:
            if MYSTATION not in self.beacons:
                logger.warning("I did not see the charging station during initialization! I can not set my absolute location.")
            else:
                if not self.beacons[MYSTATION].valid:
                    logger.warning("Charging station out of sight or too close during initialization! I can not set my absolute location.")
                else:
                    self.odom.reset(self.beacons[MYSTATION].robot_x,
                                    self.beacons[MYSTATION].robot_y)
                                    # do not set theta for now, too unreliable -> this means that the robot *must* be
                                    # initially oriented like the base station.

    def _process_main_feedback(self, msg, with_encoders = True):
        if introspection:
            introspection.ping()

        self.state["accelerometer"] = [msg[0], msg[1], msg[2]]
        self.state["sharp1"] = self.filtered("ir_sharp1", linear_interpolation(msg[3], GP2Y0A41SK0F))

        self.state["sharp2"] = self.filtered("ir_sharp2", linear_interpolation(msg[4], GP2Y0A41SK0F))

        self.state["battery"] = self.filtered("battery", msg[5])
        self.state["bumper"] = msg[6]
        self.state["scale"] = self.filtered("scale", linear_interpolation(msg[7], SCALE))
        self.state["velocity_left"] = self.filtered("lspeed", msg[8])
        self.state["velocity_right"] = self.filtered("rspeed", -msg[9])

        def decompress_touch(state):
            raw = [bool(state & 1 << i) for i in range(9)]
            return [[raw[3], raw[2], raw[1]],
                    [raw[4], raw[0], raw[8]],
                    [raw[5], raw[6], raw[7]]]

        self.state["touch_left"] = decompress_touch(msg[11])
        self.state["touch_rear"] = decompress_touch(msg[12])
        self.state["touch_right"] = decompress_touch(msg[10])
        self.state.touches.update(self.state.touch_left, self.state.touch_right)

        if with_encoders:
            # "encoders" = [msg[13], msg[14], msg[15], msg[16]]
            self.odom.update(-msg[15], msg[13]) # left, right
            x,y,th,v,w = self.odom.get()

            self.state["x"] = x
            self.state["y"] = y
            self.state["theta"] = self.pose.normalize_angle(th)
            self.state["v"] = v
            self.state["w"] = w

        self.state["charging"] = msg[13 if not with_encoders else 17]

        # when charging, and *assuming the robot is correctly positioned on
        # the charging station*, the location is known
        if self.state.charging:
            self.odom.reset(x=0.35, y=0, theta=0)

        self.state["motor_current_left"] = msg[14 if not with_encoders else 18]
        self.state["motor_current_right"] = msg[15 if not with_encoders else 19]

        self.state["freq_main"] = self.aseba.get_event_frequency("mainFeedbackWithEncoders")

        # notify the update
        self.main_update.set()
        self.main_update.clear()

    def _process_neuil_feedback(self, msg):

        self.state["ir_left"] = self.filtered("ir_left", linear_interpolation(msg[0], GP2Y0A41SK0F))
        self.state["ir_center"] = self.filtered("ir_center", linear_interpolation(msg[1], GP2Y0A02YK))
        self.state["ir_right"] = self.filtered("ir_right", linear_interpolation(msg[2], GP2Y0A41SK0F))
        self.state["lolette"] = msg[3]


        self.state["freq_neuil"] = self.aseba.get_event_frequency("neuilFeedback")

        # notify the update
        self.neuil_update.set()
        self.neuil_update.clear()


    def _process_rab_feedback(self, msg):
        """
        msg[0]: beacon ID
        msg[1]: angle where the beacon is seen, from the robot's R&B perspective
        msg[2]: distance
        msg[5]: angle where the robot is seen, from the beacon R&B perspective (ie, theta polar coord of robot in map)
        msg[4]: distance where the robot is seen, from the beacon R&B perspective (ie, r polar coord of robot in map)
        """
        id, angle, distance, my_own_id, packet_id, reverse_distance, reverse_angle, lolettes = [int(val) for val in msg[:8]] # convert DBus integers to regular int


        # convert to meters and radians
        angle = PoseManager.normalize_angle(-math.pi * angle / 1800.)
        distance = distance / 1000.
        reverse_angle = PoseManager.normalize_angle(-math.pi * reverse_angle / 1800.)
        reverse_distance = reverse_distance / 1000.

        if distance > 0: # may be zero in case of error (somewhere...)
            if id not in self.beacons:
                self.beacons["rab_%s" % id] = Beacon(id, self)

            self.beacons["rab_%s" % id].update(
                    distance = distance,
                    angle = angle,
                    reverse_distance = reverse_distance,
                    reverse_angle = reverse_angle)

            self.state["freq_rab"] = self.aseba.get_event_frequency("receiverFeedback")

            # notify the update
            self.rab_update.set()
            self.rab_update.clear()


    def _send_evt(self, id, *args, **kwargs):

        if not args:
            args = kwargs.values()
            logger_aseba.debug("Event %s(%s) sent." % (id, str(kwargs)))
        else:
            logger_aseba.debug("Event %s(%s) sent." % (id, str(args)))
        self.aseba.send_event(id, args)


# Limit value to consider a reading as valid
# ID: [min_dist, max_dist, min_angle, max_angle]
BEACONS_MODEL = {15: [1.0, 6.168, -0.43, 0.43],
                 16: [1.0, 6.168, -0.43, 0.43],
                 20: [0.1, 6.168, -0.43, 0.43]}


class TouchManager:

    def __init__(self):
        self.l_vert_history = deque(maxlen=10)
        self.l_horiz_history = deque(maxlen=10)
        self.r_vert_history = deque(maxlen=10)
        self.r_horiz_history = deque(maxlen=10)


    def update(self, left, right):
        # flatten the matrices
        self.right = [item for sublist in right for item in sublist]
        self.left = [item for sublist in left for item in sublist]

        #self.l_horiz_history.append([self.left[0] or self.left[1] or self.left[2],
        #                           self.left[3] or self.left[4] or self.left[5],
        #                           self.left[6] or self.left[7] or self.left[8]] )

        #self.r_horiz_history.append([self.right[0] or self.right[1] or self.right[2],
        #                           self.right[3] or self.right[4] or self.right[5],
        #                           self.right[6] or self.right[7] or self.right[8]] )

        #self.l_vert_history.append([self.left[0] or self.left[3] or self.left[6],
        #                           self.left[1] or self.left[4] or self.left[7],
        #                           self.left[2] or self.left[5] or self.left[8]] )

        #self.r_vert_history.append([self.right[0] or self.right[3] or self.right[6],
        #                           self.right[1] or self.right[4] or self.right[7],
        #                           self.right[2] or self.right[5] or self.right[8]] )


    def is_touched_left(self):
        return True in self.left

    def is_touched_right(self):
        return True in self.right

    def is_touched(self):
        return self.is_touched_left or self.is_touched_right

    def is_back_to_front_left(self):
        pass
    def is_back_to_front_right(self):
        pass

class Beacon:
    """

                                     /ry
                              /`.   /
                             /   `+.
                     Robot  /   .'  `.
                           /   /    /
                          `-..-'`. /
      ^ by                .-'`.   `.
      |            r  _.-' \   `-/ /`.rx
      |            .-'      `-___-'
      |Beacon   .-'            phi
    ,-|-.    .-'<-.
   /  |  _.-'      | theta
  (   ............,.........>
   \     /                  bx
    `---'

    """

    OBSOLETE_AFTER = 5 #seconds


    def __init__(self, id, robot):

        self.robot = robot
        self.last_update = time.time()

        self.id = id

        if self.id not in BEACONS_MODEL:
            logger.warning("No model available for beacon %s" % self.id)

        self.valid = False
        self.last_valid_pose = None

    def checkvalid(self, distance, angle):
        if self.id not in BEACONS_MODEL:
            return True

        min_dist, max_dist, min_angle, max_angle = BEACONS_MODEL[self.id]

        return True if min_dist < distance < max_dist and \
                       min_angle < angle < max_angle \
                    else False

    def update(self, distance, angle, 
                     reverse_distance, reverse_angle):
        """
        :param distance: distance (in m) to which the robot *believes* its
        stands from the beacon.
        :param angle: angle (in rad) to which the robot sees the beacon, in
        the robot frame (phi on the above diagram)
        :param reverse_distance: distance (in m) to which the beacon belives
        the robot is.
        :param reverse_angle: angle (in rad) to which the robot is seen, in
        the beacon frame (theta in the diagram above)

        """

        self.last_update = time.time()

        # not valid if:
        #  - distance == 6.128m, correspond to beacon out of sight
        #  - reverse_distance < 1m (reverse angle readings not meaningful if
        #  too close)
        #  - more or less facing the beacon (< ~25°)


        self.valid = self.checkvalid(reverse_distance, angle)

        self.r = reverse_distance
        self.dist = distance


        self.theta = PoseManager.normalize_angle(reverse_angle)

        # angle at which the beacon is seen by the robot's RaB
        self.phi = PoseManager.normalize_angle(angle)

        # beacon cartesian coordinates, *relative to the robot frame!*
        self.x = math.cos(self.phi) * self.r
        self.y = math.sin(self.phi) * self.r
        self.beacon_theta = -PoseManager.normalize_angle(math.pi - (self.theta - self.phi))

        # robot cartesian coordinates, *relative to the beacon frame!*
        self.robot_x = math.cos(self.theta) * self.r
        self.robot_y = math.sin(self.theta) * self.r
        # orientation of the robot, in the beacon frame
        self.robot_theta = PoseManager.normalize_angle(math.pi - (self.theta - self.phi))

        if self.valid:
            self.last_valid_pose = self.robot.pose.inframe([self.x,
                                                            self.y,
                                                            0,
                                                            0,
                                                            0,
                                                            self.beacon_theta,
                                                            "base_link"],
                                                            "map")
    def obsolete(self):
        if time.time() - self.last_update > self.OBSOLETE_AFTER:
            return True
        else:
            return False
