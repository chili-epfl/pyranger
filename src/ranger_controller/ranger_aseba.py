import logging; logger = logging.getLogger("aseba")

import math,time
import copy

from medulla import Medulla
from helpers import singleton

from threading import Thread

ID = dict(BEACON = 0,
          ROBOT1 = 1,
          ROBOT2 = 2,
          STATION1 = 3,
          STATION2 = 4)

ID["MYSTATION"] = ID["STATION1"] # TODO multirobot

BATTERY_LOW_THRESHOLD = 7200 #mV

_robot = None
def get_robot():
    """ Use this function to retrieve the (singleton) low-level
    robot accessor.
    """
    global _robot
    if not _robot:
        _robot = _RangerLowLevel()
    return _robot


def _element_mul(list1, list2):
    return [ i * j for i, j in zip(list1, list2)] 

def _element_add(list1, list2):
    return [ i + j for i, j in zip(list1, list2)] 

def _element_clip(list_i, min_val, max_val):
    return [min(max(i, min_val), max_val) for i in list_i]


class _RangerLowLevel(Thread):

    STATE = {
        # name of the field,    writable?
        "accelerometer":        False,  # 3-axis accelerometer, in m.s^-2
        "sharp1":               False,  # IR sensor, bottom left, in m
        "sharp2":               False,  # IR sensor, bottom right, in m
        "battery":              False,  # battery level, in mV
        "bumper":               False,  # is front bumper active? (bool)
        "velocity_left":        True,   # left motor velocity, in rad.s^-1
        "velocity_right":       True,   # right motor velocity, in rad.s^-1
        "touch_left":           False,  # 3x3 bool matrix, touch sensors left
        "touch_rear":           False,  # 3x3 bool matrix, touch sensors rear
        "touch_right":          False,  # 3x3 bool matrix, touchsensors right
        "encoders":             False,  # motors' 4 encoders (LLRR), in tics
        "charging":             False,  # battery currently charging? (bool)
        "motor_current_left":   False,  # current in left motor, in mA
        "motor_current_right":  False,  # current in right motor, in mA
        "ir_left":              False,  # IR sensor, front-left, in m
        "ir_center":            False,  # IR sensor, front-center, in m
        "ir_right":             False,  # IR sensor, front-right, in m
        "lolette":              False,  # is the lolette present? (bool)
        "scale":                False,  # measured weight, in kg
        "eye_left_x":           True,   # x coord of left pupil, [-100,100]
        "eye_left_y":           True,   # y coord of left pupil, [-100,100]
        "eye_right_x":          True,   # x coord of right pupil, [-100,100]
        "eye_right_y":          True    # y coord of right pupil, [-100,100]
        }

    def __init__(self):
        Thread.__init__(self)

        dummy = True
        # init medulla
        self.med = Medulla(dummy = dummy)
        #self.med = Medulla()
        nodes = self.med.GetNodesList()
        if not dummy and len(nodes) != 3:
            logger.error("One of the Ranger Aseba node is not up!!")
            logger.debug("List of active nodes: {0}".format(nodes))
            raise Exception("Missing Aseba node")

        self._send_evt("enableFeedback", enable = 1)

        self.state = {}
        self.beacons = {}

        self.running = True
        self.start()

        self.get_full_state()
        self._init_accessors()

    def close(self):
        self.running = False
        self.join()

    def run(self):

        self.main_msg = [ 1.2, 0.4, 1.5,
                    0.43, 0.54,
                    3452,
                    1,
                    1.2, 2.5,
                    0b100110100, 0b000100110, 0b001000111,
                    1234,34324,5432,34554,
                    0,
                    1.6,2.3]
        self.neuil_msg = [0.12, 1.03, 0.03,
                          0,
                          0.023]

 
        while self.running:
            time.sleep(0.02)
            if hasattr(self, 'main_msg') and self.main_msg:
                self._process_main_feedback(self.main_msg, with_encoders = True)
                self.main_msg = None

            if hasattr(self, 'neuil_msg') and self.neuil_msg:
                self._process_neuil_feedback(self.neuil_msg)
                self.neuil_msg = None

            if hasattr(self, 'rab_msg') and self.rab_msg:
                self._process_rab_feedback(self.rab_msg)
                self.rab_msg = None




    def lefteye(self, x, y):
        self._send_evt("EYE", leyex = x, leyey = y, reyex = 0, reyey = 0)

    def righteye(self, x, y):
        self._send_evt("EYE", leyex = 0, leyey = 0, reyex = x, reyey = y)

    def eyes(self, x, y):
        self._send_evt("EYE", leyex = x, leyey = y, reyex = x, reyey = y)

    def get_full_state(self):
        """Blocks until the full state of the robot has been
        received from the low-level.
        """
        MAX_WAIT = 2 # seconds

        wait_duration = 0.

        while "accelerometer" not in self.state and \
              "lolette" not in self.state:
                  time.sleep(0.1)
                  wait_duration += 0.1

                  if wait_duration > MAX_WAIT:
                      raise Exception("The robot does not transmit its state!!")

    def _process_main_feedback(self, msg, with_encoders = False):
        self.state["accelerometer"] = [msg[0], msg[1], msg[2]]
        self.state["sharp1"] = msg[3]
        self.state["sharp2"] = msg[4]
        self.state["battery"] = msg[5]
        self.state["bumper"] = msg[6]
        self.state["velocity_left"] = msg[7]
        self.state["velocity_right"] = msg[8]

        def decompress_touch(state):
            raw = [bool(state & 1 << i) for i in range(9)]
            return [raw[0:3], raw[3:6], raw[6:9]]

        self.state["touch_left"] = decompress_touch(msg[9])
        self.state["touch_rear"] = decompress_touch(msg[10])
        self.state["touch_right"] = decompress_touch(msg[11])

        if with_encoders:
            self.state["encoders"] = [msg[12], msg[13], msg[14], msg[15]]

        self.state["charging"] = msg[12 if not with_encoders else 16]
        self.state["motor_current_left"] = msg[13 if not with_encoders else 17]
        self.state["motor_current_right"] = msg[14 if not with_encoders else 18]


    def _process_neuil_feedback(self, msg):
        self.state["ir_left"] = msg[0]
        self.state["ir_center"] = msg[1]
        self.state["ir_right"] = msg[2]
        self.state["lolette"] = msg[3]
        self.state["scale"] = msg[4]

    def _process_rab_feedback(self, msg):
        id = msg[0]

        beacon = Beacon(id = id,
                distance = msg[2],
                angle = msg[1],
                data = [msg[3+i] for i in range(7)])

        self.beacons[id] = beacon
 


    def _send_evt(self, id, **kwarg):
        logger.debug("Event %s(%s) sent." % (id, str(kwarg)))
        self.med.SendEventName(id, kwarg.values())

    def _add_property(self, name, writable):

        def getter(self):
            if name in self.state:
                return self.state[name]
            else:
                return None

        def setter(self, val):
            self._send_evt(name, value = val)

        if writable:
            setattr(self.__class__, name, property(getter, setter)) 
        else:
            setattr(self.__class__,name, property(getter))


    def _init_accessors(self):

        for name, writable in self.STATE.items():
            # the creation of the setters,getters must take place in a separate
            # function, else they all refer to the same field name... Python internals...
            self._add_property(name, writable)

class Beacon:

    OBSOLETE_AFTER = 5 #seconds

    def __init__(self, id, distance, angle, data):

        self.update = time.time()

        self.id = id

        self.rawangle = math.pi * angle / 1800.

        self.angle = math.atan2(
                        math.sin(self.rawangle),
                        math.cos(self.rawangle)
                        )

        self.distance = distance / 1000.
        self.strength = 1. / ((distance/1000.)**2)

        self.data = data

        self.own_orientation = - math.pi * data[0] / 1800. + 2 * math.pi

    def obsolete(self):
        if time.time() - self.update > self.OBSOLETE_AFTER:
            return True
        else:
            return False
