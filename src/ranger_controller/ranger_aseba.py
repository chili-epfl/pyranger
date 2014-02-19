import logging; logger = logging.getLogger("aseba")

import math,time
import copy

from medulla import Medulla
from helpers import singleton
from data_conversion import *

import threading

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


class _RangerLowLevel():

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
        "eye_right_y":          True,   # y coord of right pupil, [-100,100]
        "freq_main":            False,  # update frequency of the main robot node
        "freq_neuil":           False,  # update frequency of the 'neuil' robot node
        "freq_rab":             False   # update frequency of the 'Range and Bearing' robot node
        }

    def __init__(self):

        self.state = {}
        self.beacons = {}

        self._init_accessors()

        self._update_rate = {"main": time.time(), 
                             "neuil": time.time(), 
                             "rab": time.time()}


        dummy = False
        # init medulla
        self.med = Medulla(dummy = dummy)
        #self.med = Medulla()
        nodes = self.med.get_nodes_list()
        if not dummy and len(nodes) != 3:
            logger.error("One of the Ranger Aseba node is not up!!")
            logger.debug("List of active nodes: {0}".format(nodes))
            raise Exception("Missing Aseba node")

        # (Re-)load the aesl scripts, mandatory for medulla to know about
        # the list of available events
        self.med.load_scripts("/home/lemaigna/src/ranger2/aseba/RangerMain.aesl")
        self._send_evt("enableFeedback", enable = 1)

        self.med.on_event("mainFeedback", self._process_main_feedback)
        self.med.on_event("neuilFeedback", self._process_neuil_feedback)
        self.med.on_event("receiverFeedback", self._process_rab_feedback)
        self.state["freq_main"] = 0.0
        self.state["freq_neuil"] = 0.0
        self.state["freq_rab"] = 0.0

        self.med_thread = threading.Thread(target=self.med.run)
        self.med_thread.start()

        self.get_full_state()

    def close(self):
        self.med.close()
        self.med_thread.join()

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

        while "accelerometer" not in self.state or \
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
            return [raw[0:9:3], raw[1:9:3], raw[2:9:3]]

        self.state["touch_left"] = decompress_touch(msg[10])
        self.state["touch_rear"] = decompress_touch(msg[11])
        self.state["touch_right"] = decompress_touch(msg[9])

        if with_encoders:
            self.state["encoders"] = [msg[12], msg[13], msg[14], msg[15]]

        self.state["charging"] = msg[12 if not with_encoders else 16]
        self.state["motor_current_left"] = msg[13 if not with_encoders else 17]
        self.state["motor_current_right"] = msg[14 if not with_encoders else 18]

        self.state["freq_main"] = self.med.events_freq["mainFeedback"]

    def _process_neuil_feedback(self, msg):
        self.state["ir_left"] = linear_interpolation(msg[0], GP2Y0A41SK0F)
        self.state["ir_center"] = linear_interpolation(msg[1], GP2Y0A02YK)
        self.state["ir_right"] = linear_interpolation(msg[2], GP2Y0A41SK0F)
        self.state["lolette"] = msg[3]
        self.state["scale"] = msg[4]

        self.state["freq_neuil"] = self.med.events_freq["neuilFeedback"]

    def _process_rab_feedback(self, msg):
        id = msg[0]

        beacon = Beacon(id = id,
                distance = msg[2],
                angle = msg[1],
                data = [msg[3+i] for i in range(7)])

        self.beacons[id] = beacon

        self.state["freq_rab"] = self.med.events_freq["receiverFeedback"]

    def _send_evt(self, id, **kwarg):
        logger.debug("Event %s(%s) sent." % (id, str(kwarg)))
        self.med.send_event(id, kwarg.values())

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
