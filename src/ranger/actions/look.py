import logging; logger = logging.getLogger("ranger.look")
import time
import math
from random import uniform as rand
from robots.decorators import action, lock
from robots.signals import ActionCancelled

from ranger.res import *
from ranger import Ranger # for eyelids constants

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


@action
@lock(EYES)
def lookat(robot, pose):
    placeeyes(robot, pose)

@action
@lock(EYES)
def track(robot, pose):
    try:
        while True:
            placeeyes(robot, pose)
            time.sleep(0.2)
    except ActionCancelled:
        robot.eyes((0, 0))


def placeeyes(robot, pose):
    pan, tilt = robot.pose.pantilt(pose, "eyes_link")
    if pan < -math.pi/2 or pan > math.pi/2:
        #out of field of view!
        #TODO: turn the robot? -> maybe yes if the WHEELS are not locked?
        logger.warning("Desired gaze target out of field of view! pan: %s°, tilt:%s°" % (pan * 180 / math.pi, tilt * 180 / math.pi))
        robot.eyes((0, 0))
        return

    # vertical field of view
    tilt = clamp(tilt, -math.pi/4, math.pi/3)

    x = pan/(math.pi/2) * 100
    y = tilt/(math.pi/2) * 100
    robot.eyes((x, y))


@action
@lock(EYES)
def openeyes(robot, force = False):
    if not robot.eyelids == Ranger.eyelids.OPEN:
        if force:
            lids = Ranger.eyelids.OPEN
        else:
            lids = (v * robot.innerstate.energy for v in Ranger.eyelids.OPEN)
        robot.eyes((0,0), lids)

@action
@lock(EYES)
def closeeyes(robot):
    if not robot.eyelids == Ranger.eyelids.CLOSED:
        robot.eyes((0,0), Ranger.eyelids.CLOSED)

