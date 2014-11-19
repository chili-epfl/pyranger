import logging; logger = logging.getLogger("ranger.look")
import time
import math
from random import uniform as rand
from robots.concurrency import action, ActionCancelled
from robots.resources import lock

from ranger.res import *
from ranger import Ranger # for eyelids constants

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

@action
@lock(PUPILS)
def placeeyes(robot, left, right = None):
    if right is None:
        robot.eyes(left)
    else:
        robot.eyes(left = left, right = right)

@action
def look_at_lolette(robot):
    robot.placeeyes(left=(-100,-100), right=(100,-100)).wait()

@action
@lock(PUPILS)
def rolleyes(robot, times = 2):
    r = 70
    robot.eyes((r,0))
    time.sleep(0.1)

    for n in range(times):
        for i in range(20):
            theta = i * 2 * math.pi / 20
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            robot.eyes((x,y))
            time.sleep(0.05)

    
    robot.eyes((0,0))

@action
@lock(PUPILS)
def lookat(robot, pose):
    place_eyes_towards(robot, pose)

@action
@lock(PUPILS)
def track(robot, pose):
    try:
        while True:
            place_eyes_towards(robot, pose)
            time.sleep(0.2)
    except ActionCancelled:
        robot.eyes((0, 0))


def place_eyes_towards(robot, pose):
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
            lids = ((l * robot.innerstate.energy, r * robot.innerstate.energy) for l,r in Ranger.eyelids.OPEN)
        robot.eyes((0,0), lids)

@action
@lock(EYES)
def closeeyes(robot):
    if not robot.eyelids == Ranger.eyelids.CLOSED:
        robot.eyes((0,0), Ranger.eyelids.CLOSED)

