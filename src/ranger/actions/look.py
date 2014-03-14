import logging; logger = logging.getLogger("ranger.look")
import time
import math
from random import uniform as rand
from ranger.decorators import action, lock
from ranger.resources import *
from ranger.signals import ActionCancelled

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
        robot.eyes(0,0)


def placeeyes(robot, pose):
    pan, tilt = robot.pose.pantilt(pose, "eyes_link")
    if pan < -math.pi/2 or pan > math.pi/2:
        #out of field of view!
        #TODO: turn the robot? -> maybe yes if the WHEELS are not locked?
        return

    # vertical field of view
    tilt = clamp(tilt, -math.pi/4, math.pi/3)

    x = pan/(math.pi/2) * 100
    y = tilt/(math.pi/2) * 100
    robot.eyes(x, y)


@action
@lock(EYES)
def openeyes(robot):
    robot.eyes(0,0, l_upper_lid = 100)

@action
@lock(EYES)
def closeeyes(robot):
    robot.eyes(0,0, l_upper_lid = 0)

@action
@lock(EYES)
def sneak_in(robot):

    robot.eyes(0, 0, 0, 0, rand(18, 25))
    time.sleep(rand(0.6, 0.7))
    robot.eyes(rand(55,75), 0,  l_upper_lid = rand(18, 25))
    time.sleep(rand(1.0,1.5))
    robot.eyes(rand(-55, -75), 0, l_upper_lid = rand(18,25))
    time.sleep(rand(1.0, 1.5))
    robot.eyes(rand(55,75), 0,  l_upper_lid = rand(18, 25))
    time.sleep(rand(1.0, 1.5))
    robot.eyes(0, 0)

