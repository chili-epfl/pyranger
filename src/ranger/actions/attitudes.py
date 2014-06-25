import logging; logger = logging.getLogger("ranger.attitudes")
import time
import math
from random import uniform as rand
from random import choice as rand_in
from robots.decorators import action, lock
from robots.signals import ActionCancelled

from ranger.res import *
from ranger import Ranger # for eyelids constants

@action
def idle(robot):

    try:
        while True:
            robot.sleep(rand(5, 10))
            if not robot.state.asleep:
                val = rand_in([-75,-60, 60, 75])
                robot.placeeyes((val, 0))
                robot.sleep(1)
                robot.placeeyes((0, 0))


    except ActionCancelled:
        robot.eyes(eyes = (0, 0))


@action
def active_wait(robot):

    try:
        robot.idle()

        while True:
            eyes = rand_in(range(4))
            if eyes == 1:
                robot.sneak_in()

            robot.turn(rand(0.3, 0.7)).wait()
            robot.sleep(1)
            robot.turn(-2 * rand(0.3, 0.7)).wait()
            robot.sleep(1)
            robot.turn(rand(0.3, 0.7)).wait()
            robot.sleep(rand(5,10))

    except ActionCancelled:
        pass

@action
@lock(EYES)
def wakeup(robot):

    try:
        robot.eyes(eyes = (0, 0))

        robot.eyes(left_lid = (rand(18, 25), rand(18,25)))
        robot.sleep(rand(0.6, 0.7))
        robot.eyes(right_lid = (rand(18, 25), rand(18,25)))
        robot.sleep(rand(0.6, 0.7))
        robot.eyes(lids = Ranger.eyelids.OPEN)

    except ActionCancelled:
        robot.eyes(eyes = (0, 0),
                   lids = Ranger.eyelids.OPEN)

@action
@lock(EYES)
def fall_asleep(robot):

    try:
        robot.eyes(left_lid = (rand(18, 25), rand(18,25)))
        robot.sleep(rand(0.6, 0.7))
        robot.eyes(right_lid = (rand(18, 25), rand(18,25)))
        robot.sleep(rand(0.6, 0.7))
        robot.eyes(lids = Ranger.eyelids.CLOSED)

    except ActionCancelled:
        robot.eyes(eyes = (0, 0),
                   lids = Ranger.eyelids.OPEN)


@action
@lock(LIDS)
def angry(robot, level = 0.5):

    try:
        robot.eyes(lids = (40 - (level * 20), 90))
    except ActionCancelled:
        robot.eyes(lids = Ranger.eyelids.OPEN)


@action
@lock(LIDS)
def sick(robot):

    try:
        robot.eyes(lids = (100, 30))
    except ActionCancelled:
        robot.eyes(lids = Ranger.eyelids.OPEN)

@action
@lock(LIDS)
def curious(robot):

    try:
        robot.eyes(lids = (40, 0))
    except ActionCancelled:
        robot.eyes(lids = Ranger.eyelids.OPEN)


@action
@lock(LIDS)
def blink(robot, nb = 1):

    try:
        llid, rlid = robot.state.eyelids

        for i in range(nb):
            robot.eyes(lids = Ranger.eyelids.CLOSED)
            robot.sleep(rand(0.2, 0.3))
            robot.eyes(left_lid = llid,
                    right_lid = rlid)
            robot.sleep(rand(0.2, 0.3))
    except ActionCancelled:
        robot.eyes(lids = Ranger.eyelids.OPEN)

@action
def background_blink(robot, focused = False):
    """ Continuous blink pattern, to be typically run in the background.

    Note that the LID resource is locked only during the blink itself,
    not between blinks.
    """

    # based on http://en.wikipedia.org/wiki/Blink, blink rate: ~every 6sec
    # and ~ x3 when focused
    try:
        while True:
            if focused:
                robot.sleep(12 + rand(-2, 2))
            else:
                robot.sleep(6 + rand(-2, 2))

            robot.blink()

    except ActionCancelled:
        pass

@action
@lock(EYES)
def lost(robot):

    try:
        for i in range(3):
            robot.eyes(rand_in([(90,90), (50, 90), (-90, 90), (-90, -90), (90, -90), (0, 90), (90, 0), (50, -50), (-50, 20)]))
            robot.sleep(rand(0.6, 1.2))
            with EYES:
                robot.blink(rand_in([1,2]))

        while True:
            robot.eyes((0, -70))
            robot.sleep(rand(2,5))
            robot.eyes(rand_in([(90,90), (50, 90), (-90, 90), (-90, -90), (90, -90), (0, 90), (90, 0), (50, -50), (-50, 20)]))
            robot.sleep(rand(1.6, 2))
            with EYES:
                robot.blink(rand_in([1,2]))


    except ActionCancelled:
        robot.eyes((0, 0))
        robot.eyes(lids = Ranger.eyelids.OPEN)


@action
@lock(LIDS)
def wink(robot, side = "right"):

    try:
        llid, rlid = robot.state.eyelids
        
        if side.lower() == "right":
            robot.eyes(right_lid = (0,0))
        else:
            robot.eyes(left_lid = (0,0))

        robot.sleep(rand(0.1, 0.3))

        robot.eyes(left_lid = llid,
                   right_lid = rlid)

    except ActionCancelled:
        robot.eyes(lids = Ranger.eyelids.OPEN)





@action
@lock(EYES)
def sneak_in(robot):

    try:
        robot.eyes(lids = (rand(18, 25), rand(18,25)))
        robot.sleep(rand(0.6, 0.7))
        robot.eyes(eyes = (rand(55,75), 0),
                   lids = (rand(18, 25), rand(18,25)))

        robot.sleep(rand(1.0,1.5))

        robot.eyes(eyes = (rand(-55,-75), 0),
                   lids = (rand(18, 25), rand(18,25)))

        robot.sleep(rand(1.0, 1.5))

        robot.eyes(eyes = (rand(55,75), 0),
                   lids = (rand(18, 25), rand(18,25)))

        robot.sleep(rand(1.0, 1.5))
        robot.eyes((0,0), Ranger.eyelids.OPEN)

    except ActionCancelled:
        robot.eyes((0, 0))

@action
def look_at_caresses(robot):

    try:
        while True:
            if robot.state.touches.is_touched_left():
                robot.eyes((90, 50))
                robot.sleep(rand(0.6,2))
                robot.eyes((0,0))
                robot.sleep(rand(2, 5))
            elif robot.state.touches.is_touched_right():
                robot.eyes((-90, 50))
                robot.sleep(rand(0.6,2))
                robot.eyes((0,0))
                robot.sleep(rand(2, 5))

            robot.sleep(0.3)
    except ActionCancelled:
        pass

