#! /usr/bin/env python

import logging
import time

from ranger import Ranger
from robots.decorators import action, lock
from ranger.res import ID, SOUNDS, PATTERNS
from robots.introspection import introspection

logger = logging.getLogger("ranger.scenario")
logging.getLogger("ranger.aseba").setLevel(logging.DEBUG-1) # effectively silent aseba debug messages

##################################################################################
##################################################################################
#                             BASE LINE BEHAVIOUR
##################################################################################

current_action = None
blinking = None

@action
def on_lolette(robot):
    logger.info("Lolette is back!")

    robot.blink()

    dock_ok = robot.dock_for_charging().result()
    if dock_ok:
        blinking.cancel()
        robot.fall_asleep().wait()
        robot.state["asleep"] = True

@action
def on_lolette_removed(robot):
    logger.info("Lolette removed!")

    if robot.state["asleep"]:
        robot.state["asleep"] = False
        robot.wakeup().wait()

    robot.goto([1.8, 2.2, 0, 0, 0,-0.05, 1]).wait()
    robot.blink(2)
    robot.active_wait().wait()
    #beacon_found = False
    #while not beacon_found:
    #    beacon_found = robot.look_for_beacon(ID.BEACON).result()
    #robot.goto(ID.BEACON).wait()
    #TODO: set the target orientation

@action
def on_toy_added(robot):
    logger.info("Toy added!")
    robot.playsound(SOUNDS["toy_in"])
    robot.lightpattern(PATTERNS["toy_in"])

@action
def on_toy_removed(robot):
    logger.info("Toy removed!")
    robot.playsound(SOUNDS["toy_out"])
    robot.lightpattern(PATTERNS["toy_out"])

class runner:
    def __init__(self, fn):
        self.fn = fn

    def __call__(self, robot):
        global current_action

        if current_action:
            current_action.cancel()

        current_action = self.fn(robot)

with Ranger() as robot:

    # Turn on DEBUG logging
    robot.debug()

    robot.state["asleep"] = True

    robot.show_battery()
    blinking = robot.background_blink()

    logger.info("Ok! Let's start!")
    logger.info("Waiting for the lolette to be removed...")
    robot.every("lolette", becomes = True).do(runner(on_lolette))
    robot.every("lolette", becomes = False).do(runner(on_lolette_removed))
    robot.every("scale", increase = 0.1, max_firing_freq = 0.3).do(on_toy_added)
    robot.every("scale", decrease = 0.1, max_firing_freq = 0.3).do(on_toy_removed)

    try:
        while not robot.rosactions.is_shutdown():
            time.sleep(0.5)

            if introspection:
                introspection.ping()
    except KeyboardInterrupt:
        pass

    logger.info("Byebye")

