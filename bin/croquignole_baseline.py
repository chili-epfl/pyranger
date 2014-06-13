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


@action
def on_lolette(robot):
    logger.info("Lolette is back!")
    robot.cancel_all_others()
    robot.blink()
    dock_ok = robot.dock_for_charging().result()
    if dock_ok:
        robot.closeeyes()

@action
def on_lolette_removed(robot):
    logger.info("Lolette removed!")
    robot.cancel_all_others()
    robot.openeyes()

    robot.goto([1.8, 2.2, 0, 0, 0,-0.05, 1]).wait()
    robot.blink(2)
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


with Ranger() as robot:

    # Turn on DEBUG logging
    robot.debug()

    logger.info("Ok! Let's start!")
    logger.info("Waiting for the lolette to be removed...")
    robot.every("lolette", becomes = True).do(on_lolette)
    robot.every("lolette", becomes = False).do(on_lolette_removed)
    robot.every("scale", increase = 100).do(on_toy_added)
    robot.every("scale", decrease = 100).do(on_toy_removed)

    try:
        while True:
            time.sleep(0.5)

            if introspection:
                introspection.ping()
    except KeyboardInterrupt:
        pass

    logger.info("Byebye")
