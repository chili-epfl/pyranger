#! /usr/bin/env python

import logging
from ranger.helpers.ansistrm import ColorizingStreamHandler
toplogger = logging.getLogger("ranger")
toplogger.setLevel(logging.INFO)
#console = logging.StreamHandler()
console = ColorizingStreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
toplogger.addHandler(console)

logger = logging.getLogger("ranger.scenario")
import time

from ranger import get_robot
from ranger.res import ID, SOUNDS, PATTERNS
from ranger.introspection import introspection

##################################################################################
##################################################################################
#                             BASE LINE BEHAVIOUR
##################################################################################


with get_robot(dummy = True) as robot:

    def on_lolette():
        logger.info("Lolette is back!")
        robot.cancel_all()
        robot.goto("station").wait()
        robot.closeeyes()
        robot.dock_for_charging()

    def on_lolette_removed():
        logger.info("Lolette removed!")
        robot.cancel_all()
        robot.openeyes()

        beacon_found = False
        while not beacon_found:
            beacon_found = robot.look_for_beacon(ID["BEACON"]).result()
        robot.goto(ID["BEACON"]).wait()
        #TODO: set the target orientation

    def on_toy_added():
        logger.info("Toy added!")
        robot.playsound(SOUNDS["toy_in"])
        robot.lightpattern(PATTERNS["toy_in"])

    def on_toy_removed():
        logger.info("Toy removed!")
        robot.playsound(SOUNDS["toy_out"])
        robot.lightpattern(PATTERNS["toy_out"])



    logger.info("Ok! Let's start!")
    logger.info("Waiting for the lolette to be removed...")
    robot.on("lolette", value = True).do(on_lolette)
    robot.on("lolette", value = False).do(on_lolette_removed)
    robot.on("scale", increase = 100).do(on_toy_added)
    robot.on("scale", decrease = 100).do(on_toy_removed)

    try:
        while True:
            time.sleep(0.5)

            if introspection:
                introspection.ping()
    except KeyboardInterrupt:
        robot.events.close()
        robot.cancel_all()

    logger.info("Byebye")
