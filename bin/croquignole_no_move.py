#! /usr/bin/env python

import logging
import time
from random import uniform as rand

from robots.signals import ActionCancelled
from ranger import Ranger
from ranger.lowlevel.ranger_aseba import BATTERY_MIN_CHARGED_LEVEL, BATTERY_LOW_THRESHOLD
from robots.decorators import action, lock
from ranger.res import ID, SOUNDS, PATTERNS
from robots.introspection import introspection
from ranger.helpers import colors

logger = logging.getLogger("ranger.scenario")
logging.getLogger("ranger.aseba").setLevel(logging.DEBUG-1) # effectively silent aseba debug messages

##################################################################################
##################################################################################
#                             BASE LINE BEHAVIOUR
##################################################################################

current_action = None

@action
def on_lolette(robot):
    if robot.state.asleep:
        logger.info("I'm already sleeping!")
        return

    logger.info("Lolette is back! I can go to sleep!")
    robot.look_at_lolette()
    robot.sleep(0.5)
    robot.blink()
    robot.placeeyes((0,0))
    sleep = robot.fall_asleep()
    robot.lightbar(ramp=(128, 128, 0), vertical = True, speed = -0.5).wait()
    robot.lightbar(ramp=None, vertical = True, speed = -0.5)
    sleep.wait()
    robot.state.asleep = True

    pulse = None
    try:
        while True:
            if robot.state.charging:
                pulse = robot.pulse_row(0, (255, 255 ,0))
                while robot.state.charging and not robot.state.battery > BATTERY_MIN_CHARGED_LEVEL:
                    robot.sleep(1.)
                    logger.info("Battery at %dmV (charging up to %dmV)" % (robot.state.battery, BATTERY_MIN_CHARGED_LEVEL))
                pulse.cancel()

                if robot.state.battery > BATTERY_MIN_CHARGED_LEVEL: # charging complete!
                    logger.info("Ok, I'm good to go!")
                    pulse = robot.pulse_row(0, (0, 128 ,0))

            robot.sleep(.4)

    except ActionCancelled:
        if pulse:
            pulse.cancel()


@action
def on_lolette_removed(robot):
    if not robot.state.asleep:
        logger.info("I'm already awake!")
        return

    logger.info("Lolette removed! I wake up!")
    robot.state.asleep = False
    robot.up_down_row(colors.from_hls(rand(0,1),0.8,1), times = 2)
    robot.wakeup().wait()

    robot.undock()
    robot.active_wait().wait()

@action
def on_bumped(robot):
    if robot.state.asleep:
        logger.info("Don't wake me up, I sleep!")
        robot.eyes(left=(0,0), left_lid=(18,18))
        robot.sleep(1.)
        robot.eyes(left=(50,0))
        robot.sleep(1.)
        robot.closeeyes()
        return

    logger.info("Ouhouh! I bumped into something, or someone is driving me!")
    robot.state.asleep = False
    pulse = robot.pulse_row(0, (128, 0, 128))

    robot.sleep(1.)

    while abs(robot.state.v) > 0.01:
        robot.sleep(0.2)

    pulse.cancel()


@action
def on_toy_added(robot):
    if robot.state.asleep:
        logger.info("Toy added, but I'm sleeping...")
        return

    logger.info("Toy added!")
    robot.playsound(SOUNDS["toy_in"])
    #robot.lightpattern(PATTERNS["toy_in"])
    robot.lightbar(ramp=colors.RAINBOW, speed = 1).wait()
    robot.lightbar(ramp=None, speed = 1).wait()

@action
def on_toy_removed(robot):
    if robot.state.asleep:
        logger.info("Toy removed, but I'm sleeping...")
        return

    logger.info("Toy removed!")
    robot.playsound(SOUNDS["toy_out"])
    robot.lightpattern(PATTERNS["toy_out"])
    robot.sleep(1.)

@action
def on_battery_low(robot):
    robot.events.stop_all_monitoring()
    robot.cancel_all_others()
    logger.warning("Battery low! Bring me to the charging station!")
    robot.turn_off_leds()
    robot.playsound(SOUNDS["battery_low"])
    robot.closeeyes()
    robot.state.asleep = True

    pulse = robot.pulse_row(0, (255,0,0))
    while not robot.state.charging:
        robot.sleep(0.2)
    logger.info("Charging! Good!")
    pulse.cancel()

    pulse = robot.pulse_row(0, (255, 255 ,0))
    while not robot.state.battery > BATTERY_MIN_CHARGED_LEVEL:
        robot.sleep(1.)
        logger.info("Battery at %dmV (charging up to %dmV)" % (robot.state.battery, BATTERY_MIN_CHARGED_LEVEL))
    pulse.cancel()

    logger.info("Ok, I'm good to go! Move me a bit to restart the normal behaviours")
    pulse = robot.pulse_row(0, (0, 255 ,0))
    robot.state.asleep = False
    robot.openeyes()
    robot.blink()

    x = robot.state.x
    while not abs(x - robot.state.x) > 0.1:
        robot.sleep(0.2)
    pulse.cancel()

    init_robot(robot)

class runner:
    def __init__(self, fn):
        self.fn = fn

    def __call__(self, robot):
        global current_action

        if current_action:
            current_action.cancel()

        current_action = self.fn(robot)
        return current_action


def init_robot(robot):

    #robot.show_battery()
    robot.background_blink()
    robot.look_at_caresses()

    robot.every("lolette", becomes = True).do(runner(on_lolette))
    robot.every("lolette", becomes = False).do(runner(on_lolette_removed))
    robot.every("scale", increase = 0.3).do(on_toy_added)
    robot.every("scale", decrease = 0.3).do(on_toy_removed)
    robot.every("battery", below = BATTERY_LOW_THRESHOLD).do(on_battery_low)
    #robot.every("bumper", becomes = True).do(runner(on_bumped))



with Ranger() as robot:

    # Turn on DEBUG logging
    robot.debug()

    logger.info("Ok! Let's start!")
    init_robot(robot)
    robot.state.asleep = True # force asleep to effectively wake up :-)
    runner(on_lolette_removed)(robot)
    logger.info("I'm awake, waiting for interactions!")

    try:
        while not robot.rosactions.is_shutdown():
            if robot.state.bumper:
                runner(on_bumped)(robot).wait()
                if not robot.state.asleep:
                    current_action = robot.active_wait()

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    logger.info("Byebye")
