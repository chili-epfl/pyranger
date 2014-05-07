import logging; logger = logging.getLogger("ranger.light")
from robots.decorators import action, lock
from ranger.res import *

@action
@lock(LEDS)
def lightpattern(robot, pattern, repeat = False):

    robot.led_pattern(pattern, repeat)


