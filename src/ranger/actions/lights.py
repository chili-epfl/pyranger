import logging; logger = logging.getLogger("ranger.light")
from ranger.decorators import action, lock
from ranger.resources import *

@action
@lock(LEDS)
def lightpattern(robot, pattern, repeat = False):

    robot.led_pattern(pattern, repeat)


