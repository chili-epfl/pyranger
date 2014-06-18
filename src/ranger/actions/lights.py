import logging; logger = logging.getLogger("ranger.light")

from robots.decorators import action, lock
from robots.signals import ActionCancelled
from ranger.res import *
from ranger.helpers import colors

NB_ROWS = 6
NB_COLS = 31

@action
@lock(LEDS)
def lightpattern(robot, pattern, repeat = False):

    robot.led_pattern(pattern, repeat)

@action
@lock(LEDS)
def lightbar(robot, level = 0.5, ramp = colors.BLUE_TO_RED, vertical = True, with_modifier = True):
    
    steps = NB_ROWS if vertical else NB_COLS

    if with_modifier:
        ramp = colors.get_ramp(ramp, robot)
    else:
        ramp = colors.get_ramp(ramp)

    try:
        robot.turn_off_leds()
        for i in range(int(round(steps * level))):
            if vertical:
                robot.set_led_row(i, ramp[int(float(i)*len(ramp)/steps)])
            else:
                robot.set_led_col(i, ramp[int(float(i)*len(ramp)/steps)])

    except ActionCancelled:
        robot.turn_off_leds()
