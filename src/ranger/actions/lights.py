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
@lock(LEDS, wait=False)
def blush(robot, times = 2):
    # LEDs IDs
    left_chick = [14, 15, 20, 21, 22, 26, 27]
    right_chick = [158, 159, 164, 165, 166, 170, 171]

    SPEED = 5
    STEPS = 10
    try:
        robot.turn_off_leds()

        r,g,b = (200, 30, 128)
        i = 0
        inc = 1.
        for j in range(times):
            for k in range(2 * STEPS):
                i += inc
                if i == 0:
                    inc = 1
                if i == STEPS:
                    inc = -1

                factor = float(i) / STEPS
                for id in left_chick + right_chick:
                    robot.set_led(id, (r * factor, g * factor, b * factor))
                robot.sleep(0.1/SPEED)

        robot.turn_off_leds()

    except ActionCancelled:
        robot.turn_off_leds()



@action
@lock(LEDS)
def pulse(robot, id, color, speed = 5):

    STEPS = 10
    try:
        r,g,b = color
        i = 0
        inc = 1.
        while True:
            i += inc
            if i == 0:
                inc = 1
            if i == STEPS:
                inc = -1

            factor = float(i) / STEPS
            robot.set_led(id, (r * factor, g * factor, b * factor))
            robot.sleep(0.1/speed)

    except ActionCancelled:
        robot.set_led(id, (0,0,0))

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
