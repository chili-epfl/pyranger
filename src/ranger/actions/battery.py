import logging; logger = logging.getLogger("ranger.battery")
from robots.decorators import action, lock
from robots.signals import ActionCancelled
from ranger.res import LEDS

from ranger.lowlevel.ranger_aseba import BATTERY_MAX_LEVEL, BATTERY_LOW_THRESHOLD

GREEN= (0, 20, 0)
ORANGE= (20, 20, 0)
RED= (20, 0, 0)
OFF= (0, 0, 0)

# right back corner
LEDS_RANGE = range(60,66)

@action
@lock(LEDS)
def show_battery(robot):

    prev_state = 0
    try:
        while True:
            charging = robot.state.charging
            bat_level = float(robot.state.battery - BATTERY_LOW_THRESHOLD) / (BATTERY_MAX_LEVEL - BATTERY_LOW_THRESHOLD)

            if charging:
                for i in LEDS_RANGE:
                    robot.set_led(i, GREEN)
                    robot.sleep(0.5)
                robot.turn_off_leds()
            else:
                # last LED blinking in red
                if bat_level <= 0:
                    state = 0
                    robot.set_led(LEDS_RANGE[0], RED)
                    robot.sleep(0.3)
                    robot.set_led(LEDS_RANGE[0], OFF)
                    robot.sleep(0.3)
                elif 0 < bat_level < 0.2:
                    state = 1
                    for i in LEDS_RANGE[0:1]:
                        robot.set_led(i, RED)
                    robot.sleep(0.3)
                elif 0.2 <= bat_level < 0.4:
                    state = 2
                    for i in LEDS_RANGE[0:2]:
                        robot.set_led(i, ORANGE)
                    robot.sleep(0.3)
                elif 0.4 <= bat_level < 0.6:
                    state = 3
                    for i in LEDS_RANGE[0:3]:
                        robot.set_led(i, GREEN)
                    robot.sleep(0.3)
                elif 0.6 <= bat_level < 0.8:
                    state = 4
                    for i in LEDS_RANGE[0:4]:
                        robot.set_led(i, GREEN)
                    robot.sleep(0.3)
                else:
                    state = 5
                    for i in LEDS_RANGE:
                        robot.set_led(i, GREEN)
                    robot.sleep(0.3)
                
                if state != prev_state:
                    robot.turn_off_leds()
                    prev_state = state

    except ActionCancelled:
        robot.turn_off_leds()

