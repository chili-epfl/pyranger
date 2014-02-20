import time
from random import uniform as rand
from ranger.decorators import action, lock
from ranger.resources import *

@action
@lock(EYES)
def sneak_in(robot):

    robot.eyes(0, 0, 0, 0, rand(18, 25))
    time.sleep(rand(0.6, 0.7))
    robot.eyes(rand(55,75), 0,  l_upper_lid = rand(18, 25))
    time.sleep(rand(1.0,1.5))
    robot.eyes(rand(-55, -75), 0, l_upper_lid = rand(18,25))
    time.sleep(rand(1.0, 1.5))
    robot.eyes(rand(55,75), 0,  l_upper_lid = rand(18, 25))
    time.sleep(rand(1.0, 1.5))
    robot.eyes(0, 0)

