import logging; logger = logging.getLogger("ranger.sound")
from ranger.decorators import action, lock
from ranger.resources import *

from playwave import playwave

@action
@lock(AUDIO)
def playsound(robot, file):
    playwave(file)

