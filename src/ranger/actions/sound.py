import logging; logger = logging.getLogger("ranger.sound")
from ranger.decorators import action, lock
from ranger.resources import *

@action
@lock(AUDIO)
def playsound(robot, file):
    logger.error("playsound not yet implemented!")


