import logging; logger = logging.getLogger("ranger.sound")
from robots.decorators import action, lock
from robots.signals import ActionCancelled
from ranger.res import *

from playwave import WavePlayer

player = WavePlayer()

@action
@lock(AUDIO)
def playsound(robot, file):
    try:
        player.play(file)
    except ActionCancelled:
        player.stop()

