import logging; logger = logging.getLogger("ranger.sound")
from ranger.decorators import action, lock
from ranger.resources import *
from ranger.signals import ActionCancelled

from playwave import WavePlayer

player = WavePlayer()

@action
@lock(AUDIO)
def playsound(robot, file):
    try:
        player.play(file)
    except ActionCancelled:
        player.stop()

