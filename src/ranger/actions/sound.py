import logging; logger = logging.getLogger("ranger.sound")

from robots.mw import ROS
from robots.concurrency import action, ActionCancelled
from robots.resources import lock
from ranger.res import *

#from playwave import WavePlayer

#player = WavePlayer()

#@action
#@lock(AUDIO)
#def playsound(robot, file):
#    try:
#        player.play(file)
#    except ActionCancelled:
#        player.stop()

class AudioPublisher:

    def __init__(self):

        import rospy
        from std_msgs.msg import String
        self.pub = rospy.Publisher('play_audio_file', String, queue_size=1)

    def play(self, file):
        self.pub.publish(file)

audio_publisher = None

@action
@lock(AUDIO)
def playsound(robot, file):

    if robot.supports(ROS):
        global audio_publisher

        if audio_publisher is None:
            audio_publisher = AudioPublisher()

        audio_publisher.play(file)
    else:
        logger.warning("No ROS, no sound!")
