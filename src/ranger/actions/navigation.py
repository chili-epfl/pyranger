import time, math
from random import uniform as rand
from ranger.decorators import action, lock
from ranger.resources import *

EPSILON_RAD = 0.05

@action
@lock(WHEELS)
def dock_charging(robot):
    pass


@action
@lock(WHEELS)
def face_beacon(robot, beacon_id):

    turning = False
    while(robot.beacons[beacon_id].orientation > EPSILON_RAD and \
          robot.beacons[beacon_id].orientation < (2 * math.pi) - EPSILON_RAD):

        if not turning:
            robot.speed(w = 0.1) # rad.s^-1
            turning = True
        time.sleep(0.1)

    robot.speed(0)


@action
@lock(WHEELS)
def face(robot, x, y, speed = 0.01, back = False):
    """ Rotates the robot to face a given target point, in /odom frame.

    :param speed: rotation speed, in rad.s^-1
    :param back: (default: False) If true, face 'backwards' (ie turn back to target)
    """
    angle = robot.angleto(x, y)
    if back:
        angle = robot.normalize_angle(angle + math.pi)

    duration = abs(angle / speed)

    robot.speed(w = speed if angle > 0 else -speed)

    time.sleep(duration)

@action
def back(robot, x, y, speed = 0.01):
    robot.face(x,y, speed, back = True).result()

@action
@lock(WHEELS)
def goto(robot, x, y, v = 1., w = 0.01, epsilon = 0.1, backwards = False):
    """

    :param x, y: target destination, in meters in /odom frame
    :param v: desired linear velocity
    :param w: desired rotation velocity
    :param epsilon: the acceptable error for the target to be reached, in meters
    :param backwards: if true, move backwards
    """

    WHEELS.release()
    action = robot.face(x, y, w, back = backwards)
    #waits until the robot faces the destination
    action.result()
    WHEELS.acquire()

    robot.speed(v = v if not backwards else -v)

    prev_dist = robot.distanceto(x, y)
    
    while True:
        dist = robot.distanceto(x, y)
        if dit < epsilon:
            return
        if dist > prev_dist: # we are not getting closer anymore!

            robot.speed(0)
            WHEELS.release()
            action = robot.face(x, y, w, back = backwards)
            #waits until the robot faces the destination
            action.result()
            WHEELS.acquire()
            robot.speed(v = v if not backwards else -v)

        time.sleep(0.1)
