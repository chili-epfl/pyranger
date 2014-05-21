import logging; logger = logging.getLogger("ranger.navigation")

import time, math
from random import uniform as rand
from robots.decorators import action, lock
from robots.signals import ActionCancelled
from ranger.res import *

EPSILON_RAD = 0.05

@action
@lock(WHEELS)
def stop(robot):
    robot.speed(0)

@action
@lock(WHEELS)
def dock_for_charging(robot):

    if robot.state.charging:
        logger.info("Already docked on the charging station. Fine.")
        return

    with WHEELS:
        placement = robot.goto([0.5, 0, 0, 0, 0, 0, "station"])

        try:
            placement.wait()
        except ActionCancelled:
            placement.cancel()
            return

        approach = robot.move(-0.6, v=0.03)
        robot.on("charging", value = True).do(approach.cancel)

        try:
            approach.wait()
        except ActionCancelled:
            approach.cancel()
            return

        logger.info("Successfully docked on the charging station!")

@action
@lock(WHEELS)
def undock(robot):
    if not robot.state.charging:
        logger.info("Not docked (no charging). Nothing to do.")
        return
    
    try:
        with WHEELS:
            robot.move(0.4).wait()
    except ActionCancelled:
        pass

@action
def look_for_beacon(robot, beacon_id):

    def isseen():
        if beacon_id in robot.beacons and \
            not robot.beacons[beacon_id].obsolete() \
            and robot.beacons[beacon_id].valid:
                return True
        return False

    if isseen(): return True

    sneakin = None
    turning = None
    try:
        initial_orientation = robot.state.theta

        sneakin = robot.sneak_in()

        turning = robot.turn(-math.pi / 4)
        turning.wait()

        for i in range(3):
            if isseen(): break
            turning = robot.turn(math.pi / 2)
            turning.wait()
            if isseen(): break
            turning = robot.turn(-math.pi / 2)
            turning.wait()


        turning = robot.turn(robot.pose.angular_distance(robot.state.theta, initial_orientation))

        sneakin.cancel()

        turning.wait()


        if isseen():
            logger.info("Beacon %s seen." % beacon_id)
            return True
        else:
            logger.warning("Beacon %s not found." % beacon_id)
            return False
    except ActionCancelled:
        if turning:
            turning.cancel()
        robot.speed(0)
        if sneakin:
            sneakin.cancel()
        return False

@action
def face(robot, pose, w = 0.5, backwards = False):
    """ Rotates the robot to face a given target point.

    :param speed: rotation speed, in rad.s^-1
    :param backwards: (default: False) If true, face 'backwards' (ie turn back to target)
    """
    if robot.pose.distance(pose) < 0.1:
        logger.warning("Too close to target to face for meaningful rotation. Staying where I am.")
        return

    try:
        angle, _  = robot.pose.pantilt(pose)

        if backwards:
            angle = robot.pose.normalize_angle(angle + math.pi)

        if abs(angle) < 0.02:
            logger.debug("Already facing %s. Fine." % pose)
            return

        logger.debug("Need to turn by {:.1f}° to face {:s}".format(180./math.pi*angle, pose))
        robot.turn(angle, w).result()

    except ActionCancelled:
        logger.debug("Action 'face' successfully cancelled.")
        # nothing more to do since the 'turn' sub-action takes care of setting the speed to 0

@action
def orient(robot, pose):
    """ Set the robot orientation so that it aligns with the given
    pose orientation.

    """
    
    try:
        rx,ry,rz = robot.pose.euler(robot.pose.inframe(pose, "base_link"))

        if abs(rz) < 0.02:
            logger.debug("Orientation already fine.")
            return

            logger.debug("Need to turn by {:.1f}° to reach expected orientation".format(180./math.pi*rz))

        robot.turn(rz)
    except ActionCancelled:
        # nothing more to do since the 'turn' sub-action takes care of setting the speed to 0
        pass

def eased_speed(max_speed, achieved):
    #TODO: proper ease in/ease out
    if achieved > 0.8:
        return max_speed * 0.5
    else:
        return max_speed


@action
@lock(WHEELS)
def move(robot, distance, v = 0.2, easing = True):
    """ Move forward (or backward if distance is negative) of a given distance.

    DO NOT CHECK FOR COLLISIONS!

    :param distance: distance to move, in meters
    :param v: (default: 0.5) linear velocity
    """
    last_speed = 0

    # if the distance is small, it's useless to move too quickly
    max_speed = max(0.01, min(abs(distance)/2, abs(v)))

    try:
        initial_pose = robot.pose.myself()

        while True:
            total_distance = robot.pose.distance(initial_pose)
            achieved = total_distance / abs(float(distance))
            logger.debug("Moved {:.1f}m ({:.1f}% of target)".format(total_distance, achieved))

            if achieved >= .95:
                robot.speed(v=0)
                return

            if easing:
                speed = eased_speed(max_speed, achieved)
            else:
                speed = max_speed
 
            speed = speed if distance > 0 else -speed

            if speed != last_speed:
                robot.speed(v=speed)
                last_speed = speed

            time.sleep(0.1)

    except ActionCancelled:
        logger.debug("Action 'move' successfully cancelled.")
    finally:
        robot.speed(0)

@action
@lock(WHEELS)
def turn(robot, angle, w = 0.5, easing = True):
    """ Turns of a given angle.

    DO NOT CHECK FOR COLLISIONS!

    :param angle: angle to turn, in radians. Taken as it is (ie, no angle % 2.pi occurs).
    :param w: (default: 0.5) rotation velocity
    """
    last_theta = robot.state.theta
    total_rotation = 0
    last_speed = 0

    # if the angle is small, it's useless to turn too quickly
    max_speed = max(0.1, min(abs(angle), abs(w)))

    try:
        while True:
            theta = robot.state.theta
            total_rotation += robot.pose.angular_distance(last_theta, theta)
            last_theta = theta 
            achieved = total_rotation / float(angle)
            logger.debug("Turned by {:.1f}rad ({:.1f}% of target)".format(total_rotation, achieved))

            if achieved > 0.95:
                robot.speed(w=0)
                return

            if easing:
                speed = eased_speed(max_speed, achieved)
            else:
                speed = max_speed
            speed = speed if angle > 0 else -speed

            if speed != last_speed:
                robot.speed(w = speed)
                last_speed = speed

            time.sleep(0.1)

    except ActionCancelled:
        logger.debug("Action 'turn' successfully cancelled.")
    finally:
        robot.speed(0)


@action
@lock(WHEELS)
def goto(robot, 
        pose, 
        v = 0.2, w = 0.5, 
        epsilon = 0.1, epsilon2 = 0.05,
        distance_to_target = 0.0, 
        ignore_orientation = False):
    """
    Move to a given destination.

    The algorithm is currently naive: the robot first face the destination,
    then moves straight towards it, and finally turns on itself until the
    desired final orientation is reached (except if ignore_orientation is set to True).


    :param pose: target destination (including the desired orientation!)
    :param v: desired linear velocity
    :param w: desired rotation velocity
    :param epsilon: the acceptable error for the target to be reached, in meters
    :param epsilon2: the acceptable orientation error for the target to be reached, in radians
    :param distance_to_target: desired final distance to target (default: 0)
    :param ignore_orientation: if true, do not set the final robot orientation
    """

    try:
        if robot.state.charging:
            # undock first!
            with WHEELS:
                robot.undock().wait()

        with WHEELS:
            action = robot.face(pose, w)
            #waits until the robot faces the destination
            action.result()

        distance = robot.pose.distance(pose)
        with WHEELS:
            motion = robot.move(distance, v)

            prev_dist = distance
            while True:
                dist = robot.pose.distance(pose)
                angle, _ = robot.pose.pantilt(pose)

                if dist < epsilon + distance_to_target:
                    motion.cancel()
                    logger.info("Reached target")
                    break

                if abs(angle) > epsilon2:
                    logger.debug("Correcting heading a bit...")
                    motion.cancel()
                    robot.turn(angle, w).wait()
                    motion = robot.move(dist, v)

                if dist > prev_dist: # we are not getting closer anymore!

                    logger.warning("Missed! Trying to face again my target...")
                    motion.cancel()
                    robot.turn(angle, w).wait()
                    motion = robot.move(dist, v)


                #if robot.state.bumper:
                #    logger.warning("Bumped into something!")
                #    robot.speed(0)
                #    with WHEELS:
                #        robot.resolve_collision().result()
                #        face(pose, w, backwards = backwards).result()
                #    robot.speed(v = v if not backwards else -v)

                prev_dist = dist
                time.sleep(0.1)

            if not ignore_orientation:
                robot.orient(pose).wait()

    except ActionCancelled:
        logger.warning("Goto action cancelled. Stopping here.")
    finally:
        robot.speed(0)

@action
def resolve_collision(robot):
    """ Assumes the robot in colliding with something (bumper = True), 
    tries to resolve the situation.

    Procedure:
        - if central IR does not detect anything, assume it's a low obstacle on the ground
        - if central IR detect smthg, wait 2 sec, and check again. Obstacle may have been removed.
        - else, try to bypass the obstacle by:
            - moving backward, 
            - checking with left and right IR what look like the most promising direction
            - slowly move in that direction
    """
    WAIT_FOR_REMOVAL_DURATION = 2 #sec
    DISTANCE_TO_OBSTACLE_THRESHOLD = 0.15 #distance to consider the collinding obstacle seen

    #TODO: EMOTION
    logger.info("Trying to resolve the collision...")
    ir_dist = robot.state.ir_center
    if ir_dist < DISTANCE_TO_OBSTACLE_THRESHOLD:
        obstacle_seen = True
        logger.info("Obstacle seen at %s" % ir_dist)
    else:
        obstacle_seen = False
        logger.info("Obstacle not seen by IR. Low/thin object?")
    
    if obstacle_seen:
        # wait 2 sec, in case the obstacle is removed
        start = time.time()
        while (time.time() - start) < WAIT_FOR_REMOVAL_DURATION:
            #TODO: EMOTION -> hope obstacle removed
            if robot.state.ir_center > DISTANCE_TO_OBSTACLE_THRESHOLD:
                logger.info("Obstacle removed!")
                #consider us as free again! youpi!
                #TODO: EMOTION
                return
            time.sleep(0.2)

    robot.move(-0.2).result() # backwards 20 cm

    if robot.state.ir_left < robot.state.ir_right:
        #go right!
        robot.goto(x = 0, y = -1).result()
        robot.turn(math.pi).result()
    else:
        #go left!
        robot.goto(x = 0, y = 1).result()
        robot.turn(-math.pi).result()

