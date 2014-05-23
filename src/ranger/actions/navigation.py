import logging; logger = logging.getLogger("ranger.navigation")

import time, math
from random import uniform as rand, choice
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
            return

        try:
            approach = robot.move(-0.4, v=0.05)
            robot.on("charging", value = True).do(approach.cancel)

            approach.wait()
        except ActionCancelled:
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
def sweep(robot):

    try:

        robot.sneak_in()

        robot.turn(-math.pi / 4).wait()
        robot.turn(math.pi / 2).wait()
        robot.turn(-math.pi / 4).wait()

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

    try:
        # already in sight? simply face it
        if isseen():
            logger.info("Beacon %s seen." % beacon_id)
            robot.face(beacon_id).wait()
            return True
    except ActionCancelled:
        return True

    try:
        # if a previous valid position is known, face it to start
        if beacon_id in robot.beacons and \
            not robot.beacons[beacon_id].obsolete() \
            and robot.beacons[beacon_id].last_valid_pose:
                robot.face(beacon_id)
    except ActionCancelled:
        return False


    rotation_direction = choice([-0.8, -1.2, 0.8, 1.2])

    sweep = None
    try:

        while True:
            sweep = robot.sweep()
            while not sweep.done():
                if isseen():
                    logger.info("Beacon %s seen." % beacon_id)
                    sweep.cancel()
                    robot.face(beacon_id).wait()
                    robot.openeyes()
                    robot.blink().wait()
                    return True

            logger.warning("Beacon %s not yet found :-(" % beacon_id)
            robot.turn(rotation_direction * 2/3 * math.pi)

    except ActionCancelled:
        return False


@action
def track_beacon(robot, beacon_id):

    try:
        while True:
            robot.look_for_beacon(beacon_id).wait()
            robot.sleep(0.3)
    except ActionCancelled:
        pass


@action
@lock(WHEELS)
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

        logger.debug("Need to turn by {:.1f}° to face {}".format(180./math.pi*angle, pose))

        with WHEELS:
            robot.turn(angle, w).result()

    except ActionCancelled:
        pass

@action
@lock(WHEELS)
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

        with WHEELS:
            robot.turn(rz).wait()

    except ActionCancelled:
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

            robot.sleep(0.1)

    except ActionCancelled:
        robot.speed(0)

@action
@lock(WHEELS)
def turn(robot, angle, w = 0.5, easing = True):
    """ Turns of a given angle.

    DO NOT CHECK FOR COLLISIONS!

    :param angle: angle to turn, in radians. Taken as it is (ie, no angle % 2.pi occurs).
    :param w: (default: 0.5) rotation velocity
    """

    if abs(angle) < 0.02:
        logger.info("Rotation angle < 1°: skipping it")
        return

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
        # undock first!
        with WHEELS:
            logger.debug("goto: undock")
            robot.undock().wait()

        with WHEELS:
            logger.debug("goto: face target")
            action = robot.face(pose, w)
            #waits until the robot faces the destination
            action.result()

        distance = robot.pose.distance(pose)
        with WHEELS:
            logger.debug("goto: start moving towards target")
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
                    logger.debug("goto: Correcting heading a bit...")
                    motion.cancel()
                    robot.turn(angle, w).wait()
                    motion = robot.move(dist, v)

                if dist > prev_dist: # we are not getting closer anymore!

                    logger.info("goto: Target missed! Trying to face again my target...")
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
        robot.speed(0)

@action
@lock(WHEELS)
def goto_beacon(robot, beacon_id):

    try:
        with WHEELS:
            robot.undock().wait()

            robot.look_for_beacon(beacon_id).wait()

            if robot.pose.distance(beacon_id) > 1:
                motion = robot.goto(beacon_id,
                                    distance_to_target = 1,
                                    ignore_orientation = True)

                while not motion.done():
                    robot.sleep(0.5)
                    if robot.pose.distance(beacon_id) > 1.5 and not robot.beacons[beacon_id].valid:
                        logger.warning("Target beacon lost! Moved maybe? let look for it.")
                        motion.cancel()
                        robot.look_for_beacon(beacon_id).wait()
                        motion = robot.goto(beacon_id,
                                    distance_to_target = 1,
                                    ignore_orientation = True)


            # custom servoing for final approach
            # (since the beacon angle reading is not valid)
            approach = robot.move(0.6, v=0.05)

            while not approach.done():
                distance = robot.beacons[beacon_id].r
                logger.debug("Beacon %s seen at %.2fm" % (beacon_id, distance))
                robot.sleep(0.1)
                if distance < 0.3:
                    approach.cancel()

    except ActionCancelled:
        pass

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

