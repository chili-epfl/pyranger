import logging; logger = logging.getLogger("ranger.poses")

from robots.helpers.position import FrameProvider, UnknownFrameError

from ranger.res import ID, MYSTATION # list of R&B ids known to the system

class RangerFrames(FrameProvider):

    def __init__(self, robot):
        self.robot = robot

    def get_transform(self, frame):
        """
        Returns the a frame pose in the /map frame.
        """
        if isinstance(frame, basestring):
            frame = frame.lstrip("/")

            if frame == "base_link":
                return [self.robot.state.x, self.robot.state.y, 0.0, 0.0, 0.0, self.robot.state.theta]

            if frame == "map" or frame == "station":
                # the charging station is the origin of the map
                return {"frame":"map"}

            if frame == "eyes_link":
                # position of the (middle of) the eyes
                return self.inframe(
                        [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, "base_link"],
                        "map")

            if frame == MYSTATION:
                # the charging station is the origin of the map
                return {frame:"map"}

            try:
                beacon = self.robot.beacons[frame]
            except KeyError:
                raise UnknownFrameError("Beacon %s never seen" % frame)

            if not beacon.last_valid_pose:
                raise UnknownFrameError("No valid location known for beacon %s." % frame)

            if beacon.obsolete():
                logger.warning("Beacon %s may have obsolete position (not seen since long time). Using last valid position." % frame)

            if not beacon.valid:
                logger.warning("Beacon %s currently has invalid position (out of sight, too close or not facing). Using last valid position." % frame)

            return  beacon.last_valid_pose

        raise UnknownFrameError("Frame %s does not exist." % frame)


