from robots.helpers.position import PoseManager

class RangerPoseManager(PoseManager):

    def __init__(self, robot):
        super(RangerPoseManager, self).__init__(robot)

    def getabspose(self, frame):
        """
        Returns the a frame pose in the /map frame.
        """
        frame = frame.lstrip("/")

        if frame == "base_link":
            return [self.robot.state.x, self.robot.state.y, 0.0, 0.0, 0.0, self.robot.state.theta]

        if frame == "map" or frame == "station":
            # the charging station is the origin of the map
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if frame == "eyes_link":
            # position of the (middle of) the eyes
            return self.inframe(
                    [0.2, 0.0, 0.2, 0.0, 0.0, 0.0, "base_link"],
                    "map")

        if frame.startswith("beacon"):
            beacon_id = int(frame.split("_")[1])
            try:
                beacon = self.robot.beacons[beacon_id]
            except KeyError:
                raise UnknownFrameError("Beacon %s never seen" % beacon_id)

            if beacon.obsolete():
                raise UnknownFrameError("Beacon %s has obsolete position (not seen since long time)" % beacon_id)

            return self.inframe(
                    [beacon.x, beacon.y, 0.0, 0.0, 0.0, beacon.theta, "base_link"],
                    "map")

        raise UnknownFrameError("Frame %s does not exist." % frame)


