from robots.resources import Resource, CompoundResource
from robots.helpers.helpers import enum

# hardware resource that need to be shared
LPUPIL = Resource("left pupil")
RPUPIL = Resource("right pupil")
LLID = Resource("left lid")
RLID = Resource("right lid")
LEYE = CompoundResource(LPUPIL, LLID, name = "left eye")
REYE = CompoundResource(RPUPIL, RLID, name = "right eye")
LIDS = CompoundResource(LLID, RLID, name = "eyelids")
EYES = CompoundResource(LEYE, REYE, name = "eyes")
WHEELS = Resource("wheels")
AUDIO = Resource("audio")
LEDS = Resource("LEDs")

# sound library
SOUNDS = dict(toy_in = "share/sounds/1_happy_noise_-_parade-2.wav",
              toy_out = "share/sounds/3_irritated-discharging-cut.wav")

# Pre-recorded light patterns
PATTERNS = dict(toy_in = 1,
                toy_out = 2)

# Range and Bearing IDs (the string is actually the frame name)
ID = enum(BEACON = 20,
          ROBOT1 = 10,
          ROBOT2 = 11,
          ROBOT3 = 12,
          ROBOT4 = 13,
          ROBOT5 = 14,
          STATION1 = 15,
          STATION2 = 16,
          STATION3 = 17,
          STATION4 = 18,
          STATION5 = 19)

MYSTATION = ID.STATION2 # TODO multirobot


