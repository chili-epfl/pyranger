from robots.resources import Resource, CompoundResource
from robots.helpers.misc import enum

# hardware resource that need to be shared
LPUPIL = Resource("left pupil")
RPUPIL = Resource("right pupil")
LLID = Resource("left lid")
RLID = Resource("right lid")
LEYE = CompoundResource(LPUPIL, LLID, name = "left eye")
REYE = CompoundResource(RPUPIL, RLID, name = "right eye")
LIDS = CompoundResource(LLID, RLID, name = "eyelids")
PUPILS = CompoundResource(LPUPIL, RPUPIL, name = "pupils")
EYES = CompoundResource(LEYE, REYE, name = "eyes")
WHEELS = Resource("wheels")
AUDIO = Resource("audio")
LEDS = Resource("LEDs")

# sound library
SOUNDS = dict(toy_in = "/root/sound_library/1_happy_noise_-_parade-2.wav",
              toy_out = "/root/sound_library/3_irritated-discharging-cut.wav",
              battery_low = "/root/sound_library/0_curious_-_finding_toy.wav")

# Pre-recorded light patterns
PATTERNS = dict(toy_in = 1,
                toy_out = 2)

# Range and Bearing IDs (the string is actually the frame name)
ID = enum(BEACON = "rab_20",
          ROBOT1 = "rab_10",
          ROBOT2 = "rab_11",
          ROBOT3 = "rab_12",
          ROBOT4 = "rab_13",
          ROBOT5 = "rab_14",
          STATION1 = "rab_15",
          STATION2 = "rab_16",
          STATION3 = "rab_17",
          STATION4 = "rab_18",
          STATION5 = "rab_19")

MYSTATION = ID.STATION2 # TODO multirobot


