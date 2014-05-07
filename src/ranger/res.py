from robots.resources import Resource, CompoundResource

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

# Range and Bearing IDs
ID = dict(BEACON = "beacon_20",
          ROBOT1 = "beacon_10",
          ROBOT2 = "beacon_11",
          ROBOT3 = "beacon_12",
          ROBOT4 = "beacon_13",
          ROBOT5 = "beacon_14",
          STATION1 = "beacon_15",
          STATION2 = "beacon_16",
          STATION3 = "beacon_17",
          STATION4 = "beacon_18",
          STATION5 = "beacon_19")

ID["MYSTATION"] = ID["STATION1"] # TODO multirobot


