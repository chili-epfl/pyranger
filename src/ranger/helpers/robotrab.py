import math
import numpy as np
import collections

# the file DATASET_PATH must contains R&B readings for 1 complete (360°) 
# turn of the robot, at constant speed, and acquired at REFERENCE_SAMPLING_RATE
DATASET_PATH= "rab-robot-360"
REFERENCE_SAMPLING_RATE = 10. # Hz

class OrientationEstimator:

    EPSILON_SENSOR = 5

    def __init__(self):


        self.default_nb_readings = 10
        self.last_readings = collections.deque(maxlen=self.default_nb_readings)

        self.last_angle = 0
        self.dirty = True

        with open(DATASET_PATH,'r') as data:
            self.reference_dataset = [int(val) for i, val in enumerate(data.readlines())]

        self.measures_per_rad = len(self.reference_dataset) / (2 * math.pi)

    def normalize(self, vals):
        """
        Returns the sensor readings, ordered and interpolated to
        account for the variations of speed, as well as the index (within the
        new ordering) of the last value of the original sample. This index can
        be used to select the appropriate angle (ie, the one corresponding to
        the last reading)

        """

        last_val = vals[-1][0]
        speeds = [s for v,s in vals]

        vals_at_orientation = {}
        rotation = 0
        for v,s in vals:
            rotation += s * 1/REFERENCE_SAMPLING_RATE
            vals_at_orientation[rotation] = v

        desired_nb_measurements = int(abs(rotation * self.measures_per_rad))

        orientations, vals = zip(*sorted(vals_at_orientation.items())) # 'unzip' the 2 lists. Cf http://stackoverflow.com/questions/13635032/what-is-the-inverse-function-of-zip-in-python

        index = 0
        for i,v in enumerate(vals):
            if v == last_val:
                index = i
                break
        index = int(index * float(desired_nb_measurements) / len(vals))

        # orientations must be between 0 and 'desired_nb_measurements' for interpolation
        scaling= desired_nb_measurements/(max(orientations) - orientations[0])
        norm_orientations = [(o - orientations[0]) * scaling for o in orientations]

        interp_vals = list(np.interp(range(desired_nb_measurements),
                                norm_orientations, 
                                vals))

        return (interp_vals, index)


    def match(self, vals):
        """
        Returns the angle (in radians) based on the last X readings from
        the R&B

        :param vals: a list of tuples (sensor reading, instant rotation speed)
        """
        vals, index = self.normalize(vals)
        sums = []
        for shift in range(0, len(self.reference_dataset)):
            target_range = np.roll(self.reference_dataset, -shift)[:len(vals)]
            sums.append(sum([abs(a - b) for a,b in zip(vals, target_range)]))

        best_shift = np.argmin(sums) + index 

        angle = ( best_shift % len(self.reference_dataset)) * (2*math.pi / len(self.reference_dataset))

        return angle

    @staticmethod
    def nearly_equal(a,b,sig_fig=5):
        return ( a==b or 
                int(a*10**sig_fig) == int(b*10**sig_fig)
            )

    def add_data(self, val, speed):
        if abs(speed) < 0.001:
            return
        self.last_readings.append((val, speed))
        self.dirty = True


    def get_orientation(self):
        if self.dirty and len(self.last_readings) > self.default_nb_readings:
            self.last_angle = self.match(self.last_readings)
            self.dirty = False

        return self.last_angle


