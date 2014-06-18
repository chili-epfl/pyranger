from collections import OrderedDict

from ranger.behaviours.fn_library import *
from ranger.behaviours.time_management import * # time_elapsed_in + different time periods

class InteractionsTracker:

    # normalization value: interaction densities are normalized
    # by taking nb_interaction/(duration * MAX_INTERACTIONS_PER_SECOND)
    # and then clamped between [0,1].
    MAX_INTERACTIONS_PER_SECOND=2

    def __init__(self):
        self.interactions = OrderedDict()

    def add(self, interaction):
        if isinstance(interaction, tuple):
            self.interactions.append(interaction)
        else:
            self.interactions.append((int(time.time()), interaction))

    def density_at(self, timezone):
        if timezone in [IMMEDIATE, CURRENT, RECENT]:
            idx = self.interactions
            #return len(


class EmotionalState:

    def __init__(self):
        pass

    @property
    def now(self):
        return datetime.datetime.now().time()

    @property
    def energy(self):
        """ The current level of energy of the robot, in [0.0, 1.0]
        
        Roughly:
            1 -> full of energy, 
            0 -> exhausted, 
            0.6 -> normal, 
            0.3 -> tired
        """
        ratio = ratio_time_elapsed_in(DAY)
        return dromadaire(ratio) * 0.6 + 0.4

    @property
    def valence(self):
        """ How positive the robot is: 1 -> extremely positive, -1 -> extremely
        negative
        """
        return 0

    @property
    def arousal(self):
        """ How intense the emotional state is: 1 -> extremely intense, -1 ->
        extreme bordom
        """
        return 0
