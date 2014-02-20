from pylab import *
import random
import numpy as np

class Plot:
    def __init__(self, max_y = 10, min_y = 0):
        ion()
        self.y = [0]
        self.line, = plot(self.y)
        ylim([min_y, max_y])

    def add(self, val):
        self.y.append(val)
        xlim([0, len(self.y) + 1])
        self.line.set_data(arange(0, len(self.y)), self.y) 
        draw() 

