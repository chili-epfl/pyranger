from matplotlib import pyplot as plt
from numpy import arange
from math import sin, cos

class Plot:
    def __init__(self, min_y = 0, max_y = 10):
        plt.ion()
        self.y = [0]
        self.fig = plt.Figure()
        self.ax = plt.axes()
        self.line, = self.ax.plot(self.y)
        plt.ylim([min_y, max_y])

    def add(self, val):
        self.y.append(val)
        plt.xlim([0, len(self.y) + 1])
        self.line.set_data(arange(0, len(self.y)), self.y) 
        plt.draw() 

class XYPlot:
    def __init__(self, x = (-5, 5), y = (-5, 5)):
        plt.ion()
        self.y = [0]
        self.x = [0]
        self.fig = plt.Figure()
        self.ax = plt.axes()
        self.line, = self.ax.plot(self.y)
        plt.ylim(y)
        plt.xlim(x)

    def add(self, x, y, theta = 0):
        self.x.append(x)
        self.y.append(y)
        self.line.set_data(self.x, self.y)


        plt.arrow( x, y, sin(theta) * 0.2, cos(theta) * 0.2, fc="k", ec="k", head_width=0.1, head_length=0.2 )
        plt.draw()
        self.ax.artists[0].remove() # remove previous arrow


if __name__ == "__main__":

    import time, math

    plot = XYPlot()

    plot.add(1,2,math.pi/6)
    time.sleep(1)
    plot.add(2,2, math.pi /3)
    time.sleep(1)
    plot.add(-3,0, math.pi)
    time.sleep(1)
    plot.add(1,0)
