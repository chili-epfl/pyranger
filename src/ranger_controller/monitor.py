#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
from ranger_aseba import get_robot, BATTERY_LOW_THRESHOLD, ID

from consoletk import ConsoleTK

# to poll the keyboard
import select

# Dummy values used by the low-level emulator
main_msg = [ 1.2, 0.4, 1.5,
            0.43, 0.54,
            3452,
            1,
            1.2, 2.5,
            0b100110100, 0b000100110, 0b001000111,
            1234,34324,5432,34554,
            0,
            1.6,2.3]
neuil_msg = [0.12, 1.03, 0.03,
                0,
                0.023]


class Monitor:

    def __init__(self, silent = False):
        self.robot = get_robot()

        # used for debugging purposes (-> if set to true, no ANSI sequence -> no mess with debug msgs)
        self.silent = silent

        if not silent:
            self.out = ConsoleTK(height = 12)

        self.eyes = [0, 0, 0, 0]
        self.speed = [0, 0]

    def __enter__(self):

        if not self.silent:
            self.out.__enter__()
        self.running = True
        return self

    def __exit__(self, type, value, traceback):
        self.running = False
        if not self.silent:
            self.out.__exit__(type, value, traceback)
        self.robot.close()

    def showstatus(self):

        robot = self.robot
        ###################################################
        ### General stuff
        ###################################################

        self.out.boolean(robot.lolette, "Lolette")
        self.out.relmoveto(10,0)
        self.out.boolean(robot.bumper, "Bumper")

        self.out.moveto(22,0)
        self.out.absolutebar(robot.scale, 10, "kg", label = "Scale", maxlength = 20, autocolor = True, highishot = True)
        self.out.moveto(60,0)
        self.out.absolutebar(robot.battery, 8500, 
                             "mV", label = "Battery" + (" (charging)" if robot.charging else ""), 
                             autocolor = True)

        self.out.moveto(0,2)
        self.out.absolutebar(robot.velocity_left, 100, "%", label = " Left wheel", minvalue = -100)
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.velocity_right, 100, "%", label = "Right wheel", minvalue = -100)

        ###################################################
        ### Beacons
        ###################################################

        self.out.moveto(78,2)
        self.out.vseparator(8)

        self.out.moveto(80,1)

        for beacon in robot.beacons.values():
            if not beacon.obsolete():
                self.out.relmoveto(0,1)
                self.out.label("Beacon %s detected" % beacon.id)
                self.out.relmoveto(0,1)
                self.out.label("Dist: %.2fm   Angle: %.2frad" % (beacon.distance, beacon.angle), fg="base0")
            else:
                # clear!
                self.out.clear(29, 9)

        ###################################################
        ### Touch sensors
        ###################################################

        self.out.moveto(0,5)
        self.out.label("Touch sensors")
        self.out.relmoveto(0,1)
        self.out.booleanmatrix(robot.touch_left, label = " left")

        self.out.relmoveto(7,0)
        self.out.booleanmatrix(robot.touch_rear, label = " rear")

        self.out.relmoveto(7,0)
        self.out.booleanmatrix(robot.touch_right, label = " right")

        ###################################################
        ### Infrared sensors
        ###################################################

        self.out.moveto(30,7)
        self.out.absolutebar(robot.ir_left, 0.5, "m", label = "  IR left", autocolor = True)
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.ir_center, 2.1, "m", label = "IR center", autocolor = True)
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.ir_right, 0.5, "m", label = " IR right", autocolor = True)

        ###################################################
        ### Update frequencies
        ###################################################

        self.out.moveto(0,11)
        self.out.label("Freq. main: %4.1fHz | Freq. neuil: %4.1fHz | Freq. R&B: %4.1fHz" % (robot.freq_main, robot.freq_neuil, robot.freq_rab))


    def controller(self, key):

        robot = self.robot


        if key == "w":
            self.eyes[1] += 5
            robot.eyes(*self.eyes)
        elif key == "s":
            self.eyes[1] -= 5
            robot.eyes(*self.eyes)
        elif key == "a":
            self.eyes[0] += 5
            robot.eyes(*self.eyes)
        elif key == "d":
            self.eyes[0] -= 5
            robot.eyes(*self.eyes)
        elif key == "t":
            self.eyes[3] += 5
            robot.eyes(*self.eyes)
        elif key == "g":
            self.eyes[3] -= 5
            robot.eyes(*self.eyes)
        elif key == "f":
            self.eyes[2] += 5
            robot.eyes(*self.eyes)
        elif key == "h":
            self.eyes[2] -= 5
            robot.eyes(*self.eyes)



        elif key == self.out.ARROW_UP:
            self.speed[0] += 5
            self.speed[1] += 5
            robot.speed(*self.speed)

        elif key == self.out.ARROW_DOWN:
            self.speed[0] -= 5
            self.speed[1] -= 5
            robot.speed(*self.speed)

        elif key == self.out.ARROW_LEFT:
            self.speed[0] += 5
            self.speed[1] -= 5
            robot.speed(*self.speed)

        elif key == self.out.ARROW_RIGHT:
            self.speed[0] -= 5
            self.speed[1] += 5
            robot.speed(*self.speed)

        elif key == " ":
            self.speed = [0,0]
            robot.speed(0,0)



    def run(self):

        if not hasattr(self, 'running'):
            print("Monitor.run must be called from inside a 'with' statement")

        key = ""

        try:
            while self.running and key != "q":

                if not self.silent:
                    key = self.out.get_keypress()
                    self.controller(key)

                    self.out.moveto(0,0)
                    self.showstatus()
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.robot.speed(0,0)


if __name__ == '__main__':

    with Monitor(silent = False) as monitor:
            monitor.run()

