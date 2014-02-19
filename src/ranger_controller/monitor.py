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
        self.out.absolutebar(robot.velocity_left, 3, "rad.s^-1", label = " Left wheel")
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.velocity_right, 3, "rad.s^-1", label = "Right wheel")

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
        self.out.absolutebar(robot.ir_left, 0.3, "m", label = "  IR left", autocolor = True)
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.ir_center, 1.3, "m", label = "IR center", autocolor = True)
        self.out.relmoveto(0,1)
        self.out.absolutebar(robot.ir_right, 0.3, "m", label = " IR right", autocolor = True)

        ###################################################
        ### Update frequencies
        ###################################################

        self.out.moveto(0,11)
        self.out.label("Freq. main: %10.1fHz -- Freq. neuil: %10.1fHz" % (robot.freq_main, robot.freq_neuil))


    def lowlevel_emulator(self, key):
        global main_msg, neuil_msg

        robot = self.robot

        if key == "l": # lolette
            neuil_msg[3] = int(not bool(neuil_msg[3])) # toggle...
            robot.neuil_msg = neuil_msg
        elif key == " ": # bumper
            main_msg[6] = int(not bool(main_msg[6])) # toggle...
            robot.main_msg = main_msg
        elif key == "+": # scale ++
            neuil_msg[4] += 0.1
            robot.neuil_msg = neuil_msg
        elif key == "-": # scale --
            neuil_msg[4] -= 0.1
            robot.neuil_msg = neuil_msg
        elif key == "b": # toggle battery level
            if robot.battery > BATTERY_LOW_THRESHOLD:
                main_msg[5] = int(0.5 * BATTERY_LOW_THRESHOLD)
                robot.main_msg = main_msg
            else:
                main_msg[5] = int(1.1 * BATTERY_LOW_THRESHOLD)
                robot.main_msg = main_msg


        # motor control
        elif key == self.out.ARROW_UP:
            main_msg[7] += 0.1
            main_msg[8] += 0.1
            robot.main_msg = main_msg
        elif key == self.out.ARROW_DOWN:
            main_msg[7] -= 0.1
            main_msg[8] -= 0.1
            robot.main_msg = main_msg
        elif key == self.out.ARROW_LEFT:
            main_msg[7] -= 0.1
            main_msg[8] += 0.1
            robot.main_msg = main_msg
        elif key == self.out.ARROW_RIGHT:
            main_msg[7] += 0.1
            main_msg[8] -= 0.1
            robot.main_msg = main_msg

        # range and bearing
        elif key == "1":  # BEACON
            robot.rab_msg = [ID["BEACON"], 960, 734, 0, 0, 0, 0, 0, 0, 0]
        elif key == "2": # STATION 1
            robot.rab_msg = [ID["STATION1"], 302, 74, 0, 0, 0, 0, 0, 0, 0]
        elif key == "3": # ROBOT 2
            robot.rab_msg = [ID["ROBOT2"], 402, 1704, 0, 0, 0, 0, 0, 0, 0]
        elif key == "4": # STATION 2
            robot.rab_msg = [ID["STATION2"], 2002, 504, 0, 0, 0, 0, 0, 0, 0]





    def run(self):

        if not hasattr(self, 'running'):
            print("Monitor.run must be called from inside a 'with' statement")

        key = ""

        while self.running and key != "q":

            if not self.silent:
                key = self.out.get_keypress()
                #self.lowlevel_emulator(key)

                self.out.moveto(0,0)
                self.showstatus()
            time.sleep(0.02)


if __name__ == '__main__':

    with Monitor(silent = False) as monitor:

        try:
            monitor.run()
        except KeyboardInterrupt:
            pass

