pyranger -- Python interface for high-level control of the Ranger box
=====================================================================

Implement Ranger specific behaviours on top of the pyRobots framework.

This include odometry calculation.

Check-list
----------

- ROS: `ntpdate <host>`, export the `ROS_MASTER_URI` (+ run `roscore`
  somewhere!)
- Audio: call `hack_audio.sh`
Instruction to launch directly on the robot
-------------------------------------------

- call dbus-launch --sh-syntax and execute the output (ie, export DBUS_SESSION_...)
- start asebamedulla daemon: /etc/init.d/asebamedulla start
- run the demos
