pyranger -- Python interface for high-level control of the Ranger box
=====================================================================

Introspection
-------------

An introspection server can be launched to monitor in live the
behaviour of the robot.

Launch it **before** the main behaviour scripts with:

$ python -m Pyro4.naming # Pyro nameserver
$ bin/ranger_introspection --server

The introspection server can be launched once for all.

Display the state of the robot at any time with:

$ bin/ranger_introspection

