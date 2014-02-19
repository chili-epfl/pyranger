#!/usr/bin/python
# -*- coding: utf-8 -*-

import dbus

from dbus.mainloop.glib import DBusGMainLoop
import gobject

from optparse import OptionParser

class Medulla:
    def __init__(self, system_bus = False, dummy = False):
        self.dummy = dummy

        self.callbacks = {}

        if not dummy:
            DBusGMainLoop(set_as_default=True)
            # init 
            if system_bus:
                self.bus = dbus.SystemBus()
            else:
                self.bus = dbus.SessionBus()
            self.network = dbus.Interface(self.bus.get_object('ch.epfl.mobots.Aseba', '/'), dbus_interface='ch.epfl.mobots.AsebaNetwork')
            self.clear_events()
            eventfilter = self.network.CreateEventFilter()
            self.events = dbus.Interface(self.bus.get_object('ch.epfl.mobots.Aseba', eventfilter), dbus_interface='ch.epfl.mobots.EventFilter')
            self.dispatch_handler = self.events.connect_to_signal('Event', self._dispatch_events)

    def clear_events(self):
        """ Use DBus introspection to get the list of event filters, and remove them.
        """
        try:
            introspect = dbus.Interface(self.bus.get_object('ch.epfl.mobots.Aseba', "/events_filters"), dbus_interface=dbus.INTROSPECTABLE_IFACE)
            interface = introspect.Introspect()
        except dbus.exceptions.DBusException:
            # /events_filters not yet created -> no events to delete
            return

        import xml.etree.ElementTree as ET
        root = ET.fromstring(interface)
        for n in root.iter("node"):
            if 'name' in n.attrib:
                evtfilter = dbus.Interface(self.bus.get_object('ch.epfl.mobots.Aseba', "/events_filters/%s" % n.attrib['name']), dbus_interface='ch.epfl.mobots.EventFilter')
                evtfilter.Free()

    def run(self):
        # run event loop
        self.loop = gobject.MainLoop()
        self.loop.run()

    def close(self):
        self.loop.quit()
        self.dispatch_handler.remove()
        self.events.Free()

    def dbus_reply(self):
        # correct replay on D-Bus, ignore
        pass

    def dbus_error(self, e):
        assert(not dummy)

        # there was an error on D-Bus, stop loop
        self.close()
        raise Exception('dbus error: %s' % str(e))

    def get_nodes_list(self):
        if self.dummy: return []

        dbus_array = self.network.GetNodesList()
        size = len(dbus_array)
        return [str(dbus_array[x]) for x in range(0,size)]

    def set_variable(self, node, var, value):
        if self.dummy: return
        self.network.SetVariable(node, var, value)

    def get_variable(self, node, var):
        if self.dummy: return [0] * 10
        dbus_array = self.network.GetVariable(node, var)
        size = len(dbus_array)
        if (size == 1):
            return int(dbus_array[0])
        else:
            return [int(dbus_array[x]) for x in range(0,size)]

    def send_event(self, event_id, event_args):

        if isinstance(event_id, basestring):
            return self.send_event_name(event_id, event_args)

        if self.dummy: return

        # events are sent asynchronously
        self.network.SendEvent(event_id, 
                               event_args,
                               reply_handler=self.dbus_reply,
                               error_handler=self.dbus_error)


    def send_event_name(self, event_name, event_args):
        if self.dummy: return

        # events are sent asynchronously
        self.network.SendEventName(event_name, 
                                   event_args,
                                   reply_handler=self.dbus_reply,
                                   error_handler=self.dbus_error)


    def load_scripts(self, path):
        if self.dummy: return
        self.network.LoadScripts(path)

    def _dispatch_events(self, *args):
        id, name, vals = args
        #if id in self.callbacks:
        #    self.callbacks[id](vals)
        if name in self.callbacks:
            self.callbacks[name](vals)

    def on_event(self, event_id, cb):
        if self.dummy: return

        self.callbacks[event_id] = cb

        if isinstance(event_id, basestring):
            self.events.ListenEventName(event_id)
        else:
            self.events.ListenEvent(event_id)

# *** TEST ***
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("-s", "--system", action="store_true", dest="system", default=False,
            help="use the system bus instead of the session bus")

    (options, args) = parser.parse_args()

    medulla = Medulla(options.system)

    print "List of nodes: ", medulla.GetNodesList()

    medulla.set_variable("thymio-II", "event.source", [1])
    medulla.send_event(0, [])

    print "Get variable temperature: ", medulla.get_variable("thymio-II", "temperature")
    print "Get array acc: ", medulla.get_variable("thymio-II", "acc")

