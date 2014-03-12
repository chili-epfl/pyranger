import logging; logger = logging.getLogger("ranger.events")
import threading
import weakref

from ranger.introspection import introspection

class Events:
    def __init__(self, robot):

        self.robot = robot
        self.eventmonitors = []

    def on(self, var, **kwargs):
        """
        Creates a new EventMonitor to watch a given event model.
        :returns: a new instance of EventMonitor for this event.
        """
        monitor = EventMonitor(self.robot, var, **kwargs)
        self.eventmonitors.append(weakref.ref(monitor))
        return monitor

    def close(self):
        for m in self.eventmonitors:
            monitor = m() # weakref!
            if monitor:
                monitor.close()

class EventMonitor:

    VALUE = "="
    ABOVE = ">"
    BELOW = "<"
    INCREASE = "+="
    DECREASE = "-="

    def __init__(self, robot, var, 
                        value = None, 
                        above = None, 
                        below = None,
                        increase = None,
                        decrease = None,
                        oneshot = False):
        """

        :param oneshot: if true, the event is fired once and then discarded. 
        Otherwise, the event remains active.

        """

        self.cbs= [] # callbacks

        self.robot = robot

        if var not in robot.STATE:
            raise Exception("%s is not part of the robot state" % var)

        self.var = var

        self.oneshot = oneshot

        self.monitoring = False
        self.thread = None

        # store initial value, used by INCREASE/DECREASE modes
        if not self.robot.dummy:
            self.last_value = self.robot.state[self.var] 

        if value is not None:
            self.mode = EventMonitor.VALUE
            self.target = value
        elif above is not None:
            self.mode = EventMonitor.ABOVE
            self.target = above
        elif below is not None:
            self.mode = EventMonitor.BELOW
            self.target = below
        elif increase is not None:
            self.mode = EventMonitor.INCREASE
            self.target = increase
        elif decrease is not None:
            self.mode = EventMonitor.DECREASE
            self.target = decrease
        else:
            raise Exception("Event created without condition!")


    def do(self, cb):

        if introspection:
            introspection.action_subscribe_event(self.robot.action_id.id, str(self))

        # first add callback? start a thread to monitor the event!
        if not self.thread:
            self.monitoring = True
            self.thread = threading.Thread(target=self._monitor, args=[self.robot.action_id.id])
            self.thread.start()

        self.cbs.append(cb)
        return self # to allow for chaining

    def _monitor(self, owner):
        
        # we are in a new thread, and we may execute several actions from there:
        # we need to carry on the action ID
        self.robot.action_id.id = owner

        while self.monitoring:
            self._wait_for_condition()

            if introspection:
                introspection.action_event_fired(owner, str(self))

            for cb in self.cbs:
                cb()
            if self.oneshot:
                return

    def close(self):
        self.monitoring = False
        if self.thread:
            self.thread.join()

    def _check_condition(self, val):

        if self.mode == EventMonitor.VALUE and val == self.target:
            return True
        elif self.mode == EventMonitor.ABOVE and val > self.target:
            return True
        elif self.mode == EventMonitor.BELOW and val < self.target:
            return True
        elif self.mode == EventMonitor.INCREASE and val > (self.last_value + self.target):
            self.last_value = val
            return True
        elif self.mode == EventMonitor.DECREASE and val < (self.last_value - self.target):
            self.last_value = val
            return True


    def _wait_for_condition(self, timeout = None):

        if not self.robot.dummy:
            if self.var not in self.robot.state:
                # value not yet read from the robot.
                logger.warning("Waiting for %s to be published by the robot..." % self.var)
                self.robot.update.acquire()
                while not self.var in self.robot.state:
                    self.robot.update.wait(1)
                self.robot.update.release()


            self.robot.update.acquire()
            while not self._check_condition(self.robot.state[self.var]):
                self.robot.update.wait(timeout)
            self.robot.update.release()

        logger.info("%s is true" % str(self) + " (dummy mode)" if self.robot.dummy else "")


    def wait(self, timeout = None):
        """ Blocks until an event occurs, or the timeout expires.
        """

        if introspection:
            introspection.action_waiting(self.robot.action_id.id, str(self))


        self._wait_for_condition(timeout)


        if introspection:
            introspection.action_waiting_over(self.robot.action_id.id)

    def __str__(self):
        return "condition <%s %s %s>"% (self.var, self.mode, self.target)
