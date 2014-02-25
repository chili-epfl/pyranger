import Pyro4
import time

class ActionState:
    RUNNING = 1
    COMPLETED = 2

    def __init__(self, name, owner, parent, state = ActionState.RUNNING):
        self.name = name
        self.owner = owner
        self.state = state
        self.started_time = time.time()
        self.completed_time = None

        self.parent = parent
        self.children = []

    def completed(self):
        self.state = self.COMPLETED
        self.completed_time = time.time()

    def duration(self):
        if self.state == self.RUNNING:
            return time.time() - self.started_time
        else:
            return self.completed_time - self.started_time

class IntrospectionServer(object):

    def __init__(self):
        self.state = {}

    def initiate(self, owner):
        print("\n\n\npyRanger started (main thread: %s)" % owner)

        self.state[owner] = ActionState("main", owner, ActionState.RUNNING)

    def action_submitted(self, name, owner):
        print("Action %s is being submitted by thread %s" % (name, owner))

    def action_started(self, name, owner, parent):
        print("Action %s has started in thread %s" % (name, owner))
        action = ActionState(name, owner, parent)

        self.state[owner] = action
        self.state[parent].children.append(action)

    def action_completed(self, name, owner):
        print("Action %s, started in thread %s, is finished" % (name, owner))

        self.state[owner].completed()

class IntrospectionPrinter(object):

    def __init__(self, server):
        self.server = server




introspection = IntrospectionServer()

daemon=Pyro4.Daemon()                 # make a Pyro daemon
uri=daemon.register(introspection)   # register the greeting object as a Pyro object

print "Introspection server for pyranger ready. URI =", uri      # print the uri so we can use it in the client later
daemon.requestLoop()                  # start the event loop of the server to wait for calls
