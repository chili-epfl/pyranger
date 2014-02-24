import Pyro4

class IntrospectionServer(object):

    def initiate(self, owner):
        print("\n\n\npyRanger started (main thread: %s)" % owner)

    def action_starting(self, name, owner):
        print("Action %s is being started by thread %s" % (name, owner))

    def action_started(self, name, owner):
        print("Action %s has started in thread %s" % (name, owner))


    def action_finished(self, name, owner):
        print("Action %s, started in thread %s, is finished" % (name, owner))



introspection = IntrospectionServer()

daemon=Pyro4.Daemon()                 # make a Pyro daemon
uri=daemon.register(introspection)   # register the greeting object as a Pyro object

print "Introspection server for pyranger ready. URI =", uri      # print the uri so we can use it in the client later
daemon.requestLoop()                  # start the event loop of the server to wait for calls
