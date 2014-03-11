import logging; logger = logging.getLogger("ranger." + __name__)
import threading
introspection = None

try:
    import Pyro4
    import Pyro4.errors
    uri = "PYRONAME:ranger.introspection" # uses name server
    try:
        introspection = Pyro4.Proxy(uri)
        introspection.initiate(threading.current_thread().ident)
    except Pyro4.errors.CommunicationError:
        logger.warning("Introspection server not running. No introspection.")
        introspection = None
    except Pyro4.errors.NamingError:
        logger.warning("Introspection server not running (no name server). No introspection.")
        introspection = None
 
except ImportError:
    pass
