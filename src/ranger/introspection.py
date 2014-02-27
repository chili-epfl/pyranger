import logging; logger = logging.getLogger("ranger." + __name__)
introspection = None

try:
    import Pyro4
    import Pyro4.errors
    uri = "PYRONAME:ranger.introspection" # uses name server
    try:
        introspection = Pyro4.Proxy(uri)
    except Pyro4.errors.CommunicationError:
        logger.warning("Introspection server not running. No introspection.")
except ImportError:
    pass
