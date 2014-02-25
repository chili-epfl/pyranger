import logging; logger = logging.getLogger("ranger." + __name__)
introspection = None

try:
    import Pyro4
    import Pyro4.erros
    uri = "PYRO:obj_ea811f4dcede4891a6b94415313b6cd2@localhost:45590"
    try:
        introspection = Pyro4.Proxy(uri)
    except Pyro4.errors.CommunicationError:
        logger.warning("Introspection server not running. No introspection.")
except ImportError:
    pass
