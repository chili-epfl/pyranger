introspection = None

try:
    import Pyro4
    uri = "PYRO:obj_ea811f4dcede4891a6b94415313b6cd2@localhost:45590"
    introspection = Pyro4.Proxy(uri)
except ImportError:
    pass
