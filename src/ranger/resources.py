import logging; logger = logging.getLogger("ranger.resources")
from threading import Lock
class Resource:
    def __init__(self):
        self.lock = Lock()

    def acquire(self, wait = True):
        self.lock.acquire(wait)

    def release(self):
        self.lock.release()


class CompoundResource:
    def __init__(self, *args):
        self.resources = args

    def acquire(self, wait = True):
        for res in self.resources:
            res.acquire(wait)

    def release(self):
        for res in self.resources:
            res.release()

class ResourceLockedError(RuntimeError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


LPUPIL = Resource()
RPUPIL = Resource()
LLID = Resource()
RLID = Resource()
LEYE = CompoundResource(LPUPIL, LLID)
REYE = CompoundResource(RPUPIL, RLID)
LIDS = CompoundResource(LLID, RLID)
EYES = CompoundResource(LEYE, REYE)
WHEELS = Resource()

