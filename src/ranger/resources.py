import logging; logger = logging.getLogger("ranger.resources")

from threading import Lock

LPUPIL = Lock()
RPUPIL = Lock()
LLID = Lock()
RLID = Lock()
LEYE = CounpoundResource(LPUPIL, LLID)
REYE = CounpoundResource(RPUPIL, RLID)
LIDS = CounpoundResource(LLID, RLID)
EYES = CounpoundResource(LEYE, REYE)
WHEELS = Lock()

class CounpoundResource:
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
