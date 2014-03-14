from threading import Lock
class Resource:
    def __init__(self):
        self.lock = Lock()

    def __enter__(self):
        """
        Entering a 'resource' block *release* the lock, which may seem counter-intuitive.

        It is meant to used inside an action that lock the resource, to temporarly transfer the
        lock ownership to a sub-action:

        For instance:

        @action
        @lock(WHEELS)
        def move(...):
            ...

        @action
        @lock(WHEELS)
        def goto(...):

            with WHEELS:
                move(...)

        Here, goto() calls move() by first releasing the lock on WHEELS, executing move() and reacquiring the lock,
        also if move() raises an exception.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        self.acquire()
        # here, the exception, if any, is automatically propagated

    def acquire(self, wait = True):
        self.lock.acquire(wait)

    def release(self):
        self.lock.release()


class CompoundResource:
    def __init__(self, *args):
        self.resources = args

    def __enter__(self):
        """ cf doc of Resource.__enter__.
        """
        self.release()

    def __exit__(self, exc_type, exc_value, traceback):
        """ cf doc of Resource.__exit__.
        """
        self.acquire()
        # here, the exception, if any, is automatically propagated



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
AUDIO = Resource()
LEDS = Resource()
