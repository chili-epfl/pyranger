import logging; logger = logging.getLogger("ranger.actions")
import time

import threading

from resources import *
from ranger.introspection import introspection
from lowlevel.ranger_aseba import executor
from ranger.signals import ActionCancelled

def action(fn):
    """
    When applied to a function, this decorator turns it into
    a asynchronous task, starts it in a different thread, and returns
    a 'future' object that can be used to query the result/cancel it/etc.
    """

    # wrapper for the original function that locks/unlocks shared
    # resources
    def lockawarefn(*args, **kwargs):

        try:
            # we acquire resources *within the future thread* that
            # we want to *wait* for.
            if hasattr(fn, "_locked_res"):
                for res, wait in fn._locked_res:
                    if wait:
                        threading.current_thread().name = "Ranger Action %s (waiting on resource %s)" % (fn.__name__, res)
                        res.acquire(wait)
        except ActionCancelled:
            # action cancelled while it was waiting for a resource to become
            # available
            threading.current_thread().name = "Idle Ranger action thread"
            logger.debug("Action %s cancelled while it was waiting for a lock on a resource." % fn.__name__)
            return None
 
        try:
            threading.current_thread().name = "Ranger Action %s (running)" % (fn.__name__)
            logger.debug("Starting action %s now." % fn.__name__)
            result = fn(*args, **kwargs)
            return result
        except ActionCancelled:
            logger.warning("Action cancellation ignored by %s. Forced stop!" % fn.__name__)
        finally:
            if hasattr(fn, "_locked_res"):
                for res, wait in fn._locked_res:
                    res.release()

            threading.current_thread().name = "Idle Ranger action thread"


    lockawarefn.__name__ = fn.__name__
    lockawarefn.__doc__ = fn.__doc__


    # wrapper that submits the function to the executor and returns
    # a future.
    def innerfunc(*args, **kwargs):

        immediate = False
        if hasattr(args[0], "immediate"): # if args[0] has a 'immediate' member, it's probably the lowlevel robot instance
            immediate = args[0].immediate
        else:
            raise Exception("No robot instance passed to the action!")


        # we acquire resources *outside the future* (to fail fast)
        # for resources we do not want to wait for.
        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                if not wait:
                    got_the_lock = res.acquire(False)

                    if not got_the_lock:
                        raise ResourceLockedError("Required resource <%s> locked while running %s" % ([name for name in globals() if globals()[name] is res][0], fn.__name__))

        if immediate:
            res = FakeFuture(lockawarefn(*args, **kwargs))
            return res

        else:
            if args and kwargs:
                future = executor.submit(lockawarefn, *args, **kwargs)
            elif args:
                future = executor.submit(lockawarefn, *args)
            else:
                future = executor.submit(lockawarefn)

            if introspection:

                introspection.action_started(fn.__name__, 
                                            str("FUTURE ID BROKEN"),
                                            str("ACTION ID BROKEN TDB"), #id of the current action
                                            args[1:],
                                            kwargs)
                future.add_done_callback(lambda x : introspection.action_completed(fn.__name__, str("future.id BROKEN")))

            return future

    innerfunc.__name__ = fn.__name__
    innerfunc.__doc__ = fn.__doc__
    innerfunc._action = True

    return innerfunc

def lock(res, wait = True):
    """
    Used to define which resources are acquired (and locked)
    by the action.

    :param res: one of the resource defined in resources.py
    :param wait: (default: true) if true, the action will wait
    until the resource is available, if false, the action will
    raise an ResourceLockedError exception if the resource is
    not available.
    """
    def decorator(fn):
        if hasattr(fn, "_locked_res"):
            fn._locked_res.append((res, wait))
        else:
            fn._locked_res = [(res, wait)]

        return fn

    return decorator

class FakeFuture:

    def __init__(self, result):
        self._result = result
    def result(self):
        return self._result
