from concurrent.futures import ThreadPoolExecutor
import threading
import time

from resources import *
from ranger.introspection import introspection

executor = ThreadPoolExecutor(max_workers = 40) # at most 40 tasks

def action(fn):
    """
    When applied to a function, this decorator turns it into
    a asynchronous task, starts it in a different thread, and returns
    a 'future' object that can be used to query the result/cancel it/etc.
    """

    # wrapper for the original function that locks/unlocks shared
    # resources
    def lockawarefn(*args, **kwargs):

        # we acquire resources *within the future thread* that
        # we want to *wait* for.
        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                if wait:
                    res.acquire(wait)

        result = fn(*args, **kwargs)

        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                res.release()


        return result


    # wrapper that submits the function to the executor and returns
    # a future.
    def innerfunc(*args, **kwargs):

        immediate = False
        if hasattr(args[0], "immediate"): # if args[0] has a 'immediate' member, it's probably the lowlevel robot instance
            immediate = args[0].immediate


        # we acquire resources *outside the future* (to fail fast)
        # for resources we do not want to wait for.
        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                if not wait:
                    got_the_lock = res.acquire(wait)

                    if not got_the_lock:
                        raise ResourceLockedError("Required resource <%s> locked while running %s" % ([name for name in globals() if globals()[name] is res][0], fn.__name__))

        if introspection:
            introspection.action_submitted(fn.__name__, threading.current_thread().ident)
            current_threads = set([t.ident for t in threading.enumerate()])

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
                # hack to get the ID of the newly created thread. May fail (ie, non unique value)
                # if two threads are created in parallel
                new_threads = []
                max_wait = .5 #sec
                new_threads = set([t.ident for t in threading.enumerate()]) - current_threads
                while not new_threads and max_wait > 0:
                    max_wait -= 0.01
                    time.sleep(0.01) # wait 10ms to leave some time for the future to start
                    new_threads = set([t.ident for t in threading.enumerate()]) - current_threads

                if max_wait > 0:
                    assert(len(new_threads) > 0)
                    assert(len(new_threads) == 1)
                    future_thread = new_threads.pop()

                    introspection.action_started(fn.__name__, 
                                                future_thread,
                                                threading.current_thread().ident,
                                                args[1:],
                                                kwargs)
                    future.add_done_callback(lambda x : introspection.action_completed(fn.__name__, future_thread))
                else:
                    print("Future not running after %s s... exception? max_worker overflow? (currently running %s threads)" % (max_wait, len(executor._threads)))
                    ex = future.exception()
                    if ex:
                        raise ex

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
