from concurrent.futures import ThreadPoolExecutor
import threading

from resources import *
from ranger.introspection import introspection

executor = ThreadPoolExecutor(max_workers = 5)

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

        # we acquire resources *outside the future* (to fail fast)
        # for resources we do not want to wait for.
        if hasattr(fn, "_locked_res"):
            for res, wait in fn._locked_res:
                if not wait:
                    got_the_lock = res.acquire(wait)

                    if not got_the_lock:
                        raise ResourceLockedError("Required resource <%s> locked while running %s" % ([name for name in globals() if globals()[name] is res][0], fn.__name__))

        if introspection:
            introspection.action_starting(fn.__name__, threading.current_thread().ident)
            current_threads = set([t.ident for t in threading.enumerate()])

        if args and kwargs:
            future = executor.submit(lockawarefn, *args, **kwargs)
        elif args:
            future = executor.submit(lockawarefn, *args)
        else:
            future = executor.submit(lockawarefn)

        if introspection:
            # hack to get the ID of the newly created thread. May fail (ie, non unique value)
            # if two threads are created in parallel
            new_threads = set([t.ident for t in threading.enumerate()]) - current_threads
            introspection.action_started(fn.__name__, new_threads.pop() if len(new_threads) == 1 else None)
            future.add_done_callback(lambda x : introspection.action_finished(fn.__name__, threading.current_thread().ident))

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


