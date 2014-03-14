"""
Extension of Python futures to support robot action management.

The main changes are:
 - the storage of a unique 'action_id' for
   each task submitted to the Executor in a way accessible to the
   action itself.
 - the use of 'PausableThreads', ie threads in which cancellation and
   pause can be signaled (using custom exceptions as signals).
"""
import logging; logger = logging.getLogger("ranger.actions")
import uuid
import sys

try:
    from concurrent.futures import ThreadPoolExecutor, Future, TimeoutError
    from concurrent.futures.thread import _worker, _thread_references
except ImportError:
    import sys
    sys.stderr.write("[error] install python-concurrent.futures\n")
    sys.exit(1)

import weakref
import threading 
import thread # for get_ident

from ranger.signals import ActionCancelled, ActionPaused

# this dictionary keep track of which thread runs which action.
# useful for notifying cancellation/pauses
_actions_threads = dict()

class PausableThread(threading.Thread):
    """ Based on http://ideone.com/HBvezh
    """
    def __init__(self, *args, **kwargs):
        threading.Thread.__init__(self, *args, **kwargs)
        self.debugger_trace = None

    def cancel(self):
        self.__cancel = True
    def pause(self):
        self.__pause = True

    def _Thread__bootstrap(self):
        """ The name come from Python name mangling for 
        __double_leading_underscore_names

        Note that in Python3, __bootstrap becomes _bootstrap, thus
        making it easier to override.
        """
        if threading._trace_hook is not None:
            self.debugger_trace = threading._trace_hook
            #logger.warning("Tracing function already registered (debugger?). Task cancellation/pause won't be available.")
        #else:
        self.__cancel = False
        self.__pause = False
        sys.settrace(self.__trace)

        self.name = "Ranger action thread (initialization)"
        super(PausableThread, self)._Thread__bootstrap()

    def __trace(self, frame, event, arg):
        if self.debugger_trace:
            self.debugger_trace(frame, event, arg)

        if self.__cancel:
            self.__cancel = False
            logger.debug("Cancelling thread <%s>" % self.name)
            raise ActionCancelled()
        if self.__pause:
            self.__pause = False
            logger.debug("Pausing thread <%s>" % self.name)
            raise ActionPaused()
        return self.__trace

class _RobotWorkItem(object):
    def __init__(self, future, fn, args, kwargs):
        self.future = future
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):

        # Store the action UUID in the thread local storage.
        self.args[0].action_id.id = self.future.id
        # Store the thread ID that runs this action
        _actions_threads[self.future.id] = thread.get_ident()


        if not self.future.set_running_or_notify_cancel():
            return

        try:
            result = self.fn(*self.args, **self.kwargs)
        except BaseException:
            e = sys.exc_info()[1]
            logger.error("Exception in action <%s>: %s"%(self.fn.__name__, e))
            self.future.set_exception(e)
        else:
            self.future.set_result(result)

        del _actions_threads[self.future.id]

        # clear the thread's action_id to make clear we are done
        self.args[0].action_id.id = None

class RobotAction(Future):
    def __init__(self, executor, actionname):
        Future.__init__(self)

        self.actionname = actionname
        self.id = str(uuid.uuid4())
        self._executor = executor

    def cancel(self):
        if self.done():
            return
        cancelled = super(RobotAction, self).cancel()
        if not cancelled: # already running
            self._executor.signal_cancellation(self)
            try:
                self.exception(timeout = 0.5) # waits this amount of time for the task to effectively complete
            except TimeoutError:
                raise RuntimeError("Unable to cancel action %s!" % self.actionname)

    def wait(self):
        """ alias for result()
        """
        return self.result()

    def __lt__(self, other):
        """ Overrides the comparision operator (used by ==, !=, <, >) to
        first wait for the result of the future.
        """
        return self.result().__lt__(other)

    def __le__(self, other):
        return self.result().__le__(other)

    def __eq__(self, other):
        return self.result().__eq__(other)

    def __ne__(self, other):
        return self.result().__ne__(other)

    def __gt__(self, other):
        return self.result().__gt__(other)

    def __ge__(self, other):
        return self.result().__ge__(other)

    def __repr__(self):
        """ Overrides the representation function to 
        first wait for the result of the future.
        """
        return self.result().__repr__()


class RobotActionExecutor(ThreadPoolExecutor):

    def __init__(self, max_workers):
        ThreadPoolExecutor.__init__(self, max_workers)

        self._threads_id = dict()

        self.futures = []


    def submit(self, fn, *args, **kwargs):
        with self._shutdown_lock:
            if self._shutdown:
                raise RuntimeError('cannot schedule new futures after shutdown')

            name = fn.__name__
            if args and not kwargs:
                name += "(%s)" % ", ".join([str(a) for a in args[1:]]) # start at 1 because 0 is the robot instance
            elif kwargs and not args:
                name += "(%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])
            elif args and kwargs:
                name += "(%s, " % ", ".join([str(a) for a in args[1:]])
                name += "%s)" % ", ".join(["%s=%s" % (str(k), str(v)) for k, v in kwargs.items()])

            f = RobotAction(self, name)
            w = _RobotWorkItem(f, fn, args, kwargs)

            self._work_queue.put(w)
            self._adjust_thread_count()

            self.futures.append(f)
            return f

    def _adjust_thread_count(self):
        # TODO(bquinlan): Should avoid creating new threads if there are more
        # idle threads than items in the work queue.
        if len(self._threads) < self._max_workers:
            t = PausableThread(target=_worker,
                                 args=(weakref.ref(self), self._work_queue))
            t.daemon = True
            t.start()
            self._threads.add(t)
            self._threads_id[t.ident] = weakref.ref(t)
            _thread_references.add(weakref.ref(t))


    def _get_thread_for_action(self, action_id):
        if action_id not in _actions_threads:
            return None
        else:
            return self._threads_id[_actions_threads[action_id]]() # weakref!


    def signal_cancellation(self, future):

        t = self._get_thread_for_action(future.id)
        if t is not None:
            t.cancel() # signal that we cancel the action


    def cancel_all(self):
        """ Blocks until all the currently running actions are actually stopped.
        """
        for f in self.futures:
            f.cancel()
