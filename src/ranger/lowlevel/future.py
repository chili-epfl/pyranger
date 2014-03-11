"""
Extension of Python futures to support robot action management.

The main change is the storage of a unique 'action_id' for
each task submitted to the Executor in a way accessible to the
action itself (via threading.local().action_id).
"""
import uuid
import sys

try:
    from concurrent.futures import ThreadPoolExecutor, Future
except ImportError:
    import sys
    sys.stderr.write("[error] install python-concurrent.futures\n")
    sys.exit(1)

from threading import local

class _RobotWorkItem(object):
    def __init__(self, future, fn, args, kwargs):
        self.future = future
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):

        # Store the action UUID in the thread local storage.
        self.args[0].action_id.id = self.future.id

        if not self.future.set_running_or_notify_cancel():
            return

        try:
            result = self.fn(*self.args, **self.kwargs)
        except BaseException:
            e = sys.exc_info()[1]
            self.future.set_exception(e)
        else:
            self.future.set_result(result)

        # clear the thread's action_id to make clear we are done
        self.args[0].action_id.id = None

class RobotAction(Future):
    def __init__(self):
        Future.__init__(self)

        self.id = uuid.uuid4()

    def cancel(self):
        # TODO
        pass

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

        self.futures = []


    def submit(self, fn, *args, **kwargs):
        with self._shutdown_lock:
            if self._shutdown:
                raise RuntimeError('cannot schedule new futures after shutdown')

            #f = _base.Future()
            f = RobotAction()
            w = _RobotWorkItem(f, fn, args, kwargs)

            self._work_queue.put(w)
            self._adjust_thread_count()

            self.futures.append(f)
            return f


    def cancel_all(self):
        for f in self.futures:
            if not f.done():
                f.cancel()
