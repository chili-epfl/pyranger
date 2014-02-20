from collections import deque

class valuefilter:

    MAX_LENGTH=10

    def __init__(self, maxlen = MAX_LENGTH):
        self._vals = deque(maxlen = maxlen)

        self.lastval = 0.
        self.dirty = True

    def append(self, val):
        self.dirty = True
        self._vals.append(val)

    def get(self):

        if self.dirty:
            self.lastval = sum(self._vals) / len(self._vals)
            self.dirty = False

        return self.lastval
