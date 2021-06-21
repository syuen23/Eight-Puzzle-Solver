import heapq
import itertools


class PriorityQueue:
    """Basic priority queue class.

    This class is a wrapper around around a wrapper around heapq, as described here:
    https://docs.python.org/3.7/library/heapq.html

    """

    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.entry_finder = {}  # maps task to entry (which is a [prior, count, task] list)
        self.REMOVED = '<removed-task>'  # placeholder for a removed task
        self.counter = itertools.count()  # unique sequence count

    def add(self, task, priority=0):
        """Add a new task or update the priority of an existing task.
        
        Args:
            task: any hashable python object or primitive type
            priority: priority level associated with the task (lower priorities are popped first!)
        """
        if task in self.entry_finder:
            self.remove(task)
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.pq, entry)

    def remove(self, task):
        """Mark an existing task as REMOVED.  Raise KeyError if not found."""
        entry = self.entry_finder.pop(task)
        entry[-1] = self.REMOVED

    def get(self, task):
        """Get the priority of a given task."""
        return self.entry_finder[task][0]

    def pop(self):
        """Remove and return the lowest priority task. Raise KeyError if empty."""
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            if task is not self.REMOVED:
                del self.entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        """Return true if the queue is empty."""
        return len(self.entry_finder) == 0

    def __contains__(self, key):
        return key in self.entry_finder

    def __len__(self):
        return len(self.entry_finder)

    def __str__(self):
        burn_heap = self.pq[:]
        rets = []
        while len(burn_heap) > 0:
            p, c, t = heapq.heappop(burn_heap)
            if str(t) != self.REMOVED:
                rets.append("{}: {}".format(p, t))
        return ", ".join(rets)
