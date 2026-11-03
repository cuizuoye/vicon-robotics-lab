"""Transport-only mailbox. Never creates a lease, nonce, timestamp or authority."""
from collections import deque
import queue
import threading

class Mailbox:
    def __init__(self,maxsize=128):
        self.maxsize=maxsize;self.critical=deque();self.view=None
        self.cv=threading.Condition();self.unfinished_tasks=0;self.coalesced=0;self.max_pending_views=0

    def put_nowait(self,value):
        with self.cv:
            if value['kind']=='state':
                if self.view is not None:self.coalesced+=1
                else:self.unfinished_tasks+=1
                self.view=value;self.max_pending_views=1
            else:
                if value['kind']=='result':
                    # No stale frame/challenge may overtake a terminal result.
                    self.unfinished_tasks-=len(self.critical)+(self.view is not None)
                    self.critical.clear();self.view=None;self.critical.appendleft(value)
                else:
                    if len(self.critical)>=self.maxsize:raise queue.Full
                    self.critical.append(value)
                self.unfinished_tasks+=1
            self.cv.notify()

    def get(self):
        with self.cv:
            while not self.critical and self.view is None:self.cv.wait()
            if self.critical:return self.critical.popleft()
            value,self.view=self.view,None;return value

    def task_done(self):
        with self.cv:self.unfinished_tasks-=1
