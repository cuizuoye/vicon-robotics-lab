"""A read-only X11 worker, NEVER an operator-lease/authority worker.

Replies retain query START time. Slow or frozen X11 cannot refresh input age.
The existing GUI-loop100ms and supervisor-lease150ms guards are unchanged.
"""
import os
import threading
import time
from dataclasses import dataclass
from .x11_key_state import X11KeyState as SyncReader


def remote_display():
    return bool(os.environ.get('WAND_REMOTE_X11')=='1' or os.environ.get('SSH_CONNECTION')
        or os.environ.get('SSH_CLIENT') or os.environ.get('SSH_TTY')
        or os.environ.get('DISPLAY','').startswith(('localhost:','127.0.0.1:')))


@dataclass(frozen=True)
class KeySample:
    down:bool
    started:float
    completed:float


class AsyncX11KeyState:
    MAX_AGE_S=.080
    MAX_RTT_S=.060
    POLL_S=.015

    def __init__(self,display_name,*,reader_factory=SyncReader):
        self._stop=threading.Event();self._ready=threading.Event();self._lock=threading.Lock()
        self._sample=None;self._error='';self._closed=False
        def work():
            reader=None
            try:
                reader=reader_factory(display_name)
                while not self._stop.is_set():
                    start=time.monotonic();down=reader.down();end=time.monotonic()
                    if type(down) is not bool or not 0<=end-start<=self.MAX_RTT_S:
                        raise RuntimeError('X11 keyboard reply exceeded60ms; input revoked')
                    with self._lock:self._sample=KeySample(down,start,end)
                    self._ready.set()
                    if self._stop.wait(self.POLL_S):break
            except Exception as error:
                with self._lock:self._error=type(error).__name__+': '+str(error)
                self._ready.set()
            finally:
                if reader is not None:reader.close()
        self.thread=threading.Thread(target=work,name='wand-x11-readonly',daemon=True)
        self.thread.start()
        # Startup wait grants no authority. No Xlib call runs in the GUI thread.
        if not self._ready.wait(1.):
            self.close();raise RuntimeError('X11 keyboard startup timed out; no control authority')
        try:self.snapshot()
        except Exception:
            self.close();raise

    def snapshot(self):
        with self._lock:sample,error=self._sample,self._error
        now=time.monotonic()
        if self._closed or error or sample is None or not 0<=now-sample.started<=self.MAX_AGE_S:
            raise RuntimeError(error or 'X11 input sample expired; input revoked')
        return sample

    def down(self):return self.snapshot().down

    def close(self):
        self._closed=True;self._stop.set()
        # Only the worker owns/closes its X connection. A wedged daemon may
        # not block stopping the UI, nor does it ever issue a lease or service.
        if threading.current_thread() is not self.thread:self.thread.join(.1)


def open_key_state(display_name):
    return AsyncX11KeyState(display_name) if remote_display() else SyncReader(display_name)
