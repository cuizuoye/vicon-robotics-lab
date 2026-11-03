"""Separate input freshness from bounded query liveness; NO authority thread.

Query <=60ms, input <=80ms from query START, reply progress <=80ms from
completion. In-flight time is also <=60ms, so a late reply never freshens
health after a wedged query. GUI100ms, lease150ms remain independent.
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
    MAX_REPLY_GAP_S=.080
    POLL_S=.015

    def __init__(self,display_name,*,reader_factory=SyncReader):
        self._stop=threading.Event();self._ready=threading.Event();self._lock=threading.Lock()
        self._sample=None;self._error='';self._closed=False;self._inflight=None
        self._queries=0;self._max_rtt=0.;self._max_reply_gap=0.
        def work():
            reader=None
            try:
                reader=reader_factory(display_name)
                while not self._stop.is_set():
                    start=time.monotonic()
                    with self._lock:self._inflight=start
                    down=reader.down();end=time.monotonic()
                    if type(down) is not bool or not 0<=end-start<=self.MAX_RTT_S:
                        raise RuntimeError('X11 keyboard reply exceeded60ms; input revoked')
                    with self._lock:
                        if self._sample:self._max_reply_gap=max(self._max_reply_gap,end-self._sample.completed)
                        self._max_rtt=max(self._max_rtt,end-start);self._queries+=1
                        self._sample=KeySample(down,start,end);self._inflight=None
                    self._ready.set()
                    if self._stop.wait(self.POLL_S):break
            except Exception as error:
                with self._lock:self._error=type(error).__name__+': '+str(error)
                self._ready.set()
            finally:
                if reader is not None:reader.close()
        self.thread=threading.Thread(target=work,name='wand-x11-readonly',daemon=True)
        self.thread.start()
        if not self._ready.wait(1.):
            self.close();raise RuntimeError('X11 keyboard startup timed out; no control authority')
        try:self.sample_or_pending()
        except Exception:
            self.close();raise

    def sample_or_pending(self):
        with self._lock:sample,error,inflight=self._sample,self._error,self._inflight
        now=time.monotonic()
        if self._closed or error or sample is None:
            raise RuntimeError(error or 'X11 query unavailable; input revoked')
        if not 0<=now-sample.completed<=self.MAX_REPLY_GAP_S:
            raise RuntimeError('X11 reply progress expired80ms; input revoked')
        if inflight is not None and not 0<=now-inflight<=self.MAX_RTT_S:
            raise RuntimeError('X11 in-flight query exceeded60ms; input revoked')
        # A healthy in-flight query is NOT evidence of physical key release.
        # Return no input rather than relabeling an old sample as fresh.
        return sample if 0<=now-sample.started<=self.MAX_AGE_S else None

    def snapshot(self):
        sample=self.sample_or_pending()
        if sample is None:raise RuntimeError('X11 input pending: no fresh physical key sample')
        return sample

    def down(self):
        sample=self.sample_or_pending()
        return True if sample is None else sample.down  # Unknown cannot unlock/prepare.

    def diagnostics(self):
        with self._lock:
            return dict(queries=self._queries,max_rtt_s=self._max_rtt,
                max_reply_gap_s=self._max_reply_gap,error=self._error)

    def close(self):
        self._closed=True;self._stop.set()
        if threading.current_thread() is not self.thread:self.thread.join(.1)


def open_key_state(display_name):
    return AsyncX11KeyState(display_name) if remote_display() else SyncReader(display_name)
