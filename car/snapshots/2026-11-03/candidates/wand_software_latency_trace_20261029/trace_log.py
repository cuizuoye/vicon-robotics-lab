"""Bounded asynchronous sidecar; no control envelope fields are changed."""
import json
from pathlib import Path
import queue
import threading
import time


class TraceLog:
    def __init__(self, folder, capacity=16384):
        self.folder = Path(folder)
        self.queue = queue.Queue(maxsize=capacity)
        self.dropped = 0
        self.written = 0
        self.error = ''
        self.closed = False
        self.costs = []
        self.thread = threading.Thread(target=self.run, name='latency-sidecar-writer', daemon=True)
        self.thread.start()

    def emit(self, kind, **value):
        start = time.monotonic_ns()
        row = dict(kind=kind, at_ns=start, **value)
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1
        self.costs.append(time.monotonic_ns()-start)

    def run(self):
        try:
            with (self.folder/'trace.jsonl').open('w', buffering=262144) as stream:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')
                    self.written += 1
        except Exception as error:
            self.error = repr(error)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.thread.is_alive():
            self.queue.put(None, timeout=2.)
            self.thread.join(timeout=3.)
        summary = dict(dropped=self.dropped, written=self.written, error=self.error,
                       writer_stopped=not self.thread.is_alive(), enqueue_cost_ns=self.costs)
        (self.folder/'TRACE_LOG_RESULT.json').write_text(json.dumps(summary)+'\n')


def control_id(frame):
    if frame is None:
        return None
    if isinstance(frame, dict):
        return [frame['source'], frame['sequence'], frame['epoch']]
    return [frame.source, frame.sequence, frame.epoch]


def output_id(payload):
    return [payload['source'], payload['sequence']]


class PublishTap:
    def __init__(self, publisher, publish):
        self.publisher, self.publish = publisher, publish

    def __getattr__(self, name):
        return getattr(self.publisher, name)
