"""Bounded software-test logging in a separate process, never the control GIL."""
import json
import multiprocessing as mp
import queue
from pathlib import Path


def writer(channel,path):
    with Path(path).open('x',buffering=262144) as stream:
        while True:
            value=channel.get()
            if value is None:break
            stream.write(json.dumps(value,separators=(',',':'),allow_nan=False)+'\n')


class TraceSpool:
    def __init__(self,path):
        ctx=mp.get_context('spawn')
        self.channel=ctx.Queue(maxsize=512)
        self.process=ctx.Process(target=writer,args=(self.channel,str(path)),name='4wsb-test-trace-writer')
        self.process.start();self.rows=self.dropped=0

    def emit(self,value):
        try:self.channel.put_nowait(value);self.rows+=1
        except queue.Full:self.dropped+=1

    def close(self):
        if self.process.is_alive():
            self.channel.put(None,timeout=2.);self.process.join(timeout=4.)
        stopped=not self.process.is_alive()
        if not stopped:self.process.terminate();self.process.join(timeout=2.)
        result=dict(rows=self.rows,dropped=self.dropped,writer_stopped=stopped,
                    returncode=self.process.exitcode,capacity=512)
        self.channel.close();self.channel.cancel_join_thread()
        return result
