"""Socket readiness only in the worker; all decoding/authority stays in ROS.

No packet is read or sent here. One pending guard notification is coalesced
until the executor acknowledges it; no busy polling while a callback is late.
"""
import select
import threading


class ReceiveEventPump:
    def __init__(self, sock, guard):
        self.sock, self.guard = sock, guard
        self.error = ''
        self.stopping, self.dispatched = threading.Event(), threading.Event()
        self.thread = threading.Thread(target=self._run, name='wand-rx-ready', daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopping.is_set():
            try:
                ready, _, _ = select.select([self.sock], [], [], .05)
                if not ready:
                    continue
            except (OSError, ValueError, TypeError) as error:
                self.error = type(error).__name__
            self.dispatched.clear()
            if self.stopping.is_set():
                return
            try:
                self.guard.trigger()
            except Exception as error:
                self.error = 'GUARD_TRIGGER_' + type(error).__name__
                return
            while not self.stopping.is_set() and not self.dispatched.wait(.05):
                pass
            if self.error:
                return

    def acknowledge(self):
        self.dispatched.set()

    def stop(self):
        self.stopping.set()
        self.dispatched.set()
        self.thread.join(.20)
        return not self.thread.is_alive()
