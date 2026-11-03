"""Native key adapter. Real release completes a start click; polling cannot.

Stop is immediate on the next key-down or Escape, before any service reply.
No key synthesis, OS repeat settings, watchdog changes or hardware I/O here.
"""
import time


class ToggleKeysMixin:
    def press(self, _=None):
        self.cancel_release()
        now=time.monotonic()
        self.model.tick(now)  # A queued click after a UI stall cannot start.
        was_requested=self.model.session_requested
        self.model.press(now)
        self.node.heartbeat()
        if was_requested and not self.model.session_requested:
            self.node.request('stop')
        return 'break'

    def release(self, _=None):
        self.cancel_release()
        if self.key_state is not None:
            # X11 repeat produces release/press pairs while physically down.
            if not self.server_space_down() and not self.closing:self.finish_release()
            return 'break'
        self.release_job=self.root.after(5,self.finish_release)
        return 'break'

    def finish_release(self):
        self.release_job=None
        if not self.closing:self.model.release(time.monotonic(),commit=True)
        self.node.heartbeat()

    def escape(self, _=None):
        self.cancel_release()
        self.model.end_trial(time.monotonic(),'Esc停止：本轮已撤能，不会自动恢复')
        self.node.heartbeat()
        self.node.request('stop')
        return 'break'

    def loop(self):
        self.loop_job=None
        if self.closing:return
        if self.key_state is not None:
            down=self.server_space_down()  # Read failure still closes/stops.
            if self.closing:return
            if self.model.key_down and not down:
                self.model.observe_physical_release(time.monotonic())
        self.node.pump()
        now=time.monotonic()
        if now-self.last_draw>=.033:
            self.draw(now);self.last_draw=now
        if not self.closing:self.loop_job=self.root.after(20,self.loop)
