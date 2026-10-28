"""Non-blocking remote key adapter; only real KeyRelease can commit a click."""
import time
from .toggle_keys import ToggleKeysMixin as ParentKeys
from .async_x11 import AsyncX11KeyState


class RemoteKeysMixin(ParentKeys):
    def remote_input(self):return isinstance(self.key_state,AsyncX11KeyState)

    def press(self,event=None):
        if self.remote_input() and not self.model.key_down:
            self._remote_press_at=time.monotonic()
        return super().press(event)

    def cancel_release(self):
        self._remote_release=None
        return super().cancel_release()

    def release(self,event=None):
        if not self.remote_input():return super().release(event)
        self.cancel_release()
        # A cache sample taken BEFORE this real release must not commit it.
        # The next fresh, post-event key-up sample can confirm this event only.
        self._remote_release=(time.monotonic(),self.model.click_started)
        return 'break'

    def _confirm_remote_release(self,now,sample):
        pending=getattr(self,'_remote_release',None)
        if pending is None:return False
        arrived,click_started=pending
        if not 0<=now-arrived<=.100 or self.closing:
            self._remote_release=None;self.model.observe_physical_release(now);return True
        if sample.started>=arrived and not sample.down:
            self._remote_release=None
            if self.model.click_started==click_started:
                self.finish_release()  # Existing commit=True, with all gates.
            else:self.model.observe_physical_release(now)
            return True
        return False

    def _observe_cached_release(self,now,sample):
        # A cached key-up from BEFORE KeyPress says nothing about this click.
        # While a start click is pending, wait for the REAL release event (or
        # the unchanged click timeout). The polling connection may observe
        # release before Tk receives its event; it must not cancel that click.
        if (self.model.key_down and not self.model.click_pending and not sample.down
                and sample.started>=getattr(self,'_remote_press_at',float('inf'))
                and getattr(self,'_remote_release',None) is None):
            self.model.observe_physical_release(now)

    def loop(self):
        if not self.remote_input():return super().loop()
        self.loop_job=None
        if self.closing:return
        try:sample=self.key_state.snapshot()
        except (OSError,RuntimeError):
            self.server_space_down()  # Existing fail-closed revoke/close path.
            return
        now=time.monotonic()
        if not self._confirm_remote_release(now,sample):
            self._observe_cached_release(now,sample)
        self.node.pump()  # Sole lease producer, STILL on live GUI thread.
        now=time.monotonic()
        if now-self.last_draw>=.20:
            self.draw(now);self.last_draw=now
        if not self.closing:self.loop_job=self.root.after(20,self.loop)
