"""Real click only; a pending query neither starts nor renews a UI lease."""
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
        self._remote_release=(time.monotonic(),self.model.click_started)
        return 'break'

    def _confirm_remote_release(self,now,sample):
        pending=getattr(self,'_remote_release',None)
        if pending is None:return False
        arrived,click_started=pending
        if not 0<=now-arrived<=.100 or self.closing:
            self._remote_release=None;self.model.observe_physical_release(now);return True
        if sample is not None and sample.started>=arrived and not sample.down:
            self._remote_release=None
            if self.model.click_started==click_started:self.finish_release()
            else:self.model.observe_physical_release(now)
            return True
        return False

    def _observe_cached_release(self,now,sample):
        if (sample is not None and self.model.key_down and not self.model.click_pending and not sample.down
                and sample.started>=getattr(self,'_remote_press_at',float('inf'))
                and getattr(self,'_remote_release',None) is None):
            self.model.observe_physical_release(now)

    def loop(self):
        if not self.remote_input():return super().loop()
        self.loop_job=None
        if self.closing:return
        try:sample=self.key_state.sample_or_pending()
        except (OSError,RuntimeError) as error:
            self.model.revoke(time.monotonic(),'远程输入链路失去响应：已撤能，请重开操作台检查')
            self.node.get_logger().error(str(error));self.close();return
        now=time.monotonic()
        if not self._confirm_remote_release(now,sample):self._observe_cached_release(now,sample)
        # ONLY the main UI thread pumps ROS / publishes leases. No reply,
        # timer, polling thread or post-stall callback can grant authority.
        self.node.pump()
        now=time.monotonic()
        if now-self.last_draw>=.20:
            self.draw(now);self.last_draw=now
        if not self.closing:self.loop_job=self.root.after(20,self.loop)
