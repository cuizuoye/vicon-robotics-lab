"""Auto-open after fresh machine preflight; prerequisites are standing user defaults."""
import time


class AutomaticOpenMixin:
    def __init__(self,*args,**kwargs):
        self.auto_open_queued=self.auto_open_attempted=False
        super().__init__(*args,**kwargs)
        self.open_button.pack_forget()

    def auto_open_allowed(self):
        return (not self.preview and not self.closed and not self.busy and self.backend is None
            and not self.auto_open_attempted and 0<=time.monotonic()-self.good_at<=120)

    def update_buttons(self):
        super().update_buttons()
        if self.auto_open_allowed() and not self.auto_open_queued:
            self.auto_open_queued=True
            self.root.after_idle(self.auto_open_if_ready)

    def auto_open_if_ready(self):
        self.auto_open_queued=False
        if not self.auto_open_allowed():return
        self.auto_open_attempted=True
        self.open_console()

    def preflight(self):
        if not self.busy and self.backend is None and not self.preview:
            self.auto_open_attempted=False # Explicit button/initial launch, not a failure retry.
        return super().preflight()

