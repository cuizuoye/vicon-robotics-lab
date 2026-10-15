"""Pace fixed read-only parameter refreshes; never compete with shell replies."""


class ParameterPoll:
    def __init__(self, names, interval=.05, refresh=2.):
        self.names = tuple(names)
        if not self.names or interval <= 0 or refresh <= 0:
            raise ValueError('Nonempty fixed names and positive timing required')
        self.interval, self.refresh = interval, refresh
        self.pending = []
        self.next_write = self.next_cycle = 0.

    def due(self, now, *, identity_ready, shell_pending):
        if not identity_ready or shell_pending:
            return None
        if not self.pending and now >= self.next_cycle:
            self.pending = list(self.names)
            self.next_cycle = now + self.refresh
        if not self.pending or now < self.next_write:
            return None
        self.next_write = now + self.interval
        return self.pending.pop(0)
