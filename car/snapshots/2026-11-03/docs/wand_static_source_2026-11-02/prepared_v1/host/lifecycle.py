"""Waiting-for-browser budget is distinct from the 600s ACTIVE authority cap."""
import math


def waiting_expired(start,now,active_since):
    if not all(math.isfinite(v) for v in (start,now)) or now<start:
        return True
    return active_since is None and now-start>180.
