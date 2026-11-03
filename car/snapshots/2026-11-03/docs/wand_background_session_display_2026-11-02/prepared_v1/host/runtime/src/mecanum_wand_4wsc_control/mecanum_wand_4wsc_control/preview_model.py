"""Read-only mapping state. No velocity computation or transport exists here."""

from dataclasses import dataclass, field
import math

from .core import Pose, RelativeMapping


@dataclass
class StreamQuality:
    count: int = 0
    invalid: int = 0
    duplicate: int = 0
    first_s: float | None = None
    last_s: float | None = None
    last_stamp_ns: int | None = None
    max_gap_s: float = 0.0
    max_age_s: float = 0.0
    mean: list = field(default_factory=lambda: [0.0]*3)
    m2: list = field(default_factory=lambda: [0.0]*3)
    low: list = field(default_factory=lambda: [math.inf]*3)
    high: list = field(default_factory=lambda: [-math.inf]*3)

    def accept(self, pose, stamp_ns, received_s, age_s):
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.duplicate += 1
            return False
        if self.last_s is not None:
            self.max_gap_s = max(self.max_gap_s, received_s-self.last_s)
        if self.first_s is None:
            self.first_s = received_s
        self.last_s, self.last_stamp_ns = received_s, stamp_ns
        self.max_age_s = max(self.max_age_s, age_s)
        self.count += 1
        for i, value in enumerate((pose.x, pose.y, pose.z)):
            delta = value-self.mean[i]
            self.mean[i] += delta/self.count
            self.m2[i] += delta*(value-self.mean[i])
            self.low[i], self.high[i] = min(self.low[i], value), max(self.high[i], value)
        return True

    def snapshot(self):
        elapsed = 0 if self.first_s is None else self.last_s-self.first_s
        span = [b-a for a, b in zip(self.low, self.high)] if self.count else None
        return {
            'count': self.count, 'invalid': self.invalid, 'duplicate_or_reordered': self.duplicate,
            'hz': (self.count-1)/elapsed if elapsed > 0 else 0.0,
            'max_receive_gap_s': self.max_gap_s, 'max_receiver_to_callback_age_s': self.max_age_s,
            'xyz_span_m': span,
            'xyz_std_m': [math.sqrt(v/self.count) for v in self.m2] if self.count else None,
            'stationary_candidate': bool(span and max(span) < .01),
        }


class ReadonlyMapping:
    def __init__(self, scale=2.0, timeout_s=.20):
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('scale must be finite and positive')
        self.scale, self.timeout_s = scale, timeout_s
        self.samples = {}
        self.mapping = None
        self.state, self.reason = 'WAITING', 'waiting for both rigid bodies'
        self.capture_count = 0

    def receive(self, source, pose, received_s):
        if not pose.valid():
            self.invalidate(source, 'invalid pose')
            return
        old = self.samples.get(source)
        self.samples[source] = (pose, received_s)
        if (self.mapping and old and self.state == 'PREVIEW'
                and math.dist((pose.x, pose.y), (old[0].x, old[0].y)) > .25):
            self.state, self.reason = 'RECAPTURE_REQUIRED', f'{source} position jump'

    def invalidate(self, source, reason):
        self.samples.pop(source, None)
        if self.mapping:
            self.state, self.reason = 'RECAPTURE_REQUIRED', f'{source}: {reason}'

    def fresh(self, now):
        return all(source in self.samples and 0 <= now-self.samples[source][1] <= self.timeout_s
                   for source in ('wand', 'car'))

    def capture(self, now):
        if not self.fresh(now):
            return False, 'fresh Sign and car01 rigid-body poses are required'
        self.mapping = RelativeMapping(self.samples['car'][0], self.samples['wand'][0], self.scale)
        self.capture_count += 1
        self.state, self.reason = 'PREVIEW', ''
        return True, 'preview origins captured; no actuator output exists'

    def target(self, now):
        if not self.fresh(now):
            if self.mapping:
                self.state, self.reason = 'RECAPTURE_REQUIRED', 'rigid-body stream timeout'
            return None
        if self.mapping is None or self.state != 'PREVIEW':
            return None
        return self.mapping.goal(self.samples['wand'][0])

    def pause(self):
        self.state, self.reason = 'PAUSED', 'preview paused; recapture to resume'
