"""Synthetic hand motion in the engage-time car forward/left frame."""

DEMO_KEYFRAMES = [
    (0, 0, 0), (2, 0, 0),
    (12, 0.5, 0), (18, 0.5, 0),
    (28, 0, 0), (34, 0, 0),
    (40, 0, 0.25), (46, 0, 0.25),
    (52, 0, 0), (58, 0, 0),
    (68, 0.4, 0.3), (80, 0.4, 0.3),
    (90, 0, 0), (96, 0, 0),
]


def interpolate(t, keyframes=DEMO_KEYFRAMES):
    if t <= keyframes[0][0]:
        return keyframes[0][1:]
    for a, b in zip(keyframes, keyframes[1:]):
        if t <= b[0]:
            fraction = (t - a[0]) / (b[0] - a[0])
            return tuple(x + fraction * (y - x) for x, y in zip(a[1:], b[1:]))
    return keyframes[-1][1:]
