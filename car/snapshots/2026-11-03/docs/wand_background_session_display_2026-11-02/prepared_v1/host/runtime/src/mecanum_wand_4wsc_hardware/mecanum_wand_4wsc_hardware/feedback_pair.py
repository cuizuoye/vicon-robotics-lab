"""A feedback proof requires BOTH original MCU sample clocks to advance."""
import math

def paired_advance(identity,previous):
    if len(identity)!=2 or any(type(t) not in (int,float) or not math.isfinite(t) for t in identity):return False
    if previous is None:return True
    return bool(len(previous)==2 and all(new>old for new,old in zip(identity,previous)))
