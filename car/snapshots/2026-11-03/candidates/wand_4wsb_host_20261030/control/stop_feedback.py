"""Standstill evidence over the EXISTING .5s window, not one encoder tick.

Integrate nonnegative conservative wheel-speed/yaw bounds. Out-and-back motion
cannot cancel. No enabled/nonzero sample, missing coverage or stale feedback
can qualify a window. The independent Vicon range estimator and1.5s stop
deadline remain. Both sensors must cover a COMPLETE .5s standstill window;
do not append another .5s timer after the already-windowed evidence. This
does not filter control feedback or renew any sensor timestamp.
"""
from collections import deque
from .rotation_servo import RotationInputs


class StopFeedbackInputs(RotationInputs):
    def __init__(self,c):
        super().__init__(c)
        self.feedback_history=deque()

    def observe_feedback(self,*args,**kw):
        accepted=super().observe_feedback(*args,**kw)
        if accepted:
            f=dict(self.feedback);history=self.feedback_history
            if history and f['issued_steady']<=history[-1]['issued_steady']:
                history.clear()  # no backwards clocks/coverage synthesis
            history.append(f)
            cutoff=f['issued_steady']-self.c.stationary_window_s
            while len(history)>2 and history[1]['issued_steady']<=cutoff:
                history.popleft()
            if len(history)>512:history.clear()  # bounded fail-closed
        elif self.feedback is None:
            self.feedback_history.clear()
        return accepted

    def feedback_still(self,steady,ros):
        if not self.feedback_fresh(steady,ros):return False
        history=self.feedback_history
        if len(history)<2:return False
        end=history[-1]['issued_steady'];window=self.c.stationary_window_s
        begin=end-window
        if history[0]['issued_steady']>begin+1e-9:return False
        distance=angle=0.
        for left,right in zip(history,list(history)[1:]):
            a,b=left['issued_steady'],right['issued_steady']
            if not 0<b-a<=self.c.feedback_timeout_s:return False
            if b<=begin:continue
            if any(x['enabled'] or not x['output_zero'] or x['fault_bits'] for x in (left,right)):
                return False
            dt=b-max(a,begin)
            # Upper rectangle, not signed endpoint cancellation/trapezoid.
            distance+=max(left['speed'],right['speed'])*dt
            angle+=max(left['yaw_rate'],right['yaw_rate'])*dt
        return distance<=.005*window+1e-12 and angle<=.02*window+1e-12

    def update_stationarity(self,steady,ros):
        still=(self.pose_fresh('car',steady,ros) and self.feedback_still(steady,ros)
            and self.car_speed<=.005 and self.car_yaw_rate<=.02)
        if not still:self.stationary_since=None
        elif self.stationary_since is None:self.stationary_since=steady
        # car_speed/yaw already cover every raw point for .5s; feedback_still
        # independently covers every disabled sample for .5s. A second dwell
        # here double-counted the window and consumed the1.5s stop deadline.
        return bool(still)
