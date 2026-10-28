"""Same fixed arena and guards; only offline follower selection differs."""
from dataclasses import dataclass, replace
from .feedback_policy import FEEDBACK_TIMEOUT_S
from .arena_config import ArenaSafetyConfig


@dataclass(frozen=True)
class ServoSafetyConfig(ArenaSafetyConfig):
    def __post_init__(self):
        super().__post_init__()
        # Only the explicit private MCU feedback deadline changes; other guards stay pinned.
        if (self.max_speed_mps != .20 or self.max_accel_mps2 != .08
                or self.max_yaw_rate_radps != .10 or self.max_yaw_accel_radps2 != .40
                or self.max_session_s != 120. or self.goal_boundary_policy != 'clamp'
                or self.goal_reference_policy != 'governed'):
            raise ValueError('offline servo must retain the existing 4W20/3x3 motion envelope')
        preserved = dict(pose_timeout_s=.20, lease_timeout_s=.15, command_timeout_s=.10,
            feedback_timeout_s=FEEDBACK_TIMEOUT_S, ready_window_s=.60, stationary_window_s=.50,
            stop_confirmation_timeout_s=1.50, max_control_gap_s=.10,
            max_clock_skew_s=.10, future_tolerance_s=.005, max_tracking_error_m=.65,
            lift_tolerance_m=.08, tilt_limit_rad=.25, car_jump_m=.20,
            car_jump_yaw_rad=.50, wand_jump_m=.25,
            arena_origin_x_m=.6437034797013303, arena_origin_y_m=-1.0528685411524479,
            arena_heading_rad=1.526260405304856, ground_z_m=.10790704618641533)
        if any(getattr(self,k) != v for k,v in preserved.items()):
            raise ValueError('offline servo cannot change established guards or saved arena reference')

    def follower_config(self):
        return replace(super().follower_config(), filter_tau_s=0.)

