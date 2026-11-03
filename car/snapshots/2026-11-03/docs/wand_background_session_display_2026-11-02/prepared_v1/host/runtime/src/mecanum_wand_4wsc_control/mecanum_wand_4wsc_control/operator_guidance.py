"""Read-only operator geometry/messages; does not grant authority or clip input."""
import math

from .core import Pose


REASONS = {
    'GOAL_OUTSIDE_FIXED_ARENA': '水平目标超出固定围栏（不是Z高度超限）',
    'CAR_OUTSIDE_FIXED_ARENA': '车体及安全余量越过固定围栏',
    'OPERATOR_ENABLE_RELEASED': '空格使能已释放',
    'CONTROL_TIMER_GAP': '控制回调超过100ms，保护锁存',
    'COMMAND_TIMEOUT': '尚无有效运动会话／指令已超时',
    'WAND_POSE_TIMEOUT': '真实Sign被遮挡或定位数据超时',
    'CAR_POSE_TIMEOUT': '小车定位数据超时',
    'FEEDBACK_TIMEOUT': '真实电机反馈超过200ms未更新；已停车，数据恢复不会自动续跑',
    'TRANSPORT_MCU_FEEDBACK_TIMEOUT': '底层真实反馈超过200ms未更新，旧运动会话已作废',
    'TRACKING_ERROR_LIMIT': '车与即时跟踪点偏差超过65厘米',
    'FINAL_REFERENCE_STEP_LIMIT': '即时跟踪点变化过快，末级安全门拒绝输出',
    'CAR_PROGRESS_TIMEOUT': '连续有平移指令但3秒内车未前进至少1厘米；请检查阻力或驱动',
    'RECOVERY_WAIT_STABLE': '已确认恢复，等待连续稳定反馈后建立新会话',
    'RECOVERY_CANCELLED': '重新准备已取消：反馈再次中断、提前按键或等待超时；需再次手动确认',
    'CAR_LIFT_OR_TILT': '小车被抬起或倾斜（此检查不用于杆高度）',
    'SESSION_TIME_LIMIT': '本轮会话已达到配置时限',
    'ESTOP': '仿真急停已按下并锁存',
    'explicit start required': '等待手动准备',
    'waiting for a new held-enable edge': '已准备，等待新的持续按键',
    'operator pause': '操作员暂停',
    'operator stop': '操作员停止',
}


def reason_text(reason):
    if reason.startswith('FINAL_GATE: '):
        return '末级安全门：'+reason_text(reason[len('FINAL_GATE: '):])
    return REASONS.get(reason, reason or '无故障')


def center_bounds(config, yaw):
    return config.center_bounds(yaw)


def wand_preview(config, car_origin, wand_origin, wand):
    """Exact unfiltered XY goal, height display, and conservative radial budget."""
    dx, dy, dz = wand.x-wand_origin.x, wand.y-wand_origin.y, wand.z-wand_origin.z
    goal = Pose(car_origin.x+dx, car_origin.y+dy, car_origin.z, car_origin.yaw)
    projected = (config.project_goal(goal, car_origin, config.max_goal_radius_m)
                 if config.goal_boundary_policy == 'clamp' else goal)
    xmin, xmax, ymin, ymax = center_bounds(config, car_origin.yaw)
    budget = max(0., min(car_origin.x-xmin, xmax-car_origin.x,
                         car_origin.y-ymin, ymax-car_origin.y))
    if hasattr(config, 'goal_clearance_radius'):
        budget = config.goal_clearance_radius(car_origin)
    budget = min(budget, config.max_goal_radius_m)
    return dict(dx=dx, dy=dy, dz=dz, height=wand.z, goal=goal,
                projected_goal=projected,
                limited=math.hypot(goal.x-projected.x, goal.y-projected.y)>1e-9,
                horizontal_m=math.hypot(dx, dy), radial_budget_m=budget,
                suggested_step_m=min(.10, budget*.8), inside=config.inside(goal))


def real_instruction(model, now, hardware=False):
    if model.closing: return model.instruction(now)
    state = model.status.get('state')
    if model.status.get('hard_fault') and state == 'FAULT':
        if hardware:
            return '实车保护已锁存：'+reason_text(model.status.get('reason', ''))+'；先松开空格并保持停车，实车故障/急停不能用确认按钮解除'
        return '保护已锁存：'+reason_text(model.status.get('reason', ''))+'；仅需重启本仿真，不用给实车上电'
    if state == 'FAULT':
        step = ('先松开空格' if model.key_down or model.held else
                '先轻按并松开一次空格确认松键' if model.blocked else
                '等待停稳确认' if not model.status.get('stop_confirmed') else
                '等待反馈恢复' if model.status.get('input_health') else
                '点击“确认并重新准备”（也可仅确认可恢复故障）；不要点击急停')
        return '已保护停车：'+reason_text(model.status.get('reason', ''))+'；'+step
    instruction = model.instruction(now).replace('拖动虚拟杆', '移动真实Sign').replace('拖动', '移动杆')
    if state == 'ACTIVE' and model.held and model.status.get('goal_reference_policy') == 'governed':
        return ('保持按住空格：橙色为1:1位移最终目标，青色跟踪点低速引导；'
                +('最终目标已按围栏限幅；' if model.status.get('goal_limited') else '')
                +'杆可先到目标并拿稳，车逐步追上；收回杆则逐步返回，松键立即停车')
    if state == 'ACTIVE' and model.held and model.status.get('goal_limited'):
        return '目标已到小车安全边界：正在限幅，不是故障；杆可继续活动或收回，松开空格即停车'
    if (state == 'ARMED' and not model.held and not model.blocked and not model.key_down
            and model.status.get('gate_ready')):
        return '准备就绪：先把杆拿稳，再持续按住空格记录原点；只映射水平XY，Z高度不映射'
    if state == 'ARMED' and not model.status.get('gate_ready') and not model.held:
        stage = dict(WAIT_FEEDBACK='等待真实电机反馈恢复',
                     RESET_HEALTHY_BUDGET='重置已结束的健康会话计时（不清除真实故障）',
                     WAIT_DISABLED_ZERO='等待真实失能及零输出回执',
                     WAIT_BASELINE='累计连续0.6秒失能反馈',
                     BASELINE_READY='等待末级安全门确认').get(model.status.get('transport_preparation'),
                                                             '等待独立安全门握手')
        return '准备中：'+stage+'；保持空格松开，不要提前按键'
    if state in ('STANDBY', 'PAUSED') and model.can_start(now):
        return '先持杆到舒适高度并拿稳，再点准备；按住空格的瞬间才记录原点'
    return instruction
