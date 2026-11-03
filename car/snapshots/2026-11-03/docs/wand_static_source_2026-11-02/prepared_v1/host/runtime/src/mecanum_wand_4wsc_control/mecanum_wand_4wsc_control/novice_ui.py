"""Explain pre-enable blockers and acknowledge a physically released key only."""
HEALTH={
    'CAR_POSE_TIMEOUT':'未收到新鲜car01定位：检查车体标记是否可见',
    'WAND_POSE_TIMEOUT':'未收到新鲜Sign定位：拿稳标定杆，保持无遮挡',
    'FEEDBACK_TIMEOUT':'小车反馈不稳定或中断：检查电源、Tenda与接线；不能准备',
    'ACTUATOR_FAULT':'实车保护已锁存：保持松键，停止重试；断电检查后重新上电，不会自动清故障，不能通过确认按钮解除',
    'COMMAND_SOURCE_CONFLICT':'等待唯一控制源；若持续出现，请关闭其他小车操作台/矩形程序',
    'CONTROL_MODE_CONFLICT':'等待控制模式确认；不要同时开启矩形与标定杆入口',
    'CAR_LIFT_OR_TILT':'车体高度或倾斜异常：放稳小车后重新检查',
}


def prepare_connection_ok(model, now):
    if not hasattr(model,'transport_status'):return True
    t=model.transport_status
    return bool(0<=now-model.transport_at<=.25 and t.get('fresh')
        and t.get('startup_clock',{}).get('ready') is True
        and not t.get('local_stop') and not t.get('fault')
        and not t.get('safety_enabled') and not t.get('motion_gate_requested'))


def backend_hint(model, now):
    s=model.status
    if s.get('state')=='ACTIVE':return ''
    if s.get('hard_fault') or model.gate.get('hard_fault'):
        return HEALTH['ACTUATOR_FAULT']+'；详情：'+str(s.get('reason') or model.gate.get('fault'))
    if model.pilot_done:
        from .pose_quality import stop_hint
        return stop_hint(s,model.gate)
    health=s.get('input_health','')
    if hasattr(model,'transport_status') and health in ('','FEEDBACK_TIMEOUT'):
        t=model.transport_status; clock=t.get('startup_clock',{})
        if 0<=now-model.transport_at<=.25:
            if t.get('local_stop') or t.get('fault') or clock.get('fault'):
                return '启动通信保护已阻止准备；不自动清故障。详情：'+str(t.get('fault') or clock.get('fault') or '通信锁止')
            if clock and not clock.get('ready'):
                return '等待启动握手/连续零输出遥测建基准，尚未就绪：'+str(clock.get('phase'))
    if health:return HEALTH.get(health,'准备被阻止：'+health)
    if hasattr(model,'transport_status'):
        t=model.transport_status
        if not 0<=now-model.transport_at<=.25:return '等待小车通信进程反馈；不会使能'
        if t.get('local_stop') or t.get('fault'):
            return '通信安全门已锁止：保持松键；不要反复重开。详情：'+str(t.get('fault') or '遥测超时/发送异常；请检查通信后重新打开')
        if not t.get('fresh'):return HEALTH['FEEDBACK_TIMEOUT']
    return ''


def acknowledge_physical_release(model, space_down, now):
    if (space_down or model.held or model.key_down or model.closing or model.pilot_used
            or not model.focused or model.status.get('state') not in ('STANDBY','ARMED')
            or model.status.get('hard_fault') or model.gate.get('hard_fault')):
        return False
    if model.blocked:
        model.blocked=False
        model.record(now,'physical_space_release_observed_no_enable')
    return True
