#include "bench_dual_control.h"

#include <stddef.h>
#include <string.h>

#if !defined(BENCH_FRONT_ENCODERS_READONLY)
typedef struct {
    const char *line;
    uint8_t mask;
    int16_t a;
    int16_t b;
    const char *reply;
} pulse_command_t;

/* 白名单固定10%，不提供升幅、连续调速、在线反转或故障清除。 */
static const pulse_command_t pulses[] = {
#if defined(BENCH_FRONT_DEMO30)
    /* 仅当前安装已实测的共同前进组合，不提供其它方向。 */
    {"duty AB 100 -100", 3U, 100, -100, "OK,duty=A:100,B:-100\r\n"},
#else
    {"duty A 100", 1U, 100, 0, "OK,duty=A:100,B:0\r\n"},
    {"duty A -100", 1U, -100, 0, "OK,duty=A:-100,B:0\r\n"},
    {"duty B 100", 2U, 0, 100, "OK,duty=A:0,B:100\r\n"},
    {"duty B -100", 2U, 0, -100, "OK,duty=A:0,B:-100\r\n"},
    {"duty AB 100 100", 3U, 100, 100, "OK,duty=A:100,B:100\r\n"},
    {"duty AB 100 -100", 3U, 100, -100, "OK,duty=A:100,B:-100\r\n"},
    {"duty AB -100 100", 3U, -100, 100, "OK,duty=A:-100,B:100\r\n"},
    {"duty AB -100 -100", 3U, -100, -100, "OK,duty=A:-100,B:-100\r\n"},
#endif
};
#endif

bool bench_dual_pwm_compare(int16_t duty, uint32_t *compare1, uint32_t *compare2)
{
    if (compare1 == NULL || compare2 == NULL) {
        return false;
    }
    /* D157B单极性慢衰减：100/90为+10%，90/100为-10%，0/0停机。 */
    *compare1 = 0U;
    *compare2 = 0U;
    if (duty == 100) {
        *compare1 = 100U;
        *compare2 = 90U;
    } else if (duty == -100) {
        *compare1 = 90U;
        *compare2 = 100U;
    } else if (duty != 0) {
        return false;
    }
    return true;
}

void bench_dual_init(bench_dual_control_t *control)
{
    memset(control, 0, sizeof(*control));
}

void bench_dual_stop(bench_dual_control_t *control, uint32_t fault)
{
    /* 任一通道异常均停止两路；故障锁存，不借stop或hb清除。 */
    control->armed = false;
    control->heartbeat_seen = false;
    control->pulse_used = false;
    control->forward_observe_only = false;
    control->pulse_limit_ms = 0U;
    control->mask = 0U;
    control->duty_a = 0;
    control->duty_b = 0;
    control->fault_bits |= fault;
}

void bench_dual_tick(bench_dual_control_t *control, uint32_t now_ms)
{
    if (!control->armed) {
        return;
    }
    if (!control->heartbeat_seen ||
        (uint32_t)(now_ms - control->last_heartbeat_ms) > BENCH_DUAL_HEARTBEAT_MS) {
        bench_dual_stop(control, BENCH_DUAL_FAULT_TIMEOUT);
    } else if (control->pulse_used &&
               (uint32_t)(now_ms - control->pulse_started_ms) >= control->pulse_limit_ms) {
        /* 心跳不能延长本轮；短测试主机2秒stop，专用观察5秒stop。 */
        bench_dual_stop(control, BENCH_DUAL_FAULT_PULSE_LIMIT);
    }
}

static const char *reject(bench_dual_control_t *control, const char *reply)
{
    bench_dual_stop(control, BENCH_DUAL_FAULT_PROTOCOL);
    return reply;
}

const char *bench_dual_accept(bench_dual_control_t *control,
                            const char *line, uint32_t now_ms)
{
    /* 先处理超时，再看新心跳，禁止迟到心跳复活已超时的动作。 */
    bench_dual_tick(control, now_ms);
    if (line == NULL) {
        return reject(control, "ERR,bad_command\r\n");
    }
    if (strcmp(line, "stop") == 0) {
        bench_dual_stop(control, 0U);
        return "OK,stopped_all\r\n";
    }
    if (strcmp(line, "estop") == 0) {
        bench_dual_stop(control, BENCH_DUAL_FAULT_ESTOP);
        return "OK,estop_all\r\n";
    }
#if !defined(BENCH_FRONT_ENCODERS_READONLY)
    if (strcmp(line, "hb") == 0) {
        control->heartbeat_seen = true;
        control->last_heartbeat_ms = now_ms;
        return NULL;
    }
#endif
    if (strcmp(line, "status") == 0) {
        return NULL;
    }

#if defined(BENCH_FRONT_ENCODERS_READONLY)
    /* 标定阶段仅stop/status/estop，旧arm/duty/hb也不能使能。 */
    return reject(control, "ERR,readonly_command\r\n");
#else
    uint8_t arm_mask = 0U;
    bool forward_observe_only = false;
    const char *arm_reply = NULL;
#if defined(BENCH_FRONT_DEMO30)
    if (strcmp(line, "arm AB A5 30s") == 0) {
        arm_mask = 3U;
        forward_observe_only = true;
        arm_reply = "OK,armed=AB30\r\n";
    }
#else
    if (strcmp(line, "arm A A5") == 0) {
        arm_mask = 1U;
        arm_reply = "OK,armed=A\r\n";
    } else if (strcmp(line, "arm B A5") == 0) {
        arm_mask = 2U;
        arm_reply = "OK,armed=B\r\n";
    } else if (strcmp(line, "arm AB A5") == 0) {
        arm_mask = 3U;
        arm_reply = "OK,armed=AB\r\n";
    } else if (strcmp(line, "arm AB A5 5s") == 0) {
        /* 只扩展明确授权的A+/B-观察，不延长其它单/双通道测试。 */
        arm_mask = 3U;
        forward_observe_only = true;
        arm_reply = "OK,armed=AB5\r\n";
    }
#endif
    if (arm_mask != 0U) {
        if (control->fault_bits != 0U) {
            return reject(control, "ERR,fault_latched\r\n");
        }
#if defined(BENCH_FRONT_DEMO30)
        if (control->demo_used) {
            return reject(control, "ERR,demo_already_used\r\n");
        }
#endif
        if (control->armed || control->duty_a != 0 || control->duty_b != 0) {
            return reject(control, "ERR,stop_before_arm\r\n");
        }
        if (!control->heartbeat_seen ||
            (uint32_t)(now_ms - control->last_heartbeat_ms) > BENCH_DUAL_HEARTBEAT_MS) {
            return reject(control, "ERR,heartbeat_required\r\n");
        }
        control->armed = true;
        control->mask = arm_mask;
        control->pulse_used = false;
        control->forward_observe_only = forward_observe_only;
#if defined(BENCH_FRONT_DEMO30)
        control->pulse_limit_ms = BENCH_FRONT_DEMO_LIMIT_MS;
#else
        control->pulse_limit_ms = forward_observe_only ?
            BENCH_DUAL_OBSERVE_LIMIT_MS : BENCH_DUAL_PULSE_LIMIT_MS;
#endif
        return arm_reply;
    }

    for (size_t index = 0; index < sizeof(pulses) / sizeof(pulses[0]); ++index) {
        const pulse_command_t *pulse = &pulses[index];
        if (strcmp(line, pulse->line) != 0) {
            continue;
        }
        if (!control->armed || control->fault_bits != 0U || control->mask != pulse->mask) {
            return reject(control, "ERR,channel_not_armed\r\n");
        }
        if (control->pulse_used) {
            return reject(control, "ERR,one_pulse_per_arm\r\n");
        }
        if (control->forward_observe_only && (pulse->a != 100 || pulse->b != -100)) {
            return reject(control, "ERR,observe_forward_only\r\n");
        }
        control->pulse_used = true;
#if defined(BENCH_FRONT_DEMO30)
        control->demo_used = true;
#endif
        control->pulse_started_ms = now_ms;
        control->duty_a = pulse->a;
        control->duty_b = pulse->b;
        return pulse->reply;
    }
    return reject(control, "ERR,bad_command\r\n");
#endif
}
