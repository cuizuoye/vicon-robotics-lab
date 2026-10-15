#include "bench_four_control.h"
#include <stddef.h>
#include <string.h>

bool bench_four_compare(int16_t duty, uint32_t *first, uint32_t *second)
{
    if (first == NULL || second == NULL) return false;
    *first = 0U;
    *second = 0U;
    /* 沿用已验证D157B慢衰减映射；这是电气符号，不是假定的车体前进。 */
    if (duty == 100) { *first = 100U; *second = 90U; }
    else if (duty == -100) { *first = 90U; *second = 100U; }
    else if (duty != 0) return false;
    return true;
}

void bench_four_init(bench_four_t *c) { memset(c, 0, sizeof(*c)); }

void bench_four_stop(bench_four_t *c, uint32_t fault)
{
    c->selected = 0U;
    c->heartbeat_seen = false;
    c->pulse_used = false;
    memset(c->duty, 0, sizeof(c->duty));
    c->fault |= fault;
}

void bench_four_tick(bench_four_t *c, uint32_t now)
{
    if (c->selected == 0U) return;
    if (!c->heartbeat_seen || (uint32_t)(now - c->heartbeat_ms) > FOUR_HEARTBEAT_MS)
        bench_four_stop(c, FOUR_FAULT_TIMEOUT);
    else if (c->pulse_used && (uint32_t)(now - c->pulse_ms) >= FOUR_PULSE_LIMIT_MS)
        bench_four_stop(c, FOUR_FAULT_LIMIT);
}

static const char *reject(bench_four_t *c, const char *reply)
{
    bench_four_stop(c, FOUR_FAULT_PROTOCOL);
    return reply;
}

const char *bench_four_accept(bench_four_t *c, const char *line, uint32_t now)
{
    bench_four_tick(c, now);
    if (line == NULL) return reject(c, "ERR,bad_command\r\n");
    if (strcmp(line, "stop") == 0) {
        bench_four_stop(c, 0U);
        return "OK,stopped_all\r\n";
    }
    if (strcmp(line, "estop") == 0) {
        bench_four_stop(c, FOUR_FAULT_ESTOP);
        return "OK,estop_all\r\n";
    }
    if (strcmp(line, "status") == 0) return NULL;
    if (strcmp(line, "hb") == 0) {
        c->heartbeat_seen = true;
        c->heartbeat_ms = now;
        return NULL;
    }
    static const char *arms[] = {"arm FL A5", "arm FR A5", "arm RL A5", "arm RR A5"};
    static const char *arm_replies[] = {
        "OK,armed=FL\r\n", "OK,armed=FR\r\n", "OK,armed=RL\r\n", "OK,armed=RR\r\n"};
    static const char *duties[][2] = {
        {"duty FL 100", "duty FL -100"}, {"duty FR 100", "duty FR -100"},
        {"duty RL 100", "duty RL -100"}, {"duty RR 100", "duty RR -100"}};
    static const char *duty_replies[][2] = {
        {"OK,duty=FL:100\r\n", "OK,duty=FL:-100\r\n"},
        {"OK,duty=FR:100\r\n", "OK,duty=FR:-100\r\n"},
        {"OK,duty=RL:100\r\n", "OK,duty=RL:-100\r\n"},
        {"OK,duty=RR:100\r\n", "OK,duty=RR:-100\r\n"}};
    for (unsigned i = 0U; i < FOUR_WHEELS; ++i) {
        if (strcmp(line, arms[i]) == 0) {
            if (c->fault != 0U) return reject(c, "ERR,fault_latched\r\n");
            if (c->selected != 0U) return reject(c, "ERR,stop_before_arm\r\n");
            if (!c->heartbeat_seen || (uint32_t)(now - c->heartbeat_ms) > FOUR_HEARTBEAT_MS)
                return reject(c, "ERR,heartbeat_required\r\n");
            c->selected = (uint8_t)(i + 1U);
            c->pulse_used = false;
            return arm_replies[i];
        }
        for (unsigned sign = 0U; sign < 2U; ++sign) {
            if (strcmp(line, duties[i][sign]) != 0) continue;
            if (c->fault != 0U || c->selected != i + 1U)
                return reject(c, "ERR,channel_not_armed\r\n");
            if (c->pulse_used) return reject(c, "ERR,one_pulse_per_arm\r\n");
            c->pulse_used = true;
            c->pulse_ms = now;
            c->duty[i] = sign == 0U ? 100 : -100;
            return duty_replies[i][sign];
        }
    }
    /* 无任意占空比、在线换向、组合运动、清故障或在线写标定值。 */
    return reject(c, "ERR,bad_command\r\n");
}
