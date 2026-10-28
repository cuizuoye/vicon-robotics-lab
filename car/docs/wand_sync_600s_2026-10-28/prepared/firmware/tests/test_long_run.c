/* Actual firmware and protocol with synthetic encoders; never real actuation. */
#define main inherited_controller_main
#include "test_controller.c"
#undef main

int main(void)
{
    reset_at(4, 0);
    run_for(200, 0, 500, 151000);
    const uint32_t began = l.first_motion_ms;
    assert(now-began > 150000 && l.permitted && !b.group.fault);
    run_for(200, 0, 500, 450000);
    assert(now-began >= 600000 && l.permitted && !b.group.fault);
    while (now-began < FOUR_DRIVE_RUN_LIMIT_MS-10U) tick(true, 10);
    assert(l.first_motion_ms == began && l.permitted);
    tick(true, 10);
    assert(!l.permitted && (b.group.fault & FOUR_SPEED_FAULT_DURATION));
    check_outputs();
    for(unsigned i=0;i<4;i++) assert(!b.group.raw_duty[i]);
    /* No late enabled command or fresh session may erase real duration faults. */
    velocity(true);
    assert(!l.permitted && b.group.fault);
    puts("PASS: actual C survives600s, retains independent630s deadline, no budget renewal or auto-resume");
    return 0;
}
