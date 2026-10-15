#include "speed_controller.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    speed_controller_t controller;
    int16_t duty = 123;
    speed_controller_init(&controller, 100U);
    assert(!speed_controller_enabled(&controller));
    assert(!speed_controller_tick(&controller, 0, 110U, &duty));

    speed_controller_set_target(&controller, 1000, 100U);
    assert(speed_controller_enabled(&controller));
    assert(speed_controller_target(&controller) == 1000);
    assert(!speed_controller_tick(&controller, 0, 105U, &duty));
    assert(speed_controller_tick(&controller, 0, 110U, &duty));
    assert(duty > 0 && duty <= 200);

    assert(speed_controller_tick(&controller, 2000, 120U, &duty));
    assert(duty < 0 && duty >= -200);

    speed_controller_set_target(&controller, 9000, 120U);
    assert(speed_controller_target(&controller) == 5000);
    speed_controller_set_target(&controller, 0, 120U);
    assert(speed_controller_tick(&controller, 100, 130U, &duty));
    assert(duty == 0);

    speed_controller_disable(&controller);
    assert(!speed_controller_enabled(&controller));
    assert(!speed_controller_tick(&controller, 0, 140U, &duty));

    puts("speed controller tests passed");
    return 0;
}
