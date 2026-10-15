#include "mecanum_kinematics.h"

#include <assert.h>
#include <stdio.h>

static void expect_targets(
    const mecanum_wheel_targets_t *targets,
    int32_t fl,
    int32_t fr,
    int32_t rl,
    int32_t rr)
{
    assert(targets->wheel_mm_s[MECANUM_FRONT_LEFT] == fl);
    assert(targets->wheel_mm_s[MECANUM_FRONT_RIGHT] == fr);
    assert(targets->wheel_mm_s[MECANUM_REAR_LEFT] == rl);
    assert(targets->wheel_mm_s[MECANUM_REAR_RIGHT] == rr);
}

int main(void)
{
    mecanum_wheel_targets_t targets;

    assert(mecanum_inverse_kinematics(200, 0, 0, 100, 80, 1000, &targets));
    expect_targets(&targets, 200, 200, 200, 200);
    assert(targets.scale_permille == 1000U);

    assert(mecanum_inverse_kinematics(0, 100, 0, 100, 80, 1000, &targets));
    expect_targets(&targets, -100, 100, 100, -100);

    assert(mecanum_inverse_kinematics(0, 0, 1000, 100, 80, 1000, &targets));
    expect_targets(&targets, -180, 180, -180, 180);

    assert(mecanum_inverse_kinematics(400, 200, 1000, 100, 80, 390, &targets));
    expect_targets(&targets, 10, 390, 210, 190);
    assert(targets.scale_permille == 500U);

    assert(!mecanum_inverse_kinematics(0, 0, 0, 0, 0, 1000, &targets));
    assert(!mecanum_inverse_kinematics(0, 0, 0, 100, 80, 0, &targets));
    assert(!mecanum_inverse_kinematics(0, 0, 0, 100, 80, 1000, NULL));

    puts("mecanum_kinematics_test: PASS");
    return 0;
}
