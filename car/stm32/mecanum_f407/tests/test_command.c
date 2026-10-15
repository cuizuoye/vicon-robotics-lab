#include "command.h"

#include <assert.h>
#include <stdio.h>

static void expect(const char *text, command_type_t type, int32_t argument)
{
    command_t command = {0};
    assert(command_parse(text, &command));
    assert(command.type == type);
    assert(command.argument == argument);
}

int main(void)
{
    expect("help", COMMAND_HELP, 0);
    expect(" status  ", COMMAND_STATUS, 0);
    expect("hb", COMMAND_HEARTBEAT, 0);
    expect("arm A5", COMMAND_ARM, 0xA5);
    expect("clear 5a", COMMAND_CLEAR, 0x5A);
    expect("duty 200", COMMAND_DUTY, 200);
    expect("duty -200", COMMAND_DUTY, -200);
    expect("speed 1250", COMMAND_SPEED, 1250);
    expect("speed -900", COMMAND_SPEED, -900);
    expect("brake", COMMAND_BRAKE, 0);
    expect("stop", COMMAND_STOP, 0);
    expect("estop", COMMAND_ESTOP, 0);
    expect("zero", COMMAND_ZERO_ENCODER, 0);

    command_t invalid = {0};
    assert(!command_parse("", &invalid));
    assert(!command_parse("duty", &invalid));
    assert(!command_parse("duty 12x", &invalid));
    assert(!command_parse("arm", &invalid));
    assert(!command_parse("arm 123", &invalid));
    assert(!command_parse("unknown", &invalid));
    assert(!command_parse(NULL, &invalid));
    assert(!command_parse("status", NULL));

    puts("command parser tests passed");
    return 0;
}
