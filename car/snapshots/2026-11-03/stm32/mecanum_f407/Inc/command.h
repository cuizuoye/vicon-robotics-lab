#ifndef COMMAND_H
#define COMMAND_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMMAND_INVALID = 0,
    COMMAND_HELP,
    COMMAND_STATUS,
    COMMAND_HEARTBEAT,
    COMMAND_ARM,
    COMMAND_DUTY,
    COMMAND_SPEED,
    COMMAND_BRAKE,
    COMMAND_STOP,
    COMMAND_ESTOP,
    COMMAND_CLEAR,
    COMMAND_ZERO_ENCODER,
} command_type_t;

typedef struct {
    command_type_t type;
    int32_t argument;
} command_t;

bool command_parse(const char *line, command_t *command);

#endif
