#include "command.h"

#include <ctype.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

static const char *skip_space(const char *text)
{
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    return text;
}

static bool at_end(const char *text)
{
    return *skip_space(text) == '\0';
}

static bool match_word(const char *text, const char *word, const char **rest)
{
    const size_t length = strlen(word);
    if (strncmp(text, word, length) != 0) {
        return false;
    }
    if (text[length] != '\0' && text[length] != ' ' && text[length] != '\t') {
        return false;
    }
    *rest = text + length;
    return true;
}

static bool parse_decimal(const char *text, int32_t *value, const char **rest)
{
    text = skip_space(text);
    bool negative = false;
    if (*text == '+' || *text == '-') {
        negative = *text == '-';
        ++text;
    }
    if (!isdigit((unsigned char)*text)) {
        return false;
    }

    int64_t result = 0;
    while (isdigit((unsigned char)*text)) {
        result = result * 10 + (*text - '0');
        if (result > (int64_t)INT32_MAX + (negative ? 1 : 0)) {
            return false;
        }
        ++text;
    }
    *value = negative ? (int32_t)-result : (int32_t)result;
    *rest = text;
    return true;
}

static bool parse_hex_byte(const char *text, int32_t *value, const char **rest)
{
    text = skip_space(text);
    int32_t result = 0;
    unsigned digits = 0U;
    while (digits < 2U && isxdigit((unsigned char)*text)) {
        const char character = (char)tolower((unsigned char)*text);
        const int digit = isdigit((unsigned char)character)
            ? character - '0'
            : character - 'a' + 10;
        result = result * 16 + digit;
        ++digits;
        ++text;
    }
    if (digits == 0U) {
        return false;
    }
    *value = result;
    *rest = text;
    return true;
}

bool command_parse(const char *line, command_t *command)
{
    if (line == NULL || command == NULL) {
        return false;
    }

    line = skip_space(line);
    command->type = COMMAND_INVALID;
    command->argument = 0;
    const char *rest = NULL;

    struct fixed_command {
        const char *word;
        command_type_t type;
    };
    static const struct fixed_command fixed[] = {
        {"help", COMMAND_HELP},
        {"status", COMMAND_STATUS},
        {"hb", COMMAND_HEARTBEAT},
        {"brake", COMMAND_BRAKE},
        {"stop", COMMAND_STOP},
        {"estop", COMMAND_ESTOP},
        {"zero", COMMAND_ZERO_ENCODER},
    };

    for (size_t index = 0U; index < sizeof(fixed) / sizeof(fixed[0]); ++index) {
        if (match_word(line, fixed[index].word, &rest) && at_end(rest)) {
            command->type = fixed[index].type;
            return true;
        }
    }

    if (match_word(line, "duty", &rest) &&
        parse_decimal(rest, &command->argument, &rest) && at_end(rest)) {
        command->type = COMMAND_DUTY;
        return true;
    }
    if (match_word(line, "speed", &rest) &&
        parse_decimal(rest, &command->argument, &rest) && at_end(rest)) {
        command->type = COMMAND_SPEED;
        return true;
    }
    if (match_word(line, "arm", &rest) &&
        parse_hex_byte(rest, &command->argument, &rest) && at_end(rest)) {
        command->type = COMMAND_ARM;
        return true;
    }
    if (match_word(line, "clear", &rest) &&
        parse_hex_byte(rest, &command->argument, &rest) && at_end(rest)) {
        command->type = COMMAND_CLEAR;
        return true;
    }
    return false;
}
