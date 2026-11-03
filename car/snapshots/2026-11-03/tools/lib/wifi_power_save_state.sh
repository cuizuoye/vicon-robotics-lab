#!/usr/bin/env bash
# Pure Bash: works with sudo's restricted PATH, no Codex-specific rg required.
wifi_power_save_state() {
    if [[ "$1" =~ (^|[[:space:]])Power[[:space:]]+Management:(on|off)($|[[:space:]]) ]]; then
        printf '%s\n' "${BASH_REMATCH[2]}"
    else
        return 1
    fi
}

wifi_power_save_matches() {
    local task_observed
    task_observed="$(wifi_power_save_state "$1")" || return 1
    [[ "$task_observed" == "$2" ]]
}
