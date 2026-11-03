#!/usr/bin/env bash
# Preserve SSH's DISPLAY and XAUTHORITY. Never apply desktop GDM auth remotely.
configure_tracking_display() {
    local task_display_check_only=false
    local task_arg
    for task_arg in "$@"; do
        if [ "$task_arg" = --check ]; then task_display_check_only=true; fi
    done
    if [ -n "${SSH_CONNECTION-}${SSH_CLIENT-}${SSH_TTY-}" ] || [[ ${DISPLAY-} == localhost:* || ${DISPLAY-} == 127.0.0.1:* ]]; then
        if [ "$task_display_check_only" != true ] && [ -z "${DISPLAY-}" ]; then
            printf '%s\n' 'SSH会话没有DISPLAY；为避免误开Linux本机桌面，停止启动。' >&2
            return 2
        fi
        # If unset, Xlib naturally reads the SSH user's ~/.Xauthority.
        # If explicitly supplied, retain the user's exact authority path.
        return 0
    fi
    export DISPLAY=${DISPLAY:-:1}
    if [ -z "${XAUTHORITY+x}" ] && [ -r /run/user/1000/gdm/Xauthority ]; then
        export XAUTHORITY=/run/user/1000/gdm/Xauthority
    fi
}
