#!/usr/bin/env bash
# No network/privilege operations: safe to exercise with a local mock process.
capture_log_ready() {
    local task_log=$1 task_interface=$2 task_line=''
    [[ -r "$task_log" ]] || return 1
    while IFS= read -r task_line || [[ -n "$task_line" ]]; do
        if [[ "$task_line" == *"listening on $task_interface"* ]]; then
            return 0
        fi
    done < "$task_log"
    return 1
}

capture_stop() {
    local task_pid=$1 task_index
    # Noninteractive Bash background jobs may inherit SIGINT ignored.
    # tcpdump handles SIGTERM and closes/flushes its PCAP with final stats.
    kill -TERM "$task_pid" 2>/dev/null || true
    for ((task_index=0;task_index<30;task_index++)); do
        kill -0 "$task_pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$task_pid" 2>/dev/null; then
        kill -KILL "$task_pid" 2>/dev/null || true
    fi
    if wait "$task_pid"; then
        return 0
    else
        return $?
    fi
}
