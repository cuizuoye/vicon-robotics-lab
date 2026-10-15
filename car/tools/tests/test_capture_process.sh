#!/usr/bin/env bash
set -Eeuo pipefail
task_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=tools/lib/capture_process.sh
source "$task_test_root/lib/capture_process.sh"
task_dir="$(mktemp -d /tmp/mc_capture_mock_20261014_XXXXXXXX)"
task_mock_pid=''
cleanup_test() {
    if [[ -n "$task_mock_pid" ]]; then capture_stop "$task_mock_pid" || true; fi
}
trap cleanup_test EXIT
# Missing file must return false, not terminate the shell under set -e.
if capture_log_ready "$task_dir/missing.log" wlp114s0; then exit 1; fi
: > "$task_dir/capture.log"
if capture_log_ready "$task_dir/capture.log" wlp114s0; then exit 1; fi
/usr/bin/python3 "$task_test_root/tests/mock_capture_process.py" > "$task_dir/capture.log" 2>&1 &
task_mock_pid=$!
task_ready=false
for ((task_index=0;task_index<30;task_index++)); do
    if capture_log_ready "$task_dir/capture.log" wlp114s0; then task_ready=true; break; fi
    sleep 0.1
done
[[ "$task_ready" == true ]]
if capture_log_ready "$task_dir/capture.log" another_interface; then exit 1; fi
capture_stop "$task_mock_pid"
if kill -0 "$task_mock_pid" 2>/dev/null; then exit 1; fi
task_mock_pid=''
[[ "$(< "$task_dir/capture.log")" == *'capture cleanup complete'* ]]
printf 'PASS: missing/empty log, delayed readiness, exact interface, TERM flush/exit; no network or hardware.\n'
printf 'Mock evidence: %s\n' "$task_dir"
