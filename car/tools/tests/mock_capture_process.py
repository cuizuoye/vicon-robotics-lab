"""No sockets/hardware; exercise background start and graceful TERM cleanup."""
import signal
import sys
import time

# Match the ignored SIGINT inherited by many Bash background jobs.
signal.signal(signal.SIGINT, signal.SIG_IGN)
def finish(_signal, _frame):
    print('capture cleanup complete',file=sys.stderr,flush=True)
    raise SystemExit(0)
signal.signal(signal.SIGTERM,finish)
time.sleep(0.2)
print('tcpdump: listening on wlp114s0, mock only',file=sys.stderr,flush=True)
while True:
    time.sleep(0.1)
