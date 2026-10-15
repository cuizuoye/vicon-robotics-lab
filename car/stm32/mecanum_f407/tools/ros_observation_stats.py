"""Pure readonly observation helpers; these never send transport commands."""

def valid_duration(seconds):
    # Observation time, NOT a motion duration or command/watchdog lease.
    return 5 <= seconds <= 120

def feedback_gaps_ms(topics):
    result={}
    for key in ('wheel_rpm','diagnostics','sequence'):
        times=[row['time_s'] for row in topics.get(key,[])]
        gaps=[(b-a)*1000 for a,b in zip(times,times[1:])]
        if any(gap<0 for gap in gaps):raise ValueError('Non-monotonic host observation times')
        result[key]=max(gaps) if gaps else None
    return result
