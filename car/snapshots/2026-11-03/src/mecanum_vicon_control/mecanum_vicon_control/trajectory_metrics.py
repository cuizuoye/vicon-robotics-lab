"""Geometric path errors, not a time-scheduled trajectory tracking claim."""
import math


def point_segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    if not all(math.isfinite(v) for v in (px, py, ax, ay, bx, by)):
        raise ValueError('coordinates must be finite')
    dx, dy = bx-ax, by-ay
    squared = dx*dx + dy*dy
    fraction = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/squared)) if squared else 0.0
    return math.hypot(px-ax-fraction*dx, py-ay-fraction*dy)


def percentile(values, fraction):
    if not values or not 0 <= fraction <= 1 or not all(math.isfinite(v) for v in values):
        raise ValueError('percentile requires finite values and fraction 0..1')
    ordered = sorted(values)
    index = fraction*(len(ordered)-1)
    low = int(index)
    high = min(low+1, len(ordered)-1)
    return ordered[low] + (ordered[high]-ordered[low])*(index-low)


def summarize_errors(errors):
    if not errors or not all(math.isfinite(e) and e >= 0 for e in errors):
        raise ValueError('errors must be finite, nonnegative and nonempty')
    return {'mean_m': sum(errors)/len(errors),
            'rmse_m': math.sqrt(sum(e*e for e in errors)/len(errors)),
            'p95_m': percentile(errors, .95), 'max_m': max(errors)}


def time_weighted_errors(errors, intervals):
    """Trapezoidal integration prevents irregular sample density biasing RMSE."""
    duration = sum(intervals)
    return {
        'mean_time_weighted_m': sum((a+b)*.5*dt
                                   for a,b,dt in zip(errors, errors[1:], intervals))/duration,
        'rmse_time_weighted_m': math.sqrt(sum((a*a+b*b)*.5*dt
                                            for a,b,dt in zip(errors, errors[1:], intervals))/duration),
    }


def tracking_metrics(poses, reference, active_segment_errors=None):
    """poses are [seconds,x,y]; reference is the ordered nominal polyline.

    Nearest-polyline distance can conceal phase errors at figure-eight crossings.
    Active-segment error and waypoint-order validation must accompany it.
    RMSE is sample-weighted; sample timing is reported, not assumed uniform.
    """
    if len(poses) < 2 or len(reference) < 2:
        raise ValueError('at least two poses and reference points are required')
    if any(len(row) != 3 or not all(math.isfinite(v) for v in row) for row in poses):
        raise ValueError('poses must be finite [time,x,y] triples')
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in reference):
        raise ValueError('reference must contain finite 2D points')
    intervals = [b[0]-a[0] for a,b in zip(poses, poses[1:])]
    if min(intervals) < 0 or poses[-1][0] <= poses[0][0]:
        raise ValueError('pose times must progress monotonically')
    errors = [min(point_segment_distance(row[1:], a, b)
                  for a,b in zip(reference, reference[1:])) for row in poses]
    result = {
        'definition': 'geometric deviation from waypoint polyline, not time-aligned tracking',
        'sample_count': len(poses), 'duration_s': poses[-1][0]-poses[0][0],
        'sample_interval_max_s': max(intervals),
        'effective_sample_hz': (len(poses)-1)/(poses[-1][0]-poses[0][0]),
        'reference_length_m': sum(math.dist(a,b) for a,b in zip(reference, reference[1:])),
        'travelled_length_m': sum(math.dist(a[1:],b[1:]) for a,b in zip(poses, poses[1:])),
        'nearest_polyline': summarize_errors(errors),
        'final_goal_error_m': math.dist(poses[-1][1:], reference[-1]),
    }
    result['nearest_polyline'].update(time_weighted_errors(errors, intervals))
    if active_segment_errors is not None:
        if len(active_segment_errors) != len(poses):
            raise ValueError('active-segment errors must match pose samples')
        result['active_segment'] = summarize_errors(active_segment_errors)
        result['active_segment'].update(time_weighted_errors(active_segment_errors, intervals))
    return result, errors
