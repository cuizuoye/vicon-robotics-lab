# Independent fixed 3 × 3 m / 20-second demonstration host

Human request: extend the 15 cm Sign limit to a usable demonstration range.
This release replaces **all** short-pilot spatial budgets together: 15 cm hand
and goal radius, 20 cm car displacement, 25 cm total path, 10 cm lateral and
4 cm reverse allowance. They are not silently retained as secondary stops.

The existing, fixed arena is unchanged: 0–3 m forward and 0–3 m left from the
saved original rectangle corner/heading. Targets are clamped there. A fresh
tracked Sign can move beyond a target edge and be drawn back without a
15 cm lockout; the reference display extends 0.50 m beyond each side. This
is not a new measured Vicon coverage or person-detection region. The whole
190 × 210 mm body remains subject to its independent oriented hard fence,
0.50 m physical buffer and 0.20 m preserved extra margin.

Still fixed: stage 4 (0.70 m/s, 0.80 m/s²), XY 1:1, no Z/orientation mapping,
neutral first-active origin capture, one engagement per process, 20 seconds
from held-enable (not from Prepare), 0.20 rad heading deviation guard, all
base freshness, source provenance, output ownership/signatures, clock, lease,
feedback, speed/acceleration, lifting/tilt and native fault protections.
Release, occlusion, fault or timeout ends the trial; no automatic resume.

Firmware, original rectangle program, network settings, older releases and
their historical evidence are preserved. This preparation never opens ROS,
Vicon SDK, a socket, serial device or programmer. A real launcher defaults to
file verification and requires a fresh physical-site confirmation, full saved
Flash readback and the current human range-extension request before launch.
It never synthesizes the operator's enable key.

The regression suite retains the old non-obsolete lifecycle/source/codec/UI
tests, explicitly replaces old short-radius assertions (TEST_SELECTION.json),
and adds fixed-fence independence, four directions, multi-metre cumulative
path, full goal-corner reach, boundary-clamp/return and the actual Sign trace
which caused PILOT_HAND_LIMIT after 3.18 seconds. Simulation is not physical
acceptance. The previous ~11 cm movement between stop trigger and standstill
is a single observation, **not** a validated worst-case 0.70 m/s stopping
distance. Current/temperature/braking qualification remains incomplete; this
release does not manufacture evidence or qualify continuous high-speed use.
