# V3 single-engagement 20-second observation

This is a separate host-only release of the accepted V3 active-origin pipeline.
It changes the maximum held-enable observation from 5 to 20 seconds, including
both independent timing guards, console metadata and labels, and scoped runtime
consent. Preparation does not start the timer. The first healthy held-enable
frame captures the shared mapping origin and starts the timer.

Unchanged: stage 4 (0.70 m/s, 0.80 m/s²), 1:1 XY mapping, fixed 3 × 3 m arena
and 0.50 m buffer, hand/goal 0.15 m, car displacement 0.20 m, total car path
0.25 m, heading/lateral/backward bounds, watchdogs, release/occlusion/fault
stops, one engagement per process, and no automatic resume. Firmware and all
earlier releases, including the rectangle program, remain unchanged.

The regression suite covers the independent supervisor and final gate at
5.1 s, 19.999 s and 20 s; preparation longer than 20 s; neutral active-origin
capture; one-shot lockout; console duration metadata; and early safety stops.
Preparation does not initialize ROS or open hardware. The generated launcher
defaults to file verification only; live launch still requires the full saved
Flash readback record, site confirmation, and physical user enable.

This observation is not validation of continuous high-speed motion, loaded
speed/acceleration, braking margin, current capacity or thermal behavior.
