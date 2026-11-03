# 4WG8 0.20 m/s large-course candidate (offline only)

Date: 2026-10-21.  This record covers source review, offline implementation,
native safety tests, and cross-compilation only.  The STM32 was not connected,
read, erased, reset, or programmed, and no motor command was sent.

## Requested course

- Relative to the measured starting pose and fixed vehicle heading.
- 5.0 m forward, 4.0 m left, 5.0 m backward, 4.0 m right.
- Total path length: 18.0 m.
- Maximum commanded translation: 0.20 m/s.
- Eight edge waypoints plus the repeated start point.  The anchored even-index
  vertices are exactly `(0,0)`, `(5,0)`, `(5,4)`, `(0,4)`, `(0,0)` m.

The ideal constant-speed time is 90 s.  Acceleration, waypoint acceptance and
the 500 ms measured-standstill direction changes add time, so 4WG8 uses a 150 s
absolute firmware budget.  The 150 ms command lease remains unchanged.

## Hardware evidence and motor rating

- Battery information supplied by the operator: 3S lithium, approximately
  12 V (12.6 V when fully charged).
- D157B/AT8236 module manual: 5.5-36 V input is stated in the module overview;
  its wiring table also gives a narrower 5-15 V range.  A 3S pack is inside
  both stated ranges.  The module advertises 4 A continuous and 6 A peak.
- AT8236 chip data: recommended 5.5-36 V, 3 A continuous and 4.5 A peak;
  absolute peak is 6 A, with UVLO/OCP/thermal shutdown.  The conservative
  electrical reference for commissioning is therefore 3 A continuous and
  4.5 A peak per channel, not the absolute 6 A number.
- The operator corrected the installed motor identification to an MG310 Hall
  encoder gearmotor: 20:1 reduction, 7-13 V rated range, 500 rpm no-load output
  speed, rated current at most 0.5 A, and rated output torque 0.4 kgf.cm.  The
  downloaded MG513X mechanical drawing does not describe the installed motor.

The measured per-wheel CPR values remain the usable closed-loop calibration:
`1059.67`, `1049`, `1055.67`, `1061` counts/output revolution.  With a 60 mm
wheel, 0.20 m/s is 63.66 output rpm and approximately `[1124,1113,1120,1126]`
cps.  Dividing the measured CPR by 20 gives about 52.5 counts/motor revolution,
which agrees closely with a 13-line Hall encoder decoded on all four edges.

Treating the supplied 500 rpm as the gearbox output-shaft no-load speed, a 60 mm
wheel has a theoretical no-load rim speed of 1.57 m/s.  The 0.20 m/s request is
63.66 rpm, only 12.7% of that no-load speed.  Rated torque 0.4 kgf.cm is
0.0392 N.m, or 1.31 N rim force per 30 mm-radius wheel before mecanum and other
losses.  Four rated-current motors draw at most 2 A total nominally, while each
AT8236 channel's conservative continuous rating is 3 A.  Stall current is still
unknown and is not inferred from rated current.

The existing 4WG7 feedforward seed predicts about 24.7-24.9% duty at the target,
leaving PI headroom under the unchanged 35% hard ceiling.  The installed motor
ratings therefore support keeping 0.20 m/s as the staged-test target, while the
actual loaded speed and stall margin still require measurement.

## Separate firmware profile

4WG7 remains the accepted flashed low-speed image.  4WG8 is a separate target:

- identity `4WG8` / `0x38475734`;
- body x/y and wheel-rim limit: 0.20 m/s;
- target limit: 1200 cps;
- yaw limit unchanged: 0.20 rad/s;
- PWM ceiling unchanged: 35%;
- breakaway ceiling unchanged: 30% for at most 500 ms, slew bounded and
  released on encoder progress;
- command lease unchanged: 150 ms;
- absolute motion budget: 150 s;
- same all-wheel zero-output stop, reversal standstill, one-run allowance,
  stall, feedback, timing and fault latches as 4WG7.

The ROS bridge allows 0.20 m/s only when it is configured to require the exact
`4WG8` identity.  Selecting `4WG7`, an empty tag, or any older identity retains
the 0.05 m/s host ceiling.  The new
`vicon_rectangle_5x4_hardware.launch.py` is inert at startup and does not open
either motion gate.

## Offline evidence

- Nine ASan/UBSan native motion profiles pass, including explicit 4WG8
  acceptance at 200 mm/s, rejection at 201 mm/s, 1200 cps telemetry bounds,
  150 s wrap-safe run cutoff, lease loss, reversal, stall and zero-output tests.
- ROS/Python suite: 223 passed, including the pinned 5 m x 4 m launch geometry,
  0.20 m/s cap and exact 4WG8 identity.
- `mecanum_vicon_control` colcon build completed.  The workspace scanner emits
  an unrelated historical `tmp/.../ruamel/yaml/clibz` missing-README warning;
  the requested package still builds successfully.
- ARM target `mecanum_f407_four_link_ground8` builds successfully.
- Binary size: 41,900 bytes.
- Binary SHA-256:
  `32b65461b26467dd2cfb78fcc257b6fe1eee35ac1bf1b3723de8380d0e894a54`.

## Hardware gate before a 5 m x 4 m run

Do not flash or run the full course directly from this offline result.  Preserve
a full readback of the accepted 4WG7 image, then program 4WG8 while motor power
is disconnected.  The first powered validation must establish actual current,
wheel speed and Vicon body speed in bounded stages.  Because the supplied motor
stall current remains unknown, so any observed AT8236 protection cycling,
wheel-speed saturation, excessive voltage sag, heating, or failure to track the
setpoint is a stop condition rather than permission to raise the 35% ceiling.
