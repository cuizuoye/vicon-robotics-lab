# 4WG7 feedforward + PI ground controller (offline only)

Date: 2026-10-21

Status: built and regression-tested offline. It has not been flashed or run on hardware.

## Why 4WG7 exists

4WG5 applied a fixed 30% breakaway output to every nonzero wheel target. During the
failed Vicon run, the mecanum cancellation wheels were commanded at only about 6 cps,
yet they received the same breakaway output as the approximately 270 cps driven wheels.
The first 30% pulse also produced filtered wheel speeds several times larger than the
requested speeds. 4WG6 zeroed targets below 20 cps but retained the same startup model,
so it remains an unflashed intermediate draft.

4WG7 keeps the standard X-layout mecanum kinematics and common wheel desaturation, but
replaces the ground motor policy with the control structure used by mature open-source
implementations:

- ROS 2 `mecanum_drive_controller`: body velocity limits, timeout, and standard IK;
- WPILib `MecanumBot`: per-wheel feedforward plus PID and common desaturation;
- Rover Robotics: static-friction feedforward, low-speed feedback trust handling,
  launch gating, and reset-on-stop behavior;
- Linorobot2: four independent wheel loops and explicit direction/encoder calibration.

## Controller behavior

For each wheel in normal tracking:

```text
u = clamp(sign(target) * (kS * low_speed_scale + kV * abs(target))
          + scaled_PI(target - measured), same_direction_limits)
```

The ground-v7 candidate has these bounded seed values:

| Parameter | Offline seed | Meaning |
| --- | ---: | --- |
| PWM hard limit | 350 permille | unchanged 35% ground ceiling |
| `kS` | 80 permille per wheel | conservative hold-friction seed |
| `kV` | 0.15 permille/cps per wheel | speed feedforward slope |
| low-speed trust floor | 100 cps | below this, feedback is reduced |
| cancellation deadband | 20 cps | below one encoder count per 50 ms |
| breakaway ceiling | 300 permille | a ceiling, not an immediate fixed pulse |
| breakaway slew | 1200 permille/s | 10 ms output steps are quantized to at most 20 permille |
| launch deadline | 500 ms | no encoder progress faults all outputs off |
| tracking stall duty floor | 200 permille | low control effort cannot cause a stall fault |

The four wheel structures have independent `kS`, `kV`, PI, CPR, sign, and trust values.
The initial `kS` and `kV` values are intentionally identical because the existing 4WG5
bag is a short overspeed transient and does not contain enough steady-state data to
justify four different fitted constants.

Important state behavior:

1. A wheel below 100 cps never enters breakaway and never accumulates PI integral.
2. A wheel at or above 100 cps ramps breakaway output instead of jumping to 30%.
3. The first correctly directed encoder count immediately exits breakaway.
4. Normal tracking uses `kS + kV * target + PI correction`.
5. A tracking stall requires both a significant target and at least 20% prior output.
6. Zero target clears output, integral, progress, and launch state.
7. Direction reversal still requires all outputs off and measured standstill.

## Preserved safety boundaries

- 150 ms velocity lease and command timeout;
- 120 s absolute one-shot motion budget;
- explicit disabled-zero primer and MCU permit;
- no online polarity reversal;
- 35% PWM hard ceiling;
- latched timing, feedback, command, duration, stall, and estop faults;
- host guard now requires the exact `4WG7` identity.

## Offline evidence

- Eight native streaming profiles pass under AddressSanitizer and UBSan:
  suspended, 4WG1, 4WG2, 4WG3, 4WG4, 4WG5, 4WG6, and 4WG7.
- Ground-v7 tests cover slew-bounded launch, launch timeout, first-count handoff,
  cancellation-wheel zeroing, low-target no-launch/no-integral/no-false-stall,
  reversals, lease expiry, estop, duration, timing, feedback, and output limits.
- Legacy rectangle/drive/transport native regression groups pass.
- 218 ROS/Python tests pass, including the variable launch-duty 4WG7 decoder.
- ARM cross-build succeeds with 23,296 bytes of flash image and 7,448 bytes RAM use.

## Hardware boundary

The seed feedforward model is derived conservatively from existing logs, not from a
complete per-wheel ground identification. Passing offline tests proves control logic and
safety invariants only. The first powered run must still confirm that the bounded launch
can overcome the real 1.4 kg vehicle's wheel friction and that it exits on the first
encoder progress. No claim of physical acceptance is made here.
