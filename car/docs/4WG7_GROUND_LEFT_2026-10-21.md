# 4WG7 ground-left acceptance (2026-10-21)

The operator clarified during the run that the vehicle was on the ground and
authorized ground testing.  This record therefore supersedes the initial
"suspended" assumption for this run.

## Preconditions

- Firmware identity: `4WG7`
- Initial MCU fault: `0`
- Initial permit: disabled
- Initial software and hardware PWM compares: all zero
- Vicon validity: true
- Tenda path: `192.168.50.2` to `192.168.50.1`

## Result

- A body-left goal 0.15 m from the measured start pose was commanded.
- Exact derived start pose: `(-0.507921782, 0.567263292)`, yaw
  `1.650183197 rad`.
- Final observed pose: `(-0.642006001, 0.560809755)`, yaw
  `1.640502 rad`.
- Body-frame displacement: forward `+0.00420 m`, left `+0.13417 m`.
- Yaw change: `-0.555 deg`.
- Encoder deltas: `[-746, -796, +794, +768]`.
- Maximum absolute wheel RPM: `[22.6, 22.8, 22.7, 28.2]`.
- No MCU fault occurred.  Final permit was disabled; target, duty, filtered
  speed and all eight PWM compares were zero.

The earlier symptom "left motion stops after a short distance" did not recur.
This is a successful ground-left result, but it is not a completed rectangle:
the commissioning left-goal node intentionally overrode the trajectory goals,
which caused the trajectory generator's completion state to advance after the
left target was reached.  A formal rectangle run must omit that override node.
