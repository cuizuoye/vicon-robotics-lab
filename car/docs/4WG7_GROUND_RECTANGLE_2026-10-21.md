# 4WG7 Vicon ground rectangle acceptance (2026-10-21)

## Scope

- Real vehicle on the ground; operator authorized the run and kept the area clear.
- Relative rectangle: 0.30 m forward by 0.20 m left, fixed vehicle heading.
- Nine goals: eight edge waypoints plus the repeated start point.
- Maximum commanded translation: 0.03 m/s.
- Firmware identity: `4WG7`.
- Protection layers: ROS pose/command watchdog 0.20 s, MCU command lease
  0.15 s, run guard 55 s, independent cutoff 60 s, firmware hard limit 120 s.
- Recorded bag: `bags/4wg7_ground_rectangle_20261021`.

## Result

- One rectangle completed normally in 31.957 s.
- All nine goals were published and reached in order.
- Planned body-frame points were exactly:
  `(0,0)`, `(0.15,0)`, `(0.30,0)`, `(0.30,0.10)`,
  `(0.30,0.20)`, `(0.15,0.20)`, `(0,0.20)`, `(0,0.10)`, `(0,0)` m.
- Maximum waypoint acceptance error: 19.88 mm.
- Final loop-closure error: 19.88 mm.  This matches the current 20 mm goal
  tolerance rather than an uncontrolled stop.
- Observed body-frame path extent:
  - forward: -2.0 mm to 302.4 mm
  - left: -4.0 mm to 202.7 mm
- Maximum absolute heading error: 1.046 deg.
- Final heading error: -0.310 deg.
- Maximum recorded pose interval during motion: 11.84 ms.

## Link and actuator evidence

- MCU fault values during the complete motion: only `0`.
- MC-Link disconnected samples during motion: `0`.
- ROS safety-disabled samples before trajectory completion: `0`.
- Maximum absolute wheel RPM: `[28.3, 34.3, 28.4, 28.2]`.
- Maximum absolute per-wheel duty: `[10%, 12%, 11%, 11%]`, below the 35%
  firmware ceiling.
- Final live diagnostic after the run: permit disabled, fault `0`, stationary
  500 ms, target/filtered speed/raw duty all zero, and all eight PWM compare
  registers zero.

## Conclusion

The 4WG7 controller passed the first real Vicon-guided ground rectangle.  The
earlier short left-motion stop did not recur.  This accepts the low-speed
0.30 x 0.20 m rectangle at the current 0.03 m/s cap; it does not yet validate
higher speeds, heavier payload changes, or a larger course.
