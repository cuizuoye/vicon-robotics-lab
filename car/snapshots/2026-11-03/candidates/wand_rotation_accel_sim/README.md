# Independent acceleration A2 simulation

Real Sign to SOFTWARE car only. This adds a new candidate without editing any
accepted source, old launcher, rectangle program, firmware, receiver or network.
Speed remains XY .70m/s and yaw 2rad/s, both mappings 1:1. Use the latest signed
region forward [-1.5,.5], left [-.4,.5]m with original footprint/buffer guards.
Physical car stays OFF. No command publisher/service/UDP/serial output exists.

Acceleration comparison enumerates XY {.8,1.2,2.0}m/s2 and yaw {6,8,12}rad/s2.
Identical trajectories, 120s budget, safety guards, wheel limit 400RPM, first-order
plant .12s and gains are retained. No time-shifted alignment or increased speeds,
lowered plant lag, or new predictive filter to hide actual tracking errors.
Fast cases may remain speed/plant-lag limited; a functional pass is not a tracking
acceptance. Selection requires >=10% XY RMS improvement, no >2% mean yaw RMS
degradation and the normal yaw threshold, then picks the smallest pair within
1% of the best normalized error score. Also compare plant lag .06/.20s.

The new Engine overrides only the configuration creation hook; source binding,
independent supervisor/gate, original raw source timestamps and simulation plant
are the pinned previous implementation. The UI inherits all key/focus/stop paths
and changes the parameter label/title with an exact pinned-layout check.

Explicit prepare is still necessary. Fresh complete Space tap starts 120s;
second Space press/Esc/focus loss/loss of Sign stops. Releasing Space does NOT
stop. No automatic recovery; human reprepare, stable stop and fresh click needed.
Occlusion/manual recovery acceptance for XY+yaw remains a separate real-user test.

Run after preparation:

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_sign_rotation_accel_sim.sh
```

`--check` only verifies hashes, no ROS initialization. New and old SIM entries
share the same advisory lock to avoid two input consoles. Do not treat any of
these software angular/acceleration parameters as hardware-qualified settings.
