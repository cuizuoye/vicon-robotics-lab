# Real Sign -> SOFTWARE XY + rotation car

Independent candidate. No flash, physical motion, command publishers, UDP,
services, receiver/network changes, or edits to rectangle/accepted XY programs.

Run after offline preparation:

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_sign_rotation_sim.sh
```

`--check` validates hashes only, without ROS initialization. Keep physical car
OFF. Reuse the already-running, source-reviewed domain-0 Vicon receiver; absent
or conflicting receiver fails closed, without launching/stopping anything.

Human-confirmed geometry: crossbar Sign1--Sign5--Sign2; forward outside crossbar,
away from stem Sign3/Sign4. Control point Sign5 removes the raw rigid-origin's
75mm offset artifact; a real Sign5 arc from wrist rotation is real translation.
Relative horizontal yaw and fixed startup-car-frame translation are both 1:1.
Height ignored; near-vertical projected forward is invalid.

Software car starts at the current arena's center (-.5m forward, .05m left),
NOT the physical car pose. Latest arena is forward [-1.5,.5], left [-.4,.5]m,
with existing footprint/buffer guards, NOT historic 3x3m. XY .70m/s, .80m/s2;
software yaw 2rad/s, 6rad/s2. These are NOT a hardware-approved rotation profile.

1280x720 console: prepare, wait READY/ARMED, tap and release Space. It starts
120s logical session; release does NOT stop. Second Space press, Esc, focus
loss, geometry/source failure, stale data >200ms, control gap >100ms, reference
or tracking errors stop. No auto resume. After stop and valid stationary input,
explicitly click confirm/reprepare; this creates new sim cores, preserving the
plant pose/velocity, waits full readiness windows, then needs another fresh tap.
Origins are captured only on engagement, not prepare. No hidden car resets.

Real ROS boundary has only 7 passive subscriptions via the unchanged reviewed
LiveInput. Selected raw Sign+5-marker packet is immutable and same-source-frame;
original stamps/receipt times retained, no interpolation or restamping. Both
local safety cores independently derive pose from it. Command envelope carries
digest+source stamp, checked again at local final gate. This is local process
integrity, NOT DDS cryptographic authentication or a proven multi-process wire
protocol. Software plant is a first-order illustrative model, NOT measured
hardware dynamics, braking, current, temperature, or physical rotation proof.

Evidence: per-session contract, receiver provenance, 20Hz source-packet and
state/command logs, human input/session events, and final result. Real-user
tracking, stop, occlusion and explicit recovery acceptance remain separate.
