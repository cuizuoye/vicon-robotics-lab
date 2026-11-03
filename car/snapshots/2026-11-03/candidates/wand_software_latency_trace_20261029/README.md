# Independent same-sequence software latency profiler

This directory is not on the original demonstration entry's import path.
It reuses the pinned real-Sign/private-ROS/software-car chain; process-local
subclasses add passive sidecar timestamps only. There is no device bridge,
serial/socket actuator transport, firmware change or Vicon/Tenda reconfiguration.

## Entry

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_software_latency_trace.sh --check
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_software_latency_trace.sh --launch
```

Keep the REAL car battery disconnected. Real Vicon is read on domain 0;
control, gate, signed output and software plant stay on private domain 189.
Start remains a fresh human click/space after actual READY. Esc, focus loss,
input loss and existing timing checks still stop/revoke. Restore input then
explicitly reprepare; no automatic resume. Close the window to flush and analyze.

Preserved: 1:1 XY/relative yaw, Z ignored; XY 0.70 m/s, 0.80 m/s²;
yaw 0.10 rad/s, 0.40 rad/s²; session 120s; no fixed fence;
source 200ms, command/UI/signed output 100ms, lease 150ms, feedback 300ms.
No gain/filter/controller/model/PWM/hardware parameter changes.

## Timing identities

1. Original immutable six-topic packet SHA256 and original source ROS stamp.
2. Controller owner + sequence + arm epoch, actual publish start/end.
3. Gate callback arrival and exact-source core acceptance start/end.
4. HMAC signer output owner + sequence, linked to the exact current gate frame.
5. Signed DDS publish, actual plant callback, successful verifier start/end.
6. Model integration sequence, exact signed command actually selected by watchdog.
7. Snapshot sequence, current goal controller ID AND model command ID separately.
8. Tk draw sequence, exact consumed snapshot/model IDs and draw start/end.

All local intervals use `monotonic_ns`. Source ROS->monotonic conversion uses
a bracketed local clock pair; clock-pair span is reported, not silently ignored.
The receiver header is a PC post-receipt stamp, NOT camera exposure.
Tk draw end is commands submitted, NOT Xorg presentation/monitor photons.
Model integration is command application, NOT reaching a moving target.
The unchanged 0.12s first-order model remains a separate dynamic response.

Gate outputs (100Hz), controls (50Hz), snapshots (~40Hz) and draw (~25Hz)
do not pair one-to-one. The analyzer joins exact IDs, reports superseded outputs,
first command application and first visible state independently. It does NOT
match nearest times/values or add percentile components into an end-to-end P95.

## Logging and validity

The producer queues small sidecar records without disk writes, with bounded
capacity; a separate writer serializes JSON. Producer enqueue costs are measured.
Any overflow, writer error, negative interval, worker/shutdown error invalidates
the sample. The parent synchronous observer disk logs are disabled only in this
profiler, while observer callbacks and ROS graph/QoS checks remain unchanged.
Thus this measures the independently instrumented software simulation, not
the original physical-demo hardware path or its disk-logging overhead.

`LATENCY_RESULT.json` contains median/P95/P99/max/count, exact paired IDs,
coverage, drops and original-file hash comparison. `LATENCY_REPORT.md` is the
human-readable table. No launch secret is logged.

## Offline evidence

`regression.py --output NEW_DIR --safety` executes the unchanged 15-case actual
ROS safety suite WITH profiling hooks, plus a real Tk/ROS exact-ID trace test.
The dedicated synthetic test has a known 1ms artificial ingress timestamp age;
that fixture-only offset is never applied to genuine Sign data.
`test_trace.py` checks exact joins, duplicate identities, wrong IDs, negative
intervals, rejected frames, subsampling and asynchronous flush.
`prepare.py --evidence DIR` pins source and evidence hashes before live launch.
