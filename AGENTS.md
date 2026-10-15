# Robotics handoff rules

Read `README.md`, `handoff/CURRENT_STATE.json`, and the relevant project README
before making changes. The dated handoff takes precedence over historical code
comments and example defaults. This is a curated public software snapshot, not
a live hardware permission or a full copy of the primary PC.

## Work and version boundaries

- Do development in `dev/<machine>/<topic>`, never directly on `main` or a baseline tag.
- Keep car, real MicoAir PX4 1.14.3, and PX4 1.17 SITL results separate.
- Do not replace the primary PC workspace with this checkout, `git pull` into it,
  or rewrite its history. Review and test individual commits before integrating.
- Do not force-push, reset hard, remove local changes, or change network/proxy/VPN
  configuration without explicit user permission.
- Use `apply_patch` for source changes. Preserve unrelated edits. Do not delegate
  to other agents unless the user explicitly asks.

## Offline first

- Start with `bash scripts/doctor.sh` and `bash scripts/test_offline.sh`.
- These commands must not open serial/USB devices, flash/reset boards, publish
  into the live lab ROS domain, modify parameters, or enable motor output.
- A passing test, successful build or a received ACK is not physical execution proof.
- Do not weaken the car's 150ms MCU lease or 750ms ESP32 timeout, remove fault
  latches, automatically re-enable after reconnection, or relax drone safety gates.
- Drone props are treated as INSTALLED. No automatic arming, flight, actuator test,
  lockdown clearing, fusion injection or restoration of old permits is allowed.
- All migrated device IDs are synthetic; provision private identities locally
  only after a fresh physical review. Never change IDs just to bypass a guard.
- Expired one-use permits, runtime latches, complete FC parameter dumps, Wi-Fi
  passwords, full Flash/NVS backups, and proprietary Vicon SDK binaries are NOT
  in this repository. Do not infer their absence means a safety hold is cleared.

## Before ending a task

Report files changed, exact offline commands/results, remaining hardware checks,
and whether any hardware command was sent. Update the handoff for material progress.
Run `python3 scripts/public_scan.py` before committing. Never commit `.local/`,
credentials or raw device dumps. Software rollback does not roll back MCU Flash,
ESP NVS, PX4 parameters, marker geometry, wiring, or firmware already installed.
