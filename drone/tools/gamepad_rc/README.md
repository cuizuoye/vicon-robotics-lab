# Migrated diagnostic and safety-policy tools

Read `../../README.md` and the root handoff first. These files preserve the
read-only whitelists, message parsing, complete-reply rules, incident holds,
physical holds, one-use exception policies and rigid-body revision guards.

The old full `gamepad_rc.py` remains blocked by the retained incident. Do not
re-enable joystick flight just because the code is present. Current intent is
autonomous PC control with gamepad HOME as a latched stop path, not a traditional
independent RC transmitter. Temporary test listeners are not flight-ready services.

All hardware UIDs in this public copy are synthetic. Private parameter backups,
real extrinsics, calibration receipts, consumed permits and runtime latches were
not copied. Missing old evidence is a reason to stop, not to bypass a gate.

Run only the offline `test/` suite through the root `scripts/test_offline.sh` by
default. Individual scripts can communicate with real devices if deliberately
invoked; they are NOT automatically safe just because their name says diagnostic.
