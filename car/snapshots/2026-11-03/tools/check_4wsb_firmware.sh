#!/usr/bin/env bash
# FILE CHECK ONLY. No ROS/network/programmer/serial/device access.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_firmware_4wsb_2026-10-29/ready/flash_bundle/verify_bundle.py --folder /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_firmware_4wsb_2026-10-29/ready/flash_bundle "$@"
