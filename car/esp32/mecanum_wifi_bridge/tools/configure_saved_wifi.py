#!/usr/bin/env python3
"""Generate a private build sdkconfig from one authorized saved Wi-Fi profile.

Never change NetworkManager, print its PSK, or modify the original sdkconfig.
Only supports ordinary WPA-PSK networks; other types need manual configuration.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess


def saved_value(profile, field, secret=False):
    cmd = ['nmcli', '--escape', 'no']
    if secret:
        cmd.append('--show-secrets')
    cmd += ['-g', field, 'connection', 'show', profile]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    if result.returncode:
        raise RuntimeError('Cannot read the selected saved Wi-Fi profile')
    return result.stdout.rstrip('\n')


def configured_text(source, ssid, psk):
    # The current bridge uses strlcpy into 32/64-byte fields; reserve a NUL.
    # Reject unsupported full-width fields rather than silently truncating.
    if not 1 <= len(ssid.encode('utf-8')) <= 31:
        raise ValueError('Unsupported SSID length')
    if psk in ('<hidden>', '--') or any(ord(c) < 32 for c in psk):
        raise ValueError('Saved WPA PSK is unavailable or unsupported')
    if not 8 <= len(psk.encode('utf-8')) <= 63:
        raise ValueError('Saved WPA PSK is unavailable or unsupported')
    for name, value in [('MC_WIFI_SSID', ssid), ('MC_WIFI_PASSWORD', psk)]:
        replacement = 'CONFIG_' + name + '=' + json.dumps(value, ensure_ascii=False)
        pattern = r'^CONFIG_' + name + r'=.*$'
        source, count = re.subn(pattern, lambda _match: replacement, source, flags=re.M)
        if count != 1:
            raise ValueError('Source sdkconfig must contain exactly one Wi-Fi field')
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--expected-ssid', required=True)
    parser.add_argument('--source-sdkconfig', type=Path, required=True)
    parser.add_argument('--output-sdkconfig', type=Path, required=True)
    args = parser.parse_args()
    ssid = saved_value(args.profile, '802-11-wireless.ssid')
    if ssid != args.expected_ssid:
        raise RuntimeError('Selected profile SSID does not match the authorized network')
    if saved_value(args.profile, '802-11-wireless-security.key-mgmt') != 'wpa-psk':
        raise RuntimeError('Only a saved WPA-PSK network is supported')
    psk = saved_value(args.profile, '802-11-wireless-security.psk', secret=True)
    text = configured_text(args.source_sdkconfig.read_text(), ssid, psk)
    # This is a generated private build input, not an edit of project sources.
    fd = os.open(args.output_sdkconfig, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as output:
        output.write(text)
    print(json.dumps(dict(ssid=ssid, output=str(args.output_sdkconfig),
        credentials_printed=False, network_settings_changed=False,
        original_config_changed=False)))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError) as exc:
        # Do not include raw command output or generated configuration in errors.
        raise SystemExit(str(exc))
