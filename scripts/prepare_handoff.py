#!/usr/bin/env python3
"""Mechanical public-export normalization and provenance; never talks to hardware.

Run once on an uncommitted migration branch after the selected source trees were
copied. The original source roots are READ ONLY. No credentials are read here.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import urlsplit, urlunsplit

ROOT=Path(__file__).resolve().parents[1]

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def git(path,*args):
    return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()

def upstream(path):
    remotes=git(path,'remote').splitlines()
    url=git(path,'remote','get-url','origin') if 'origin' in remotes else None
    if url and (url.startswith('https://') or url.startswith('http://')):
        u=urlsplit(url);url=urlunsplit((u.scheme,u.hostname or '',u.path,'',''))
    return dict(url=url,commit=git(path,'rev-parse','HEAD'))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-car',type=Path,required=True)
    p.add_argument('--source-drone',type=Path,required=True)
    p.add_argument('--source-vicon',type=Path,required=True)
    p.add_argument('--source-firmware',type=Path,required=True)
    a=p.parse_args()
    assert not (ROOT/'handoff/EXPORT_MANIFEST.json').exists(), 'one-time export, do not overwrite baseline'
    dependencies=dict(vicon_receiver=upstream(a.source_vicon),px4_real=upstream(a.source_firmware),
        px4_sitl=upstream(a.source_drone/'px4_firmware'),
        xrce_agent=upstream(a.source_drone/'Micro-XRCE-DDS-Agent'),
        esp_idf=dict(version='6.1',commit='fff9895c82d744c7237be8847347bdd1b07c6643'),
        stm32_gcc='14.3.1+st.2',host=dict(os='Ubuntu 24.04',arch='x86_64',python='3.12',ros='jazzy'))
    if dependencies['px4_real']['url'] is None:
        assessment=json.loads((a.source_firmware.parent/'assessment.json').read_text())
        dependencies['px4_real']['url']=assessment['source_repository']
        dependencies['px4_real']['url_provenance']='local assessment; source clone has no origin remote'
    replacements={}
    for source in (a.source_drone/'tools/gamepad_rc').glob('*.py'):
        tree=ast.parse(source.read_text())
        for n in tree.body:
            if isinstance(n,ast.Assign) and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str):
                if any(isinstance(t,ast.Name) and t.id in ('UID','UID2') for t in n.targets):
                    value=n.value.value
                    if re.fullmatch('[0-9a-fA-F]{32,40}',value):replacements[value]='0'*len(value)
    # Device IDs are not passwords, but do not need to be public. Exact
    # replacements preserve fail-closed identity comparisons and test fixtures.
    for source in (ROOT/'car').rglob('*'):
        if not source.is_file() or 'ThirdParty' in source.parts:continue
        try:text=source.read_text()
        except UnicodeError:continue
        for match in re.findall(r'sn=([A-F0-9]{20,32})',text):replacements[match]='0'*len(match)
        for match in re.findall(r'wlx[0-9a-f]{12}',text):replacements[match]='wlx000000000000'
        for match in re.findall(r'usb-1a86_USB_Single_Serial_([A-F0-9]{8,16})',text):
            replacements[match]='0'*len(match)
    normalized=[]
    for prefix in ('car','drone','shared'):
        for source in (ROOT/prefix).rglob('*'):
            if not source.is_file() or source.is_symlink() or 'ThirdParty' in source.parts:continue
            try:before=source.read_text()
            except UnicodeError:continue
            after=before
            for old,new in replacements.items():after=after.replace(old,new)
            # Local machine locations become explicit placeholders, never an
            # accidental fallback to the primary working tree.
            after=after.replace('/home/czy','/home/PRIMARY_USER')
            after=re.sub(r"UUID='[0-9a-f-]{36}'", "UUID='00000000-0000-4000-8000-000000000000'",after)
            relative=source.relative_to(ROOT)
            if prefix=='car':original=a.source_car/relative.relative_to('car')
            elif prefix=='drone':original=a.source_drone/relative.relative_to('drone')
            elif relative.is_relative_to('shared/vicon_receiver_ws/src/ros2-vicon-receiver'):
                original=a.source_vicon/relative.relative_to('shared/vicon_receiver_ws/src/ros2-vicon-receiver')
            else:
                # Wrapper documentation was created for this export and has no
                # counterpart in the receiver's upstream source tree.
                original=source
            original_bytes=original.read_bytes() if original.is_file() else before.encode()
            if after.encode()!=original_bytes and source.name!='README.md':
                normalized.append(dict(path=str(source.relative_to(ROOT)),
                    original_sha256=hashlib.sha256(original_bytes).hexdigest(),
                    exported_sha256=hashlib.sha256(after.encode()).hexdigest()))
            if after!=before:
                source.write_text(after)
    patch=ROOT/'drone/firmware/micoair-v1.14.3/changes.patch'
    patch_bytes=subprocess.check_output(['git','-C',str(a.source_firmware),'diff','--no-ext-diff','--binary','HEAD'])
    if patch.exists():assert patch.read_bytes()==patch_bytes, 'existing patch differs; do not overwrite'
    else:
        with patch.open('xb') as stream:stream.write(patch_bytes)
    (ROOT/'drone/firmware/micoair-v1.14.3/upstream.json').write_text(json.dumps({
        **dependencies['px4_real'],'patch_sha256':digest(patch),
        'build_target':'micoair_h743-v2_default','board_id':1179,
        'verified_compiler':'10.2.1 20201103',
        'installed_package_sha256':'4fbaa8f64a66d1e5f138355dfa78f150a8fc4bd97780e19f499018afc60a143b',
        'factory_binary_included':False,'flight_authorized':False},indent=2)+'\n')
    parameters=a.source_drone/'mircoair_rebuilt_20261015_U2klYU/parameters_after_revision_static/parameters_verified.json'
    full=json.loads(parameters.read_text())
    wanted={'COM_RC_IN_MODE','COM_OBL_RC_ACT','COM_OF_LOSS_T','EKF2_HGT_REF','EKF2_EV_CTRL',
            'EKF2_EV_DELAY','EKF2_AID_MASK','COM_ARM_HFLT_CHK'}
    rows=full['records']
    selected={r.get('name',r.get('param_id')):r.get('value') for r in rows
              if r.get('name',r.get('param_id')) in wanted}
    manifest=dict(date='2026-10-15',original_workspaces_modified=False,
        scope='Curated PUBLIC source snapshot, not a disk image or hardware authority',
        normalized_files=normalized,source_roots_omitted_from_public_manifest=True,
        excluded=['passwords/sdkconfig/private.defaults','Flash/NVS/FC binary dumps','one-use permits and runtime latches',
                  'full parameter backups and real calibration receipts','Vicon proprietary SDK/binaries',
                  'build/install/log/tmp/venv/third-party binary caches','Codex sessions and credentials'],
        dependencies=dependencies,
        historical_key_parameters_not_live_observation=selected,
        board_uid_replaced_by_synthetic_zero=True)
    with (ROOT/'handoff/EXPORT_MANIFEST.json').open('x') as stream:json.dump(manifest,stream,indent=2)
    print(json.dumps(dict(normalized_file_count=len(normalized),patch_created=True,
                         key_parameter_names=sorted(selected),original_roots_written=False)))

if __name__=='__main__':main()
