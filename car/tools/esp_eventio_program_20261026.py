#!/usr/bin/env python3
"""Authorized, isolated ESP32 programming; never touch STM32/network/motor I/O.

Requires explicit battery/UART/external-5V isolation confirmation. Backup and
write are separate invocations. Only the candidate application region is
written, and a full post-read verifies all other flash bytes are unchanged.
All generated flash/config records are private and never printed as contents.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_eventio/ready_v2'
PYTHON='/home/PRIMARY_USER/.espressif/tools/python/v6.1/venv/bin/python'
PORT='/dev/serial/by-id/usb-1a86_USB_Single_Serial_00000000-if00'
MAC='02:00:00:00:00:00'
IMAGE_SHA='1185a34026b7d15ff24eb871a699d8f6049e7e492150f15f002bfed2e2304c4f'
FLASH_BYTES=16*1024*1024
APP_OFFSET=0x10000

def digest(data):return hashlib.sha256(data).hexdigest()
def save(folder,name,data):
    path=folder/name
    if path.exists():raise ValueError('refuse to overwrite existing record: '+name)
    path.write_text(json.dumps(data,indent=2)+'\n');path.chmod(0o600)

def network():
    def read(*args):return subprocess.check_output(args,text=True,timeout=15).strip()
    # Configuration rather than transient AP association: resetting the ESP32
    # necessarily disconnects its station, but no PC configuration is changed.
    fields='connection.id,connection.uuid,connection.interface-name,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.never-default,ipv4.ignore-auto-dns,802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.channel'
    return dict(default_route=read('ip','-4','route','show','default'),
        tenda_config=read('nmcli','-g',fields,'connection','show','79614b9e-7213-452a-be72-34f9cdf1e9aa'),
        resolver_sha256=digest(Path('/etc/resolv.conf').read_bytes()))

def run(folder,name,*args):
    path=folder/(name+'.log')
    if path.exists():raise ValueError('log already exists: '+name)
    command=[PYTHON,'-m','esptool','--chip','esp32s3','--port',PORT,
        '--baud','921600','--before','default-reset','--after','no-reset',*map(str,args)]
    print(name+': START',flush=True)
    started=time.monotonic()
    with path.open('x') as stream:
        process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=900)
    path.chmod(0o600)
    if process.returncode:raise ValueError(name+' failed; inspect private log; no automatic retry')
    print(name+': PASS ('+str(round(time.monotonic()-started,1))+'s)',flush=True)
    return path.read_text()

def candidate():
    record=json.loads((READY/'ESP_BUILD_RESULT.json').read_text())
    image=Path(record['image']);data=image.read_bytes()
    if not record['passed'] or digest(data)!=IMAGE_SHA or record['image_sha256']!=IMAGE_SHA:
        raise ValueError('candidate image hash/record mismatch')
    pins=json.loads((READY/'ESP_SOURCE_INPUTS.json').read_text())
    if any(digest(Path(p).read_bytes())!=v for p,v in pins.items()):raise ValueError('generated source drift')
    if digest((READY/'sdkconfig.eventio').read_bytes())!=record['sdkconfig_sha256']:
        raise ValueError('candidate config drift')
    return image,data

def identify(folder,prefix):
    identity=run(folder,prefix+'_flash_id','flash-id')
    if MAC not in identity.lower() or 'Detected flash size: 16MB' not in identity:
        raise ValueError('chip MAC/flash capacity differs; no flash allowed')
    if 'ESP32-S3' not in identity:raise ValueError('not the expected ESP32-S3')
    security=run(folder,prefix+'_security','get-security-info')
    # Exact human-readable security report is checked explicitly before flash.
    save(folder,prefix+'_identity.json',dict(mac=MAC,chip='esp32s3',flash_bytes=FLASH_BYTES,
        usb_path=PORT,security_log=str(folder/(prefix+'_security.log'))))
    return security

def partition_check(folder,data):
    path=Path('/home/PRIMARY_USER/.espressif/v6.1/esp-idf/components/partition_table/gen_esp32part.py')
    spec=importlib.util.spec_from_file_location('sdk_partition_table',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    current=module.PartitionTable.from_binary(data[0x8000:0x9000])
    generated=(READY/'esp_build/partition_table/partition-table.bin').read_bytes()
    # Compare exact stored table including its MD5/padding, not just app name.
    if data[0x8000:0x8000+len(generated)]!=generated:
        raise ValueError('current partition table differs; no bootloader/table write authorized')
    apps=[p for p in current if p.type==module.APP_TYPE]
    if len(apps)!=1 or apps[0].offset!=APP_OFFSET or apps[0].name!='factory':
        raise ValueError('unexpected app selection/OTA layout')
    _,image=candidate()
    if len(image)>apps[0].size:raise ValueError('app too large')
    parts=[dict(name=p.name,type=p.type,subtype=p.subtype,offset=p.offset,size=p.size) for p in current]
    save(folder,'PARTITION_CHECK.json',dict(passed=True,partitions=parts,
        app_offset=APP_OFFSET,app_size=len(image),partition_table_exact_match=True,
        bootloader_write_authorized=False,nvs_write_authorized=False))

def backup():
    candidate()
    parent=ROOT/'esp32/mecanum_wifi_bridge/backups';parent.mkdir(exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='pre_eventio_20261026_',dir=parent));folder.chmod(0o700)
    print('PRIVATE_BACKUP='+str(folder),flush=True)
    save(folder,'AUTHORIZATION_AND_BEFORE.json',dict(battery_disconnected=True,
        external_5v_disconnected=True,uart_disconnected=True,usb_only=True,
        motion_authorized=False,stm32_write_authorized=False,network_before=network(),
        candidate_sha256=IMAGE_SHA,wall_time=time.time()))
    identify(folder,'before')
    for name in ('flash_before_pass1_16MiB.bin','flash_before_pass2_16MiB.bin'):
        run(folder,name.removesuffix('.bin'),'read-flash','0',hex(FLASH_BYTES),folder/name)
        (folder/name).chmod(0o600)
    first=(folder/'flash_before_pass1_16MiB.bin').read_bytes()
    second=(folder/'flash_before_pass2_16MiB.bin').read_bytes()
    if len(first)!=FLASH_BYTES or first!=second:raise ValueError('full reads mismatch; no flash')
    partition_check(folder,first)
    save(folder,'BACKUP_VERIFIED.json',dict(passed=True,full_flash_bytes=FLASH_BYTES,
        identical_full_read_passes=2,full_flash_sha256=digest(first),
        candidate_sha256=IMAGE_SHA,mac=MAC,flash_written=False))
    print('BACKUP_VERIFIED='+str(folder),flush=True)

def flash(folder):
    folder=folder.resolve()
    if folder.parent!=ROOT/'esp32/mecanum_wifi_bridge/backups':raise ValueError('invalid backup directory')
    if (folder/'FLASH_STARTED.json').exists():raise ValueError('write already attempted; no automatic retry')
    image,data=candidate()
    backup_record=json.loads((folder/'BACKUP_VERIFIED.json').read_text())
    first=(folder/'flash_before_pass1_16MiB.bin').read_bytes()
    second=(folder/'flash_before_pass2_16MiB.bin').read_bytes()
    if not backup_record['passed'] or first!=second or digest(first)!=backup_record['full_flash_sha256']:
        raise ValueError('backup proof invalid')
    if not json.loads((folder/'PARTITION_CHECK.json').read_text())['passed']:
        raise ValueError('partition not approved')
    identify(folder,'prewrite')
    security=(folder/'prewrite_security.log').read_text()
    if 'Secure Boot: Disabled' not in security or 'Flash Encryption: Disabled' not in security:
        raise ValueError('security state not explicitly disabled; do not force')
    save(folder,'FLASH_STARTED.json',dict(app_offset=APP_OFFSET,image=str(image),
        image_sha256=IMAGE_SHA,image_bytes=len(data),wall_time=time.time(),
        erase_all=False,bootloader_written=False,partition_table_written=False,nvs_written=False))
    run(folder,'write_application_only','write-flash','--flash-mode','keep',
        '--flash-size','keep','--flash-freq','keep',hex(APP_OFFSET),image)
    # Remain in ROM/stub: do not boot the application until the full post-read.
    run(folder,'flash_after_16MiB','read-flash','0',hex(FLASH_BYTES),folder/'flash_after_16MiB.bin')
    after_path=folder/'flash_after_16MiB.bin';after_path.chmod(0o600);after=after_path.read_bytes()
    end=APP_OFFSET+len(data);erased_end=math.ceil(end/4096)*4096
    if len(after)!=FLASH_BYTES or after[APP_OFFSET:end]!=data:
        raise ValueError('post-read candidate mismatch; keep battery/UART disconnected')
    if first[:APP_OFFSET]!=after[:APP_OFFSET] or first[erased_end:]!=after[erased_end:]:
        raise ValueError('bytes outside app erase footprint changed; keep disconnected')
    if any(b!=255 for b in after[end:erased_end]):raise ValueError('unexpected app erase tail')
    before=json.loads((folder/'AUTHORIZATION_AND_BEFORE.json').read_text())
    net_after=network()
    result=dict(passed=True,full_backup_identical_passes=2,full_backup_sha256=digest(first),
        post_flash_full_sha256=digest(after),candidate_sha256=IMAGE_SHA,
        readback_app_sha256=digest(after[APP_OFFSET:end]),readback_byte_identical=True,
        app_offset=APP_OFFSET,app_bytes=len(data),erased_end=erased_end,
        outside_application_erase_footprint_unchanged=True,
        bootloader_unchanged=True,partition_table_unchanged=True,nvs_unchanged=True,
        network_after=net_after,pc_network_configuration_unchanged=net_after==before['network_before'],
        stm32_written=False,uart_physically_disconnected=True,motion_started=False,
        held_in_download_mode=True,wall_time=time.time())
    if not result['pc_network_configuration_unchanged']:raise ValueError('network snapshot differs; inspect without mutation')
    save(folder,'POST_FLASH_VERIFIED.json',result)
    print(json.dumps(result,indent=2),flush=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode',choices=('backup','flash'))
    ap.add_argument('--backup',type=Path)
    ap.add_argument('--confirm-battery-and-external-5v-and-uart-disconnected',action='store_true',required=True)
    args=ap.parse_args()
    if not Path(PORT).exists():ap.error('expected COM device absent')
    os.umask(0o077)
    if args.mode=='backup':backup()
    elif args.backup is None:ap.error('--backup required')
    else:flash(args.backup)

if __name__=='__main__':main()
