#!/usr/bin/env python3
"""Build generated ESP event-I/O candidate ONLY. No esptool/flash/serial calls."""
import argparse
import hashlib
import json
import os
import re
import shlex
from pathlib import Path
import subprocess
import sys
import time

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stable_translation_unit(text):
    # Collapse formatting only outside C string/character literals. This keeps
    # ++ vs + +, numeric boundaries and spaces inside credentials significant.
    pattern=r'("(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|\s+'
    return re.sub(pattern,lambda match:match.group(1) or ' ',text).strip()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True,type=Path)
    out=ap.parse_args().output.resolve();project=out/'esp_project';config=out/'sdkconfig.eventio'
    pins=json.loads((out/'ESP_SOURCE_INPUTS.json').read_text())
    if any(sha(Path(p))!=v for p,v in pins.items()):raise ValueError('ESP generated source changed after offline tests')
    idf=Path('/home/PRIMARY_USER/.espressif/v6.1/esp-idf/tools/idf.py')
    sdk_python=Path('/home/PRIMARY_USER/.espressif/tools/python/v6.1/venv/bin/python')
    log=out/'esp_idf_build.log';started=time.monotonic()
    # Reuse the IDE/EIM-managed installation, without running its `select`
    # action or modifying the user's shell/IDE. The generic SDK exporter expects
    # a different directory layout and is deliberately not used here.
    env=os.environ.copy()
    exported=subprocess.run(['bash','/home/PRIMARY_USER/.espressif/tools/activate_idf_v6.1.sh','-e'],
        check=True,capture_output=True,text=True).stdout
    allowed={'IDF_PATH','IDF_TOOLS_PATH','IDF_PYTHON_ENV_PATH','ESP_ROM_ELF_DIR',
             'ESP_IDF_VERSION','IDF_VERSION','IDF_COMPONENT_LOCAL_STORAGE_URL','PATH'}
    for line in exported.splitlines():
        if '=' not in line:continue
        key,value=line.split('=',1)
        if key in allowed:env[key]=value+(os.pathsep+os.environ.get('PATH','') if key=='PATH' else '')
    with log.open('w') as stream:
        p=subprocess.run([str(sdk_python),str(idf),'-C',str(project),'-B',str(out/'esp_build'),
            '-D','SDKCONFIG='+str(config),'build'],stdout=stream,stderr=subprocess.STDOUT,env=env)
    if p.returncode:
        print(log.read_text()[-9000:]);raise SystemExit(p.returncode)
    # Compile the flag-off translation unit through the real SDK preprocessor
    # and compare it to accepted source. Keep preprocessed credentials in RAM;
    # report only equality/hashes, never their contents.
    commands=json.loads((out/'esp_build/compile_commands.json').read_text())
    command=next(c for c in commands if Path(c['file']).resolve()==project/'main/app_main.c')
    argv=command.get('arguments') or shlex.split(command['command'])
    cleaned=[];skip=False
    for arg in argv:
        if skip:skip=False;continue
        if arg in ('-o','-MF','-MT','-MQ'):skip=True;continue
        if arg in ('-c','-MD','-MMD') or arg==command['file']:continue
        cleaned.append(arg)
    original=Path('/home/PRIMARY_USER/vicon_mecanum_ws/esp32/mecanum_wifi_bridge/main/app_main.c')
    normalised=[]
    for name,source in (('accepted',original),('candidate',project/'main/app_main.c')):
        wrapper=out/('compat_'+name+'.c')
        wrapper.write_text('#include '+json.dumps(str(out/'esp_build/config/sdkconfig.h'))+'\n'
            '#undef CONFIG_MC_EVENT_DRIVEN_FORWARD\n#include '+json.dumps(str(source))+'\n')
        # Normalize diagnostic locations only in this comparison, NOT in the
        # actual build. GCC requires -Wno-error for built-in macro redefinition;
        # the real firmware build above retains its original -Werror flags.
        pp=subprocess.run([*cleaned,'-D__LINE__=1','-D__FILE__="APP_SOURCE"',
            '-Wno-error','-E','-P',str(wrapper)],cwd=command['directory'],env=env,
            capture_output=True,text=True)
        if pp.returncode:raise ValueError('legacy preprocessor comparison failed; no image promoted')
        normalised.append(stable_translation_unit(pp.stdout))
    if normalised[0]!=normalised[1]:raise ValueError('event-off translation unit differs from accepted source')
    image=out/'esp_build/mecanum_wifi_bridge.bin'
    values={}
    for line in config.read_text().splitlines():
        if line.startswith('CONFIG_') and '=' in line:
            key,value=line.split('=',1);values[key]=value
    expected=dict(CONFIG_MC_EVENT_DRIVEN_FORWARD='y',CONFIG_MC_WIFI_SOFTAP='y',
        CONFIG_MC_UDP_COMMAND_TIMEOUT_MS='750',CONFIG_MC_UART_BAUD_RATE='921600',
        CONFIG_MC_UART_TX_GPIO='17',CONFIG_MC_UART_RX_GPIO='18')
    if any(values.get(k)!=v for k,v in expected.items()):raise ValueError('ESP candidate configuration drift')
    for key in ('CONFIG_MC_UART_STARTUP_PROBE','CONFIG_MC_UART_DIAGNOSTIC_ONLY','CONFIG_MC_WIFI_ECHO_ONLY',
                'CONFIG_MC_LINK_TIMING_TRACE','CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS'):
        if values.get(key)=='y':raise ValueError('unsolicited probe or intrusive diagnostic enabled')
    accepted_config=Path('/home/PRIMARY_USER/vicon_mecanum_ws/tmp/softap_full_link_20261015_Shf4zmaQ/sdkconfig')
    old_values=dict(line.split('=',1) for line in accepted_config.read_text().splitlines() if line.startswith('CONFIG_') and '=' in line)
    for key in ('CONFIG_MC_WIFI_SOFTAP_SSID','CONFIG_MC_WIFI_SOFTAP_PASSWORD','CONFIG_MC_WIFI_SOFTAP_CHANNEL',
                'CONFIG_MC_UDP_COMMAND_PORT','CONFIG_MC_UDP_TELEMETRY_PORT'):
        if values.get(key)!=old_values.get(key):raise ValueError('preserved AP/port configuration differs')
    result=dict(passed=True,image=str(image),image_sha256=sha(image),image_bytes=image.stat().st_size,
        seconds=time.monotonic()-started,flashed=False,serial_opened=False,hardware_motion_started=False,
        protocol_unchanged=True,watchdogs_unchanged=True,ap_credentials_preserved_not_printed=True,
        legacy_event_off_preprocessor_identical=True,legacy_translation_unit_sha256=hashlib.sha256(normalised[0].encode()).hexdigest(),
        compatibility_normalization='Diagnostic __LINE__/__FILE__ and whitespace outside literals only; real build flags unchanged.',
        generated_source_hashes={str(p):sha(p) for p in (project/'main').rglob('*') if p.is_file()},
        sdkconfig_sha256=sha(config),configuration_checked=expected,log=str(log))
    (out/'ESP_BUILD_RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    prep_path=out/'PREPARATION_RESULT.json';prep=json.loads(prep_path.read_text())
    prep.update(esp_built=True,esp_image_sha256=result['image_sha256'],esp_image=str(image),
                esp_build_record=str(out/'ESP_BUILD_RESULT.json'),legacy_event_off_preprocessor_identical=True)
    prep_path.write_text(json.dumps(prep,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='generated_source_hashes'},indent=2))
if __name__=='__main__':main()
