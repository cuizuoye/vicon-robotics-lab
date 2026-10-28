#!/usr/bin/env python3
"""Separate event-I/O host and ESP candidates; NEVER flash or launch hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_clockfix/ready'
ESP=ROOT/'esp32/mecanum_wifi_bridge'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file() and not any(
        k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(p,old,new):
    text=p.read_text()
    if text.count(old)!=1:raise ValueError('source fragment drift: '+str(p)+' '+old[:80])
    p.write_text(text.replace(old,new))

def generate_esp(out):
    accepted=json.loads((ROOT/'docs/tenda_softap_full_link_20261015.json').read_text())
    if sha(ESP/'main/app_main.c')!=accepted['software_changes']['source_sha256']:
        raise ValueError('accepted ESP source drift; do not generate')
    project=out/'esp_project';project.mkdir()
    for name in ('main','tests'):
        shutil.copytree(ESP/name,project/name,ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('CMakeLists.txt','sdkconfig.defaults'):shutil.copyfile(ESP/name,project/name)
    edit(project/'CMakeLists.txt','project(mecanum_wifi_bridge)',
        'set(PROJECT_VER "eventio-20261026")\nproject(mecanum_wifi_bridge)')
    shutil.copyfile(HERE/'mc_event_forward.h',project/'main/mc_event_forward.h')
    cm=project/'main/CMakeLists.txt'
    edit(cm,'"../../../protocol"',json.dumps(str(ROOT/'protocol')))
    edit(cm,'nvs_flash lwip esp_driver_uart','nvs_flash lwip esp_driver_uart vfs')
    k=project/'main/Kconfig.projbuild'
    edit(k,'menu "MC-Link Bridge"','''menu "MC-Link Bridge"

    config MC_EVENT_DRIVEN_FORWARD
        bool "Independent event-driven UART/UDP forwarding candidate"
        depends on !MC_WIFI_ECHO_ONLY && !MC_UART_DIAGNOSTIC_ONLY
        default n
        help
            Read one UART byte, drain only available bytes, and wake the sole
            UDP owner with eventfd. No wire, network, watchdog or recovery change.
''')
    app=project/'main/app_main.c'
    edit(app,'#include "mc_radio_trace.h"','''#include "mc_radio_trace.h"
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
#include "esp_vfs_eventfd.h"
#include "mc_event_forward.h"
static int s_uart_wakeup_fd = -1;
static _Atomic bool s_uart_wakeup_error;

static bool forward_wakeup_init(void)
{
    const esp_vfs_eventfd_config_t config = {.max_fds=1};
    if (esp_vfs_eventfd_register(&config) != ESP_OK) return false;
    s_uart_wakeup_fd = eventfd(0, 0);
    if (s_uart_wakeup_fd < 0 || s_uart_wakeup_fd >= FD_SETSIZE) {
        if (s_uart_wakeup_fd >= 0) close(s_uart_wakeup_fd);
        s_uart_wakeup_fd = -1;
        (void)esp_vfs_eventfd_unregister();
        return false;
    }
    return true;
}
#endif''')
    edit(app,'''        const int received = uart_read_bytes(
            (uart_port_t)CONFIG_MC_UART_PORT,
            chunk,
            sizeof(chunk),
            pdMS_TO_TICKS(20));''','''#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        int received = uart_read_bytes((uart_port_t)CONFIG_MC_UART_PORT,
                                      chunk, 1u, portMAX_DELAY);
        if (received > 0) {
            size_t buffered = 0;
            if (uart_get_buffered_data_len((uart_port_t)CONFIG_MC_UART_PORT, &buffered) != ESP_OK) {
                atomic_store(&s_uart_wakeup_error, true);
                vTaskDelete(NULL); return;
            }
            const size_t take = mc_forward_drain_count(buffered, sizeof(chunk));
            if (take) {
                const int extra = uart_read_bytes((uart_port_t)CONFIG_MC_UART_PORT, chunk+1u, take, 0);
                if (extra < 0) { atomic_store(&s_uart_wakeup_error, true); vTaskDelete(NULL); return; }
                received += extra;
            }
        } else if (received < 0) {
            atomic_store(&s_uart_wakeup_error, true); vTaskDelete(NULL); return;
        }
#else
        const int received = uart_read_bytes(
            (uart_port_t)CONFIG_MC_UART_PORT,
            chunk,
            sizeof(chunk),
            pdMS_TO_TICKS(20));
#endif''')
    edit(app,'''                            ESP_LOGW(TAG, "Telemetry queue full; frame dropped");
                        }''','''                            ESP_LOGW(TAG, "Telemetry queue full; frame dropped");
                        }
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
                        else if (!mc_forward_notify(s_uart_wakeup_fd)) {
                            atomic_store(&s_uart_wakeup_error, true);
                        }
#endif''')
    edit(app,'''        while ((xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0u
#ifdef CONFIG_MC_WIFI_SOFTAP''','''#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        bool event_failure = false;
#endif
        while ((xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0u
#ifdef CONFIG_MC_WIFI_SOFTAP''')
    edit(app,'''            struct sockaddr_in sender_address = {0};
            socklen_t sender_length = sizeof(sender_address);''','''#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
            if (atomic_load(&s_uart_wakeup_error)) {
                ESP_LOGE(TAG, "UART event forwarding failed; close UDP, no automatic enable or clear");
                event_failure = true; break;
            }
            const int event_ready = mc_forward_wait(socket_fd, s_uart_wakeup_fd);
            if (event_ready < 0) { ESP_LOGE(TAG, "Forwarding select failed"); event_failure = true; break; }
            if (!event_ready) continue; /* UART wake: drain validated queue at loop top. */
#endif
            struct sockaddr_in sender_address = {0};
            socklen_t sender_length = sizeof(sender_address);''')
    edit(app,'''                socket_fd, receive_buffer, sizeof(receive_buffer), 0,
                (struct sockaddr *)&sender_address, &sender_length);''','''                socket_fd, receive_buffer, sizeof(receive_buffer),
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
                MSG_DONTWAIT,
#else
                0,
#endif
                (struct sockaddr *)&sender_address, &sender_length);''')
    edit(app,'''        close(socket_fd);
        ESP_LOGW(TAG, "UDP bridge stopped; waiting for Wi-Fi");''','''        close(socket_fd);
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        if (event_failure) {
            ESP_LOGE(TAG, "Event forwarder stopped; manual inspection required, no automatic restart");
            vTaskDelete(NULL); return;
        }
#endif
        ESP_LOGW(TAG, "UDP bridge stopped; waiting for Wi-Fi");''')
    edit(app,'''#endif
    uart_start();
    if (s_uart_enabled) {''','''#endif
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
    if (!forward_wakeup_init()) { ESP_LOGE(TAG, "Forward wake initialization failed; no UART/Wi-Fi start"); return; }
#endif
    uart_start();
    if (s_uart_enabled) {''')
    # Reuse only the accepted private SoftAP configuration; do not expose credentials.
    config=out/'sdkconfig.eventio';shutil.copyfile(ROOT/accepted['evidence_root']/'sdkconfig',config)
    config.write_text(config.read_text()+'\nCONFIG_MC_EVENT_DRIVEN_FORWARD=y\n');config.chmod(0o600)
    return project,config

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').write_text('')
    source_before=hashes(HERE)
    pins=dict(m['protected_input_hashes'])
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[group])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    pins.update(hashes(ESP/'main'))
    for name in ('CMakeLists.txt','sdkconfig','sdkconfig.defaults'):pins[str(ESP/name)]=sha(ESP/name)
    project,config=generate_esp(out)
    (out/'ESP_SOURCE_INPUTS.json').write_text(json.dumps(hashes(project),indent=2)+'\n')
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    h=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'receive_event.py',h/'receive_event.py')
    nodes=h/'nodes.py'
    edit(nodes,'from .startup_bridge import StartupBridgeMixin','from .startup_bridge import StartupBridgeMixin\nfrom .receive_event import ReceiveEventPump')
    edit(nodes,'        self.create_timer(.01, self.safety_tick)\n','''        self.create_timer(.01, self.safety_tick)
        self.receive_guard = self.create_guard_condition(self.receive_event)
        self.receive_pump = ReceiveEventPump(self.sock, self.receive_guard)

    def receive_event(self):
        try:
            if self.receive_pump.error:
                self._block_startup('RECEIVE_EVENT_PUMP_FAILED')
            self.drain_receive()
            self.safety_tick()
        finally:
            self.receive_pump.acknowledge()

    def safety_tick(self):
        # A failed worker/guard notification must also be noticed by the
        # original 100Hz safety timer; do not rely on the broken guard itself.
        if hasattr(self, 'receive_pump') and self.receive_pump.error:
            self._block_startup('RECEIVE_EVENT_PUMP_FAILED')
        return super().safety_tick()
''')
    edit(nodes,'    def destroy_node(self):\n        self.stop_wire()','''    def destroy_node(self):
        self.receive_pump.stop()
        self.stop_wire()''')
    app=out/'oneclick_app.py';shutil.copyfile(OLD/'oneclick_app.py',app)
    edit(app,'READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')')
    edit(app,'启动时钟修复候选','事件接收修复候选')
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copyfile(HERE/'test_eventio.py',tests/'test_eventio.py')
    edit(tests/'test_startup_clock.py','    b=nodes.HardwareBridge()','''    monkeypatch.setattr(Node,'create_guard_condition',lambda self,callback:NS(trigger=lambda:None))
    monkeypatch.setattr(nodes,'ReceiveEventPump',lambda *a:NS(error='',acknowledge=lambda:None,stop=lambda:True))
    b=nodes.HardwareBridge()''')
    def run(name,args,env=None,timeout=60):
        p=subprocess.run(list(map(str,args)),cwd=ROOT,env=env,capture_output=True,text=True,timeout=timeout)
        (out/(name+'.log')).write_text(p.stdout+p.stderr)
        print(name+': '+('PASS' if p.returncode==0 else 'FAIL'),flush=True)
        if p.returncode:print((p.stdout+p.stderr)[-14000:],flush=True);raise ValueError(name+' failed')
        return p.stdout+p.stderr
    result=dict(passed=False,hardware_started=False,flashed=False,host_network_changed=False,
        hardware_motion_authorized=False,old_aliases_changed=False)
    try:
        run('native_event_build',['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-g',
            '-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',
            '-I'+str(HERE),HERE/'test_event_forward.c','-o',out/'test_event_forward'])
        run('native_event_test',[out/'test_event_forward'])
        env=dict(os.environ,STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(ROOT/'candidates/wand_demo_120s_toggle'),
            EVENTIO_OUTPUT=str(out),PYTHONPATH=str(BASE)+os.pathsep+os.environ.get('PYTHONPATH',''))
        tests_log=run('host_tests',[sys.executable,'-m','pytest','-q',tests,'--junitxml='+str(out/'host_tests.xml')],env)
        run('host_build',['colcon','--log-base',out/'colcon_log','build','--base-paths',runtime,
            '--build-base',out/'runtime/build','--install-base',out/'runtime/install','--symlink-install',
            '--packages-select',m['control_package'],m['hardware_package']],env)
        if hashes(HERE)!=source_before:raise ValueError('candidate changed during testing')
        manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
            source_hashes={**m['source_hashes'],**source_before,str(app):sha(app)},protected_input_hashes=pins,
            draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
            host_change='Receive readiness guard, no thread I/O, unchanged 40ms startup cap and control clock/safety.')
        launcher=ROOT/'tools/start_car_demo_120s_toggle_eventio.sh'
        manifest['source_hashes'][str(launcher)]=sha(launcher)
        (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');check(out/'build_manifest.json')
        result.update(passed=True,tests=tests_log.strip().splitlines()[-1],protected_files_unchanged=len(pins),
            esp_project=str(project),esp_sdkconfig=str(config),esp_built=False,esp_flashed=False,
            stm32_firmware_changed=False,hello_cap_ms=40)
    finally:
        (out/'PREPARATION_RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
