"""Deliver honest offline progress with unresolved stability; NO admission."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile
from prepare import ROOT,HERE,FW,sha,hashes,save
from bootstrap import OUT
from acceptance import plan,blank_observation


def run():
    if OUT.name!='prepared_v6':raise ValueError('Historical prepared_v6 draft writer only; do not downgrade a newer candidate')
    m=json.loads((OUT/'HOST_MANIFEST.json').read_text())
    cases=ET.parse(OUT/'host_tests_final.xml').getroot()
    assert not cases.findall('.//failure') and not cases.findall('.//error')
    n=len(cases.findall('.//testcase'))
    fault=json.loads((OUT/'ros_faults_final/RESULT.json').read_text())
    assert fault['passed'] and len(fault['cases'])==14
    assert hashes(OUT/'runtime/src')==m['runtime_hashes']
    for p,digest in m['protected'].items():
        assert sha(p)==digest,p
    assert sha(m['firmware']['path'])==m['firmware']['sha256']
    for stage in range(8):
        save(OUT/f'acceptance/STAGE{stage}_PLAN.json',plan(stage))
        for purpose in ('suspended','sign'):
            save(OUT/f'acceptance/STAGE{stage}_{purpose}_BLANK_NOT_APPROVED.json',blank_observation(stage,purpose))
    previous=ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v5'
    baseline=json.loads((previous/'STAGE_MATRIX.json').read_text())
    assert baseline['passed'] and baseline['actual_c_ros_stages']==8
    result=dict(delivered=True,host_revision='prepared_v6',identity='4WSB',authority_version=7,
        diagnostic_payload_bytes=208,unit_tests_passed=n,functional_fault_cases_passed=14,
        eight_stage_functional_baseline=dict(path=str(previous/'STAGE_MATRIX.json'),passed=True,
            note='preceding host revision before QoS split; NOT latest-revision stability evidence'),
        latest_stage_matrix_passed=False,runtime_stability_qualified=False,host_offline_passed=False,
        hardware_admitted=False,firmware_flashed=False,hardware_output=False,
        original_files_unchanged=len(m['protected']),firmware_sha256=m['firmware']['sha256'],
        original_os_freeze_fixed=False,network_settings_changed=False,
        blockers=['ROS subscription/processing lag under concurrent load; final QoS split not long-run qualified',
            'cold launch concurrent with build rejected expired disabled authority; no motion occurred',
            'serial stage1 withdrew output but did not qualify stationarity within1.5s in the uncalibrated software plant',
            'native desktop freeze not reproduced or fixed',
            'fresh backup/readback and physical stop/current/thermal qualification remain future gates'])
    save(OUT/'DRAFT_DELIVERY.json',result)
    evidence=[OUT/'host_tests_final.xml',OUT/'ros_faults_final/RESULT.json',previous/'STAGE_MATRIX.json',
              previous/'ui_600s_actual_pixels/normal_run/normal/RESULT.json',
              previous/'ui_600s_actual_pixels/full_geometry_run/full_geometry/RESULT.json',
              OUT/'ros_stage0/RESULT.json',OUT/'stage0_unloaded_probe/RESULT.json',
              OUT/'serial_matrix/stage0/RESULT.json',OUT/'serial_matrix/stage1/RESULT.json',
              OUT.parent/'DRAFT_REPORT.md']
    m.update(host_software_port_complete=True,host_offline_passed=False,stability_qualified=False,
        hardware_admitted=False,flashed=False,source_hashes=hashes(HERE),
        evidence={str(p):sha(p) for p in evidence},file_checker_sha256=sha(ROOT/'tools/start_car_sync_4wsb.sh'),
        unresolved=result['blockers'])
    save(OUT/'HOST_MANIFEST.json',m)
    save(OUT/'PROTECTED_AFTER.json',{p:sha(p) for p in m['protected']})
    save(OUT/'ACTIVATION_BLOCKERS.json',dict(hardware_admitted=False,flashed=False,
        fresh_full_backup=False,independent_readback=False,disabled_link_qualified=False,
        physical_qualified=False,host_stability_qualified=False,automatic_enable=False))
    archive=OUT.parent/'4WSB_HOST_DRAFT_STABILITY_PENDING.zip'
    if archive.exists():raise ValueError('do not overwrite an existing draft delivery')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for base,prefix in ((HERE,Path('candidate_source')),(OUT/'runtime/src',Path('runtime/src')),
                            (OUT/'acceptance',Path('acceptance'))):
            for p in sorted(base.rglob('*')):
                if p.is_file() and '__pycache__' not in p.parts:z.write(p,prefix/p.relative_to(base))
        for name in ('HOST_MANIFEST.json','DRAFT_DELIVERY.json','ACTIVATION_BLOCKERS.json'):
            z.write(OUT/name,name)
        z.write(OUT.parent/'DRAFT_REPORT.md','DRAFT_REPORT.md')
    save(OUT/'ARCHIVE.json',dict(path=str(archive),sha256=sha(archive),
        deployable=False,stability_qualified=False,hardware_admitted=False))
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':run()
