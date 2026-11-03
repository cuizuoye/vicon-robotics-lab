"""Freeze the accepted fixed PC controller + unflashed MCU artifacts, no IO."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
BASE=ROOT/'docs/wand_browser_direct_2026-10-30/prepared_v1'
FW=ROOT/'docs/wand_firmware_4wsc_2026-10-30/prepared_v4/flash_bundle'
OUT=ROOT/'docs/wand_migration_2026-10-30/fixed_baseline_v1'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    if OUT.exists():raise ValueError('never overwrite a frozen snapshot')
    subprocess.run(['bash',str(ROOT/'tools/start_sign_browser_direct.sh'),'--check'],check=True,capture_output=True)
    subprocess.run([sys.executable,str(FW/'verify_bundle.py'),'--check','--folder',str(FW)],check=True,capture_output=True)
    OUT.mkdir(parents=True)
    pc=OUT/'fixed_pc_baseline';pc.mkdir()
    for p in BASE.iterdir():
        if p.is_file() and p.suffix in ('.py','.cjs','.json'):shutil.copyfile(p,pc/p.name)
    shutil.copytree(BASE/'web',pc/'web')
    shutil.copytree(BASE/'runtime/src',pc/'runtime/src',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copytree(ROOT/'candidates/wand_browser_direct_20261030',OUT/'fixed_pc_source',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copytree(FW,OUT/'stm32_4WSC',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    shutil.copyfile(ROOT/'docs/wand_firmware_4wsc_2026-10-30/MIGRATION_STATUS.md',OUT/'MIGRATION_STATUS.md')
    shutil.copyfile(ROOT/'tools/start_sign_browser_direct.sh',OUT/'FIXED_PC_ENTRY_SNAPSHOT.sh')
    record=dict(fixed_baseline_manifest_sha256=sha(BASE/'MANIFEST.json'),firmware_bin_sha256=sha(FW/'mecanum_f407_4wsc_staged_NOT_FLASHED.bin'),
        mapping='direct 1:1 XY+yaw; no intermediate reference or fixed fence',speed_mps=1.2,accel_mps2=4.,yaw_radps=2.,yaw_accel_radps2=12.,
        adaptive_experiment_included=False,physical_qualified=False,host_hardware_bridge_migrated=False,
        firmware_flashed=False,hardware_opened=False,network_changed=False,
        source_snapshot_not_portable_installer=True,first_hardware_test_stage=0)
    (OUT/'BASELINE.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    (OUT/'README.txt').write_text('固定PC基线与独立STM32候选快照，不包含无级实验。\n'
        '高层控制继续运行在Linux电脑，只有stm32_4WSC中的BIN/HEX写入STM32；本包不自动刷写或启动。\n'
        '请先读MIGRATION_STATUS.md：配套实机浏览器准入尚未完成，不可直接用旧入口实车演示。\n'
        '源码保留本机绝对依赖路径，是证据快照，不是可移植安装包。当前完整Flash备份仍须现场重新读取。\n')
    sums={p.relative_to(OUT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file()}
    (OUT/'SHA256.json').write_text(json.dumps(sums,ensure_ascii=False,indent=2)+'\n')
    archive=OUT.parent/'FIXED_SYNC_PC_AND_4WSC_NOT_FLASHED.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in OUT.rglob('*'):
            if p.is_file():z.write(p,p.relative_to(OUT))
    with zipfile.ZipFile(archive) as z:
        for name,digest in sums.items():
            if hashlib.sha256(z.read(name)).hexdigest()!=digest:raise ValueError('archive corruption: '+name)
    (OUT.parent/'FROZEN.json').write_text(json.dumps(dict(passed=True,files=len(sums),archive=str(archive),
        archive_sha256=sha(archive),**record),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=True,archive=str(archive),files=len(sums),firmware_flashed=False,host_migration_complete=False)))
if __name__=='__main__':main()
