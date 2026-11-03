"""File-only ARM/HEX/vector/protection verification. No device APIs."""
import importlib.util
import json
from pathlib import Path
import struct
from build import ROOT,OUT,FWBASE,TARGET,sha,save
def main():
    m=json.loads((OUT/'BUILD.json').read_text())
    for group in ('protected','source_hashes','firmware_source_hashes'):
        for p,digest in m[group].items():assert sha(p)==digest,'changed '+p
    b=OUT/'flash_bundle';package=json.loads((b/'BUNDLE.json').read_text())
    for p,digest in json.loads((b/'SHA256.json').read_text()).items():assert sha(p)==digest
    assert package['flashed'] is False and package['host_hardware_launch_enabled'] is False
    binary=(b/(TARGET+'.bin')).read_bytes();assert len(binary)==package['image_bytes']<65536
    assert sha(b/(TARGET+'.bin'))==package['image_sha256']==m['binary_sha256']
    sp,reset=struct.unpack_from('<II',binary);base=0x08000000
    assert 0x20000000<sp<=0x20020000 and sp%8==0 and reset&1 and base<=(reset&~1)<base+len(binary)
    elf=(b/(TARGET+'.elf')).read_bytes();assert elf[:6]==b'\x7fELF\x01\x01' and struct.unpack_from('<H',elf,18)[0]==40
    spec=importlib.util.spec_from_file_location('parent_file_only_verifier',FWBASE/'flash_bundle/verify_bundle.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    hex_result=helper.check_hex(b/(TARGET+'.hex'),binary,base)
    report=dict(passed=True,image_bytes=len(binary),image_sha256=sha(b/(TARGET+'.bin')),hex_bin_equal=hex_result['hex_bin_equal'],
        initial_stack=hex(sp),reset_handler=hex(reset),ARM_elf_verified=True,flashed=False,hardware_output=False,
        protected_files=len(m['protected']),old_sources_unchanged=True,physical_qualified=False)
    save(OUT/'ARTIFACT_VERIFICATION.json',report);print(json.dumps(report))
if __name__=='__main__':main()
