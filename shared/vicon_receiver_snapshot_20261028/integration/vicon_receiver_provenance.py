"""Fail-closed local ELF/install/library provenance, not a Vicon/flight permit.

No receiver is executed here. glibc --list resolves dependencies without
running the ELF entry point or its constructors. The review manifest is pinned
offline, never regenerated or extended by a live capture. Ordinary rebuilds or
package upgrades require a new explicit review, not automatic acceptance.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import struct
import subprocess

MANIFEST = Path(__file__).parent/'plan_b/phase2/vicon_receiver_provenance.json'


class ProvenanceError(RuntimeError):
    pass


def digest(data):
    return hashlib.sha256(data).hexdigest()


class Elf:
    """Bounded ELF64 LE x86-64 metadata reader; no external parser/execution."""
    def __init__(self, data):
        self.data = data
        if len(data)<64 or data[:7]!=b'\x7fELF\x02\x01\x01':
            raise ProvenanceError('unsupported_or_truncated_ELF')
        if struct.unpack_from('<H',data,18)[0]!=62:
            raise ProvenanceError('unexpected_ELF_machine')
        phoff=struct.unpack_from('<Q',data,32)[0]
        phsize,phcount=struct.unpack_from('<HH',data,54)
        if phsize!=56 or not 0<phcount<=256:
            raise ProvenanceError('invalid_ELF_program_headers')
        self.slice(phoff,phsize*phcount)
        interpreters=[]
        for i in range(phcount):
            ph=struct.unpack_from('<IIQQQQQQ',data,phoff+i*phsize)
            if ph[0]==3:
                raw=self.slice(ph[2],ph[5])
                if not raw.endswith(b'\0') or b'\0' in raw[:-1]:
                    raise ProvenanceError('invalid_ELF_interpreter')
                interpreters.append(raw[:-1].decode())
        if len(interpreters)!=1:
            raise ProvenanceError('one_ELF_interpreter_required')
        self.interpreter=interpreters[0]
        off=struct.unpack_from('<Q',data,40)[0]
        size,count,names=struct.unpack_from('<HHH',data,58)
        if size!=64 or not 0<count<=4096 or names>=count:
            raise ProvenanceError('invalid_ELF_section_table')
        self.slice(off,size*count)
        self.headers=[struct.unpack_from('<IIQQQQIIQQ',data,off+i*size) for i in range(count)]
        h=self.headers[names]; strings=self.slice(h[4],h[5])
        self.sections={}
        for h in self.headers:
            if h[0]>=len(strings): raise ProvenanceError('invalid_ELF_section_name')
            name=strings[h[0]:].split(b'\0',1)[0].decode('ascii')
            if name in self.sections: raise ProvenanceError('duplicate_ELF_section')
            self.sections[name]=h
        dynamics=[h for h in self.headers if h[1]==6]
        if len(dynamics)!=1: raise ProvenanceError('one_dynamic_section_required')
        h=dynamics[0]
        if h[6]>=count or h[9]!=16 or h[5]%16: raise ProvenanceError('invalid_dynamic_section')
        self.strings=self.headers[h[6]]
        if self.strings[1]!=3: raise ProvenanceError('dynamic_string_table_required')
        self.slice(self.strings[4],self.strings[5])
        self.dynamic=[]
        for off in range(h[4],h[4]+h[5],16):
            tag,val=struct.unpack('<qQ',self.slice(off,16))
            if tag==0: break
            self.dynamic.append((tag,val))
        if sum(t==29 for t,v in self.dynamic)!=1 or any(t==15 for t,v in self.dynamic):
            raise ProvenanceError('exactly_one_RUNPATH_and_no_RPATH_required')
        self.run_offset=next(v for t,v in self.dynamic if t==29)
        self.runpath=self.string(self.run_offset)
        self.needed=[self.string(v) for t,v in self.dynamic if t==1]
        if any('/' in n or not n for n in self.needed):
            raise ProvenanceError('unexpected_DT_NEEDED_path')

    def slice(self, off, length):
        if off<0 or length<0 or off+length>len(self.data):
            raise ProvenanceError('ELF_range_outside_file')
        return self.data[off:off+length]

    def string(self, off):
        if not 0<=off<self.strings[5]: raise ProvenanceError('dynamic_string_offset_invalid')
        raw=self.slice(self.strings[4]+off,self.strings[5]-off)
        if b'\0' not in raw: raise ProvenanceError('unterminated_dynamic_string')
        return raw.split(b'\0',1)[0].decode('utf-8')


def verify_installation(build, installed, policy):
    """All bytes equal except ONE pinned RUNPATH string plus its zero padding.

The rest of .dynstr (symbol names, SONAME/NEEDED strings etc.) is compared,
as are ELF/program/section headers, .dynamic, code, relocations and trailing
bytes. Unknown stripping, rebuilt code or any other difference is refused.
"""
    try:
        before=Path(build).read_bytes(); after=Path(installed).read_bytes()
    except OSError as exc:
        raise ProvenanceError('missing_or_unreadable_receiver_artifact') from exc
    if digest(before)!=policy['build_sha256']:
        raise ProvenanceError('unreviewed_build_version_or_content')
    a,b=Elf(before),Elf(after)
    if a.needed!=policy['needed'] or b.needed!=a.needed:
        raise ProvenanceError('DT_NEEDED_changed_or_unreviewed')
    if a.runpath!=policy['build_runpath']:
        raise ProvenanceError('unreviewed_build_RUNPATH')
    if before==after:
        change='identical_reviewed_ELF'
    else:
        if b.runpath!=policy['install_runpath']:
            raise ProvenanceError('unapproved_install_RUNPATH')
        start=a.strings[4]+a.run_offset
        old=a.runpath.encode()+b'\0'; new=policy['install_runpath'].encode()+b'\0'
        if len(new)>len(old): raise ProvenanceError('RUNPATH_growth_not_supported')
        expected=before[:start]+new.ljust(len(old),b'\0')+before[start+len(old):]
        if after!=expected:
            raise ProvenanceError('unexplained_ELF_difference_outside_exact_RUNPATH_rewrite')
        change='exact_CMake_RUNPATH_rewrite_only'
    return dict(verified=True,change=change,build=str(Path(build).resolve()),
        installed=str(Path(installed).resolve()),build_sha256=digest(before),installed_sha256=digest(after),
        build_runpath=a.runpath,installed_runpath=b.runpath,DT_NEEDED=b.needed,
        changed_bytes=sum(x!=y for x,y in zip(before,after)),
        text_sha256=digest(a.slice(a.sections['.text'][4],a.sections['.text'][5])),
        rodata_sha256=digest(a.slice(a.sections['.rodata'][4],a.sections['.rodata'][5])),
        full_ELF_comparison_except_exact_RUNPATH=True)


def file_identity(path):
    s=Path(path).stat()
    return [s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns]


def child_environment(base, policy):
    """Only child process environment changes; never the shared receiver's."""
    env={k:v for k,v in base.items() if not k.startswith(('LD_','RMW_','FASTRTPS_','FASTDDS_','CYCLONEDDS_'))
         and k not in ('GLIBC_TUNABLES','ROS_SECURITY_ENABLE','ROS_SECURITY_KEYSTORE','ROS_SECURITY_STRATEGY')}
    paths=policy['library_search_paths']
    if not paths or any(not p.startswith('/') or not p or ':' in p for p in paths):
        raise ProvenanceError('unsafe_library_search_path')
    env.update(LD_LIBRARY_PATH=':'.join(paths),LD_BIND_NOW='1',ROS_DOMAIN_ID='83',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        AMENT_PREFIX_PATH=':'.join(policy['ament_prefix_paths']))
    return env


def parse_loader_list(text):
    result={}
    for line in text.splitlines():
        line=line.strip()
        if not line or line.startswith('linux-vdso.so.1 '): continue
        matched=re.fullmatch(r'(\S+) => (/\S+) \(0x[0-9a-fA-F]+\)',line)
        direct=re.fullmatch(r'(/\S+) \(0x[0-9a-fA-F]+\)',line)
        if matched: name,path=matched.groups()
        elif direct:
            path=direct[1]; name=Path(path).name
        else: raise ProvenanceError('unresolved_or_unexpected_loader_output:'+line)
        real=str(Path(path).resolve(strict=True))
        if name in result and result[name]!=real: raise ProvenanceError('ambiguous_loader_dependency')
        result[name]=real
    if not result: raise ProvenanceError('empty_loader_dependency_result')
    return result


def verify_library_files(records):
    identities={}
    for path,record in records.items():
        p=Path(path)
        try:
            # Canonical targets, not just a directory prefix or a SONAME.
            if str(p.resolve(strict=True))!=path or not p.is_file():
                raise ProvenanceError('library_canonical_path_changed:'+path)
            raw=p.read_bytes()
            if not raw.startswith(b'\x7fELF') or digest(raw)!=record['sha256']:
                raise ProvenanceError('library_content_unreviewed:'+path)
            identities[path]=file_identity(p)
        except OSError as exc:
            raise ProvenanceError('library_missing_or_unreadable:'+path) from exc
    return identities


def verify_current(manifest_path=MANIFEST, *, resolver=None):
    """Offline gate. It may run ld.so --list, never the receiver entry point."""
    manifest_path=Path(manifest_path)
    policy=json.loads(manifest_path.read_text())
    if policy['schema']!=1: raise ProvenanceError('unsupported_provenance_manifest')
    report=verify_installation(policy['build'],policy['installed'],policy['receiver'])
    interpreter=Elf(Path(policy['installed']).read_bytes()).interpreter
    if str(Path(interpreter).resolve(strict=True))!=policy['loader']:
        raise ProvenanceError('unreviewed_ELF_interpreter_resolution')
    if report['installed_runpath']!=policy['receiver']['install_runpath']:
        # Equality is valid provenance, but an uninstalled build's trailing
        # empty search component is NOT a launch-safe current-directory path.
        raise ProvenanceError('runtime_requires_reviewed_installed_RUNPATH')
    for path,sha in policy['source_evidence'].items():
        if digest(Path(path).read_bytes())!=sha:
            raise ProvenanceError('build_or_source_evidence_changed:'+path)
    if Path('/etc/ld.so.preload').exists() and Path('/etc/ld.so.preload').read_text().strip():
        raise ProvenanceError('system_preload_requires_separate_review')
    identities=verify_library_files(policy['libraries'])
    env=child_environment({},policy)
    observed={}
    for target,expected in policy['loader_resolutions'].items():
        if resolver is None:
            # The already hash-checked system loader only enumerates mappings.
            reply=subprocess.run([policy['loader'],'--list',target],env={**env,'LC_ALL':'C'},
                capture_output=True,text=True,timeout=5,check=False)
            if reply.returncode or reply.stderr.strip():
                raise ProvenanceError('dynamic_loader_dependency_check_failed:'+target)
            actual=parse_loader_list(reply.stdout)
        else:
            actual=resolver(target,env)
        if actual!=expected: raise ProvenanceError('runtime_dependency_resolution_changed:'+target)
        for path in actual.values():
            if path not in identities: raise ProvenanceError('unreviewed_runtime_library:'+path)
        observed[target]=actual
    report.update(manifest=str(manifest_path.resolve()),manifest_sha256=digest(manifest_path.read_bytes()),
        library_count=len(identities),library_file_identities=identities,loader_resolutions=observed,
        source_evidence=policy['source_evidence'],policy=policy,
        interpreter_request=interpreter,interpreter_resolved=policy['loader'],
        offline_only=True,receiver_started=False,flight_authorized=False,EV_DELAY_approved=False,
        limitation='No OS sandbox or constructor-before-SDK handshake. Runtime maps are a detection/abort guard, not a proof that arbitrary future dlopen can never execute.')
    report['receiver_file_identity']=file_identity(policy['installed'])
    return report


def recheck_before_spawn(provenance):
    """Fail if a reviewed file was replaced since preflight; no auto-rebaseline."""
    expected={**provenance['library_file_identities'],
              provenance['installed']:provenance['receiver_file_identity']}
    if 'interpreter_request' in provenance and str(Path(provenance['interpreter_request']).resolve(strict=True))!=provenance['interpreter_resolved']:
        raise ProvenanceError('ELF_interpreter_changed_before_spawn')
    for path,identity in expected.items():
        if file_identity(path)!=identity: raise ProvenanceError('reviewed_file_changed_before_spawn:'+path)


MAP_LINE = re.compile(r'^([0-9a-fA-F]+)-([0-9a-fA-F]+)\s+'
    r'([r-][w-][x-][ps])\s+([0-9a-fA-F]+)\s+'
    r'([0-9a-fA-F]+):([0-9a-fA-F]+)\s+(\d+)(?:\s+(.*))?$')


def parse_mapping(line):
    """Preserve pathname spaces/deletion marker; never infer type by suffix."""
    match=MAP_LINE.fullmatch(line.rstrip())
    if not match: raise ProvenanceError('malformed_proc_maps_line')
    start,end,perms,offset,major,minor,inode,name=match.groups()
    result=dict(start=int(start,16),end=int(end,16),permissions=perms,offset=int(offset,16),
        device=[int(major,16),int(minor,16)],inode=int(inode),name=name or '')
    if result['end']<=result['start']: raise ProvenanceError('invalid_proc_maps_range')
    result['executable']='x' in perms
    result['deleted']=result['name'].endswith(' (deleted)')
    result['path']=result['name'][:-10] if result['deleted'] else result['name']
    return result


def inspect_mapping_file(path):
    """Read a bounded real-file header, not process memory or device contents."""
    p=Path(path).resolve(strict=True)
    if not stat.S_ISREG(p.stat().st_mode):
        return dict(format='unrecognized',path=str(p),reason='not_a_regular_file')
    descriptor=os.open(p,os.O_RDONLY|os.O_NONBLOCK|os.O_CLOEXEC)
    try:
        info=os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode):
            return dict(format='unrecognized',path=str(p),reason='not_a_regular_file')
        header=os.read(descriptor,64)
    finally:
        os.close(descriptor)
    result=dict(path=str(p),identity=[info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns,info.st_ctime_ns],
        device=[os.major(info.st_dev),os.minor(info.st_dev)],inode=info.st_ino)
    if header.startswith(b'\x7fELF'):
        if len(header)<64 or header[4:7]!=b'\x02\x01\x01' or struct.unpack_from('<H',header,18)[0]!=62:
            return dict(**result,format='unrecognized',reason='invalid_or_unsupported_ELF_header')
        elf_type=struct.unpack_from('<H',header,16)[0]
        return dict(**result,format='elf',elf_type=elf_type)
    return dict(**result,format='non_elf')


def classify_mapping(mapping, provenance, *, inspector=inspect_mapping_file):
    """Classification is separate from approval. Unknown executable => reject.

Unresolvable NON-executable mappings are recorded as unknown (e.g. unlinked
shared memory), never called an approved library. If any executable segment
of such an object appears, it is rejected. This is not an OS loader sandbox.
"""
    row=dict(mapping)
    name=mapping['name']; execute=mapping['executable']
    row.update(allowed=False,library_approved=False)
    kernel={'[vdso]','[vsyscall]','[vvar]','[vvar_vclock]'}
    if name in kernel and mapping['inode']==0 and mapping['device']==[0,0]:
        row['category']='kernel_special'
        row['allowed']=('w' not in mapping['permissions'] and
                        (not execute or name in ('[vdso]','[vsyscall]')))
        return row
    anonymous=(not name or name in ('[heap]','[stack]') or
        re.fullmatch(r'\[stack:\d+\]',name) or name.startswith(('[anon:','[anon_shmem:')))
    if anonymous and mapping['inode']==0:
        row.update(category='anonymous',allowed=not execute)
        return row
    if not mapping['path'].startswith('/'):
        row.update(category='unrecognized',allowed=not execute,uncertainty='unknown_nonexecutable_mapping' if not execute else 'unknown_executable_mapping')
        return row
    known={**provenance['library_file_identities'],provenance['installed']:provenance['receiver_file_identity']}
    try:
        info=inspector(mapping['path'])
    except OSError as exc:
        if mapping['path'] in known:
            row.update(category='unrecognized',rejection='reviewed_mapped_file_unavailable:'+mapping['path'])
            return row
        row.update(category='unrecognized',allowed=not execute,
            uncertainty='file_header_unavailable_nonexecutable' if not execute else 'file_header_unavailable_executable',
            inspection_error=type(exc).__name__)
        return row
    row['canonical_path']=info['path']
    if mapping['path'] in known and info['path']!=mapping['path']:
        row.update(category='unrecognized',rejection='reviewed_mapping_path_redirected:'+mapping['path'])
        return row
    if info['path'] in known and info.get('identity')!=known[info['path']]:
        row.update(category='unrecognized',rejection='mapped_file_changed_since_review:'+info['path'])
        return row
    if info['format']=='non_elf':
        row.update(category='non_elf_file',allowed=not execute)
        return row
    if info['format']=='unrecognized' and info.get('reason')!='invalid_or_unsupported_ELF_header':
        row.update(category='unrecognized',allowed=not execute,uncertainty=info.get('reason','unknown_file_type'))
        return row
    if info['format']!='elf' or info['elf_type'] not in (2,3):
        # A corrupt/unsupported ELF magic is not ordinary data and never an
        # excuse to skip library approval, even for its read-only segment.
        row.update(category='unrecognized',uncertainty=info.get('reason','unsupported_ELF_type'))
        return row
    path=info['path']
    loader=provenance.get('interpreter_resolved',provenance.get('policy',{}).get('loader'))
    row['category']=('elf_loader' if path==loader else 'elf_executable'
        if path==provenance['installed'] or info['elf_type']==2 else 'elf_shared_object')
    if mapping['deleted']:
        row['rejection']='deleted_runtime_ELF_mapping'
    elif path not in known:
        row['rejection']='unreviewed_mapped_library_or_executable:'+path
    elif info['identity']!=known[path]:
        row['rejection']='mapped_file_changed_since_review:'+path
    elif info['inode']!=mapping['inode'] or info['device']!=mapping['device']:
        row['rejection']='mapped_file_inode_or_device_mismatch:'+path
    else:
        # Preflight verified the whole-file hash; unchanged fd identity binds
        # this header to that reviewed file, not only to its filename.
        row.update(allowed=True,library_approved=True)
    return row


def verify_runtime_maps(text, provenance, *, details=None, inspector=inspect_mapping_file):
    """Inspect our child only; classify by file header, not '.so' path text.

Post-launch detection, NOT a kernel loader allowlist. The caller retains the
raw bounded snapshot BEFORE calling this function, including on rejection.
"""
    audit=details if details is not None else {}
    audit.update(mappings=[],category_counts={},unknown_nonexecutable_count=0)
    loaded=set()
    # Several segments refer to one file. Inspect once per snapshot, not once
    # per segment; no cache survives into the next mapping audit.
    inspected={}
    def inspect_once(path):
        if path not in inspected: inspected[path]=inspector(path)
        return inspected[path]
    for line in text.splitlines():
        if not line.strip(): continue
        try:
            mapping=parse_mapping(line)
            row=classify_mapping(mapping,provenance,inspector=inspect_once)
        except ProvenanceError as exc:
            audit['rejection']=str(exc)
            raise
        audit['mappings'].append(row)
        category=row['category']
        audit['category_counts'][category]=audit['category_counts'].get(category,0)+1
        if category=='unrecognized' and row['allowed']:
            audit['unknown_nonexecutable_count']+=1
        if not row['allowed']:
            reason=row.get('rejection','unapproved_'+category+'_mapping:'+row['name'])
            audit['rejection']=reason
            raise ProvenanceError(reason)
        if row['library_approved']: loaded.add(row['canonical_path'])
    audit['verified_elf_paths']=sorted(loaded)
    return sorted(loaded)
