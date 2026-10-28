import hashlib
import importlib.util
from pathlib import Path
from zipfile import ZipFile

import pytest

spec=importlib.util.spec_from_file_location('public_scan',Path(__file__).with_name('public_scan.py'))
scan=importlib.util.module_from_spec(spec)
spec.loader.exec_module(scan)


def make(tmp_path, members):
    path=tmp_path/'test.docx'
    with ZipFile(path,'w') as z:
        for k,v in members.items():z.writestr(k,v)
    return path,hashlib.sha256(path.read_bytes()).hexdigest()


def test_reviewed_document_requires_exact_hash(tmp_path):
    p,h=make(tmp_path,{'word/document.xml':'<doc/>'})
    scan.reviewed_docx(p,h)
    with pytest.raises(ValueError):scan.reviewed_docx(p,'0'*64)


@pytest.mark.parametrize('members',[
    {'../escape.xml':'<doc/>'},
    {'word/vbaProject.bin':b'unsafe'},
    {'word/document.xml':'<doc>'+('/home/'+'czy/private')+'</doc>'},
    {'word/_rels/document.xml.rels':'<Relationships><Relationship TargetMode="External" Target="file:///private"/></Relationships>'},
    {'word/media/a.png':b'not a png'},
])
def test_reviewed_hash_does_not_bypass_archive_checks(tmp_path,members):
    p,h=make(tmp_path,members)
    with pytest.raises(ValueError):scan.reviewed_docx(p,h)
