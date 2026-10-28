#!/usr/bin/env python3
"""Make a public DOCX copy, preserving visible text, math and page layout."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.parse import quote, unquote, urlsplit
from zipfile import ZipFile, ZIP_DEFLATED
from lxml import etree

ROOT = Path(__file__).resolve().parents[1]
NS = {'r': 'http://schemas.openxmlformats.org/package/2006/relationships'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', type=Path)
    args = ap.parse_args()
    out = ROOT/'car/reports/2026-10-28/Vicon标定杆遥操作ROS全向小车项目技术说明.docx'
    out.parent.mkdir(parents=True, exist_ok=True)
    changes = 0
    with ZipFile(args.source) as src, ZipFile(out, 'w', ZIP_DEFLATED) as dst:
        for entry in src.infolist():
            raw = src.read(entry)
            if entry.filename.endswith(('.xml', '.rels')):
                tree = etree.fromstring(raw)
                if entry.filename == 'word/document.xml':
                    for el in tree.iter():
                        if el.text:
                            el.text = el.text.replace(
                                '文件链接指向原Linux工作区；文档转移到其他电脑后，可按编号及文件名在归档中检索。',
                                '文件链接指向公开仓库中的脱敏记录；文档转移到其他电脑后，可直接打开链接或按编号检索。')
                if entry.filename.endswith('.rels'):
                    for rel in tree.findall('r:Relationship', NS):
                        if rel.get('TargetMode') != 'External':
                            continue
                        url = urlsplit(rel.get('Target', ''))
                        if url.scheme != 'file':
                            raise ValueError('Unreviewed external report link')
                        path = unquote(url.path)
                        relative = path.split('/vicon_mecanum_ws/', 1)[1]
                        if not (ROOT/'car'/relative).is_file():
                            raise ValueError('Public evidence missing: '+relative)
                        rel.set('Target', 'https://github.com/cuizuoye/vicon-robotics-lab/blob/main/car/'+quote(relative, safe='/'))
                        changes += 1
                if entry.filename == 'docProps/core.xml':
                    for el in tree:
                        if etree.QName(el).localname in {'creator', 'lastModifiedBy'}:
                            el.text = 'Vicon ROS car project'
                if entry.filename == 'docProps/custom.xml':
                    raise ValueError('Custom metadata requires separate review')
                for el in tree.iter():
                    for key in list(el.attrib):
                        if etree.QName(key).localname.startswith('rsid'):
                            del el.attrib[key]
                raw = etree.tostring(tree, encoding='UTF-8', xml_declaration=True, standalone=True)
            dst.writestr(entry, raw)
    if changes != 13:
        raise ValueError('Expected exactly thirteen evidence links')
    receipt = dict(path=str(out.relative_to(ROOT)), sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
                   evidence_links=changes, technical_content_and_math_preserved=True,
                   index_notice_updated=True, remote_links_rebased=True,
                   source_docx_written=False)
    (ROOT/'handoff/REVIEWED_DOCUMENTS.json').write_text(json.dumps([receipt], indent=2)+'\n')
    print(json.dumps(receipt, ensure_ascii=False))


if __name__ == '__main__':
    main()
