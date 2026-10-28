#!/usr/bin/env python3
"""Normalize the existing report builder, without executing or overwriting it."""
import argparse
from pathlib import Path

from export_car_snapshot import ROOT, normalize


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('source',type=Path)
    args=ap.parse_args()
    text=normalize(args.source.read_text(),Path('/nonexistent/source'))
    text=text.replace("ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')\nWORK = Path(__file__).resolve().parent",
        "ROOT = Path(__file__).resolve().parents[3]\nWORK = ROOT.parent / '.local/report-rebuild'\nWORK.mkdir(parents=True, exist_ok=True)")
    text=text.replace("OUT = WORK.parent /", "OUT = WORK /")
    text=text.replace('import math\n', 'import math\nfrom urllib.parse import quote\n')
    text=text.replace("src.as_uri()", "'https://github.com/cuizuoye/vicon-robotics-lab/blob/main/car/'+quote(rel, safe='/')")
    text=text.replace('文件链接指向原Linux工作区；文档转移到其他电脑后，可按编号及文件名在归档中检索。',
                      '文件链接指向公开仓库中的脱敏记录；文档转移到其他电脑后，可直接打开链接或按编号检索。')
    out=ROOT/'car/reports/2026-10-28/source/build_report.py'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(text)
    print(out.relative_to(ROOT))


if __name__=='__main__':main()
