"""Join SAME control sequences; not a camera-to-wheel physical latency."""
import argparse
import json
from pathlib import Path
from prepare import save


def stats(values):
    v=sorted(values)
    def q(p):return v[int((len(v)-1)*p)]*1000 if v else None
    return dict(samples=len(v),p50_ms=q(.5),p95_ms=q(.95),p99_ms=q(.99),
        max_ms=v[-1]*1000 if v else None)


def summarize(producer,consumer):
    pub={r['seq']:r for r in producer if r.get('kind')=='control_publish' and r.get('state')=='ACTIVE'}
    callback={r['seq']:r for r in consumer if r.get('kind')=='gate_callback_enter'}
    core={r['seq']:r for r in consumer if r.get('kind')=='gate_core_receive'}
    rows=[]
    for seq,p in pub.items():
        if seq not in callback or seq not in core:continue
        c,g=callback[seq],core[seq]
        if p['issued']!=c['issued'] or p['issued']!=g['issued']:
            raise ValueError('same sequence with different issue time')
        if not p['at']<=c['at']<=g['at']:
            raise ValueError('invalid same-clock event order')
        rows.append((c['at']-p['at'],g['at']-c['at'],g['at']-p['issued'],g['accepted']))
    return dict(active_publications=len(pub),matched=len(rows),unmatched=len(pub)-len(rows),
        publish_call_to_callback=stats([r[0] for r in rows]),
        callback_to_core_return=stats([r[1] for r in rows]),
        issued_to_core_return=stats([r[2] for r in rows]),
        rejected_matched=sum(not r[3] for r in rows),physical_latency_measured=False,
        scope='same-sequence software timestamps; publication call includes serialization, not a NIC timestamp')


def folder(path):
    def read(name):
        with (path/name).open() as f:return [json.loads(line) for line in f]
    return summarize(read('supervisor_TIMING.jsonl'),read('gate_TIMING.jsonl'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    p.add_argument('--output',type=Path);a=p.parse_args();value=folder(a.folder)
    if a.output:
        if a.output.exists():raise ValueError('do not overwrite timing evidence')
        save(a.output,value)
    print(json.dumps(value,ensure_ascii=False,indent=2))
