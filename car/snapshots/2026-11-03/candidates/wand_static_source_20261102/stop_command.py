"""Out-of-browser STOP only. HTTP acceptance is never called a stopped car."""
import json
from pathlib import Path
import time
from urllib.parse import urlsplit
from urllib.request import Request,urlopen
from build import OUT


def stop():
    info=json.loads((OUT/'LATEST.json').read_text())
    u=urlsplit(info['url'])
    if u.scheme!='http' or u.hostname!='127.0.0.1' or not u.port or not u.fragment:raise ValueError('invalid local session URL')
    session=Path(info['session']).resolve()
    if session.parent!=(OUT/'host/sessions').resolve():raise ValueError('wrong independent session')
    def confirmed():
        try:
            r=json.loads((session/'result.json').read_text());s=r.get('stop_evidence') or {}
            return r if s.get('stop_confirmed') is True and s.get('gate_output_zero') is True else None
        except (OSError,ValueError):return None
    prior=confirmed()
    if prior:
        print(json.dumps(dict(stop_confirmed=True,already_ended=True,reason=prior['reason']),ensure_ascii=False));return
    origin='http://127.0.0.1:'+str(u.port)
    data=json.dumps(dict(type='stop',token=u.fragment,reason='LOCAL_STOP')).encode()
    req=Request(origin+'/operator-stop',data=data,headers={'Origin':origin,'Content-Type':'application/json'},method='POST')
    with urlopen(req,timeout=2) as response:
        if response.status!=202:raise ValueError('STOP not accepted')
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        r=confirmed()
        if r:
            print(json.dumps(dict(stop_confirmed=True,already_ended=False,reason=r['reason']),ensure_ascii=False));return
        time.sleep(.05)
    raise ValueError('STOP sent but not confirmed; check the car and disconnect power if necessary')
