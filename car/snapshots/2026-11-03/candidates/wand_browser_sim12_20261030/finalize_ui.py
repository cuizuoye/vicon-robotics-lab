"""Display-only final status correction; never alters proof/key/control code."""
import hashlib
import json
from pathlib import Path

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
DOCS=ROOT/'docs/wand_browser_sim12_2026-10-30'
OLD_RESULT_LINE='resultSeen=ended=true;latest={...latest,result:v.result};'
NEW_RESULT_LINE="""resultSeen=ended=true;
      const stopped=v.result.software_stop_evidence;
      latest={...latest,result:v.result,session_requested:false,trial_ended:true,
        state:stopped?.stop_confirmed&&stopped?.gate_output_zero?'STOPPED_CONFIRMED':'STOP_UNCONFIRMED',
        reason:v.result.reason,goal:null,reference:null};"""


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


if __name__=='__main__':
    app=DOCS/'prepared_v4/web/app.js';manifest=json.loads((DOCS/'prepared_v4/MANIFEST.json').read_text())
    expected=manifest['artifact_hashes'][str(app)]
    if sha(app)!=expected:raise ValueError('unreviewed prior UI drift')
    text=app.read_text()
    if text.count(OLD_RESULT_LINE)!=1:raise ValueError('unexpected final result handler')
    before=DOCS/'ui_before_stop_label.js'
    if before.exists():raise ValueError('do not overwrite previous UI snapshot')
    before.write_text(text)
    app.write_text(text.replace(OLD_RESULT_LINE,NEW_RESULT_LINE))
    record=dict(before_sha256=sha(before),after_sha256=sha(app),
        patch='single final-result display assignment only; no sender, key, guard or control changes',
        prepare_sha256=sha(Path(__file__).with_name('prepare.py')),hardware_output=False)
    (DOCS/'UI_CHANGE.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record))
