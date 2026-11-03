'use strict';
// Observation only. No timer, keyboard, lease, nonce or connection authority.
class FocusEvidence {
  constructor(){this.sequence=0;}
  observe(kind,event,target,state){
    const record={sequence:++this.sequence,kind,at_ms:state.at_ms,
      trusted:event?.isTrusted===true,target,focused:state.focused===true,
      hidden:state.hidden===true,visibility:state.visibility,
      active_tag:state.active_tag,width:state.width,height:state.height,
      fullscreen:state.fullscreen===true};
    // An event name alone is not evidence of focus loss. Genuine loss is
    // checked here AND by the unchanged 20ms periodic/100ms proof guards.
    // No grace window, delayed enable, focus recovery or lease renewal.
    let reason='';
    if(record.hidden)reason='WINDOW_HIDDEN';
    else if(!record.focused)reason=kind==='blur'?'WINDOW_BLUR':'WINDOW_NOT_FOREGROUND';
    else if(kind==='pagehide'&&record.trusted&&target==='window')reason='PAGE_CLOSED';
    return {record,reason};
  }
}
if(typeof module!=='undefined')module.exports={FocusEvidence};
