'use strict';
// A socket callback may ASK permission, but must not renew the periodic UI task.
class InputGuard{
  constructor(){this.started=false;this.last=null;this.interval=0;this.maxGap=0;this.fault='';}
  begin(now){if(this.started)return false;this.started=true;this.last=now;return true;}
  fail(reason){if(!this.fault)this.fault=reason;return false;}
  tick(now){
    if(!this.started||this.fault)return false;
    const gap=now-this.last;this.maxGap=Math.max(this.maxGap,gap);
    if(!(gap>=0&&gap<=100))return this.fail('CLIENT_LOOP_GAP');
    this.interval=gap;this.last=now;return true;
  }
  permit(now){
    if(!this.started||this.fault)return false;
    const gap=now-this.last;
    if(!(gap>=0&&gap<=100))return this.fail('CLIENT_LOOP_GAP');
    return true; // NEVER changes last/interval/maxGap.
  }
  age(now){return Math.max(this.interval,now-this.last);}
}
if(typeof module!=='undefined')module.exports={InputGuard};
