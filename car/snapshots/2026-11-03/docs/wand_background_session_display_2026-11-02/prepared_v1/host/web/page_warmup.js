'use strict';
// Pre-connection only. This never refreshes an InputGuard or creates a pulse.
class PageWarmup {
  constructor(){this.first=null;this.last=null;this.stableSince=null;this.frames=0;this.ready=false;this.maxGap=0;}
  frame(now,drawCost){
    if(this.ready)return true;
    if(this.first===null)this.first=now;
    const gap=this.last===null?0:now-this.last;this.last=now;this.maxGap=Math.max(this.maxGap,gap);this.frames++;
    if(gap>60||drawCost>20||this.stableSince===null)this.stableSince=now;
    this.ready=this.frames>=8&&now-this.stableSince>=500;
    return this.ready;
  }
}
if(typeof module!=='undefined'&&module.exports)module.exports=PageWarmup;
