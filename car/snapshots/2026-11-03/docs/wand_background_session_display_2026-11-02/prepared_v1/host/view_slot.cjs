'use strict';
// One in-flight frame (client ACK), one latest pending. No TCP-sized FIFO.
class ViewSlot{
  constructor(send,fail){this.send=send;this.fail=fail;this.pending=null;this.flight=null;this.next=0;
    this.offered=0;this.sent=0;this.coalesced=0;this.maxBytes=0;this.maxPending=0;}
  offer(value){
    const text=JSON.stringify(value),bytes=Buffer.byteLength(text);
    if(bytes>8192){this.fail('VIEW_FRAME_LIMIT');return;}
    this.maxBytes=Math.max(this.maxBytes,bytes);this.offered++;
    if(this.pending)this.coalesced++;
    this.pending=value;this.maxPending=1;this.flush();
  }
  flush(){
    if(!this.pending||this.flight!==null)return;
    const value=this.pending;this.pending=null;this.flight=++this.next;this.sent++;
    this.send({...value,kind:'view',view_sequence:this.flight});
  }
  ack(sequence){
    if(!Number.isSafeInteger(sequence)||sequence!==this.flight){this.fail('VIEW_ACK_INVALID');return false;}
    this.flight=null;this.flush();return true;
  }
  stats(){return {offered:this.offered,sent:this.sent,coalesced:this.coalesced,max_pending:this.maxPending,max_frame_bytes:this.maxBytes};}
}
module.exports={ViewSlot};
