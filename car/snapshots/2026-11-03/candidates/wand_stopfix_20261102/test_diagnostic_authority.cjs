'use strict';
// Protocol peer, not browser automation. Only a loopback rehearsal runtime.
const {spawn}=require('child_process'),fs=require('fs'),path=require('path'),assert=require('assert');
const WS=require('/usr/share/nodejs/ws');
const root=path.resolve(process.argv[2]),output=path.resolve(process.argv[3]),name=process.argv[4],seconds=Number(process.argv[5]);
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function until(fn,timeout){const start=Date.now();while(!fn()){if(Date.now()-start>timeout)throw Error('timeout');await wait(10);}}
(async()=>{
 const args=[path.join(root,'server.cjs'),'--rehearsal',name==='occlusion'?'--test-occlusion':'--synthetic','--seconds',String(seconds)];
 const child=spawn(process.execPath,args,{env:process.env});let stdout='',stderr='',control,view,auditTimer;
 child.stdout.on('data',b=>stdout+=b);child.stderr.on('data',b=>stderr+=b);
 try{
  await until(()=>stdout.includes('BROWSER_READY '),30000);
  const info=JSON.parse(stdout.split('BROWSER_READY ')[1].split('\n')[0]);assert.equal(info.mode,'rehearsal');assert.equal(info.hardware_entry,false);
  const origin=`http://127.0.0.1:${info.port}`,token=info.url.split('#')[1];let sequence=0,pulse=true,space=false,started=false,stopQueued=false,active=null,firstPulse,views=0,present=true;
  const events=[];control=new WS(`ws://127.0.0.1:${info.port}/control`,{origin});control.on('error',()=>{});
  control.on('open',()=>control.send(JSON.stringify({type:'hello',token})));
  control.on('message',raw=>{
   const v=JSON.parse(raw);
   if(v.kind==='hello'){
    view=new WS(`ws://127.0.0.1:${info.port}/view`,{origin});view.on('error',()=>{});
    view.on('open',()=>view.send(JSON.stringify({type:'view_hello',token,owner:v.owner})));
    view.on('message',b=>{const x=JSON.parse(b);if(x.kind==='view'){
     assert(!Object.hasOwn(x,'reference'),'intermediate reference leaked');if(x.goal)assert.equal(x.target_policy,'DIRECT_SIGN_1TO1');
     views++;view.send(JSON.stringify({type:'ack',sequence:x.view_sequence}));
    }});
   }else if(v.kind==='challenge'&&pulse){
    const p={type:'pulse',nonce:v.nonce,sequence:++sequence,focused:present,visible:present,space,ui_gap_ms:20,events:events.splice(0)};
    firstPulse??=p;control.send(JSON.stringify(p));
   }else if(v.kind==='status'){
    if(v.prepare_phase==='READY'&&!started){
     assert(v.remaining_s<=seconds&&v.remaining_s>0,'UI countdown does not match this trial timer');
     if(seconds<600)assert(!v.instruction.includes('600秒'),'short trial must not advertise 600s'); else assert(v.remaining_s>599,'600s trial must not retain a 60s countdown');
     started=true;
     if(name==='no_start'){setTimeout(()=>control.send(JSON.stringify({type:'stop',reason:'TEST_NO_START'})),600);}
     else{space=true;events.push({kind:'press'});setTimeout(()=>{space=false;events.push({kind:'release'});},40);}
    }
    if(v.state==='ACTIVE'&&active===null){
     assert(v.remaining_s<=seconds,'active UI countdown exceeds this trial timer');
     active=Date.now();
     if(name==='esc')setTimeout(()=>control.send(JSON.stringify({type:'stop',reason:'ESC'})),1500);
     if(name==='blur')setTimeout(()=>present=false,1500);
     if(name==='proof_timeout')setTimeout(()=>{
      let auditSequence=0;
      const sendAudit=()=>{
       const record={sequence:++auditSequence,kind:'resize',at_ms:auditSequence*10,trusted:true,target:'window',
        focused:true,hidden:false,visibility:'visible',active_tag:'CANVAS',width:1280,height:720,fullscreen:false};
       const body=JSON.stringify({token,record});
       const req=require('http').request({hostname:'127.0.0.1',port:info.port,path:'/diagnostic',method:'POST',
        headers:{Origin:origin,'Content-Type':'application/json','Content-Length':Buffer.byteLength(body)}},res=>res.resume());
       req.on('error',()=>{});req.end(body);
      };
      sendAudit();auditTimer=setInterval(sendAudit,10);pulse=false;
     },1500);
     if(name==='view_disconnect')setTimeout(()=>view.close(),1500);
     if(name==='control_disconnect')setTimeout(()=>control.close(),1500);
     if(name==='second_space')setTimeout(()=>{space=true;events.push({kind:'press'});},1500);
     if(name==='nonce_replay')setTimeout(()=>{pulse=false;control.send(JSON.stringify({...firstPulse,sequence:++sequence}));},1500);
     if(name==='setpoint_rejected')setTimeout(()=>events.push({kind:'setpoint',x:0,y:0,yaw:0,visible:true}),1500);
    }
   }
  });
  const file=path.join(info.session,'result.json');await until(()=>fs.existsSync(file),(seconds+40)*1000);
  const r=JSON.parse(fs.readFileSync(file));assert(r.ros_initialized&&r.ros_domain===196);
  assert(r.hardware_bridge_constructed&&!r.hardware_socket_opened&&!r.physical_motion_started);
  assert.equal(r.authority_ever_acquired,name!=='no_start',JSON.stringify(r.last_state));
  assert(r.stop_evidence?.stop_confirmed&&r.stop_evidence.gate_output_zero,JSON.stringify(r));
  const expected={normal:'TIMER_COMPLETE',esc:'CLIENT_ESC',blur:'WINDOW_NOT_FOREGROUND',proof_timeout:'BROWSER_PROOF_TIMEOUT',view_disconnect:'VIEW_DISCONNECTED',
   control_disconnect:'CONTROL_DISCONNECTED',second_space:'OPERATOR_SECOND_SPACE',nonce_replay:'CHALLENGE_EXPIRED_OR_REPLAYED',
   setpoint_rejected:'ACTION_NOT_ALLOWED',no_start:'CLIENT_TEST_NO_START'}[name];
  if(name==='occlusion')assert(r.reason==='WAND_POSE_TIMEOUT'||r.reason.startsWith('car or wand pose timeout:'),JSON.stringify(r));
  else if(name==='normal')assert(['TIMER_COMPLETE','STAGE6_LONG600_TIME_COMPLETE'].includes(r.reason),JSON.stringify(r));
  else assert.equal(r.reason,expected,JSON.stringify(r));
  if(name==='normal')assert(r.passed&&r.completed);
  const audit=JSON.parse(fs.readFileSync(path.join(info.session,'transport.json'))).focus_audit;
  assert(audit.rows>0&&audit.diagnostic_only,'diagnostic route must have accepted actual records');
  assert.equal(r.reason,'BROWSER_PROOF_TIMEOUT','audit traffic cannot renew a motion proof');
  const record={focus_audit:audit,audit_cannot_renew_authority:true,case:name,passed:true,session:info.session,reason:r.reason,views,stop_confirmed:true,
   actual_ROS_UDP_C:true,physical_output:false,max_proof_age_s:r.max_proof_age_s,max_client_gap_s:r.max_client_gap_s};
  fs.writeFileSync(output,JSON.stringify(record,null,2)+'\n');console.log(JSON.stringify(record));
 }catch(e){fs.writeFileSync(output,JSON.stringify({case:name,passed:false,error:e.message,stdout,stderr},null,2)+'\n');throw e;}
 finally{
  if(auditTimer)clearInterval(auditTimer);
  if(control)control.terminate();if(view)view.terminate();
  if(child.exitCode===null){child.kill('SIGTERM');await wait(700);if(child.exitCode===null)child.kill('SIGKILL');}
 }
})().catch(e=>{console.error(e);process.exitCode=1;});
