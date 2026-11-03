'use strict';
// Protocol peer, NOT a browser/UI automation tool. Synthetic software only.
const {spawn}=require('child_process'),fs=require('fs'),path=require('path'),assert=require('assert');
const WS=require('/usr/share/nodejs/ws');
const root=path.resolve(process.argv[2]),output=path.resolve(process.argv[3]);
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function until(f,timeout=20000){const t=Date.now();while(!f()){if(Date.now()-t>timeout)throw Error('timeout');await wait(10);}}
async function run(name,duration){
  const child=spawn(process.execPath,[path.join(root,'server.cjs'),name==='occlusion'?'--test-occlusion':'--synthetic','--seconds',String(duration)]);
  let stdout='',stderr='';child.stdout.on('data',b=>stdout+=b);child.stderr.on('data',b=>stderr+=b);
  let control,view;
  try{
    await until(()=>stdout.includes('BROWSER_READY '));
    const info=JSON.parse(stdout.split('BROWSER_READY ')[1].split('\n')[0]);
    const origin=`http://127.0.0.1:${info.port}`,token=info.url.split('#')[1];
    let phase='',state='',sequence=0,pulse=true,activeAt=null,stopped=false,space=false,firstPulse;
    const actions=[];let viewCount=0;
    control=new WS(`ws://127.0.0.1:${info.port}/control`,{origin});control.on('error',()=>{});
    control.on('open',()=>control.send(JSON.stringify({type:'hello',token})));
    control.on('message',bytes=>{
      const v=JSON.parse(bytes);
      if(v.kind==='hello'){
        view=new WS(`ws://127.0.0.1:${info.port}/view`,{origin});view.on('error',()=>{});
        view.on('open',()=>view.send(JSON.stringify({type:'view_hello',token,owner:v.owner})));
        view.on('message',b=>{const x=JSON.parse(b);if(x.kind==='view'){
          viewCount++;view.send(JSON.stringify({type:'ack',sequence:x.view_sequence}));
        }});
      }else if(v.kind==='challenge'&&pulse){
        const data={type:'pulse',nonce:v.nonce,sequence:++sequence,focused:true,visible:true,space,ui_gap_ms:20,events:actions.splice(0)};
        if(!firstPulse)firstPulse=data;
        control.send(JSON.stringify(data));
      }else if(v.kind==='status'){
        phase=v.prepare_phase;state=v.state;
        if(phase==='READY'&&!activeAt&&!stopped&&name==='no_start'){
          stopped=true;setTimeout(()=>control.send(JSON.stringify({type:'stop',reason:'TEST_NO_START'})),700);
        }else if(phase==='READY'&&!activeAt&&!stopped){
          // Explicit test-generated press/release, never used in human entry.
          stopped=true;space=true;actions.push({kind:'press'});
          setTimeout(()=>{space=false;actions.push({kind:'release'});},40);
        }
        if(state==='ACTIVE'&&activeAt===null){
          activeAt=Date.now();
          if(name==='proof_timeout')setTimeout(()=>{pulse=false;},2500);
          if(name==='view_disconnect')setTimeout(()=>view.close(),2500);
          if(name==='control_disconnect')setTimeout(()=>control.close(),2500);
          if(name==='setpoint_rejected')setTimeout(()=>actions.push({kind:'setpoint',x:0,y:0,yaw:0,visible:true}),2500);
          if(name==='second_space')setTimeout(()=>{space=true;actions.push({kind:'press'});},2500);
          if(name==='nonce_replay')setTimeout(()=>{pulse=false;control.send(JSON.stringify({...firstPulse,sequence:++sequence}));},2500);
        }
      }
    });
    const resultPath=path.join(info.session,'result.json');
    await until(()=>fs.existsSync(resultPath),(duration+40)*1000);
    const result=JSON.parse(fs.readFileSync(resultPath));
    assert(result.ros_initialized&&result.ros_domain===191);
    assert(!result.hardware_bridge_constructed&&!result.hardware_socket_opened&&!result.physical_motion_started);
    assert.equal(result.authority_ever_acquired,name!=='no_start','activation mismatch: '+JSON.stringify(result.last_state));
    assert(result.software_stop_evidence.stop_confirmed,JSON.stringify(result.software_stop_evidence));
    assert(result.software_stop_evidence.gate_output_zero);
    assert.equal(result.software_stop_evidence.logging.dropped,0);
    assert(JSON.parse(fs.readFileSync(path.join(info.session,'PROTECTED.json'))).unchanged);
    const expected={normal:'TIMER_COMPLETE',proof_timeout:'BROWSER_PROOF_TIMEOUT',view_disconnect:'VIEW_DISCONNECTED',
      second_space:'OPERATOR_SECOND_SPACE',nonce_replay:'CHALLENGE_EXPIRED_OR_REPLAYED',no_start:'CLIENT_TEST_NO_START',
      control_disconnect:'CONTROL_DISCONNECTED',setpoint_rejected:'ACTION_NOT_ALLOWED',occlusion:'WAND_POSE_TIMEOUT'}[name];
    assert.equal(result.reason,expected);
    if(name==='normal')assert(result.passed&&result.completed);
    const record={case:name,passed:true,session:info.session,reason:result.reason,view_count:viewCount,
      max_ui_gap_s:result.max_client_gap_s,max_proof_age_s:result.max_proof_age_s,
      stop_confirmed:true,hardware_output:false};
    console.log(JSON.stringify(record));return record;
  }catch(e){throw Error(name+': '+e.message+'\n'+stderr.slice(-7000)+'\n'+stdout.slice(-3000));}
  finally{
    if(control)control.terminate();if(view)view.terminate();
    if(child.exitCode===null){child.kill('SIGTERM');await wait(1000);if(child.exitCode===null)child.kill('SIGKILL');}
  }
}
(async()=>{
  fs.mkdirSync(path.dirname(output),{recursive:true});const results=[];
  for(const name of ['normal','no_start','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'])results.push(await run(name,name==='normal'?74:10));
  fs.writeFileSync(output,JSON.stringify({passed:true,root,scope:'REAL_ROS_SYNTHETIC_PROTOCOL_PEER_NOT_BROWSER',results},null,2)+'\n');
})().catch(e=>{console.error(e);process.exitCode=1;});
